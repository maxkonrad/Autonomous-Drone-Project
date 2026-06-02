import genesis as gs
import numpy as np
import cv2
import time

# --- 1. DONANIM VE FİZİKSEL PARAMETRELER (REAL WORLD SPEC) ---
DRONE_MASS = 18.11 / 9.81 
GRAVITY = 9.81
HOVER_PWM = 1362.0
CENTER_PWM = 1500.0

BATTERY_VOLTAGE_MAX = 22.2  # 6S Tam Dolu LiPo
MOTOR_KV = 900.0        
C_T = 3.4618e-7

ARUCO_COMMANDS = {
    0: "TURN_PLUS_Y",   
    1: "PRECISION_LAND" 
}

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# --- 2. SENSÖR SİMÜLATÖRÜ ---
class SimulatedSensors:
    def __init__(self):
        self.gps_noise_std = 0.02  
        self.baro_noise_std = 0.01 
        self.z_drift = 0.0         

    def read_sensors(self, true_pos, dt):
        self.z_drift += np.random.normal(0, 0.0005) * dt 
        noisy_x = true_pos[0] + np.random.normal(0, self.gps_noise_std)
        noisy_y = true_pos[1] + np.random.normal(0, self.gps_noise_std)
        noisy_z = true_pos[2] + np.random.normal(0, self.baro_noise_std) + self.z_drift
        return np.array([noisy_x, noisy_y, noisy_z])

# --- 3. ALÇAK GEÇİREN FİLTRE ---
class VectorLowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.filtered_vector = None

    def compute(self, current_vector):
        if self.filtered_vector is None:
            self.filtered_vector = current_vector 
        else:
            self.filtered_vector = (self.alpha * current_vector) + ((1.0 - self.alpha) * self.filtered_vector)
        return self.filtered_vector

# --- 4. SİMÜLASYON VE SAHNE KURULUMU ---
gs.init(backend=gs.cpu)
scene = gs.Scene(show_viewer=True)
scene.add_entity(gs.morphs.Plane())

scene.add_entity(gs.morphs.URDF(file='aruco_turn.urdf', pos=(3.0, 0.0, 0.0)))
scene.add_entity(gs.morphs.URDF(file='aruco_land.urdf', pos=(3.0, 3.0, 0.0)))

drone = scene.add_entity(gs.morphs.URDF(file='helionv4.urdf', pos=(0.0, 0.0, 0.05)))
camera = scene.add_camera(res=(640, 480), pos=(0.0, 0.0, 0.05), lookat=(0.0, 0.0, 0.0), fov=62)
scene.build()

# --- 5. DONANIMSAL MSP (Voltaj Çökmesi ve Yer Etkisi Simülatörü) ---
class SimulatedMSP:
    def apply_rc_to_physics(self, roll, pitch, throttle, true_z):
        throttle_pct = max(0.0, (throttle - 1000.0) / 1000.0)
        
        # ⚡ VOLTAJ ÇÖKMESİ (Voltage Sag)
        voltage_drop = (throttle_pct ** 2) * 2.5 
        current_voltage = BATTERY_VOLTAGE_MAX - voltage_drop
        dynamic_max_rpm = current_voltage * MOTOR_KV
        
        # Temel Karesel İtki
        rpm_z = throttle_pct * dynamic_max_rpm
        force_z = C_T * (rpm_z ** 2) 

        # 🌪️ YER ETKİSİ (Ground Effect)
        if true_z < 0.4 and true_z > 0.0:
            ground_cushion_multiplier = 1.0 + (0.4 - true_z) * 0.6 
            force_z *= ground_cushion_multiplier
            force_z += np.random.normal(0, 0.3)

        pitch_pct = (1500.0 - pitch) / 500.0  
        roll_pct = (1500.0 - roll) / 500.0    
        
        force_x = force_z * pitch_pct * 2.0
        force_y = force_z * roll_pct * 2.0

        kuvvet = np.zeros(drone.n_dofs)
        if drone.n_dofs >= 3:
            kuvvet[0], kuvvet[1], kuvvet[2] = force_x, force_y, force_z
        drone.control_dofs_force(kuvvet)
        
        return current_voltage

# --- 6. PID KONTROLCÜ ---
class PIDController:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint, self.integral, self.last_error = 0.0, 0.0, 0.0
        self.out_min, self.out_max = out_min, out_max
        self.orig_kp, self.orig_kd = kp, kd 

    def compute(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral = np.clip(self.integral + error * dt, -20.0, 20.0)
        p_out = self.kp * error
        i_out = self.ki * self.integral
        d_out = self.kd * (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error
        return np.clip(p_out + i_out + d_out, self.out_min, self.out_max)
        
    def reset_tuning(self):
        self.kp, self.kd = self.orig_kp, self.orig_kd

# --- 7. OTONOM UÇUŞ KONTROLCÜSÜ ---
class AutonomousFlightController:
    def __init__(self):
        self.pid_alt = PIDController(15.0, 0.0, 40.0, -100.0, 100.0)
        self.pid_pitch = PIDController(12.0, 1.2, 28.0, -25.0, 25.0) 
        self.pid_roll = PIDController(12.0, 1.2, 28.0, -25.0, 25.0)  

    def compute_rc_channels(self, current_pos, target_pos, dt, mod):
        self.pid_alt.setpoint = target_pos[2]
        base_throttle = HOVER_PWM if target_pos[2] > 0.3 else 1345.0 
        
        # 🪂 ÖZEL İNİŞ ALGORİTMASI (Ground Effect'i Delmek İçin)
        if mod == "PRECISION_LAND" and current_pos[2] < 0.6:
            base_throttle = 1315.0 
            self.pid_alt.kp = 6.0  
            self.pid_alt.kd = 10.0 
        else:
            self.pid_alt.reset_tuning()

        throttle_offset = self.pid_alt.compute(current_pos[2], dt)
        final_throttle = np.clip(base_throttle + throttle_offset, 1000.0, 2000.0)

        self.pid_pitch.setpoint = target_pos[0]
        pitch_offset = self.pid_pitch.compute(current_pos[0], dt)
        final_pitch = np.clip(CENTER_PWM - pitch_offset, 1000.0, 2000.0)

        self.pid_roll.setpoint = target_pos[1]
        roll_offset = self.pid_roll.compute(current_pos[1], dt)
        final_roll = np.clip(CENTER_PWM - roll_offset, 1000.0, 2000.0)
        
        return final_roll, final_pitch, final_throttle

# --- 8. ANA UÇUŞ DÖNGÜSÜ ---
sensors = SimulatedSensors()
pos_filter = VectorLowPassFilter(alpha=0.15) 
msp_bridge = SimulatedMSP()
fc = AutonomousFlightController()

last_log_time = 0.0
turn_target_x = 0.0 
land_target_y = 0.0 

mod = "KALKIS"
target = [0.0, 0.0, 2.0]

with open("helion_v12_detayli_log.txt", "w") as log_file:
    # 📝 LOG SÜTUNLARI GENİŞLETİLDİ: Roll ve Pitch eklendi!
    log_file.write(f"{'Zaman(s)':<10}\t{'Ucus_Modu':<15}\t{'Gercek_Z':<10}\t{'Filtreli_Z':<12}\t{'Roll_PWM':<10}\t{'Pitch_PWM':<10}\t{'Throt_PWM':<10}\t{'Voltaj(V)':<10}\n")
    log_file.write("-" * 105 + "\n")
    
    print("\n🚀 Helion 10 v12 - Manevra Loglamalı Tam Simülasyon Başlıyor!")
    
    try:
        for i in range(3500): 
            dt = 0.015
            elapsed = i * dt
            
            true_pos = drone.get_pos().numpy()
            noisy_pos = sensors.read_sensors(true_pos, dt)
            filtered_pos = pos_filter.compute(noisy_pos)
            
            camera.set_pose(pos=true_pos + np.array([0,0,-0.05]), lookat=true_pos + np.array([0,0,-1]))
            scene.step()
            
            img = camera.render()
            img_np = np.array(img[0] if isinstance(img, (list, tuple)) else img, dtype=np.uint8)
            
            if len(img_np.shape) == 3 and img_np.shape[2] == 4:
                img_gray = cv2.cvtColor(img_np, cv2.COLOR_RGBA2GRAY)
            else:
                img_gray = cv2.cvtColor(img_np, cv2.COLOR_RGB2GRAY)
                
            corners, ids, _ = detector.detectMarkers(img_gray)

            if ids is not None:
                detected_id = ids[0][0] 
                marker_corners = corners[0][0]
                px_x, px_y = np.mean(marker_corners[:, 0]), np.mean(marker_corners[:, 1])
                dist_to_center = np.sqrt((px_x - 320)**2 + (px_y - 240)**2)
                
                if detected_id in ARUCO_COMMANDS and dist_to_center < 220:
                    komut = ARUCO_COMMANDS[detected_id]
                    
                    if komut == "TURN_PLUS_Y" and mod in ["SEARCHING_X", "KALKIS"]:
                        print(f"\n📥 KOMUT ALINDI: Dönüş (Merkez Sapması: {dist_to_center:.0f}px)")
                        turn_target_x = filtered_pos[0] 
                        mod = "SEARCHING_Y"
                        
                    elif komut == "PRECISION_LAND" and mod != "PRECISION_LAND":
                        print(f"\n🎯 KOMUT ALINDI: İniş! Ground Effect (Yer Etkisi) Modeli Aktif Ediliyor.")
                        land_target_y = filtered_pos[1]
                        mod = "PRECISION_LAND"

            if mod == "KALKIS":
                target = [0.0, 0.0, 2.0]
                if elapsed > 4.0:
                    mod = "SEARCHING_X"
            elif mod == "SEARCHING_X":
                target = [filtered_pos[0] + 0.45, 0.0, 2.0]
            elif mod == "SEARCHING_Y":
                target = [turn_target_x, filtered_pos[1] + 0.45, 2.0]
            elif mod == "PRECISION_LAND":
                target = [turn_target_x, land_target_y, 0.05] 
            
            roll_pwm, pitch_pwm, throttle_pwm = fc.compute_rc_channels(filtered_pos, target, dt, mod)
            current_voltage = msp_bridge.apply_rc_to_physics(roll_pwm, pitch_pwm, throttle_pwm, true_pos[2])
            
            # 🎯 1 Saniyelik Tüm Detayların Loglanması
            if elapsed - last_log_time >= 1.0:
                log_file.write(f"{elapsed:<10.2f}\t{mod:<15}\t{true_pos[2]:<10.3f}\t{filtered_pos[2]:<12.3f}\t{roll_pwm:<10.1f}\t{pitch_pwm:<10.1f}\t{throttle_pwm:<10.1f}\t{current_voltage:<10.2f}\n")
                last_log_time = elapsed
                
            if i % 20 == 0:
                print(f"\r⏱️ {elapsed:04.1f}s | Mod: {mod:<15} | Z: {filtered_pos[2]:.2f}m | Gaz: {throttle_pwm:.0f} | Pil: {current_voltage:.1f}V", end="")

            if mod == "PRECISION_LAND" and true_pos[2] < 0.12:
                print(f"\n\n🏁 Helion Voltaj Çökmesine direndi, Yer Etkisini deldi ve indi!")
                break 
                
    except Exception as e:
        print(f"\nUçuş esnasında hata: {e}")

print("\n✅ v12 Tam Gerçeklik Testi tamamlandı.")
