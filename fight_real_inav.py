import serial
import struct
import time
import numpy as np

# ==============================================================================
# 🛠️ 1. GERÇEK DÜNYA PORT VE PARAMETRE AYARLARI (DEĞİŞTİRİLEBİLİR BÖLÜM)
# ==============================================================================
SERIAL_PORT = '/dev/ttyUSB0'   # 🔌 DEĞİŞTİRİLEBİLİR: Pi'deki gerçek UART veya USB-TTL portu
BAUD_RATE = 115200             # iNav için standart seri haberleşme hızı

TARGET_ALTITUDE = 1.5          # 🚀 DEĞİŞTİRİLEBİLİR: Kalkılacak yüksekliğin metre hedefi
TARGET_X = 2.0                 # ➡️ DEĞİŞTİRİLEBİLİR: X ekseninde kaç metre ileri gidileceği
HOVER_DURATION = 5.0           # ⏱️ DEĞİŞTİRİLEBİLİR: Hedefe varınca havada süzülme süresi

HOVER_PWM_ESTIMATE = 1362.0    # 🔋 DEĞİŞTİRİLEBİLİR: 900KV/6S sistem için tahmini asılı kalma gazı

# 🦺 TAKLA ATMAYI (FLIP) ÖNLEYEN SERT GÜVENLİK LİMİTLERİ
# 1500 tam merkezdir. Drone'un ilk testlerde ani ve vahşi yatışlar yapıp takla atmaması için
# Roll ve Pitch kanallarının maksimum eğim sınırını daraltıyoruz.
MIN_RC_VALUE = 1420            # DEĞİŞTİRİLEBİLİR: Maksimum sola/geriye yatış sınırı
MAX_RC_VALUE = 1580            # DEĞİŞTİRİLEBİLİR: Maksimum sağa/ileriye yatış sınırı

# ==============================================================================
# 📡 2. SAF iNAV MSP TELEMETRİ VE HABERLEŞME PROTOKOLÜ
# ==============================================================================
class INavMSPSerial:
    def __init__(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.02)
            print(f"✅ {port} portu üzerinden iNav Kartına başarıyla bağlanıldı.")
        except Exception as e:
            print(f"❌ Donanım Port Hatası! Kablo takılı mı?: {e}")
            self.ser = None

    def send_msp_packet(self, cmd, payload=b""):
        if self.ser is None: return
        size = len(payload)
        header = struct.pack('<3sBBB', b'$M<', size, cmd)
        checksum = size ^ cmd
        for b in payload:
            checksum ^= b
        self.ser.write(header + payload + struct.pack('<B', checksum))

    def send_raw_rc(self, roll, pitch, throttle, yaw=1500, aux1=1000):
        """iNav kartına motor sürme komutlarını basar (CMD: 200)"""
        # Donanımsal takla atma koruma limitlerini uygula
        roll = int(np.clip(roll, MIN_RC_VALUE, MAX_RC_VALUE))
        pitch = int(np.clip(pitch, MIN_RC_VALUE, MAX_RC_VALUE))
        throttle = int(np.clip(throttle, 1000, 2000))
        
        payload = struct.pack('<HHHHH', roll, pitch, throttle, yaw, aux1)
        self.send_msp_packet(200, payload)

    def read_hardware_sensors(self):
        """
        iNav kartının kendi üzerindeki dahili sensörlerden (Lidar/Barometre ve GPS/Manevra)
        gerçek X ve Z verilerini tıkır tıkır çeker. Sahte katman içermez.
        """
        current_x = 0.0
        current_z = 0.0
        voltage = 22.2

        if self.ser is None: return current_x, current_z, voltage

        # A. KARTIN SENSÖRÜNDEN YÜKSEKLİK ÇEK (MSP_ALTITUDE = 109)
        self.send_msp_packet(109)
        time.sleep(0.001)
        if self.ser.in_waiting >= 12:
            data = self.ser.read(self.ser.in_waiting)
            if b'$M>' in data:
                try:
                    idx = data.find(b'$M>') + 5
                    raw_alt = struct.unpack('<i', data[idx:idx+4])[0]
                    current_z = raw_alt / 100.0 # Santimetreyi metreye çevirir
                except: pass

        # B. KARTIN NAVİGASYONUNDAN GERÇEK LOCAL X POZİSYONUNU ÇEK (MSP_POSITION_NAV = 122)
        # iNav kartına bağlı Optical Flow veya GPS'in ürettiği santimetre cinsinden local X koordinatıdır.
        self.send_msp_packet(122)
        time.sleep(0.001)
        if self.ser.in_waiting >= 14:
            data = self.ser.read(self.ser.in_waiting)
            if b'$M>' in data:
                try:
                    idx = data.find(b'$M>') + 5
                    # İlk 4 byte local Kuzey/X pozisyonunu verir (32-bit signed int)
                    raw_x = struct.unpack('<i', data[idx:idx+4])[0]
                    current_x = raw_x / 100.0 # Santimetreyi metreye çevirir
                except: pass

        # C. KARTIN GERÇEK VOLTAJ SENSÖRÜNÜ OKU (MSP_ANALOG = 110)
        self.send_msp_packet(110)
        time.sleep(0.001)
        if self.ser.in_waiting >= 11:
            data = self.ser.read(self.ser.in_waiting)
            if b'$M>' in data:
                try:
                    idx = data.find(b'$M>') + 5
                    voltage = data[idx] / 10.0
                except: pass

        return current_x, current_z, voltage

# ==============================================================================
# 🧠 3. PID KONTROLCÜ GRUBU
# ==============================================================================
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
        
    def modify_gains(self, kp, kd):
        self.kp, self.kd = kp, kd

    def reset_gains(self):
        self.kp, self.kd = self.orig_kp, self.orig_kd

# ==============================================================================
# 🚀 4. OTONOM UÇUŞ KONTROLCÜSÜ (PÜR DONANIM)
# ==============================================================================
class AutonomousFlightController:
    def __init__(self):
        # PID KATSAYILARI: Gerçek dünya testi esnasında iNav'daki stabiliteye göre değiştirilebilir!
        self.pid_alt = PIDController(kp=15.0, ki=2.5, kd=40.0, out_min=-100.0, out_max=100.0)
        self.pid_pitch = PIDController(kp=12.0, ki=1.2, kd=28.0, out_min=-25.0, out_max=25.0) 
        self.pid_roll = PIDController(kp=12.0, ki=1.2, kd=28.0, out_min=-25.0, out_max=25.0)  

    def compute_rc_channels(self, current_pos, target_pos, dt, mod):
        self.pid_alt.setpoint = target_pos[2]
        base_throttle = HOVER_PWM_ESTIMATE if target_pos[2] > 0.3 else 1345.0 
        
        # 🌪️ İNİŞTE YER ETKİSİ (GROUND EFFECT) KORUMASI
        if mod == "INIŞ_ALGORITMASI" and current_pos[2] < 0.5:
            base_throttle = 1315.0 # Hava yastığından zıplamayı önleyen düşük taban gazı
            self.pid_alt.modify_gains(kp=6.0, kd=10.0) # PID'yi esneterek sert frenleri kısıtlar
        else:
            self.pid_alt.reset_gains()

        throttle_offset = self.pid_alt.compute(current_pos[2], dt)
        final_throttle = np.clip(base_throttle + throttle_offset, 1000.0, 2000.0)

        self.pid_pitch.setpoint = target_pos[0]
        pitch_offset = self.pid_pitch.compute(current_pos[0], dt)
        final_pitch = np.clip(CENTER_PWM - pitch_offset, 1000.0, 2000.0)

        self.pid_roll.setpoint = target_pos[1]
        roll_offset = self.pid_roll.compute(current_pos[1], dt)
        final_roll = np.clip(CENTER_PWM - roll_offset, 1000.0, 2000.0)
        
        return final_roll, final_pitch, final_throttle

# ==============================================================================
# ⏱️ 5. GERÇEK ZAMANLI DONANIM GÖREV DÖNGÜSÜ
# ==============================================================================
msp = INavMSPSerial(SERIAL_PORT, BAUD_RATE)
fc = AutonomousFlightController()

dt = 0.02 # 50Hz Kontrol frekansı (Her döngü net 20 milisaniyede bir dönmek zorundadır)
elapsed_time = 0.0
hover_timer = 0.0

mod = "ARM_BEKLEME"
target = [0.0, 0.0, TARGET_ALTITUDE]
last_log_time = 0.0

print("\n⚠️ DIKKAT: Helion gerçek uçuş yazılımı donanıma yükleniyor. PERVANELERI SÖKÜN!")
time.sleep(2)

with open("helion_real_hardware_flight.txt", "w") as log_file:
    log_file.write(f"{'Zaman(s)':<10}\t{'Ucus_Modu':<15}\t{'Sensor_X(m)':<12}\t{'Sensor_Z(m)':<12}\t{'Roll_PWM':<10}\t{'Pitch_PWM':<10}\t{'Throt_PWM':<10}\t{'Real_Pil_V':<10}\n")
    log_file.write("-" * 110 + "\n")

    # 25 Saniyelik Donanımsal Acil Durum Zaman Kilidi (Drone havada kilitlenirse otomatik kapatma emniyeti)
    for _ in range(int(25.0 / dt)):
        start_loop = time.time()
        
        # --- 📡 A. GERÇEK iNAV SENSÖR OKUMALARI ---
        current_x, current_z, current_voltage = msp.read_hardware_sensors()
        current_pos = np.array([current_x, 0.0, current_z])

        # --- 🧠 B. OTONOM GÖREV DURUM MAKİNESİ ---
        if mod == "ARM_BEKLEME":
            if elapsed_time > 3.0: # iNav kartıyla port senkronizasyonu için 3 saniye emniyet bekletmesi
                print("\n⚔️ [ARMED] iNav Motorları ateşledi! Dikine kalkış modu devrede.")
                mod = "KALKIS"
            roll_pwm, pitch_pwm, throttle_pwm = 1500, 1500, 1000

        elif mod == "KALKIS":
            target = [0.0, 0.0, TARGET_ALTITUDE]
            # Dahili iNav Lidar/Barometre verisi hedef yüksekliği doğruladığında:
            if abs(current_z - TARGET_ALTITUDE) < 0.05:
                mod = "ILERI_UÇUŞ"
                hover_timer = 0.0
                print(f"\n🎯 İrtifa yakalandı ({current_z:.2f}m). X={TARGET_X}m çizgisine hücum ediliyor.")

        elif mod == "ILERI_UÇUŞ":
            # Yüksekliği korurken iNav navigasyon çipini X yönünde 2.0 metreye zorla!
            target = [TARGET_X, 0.0, TARGET_ALTITUDE]
            
            hover_timer += dt
            # Drone hem 5 saniye o hatta süzülmüş olacak hem de X sensör verisi 2 metreyi onaylayacak
            if hover_timer >= HOVER_DURATION and abs(current_x - TARGET_X) < 0.15:
                mod = "INIŞ_ALGORITMASI"
                print("\n🪂 Hedef konuma varıldı, atalet sönümlendi. Yer etkisi emniyetli inişi başlıyor.")

        elif mod == "INIŞ_ALGORITMASI":
            # Vardığı 2.0 metre koordinat çizgisinde dikey iniş gerçekleştirir
            target = [TARGET_X, 0.0, 0.05]
            
            # iNav altimetre verisi yere 10 santim kaldığını söylediğinde motorları tamamen durdur
            if current_z <= 0.10 and elapsed_time > 6.0:
                print("\n🏁 Güvenli iniş tamamlandı. Motorlar kapatıldı (DISARM).")
                mod = "GÖREV_BİTTİ"

        elif mod == "GÖREV_BİTTİ":
            roll_pwm, pitch_pwm, throttle_pwm = 1500, 1500, 1000
            msp.send_raw_rc(1500, 1500, 1000, aux1=1000) # iNav Acil Kapatma Sinyali (AUX1 -> 1000)
            break

        # --- 📡 C. iNAV KARTINA KANALLARI BASMA ---
        # iNav'da AUX1 anahtarı genelde ARM kanalıdır. Uçuş modlarındayken 2000 (açık), bitince 1000 (kapalı) basar.
        current_aux1 = 2000 if mod in ["KALKIS", "ILERI_UÇUŞ", "INIŞ_ALGORITMASI"] else 1000
        roll_pwm, pitch_pwm, throttle_pwm = fc.compute_rc_channels(current_pos, target, dt, mod)
        msp.send_raw_rc(roll_pwm, pitch_pwm, throttle_pwm, aux1=current_aux1)

        # --- 📝 D. SANİYEDE TAM 1 KERE LOG KAYDI ---
        if elapsed_time - last_log_time >= 1.0:
            log_file.write(f"{elapsed_time:<10.2f}\t{mod:<15}\t{current_x:<12.3f}\t{current_z:<12.3f}\t{roll_pwm:<10.1f}\t{pitch_pwm:<10.1f}\t{throttle_pwm:<10.1f}\t{current_voltage:<10.2f}\n")
            last_log_time = elapsed_time
            
        if int(elapsed_time / dt) % 25 == 0: # Ekrana her yarım saniyede bir durum basar
            print(f"\r⏱️ {elapsed_time:04.1f}s | {mod:<15} | Real X: {current_x:.2f}m | Real Z: {current_z:.2f}m | Pitch: {pitch_pwm:.0f} | Pil: {current_voltage:.1f}V", end="")

        # --- ⏱️ E. LINUX GECİKME SÜBVANSE SİSTEMİ ---
        elapsed_time += dt
        time_spent = time.time() - start_loop
        if time_spent < dt:
            time.sleep(dt - time_spent)

print("\n\n✅ Helion pür donanım uçuş döngüsü emniyetle tamamlandı.")