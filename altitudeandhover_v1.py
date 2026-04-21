import serial  # connected FC with UART port
import struct  # transfer data to bytes
import time    
import sys     # clean exit
import signal  # emergency exit handling 

# DRONE CONFIG
# connect with USB --> '/dev/ttyACM0'
# connect TX/RX --->'/dev/ttyAMA0'
SERIAL_PORT         = '/dev/ttyAMA0' 
BAUD_RATE           = 115200         
TARGET_ALTITUDE_CM  = 200            # goal altitude in cm (2 meters)
HOLD_DURATION_SEC   = 20             # waiting at target altitude for 20 seconds
ALTITUDE_TOLERANCE  = 30             # tolerance in cm for considering "at altitude"
LAND_THRESHOLD_CM   = 15             # if altitude is below this, consider landed

# RC Constants (PWM values)
RC_LOW  = 1000  # motor stop or switch off
RC_MID  = 1500  # middle value (e.g., sticks centered)
RC_HIGH = 2000  # maximum value (e.g., switch fully on)

#Throttle values for different actions (PWM)
CLIMB_THROTTLE   = 1600 # for climbing
DESCEND_THROTTLE = 1350 # for descending
HOVER_THROTTLE   = 1500 # for stay posiition

#MSP Command Codes
MSP_STATUS_EX  = 150 # connect FC
MSP_ALTITUDE   = 109 # get altitude data from barometer
MSP_SET_RAW_RC = 200 # control motors with RC override (PWM values)

# MSP Communication Class
class MSP:

    def __init__(self, timeout=1.0):
        # UART connection to the flight controller (FC)
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=timeout)
        time.sleep(2)  

    def send(self, cmd, payload=None):
        #make sure payload is a list of bytes
        if payload is None:
            payload = []
        
        size = len(payload)
        cs = size ^ cmd
        for b in payload:
            cs ^= b
        
        frame = struct.pack('<3sBB', b'$M<', size, cmd)
        frame += bytes(payload) + struct.pack('<B', cs & 0xFF)
        
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        return self._read()

    def _read(self):
        #transfer data from FC to script, check header and checksum
        hdr = self.ser.read(3)
        if len(hdr) < 3 or hdr not in (b'$M>', b'$M!'):
            return None
        
        raw = self.ser.read(2)
        size, cmd = struct.unpack('<BB', raw)
        data = list(self.ser.read(size))
        cs = struct.unpack('<B', self.ser.read(1))[0]
        
        return {'cmd': cmd, 'data': data}

    # --- UÇUŞ İÇİN BİZE GEREKEN ÖZEL METOTLAR ---

    def set_raw_rc(self, channels):
        """Bu bizim sanal kumandamız! 8 kanallı listeyi baytlara çevirip gönderir."""
        payload = []
        for ch in channels[:8]: 
            # Güvenlik: Adım 1'deki RC_LOW (1000) ve RC_HIGH (2000) sabitlerini kullanıyoruz
            sinirlandirilmis_deger = max(RC_LOW, min(RC_HIGH, ch))
            payload.extend(struct.pack('<H', sinirlandirilmis_deger))
        
        self.send(MSP_SET_RAW_RC, payload) # Yine Adım 1'deki komut ID'si

    def get_altitude(self):
        #altitude 
        r = self.send(MSP_ALTITUDE) # Adım 1'deki komut ID'si
        if r and len(r['data']) >= 6:
            alt = struct.unpack('<i', bytes(r['data'][0:4]))[0] #altitude in cm
            vario = struct.unpack('<h', bytes(r['data'][4:6]))[0] #vertical speed in cm/s
            return alt, vario
        return None, None

    def close(self):
        #safely close the serial connection
        if self.ser and self.ser.is_open:
            self.ser.close()

#Flight Logic and Control System

class DroneController:

    def __init__(self, msp_connection):
        self.msp = msp_connection
        
        self.rc = [
            RC_MID,  # 0: Roll (1500 center)
            RC_MID,  # 1: Pitch (1500 center)
            RC_LOW,  # 2: Throttle (1000 bottom)
            RC_MID,  # 3: Yaw (1500 center)
            RC_LOW,  # 4: AUX1 (ARM switch off)
            RC_LOW,  # 5: AUX2 (Flight Mode switch off)
            RC_LOW,  # 6: AUX3 (Unused)
            RC_LOW   # 7: AUX4 (Unused)
        ] #initial state of RC channels (safety: throttle off, ARM switch off)

    def _send_rc(self):
        #Sends current RC values to FC.
        self.msp.set_raw_rc(self.rc)

    def arm(self):
        #arms the motors ready for flight
        print("Arming motors...")
        self.rc[2] = RC_LOW   # #throtte =0
        self.rc[4] = RC_HIGH  # arm switch open
        
        for _ in range(10): #use loop because some FCs require multiple packets to recognize the command
            self._send_rc()
            time.sleep(0.05)
        print("Drone is ARMED!")

    def disarm(self):
        #safe disarm
        print("Disarming motors...")
        self.rc[2] = RC_LOW  # throtte=0
        self.rc[4] = RC_LOW  # arm switch off
        
        for _ in range(10): #use loop because some FCs require multiple packets to recognize the command
            self._send_rc()
            time.sleep(0.05)
        print("Drone is DISARMED.")

    def run_mission(self):
        #main mission logic: takeoff, hover, land
        
        # 1. Arm motors
        self.arm()
        time.sleep(1) # Wait for props to spin up

        # 2. Enable flight mode and start climbing
        print(f"\nTAKEOFF INITIATED! Target: {TARGET_ALTITUDE_CM} cm")
        self.rc[5] = RC_HIGH          # flight mode open
        self.rc[2] = CLIMB_THROTTLE   # throte = climb value
        
        # --- CLIMB LOOP ---
        while True:
            self._send_rc() #every loop, send the current RC values to ensure the drone keeps climbing
            alt, vario = self.msp.get_altitude() #save values
            
            if alt is not None:
                sys.stdout.write(f"\r   Current Altitude: {alt:4d} cm")
                sys.stdout.flush()
                
                if alt >= TARGET_ALTITUDE_CM - ALTITUDE_TOLERANCE: #control target
                    print(f"\nTarget altitude reached!")
                    break
            time.sleep(0.05) 

        # 3. Hover
        print(f"\nHovering for {HOLD_DURATION_SEC} seconds...")
        self.rc[2] = HOVER_THROTTLE 
        
        start_time = time.time()
        # --- HOVER LOOP ---
        while time.time() - start_time < HOLD_DURATION_SEC: #loop devam duration boyunca
            self._send_rc() 
            
            remaining_time = HOLD_DURATION_SEC - (time.time() - start_time)
            sys.stdout.write(f"\r   Time Remaining: {remaining_time:4.1f} s")
            sys.stdout.flush()
            
            time.sleep(0.05)

        """     
        # 3. Hover (Aktif İrtifa Düzeltme Modu)
        print(f"\n⏸️ Aktif Hover Başlıyor... Hedef: {TARGET_ALTITUDE_CM} cm")
        
        start_time = time.time()
        
        # --- AKTİF KONTROL PARAMETRELERİ ---
        DEADBAND = 10         # ±10 cm içindeyse müdahale etme (iNAV'a güven)
        Kp = 1.5              # Oransal Kazanç (Hata başına eklenecek gaz miktarı)
        MAX_THROTTLE = 1650   # Düzeltme yaparken verilecek maksimum gaz
        MIN_THROTTLE = 1350   # Düzeltme yaparken verilecek minimum gaz
        
        # --- HOVER DÖNGÜSÜ (20Hz) ---
        while True:
            # 1. Zamanı Kontrol Et
            elapsed_time = time.time() - start_time
            remaining_time = HOLD_DURATION_SEC - elapsed_time
            
            if remaining_time <= 0:
                print("\n✅ Hover başarıyla tamamlandı.")
                break

            # 2. Sensörleri Oku
            alt, _ = self.msp.get_altitude()
            vbat = self.msp.get_battery()
            
            if alt is not None:
                # 3. HATA HESAPLAMA (Error = Target - Current)
                error = TARGET_ALTITUDE_CM - alt
                
                # 4. PATRONUN KARAR MEKANİZMASI (Senin Mantığın)
                if abs(error) <= DEADBAND:
                    # Drone hedefe çok yakın (Örn: 195cm - 205cm arası). Karışma, 1500 ver!
                    guncel_gaz = HOVER_THROTTLE
                    durum_mesaji = "Sabit "
                else:
                    # Drone hedeften uzaklaştı. Hata payına (error) göre gaz hesapla.
                    # Eğer hata pozitifse (aşağıdayız) gaz artar, negatifse (yukarıdayız) gaz azalır.
                    hesaplanan_gaz = HOVER_THROTTLE + (error * Kp)
                    
                    # Güvenlik: Uçuş kontrolcüsüne saçma sapan değerler gitmesin diye sınırlandır
                    guncel_gaz = max(MIN_THROTTLE, min(MAX_THROTTLE, int(hesaplanan_gaz)))
                    
                    durum_mesaji = "Düzeltiliyor"

                # Kumanda dizimizi yeni gaz değeriyle güncelle ve FC'ye yolla
                self.rc[2] = guncel_gaz
                self._send_rc() 
                
                # 5. Ekrana Canlı Dashboard Bas
                if vbat is not None:
                    sys.stdout.write(
                        f"\r   ⏳ {remaining_time:4.1f}s | 🔋 {vbat:.1f}V | "
                        f"📏 İrtifa: {alt:4d}cm (Hata: {error:+3d}) | "
                        f"🚀 Gaz: {guncel_gaz} | Durum: {durum_mesaji}    "
                    )
                    sys.stdout.flush()
                
                # (İsteğe bağlı: Burada yine pili kontrol edip acil iniş komutu verebilirsin)
                if vbat is not None and vbat < MIN_SAFE_VOLTAGE:
                    print(f"\n\n🚨 CRITICAL BATTERY ({vbat}V) DETECTED! Acil İniş!")
                    break
            
            time.sleep(0.05) # 20Hz Döngü (Senin istediğin yenileme hızı)    
        """
        # 4. Descent
        print("\n\nLANDING INITIATED...")
        self.rc[2] = DESCEND_THROTTLE #throte = descend pwm
        
        while True:
            self._send_rc()
            alt, vario = self.msp.get_altitude() #save values
            
            if alt is not None:
                sys.stdout.write(f"\r   Current Altitude: {alt:4d} cm")
                sys.stdout.flush()
                
                if alt <= LAND_THRESHOLD_CM:
                    print(f"\nTouchdown! ({alt} cm)")
                    break
            time.sleep(0.05)

        # 5. End mission
        self.disarm()

# main

def main():
    print("Autonomous Flight System Initializing...")
    
    #serial port connection
    try:
        connection = MSP()
        print("Connected to Flight Controller.")
    except Exception as e:
        print(f"Connection error: Could not open serial port! Details: {e}")
        return # Exit the program if connection fails
        
    #connect to drone controller logic
    drone = DroneController(connection)
    
    # countdown
    print("\nWARNING! Keep clear of the propellers!")
    for i in range(5, 0, -1):
        sys.stdout.write(f"\r Flight starting in {i} seconds... ")
        sys.stdout.flush()
        time.sleep(1)
    print("\n")

    #start mission
    try:
        drone.run_mission()
        
    except KeyboardInterrupt:
        # This block catches CTRL+C if you need to abort the flight manually
        print("\n\nEMERGENCY STOP (CTRL+C) DETECTED!")
        drone.disarm() # Immediately kill the motors!
        
    finally:
        #safe close
        drone.disarm() # Ensure motors are off
        connection.close()
        print("Serial port connection closed. System is safe.")
        

#main() runs only when this script is executed directly
if __name__ == '__main__':
    main()
