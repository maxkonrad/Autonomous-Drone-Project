import serial
import struct
import time

# DRONE CONFIG
# connect with USB --> '/dev/ttyACM0'
# connect TX/RX --->'/dev/ttyAMA0'
SERIAL_PORT = '/dev/ttyACMM0' 
BAUD_RATE = 115200  # standart iletişim hızı drone projeleri için

# MSP Command Codes
MSP_SET_RAW_MOTORS = 214

def send_msp_command(ser, cmd, data):
    """iNav'ın anladığı MSP v1 paketini paketleyip gönderir."""
    size = len(data)
    # Checksum: Veri boyutu ile Komut ID'sinin XOR işlemine sokulmuş halidir.
    checksum = size ^ cmd
    for b in data:
        checksum ^= b
    
    # $M< + Boyut + Komut + Veri + Checksum
    header = struct.pack('<3sBB', b'$M<', size, cmd)
    ser.write(header + bytes(data) + struct.pack('<B', checksum))

def set_motors(ser, m1, m2, m3, m4):
    """8 motor kanalını da doldurup iNav'a gönderir."""
    # iNav 8 kanal bekler. İlk 4 motor aktif, son 4 motor kapalı
    #motor work values: 1000: close-2000: full throttle
    data = struct.pack('<HHHHHHHH', m1, m2, m3, m4, 1000, 1000, 1000, 1000)
    send_msp_command(ser, MSP_SET_RAW_MOTORS, list(data))

def run_motor_test():
    print("[SYSTEM] Starting motor test")
    
    try:
        # Seri Port Bağlantısı
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"[CONNECTION] {SERIAL_PORT} successfully opened.")
        time.sleep(2)

        # 1. PART: WAIT AND IDLE (ARM)
        print("[IDLE] Motors will be idle for 1 seconds...")
        set_motors(ser, 1000, 1000, 1000, 1000)
        time.sleep(1)

        # 2. PART: THROTTLE UP (TEST)
        # DroneKit'teki '1100 PWM' verme aşaması
        print("[ACTION] Motors work. (5 seconds at 1150 PWM)")
        start_time = time.time()
        while time.time() - start_time < 5:
            # iNav'da motorların dönmesi için bu komutu sürekli (loop) göndermelisin
            set_motors(ser, 1150, 1150, 1150, 1150)
            time.sleep(0.05) # 20Hz hızında gönderim (Stabilitenin sağlansın diye)

        # 3. PART: STOP (DISARM)
        print("[STOPPING] Test completed, stopping motors.")
        set_motors(ser, 1000, 1000, 1000, 1000)
        
        print("[FINISHED] Motorlar durduruldu. Sistem güvenli.")

    except Exception as e:
        print(f"[CRITICAL ERROR] Hata oluştu: {e}")
    
    finally:
        if 'ser' in locals() and ser.is_open:
            # Her ihtimale karşı motorları kapat ve portu bırak
            set_motors(ser, 1000, 1000, 1000, 1000)
            ser.close()
            print("[SYSTEM] Port successfully closed.")

if __name__ == "__main__":
    run_motor_test()