import serial
import struct
import time

# Raspberry Pi'de uçuş kontrolcüsünün bağlı olduğu port
SERIAL_PORT = '/dev/ttyAMA0' 
BAUD_RATE = 115200

# MSP Komutları
MSP_SET_RAW_MOTORS = 214

def send_msp_command(ser, cmd, data):
    size = len(data)
    checksum = size ^ cmd
    for b in data:
        checksum ^= b
    
    # MSP Frame Yapısı: $ M < (yön) [size] [cmd] [payload] [checksum]
    header = struct.pack('<3sBB', b'$M<', size, cmd)
    ser.write(header + bytes(data) + struct.pack('<B', checksum))

def percent_to_pwm(percentage):
    """
    Yüzdelik değeri (0-100), FC'nin beklediği 1000-2000 aralığına çevirir.
    Güvenlik için girilen değeri 0 ile 100 arasında sınırlandırır (clamp).
    """
    percentage = max(0.0, min(100.0, float(percentage)))
    pwm_value = 1000 + (1000 * (percentage / 100.0))
    return int(pwm_value)

def set_motors_percent(ser, m1_pct, m2_pct, m3_pct, m4_pct):
    """
    Motorlara yüzdelik (%0 - %100) cinsinden güç verir.
    """
    # Yüzdeleri PWM aralığına çevir
    m1 = percent_to_pwm(m1_pct)
    m2 = percent_to_pwm(m2_pct)
    m3 = percent_to_pwm(m3_pct)
    m4 = percent_to_pwm(m4_pct)
    
    data = struct.pack('<HHHH', m1, m2, m3, m4)
    # Diğer 4 motor kanalını (toplam 8 kanal için) 1000 (kapalı) olarak dolduruyoruz
    data += struct.pack('<HHHH', 1000, 1000, 1000, 1000)
    
    send_msp_command(ser, MSP_SET_RAW_MOTORS, list(data))

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Bağlantı kuruldu: {SERIAL_PORT}")
    time.sleep(2) # Başlatma için bekle

    print("Motorlar test ediliyor...")
    print("DİKKAT: Pervanelerin ÇIKARILMIŞ olduğundan emin olun!")
    
    # Test Senaryosu: İlk başta bahsettiğiniz %7 - %9 değerlerini sırayla test edelim
    test_degerleri = [7, 8, 9] 
    
    for yuzde in test_degerleri:
        print(f"Motorlara %{yuzde} güç veriliyor...")
        start_time = time.time()
        
        # Her bir yüzdelik dilimde 2 saniye boyunca çalıştır
        while time.time() - start_time < 2:
            # Tüm motorlara aynı yüzdelik gücü veriyoruz
            set_motors_percent(ser, yuzde, yuzde, yuzde, yuzde)
            time.sleep(0.05) # MSP stabilite için periyodik gönderim gerekir
            
    # Motorları durdur ( %0 güç )
    set_motors_percent(ser, 0, 0, 0, 0)
    print("Test tamamlandı, motorlar durduruldu.")

except Exception as e:
    print(f"Hata: {e}")
finally:
    if 'ser' in locals():
        # Güvenlik: Kapanmadan önce motorlara kesin olarak %0 komutu gönder
        set_motors_percent(ser, 0, 0, 0, 0)
        ser.close()