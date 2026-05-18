import serial
import struct
import time
import collections

# ============================================================
#  AYARLAR
# ============================================================
SERIAL_PORT   = '/dev/ttyACM0'
BAUD_RATE     = 115200

TARGET_ALT_CM     = 200    # Hedef irtifa (cm)
HOVER_DURATION    = 10.0   # Havada asılı kalma süresi (saniye)
CLIMB_THROTTLE    = 1150   # Yükseliş gas değeri — drone'a göre ayarla
DESCENT_THROTTLE  = 1050   # İniş gas değeri — drone'a göre ayarla
HOVER_THROTTLE    = 1500   # INAV ALTHOLD nötr gas

LOOP_HZ    = 20
LOOP_DT    = 1.0 / LOOP_HZ

# EMA filtresi katsayısı (0.0-1.0 arası; düşük = daha yumuşak)
EMA_ALPHA  = 0.2

# İniş onay sayacı (ardışık "yerde" okuması)
LAND_CONFIRM_NEEDED = 5

# ============================================================
#  MSP YARDIMCI FONKSİYONLAR
# ============================================================
def send_msp(ser, cmd, data=[]):
    """MSP v1 paketi gönderir."""
    size     = len(data)
    checksum = size ^ cmd
    for b in data:
        checksum ^= b
    packet = struct.pack('<3sBB', b'$M<', size, cmd) + bytes(data) + struct.pack('<B', checksum)
    ser.write(packet)


def read_msp_response(ser, expected_cmd, timeout=0.1):
    """
    Header'ı byte-by-byte arayarak güvenli MSP okuma.
    Yarış koşullarına karşı dayanıklı.
    """
    deadline = time.time() + timeout
    buf = collections.deque(maxlen=3)

    while time.time() < deadline:
        if ser.in_waiting == 0:
            time.sleep(0.001)
            continue
        byte = ser.read(1)
        if not byte:
            continue
        buf.append(byte)

        # Header: $ M >
        if buf == collections.deque([b'$', b'M', b'>'], maxlen=3):
            header = ser.read(2)   # size, cmd
            if len(header) < 2:
                continue
            size, cmd = header[0], header[1]
            if cmd != expected_cmd:
                continue
            payload  = ser.read(size)
            _chk     = ser.read(1)  # checksum (basit doğrulama eklenebilir)
            if len(payload) == size:
                return payload
    return None


def send_rc(ser, roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4):
    """
    MSP_SET_RAW_RC (200)
    INAV kanal sırası: Roll, Pitch, Throttle, Yaw, AUX1..AUX4
    """
    data = struct.pack('<8H', roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4)
    send_msp(ser, 200, list(data))


def read_altitude_raw(ser):
    """
    MSP_ALTITUDE (109) → INAV tahmini irtifa (cm)
    Payload: int32 irtifa, int16 vario
    """
    send_msp(ser, 109)
    payload = read_msp_response(ser, 109, timeout=0.08)
    if payload and len(payload) >= 4:
        return struct.unpack('<i', payload[:4])[0]
    return None


def read_rc_channels(ser):
    """
    MSP_RC (105) → RC kanal değerleri
    Dönen tuple: (ch1, ch2, ch3, ch4, ch5, ch6, ...)
    """
    send_msp(ser, 105)
    payload = read_msp_response(ser, 105, timeout=0.08)
    if payload and len(payload) >= 12:
        return struct.unpack('<6H', payload[:12])
    return None


# ============================================================
#  EMA FİLTRESİ
# ============================================================
class EMAFilter:
    def __init__(self, alpha, init_val=0.0):
        self.alpha = alpha
        self.value = init_val
        self.initialized = False

    def update(self, raw):
        if not self.initialized:
            self.value = raw
            self.initialized = True
        else:
            self.value = self.alpha * raw + (1 - self.alpha) * self.value
        return self.value


# ============================================================
#  ANA GÖREV DÖNGÜSÜ
# ============================================================
def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2)  # FC boot bekleme
    ser.reset_input_buffer()

    alt_filter    = EMAFilter(EMA_ALPHA)
    filtered_alt  = 0.0
    last_valid_alt = 0.0  # None gelirse son geçerli değeri koru

    mission_state      = "IDLE"
    hover_start_time   = 0.0
    arm_wait_start     = 0.0
    land_confirm_count = 0

    # Watchdog: son RC gönderim zamanı
    last_send_time     = time.time()
    MAX_SILENT_SEC     = 0.5  # Bu kadar süre komut gönderilmezse FC failsafe devreye girer

    print("\n╔══════════════════════════════════════╗")
    print("║   INAV OTONOM UÇUŞ — HAZIR          ║")
    print("╠══════════════════════════════════════╣")
    print("║  CH5 > 1500 → ARM                   ║")
    print("║  CH6 > 1500 → MSP OVERRIDE + BAŞLAT ║")
    print("║  CH6 < 1500 → ACİL İPTAL            ║")
    print("╚══════════════════════════════════════╝\n")

    try:
        while True:
            loop_start = time.time()

            # ── 1. SENSÖR OKUMA ─────────────────────────────
            raw_alt = read_altitude_raw(ser)
            if raw_alt is not None:
                filtered_alt  = alt_filter.update(raw_alt)
                last_valid_alt = filtered_alt
            else:
                filtered_alt  = last_valid_alt  # Kayıp pakette paniğe gerek yok

            current_alt = filtered_alt

            # ── 2. RC OKUMA (her 2 loop'ta bir — zamanlama tasarrufu) ──
            rc = read_rc_channels(ser)
            aux1_arm      = rc[4] if rc else 1000   # CH5: ARM
            aux2_override = rc[5] if rc else 1000   # CH6: MSP OVERRIDE

            # ── 3. DURUM MAKİNESİ ───────────────────────────

            # ACİL İPTAL — her an CH6 inince
            if aux2_override < 1500 and mission_state not in ("IDLE",):
                print("\n⚠️  PILOT OTONOMIYI İPTAL ETTİ — KONTROL SİZDE!")
                mission_state = "IDLE"

            # --- IDLE ---
            if mission_state == "IDLE":
                # Komut göndermiyoruz; FC kendi failsafe/RC'sine göre davranır
                pass

            # --- BAŞLATMA BEKLEMESİ ---
            elif mission_state == "ARMED_WAIT":
                # sleep() YOK — döngü çalışmaya devam eder
                send_rc(ser, 1500, 1500, 1000, 1500, 2000, 2000, 1000, 1000)
                if time.time() - arm_wait_start >= 2.0:
                    mission_state = "CLIMBING"
                    alt_filter.initialized = False  # Filtreyi sıfırla
                    print(f"\n🚀 KALKIŞ BAŞLIYOR | Hedef: {TARGET_ALT_CM} cm")

            # --- YÜKSELİŞ (Python throttle verir, ALTHOLD KAPALI) ---
            elif mission_state == "CLIMBING":
                # AUX3 = 1000 → ALTHOLD kapalı, Python kontrol eder
                send_rc(ser, 1500, 1500, CLIMB_THROTTLE, 1500, 2000, 2000, 1000, 1000)
                print(f"  🔼 YÜKSELİYOR | İrtifa: {int(current_alt)} cm / {TARGET_ALT_CM} cm")

                if current_alt >= TARGET_ALT_CM - 20:
                    # Hedefe yaklaştık → ALTHOLD'u AÇ, INAV bu irtifayı kilitler
                    mission_state = "HOVERING"
                    hover_start_time = time.time()
                    print(f"\n✅ HOVER | INAV ALTHOLD KİLİTLENDİ @ {int(current_alt)} cm")
                    print(f"   {HOVER_DURATION:.0f} saniye asılı kalınacak...\n")

            # --- HOVER (INAV ALTHOLD, Python sadece modu açık tutar) ---
            elif mission_state == "HOVERING":
                # AUX3 = 2000 → ALTHOLD açık, throttle nötrde
                # INAV kendi PID'iyle irtifayı tutar
                send_rc(ser, 1500, 1500, HOVER_THROTTLE, 1500, 2000, 2000, 2000, 1000)

                elapsed = time.time() - hover_start_time
                remaining = HOVER_DURATION - elapsed
                print(f"  🔵 HOVER | İrtifa: {int(current_alt)} cm | Kalan: {remaining:.1f}s")

                if elapsed >= HOVER_DURATION:
                    mission_state = "DESCENDING"
                    land_confirm_count = 0
                    print("\n⬇️  İNİŞ BAŞLIYOR | ALTHOLD KAPATILUYOR")

            # --- İNİŞ (ALTHOLD kapalı, Python throttle düşürür) ---
            elif mission_state == "DESCENDING":
                # AUX3 = 1000 → ALTHOLD kapalı, Python kontrol eder
                send_rc(ser, 1500, 1500, DESCENT_THROTTLE, 1500, 2000, 2000, 1000, 1000)
                print(f"  🔽 İNİYOR | İrtifa: {int(current_alt)} cm")

                # Güvenli iniş tespiti: ardışık 5 okuma yerde olmalı
                if current_alt <= 15:
                    land_confirm_count += 1
                    if land_confirm_count >= LAND_CONFIRM_NEEDED:
                        mission_state = "LANDED"
                        print("\n✅ İNİŞ BAŞARILI!")
                else:
                    land_confirm_count = 0  # Sıfırla — sürekli olmalı

            # --- İNİŞ TAMAMLANDI ---
            elif mission_state == "LANDED":
                # Throttle 1000, ARM kanalı da düşür → motorlar söner
                send_rc(ser, 1500, 1500, 1000, 1500, 1000, 2000, 1000, 1000)
                print("🏁 GÖREV TAMAMLANDI — CH6'yı indirerek sistemi kapatın.")
                # Birkaç kez gönder, sonra IDLE'a geç
                for _ in range(10):
                    send_rc(ser, 1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000)
                    time.sleep(0.05)
                mission_state = "IDLE"

            # ── 4. BAŞLATMA TETİKLEYİCİ ─────────────────────
            # (IDLE'dayken CH5+CH6 açılırsa görevi başlat)
            if mission_state == "IDLE" and aux1_arm > 1500 and aux2_override > 1500:
                mission_state  = "ARMED_WAIT"
                arm_wait_start = time.time()
                print("\n[+] Otonomi aktif! 2 saniye sonra kalkış...")

            # ── 5. DÖNGÜ ZAMANLAMASI (20 Hz) ─────────────────
            elapsed = time.time() - loop_start
            sleep_time = LOOP_DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\n⛔ KLAVYEDEN DURDURULDU")

    finally:
        print("Güvenli kapatma...")
        for _ in range(10):
            send_rc(ser, 1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000)
            time.sleep(0.05)
        ser.close()
        print("✅ Port kapatıldı. Güvenli.")


if __name__ == "__main__":
    main()