#!/usr/bin/env python3
"""
Motor Test + İrtifa Monitörü
=============================
Motorları düşük sabit hızda döndürür (drone kalkmaz),
siz elimizle indir/kaldır yaparken irtifayı gösterir.

Yöntem : MSP_SET_RAW_RC  (ana kodla aynı)
Mod     : ANGLE + ALTHOLD aktif, throttle sabit düşük

⚠️  UYARILAR:
  - Pervanesiz çalıştırın!
  - Drone'u yere sabitleyin ya da sıkıca tutun!
  - Ctrl+C → anında disarm!

Kullanım:
  python3 motor_test.py
"""

import serial
import struct
import time
import sys
import signal

# ── Ayarlar ───────────────────────────────────────────────────
SERIAL_PORT    = '/dev/ttyAMA0'
BAUD_RATE      = 115200
REFRESH_HZ     = 50          # RC gönderme + okuma hızı

# Throttle ayarı — drone KALKMAYACAK şekilde ayarla
# 1000 = tam kapalı, 1500 = orta (hover), 1200 = düşük test
# Pervanesiz test için 1200-1250 arası güvenli
TEST_THROTTLE  = 1200

# RC kanal indeksleri
CH_ROLL     = 0
CH_PITCH    = 1
CH_THROTTLE = 2
CH_YAW      = 3
CH_AUX1     = 4   # ARM
CH_AUX2     = 5   # ANGLE + NAV ALTHOLD

RC_LOW  = 1000
RC_MID  = 1500
RC_HIGH = 2000
# ─────────────────────────────────────────────────────────────

# Global seri port (emergency stop için)
ser = None


# ╔══════════════════════════════════════════════════════════════╗
# ║                    MSP FONKSİYONLARI                        ║
# ╚══════════════════════════════════════════════════════════════╝

def msp_send(s, cmd, payload=None):
    """MSP v1 komutu gönder, yanıt verisini döndür."""
    if payload is None:
        payload = []
    size = len(payload)
    cs = size ^ cmd
    for b in payload:
        cs ^= b

    frame = struct.pack('<3sBB', b'$M<', size, cmd)
    frame += bytes(payload) + struct.pack('<B', cs & 0xFF)

    s.reset_input_buffer()
    s.write(frame)

    # Yanıt oku
    hdr = s.read(3)
    if len(hdr) < 3 or hdr not in (b'$M>', b'$M!'):
        return None
    raw = s.read(2)
    if len(raw) < 2:
        return None
    sz, _ = struct.unpack('<BB', raw)
    data = list(s.read(sz))
    s.read(1)  # checksum
    return data if len(data) == sz else None


def set_raw_rc(s, channels):
    """8 kanallı RC değeri gönder (1000–2000)."""
    payload = []
    for ch in channels[:8]:
        payload.extend(struct.pack('<H', max(RC_LOW, min(RC_HIGH, ch))))
    msp_send(s, 200, payload)  # MSP_SET_RAW_RC = 200


def get_altitude(s):
    """(irtifa_cm, vario_cm_s) döndürür."""
    data = msp_send(s, 109)  # MSP_ALTITUDE
    if data and len(data) >= 6:
        alt   = struct.unpack('<i', bytes(data[0:4]))[0]
        vario = struct.unpack('<h', bytes(data[4:6]))[0]
        return alt, vario
    return None, None


def get_status(s):
    """arm durumu ve sensör bilgisi."""
    data = msp_send(s, 150)  # MSP_STATUS_EX
    if data and len(data) >= 15:
        box_flags = struct.unpack('<I', bytes(data[6:10]))[0]
        sensors   = struct.unpack('<H', bytes(data[4:6]))[0]
        return {
            'armed':    bool(box_flags & 1),
            'has_acc':  bool(sensors & (1 << 0)),
            'has_baro': bool(sensors & (1 << 1)),
        }
    return None


def get_attitude(s):
    data = msp_send(s, 108)  # MSP_ATTITUDE
    if data and len(data) >= 6:
        roll  = struct.unpack('<h', bytes(data[0:2]))[0] / 10.0
        pitch = struct.unpack('<h', bytes(data[2:4]))[0] / 10.0
        yaw   = struct.unpack('<h', bytes(data[4:6]))[0]
        return roll, pitch, yaw
    return None, None, None


def build_bar(value_cm, max_cm=300, width=25):
    filled = int((max(0, value_cm) / max_cm) * width)
    filled = min(width, filled)
    return '[' + '█' * filled + '░' * (width - filled) + ']'


# ╔══════════════════════════════════════════════════════════════╗
# ║                    EMERGENCY STOP                           ║
# ╚══════════════════════════════════════════════════════════════╝

def emergency_stop(sig=None, frame=None):
    """Ctrl+C → anında disarm."""
    print("\n\n🚨 ACİL DURUM — DISARM!")
    if ser and ser.is_open:
        rc = [RC_MID] * 8
        rc[CH_THROTTLE] = RC_LOW
        rc[CH_AUX1]     = RC_LOW   # disarm
        rc[CH_AUX2]     = RC_LOW
        for _ in range(50):        # 1 saniye boyunca gönder
            set_raw_rc(ser, rc)
            time.sleep(0.02)
        ser.close()
    print("🔒 Disarm edildi. Çıkılıyor.")
    sys.exit(0)

signal.signal(signal.SIGINT, emergency_stop)


# ╔══════════════════════════════════════════════════════════════╗
# ║                       MAIN                                  ║
# ╚══════════════════════════════════════════════════════════════╝

def main():
    global ser

    print("""
╔══════════════════════════════════════════════════════════╗
║          MOTOR TEST + İRTİFA MONİTÖRÜ                   ║
╠══════════════════════════════════════════════════════════╣
║  ⚠️  PERVANE TAKMADAN ÇALIŞTIRIN!                        ║
║  ⚠️  DRONE'U YERE SABİTLEYİN!                            ║
║  Ctrl+C → anında disarm                                 ║
╚══════════════════════════════════════════════════════════╝
""")

    # ── Bağlan ────────────────────────────────────────────────
    print(f"🔌 Bağlanıyor: {SERIAL_PORT} @ {BAUD_RATE} ...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.3)
        time.sleep(2)
        print("✅ Bağlantı tamam!\n")
    except Exception as e:
        print(f"❌ Seri port açılamadı: {e}")
        sys.exit(1)

    # ── Sensör kontrolü ───────────────────────────────────────
    print("🔍 Sensörler kontrol ediliyor...")
    st = get_status(ser)
    if not st:
        print("❌ FC'den yanıt alınamadı!")
        ser.close(); sys.exit(1)

    print(f"   ACC  : {'✅' if st['has_acc']  else '❌'}")
    print(f"   BARO : {'✅' if st['has_baro'] else '❌'}")

    if not st['has_acc'] or not st['has_baro']:
        print("❌ ACC ve BARO gerekli!")
        ser.close(); sys.exit(1)

    # ── Referans irtifa ───────────────────────────────────────
    print("\n📍 Referans irtifa alınıyor...")
    ref_alt = None
    for _ in range(15):
        alt, _ = get_altitude(ser)
        if alt is not None:
            ref_alt = alt
            break
        time.sleep(0.1)

    if ref_alt is None:
        print("❌ İrtifa okunamadı!")
        ser.close(); sys.exit(1)
    print(f"✅ Referans: {ref_alt} cm\n")

    # ── Başlangıç RC durumu ───────────────────────────────────
    rc = [RC_MID] * 8
    rc[CH_THROTTLE] = RC_LOW   # başta gaz kapalı
    rc[CH_AUX1]     = RC_LOW   # disarmed
    rc[CH_AUX2]     = RC_LOW

    # ── Arm bekleme ───────────────────────────────────────────
    print("⏳ Drone'u ARM etmenizi bekliyoruz...")
    print("   (iNAV Configurator'da veya kumandanızla arm edin)\n")

    rc[CH_AUX1] = RC_HIGH  # ARM sinyali hazır

    while True:
        set_raw_rc(ser, rc)
        st = get_status(ser)
        if st and st['armed']:
            print("✅ ARM edildi!\n")
            break
        time.sleep(0.3)

    # ── Throttle uygula ───────────────────────────────────────
    print(f"🔧 Throttle {TEST_THROTTLE} uygulanıyor (motorlar dönecek)...")
    print("   ANGLE + ALTHOLD aktif ediliyor...\n")

    rc[CH_AUX2]     = RC_HIGH       # ANGLE + ALTHOLD
    rc[CH_THROTTLE] = TEST_THROTTLE

    print("─" * 60)
    print("  Drone'u elimizle indir/kaldır — irtifa değişimini izleyin")
    print("  Ctrl+C → anında disarm")
    print("─" * 60 + "\n")

    interval  = 1.0 / REFRESH_HZ
    min_rel   =  9999
    max_rel   = -9999

    try:
        while True:
            t0 = time.time()

            # RC gönder (her döngüde — timeout olmasın)
            set_raw_rc(ser, rc)

            # Telemetri oku
            alt, vario = get_altitude(ser)
            roll, pitch, yaw = get_attitude(ser)

            if alt is None:
                sys.stdout.write("\r⚠️  Okuma hatası...    ")
                sys.stdout.flush()
                time.sleep(interval)
                continue

            rel = alt - ref_alt

            # Min/max takip
            min_rel = min(min_rel, rel)
            max_rel = max(max_rel, rel)

            # Trend
            if   vario >  15: trend = '⬆️ '
            elif vario < -15: trend = '⬇️ '
            else:              trend = '➡️ '

            # Bar
            bar = build_bar(max(0, rel), max_cm=200)

            # Attitude
            att = ''
            if roll is not None:
                att = f'  R:{roll:+5.1f}° P:{pitch:+5.1f}°'

            line = (
                f'\r  İrtifa: {rel:5d} cm {trend}'
                f'│ Vario: {vario:+4d} cm/s '
                f'│ {bar}'
                f'│ Thr:{TEST_THROTTLE}'
                f'{att}    '
            )
            sys.stdout.write(line)
            sys.stdout.flush()

            # Döngü hızını koru
            elapsed = time.time() - t0
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        pass  # emergency_stop signal handler devralır

    finally:
        print(f"\n\n{'─' * 60}")
        print(f"  Test özeti:")
        print(f"  En düşük : {min_rel:5d} cm")
        print(f"  En yüksek: {max_rel:5d} cm")
        print(f"  Fark     : {max_rel - min_rel:5d} cm")
        print(f"{'─' * 60}")
        emergency_stop()


if __name__ == '__main__':
    main()