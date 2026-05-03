#!/usr/bin/env python3
"""
Drone İrtifa Monitörü
=====================
Drone'u elimizle yukarı/aşağı hareket ettirirken
anlık irtifayı terminalde gösterir.

Kullanım:
  python3 altitude_monitor.py

Çıkmak için Ctrl+C
"""

import serial
import struct
import time
import sys

# ── Ayarlar ───────────────────────────────────────────────────
SERIAL_PORT  = '/dev/ttyAMA0'
BAUD_RATE    = 115200
REFRESH_HZ   = 20        # Saniyede kaç kez okusun (20 = her 50ms)
HISTORY_SIZE = 10        # Mini grafik için kaç sample tutulsun
# ─────────────────────────────────────────────────────────────


def build_bar(value_cm, max_cm=300, width=30):
    """İrtifayı basit bir ASCII bar olarak gösterir."""
    filled = int((value_cm / max_cm) * width)
    filled = max(0, min(width, filled))
    bar = '█' * filled + '░' * (width - filled)
    return f'[{bar}]'


def msp_request(ser, cmd):
    """MSP v1 komutu gönder, ham data listesi döndür."""
    cs = 0 ^ cmd  # payload yok, size=0
    frame = struct.pack('<3sBB', b'$M<', 0, cmd) + struct.pack('<B', cs & 0xFF)
    ser.reset_input_buffer()
    ser.write(frame)

    # Header oku
    hdr = ser.read(3)
    if len(hdr) < 3 or hdr not in (b'$M>', b'$M!'):
        return None

    raw = ser.read(2)
    if len(raw) < 2:
        return None
    size, _ = struct.unpack('<BB', raw)

    data = list(ser.read(size))
    ser.read(1)  # checksum (doğrulama yapmıyoruz, hız için)
    return data if len(data) == size else None


def get_altitude(ser):
    """(irtifa_cm, vario_cm_s) döndürür. Hata varsa (None, None)."""
    data = msp_request(ser, 109)  # MSP_ALTITUDE = 109
    if data and len(data) >= 6:
        alt   = struct.unpack('<i', bytes(data[0:4]))[0]  # signed int32
        vario = struct.unpack('<h', bytes(data[4:6]))[0]  # signed int16
        return alt, vario
    return None, None


def get_attitude(ser):
    """(roll, pitch, yaw) döndürür."""
    data = msp_request(ser, 108)  # MSP_ATTITUDE = 108
    if data and len(data) >= 6:
        roll  = struct.unpack('<h', bytes(data[0:2]))[0] / 10.0
        pitch = struct.unpack('<h', bytes(data[2:4]))[0] / 10.0
        yaw   = struct.unpack('<h', bytes(data[4:6]))[0]
        return roll, pitch, yaw
    return None, None, None


def main():
    print("=" * 55)
    print("   DRONE İRTİFA MONİTÖRÜ")
    print("   Drone'u elimizle hareket ettirin")
    print("   Çıkmak için  Ctrl+C")
    print("=" * 55)
    print(f"\n🔌 Bağlanıyor: {SERIAL_PORT} @ {BAUD_RATE} ...\n")

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
        time.sleep(2)
        print("✅ Bağlantı tamam!\n")
    except Exception as e:
        print(f"❌ Seri port açılamadı: {e}")
        sys.exit(1)

    # Referans irtifasını al (başlangıç noktası = 0)
    print("📍 Referans irtifası alınıyor...")
    ref_alt = None
    for _ in range(10):
        alt, _ = get_altitude(ser)
        if alt is not None:
            ref_alt = alt
            break
        time.sleep(0.1)

    if ref_alt is None:
        print("❌ İrtifa okunamadı! FC bağlı mı?")
        ser.close()
        sys.exit(1)

    print(f"✅ Referans: {ref_alt} cm (ham değer)")
    print(f"\n{'─' * 55}")
    print("Şimdi drone'u elimizle hareket ettirin...\n")

    interval   = 1.0 / REFRESH_HZ
    history    = []   # son N irtifa değeri (trend için)
    read_count = 0
    err_count  = 0

    try:
        while True:
            t_start = time.time()

            alt, vario = get_altitude(ser)
            roll, pitch, yaw = get_attitude(ser)

            if alt is None:
                err_count += 1
                sys.stdout.write(f"\r⚠️  Okuma hatası ({err_count})    ")
                sys.stdout.flush()
                time.sleep(interval)
                continue

            read_count += 1
            rel = alt - ref_alt   # başlangıca göre bağıl irtifa

            # Geçmiş kaydet (trend hesabı için)
            history.append(rel)
            if len(history) > HISTORY_SIZE:
                history.pop(0)

            # Trend: son N örnekten çıkarılır
            if len(history) >= 3:
                trend = history[-1] - history[0]
                if   trend >  10: trend_sym = '⬆️ '
                elif trend < -10: trend_sym = '⬇️ '
                else:             trend_sym = '➡️ '
            else:
                trend_sym = '   '

            # Dikey hız rengi (sadece pozitif/negatif işareti)
            if   vario >  20: vario_str = f'+{vario:4d} ↑'
            elif vario < -20: vario_str = f'{vario:4d} ↓'
            else:              vario_str = f'{vario:4d} —'

            # ASCII bar (maks 300 cm göster)
            bar = build_bar(max(0, rel), max_cm=300)

            # Attitude varsa göster
            att_str = ''
            if roll is not None:
                att_str = f'  R:{roll:+6.1f}°  P:{pitch:+6.1f}°  Y:{yaw:4d}°'

            # ── Ana çıktı satırı ──────────────────────────────
            line = (
                f'\r  İrtifa: {rel:5d} cm  {trend_sym} '
                f'│ Vario: {vario_str} cm/s '
                f'│ {bar}'
                f'{att_str}    '
            )
            sys.stdout.write(line)
            sys.stdout.flush()

            # 50 okumada bir istatistik satırı yaz
            if read_count % (REFRESH_HZ * 5) == 0:
                mn = min(history)
                mx = max(history)
                sys.stdout.write(
                    f'\n  [Son {HISTORY_SIZE} örnek]  '
                    f'Min: {mn:4d} cm  Max: {mx:4d} cm  '
                    f'Fark: {mx-mn:3d} cm\n'
                )

            # Döngü hızını koru
            elapsed = time.time() - t_start
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print(f'\n\n{"─" * 55}')
        print(f'  Toplam okuma : {read_count}')
        print(f'  Hata sayısı  : {err_count}')
        print('  Çıkılıyor...')

    finally:
        ser.close()
        print('🔌 Seri port kapatıldı.')


if __name__ == '__main__':
    main()