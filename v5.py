#!/usr/bin/env python3
"""
Otonom Drone Kontrolcüsü - Sabit Gaz (Sensörsüz Kör Uçuş)
======================================================
Yarış dronları ve barometre parazitleri için idealdir.
Sensör verisi okumaz, sadece belirlenen sürelerde 
belirlenen gaz (throttle) değerlerini FC'ye basar.
Kalkış, bekleme, iniş ve acil durum anları 'drone_ucus_log.txt' 
dosyasına zaman damgalı olarak kaydedilir.
"""

import serial
import struct
import time
import sys
import signal
import logging

# ╔══════════════════════════════════════════════════════════════╗
# ║                   LOGLAMA AYARLARI                           ║
# ╚══════════════════════════════════════════════════════════════╝
# Konsola ve 'drone_ucus_log.txt' dosyasına aynı anda kayıt yapar
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[
        logging.FileHandler("drone_ucus_log.txt", mode='a', encoding='utf-8'),
        logging.StreamHandler(sys.stdout)
    ]
)

SERIAL_PORT         = '/dev/ttyAMA0'
BAUD_RATE           = 115200

# -- Uçuş Senaryosu Süreleri --
KALKIS_SURESI_SN    = 3
BEKLEME_SURESI_SN   = 5
INIS_SURESI_SN      = 3

# -- Gaz (Throttle) Ayarları --
KALKIS_GAZI         = 1340      # Drone'u yavaşça kaldıracak gaz
HOVER_GAZI          = 1300      # Dronun havada tutunacağı gaz
INIS_GAZI           = 1250      # Dronun yavaşça çökeceği gaz

RC_LOOP_HZ          = 50
interval            = 1.0 / RC_LOOP_HZ

# ╔══════════════════════════════════════════════════════════════╗
# ║              MSP (MultiWii Serial Protocol) ID'leri          ║
# ╚══════════════════════════════════════════════════════════════╝
# Uçuş kontrolcüsünden "Sensörlerin ne durumda? ARM edildin mi?" bilgisini çeker.
MSP_STATUS_EX       = 150

# Python'daki Sanal Kumanda komutlarımızı uçuş kontrolcüsüne yollar.
MSP_SET_RAW_RC      = 200

# ╔══════════════════════════════════════════════════════════════╗
# ║              RC KANAL İNDEKSLERİ (KANAL SIRALAMASI)          ║
# ╚══════════════════════════════════════════════════════════════╝
CH_ROLL     = 0   # Sağ/Sol yatma
CH_PITCH    = 1   # İleri/Geri eğilme
CH_THROTTLE = 2   # GAZ (Kodun yönettiği ana kanal)
CH_YAW      = 3   # Kendi etrafında dönme
CH_AUX1     = 4   # ARM Şalteri
CH_AUX2     = 5   # Uçuş Modu Şalteri (ANGLE)

RC_LOW  = 1000
RC_MID  = 1500
RC_HIGH = 2000

class MSP:
    def __init__(self, port, baudrate):
        logging.info(f"Seri port açılıyor: {port} @ {baudrate}")
        self.ser = serial.Serial(port, baudrate, timeout=1.0)
        time.sleep(2)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            logging.info("Seri port kapatıldı.")

    def send(self, cmd, payload=None):
        if payload is None: payload = []
        size = len(payload)
        cs = size ^ cmd
        for b in payload: cs ^= b

        frame = struct.pack('<3sBB', b'$M<', size, cmd)
        frame += bytes(payload) + struct.pack('<B', cs & 0xFF)

        self.ser.reset_input_buffer()
        self.ser.write(frame)
        return self._read()

    def _read(self):
        hdr = self.ser.read(3)
        if len(hdr) < 3 or hdr not in (b'$M>', b'$M!'): return None
        raw = self.ser.read(2)
        if len(raw) < 2: return None
        size, cmd = struct.unpack('<BB', raw)
        data = list(self.ser.read(size))
        if len(data) < size: return None
        cs = struct.unpack('<B', self.ser.read(1))[0]
        expected = size ^ cmd
        for b in data: expected ^= b
        if cs != (expected & 0xFF): return None
        return {'cmd': cmd, 'data': data, 'error': hdr == b'$M!'}

    def get_status(self):
        r = self.send(MSP_STATUS_EX)
        if not r or r['error'] or len(r['data']) < 15: return None
        box_flags = struct.unpack('<I', bytes(r['data'][6:10]))[0]
        return {'armed': bool(box_flags & 1)}

    def set_raw_rc(self, channels):
        payload = []
        for ch in channels[:16]:
            payload.extend(struct.pack('<H', max(RC_LOW, min(RC_HIGH, ch))))
        self.send(MSP_SET_RAW_RC, payload)


class DroneController:
    def __init__(self, msp: MSP):
        self.msp = msp
        self.abort = False
        
        self.rc = [RC_MID] * 8
        self.rc[CH_THROTTLE] = RC_LOW
        self.rc[CH_AUX1] = RC_LOW   
        self.rc[CH_AUX2] = RC_LOW   
        
        signal.signal(signal.SIGINT, self._emergency_signal)

    def _emergency_signal(self, signum, frame):
        self._tetikle_acil_cikis("SİNYAL (CTRL+C)")

    def _tetikle_acil_cikis(self, kaynak):
        if not self.abort:
            logging.error(f"[{kaynak}] ACİL DURDURMA TETİKLENDİ! MOTORLAR KESİLİYOR!")
            self.abort = True
            self.disarm()
            sys.exit(1)

    def _tx(self):
        self.msp.set_raw_rc(self.rc)

    def wait_for_arm(self):
        logging.info("Kumandadan ARM anahtarının açılması bekleniyor...")
        self.rc[CH_AUX1] = RC_HIGH
        while not self.abort:
            st = self.msp.get_status()
            if st and st['armed']:
                logging.info("Kumandadan ARM sinyali algılandı.")
                return True
            time.sleep(0.5)
        return False

    def disarm(self):
        logging.info("Disarm ediliyor (Motorlar durduruluyor)...")
        self.rc[CH_THROTTLE] = RC_LOW
        self.rc[CH_AUX1] = RC_LOW
        self.rc[CH_AUX2] = RC_LOW
        
        for _ in range(50): 
            self._tx()
            time.sleep(0.02)
        logging.info("Motorlar güvenle disarm edildi.")

    def ucus_baslat(self):
        logging.info("=== OTONOM UÇUŞ SENARYOSU BAŞLATILDI ===")

        try:
            if not self.wait_for_arm(): return
            time.sleep(1)

            self.rc[CH_AUX2] = RC_HIGH # ANGLE modu açık
            
            # ==========================================
            # AŞAMA 1: KALKIŞ
            # ==========================================
            logging.info(f"AŞAMA 1 (KALKIŞ): {KALKIS_GAZI} gaz ile {KALKIS_SURESI_SN} saniye kalkış yapılıyor.")
            self.rc[CH_THROTTLE] = KALKIS_GAZI
            t0 = time.time()
            while time.time() - t0 < KALKIS_SURESI_SN and not self.abort:
                self._tx()
                # Sadece ekranda zamanın akması için (log dosyasına yazılmaz)
                sys.stdout.write(f"\rKalkış süresi: {time.time()-t0:.1f} sn   ")
                sys.stdout.flush()
                time.sleep(interval)
            print() # Satır atla
            
            # ==========================================
            # AŞAMA 2: HAVADA TUTMA (HOVER)
            # ==========================================
            logging.info(f"AŞAMA 2 (HOVER): {HOVER_GAZI} gaz ile {BEKLEME_SURESI_SN} saniye havada tutuluyor.")
            self.rc[CH_THROTTLE] = HOVER_GAZI
            t0 = time.time()
            while time.time() - t0 < BEKLEME_SURESI_SN and not self.abort:
                self._tx()
                sys.stdout.write(f"\rHover süresi: {time.time()-t0:.1f} sn   ")
                sys.stdout.flush()
                time.sleep(interval)
            print()
            
            # ==========================================
            # AŞAMA 3: İNİŞ
            # ==========================================
            logging.info(f"AŞAMA 3 (İNİŞ): {INIS_GAZI} gaz ile {INIS_SURESI_SN} saniye iniş yapılıyor.")
            self.rc[CH_THROTTLE] = INIS_GAZI
            t0 = time.time()
            while time.time() - t0 < INIS_SURESI_SN and not self.abort:
                self._tx()
                sys.stdout.write(f"\rİniş süresi: {time.time()-t0:.1f} sn   ")
                sys.stdout.flush()
                time.sleep(interval)
            print()

            self.disarm()
            logging.info("=== UÇUŞ BAŞARIYLA TAMAMLANDI ===")

        except KeyboardInterrupt:
            self._tetikle_acil_cikis("KLAVYE (CTRL+C)")
        except Exception as e:
            logging.error(f"Beklenmeyen Hata Oluştu: {e}")
            self.disarm()

if __name__ == '__main__':
    try:
        msp = MSP(SERIAL_PORT, BAUD_RATE)
        drone = DroneController(msp)
        
        onay = input("Hazırsanız uçuşu başlatmak için 'E' yazın (Çıkmak için Q): ").strip().upper()
        if onay == 'E':
            logging.info("Kullanıcı uçuşu onayladı. Ellerinizi drondan çekin.")
            time.sleep(2)
            drone.ucus_baslat()
        else:
            logging.info("Kullanıcı uçuşu iptal etti.")
            
    except KeyboardInterrupt:
        logging.warning("Kullanıcı programı CTRL+C ile sonlandırdı.")
    except Exception as e:
        logging.error(f"Başlatma Hatası: {e}")
    finally:
        if 'msp' in locals(): msp.close()