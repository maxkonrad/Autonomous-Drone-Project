import serial
import struct
import time
import sys
import logging

"""BU KODDA: ARM olunca motorlar düşük sesle uğuldar, 1050 civarında. Kod çalışırken:

Drone'u elinle hafifçe sağa-sola eğ
Log'da Z_net değerinin değiştiğini göreceksin
PID'in buna tepki olarak Thr değerini artırıp azalttığını göreceksin

Bu şekilde PID'in tepki verip vermediğini, Z ekseninin doğru okunup okunmadığını anlayabilirsin."""

# --- LOG SİSTEMİ ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler("drone_hover_pid.log", encoding='utf-8'),
        logging.StreamHandler(sys.stdout)
    ]
)

# ==============================================================
#  BURASI DENEME YANIL MA PANELİ — sadece burayı değiştir
# ==============================================================
SERIAL_PORT     = '/dev/ttyAMA0'
BAUD_RATE       = 115200

START_THROTTLE  = 1050   # Başlangıç gazı
MAX_THROTTLE    = 1400   # Güvenlik tavanı — asla geçilmez
MIN_THROTTLE    = 1000   # Güvenlik tabanı

FLIGHT_DURATION = 3.0    # Kaç saniye havada kalsın
LOOP_HZ         = 50     # Döngü frekansı

# --- THROTTLE PID KAZANÇLARI ---
# Hedef: Z ivmesi 0 m/s² (ne yukarı ne aşağı gitmek)
# İlk denemede küçük P ile başla, titreşim varsa düşür
P_GAIN = 2.0    # Büyütürsen tepki hızlı ama titreşir
I_GAIN = 0.05   # İntegral — zamana göre birikim (dikkatli artır)
D_GAIN = 0.5    # Türev — salınımı söndürür

INTEGRAL_LIMIT  = 100.0  # Anti-windup sınırı
# ==============================================================


RC_MID = 1500


class PIDController:
    """Throttle PID — anti-windup ve dt=0 korumalı"""
    """Anti-windup, PID'in integral teriminin kontrolden çıkmasını önleyen bir koruma mekanizması."""

    def __init__(self, p, i, d, integral_limit=INTEGRAL_LIMIT):
        self.kp = p
        self.ki = i
        self.kd = d
        self.integral_limit = integral_limit
        self.prev_error = 0.0
        self.integral   = 0.0

    def calculate(self, target, current, dt):
        dt = max(dt, 1e-4)
        error = target - current
        self.integral += error * dt
        self.integral  = max(-self.integral_limit,
                             min(self.integral_limit, self.integral))
        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return output

    def reset(self):
        self.prev_error = 0.0
        self.integral   = 0.0


class DroneHoverControl:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
        self.pid = PIDController(P_GAIN, I_GAIN, D_GAIN)

        # RC: [Roll, Pitch, Throttle, Yaw, AUX1..AUX4]
        self.rc = [RC_MID, RC_MID, START_THROTTLE, RC_MID,
                   1000, 1000, 1000, 1000]

    # ------------------------------------------------------------------ #
    #  MSP ALTYAPI
    # ------------------------------------------------------------------ #

    def _send_msp(self, cmd, payload: bytes = b''):
        size = len(payload)
        checksum = size ^ cmd
        for b in payload:
            checksum ^= b
        packet = struct.pack('<3sBB', b'$M<', size, cmd) + payload + struct.pack('<B', checksum)
        self.ser.write(packet)

    def _read_msp(self):
        header = self.ser.read(3)
        if header != b'$M>':
            return None
        size_raw = self.ser.read(1)
        if not size_raw:
            return None
        size = struct.unpack('<B', size_raw)[0]
        self.ser.read(1)  # cmd byte
        data = self.ser.read(size)
        self.ser.read(1)  # checksum
        if len(data) < size:
            return None
        return data

    # ------------------------------------------------------------------ #
    #  IMU — Z EKSENİ İVMESİ
    # ------------------------------------------------------------------ #

    def get_z_accel(self):
        """
        MSP_RAW_IMU (102) ile ham ivmeölçer verisini okur.
        Z ivmesini (dikey eksen) döndürür.

        Değer yorumu:
          0'dan büyük → drone yukarı ivmeleniyor (çok gaz)
          0'dan küçük → drone aşağı düşüyor (az gaz)
          Yerçekimi etkisi: zeminde yaklaşık +512 (1g) okur,
          havada dengede ise yaklaşık 0 olmayı hedefleriz.

        NOT: Flight controller'ın IMU yönüne göre işaret değişebilir.
        İlk testte logu izle, gerekirse aşağıdaki satırı çevir:
            return -z   →   return z
        """
        self._send_msp(102)  # MSP_RAW_IMU
        data = self._read_msp()
        if data is None or len(data) < 6:
            return None
        # ax=data[0:2], ay=data[2:4], az=data[4:6]
        z_raw = struct.unpack('<h', data[4:6])[0]
        # Ham değeri ölçekle (Betaflight'ta 512 LSB ≈ 1g)
        z_g = z_raw / 512.0
        # Yerçekimini çıkar: hoverde net ivme 0 olmalı
        # Betaflight Z ekseni aşağı pozitif olduğu için 1g çıkarıyoruz
        z_net = z_g - 1.0
        #z_net = -(z_g - 1.0)  # Eğer drone'un IMU'su Z eksenini ters okuyor ve pozitif yukarı geliyorsa, bu satırı kullanıcaz !!!!
        return z_net

    # ------------------------------------------------------------------ #
    #  ARM BEKLEME
    # ------------------------------------------------------------------ #

    def wait_for_arm(self):
        logging.info("BEKLENİYOR: Kumandadan ARM switch'ini çek!")
        while True:
            self._send_msp(150)  # MSP_STATUS_EX
            data = self._read_msp()
            if data is not None and len(data) >= 9:
                armed = bool(struct.unpack('<I', bytes(data[5:9]))[0] & 1)
                if armed:
                    logging.info("✅ ARM algılandı! Hover PID başlıyor.")
                    return True
            time.sleep(0.2)

    # ------------------------------------------------------------------ #
    #  RC GÖNDERİMİ
    # ------------------------------------------------------------------ #

    def send_rc(self):
        payload = b''.join(struct.pack('<H', ch) for ch in self.rc)
        self._send_msp(200, payload)

    # ------------------------------------------------------------------ #
    #  YUMUŞAK İNİŞ
    # ------------------------------------------------------------------ #

    def soft_land(self, from_throttle):
        logging.info("İniş başlıyor...")
        throttle = from_throttle
        step_time = 1.0 / LOOP_HZ
        while throttle > MIN_THROTTLE:
            throttle = max(MIN_THROTTLE, throttle - 8)
            self.rc[2] = throttle
            self.send_rc()
            time.sleep(step_time)
        logging.info("Motorlar durdu.")

    # ------------------------------------------------------------------ #
    #  ANA HOVER DÖNGÜSÜ
    # ------------------------------------------------------------------ #

    def run(self):
        if not self.wait_for_arm():
            return

        self.pid.reset()

        current_throttle = START_THROTTLE
        start_time   = time.time()
        last_time    = start_time
        imu_fail_cnt = 0
        MAX_FAIL     = 10
        loop_period  = 1.0 / LOOP_HZ

        logging.info(f"Hover başladı | Süre: {FLIGHT_DURATION}s | "
                     f"P={P_GAIN} I={I_GAIN} D={D_GAIN}")

        while (time.time() - start_time) < FLIGHT_DURATION:
            loop_start = time.time()
            dt = max(loop_start - last_time, 1e-4)

            # --- Z EKSENİ OKU ---
            z_accel = self.get_z_accel()

            if z_accel is None:
                imu_fail_cnt += 1
                logging.warning(f"IMU okunamadı ({imu_fail_cnt}/{MAX_FAIL})")
                if imu_fail_cnt >= MAX_FAIL:
                    logging.error("Çok fazla IMU hatası — acil iniş!")
                    break
                time.sleep(loop_period)
                continue
            else:
                imu_fail_cnt = 0

            # --- PID: hedef Z ivmesi = 0 (ne yukarı ne aşağı) ---
            throttle_correction = self.pid.calculate(
                target=0.0,
                current=z_accel,
                dt=dt
            )

            # Gaz = mevcut gaz + PID düzeltmesi
            current_throttle += throttle_correction
            current_throttle  = max(MIN_THROTTLE,
                                    min(MAX_THROTTLE, current_throttle))

            self.rc[2] = int(current_throttle)
            self.send_rc()

            elapsed = round(loop_start - start_time, 2)
            logging.info(
                f"t={elapsed}s | "
                f"Z_net={z_accel:+.3f}g | "
                f"Düzeltme={throttle_correction:+.1f} | "
                f"Thr={int(current_throttle)}"
            )

            last_time = loop_start
            sleep_time = loop_period - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

        # --- BİTİŞ ---
        self.soft_land(int(current_throttle))
        logging.info("=== TEST BİTTİ ===")
        logging.info("Log dosyasına bak, Z değerlerine göre P/I/D'yi ayarla.")


if __name__ == "__main__":
    drone = DroneHoverControl(SERIAL_PORT)
    drone.run()