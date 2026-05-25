import serial
import struct
import time

# =============================================================================
# PORT AYARLARI
# =============================================================================
SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE   = 115200

# =============================================================================
# MSP KOMUTLARI
# =============================================================================
MSP_RC         = 105
MSP_SET_RAW_RC = 200
MSP_SET_MOTOR  = 214   # Direkt motor kontrolü (iNav mixer bypass)
MSP_ATTITUDE   = 108   # Roll / Pitch / Yaw okuma

# =============================================================================
# KANAL TANIMLARI
# =============================================================================
CH_ROLL     = 0
CH_PITCH    = 1
CH_YAW      = 2
CH_THROTTLE = 3
CH_ARM      = 5   # SWA
CH_OVERRIDE = 6   # SWA yanındaki tuş

# =============================================================================
# MOTOR INDEX TANIMI
#
# Senin drone'un (pervane düzeninden):
#
#       M4 (Sol-Ön)    M2 (Sağ-Ön)
#             \          /
#              \        /
#       M3 (Sol-Arka)  M1 (Sağ-Arka)
#
# iNav MSP_SET_MOTOR index karşılıkları:
#   Sol-Ön  (M4) → index 0
#   Sağ-Ön  (M2) → index 1
#   Sol-Arka(M3) → index 2
#   Sağ-Arka(M1) → index 3
#
# Configurator → Motors sekmesinde tek tek test ederek doğrula.
# Yanlışsa sadece aşağıdaki 4 sabiti değiştir, başka bir şeye dokunma.
# =============================================================================
IDX_SOL_ON   = 0   # M4
IDX_SAG_ON   = 1   # M2
IDX_SOL_ARKA = 2   # M3
IDX_SAG_ARKA = 3   # M1

# =============================================================================
# MOTOR TRIM DEĞERLERİ (PWM birimi, 1000-2000 aralığında)
#
# Drone sola yatıyorsa sol taraftaki motorları artır:
#   IDX_SOL_ON ve IDX_SOL_ARKA → pozitif değer ver
#
# Sağa yatıyorsa:
#   IDX_SAG_ON ve IDX_SAG_ARKA → pozitif değer ver
#
# Öne yatıyorsa:
#   IDX_SOL_ON ve IDX_SAG_ON → pozitif değer ver
#
# Başlangıç: hepsini 0 bırak, test uç, gözlemle, sonra ayarla.
# Adım büyüklüğü: 3-5 PWM birimi. 20'yi geçme.
# =============================================================================
MOTOR_TRIM = {
    IDX_SOL_ON:   0,   # M4 Sol-Ön
    IDX_SAG_ON:   0,   # M2 Sağ-Ön
    IDX_SOL_ARKA: 0,   # M3 Sol-Arka
    IDX_SAG_ARKA: 0,   # M1 Sağ-Arka
}

# =============================================================================
# UÇUŞ PARAMETRELERİ
# =============================================================================
HOVER_THROTTLE = 1260   # %26 ≈ 1260 PWM

KALKIS_ADIM_SAYISI   = 60     # 60 adım × 0.05sn = 3 saniyelik rampa
KALKIS_ADIM_GECIKMESI = 0.05

HOVER_SURESI = 3.0            # saniye

INIS_ADIM_SAYISI    = 60
INIS_ADIM_GECIKMESI = 0.05

PWM_MIN = 1000
PWM_MAX = 1400   # Hellion 10 çok güçlü — test için mutlak üst sınır

# =============================================================================
# PID KATSAYILARI
# Küçük başla. Titreme varsa Kp'yi yarıya indir.
# =============================================================================
PID_KP = 0.8
PID_KI = 0.05
PID_KD = 0.3
INTEGRAL_LIMIT = 5.0

# =============================================================================
# ACİL DURDURMA İSTİSNASI
# =============================================================================
class AcilDurdurma(Exception):
    pass

# =============================================================================
# MSP FONKSİYONLARI
# =============================================================================

def send_msp(ser, cmd, data=b''):
    if isinstance(data, list):
        data = bytes(data)
    size     = len(data)
    checksum = size ^ cmd
    for b in data:
        checksum ^= b
    packet = b'$M<' + bytes([size, cmd]) + data + bytes([checksum])
    ser.write(packet)


def recv_msp(ser, expected_cmd, timeout=0.1):
    deadline = time.time() + timeout
    state = 0; size = 0; cmd = 0; data = b''

    while time.time() < deadline:
        if ser.in_waiting > 0:
            c = ser.read(1)
            if   state == 0 and c == b'$': state = 1
            elif state == 1 and c == b'M': state = 2
            elif state == 2 and c == b'>': state = 3
            elif state == 3: size = ord(c); state = 4
            elif state == 4: cmd  = ord(c); state = 5
            elif state == 5:
                data += c
                if len(data) == size: state = 6
            elif state == 6:
                if cmd == expected_cmd:
                    return data
                return None
    return None


def get_rc_channels(ser):
    send_msp(ser, MSP_RC, [])
    data = recv_msp(ser, MSP_RC)
    if data and len(data) >= 32:
        return list(struct.unpack('<16H', data[:32]))
    return None


def get_attitude(ser):
    """Roll, Pitch, Yaw oku. iNav değerleri 10x gelir."""
    send_msp(ser, MSP_ATTITUDE, [])
    data = recv_msp(ser, MSP_ATTITUDE)
    if data and len(data) >= 6:
        roll  = struct.unpack_from('<h', data, 0)[0] / 10.0
        pitch = struct.unpack_from('<h', data, 2)[0] / 10.0
        yaw   = struct.unpack_from('<h', data, 4)[0]
        return roll, pitch, yaw
    return 0.0, 0.0, 0


def set_motors(ser, pwm_listesi):
    """
    4 motora doğrudan PWM gönder.
    pwm_listesi = [idx0_pwm, idx1_pwm, idx2_pwm, idx3_pwm]

    Senin drone'un için:
      pwm_listesi[0] → M4 Sol-Ön
      pwm_listesi[1] → M2 Sağ-Ön
      pwm_listesi[2] → M3 Sol-Arka
      pwm_listesi[3] → M1 Sağ-Arka
    """
    all_motors = [1000] * 8
    for i, val in enumerate(pwm_listesi[:4]):
        all_motors[i] = int(max(PWM_MIN, min(PWM_MAX, val)))
    data = struct.pack('<8H', *all_motors)
    send_msp(ser, MSP_SET_MOTOR, list(data))


def motor_durdur(ser):
    set_motors(ser, [1000, 1000, 1000, 1000])
    print("[STOP] Tüm motorlar durduruldu.")

# =============================================================================
# GÜVENLİK KONTROLÜ
# Her döngüde çağrılır. ARM veya OVERRIDE kapandıysa AcilDurdurma fırlatır.
# =============================================================================

def guvenlik_kontrol(ser):
    """
    Kumanda kanallarını oku.
    ARM veya OVERRIDE tuşu kapandıysa AcilDurdurma exception fırlat.
    Okuma başarısız olursa sessizce geç (bağlantı geçici kopukluğu).
    """
    channels = get_rc_channels(ser)
    if channels is None:
        return  # okuma başarısız, bu döngüyü atla

    arm_acik      = channels[CH_ARM]      > 1500
    override_acik = channels[CH_OVERRIDE] > 1500

    if not (arm_acik and override_acik):
        raise AcilDurdurma("Kumanda tuşu kapandı — acil durdurma!")

# =============================================================================
# PID SINIFI
# =============================================================================

class PIDController:
    def __init__(self, kp, ki, kd, integral_limit=INTEGRAL_LIMIT):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.integral_limit = integral_limit
        self.integral   = 0.0
        self.prev_error = 0.0
        self.last_time  = time.time()

    def reset(self):
        self.integral   = 0.0
        self.prev_error = 0.0
        self.last_time  = time.time()

    def compute(self, setpoint, measured):
        now = time.time()
        dt  = max(now - self.last_time, 0.001)
        self.last_time = now

        error = setpoint - measured
        self.integral = max(-self.integral_limit,
                        min( self.integral_limit, self.integral + error * dt))
        derivative    = (error - self.prev_error) / dt
        self.prev_error = error

        return self.kp * error + self.ki * self.integral + self.kd * derivative

# =============================================================================
# MOTOR MIXER
#
# Senin X konfigürasyonun (pervane dönüş yönleriyle):
#
#   M4 Sol-Ön  (idx 0): ↻ CW   M2 Sağ-Ön  (idx 1): ↺ CCW
#   M3 Sol-Arka(idx 2): ↺ CCW  M1 Sağ-Arka(idx 3): ↻ CW
#
# Roll sağa   (+corr) → Sol motorlar (idx 0,2) hızlanır, Sağ (idx 1,3) yavaşlar
# Pitch ileri (+corr) → Arka motorlar (idx 2,3) hızlanır, Ön (idx 0,1) yavaşlar
# =============================================================================

def hesapla_motor_pwm(base_throttle, roll_pwm, pitch_pwm):
    """
    Döner: [idx0_pwm, idx1_pwm, idx2_pwm, idx3_pwm]
    yani:  [M4_Sol-Ön, M2_Sağ-Ön, M3_Sol-Arka, M1_Sağ-Arka]
    """
    t = MOTOR_TRIM

    idx0 = base_throttle + t[IDX_SOL_ON]   - roll_pwm - pitch_pwm  # M4 Sol-Ön
    idx1 = base_throttle + t[IDX_SAG_ON]   + roll_pwm - pitch_pwm  # M2 Sağ-Ön
    idx2 = base_throttle + t[IDX_SOL_ARKA] - roll_pwm + pitch_pwm  # M3 Sol-Arka
    idx3 = base_throttle + t[IDX_SAG_ARKA] + roll_pwm + pitch_pwm  # M1 Sağ-Arka

    return [int(idx0), int(idx1), int(idx2), int(idx3)]

# =============================================================================
# UÇUŞ SEKANS FONKSİYONLARI
# Her adımda guvenlik_kontrol() çağrılır.
# AcilDurdurma fırlarsa üst katman yakalar ve motorları durdurur.
# =============================================================================

def kalkis(ser, pid_roll, pid_pitch):
    print("\n[KALKIŞ] Başlıyor...")
    pid_roll.reset()
    pid_pitch.reset()

    for adim in range(KALKIS_ADIM_SAYISI + 1):

        guvenlik_kontrol(ser)   # ← tuş kapandıysa burada durur

        throttle = int(1000 + (HOVER_THROTTLE - 1000) * (adim / KALKIS_ADIM_SAYISI))
        roll, pitch, _ = get_attitude(ser)
        roll_pwm  = int(pid_roll.compute(0.0, roll)   * 10)
        pitch_pwm = int(pid_pitch.compute(0.0, pitch) * 10)

        motors = hesapla_motor_pwm(throttle, roll_pwm, pitch_pwm)
        set_motors(ser, motors)

        if adim % 10 == 0:
            print(
                f"  [{adim:02d}/{KALKIS_ADIM_SAYISI}] Throttle:{throttle} | "
                f"Roll:{roll:+.1f}° Pitch:{pitch:+.1f}° | "
                f"M4:{motors[IDX_SOL_ON]} M2:{motors[IDX_SAG_ON]} "
                f"M3:{motors[IDX_SOL_ARKA]} M1:{motors[IDX_SAG_ARKA]}"
            )
        time.sleep(KALKIS_ADIM_GECIKMESI)

    print("[KALKIŞ] Tamamlandı.")


def hover(ser, pid_roll, pid_pitch):
    print(f"\n[HOVER] {HOVER_SURESI}sn hover başlıyor...")
    baslangic = time.time()

    while time.time() - baslangic < HOVER_SURESI:

        guvenlik_kontrol(ser)   # ← tuş kapandıysa burada durur

        roll, pitch, _ = get_attitude(ser)
        roll_pwm  = int(pid_roll.compute(0.0, roll)   * 10)
        pitch_pwm = int(pid_pitch.compute(0.0, pitch) * 10)

        motors = hesapla_motor_pwm(HOVER_THROTTLE, roll_pwm, pitch_pwm)
        set_motors(ser, motors)

        kalan = HOVER_SURESI - (time.time() - baslangic)
        print(
            f"\r  Kalan:{kalan:.1f}s | Roll:{roll:+.1f}° Pitch:{pitch:+.1f}° | "
            f"M4:{motors[IDX_SOL_ON]} M2:{motors[IDX_SAG_ON]} "
            f"M3:{motors[IDX_SOL_ARKA]} M1:{motors[IDX_SAG_ARKA]}   ",
            end="", flush=True
        )
        time.sleep(0.02)  # 50 Hz

    print("\n[HOVER] Tamamlandı.")


def inis(ser, pid_roll, pid_pitch):
    print("\n[İNİŞ] Başlıyor...")

    for adim in range(INIS_ADIM_SAYISI + 1):

        guvenlik_kontrol(ser)   # ← tuş kapandıysa burada durur

        throttle = int(HOVER_THROTTLE - (HOVER_THROTTLE - 1000) * (adim / INIS_ADIM_SAYISI))
        roll, pitch, _ = get_attitude(ser)
        roll_pwm  = int(pid_roll.compute(0.0, roll)   * 10)
        pitch_pwm = int(pid_pitch.compute(0.0, pitch) * 10)

        motors = hesapla_motor_pwm(throttle, roll_pwm, pitch_pwm)
        set_motors(ser, motors)

        if adim % 10 == 0:
            print(f"  [{adim:02d}/{INIS_ADIM_SAYISI}] Throttle:{throttle} | "
                  f"M4:{motors[IDX_SOL_ON]} M2:{motors[IDX_SAG_ON]} "
                  f"M3:{motors[IDX_SOL_ARKA]} M1:{motors[IDX_SAG_ARKA]}")
        time.sleep(INIS_ADIM_GECIKMESI)

    motor_durdur(ser)
    print("[İNİŞ] Tamamlandı.")

# =============================================================================
# ANA DÖNGÜ
# =============================================================================

def main():
    pid_roll  = PIDController(PID_KP, PID_KI, PID_KD)
    pid_pitch = PIDController(PID_KP, PID_KI, PID_KD)

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Bağlantı kuruldu: {SERIAL_PORT}")
        time.sleep(2)

        print("\n=== OTONOM UÇUŞ SİSTEMİ ===")
        print(f"Hover throttle : {HOVER_THROTTLE} PWM")
        print(f"Hover süresi   : {HOVER_SURESI} sn")
        print(f"Motor trim     : {MOTOR_TRIM}")
        print()
        print("Motor eşleşmesi:")
        print(f"  iNav index 0 → M4 Sol-Ön")
        print(f"  iNav index 1 → M2 Sağ-Ön")
        print(f"  iNav index 2 → M3 Sol-Arka")
        print(f"  iNav index 3 → M1 Sağ-Arka")
        print()
        print("GÜVENLİK: ARM + OVERRIDE tuşlarından birini kapat → anında durur.")
        print("DRONU MASAYA SIKICA BAGLAYIN!")
        print("\nARM (SWA) + OVERRIDE tuşlarını aç → sekans başlar.\n")

        sekans_calisiyor = False

        while True:
            channels = get_rc_channels(ser)

            if channels:
                arm_acik      = channels[CH_ARM]      > 1500
                override_acik = channels[CH_OVERRIDE] > 1500

                if arm_acik and override_acik and not sekans_calisiyor:
                    sekans_calisiyor = True
                    print("[BAŞLAT] Otonom sekans tetiklendi!")

                    try:
                        kalkis(ser, pid_roll, pid_pitch)
                        hover(ser, pid_roll, pid_pitch)
                        inis(ser, pid_roll, pid_pitch)
                        print("[BİTTİ] Sekans tamamlandı. Kontrol kumandada.\n")

                    except AcilDurdurma as e:
                        print(f"\n[ACİL DURDURMA] {e}")
                        motor_durdur(ser)

                    except Exception as e:
                        print(f"\n[HATA] {e}")
                        motor_durdur(ser)

                    finally:
                        sekans_calisiyor = False

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[CTRL+C] Program durduruldu.")
    except Exception as e:
        print(f"[HATA] {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            motor_durdur(ser)
            ser.close()
            print("Seri port kapatıldı.")


if __name__ == "__main__":
    main()