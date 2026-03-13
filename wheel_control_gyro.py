#!/usr/bin/env python3
"""
Wheel control with MPU6050 gyro for:
  - Exact 90° in-place turns
  - Gyro-corrected straight driving
  - Gyro-corrected reverse driving

Wiring (I2C):
  MPU6050 VCC -> 3.3V (pin 1)
  MPU6050 GND -> GND
  MPU6050 SDA -> GPIO2 (pin 3)
  MPU6050 SCL -> GPIO3 (pin 5)
  MPU6050 AD0 -> GND  (I2C address = 0x68)

Install deps:
  pip install smbus2
"""

import RPi.GPIO as GPIO
import time
import os
import json
import smbus2
import math

# ============================================================
# PIN MAP (BCM) — unchanged from original
# ============================================================
RF_IN1, RF_IN2 = 17, 27
LF_IN1, LF_IN2 = 22, 23
LB_IN1, LB_IN2 = 21, 20
RB_IN1, RB_IN2 = 16, 6

ALL_DIR_PINS = [RF_IN1, RF_IN2, LF_IN1, LF_IN2,
                LB_IN1, LB_IN2, RB_IN1, RB_IN2]

EN_RF = 13
EN_LF = 24
EN_LB = 12
EN_RB = 19

p_rf = p_lf = p_lb = p_rb = None

INVERT_DRIVE = True

WHEEL_INVERT = {
    "RF": False,
    "LF": False,
    "LB": True,
    "RB": False
}

PWM_HZ     = 1000
MAX_DUTY   = 100
BASE_DRIVE_DUTY = 100
BASE_TURN_DUTY  = 100

CALIB_FILE = "/home/ikkiocean/wheel_calib.json"

left_scale       = 1.00
right_scale      = 1.00
turn_left_scale  = 1.00
turn_right_scale = 1.00


# ============================================================
# MPU6050 driver (minimal, no external lib required beyond smbus2)
# ============================================================
MPU_ADDR  = 0x68          # AD0 low; use 0x69 if AD0 high
PWR_MGMT  = 0x6B
GYRO_ZOUT_H = 0x47        # Gyro Z high byte (yaw axis)
ACCEL_XOUT_H = 0x3B       # Accel X high byte

# Gyro full-scale ±250 °/s -> sensitivity 131 LSB/(°/s)
GYRO_SENS = 131.0

# How long (seconds) to average during bias calibration
GYRO_CAL_SECS = 2.0

_bus       = None
_gyro_z_bias = 0.0        # deg/s offset measured at rest


def _read_word_2c(reg):
    """Read signed 16-bit big-endian from MPU register."""
    high = _bus.read_byte_data(MPU_ADDR, reg)
    low  = _bus.read_byte_data(MPU_ADDR, reg + 1)
    val  = (high << 8) | low
    return val - 65536 if val >= 32768 else val


def mpu_init():
    """Power on MPU6050 and calibrate gyro bias."""
    global _bus, _gyro_z_bias

    _bus = smbus2.SMBus(1)          # /dev/i2c-1
    # Wake the chip (clear SLEEP bit)
    _bus.write_byte_data(MPU_ADDR, PWR_MGMT, 0x00)
    time.sleep(0.1)

    # Set gyro range ±250 °/s (register 0x1B, value 0x00)
    _bus.write_byte_data(MPU_ADDR, 0x1B, 0x00)
    # Set accel range ±2g  (register 0x1C, value 0x00)
    _bus.write_byte_data(MPU_ADDR, 0x1C, 0x00)

    print("MPU6050 online — calibrating gyro bias (keep robot still)…")
    _calibrate_gyro()
    print(f"  Gyro Z bias = {_gyro_z_bias:.4f} °/s")


def _calibrate_gyro():
    global _gyro_z_bias
    samples = []
    t_end = time.time() + GYRO_CAL_SECS
    while time.time() < t_end:
        raw = _read_word_2c(GYRO_ZOUT_H)
        samples.append(raw / GYRO_SENS)
        time.sleep(0.005)
    _gyro_z_bias = sum(samples) / len(samples)


def read_gyro_z() -> float:
    """Return bias-corrected yaw rate in °/s."""
    raw = _read_word_2c(GYRO_ZOUT_H)
    return (raw / GYRO_SENS) - _gyro_z_bias


def integrate_yaw(duration_s: float, interval_s: float = 0.005) -> float:
    """Integrate gyro Z for up to duration_s seconds; return total °."""
    angle = 0.0
    t_end = time.time() + duration_s
    prev  = time.time()
    while time.time() < t_end:
        now = time.time()
        dt  = now - prev
        prev = now
        angle += read_gyro_z() * dt
        time.sleep(interval_s)
    return angle


# ============================================================
# Calibration helpers (unchanged)
# ============================================================
def clamp_dc(x):
    x = float(x)
    return int(round(max(0, min(100, x))))


def load_calib():
    global left_scale, right_scale, turn_left_scale, turn_right_scale
    try:
        if os.path.exists(CALIB_FILE):
            with open(CALIB_FILE) as f:
                d = json.load(f)
            left_scale       = float(d.get("left_scale",       1.0))
            right_scale      = float(d.get("right_scale",      1.0))
            turn_left_scale  = float(d.get("turn_left_scale",  1.0))
            turn_right_scale = float(d.get("turn_right_scale", 1.0))
            print(f"✓ LOADED: left={left_scale:.3f} right={right_scale:.3f} "
                  f"turnL={turn_left_scale:.3f} turnR={turn_right_scale:.3f}")
            return True
    except Exception as e:
        print(f"⚠ Load calib failed: {e}")
    print("⚠ No saved calibration, using defaults (1.0)")
    return False


def save_calib():
    d = {
        "left_scale":       left_scale,
        "right_scale":      right_scale,
        "turn_left_scale":  turn_left_scale,
        "turn_right_scale": turn_right_scale,
    }
    try:
        with open(CALIB_FILE, "w") as f:
            json.dump(d, f, indent=2)
        print("✓ Calibration saved")
    except Exception as e:
        print(f"✗ Save calib failed: {e}")


# ============================================================
# GPIO / PWM setup
# ============================================================
def setup():
    global p_rf, p_lf, p_lb, p_rb

    GPIO.setmode(GPIO.BCM)
    for p in ALL_DIR_PINS:
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
    for en in [EN_RF, EN_LF, EN_LB, EN_RB]:
        GPIO.setup(en, GPIO.OUT)

    p_rf = GPIO.PWM(EN_RF, PWM_HZ)
    p_lf = GPIO.PWM(EN_LF, PWM_HZ)
    p_lb = GPIO.PWM(EN_LB, PWM_HZ)
    p_rb = GPIO.PWM(EN_RB, PWM_HZ)

    for p in [p_rf, p_lf, p_lb, p_rb]:
        p.start(0)


def stop_all():
    for p in ALL_DIR_PINS:
        GPIO.output(p, GPIO.LOW)
    for pwm in [p_rf, p_lf, p_lb, p_rb]:
        try:
            if pwm:
                pwm.ChangeDutyCycle(0)
        except Exception:
            pass


def set_pair(in1, in2, forward=True):
    if forward:
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    else:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)


def apply_pwm(base_duty, extra_left=1.0, extra_right=1.0):
    base_duty = clamp_dc(base_duty)
    lf = clamp_dc(base_duty * left_scale * extra_left)
    lb = clamp_dc(base_duty * left_scale * extra_left)
    rf = clamp_dc(base_duty * right_scale * extra_right)
    rb = clamp_dc(base_duty * right_scale * extra_right)
    p_lf.ChangeDutyCycle(lf)
    p_lb.ChangeDutyCycle(lb)
    p_rf.ChangeDutyCycle(rf)
    p_rb.ChangeDutyCycle(rb)


# ============================================================
# Low-level direction helpers
# ============================================================
def _set_forward_dirs():
    fw = not INVERT_DRIVE
    set_pair(RF_IN1, RF_IN2, forward=fw ^ WHEEL_INVERT["RF"])
    set_pair(LF_IN1, LF_IN2, forward=fw ^ WHEEL_INVERT["LF"])
    set_pair(LB_IN1, LB_IN2, forward=fw ^ WHEEL_INVERT["LB"])
    set_pair(RB_IN1, RB_IN2, forward=fw ^ WHEEL_INVERT["RB"])


def _set_backward_dirs():
    bw = INVERT_DRIVE
    set_pair(RF_IN1, RF_IN2, forward=bw ^ WHEEL_INVERT["RF"])
    set_pair(LF_IN1, LF_IN2, forward=bw ^ WHEEL_INVERT["LF"])
    set_pair(LB_IN1, LB_IN2, forward=bw ^ WHEEL_INVERT["LB"])
    set_pair(RB_IN1, RB_IN2, forward=bw ^ WHEEL_INVERT["RB"])


def _set_turn_left_dirs():
    left_fw  = INVERT_DRIVE
    right_fw = not INVERT_DRIVE
    set_pair(LF_IN1, LF_IN2, forward=left_fw  ^ WHEEL_INVERT["LF"])
    set_pair(LB_IN1, LB_IN2, forward=left_fw  ^ WHEEL_INVERT["LB"])
    set_pair(RF_IN1, RF_IN2, forward=right_fw ^ WHEEL_INVERT["RF"])
    set_pair(RB_IN1, RB_IN2, forward=right_fw ^ WHEEL_INVERT["RB"])


def _set_turn_right_dirs():
    left_fw  = not INVERT_DRIVE
    right_fw = INVERT_DRIVE
    set_pair(LF_IN1, LF_IN2, forward=left_fw  ^ WHEEL_INVERT["LF"])
    set_pair(LB_IN1, LB_IN2, forward=left_fw  ^ WHEEL_INVERT["LB"])
    set_pair(RF_IN1, RF_IN2, forward=right_fw ^ WHEEL_INVERT["RF"])
    set_pair(RB_IN1, RB_IN2, forward=right_fw ^ WHEEL_INVERT["RB"])


# ============================================================
# Simple (timed) motions — kept from original
# ============================================================
def forward():
    _set_forward_dirs()
    apply_pwm(BASE_DRIVE_DUTY)


def backward():
    _set_backward_dirs()
    apply_pwm(BASE_DRIVE_DUTY)


def turn_left_inplace():
    _set_turn_left_dirs()
    apply_pwm(BASE_TURN_DUTY,
              extra_left=turn_left_scale, extra_right=turn_left_scale)


def turn_right_inplace():
    _set_turn_right_dirs()
    apply_pwm(BASE_TURN_DUTY,
              extra_left=turn_right_scale, extra_right=turn_right_scale)


# ============================================================
# GYRO-ASSISTED MOTIONS
# ============================================================

# ── Straight driving with yaw correction ────────────────────
STRAIGHT_KP = 3.0   # proportional gain for yaw correction
                     # increase if drift is large; decrease if oscillating

def drive_straight_gyro(duration_s: float, duty: float = BASE_DRIVE_DUTY):
    """
    Drive forward for duration_s seconds while using gyro yaw rate
    to keep the robot going straight.

    The corrector adjusts left/right PWM in real-time:
      - positive yaw_rate means drifting left  → boost right side
      - negative yaw_rate means drifting right → boost left side
    """
    print(f"▶ Straight forward {duration_s:.1f}s (gyro-corrected)…")
    _set_forward_dirs()

    t_end   = time.time() + duration_s
    prev_t  = time.time()
    yaw_acc = 0.0          # accumulated yaw error in degrees

    while time.time() < t_end:
        now = time.time()
        dt  = now - prev_t
        prev_t = now

        yaw_rate = read_gyro_z()          # °/s
        yaw_acc += yaw_rate * dt          # integrate → total heading error

        # Correction: scale total error by Kp to get PWM offset
        correction = STRAIGHT_KP * yaw_acc

        # Apply asymmetric duty to steer back
        base = clamp_dc(duty)
        lf = clamp_dc(base * left_scale  - correction)
        lb = clamp_dc(base * left_scale  - correction)
        rf = clamp_dc(base * right_scale + correction)
        rb = clamp_dc(base * right_scale + correction)

        p_lf.ChangeDutyCycle(lf)
        p_lb.ChangeDutyCycle(lb)
        p_rf.ChangeDutyCycle(rf)
        p_rb.ChangeDutyCycle(rb)

        time.sleep(0.005)

    stop_all()
    print(f"   Done. Total yaw drift ≈ {yaw_acc:.1f}°")


def drive_backward_gyro(duration_s: float, duty: float = BASE_DRIVE_DUTY):
    """
    Drive backward for duration_s seconds, gyro-corrected.
    Same correction logic; note sign flip because backward swaps
    which side needs boosting.
    """
    print(f"◀ Straight backward {duration_s:.1f}s (gyro-corrected)…")
    _set_backward_dirs()

    t_end   = time.time() + duration_s
    prev_t  = time.time()
    yaw_acc = 0.0

    while time.time() < t_end:
        now = time.time()
        dt  = now - prev_t
        prev_t = now

        yaw_rate = read_gyro_z()
        yaw_acc += yaw_rate * dt

        correction = STRAIGHT_KP * yaw_acc

        base = clamp_dc(duty)
        # For backward motion the correction direction is inverted
        lf = clamp_dc(base * left_scale  + correction)
        lb = clamp_dc(base * left_scale  + correction)
        rf = clamp_dc(base * right_scale - correction)
        rb = clamp_dc(base * right_scale - correction)

        p_lf.ChangeDutyCycle(lf)
        p_lb.ChangeDutyCycle(lb)
        p_rf.ChangeDutyCycle(rf)
        p_rb.ChangeDutyCycle(rb)

        time.sleep(0.005)

    stop_all()
    print(f"   Done. Total yaw drift ≈ {yaw_acc:.1f}°")


# ── Exact-angle turns ────────────────────────────────────────
# Braking margin: robot coasts a few degrees after stop_all().
# Measure your robot's coast angle and set TURN_COAST_DEG.
TURN_COAST_DEG = 3.0    # degrees the robot overshoots after motors cut
                         # increase if it over-rotates; decrease if under


def turn_gyro(target_deg: float, direction: str = "left",
              duty: float = BASE_TURN_DUTY):
    """
    Rotate target_deg degrees in-place using gyro integration.

    direction : "left" or "right"
    Stops motors TURN_COAST_DEG early to account for mechanical coast.

    Convention (MPU Z-axis):
      CCW (left turn)  → positive gyro Z reading
      CW  (right turn) → negative gyro Z reading
    If your robot is opposite, swap the sign checks below.
    """
    effective_target = max(0.0, target_deg - TURN_COAST_DEG)
    print(f"↺ Turn {direction} {target_deg:.1f}° "
          f"(stopping at {effective_target:.1f}°, coast={TURN_COAST_DEG}°)")

    if direction == "left":
        _set_turn_left_dirs()
        apply_pwm(duty, extra_left=turn_left_scale, extra_right=turn_left_scale)
    else:
        _set_turn_right_dirs()
        apply_pwm(duty, extra_left=turn_right_scale, extra_right=turn_right_scale)

    angle_turned = 0.0
    prev_t = time.time()
    POLL   = 0.005         # seconds between gyro reads

    while abs(angle_turned) < effective_target:
        now = time.time()
        dt  = now - prev_t
        prev_t = now

        rate = read_gyro_z()   # °/s; sign convention: left → positive
        if direction == "right":
            rate = -rate       # make right turn accumulate positively too

        angle_turned += rate * dt
        time.sleep(POLL)

    stop_all()
    print(f"   Motors cut at {angle_turned:.1f}° (target={target_deg}°)")


def turn_left_90():
    turn_gyro(90.0, direction="left")


def turn_right_90():
    turn_gyro(90.0, direction="right")


# ============================================================
# Tests
# ============================================================
def square_test_gyro():
    """
    Square: forward 2s + left 90° × 4 — fully gyro-assisted.
    """
    print("\n🔲 GYRO SQUARE TEST")
    for i in range(4):
        print(f"\nLeg {i+1}/4")
        drive_straight_gyro(2.0)
        time.sleep(0.3)
        turn_left_90()
        time.sleep(0.3)
    print("✅ Gyro square test done")


def straight_back_test():
    """Drive forward 2 s, pause, drive backward 2 s."""
    print("\n↕ STRAIGHT + BACK TEST")
    drive_straight_gyro(2.0)
    time.sleep(0.5)
    drive_backward_gyro(2.0)
    print("✅ Done")


# ============================================================
# Calibration helpers (unchanged from original)
# ============================================================
def calibrate_straight():
    global left_scale, right_scale
    print("\n" + "="*60)
    print("STRAIGHT CALIBRATION")
    while True:
        print(f"\nleft_scale={left_scale:.3f}  right_scale={right_scale:.3f}")
        input("Press Enter to run forward test (3s)…")
        stop_all()
        drive_straight_gyro(3.0)
        ans = input("Veer? (l/r/s=straight, q=save+quit): ").strip().lower()
        if ans in ("q", "s"):
            save_calib(); return
        step = float(input("Step (suggest 0.02): ").strip() or "0.02")
        if ans == "l":
            left_scale = max(0.50, left_scale - step)
        elif ans == "r":
            right_scale = max(0.50, right_scale - step)


def calibrate_turns():
    global turn_left_scale, turn_right_scale
    print("\n" + "="*60)
    print("TURN CALIBRATION")
    while True:
        print(f"\nturnL={turn_left_scale:.3f}  turnR={turn_right_scale:.3f}")
        side = input("Test (l/r/q): ").strip().lower()
        if side == "q":
            save_calib(); return
        if side not in ("l", "r"):
            continue
        input("Press Enter to turn 90°…")
        stop_all()
        if side == "l":
            turn_left_90()
        else:
            turn_right_90()
        res = input("Result? (w=under / o=over / g=good / q): ").strip().lower()
        if res in ("q", "g"):
            save_calib(); return
        step = float(input("Step (suggest 0.05): ").strip() or "0.05")
        if res == "w":                     # under-rotated → need more power
            if side == "l": turn_left_scale  = min(2.0, turn_left_scale  + step)
            else:           turn_right_scale = min(2.0, turn_right_scale + step)
        elif res == "o":                   # over-rotated → reduce TURN_COAST_DEG
            print("  Tip: reduce TURN_COAST_DEG at the top of this file.")


def tune_coast():
    """Interactive helper to dial in TURN_COAST_DEG."""
    global TURN_COAST_DEG  # noqa: PLW0603
    print("\n" + "="*60)
    print("COAST TUNING  (adjusts TURN_COAST_DEG)")
    while True:
        print(f"\nCurrent TURN_COAST_DEG = {TURN_COAST_DEG:.1f}°")
        side = input("Test turn (l/r/q): ").strip().lower()
        if side == "q":
            print(f"Set TURN_COAST_DEG = {TURN_COAST_DEG:.1f} in source file.")
            return
        input("Press Enter to turn 90°…")
        stop_all()
        if side == "l":
            turn_left_90()
        else:
            turn_right_90()
        res = input("Over(o) / Under(u) / Good(g) / quit(q): ").strip().lower()
        if res in ("g", "q"):
            return
        step = float(input("Step (suggest 1.0): ").strip() or "1.0")
        if res == "o":
            TURN_COAST_DEG = max(0.0, TURN_COAST_DEG + step)
        elif res == "u":
            TURN_COAST_DEG = max(0.0, TURN_COAST_DEG - step)


# ============================================================
# Menu
# ============================================================
def show_menu():
    print("\n" + "="*70)
    print("🤖 GYRO WHEEL CONTROL")
    print(f"  Straight scales : left={left_scale:.3f}  right={right_scale:.3f}")
    print(f"  Turn scales     : L={turn_left_scale:.3f}  R={turn_right_scale:.3f}")
    print(f"  Coast margin    : {TURN_COAST_DEG:.1f}°")
    print("="*70)
    print(" 1. Forward  3s   (gyro-straight)")
    print(" 2. Backward 3s   (gyro-straight)")
    print(" 3. Turn LEFT  90° (gyro)")
    print(" 4. Turn RIGHT 90° (gyro)")
    print(" 5. Square test   (gyro)")
    print(" 6. Straight + Back test")
    print(" 7. Calibrate STRAIGHT")
    print(" 8. Calibrate TURNS")
    print(" 9. Tune COAST margin")
    print(" 0. Exit")
    print("="*70)


def main():
    load_calib()
    setup()
    mpu_init()
    try:
        while True:
            show_menu()
            c = input("Choose: ").strip()
            if   c == "1": drive_straight_gyro(3.0)
            elif c == "2": drive_backward_gyro(3.0)
            elif c == "3": turn_left_90()
            elif c == "4": turn_right_90()
            elif c == "5": square_test_gyro()
            elif c == "6": straight_back_test()
            elif c == "7": calibrate_straight()
            elif c == "8": calibrate_turns()
            elif c == "9": tune_coast()
            elif c == "0":
                save_calib()
                break
            else:
                print("Invalid!")
    finally:
        stop_all()
        for pwm in [p_rf, p_lf, p_lb, p_rb]:
            try:
                if pwm: pwm.stop()
            except Exception:
                pass
        GPIO.cleanup()


if __name__ == "__main__":
    main()
