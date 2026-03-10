#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import os
import json

# ============================================================
# PIN MAP (BCM)
# Right H-bridge -> FRONT wheels (EN pins: 13 & 24)
#   EN 13 -> Right Front wheel (RF)  dir: 17/27
#   EN 24 -> Left  Front wheel (LF)  dir: 22/23
#
# Left H-bridge  -> BACK wheels  (EN pins: 12 & 19)
#   EN 12 -> Left  Back wheel (LB)    dir: 21/20
#   EN 19 -> Right Back wheel (RB)    dir: 16/6
# ============================================================

# Direction pins
RF_IN1, RF_IN2 = 17, 27
LF_IN1, LF_IN2 = 22, 23
LB_IN1, LB_IN2 = 21, 20
RB_IN1, RB_IN2 = 16, 6

ALL_DIR_PINS = [RF_IN1, RF_IN2, LF_IN1, LF_IN2, LB_IN1, LB_IN2, RB_IN1, RB_IN2]

# Enable pins (PWM)
EN_RF = 13
EN_LF = 24
EN_LB = 12
EN_RB = 19

# PWM objects
p_rf = p_lf = p_lb = p_rb = None

# ============================================================
# Invert drive globally if your wiring swaps forward/back
INVERT_DRIVE = True

# Per-wheel inversion (True = wired backward)
WHEEL_INVERT = {
    "RF": False,
    "LF": False,
    "LB": True,   # back-left wheel is reversed
    "RB": False
}

# PWM settings
PWM_HZ = 1000
MAX_DUTY = 100

BASE_DRIVE_DUTY = 100     # forward/backward test
BASE_TURN_DUTY  = 100     # in-place turn test

# Calibration file
CALIB_FILE = "/home/ikkiocean/wheel_calib.json"

# Straight trim (left vs right)
left_scale = 1.00
right_scale = 1.00

# Turn trims
turn_left_scale = 1.00
turn_right_scale = 1.00


# ======================
# Utilities
# ======================
def clamp_dc(x):
    x = float(x)
    if x < 0: return 0
    if x > 100: return 100
    return int(round(x))


def load_calib():
    global left_scale, right_scale, turn_left_scale, turn_right_scale
    try:
        if os.path.exists(CALIB_FILE):
            with open(CALIB_FILE, "r") as f:
                d = json.load(f)
            left_scale = float(d.get("left_scale", 1.0))
            right_scale = float(d.get("right_scale", 1.0))
            turn_left_scale = float(d.get("turn_left_scale", 1.0))
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
        "left_scale": left_scale,
        "right_scale": right_scale,
        "turn_left_scale": turn_left_scale,
        "turn_right_scale": turn_right_scale,
    }
    try:
        with open(CALIB_FILE, "w") as f:
            json.dump(d, f, indent=2)
        print("✓ Calibration saved")
    except Exception as e:
        print(f"✗ Save calib failed: {e}")


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

    # Start at 0
    for p in [p_rf, p_lf, p_lb, p_rb]:
        p.start(0)


def stop_all():
    for p in ALL_DIR_PINS:
        GPIO.output(p, GPIO.LOW)
    for pwm in [p_rf, p_lf, p_lb, p_rb]:
        try:
            if pwm:
                pwm.ChangeDutyCycle(0)
        except:
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


# ======================
# Motion primitives
# ======================
def forward():
    fw = not INVERT_DRIVE
    set_pair(RF_IN1, RF_IN2, forward=fw ^ WHEEL_INVERT["RF"])
    set_pair(LF_IN1, LF_IN2, forward=fw ^ WHEEL_INVERT["LF"])
    set_pair(LB_IN1, LB_IN2, forward=fw ^ WHEEL_INVERT["LB"])
    set_pair(RB_IN1, RB_IN2, forward=fw ^ WHEEL_INVERT["RB"])
    apply_pwm(BASE_DRIVE_DUTY)


def backward():
    bw = INVERT_DRIVE
    set_pair(RF_IN1, RF_IN2, forward=bw ^ WHEEL_INVERT["RF"])
    set_pair(LF_IN1, LF_IN2, forward=bw ^ WHEEL_INVERT["LF"])
    set_pair(LB_IN1, LB_IN2, forward=bw ^ WHEEL_INVERT["LB"])
    set_pair(RB_IN1, RB_IN2, forward=bw ^ WHEEL_INVERT["RB"])
    apply_pwm(BASE_DRIVE_DUTY)


def turn_left_inplace():
    left_fw = INVERT_DRIVE
    right_fw = not INVERT_DRIVE
    set_pair(LF_IN1, LF_IN2, forward=left_fw ^ WHEEL_INVERT["LF"])
    set_pair(LB_IN1, LB_IN2, forward=left_fw ^ WHEEL_INVERT["LB"])
    set_pair(RF_IN1, RF_IN2, forward=right_fw ^ WHEEL_INVERT["RF"])
    set_pair(RB_IN1, RB_IN2, forward=right_fw ^ WHEEL_INVERT["RB"])
    apply_pwm(BASE_TURN_DUTY, extra_left=turn_left_scale, extra_right=turn_left_scale)


def turn_right_inplace():
    left_fw = not INVERT_DRIVE
    right_fw = INVERT_DRIVE
    set_pair(LF_IN1, LF_IN2, forward=left_fw ^ WHEEL_INVERT["LF"])
    set_pair(LB_IN1, LB_IN2, forward=left_fw ^ WHEEL_INVERT["LB"])
    set_pair(RF_IN1, RF_IN2, forward=right_fw ^ WHEEL_INVERT["RF"])
    set_pair(RB_IN1, RB_IN2, forward=right_fw ^ WHEEL_INVERT["RB"])
    apply_pwm(BASE_TURN_DUTY, extra_left=turn_right_scale, extra_right=turn_right_scale)


# ======================
# Tests
# ======================
def test_one_wheel(name, in1, in2, pwm_obj):
    print(f"\n🛞 {name} wheel: FORWARD 1.5s @100%")
    stop_all()
    pwm_obj.ChangeDutyCycle(100)
    # determine correct software forward
    fwd = True
    if name == "LEFT BACK":  # LB is inverted
        fwd = not True
    set_pair(in1, in2, forward=fwd)
    time.sleep(1.5)
    stop_all()
    time.sleep(0.5)


def wheel_mapping_test():
    test_one_wheel("RIGHT FRONT", RF_IN1, RF_IN2, p_rf)
    test_one_wheel("LEFT FRONT",  LF_IN1, LF_IN2, p_lf)
    test_one_wheel("LEFT BACK",   LB_IN1, LB_IN2, p_lb)
    test_one_wheel("RIGHT BACK",  RB_IN1, RB_IN2, p_rb)


def square_test():
    print("\n🔲 SQUARE TEST (forward 2s + left turn 2s) @100%")
    for i in range(4):
        print(f"Leg {i+1}/4: forward")
        stop_all()
        forward()
        time.sleep(2)
        stop_all()
        time.sleep(0.3)

        print(f"Leg {i+1}/4: left turn")
        stop_all()
        turn_left_inplace()
        time.sleep(2)
        stop_all()
        time.sleep(0.3)
    print("✅ Square test done")


# ======================
# Calibration functions remain unchanged
# ======================
def calibrate_straight():
    global left_scale, right_scale
    print("\n" + "="*60)
    print("STRAIGHT CALIBRATION (left vs right scaling)")
    while True:
        print(f"\nCurrent: left_scale={left_scale:.3f} right_scale={right_scale:.3f}")
        input("Press Enter to run forward test (3s)...")
        stop_all()
        forward()
        time.sleep(3)
        stop_all()
        ans = input("Veer? (l/r/s=straight, q=save+quit): ").strip().lower()
        if ans == "q":
            save_calib()
            return
        if ans == "s":
            save_calib()
            return
        step = float(input("Adjustment step (suggest 0.02): ").strip() or "0.02")
        if ans == "l":
            left_scale = max(0.50, left_scale - step)
        elif ans == "r":
            right_scale = max(0.50, right_scale - step)


def calibrate_turns():
    global turn_left_scale, turn_right_scale
    print("\n" + "="*60)
    print("TURN CALIBRATION (in-place spin)")
    while True:
        print(f"\nCurrent: turnL={turn_left_scale:.3f} turnR={turn_right_scale:.3f}")
        side = input("Test which? (l=left / r=right / q=save+quit): ").strip().lower()
        if side == "q":
            save_calib()
            return
        if side not in ("l", "r"):
            continue
        input("Press Enter to run turn test (2s)...")
        stop_all()
        if side == "l":
            turn_left_inplace()
        else:
            turn_right_inplace()
        time.sleep(2)
        stop_all()
        res = input("Result? (w=weak / g=good / q=quit): ").strip().lower()
        if res == "q":
            save_calib()
            return
        if res == "g":
            continue
        if res == "w":
            step = float(input("Increase by step (suggest 0.05): ").strip() or "0.05")
            if side == "l":
                turn_left_scale = min(2.00, turn_left_scale + step)
            else:
                turn_right_scale = min(2.00, turn_right_scale + step)


# ======================
# Menu
# ======================
def show_menu():
    print("\n" + "="*70)
    print("🤖 WHEEL CONTROL + CALIBRATION")
    print(f"INVERT_DRIVE={INVERT_DRIVE}")
    print(f"Straight scales: left={left_scale:.3f} right={right_scale:.3f}")
    print(f"Turn scales:     turnL={turn_left_scale:.3f} turnR={turn_right_scale:.3f}")
    print("="*70)
    print("1. Wheel mapping test (each wheel forward)")
    print("2. Forward (3s)")
    print("3. Backward (3s)")
    print("4. Turn LEFT in-place (2s)")
    print("5. Turn RIGHT in-place (2s)")
    print("6. Calibrate STRAIGHT")
    print("7. Calibrate TURNS")
    print("8. Square test")
    print("0. Exit")
    print("="*70)


def main():
    load_calib()
    setup()
    try:
        while True:
            show_menu()
            c = input("Choose: ").strip()
            if c == "1":
                wheel_mapping_test()
            elif c == "2":
                stop_all()
                forward()
                time.sleep(3)
                stop_all()
            elif c == "3":
                stop_all()
                backward()
                time.sleep(3)
                stop_all()
            elif c == "4":
                stop_all()
                turn_left_inplace()
                time.sleep(2)
                stop_all()
            elif c == "5":
                stop_all()
                turn_right_inplace()
                time.sleep(2)
                stop_all()
            elif c == "6":
                calibrate_straight()
            elif c == "7":
                calibrate_turns()
            elif c == "8":
                square_test()
            elif c == "0":
                save_calib()
                break
            else:
                print("Invalid!")
    finally:
        stop_all()
        for pwm in [p_rf, p_lf, p_lb, p_rb]:
            try:
                if pwm:
                    pwm.stop()
            except:
                pass
        GPIO.cleanup()


if __name__ == "__main__":
    main()