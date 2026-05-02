#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import os

# ===== GPIO ASSIGNMENT (BCM) - SAME =====
RIGHT_IN1 = 17; RIGHT_IN2 = 27; RIGHT_IN3 = 22; RIGHT_IN4 = 23
LEFT_IN1  = 21; LEFT_IN2 = 20; LEFT_IN3 = 16; LEFT_IN4 = 6

ALL_IN_PINS = [RIGHT_IN1, RIGHT_IN2, RIGHT_IN3, RIGHT_IN4, LEFT_IN1, LEFT_IN2, LEFT_IN3, LEFT_IN4]

RIGHT_EN_FRONT = 13; RIGHT_EN_BACK  = 24
LEFT_EN_BACK   = 12; LEFT_EN_FRONT  = 19

# PWM objects (global)
pr_front = pr_back = pl_back = pl_front = None

# STRAIGHT CALIBRATION VALUES
CALIB_FILE = "/home/ikkiocean/robot_calib.txt"
RIGHT_DUTY = 100
LEFT_DUTY = 100

# TURN CALIBRATION (for small wheelbase tight turns)
TURN_LEFT_DUTY_R = 100  # Right forward speed when turning left
TURN_LEFT_DUTY_L = 60   # Left backward speed when turning left  
TURN_RIGHT_DUTY_R = 60  # Right backward speed when turning right
TURN_RIGHT_DUTY_L = 100 # Left forward speed when turning right
TURN_CALIB_FILE = "/home/ikkiocean/robot_turn_calib.txt"

def load_calibration():
    global RIGHT_DUTY, LEFT_DUTY
    try:
        if os.path.exists(CALIB_FILE):
            with open(CALIB_FILE, 'r') as f:
                RIGHT_DUTY = int(f.readline().strip())
                LEFT_DUTY = int(f.readline().strip())
            print(f"✓ LOADED: RIGHT={RIGHT_DUTY}%, LEFT={LEFT_DUTY}%")
            return True
    except:
        pass
    print("⚠ No saved calibration, using defaults 100/100")
    return False

def load_turn_calibration():
    global TURN_LEFT_DUTY_R, TURN_LEFT_DUTY_L, TURN_RIGHT_DUTY_R, TURN_RIGHT_DUTY_L
    try:
        if os.path.exists(TURN_CALIB_FILE):
            with open(TURN_CALIB_FILE, 'r') as f:
                TURN_LEFT_DUTY_R = int(f.readline().strip())
                TURN_LEFT_DUTY_L = int(f.readline().strip())
                TURN_RIGHT_DUTY_R = int(f.readline().strip())
                TURN_RIGHT_DUTY_L = int(f.readline().strip())
            print(f"✓ TURN LOADED: L(R:{TURN_LEFT_DUTY_R}%,L:{TURN_LEFT_DUTY_L}%) R(R:{TURN_RIGHT_DUTY_R}%,L:{TURN_RIGHT_DUTY_L}%)")
            return True
    except:
        pass
    print("⚠ Using default turn calibration")
    return False

def save_calibration():
    try:
        with open(CALIB_FILE, 'w') as f:
            f.write(f"{RIGHT_DUTY}\n{LEFT_DUTY}\n")
        print(f"✓ STRAIGHT SAVED: RIGHT={RIGHT_DUTY}%, LEFT={LEFT_DUTY}%")
    except Exception as e:
        print(f"✗ Save failed: {e}")

def save_turn_calibration():
    try:
        with open(TURN_CALIB_FILE, 'w') as f:
            f.write(f"{TURN_LEFT_DUTY_R}\n{TURN_LEFT_DUTY_L}\n{TURN_RIGHT_DUTY_R}\n{TURN_RIGHT_DUTY_L}\n")
        print("✓ TURNS SAVED!")
    except Exception as e:
        print(f"✗ Turn save failed: {e}")

def setup():
    global pr_front, pr_back, pl_back, pl_front
    GPIO.setmode(GPIO.BCM)
    for p in ALL_IN_PINS:
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(RIGHT_EN_FRONT, GPIO.OUT); GPIO.setup(RIGHT_EN_BACK, GPIO.OUT)
    GPIO.setup(LEFT_EN_BACK, GPIO.OUT); GPIO.setup(LEFT_EN_FRONT, GPIO.OUT)
    
    pr_front = GPIO.PWM(RIGHT_EN_FRONT, 1000)
    pr_back  = GPIO.PWM(RIGHT_EN_BACK,  1000)
    pl_back  = GPIO.PWM(LEFT_EN_BACK,   1000)
    pl_front = GPIO.PWM(LEFT_EN_FRONT,  1000)
    
    pr_front.start(RIGHT_DUTY); pr_back.start(RIGHT_DUTY)
    pl_back.start(LEFT_DUTY); pl_front.start(LEFT_DUTY)

def stop_all():
    for p in ALL_IN_PINS: GPIO.output(p, GPIO.LOW)

# ===== FIXED MOTOR LOGIC (REVERSED + SPEED FIRST) =====
def forward():
    GPIO.output(RIGHT_IN1, GPIO.LOW); GPIO.output(RIGHT_IN2, GPIO.HIGH)
    GPIO.output(RIGHT_IN3, GPIO.LOW); GPIO.output(RIGHT_IN4, GPIO.HIGH)
    GPIO.output(LEFT_IN1, GPIO.LOW); GPIO.output(LEFT_IN2, GPIO.HIGH)
    GPIO.output(LEFT_IN3, GPIO.LOW); GPIO.output(LEFT_IN4, GPIO.HIGH)

def backward():
    GPIO.output(RIGHT_IN1, GPIO.HIGH); GPIO.output(RIGHT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN3, GPIO.HIGH); GPIO.output(RIGHT_IN4, GPIO.LOW)
    GPIO.output(LEFT_IN1, GPIO.HIGH); GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(LEFT_IN3, GPIO.HIGH); GPIO.output(LEFT_IN4, GPIO.LOW)

def turn_left():
    # SPEED FIRST, then direction (fixes backward issue)
    apply_speed(TURN_LEFT_DUTY_R, TURN_LEFT_DUTY_L)
    time.sleep(0.05)  # PWM settle
    GPIO.output(RIGHT_IN1, GPIO.LOW); GPIO.output(RIGHT_IN2, GPIO.HIGH)
    GPIO.output(RIGHT_IN3, GPIO.LOW); GPIO.output(RIGHT_IN4, GPIO.HIGH)
    GPIO.output(LEFT_IN1, GPIO.HIGH); GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(LEFT_IN3, GPIO.HIGH); GPIO.output(LEFT_IN4, GPIO.LOW)

def turn_right():
    # SPEED FIRST, then direction
    apply_speed(TURN_RIGHT_DUTY_R, TURN_RIGHT_DUTY_L)
    time.sleep(0.05)  # PWM settle
    GPIO.output(RIGHT_IN1, GPIO.HIGH); GPIO.output(RIGHT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN3, GPIO.HIGH); GPIO.output(RIGHT_IN4, GPIO.LOW)
    GPIO.output(LEFT_IN1, GPIO.LOW); GPIO.output(LEFT_IN2, GPIO.HIGH)
    GPIO.output(LEFT_IN3, GPIO.LOW); GPIO.output(LEFT_IN4, GPIO.HIGH)

# ===== SPEED MODES =====
def set_full_speed(): 
    print(f"FULL: R={RIGHT_DUTY}% L={LEFT_DUTY}%")
    return RIGHT_DUTY, LEFT_DUTY

def set_slow_speed(): 
    r = int(RIGHT_DUTY * 0.7); l = int(LEFT_DUTY * 0.75)
    print(f"SLOW: R={r}% L={l}%")
    return r, l

def set_ultra_slow(): 
    r = int(RIGHT_DUTY * 0.5); l = int(LEFT_DUTY * 0.55)
    print(f"ULTRA SLOW: R={r}% L={l}%")
    return r, l

def apply_speed(right_duty, left_duty):
    pr_front.ChangeDutyCycle(right_duty); pr_back.ChangeDutyCycle(right_duty)
    pl_back.ChangeDutyCycle(left_duty); pl_front.ChangeDutyCycle(left_duty)

# ===== STRAIGHT CALIBRATION =====
def calibrate_straight():
    global RIGHT_DUTY, LEFT_DUTY
    print("\n" + "="*60)
    print("🎯 STRAIGHT LINE CALIBRATION (REVERSED MOTORS)")
    print("="*60)
    
    while True:
        test_duty_r, test_duty_l = set_full_speed()
        apply_speed(test_duty_r, test_duty_l)
        forward()
        print("RUNNING FORWARD 5s... WATCH!")
        time.sleep(5)
        stop_all()
        
        print(f"\nCurrent: RIGHT={RIGHT_DUTY}%, LEFT={LEFT_DUTY}%")
        veer = input("Veer? (l=left/r=right/s=straight/q=quit): ").lower()
        
        if veer == 'q':
            save_calibration()
            break
        elif veer == 'l':  # veers LEFT → right too strong
            new_r = int(input(f"New RIGHT_DUTY (<{RIGHT_DUTY}): "))
            RIGHT_DUTY = max(50, new_r)
        elif veer == 'r':  # veers RIGHT → left too weak
            new_l = int(input(f"New LEFT_DUTY (>{LEFT_DUTY}): "))
            LEFT_DUTY = min(100, new_l)
        elif veer == 's':
            print("🎉 PERFECT STRAIGHT!")
            save_calibration()
            break
        else:
            print("l=left, r=right, s=straight, q=quit")

# ===== TURN CALIBRATION (FIXED) =====
def calibrate_turns():
    global TURN_LEFT_DUTY_R, TURN_LEFT_DUTY_L, TURN_RIGHT_DUTY_R, TURN_RIGHT_DUTY_L
    print("\n🎯 TURN RADIUS CALIBRATION - FIXED")
    print("Goal: Tight 90° turns (~0.5-1m radius)\n")
    
    # LEFT TURN
    print("--- LEFT TURN (Right FWD + Left BACK) ---")
    while True:
        print(f"Current LEFT: Right={TURN_LEFT_DUTY_R}%(fwd), Left={TURN_LEFT_DUTY_L}%(back)")
        turn_left()
        print("LEFT TURN 3s... CHECK DIRECTION + RADIUS!")
        time.sleep(3)
        stop_all()
        
        resp = input("Status? (b=backwards/f=forward/t=tight/w=wide/g=good/q=quit): ").lower()
        if resp == 'q': break
        elif resp == 'b':
            print("⚠ BACKWARDS! Check motor wiring or IN pins")
            break
        elif resp == 'f':
            rad = input("Radius? (t=tight/w=wide/g=good): ").lower()
            if rad == 't': TURN_LEFT_DUTY_L = int(input(f"New LEFT BACK (>{TURN_LEFT_DUTY_L}): "))
            elif rad == 'w': TURN_LEFT_DUTY_R = int(input(f"New RIGHT FWD (>{TURN_LEFT_DUTY_R}): "))
            elif rad == 'g': print("✅ LEFT GOOD!"); break
    
    # RIGHT TURN
    print("\n--- RIGHT TURN (Right BACK + Left FWD) ---")
    while True:
        print(f"Current RIGHT: Right={TURN_RIGHT_DUTY_R}%(back), Left={TURN_RIGHT_DUTY_L}%(fwd)")
        turn_right()
        print("RIGHT TURN 3s... CHECK DIRECTION + RADIUS!")
        time.sleep(3)
        stop_all()
        
        resp = input("Status? (b=backwards/f=forward/t=tight/w=wide/g=good/q=quit): ").lower()
        if resp == 'q': break
        elif resp == 'b':
            print("⚠ BACKWARDS! Check motor wiring or IN pins")
            break
        elif resp == 'f':
            rad = input("Radius? (t=tight/w=wide/g=good): ").lower()
            if rad == 't': TURN_RIGHT_DUTY_R = int(input(f"New RIGHT BACK (>{TURN_RIGHT_DUTY_R}): "))
            elif rad == 'w': TURN_RIGHT_DUTY_L = int(input(f"New LEFT FWD (>{TURN_RIGHT_DUTY_L}): "))
            elif rad == 'g': print("✅ RIGHT GOOD!"); break
    
    save_turn_calibration()

# ===== FULL SYSTEM TEST =====
def full_system_test():
    global RIGHT_DUTY, LEFT_DUTY, TURN_LEFT_DUTY_R, TURN_LEFT_DUTY_L, TURN_RIGHT_DUTY_R, TURN_RIGHT_DUTY_L
    
    print("\n" + "="*70)
    print("🧪 FULL SYSTEM TEST - ALL DIRECTIONS + CALIBRATION")
    print("="*70)
    print(f"STRAIGHT: R={RIGHT_DUTY}% L={LEFT_DUTY}%")
    print(f"TURNS: L(R:{TURN_LEFT_DUTY_R},L:{TURN_LEFT_DUTY_L}) R(R:{TURN_RIGHT_DUTY_R},L:{TURN_RIGHT_DUTY_L})")
    print("\n⏳ Press Ctrl+C to stop any test early")
    input("Press Enter to START FULL TEST...")
    
    try:
        # 1. INDIVIDUAL WHEEL TEST
        print("\n1️⃣ WHEEL DIRECTION TEST (1s each)")
        print("RIGHT FORWARD:")
        apply_speed(RIGHT_DUTY, 0)
        forward()
        time.sleep(1); stop_all(); time.sleep(0.5)
        
        print("RIGHT BACKWARD:")
        apply_speed(RIGHT_DUTY, 0)
        backward()
        time.sleep(1); stop_all(); time.sleep(0.5)
        
        print("LEFT FORWARD:")
        apply_speed(0, LEFT_DUTY)
        forward()
        time.sleep(1); stop_all(); time.sleep(0.5)
        
        print("LEFT BACKWARD:")
        apply_speed(0, LEFT_DUTY)
        backward()
        time.sleep(1); stop_all(); time.sleep(1)
        
        wheel_ok = input("✅ Wheels correct direction? (y/n): ").lower() == 'y'
        
        # 2. STRAIGHT LINE TEST
        print("\n2️⃣ STRAIGHT LINE TEST (5s)")
        r, l = set_full_speed()
        apply_speed(r, l)
        forward()
        print("GO STRAIGHT 5s... WATCH VEER!")
        time.sleep(5)
        stop_all()
        straight_ok = input("✅ Goes straight? (y/n): ").lower() == 'y'
        
        # 3. TURN TESTS
        print("\n3️⃣ TURN TESTS (3s each)")
        print("LEFT TURN:")
        turn_left()
        print("LEFT TURN 3s... CHECK!")
        time.sleep(3)
        stop_all()
        left_ok = input("✅ Left turn good? (y/n): ").lower() == 'y'
        
        print("RIGHT TURN:")
        turn_right()
        print("RIGHT TURN 3s... CHECK!")
        time.sleep(3)
        stop_all()
        right_ok = input("✅ Right turn good? (y/n): ").lower() == 'y'
        
        # 4. BACKWARD TEST
        print("\n4️⃣ BACKWARD STRAIGHT (3s)")
        r, l = set_full_speed()
        apply_speed(r, l)
        backward()
        print("GO BACKWARD 3s...")
        time.sleep(3)
        stop_all()
        back_ok = input("✅ Backward straight? (y/n): ").lower() == 'y'
        
        # 5. SQUARE TEST
        print("\n5️⃣ SQUARE TEST (closes loop = perfect)")
        for i in range(4):
            print(f"SIDE {i+1}: FORWARD 2s")
            r, l = set_full_speed()
            apply_speed(r, l)
            forward()
            time.sleep(2); stop_all(); time.sleep(0.3)
            
            print(f"SIDE {i+1}: LEFT TURN")
            turn_left()
            time.sleep(2.2)
            stop_all(); time.sleep(0.3)
        
        square_ok = input("✅ Square closed perfectly? (y/n): ").lower() == 'y'
        
        print("\n" + "="*70)
        print("📊 FINAL SUMMARY:")
        print(f"  🛞 Wheels:     {'✅' if wheel_ok else '❌'}")
        print(f"  ➡️  Straight:  {'✅' if straight_ok else '❌'}")
        print(f"  ↰  Left:       {'✅' if left_ok else '❌'}")
        print(f"  ↱  Right:      {'✅' if right_ok else '❌'}")
        print(f"  ⬅️  Backward:   {'✅' if back_ok else '❌'}")
        print(f"  🔲  Square:     {'✅ PERFECT' if square_ok else '⚠ Needs tuning'}")
        print("="*70)
        
    except KeyboardInterrupt:
        print("\n⏹️ Test interrupted")
    finally:
        stop_all()

# ===== SQUARE TEST =====
def square_test():
    print("\n🔲 SQUARE TEST - 4x (forward 2s → turn 90°)")
    r, l = set_full_speed()
    apply_speed(r, l)
    
    for i in range(4):
        print(f"Leg {i+1}/4: FORWARD")
        forward()
        time.sleep(2)
        stop_all(); time.sleep(0.5)
        
        print(f"Leg {i+1}/4: TURN LEFT 90°")
        turn_left()
        time.sleep(2.5)
        stop_all(); time.sleep(0.5)
    
    print("✅ SQUARE COMPLETE!")

# ===== MAIN INTERFACE =====
def show_menu():
    print("\n" + "="*60)
    print("🤖 ROBOT CONTROL - FULL CALIBRATION + TESTS")
    print(f"⚙️  STRAIGHT: R={RIGHT_DUTY}% | L={LEFT_DUTY}%")
    print(f"🔄 TURNS: L(R:{TURN_LEFT_DUTY_R},L:{TURN_LEFT_DUTY_L}) R(R:{TURN_RIGHT_DUTY_R},L:{TURN_RIGHT_DUTY_L})")
    print("="*60)
    print("1. FULL SPEED TEST")
    print("2. SLOW SPEED TEST") 
    print("3. ULTRA SLOW TEST")
    print("4. CALIBRATE STRAIGHT")
    print("5. STRAIGHT TEST (5s)")
    print("6. MANUAL CONTROL (w/a/s/d)")
    print("7. CONTINUOUS FORWARD")
    print("8. CALIBRATE TURNS")
    print("9. SQUARE TEST")
    print("🔥 10. FULL SYSTEM TEST")
    print("0. EXIT")
    print("="*60)

def main():
    load_calibration()
    load_turn_calibration()
    setup()
    try:
        while True:
            show_menu()
            choice = input("Choose: ").strip()
            
            if choice == '1':
                r, l = set_full_speed()
                apply_speed(r, l)
                forward(); time.sleep(2); stop_all()
            elif choice == '2':
                r, l = set_slow_speed()
                apply_speed(r, l)
                forward(); time.sleep(2); stop_all()
            elif choice == '3':
                r, l = set_ultra_slow()
                apply_speed(r, l)
                forward(); time.sleep(2); stop_all()
            elif choice == '4':
                calibrate_straight()
            elif choice == '5':
                r, l = set_full_speed()
                apply_speed(r, l)
                forward(); time.sleep(5); stop_all()
            elif choice == '6':
                print("w=fwd s=back a=left d=right q=quit (calibrated speeds)")
                while True:
                    cmd = input("Cmd: ").lower()
                    if cmd == 'q': break
                    elif cmd == 'w': r,l=set_full_speed(); apply_speed(r,l); forward()
                    elif cmd == 's': r,l=set_full_speed(); apply_speed(r,l); backward()
                    elif cmd == 'a': turn_left()
                    elif cmd == 'd': turn_right()
                    else: stop_all()
                    if cmd != 'q': time.sleep(0.1)
            elif choice == '7':
                try:
                    r,l = set_full_speed()
                    apply_speed(r,l)
                    forward()
                    while True:
                        time.sleep(1)
                except KeyboardInterrupt:
                    pass
                stop_all()
            elif choice == '8':
                calibrate_turns()
            elif choice == '9':
                square_test()
            elif choice == '10':
                full_system_test()
            elif choice == '0':
                save_calibration()
                save_turn_calibration()
                break
            else:
                print("Invalid!")
    finally:
        stop_all()
        global pr_front, pr_back, pl_back, pl_front
        for pwm in [pr_front, pr_back, pl_back, pl_front]:
            try:
                if pwm: pwm.stop()
            except:
                pass
        GPIO.cleanup()

if __name__ == "__main__":
    main()
