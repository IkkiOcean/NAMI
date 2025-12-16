#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# ===== GPIO ASSIGNMENT (BCM) =====
# RIGHT H-bridge
RIGHT_IN1 = 17  # front-right motor input A
RIGHT_IN2 = 27  # front-right motor input B
RIGHT_IN3 = 22  # back-right motor input A
RIGHT_IN4 = 23  # back-right motor input B

# LEFT H-bridge
LEFT_IN1 = 21   # back-left motor input A
LEFT_IN2 = 20   # back-left motor input B
LEFT_IN3 = 16   # front-left motor input A
LEFT_IN4 = 12   # front-left motor input B

ALL_PINS = [
    RIGHT_IN1, RIGHT_IN2, RIGHT_IN3, RIGHT_IN4,
    LEFT_IN1, LEFT_IN2, LEFT_IN3, LEFT_IN4
]

def setup():
    GPIO.setmode(GPIO.BCM)
    for p in ALL_PINS:
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

def stop_all():
    for p in ALL_PINS:
        GPIO.output(p, GPIO.LOW)

# ===== LOW-LEVEL MOTOR HELPERS =====
def motor_forward(in_a, in_b):
    GPIO.output(in_a, GPIO.HIGH)
    GPIO.output(in_b, GPIO.LOW)

def motor_backward(in_a, in_b):
    GPIO.output(in_a, GPIO.LOW)
    GPIO.output(in_b, GPIO.HIGH)

def motor_brake(in_a, in_b):
    GPIO.output(in_a, GPIO.LOW)
    GPIO.output(in_b, GPIO.LOW)

# ===== SIDE HELPERS =====
def right_forward():
    motor_forward(RIGHT_IN1, RIGHT_IN2)
    motor_forward(RIGHT_IN3, RIGHT_IN4)

def right_backward():
    motor_backward(RIGHT_IN1, RIGHT_IN2)
    motor_backward(RIGHT_IN3, RIGHT_IN4)

def right_brake():
    motor_brake(RIGHT_IN1, RIGHT_IN2)
    motor_brake(RIGHT_IN3, RIGHT_IN4)

def left_forward():
    motor_forward(LEFT_IN1, LEFT_IN2)
    motor_forward(LEFT_IN3, LEFT_IN4)

def left_backward():
    motor_backward(LEFT_IN1, LEFT_IN2)
    motor_backward(LEFT_IN3, LEFT_IN4)

def left_brake():
    motor_brake(LEFT_IN1, LEFT_IN2)
    motor_brake(LEFT_IN3, LEFT_IN4)

# ===== ROBOT MOTION =====
def forward():
    right_forward()
    left_forward()

def backward():
    right_backward()
    left_backward()

def turn_left():
    right_forward()
    left_backward()

def turn_right():
    right_backward()
    left_forward()

def main():
    setup()
    try:
        print("Both sides FORWARD 2s")
        forward()
        time.sleep(2.0)

        print("Stop 1s")
        stop_all()
        time.sleep(1.0)

        print("Both sides BACKWARD 2s")
        backward()
        time.sleep(2.0)

        print("Stop 1s")
        stop_all()
        time.sleep(1.0)

        print("Turn left 1s")
        turn_left()
        time.sleep(1.0)
        stop_all()
        time.sleep(1.0)

        print("Turn right 1s")
        turn_right()
        time.sleep(1.0)
        stop_all()

        print("Right only forward 2s")
        right_forward()
        time.sleep(2.0)
        right_brake()
        time.sleep(1.0)

        print("Left only forward 2s")
        left_forward()
        time.sleep(2.0)
        left_brake()
        time.sleep(1.0)

        print("Done.")

    finally:
        stop_all()
        GPIO.cleanup()

if __name__ == "__main__":
    main()