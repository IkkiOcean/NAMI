#!/usr/bin/env python3
"""
motor_bridge.py  —  ROS2 node that subscribes to /cmd_vel (Twist)
and drives the 4-wheel GPIO bot via wheel_control_gyro functions.

Run alongside explore.py:
  Terminal 4:  python3 motor_bridge.py

Twist convention used by Nav2 / explore.py:
  linear.x  > 0  →  forward
  linear.x  < 0  →  backward
  angular.z > 0  →  left  (CCW)
  angular.z < 0  →  right (CW)

The bridge maps those to PWM duty-cycle pairs so the robot moves
smoothly instead of snap-to-90°.  Gyro-exact turns are only used
when explore.py sends a pure-rotate command (linear.x == 0).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ── import the hardware layer ──────────────────────────────────
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

import RPi.GPIO as GPIO

# Reuse all pin/PWM globals from wheel_control_gyro
from wheel_control_gyro import (
    setup, stop_all, mpu_init, load_calib,
    _set_forward_dirs, _set_backward_dirs,
    _set_turn_left_dirs, _set_turn_right_dirs,
    turn_gyro,
    clamp_dc, apply_pwm,
    p_rf, p_lf, p_lb, p_rb,
    left_scale, right_scale,
    BASE_DRIVE_DUTY, BASE_TURN_DUTY,
)

# ── tuning ─────────────────────────────────────────────────────
# Max linear speed maps to this duty cycle
LINEAR_MAX_DUTY  = 80    # % (keep < 100 for control headroom)
# Max angular speed maps to this duty cycle differential
ANGULAR_MAX_DIFF = 60    # % per side

# Threshold: if |linear.x| < this, treat as pure rotation
LINEAR_DEADBAND  = 0.02  # m/s


class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')
        self.get_logger().info("Motor bridge starting — init GPIO + MPU…")

        load_calib()
        setup()
        mpu_init()

        # Re-import p_* after setup() populates them
        import wheel_control_gyro as wc
        self._wc = wc

        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.get_logger().info("✅ Motor bridge ready, listening on /cmd_vel")

    def _cmd_cb(self, msg: Twist):
        lx = msg.linear.x
        az = msg.angular.z

        # ── pure stop ─────────────────────────────────────────
        if abs(lx) < LINEAR_DEADBAND and abs(az) < 0.01:
            stop_all()
            return

        # ── pure rotation → use gyro for accuracy ─────────────
        if abs(lx) < LINEAR_DEADBAND:
            # angular.z in rad/s; we just care about direction here.
            # explore.py will only send short bursts; let bridge
            # handle them as timed spin at fixed duty.
            direction = "left" if az > 0 else "right"
            # Convert rad/s magnitude to a short timed spin
            # (explore.py sends velocity, not angle, so we run at
            #  fixed duty and let the caller time it via cmd_vel bursts)
            if direction == "left":
                _set_turn_left_dirs()
            else:
                _set_turn_right_dirs()
            duty = clamp_dc(BASE_TURN_DUTY * min(1.0, abs(az) / 1.5))
            self._wc.p_lf.ChangeDutyCycle(duty)
            self._wc.p_lb.ChangeDutyCycle(duty)
            self._wc.p_rf.ChangeDutyCycle(duty)
            self._wc.p_rb.ChangeDutyCycle(duty)
            return

        # ── combined drive + steer (differential) ─────────────
        # Scale linear speed
        lin_duty = clamp_dc(LINEAR_MAX_DUTY * min(1.0, abs(lx)))
        # Differential: positive az → left turn → reduce left, boost right
        diff = clamp_dc(ANGULAR_MAX_DIFF * min(1.0, abs(az) / 1.5))

        if lx > 0:
            _set_forward_dirs()
        else:
            _set_backward_dirs()
            diff = -diff   # invert differential for reverse

        if az >= 0:   # turn left
            left_duty  = clamp_dc((lin_duty - diff) * left_scale)
            right_duty = clamp_dc((lin_duty + diff) * right_scale)
        else:         # turn right
            left_duty  = clamp_dc((lin_duty + diff) * left_scale)
            right_duty = clamp_dc((lin_duty - diff) * right_scale)

        self._wc.p_lf.ChangeDutyCycle(left_duty)
        self._wc.p_lb.ChangeDutyCycle(left_duty)
        self._wc.p_rf.ChangeDutyCycle(right_duty)
        self._wc.p_rb.ChangeDutyCycle(right_duty)

    def destroy_node(self):
        stop_all()
        for pwm in [self._wc.p_rf, self._wc.p_lf,
                    self._wc.p_lb, self._wc.p_rb]:
            try:
                if pwm: pwm.stop()
            except Exception:
                pass
        GPIO.cleanup()
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()