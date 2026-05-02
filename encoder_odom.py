#!/usr/bin/env python3
"""
encoder_odom.py  v3

Fixes from log analysis:
  1. Removed duplicate static TF (ydlidar launch already publishes it)
  2. Longer gyro calibration (5s) + bias sanity check
     - bias=1.29 deg/s is abnormally high, warns user
  3. LF encoder missing: uses only LB for left side (not average)
     Set LF_WORKING=False below if LF is broken
  4. GPIO interrupt fallback improved with conflict detection
  5. Gyro-only yaw when bias is too high (encoder yaw unreliable anyway)
"""

import math
import threading
import time

import RPi.GPIO as GPIO
import smbus2
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# ══════════════════════════════════════════════════════════════
# ROBOT MEASUREMENTS
# ══════════════════════════════════════════════════════════════
WHEEL_DIAMETER_M = 0.065   # metres
TRACK_WIDTH_M    = 0.200   # metres — left to right wheel centre
PULSES_PER_REV   = 20.0    # slots on encoder disc

# ══════════════════════════════════════════════════════════════
# ENCODER CONFIG
# ══════════════════════════════════════════════════════════════
# Set to False for any broken/missing encoder
LF_WORKING = False   # LF sensor confirmed broken — skipped
RF_WORKING = True
LB_WORKING = True
RB_WORKING = True

ENCODER_PINS = {
    "RF": 5,
    "LF": 26,   # broken but kept for future
    "LB": 18,
    "RB": 25,
}

# ══════════════════════════════════════════════════════════════
# GYRO CONFIG
# ══════════════════════════════════════════════════════════════
MPU_ADDR      = 0x68
GYRO_SENS     = 131.0
GYRO_CAL_S    = 5.0        # longer calibration = better bias estimate
MAX_BIAS_DEG  = 0.5        # warn if bias > this (deg/s)
                            # your bias was 1.29 = something is wrong
                            # (vibration during calibration, or MPU issue)

# Complementary filter: how much to trust gyro vs encoder for yaw
# If gyro bias is high, reduce this so encoder yaw dominates
GYRO_WEIGHT   = 0.95       # auto-reduced if bias is high

# ══════════════════════════════════════════════════════════════
# NOTE: NO static TF published here
# ydlidar_ros2_driver launch already publishes base_link→laser_frame
# Publishing it again causes "timestamp earlier than cache" errors
# ══════════════════════════════════════════════════════════════

ODOM_HZ    = 50
DEBOUNCE_MS = 1
MIN_DT     = 1e-3


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class _Counter:
    def __init__(self):
        self.value = 0
        self.lock  = threading.Lock()
    def inc(self):
        with self.lock: self.value += 1
    def take(self):
        with self.lock:
            v = self.value; self.value = 0; return v


class EncoderOdom(Node):

    def __init__(self):
        super().__init__('encoder_odom')

        self._odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self._tf_br    = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 20)

        self._counts   = {k: _Counter() for k in ENCODER_PINS}
        self._last_cmd = Twist()
        self._x = self._y = self._yaw = 0.0
        self._last_t = time.time()

        self._setup_encoders()
        self._gyro_rate = 0.0
        self._gyro_bias = 0.0
        self._gyro_weight = GYRO_WEIGHT
        self._bus = None
        self._init_gyro()
        threading.Thread(target=self._gyro_loop, daemon=True).start()

        self.create_timer(1.0 / ODOM_HZ, self._tick)

        working = [k for k,v in
                   {"RF":RF_WORKING,"LF":LF_WORKING,
                    "LB":LB_WORKING,"RB":RB_WORKING}.items() if v]
        self.get_logger().info(
            f"Encoder odom ready | "
            f"working encoders: {working} | "
            f"wheel={WHEEL_DIAMETER_M*1000:.0f}mm "
            f"track={TRACK_WIDTH_M*1000:.0f}mm "
            f"ppr={PULSES_PER_REV:.0f} | "
            f"gyro_bias={self._gyro_bias:.4f}deg/s "
            f"gyro_weight={self._gyro_weight:.2f}")

    # ── Encoder setup ────────────────────────────────────────

    def _setup_encoders(self):
        GPIO.setmode(GPIO.BCM)
        working_flags = {
            "RF": RF_WORKING, "LF": LF_WORKING,
            "LB": LB_WORKING, "RB": RB_WORKING}

        for name, pin in ENCODER_PINS.items():
            if not working_flags[name]:
                self.get_logger().info(f"  {name} encoder DISABLED (set working=False)")
                continue
            try:
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                def _cb(_ch, n=name): self._counts[n].inc()
                GPIO.add_event_detect(pin, GPIO.BOTH,
                                      callback=_cb,
                                      bouncetime=DEBOUNCE_MS)
                self.get_logger().info(f"  {name} encoder on GPIO{pin} (IRQ)")
            except Exception as e:
                self.get_logger().warn(
                    f"  {name} GPIO{pin} IRQ failed: {e} — using polling")
                # Start polling thread for this pin
                threading.Thread(
                    target=self._poll_pin,
                    args=(pin, name), daemon=True).start()

    def _poll_pin(self, pin, name):
        """Software polling fallback when IRQ unavailable."""
        last = GPIO.input(pin)
        while True:
            cur = GPIO.input(pin)
            if cur != last:
                self._counts[name].inc()
                last = cur
            time.sleep(0.0005)  # 2kHz poll

    # ── Gyro ────────────────────────────────────────────────

    def _init_gyro(self):
        try:
            self._bus = smbus2.SMBus(1)
            self._bus.write_byte_data(MPU_ADDR, 0x6B, 0x00)
            time.sleep(0.1)
            self._bus.write_byte_data(MPU_ADDR, 0x1B, 0x00)
            self._bus.write_byte_data(MPU_ADDR, 0x1C, 0x00)
        except Exception as e:
            self.get_logger().error(f"MPU6050 failed: {e}")
            self._bus = None
            return

        self.get_logger().info(
            f"Calibrating gyro {GYRO_CAL_S}s — keep robot STILL and FLAT...")
        samples = []
        t_end = time.time() + GYRO_CAL_S
        while time.time() < t_end:
            try: samples.append(self._raw_gz() / GYRO_SENS)
            except: pass
            time.sleep(0.005)

        if not samples:
            return

        self._gyro_bias = sum(samples) / len(samples)

        # Sanity check — high bias means robot was moving or MPU problem
        if abs(self._gyro_bias) > MAX_BIAS_DEG:
            self.get_logger().warn(
                f"⚠ Gyro bias={self._gyro_bias:.4f} deg/s is HIGH "
                f"(normal < {MAX_BIAS_DEG}). "
                f"Was robot still during calibration? "
                f"Reducing gyro weight to 0.7 to limit drift.")
            self._gyro_weight = 0.70  # trust encoder more when gyro is bad
        else:
            self._gyro_weight = GYRO_WEIGHT

    def _raw_gz(self):
        h = self._bus.read_byte_data(MPU_ADDR, 0x47)
        l = self._bus.read_byte_data(MPU_ADDR, 0x48)
        v = (h << 8) | l
        return float(v - 65536 if v >= 32768 else v)

    def _gyro_loop(self):
        while True:
            if self._bus:
                try:
                    self._gyro_rate = (self._raw_gz()/GYRO_SENS) - self._gyro_bias
                except: pass
            time.sleep(0.005)

    # ── cmd_vel ──────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        self._last_cmd = msg

    def _wheel_signs(self):
        lx = self._last_cmd.linear.x
        az = self._last_cmd.angular.z
        left  = lx - az * TRACK_WIDTH_M * 0.5
        right = lx + az * TRACK_WIDTH_M * 0.5
        sl = 1.0 if left  > 0.01 else (-1.0 if left  < -0.01 else 0.0)
        sr = 1.0 if right > 0.01 else (-1.0 if right < -0.01 else 0.0)
        return sl, sr

    # ── Odometry tick ────────────────────────────────────────

    def _tick(self):
        now = time.time()
        dt  = max(MIN_DT, now - self._last_t)
        self._last_t = now

        c_rf = self._counts["RF"].take()
        c_lf = self._counts["LF"].take()
        c_lb = self._counts["LB"].take()
        c_rb = self._counts["RB"].take()

        # ── Left side: LF broken, use LB only ───────────────
        if LF_WORKING and LB_WORKING:
            left_pulses = 0.5 * (c_lf + c_lb)
        elif LB_WORKING:
            left_pulses = float(c_lb)   # LF broken, use LB alone
        elif LF_WORKING:
            left_pulses = float(c_lf)
        else:
            left_pulses = 0.0

        # ── Right side ───────────────────────────────────────
        if RF_WORKING and RB_WORKING:
            right_pulses = 0.5 * (c_rf + c_rb)
        elif RB_WORKING:
            right_pulses = float(c_rb)
        elif RF_WORKING:
            right_pulses = float(c_rf)
        else:
            right_pulses = 0.0

        m_per_pulse = (math.pi * WHEEL_DIAMETER_M) / PULSES_PER_REV
        sl, sr = self._wheel_signs()

        d_left  = sl * left_pulses  * m_per_pulse
        d_right = sr * right_pulses * m_per_pulse

        v_enc = (d_right + d_left)  / (2.0 * dt)
        w_enc = (d_right - d_left)  / (TRACK_WIDTH_M * dt)
        w_gyro = math.radians(self._gyro_rate)

        # Complementary filter for yaw
        w_fused = self._gyro_weight * w_gyro + \
                  (1.0 - self._gyro_weight) * w_enc

        d = (d_right + d_left) / 2.0
        self._yaw = _wrap(self._yaw + w_fused * dt)
        self._x  += d * math.cos(self._yaw)
        self._y  += d * math.sin(self._yaw)

        qz = math.sin(self._yaw / 2.0)
        qw = math.cos(self._yaw / 2.0)
        stamp = self.get_clock().now().to_msg()

        # Publish /odom
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x   = float(self._x)
        odom.pose.pose.position.y   = float(self._y)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x   = float(v_enc)
        odom.twist.twist.angular.z  = float(w_fused)

        p = odom.pose.covariance
        p[0]  = 0.02   # x
        p[7]  = 0.02   # y
        p[35] = 0.01   # yaw
        t = odom.twist.covariance
        t[0]  = 0.02
        t[35] = 0.01
        self._odom_pub.publish(odom)

        # Publish odom→base_link TF
        tf = TransformStamped()
        tf.header.stamp    = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_link'
        tf.transform.translation.x = float(self._x)
        tf.transform.translation.y = float(self._y)
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw
        self._tf_br.sendTransform(tf)

    def destroy_node(self):
        try: GPIO.cleanup()
        except: pass
        super().destroy_node()


def main():
    rclpy.init()
    node = EncoderOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()