#!/usr/bin/env python3
"""
explore-grid.py  v2 — DFS with lidar snapshot + drift-aware backtracking

Key fixes for L-shape miss:
  1. OBSTACLE_DIST raised to 0.30m (was 0.25 — too tight for drifted robot)
  2. Pre-turn blocking threshold = 60% of OBSTACLE_DIST (hard walls only)
     Borderline readings must be verified by turning to face direction
  3. Post-turn is authoritative — only marks tried AFTER turning
  4. 0.5s settle after backtrack so lidar gets fresh reading
  5. Uses /odom for gyro-confirmed turns when available
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist

import math, time, struct, os, threading, json
import numpy as np

# ══════════════════════════════════════════════════════════════
# CALIBRATION
# ══════════════════════════════════════════════════════════════
DRIVE_SPEED    = 0.50
TURN_SPEED     = 1.20
DRIVE_TIME     = 0.80   # s — ~31cm
TURN_90_TIME   = 0.585  # s — ~90deg timed fallback
BACKUP_TIME    = 0.55   # s
BACKUP_SPEED   = 0.35   # m/s

# ══════════════════════════════════════════════════════════════
# LIDAR
# ══════════════════════════════════════════════════════════════
LIDAR_OFFSET   = 180    # lidar 0deg points backward
OBSTACLE_DIST  = 0.30   # m — raised from 0.25 (robot drifts off-center)
HARD_BLOCK     = 0.18   # m — only skip without turning if < this
FRONT_DEG      = 22     # +/- degrees

# Relative lidar angles for each facing (from robot's current front)
FACING_REL_DEG = {0: 0, 1: -90, 2: 180, 3: 90}

# ══════════════════════════════════════════════════════════════
# GYRO via /odom
# ══════════════════════════════════════════════════════════════
USE_GYRO       = True    # set False if encoder_odom not running
TURN_TOL_RAD   = 0.08    # ~5deg — stop turn when this close to target
COAST_RAD      = 0.07    # ~4deg — stop motors this early, coast the rest
TURN_TIMEOUT   = 3.0     # s

# ══════════════════════════════════════════════════════════════
# DFS
# ══════════════════════════════════════════════════════════════
FACING_NAME    = {0:"N", 1:"E", 2:"S", 3:"W"}
FACING_DELTA   = {0:(0,1), 1:(1,0), 2:(0,-1), 3:(-1,0)}
PATH_FILE      = "/root/NAMI/dfs_path.json"
MAP_SAVE_PATH  = "/root/NAMI/apartment_map"
# ══════════════════════════════════════════════════════════════


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _save_map(path, grid, info):
    if grid is None or info is None:
        print("No map data"); return
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    h, w = info.height, info.width
    pgm  = path + ".pgm"
    with open(pgm, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        for row in range(h-1, -1, -1):
            for col in range(w):
                v = int(grid[row, col])
                if   v == -1: px = 205
                elif v ==  0: px = 254
                else:         px = max(0, int(254-v/100.0*254))
                f.write(struct.pack("B", px))
    with open(path+".yaml","w") as f:
        f.write(f"image: {os.path.basename(pgm)}\n"
                f"resolution: {info.resolution}\n"
                f"origin: [{info.origin.position.x},"
                f"{info.origin.position.y},0.0]\n"
                f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    print(f"Map saved: {pgm}")


class Explorer(Node):

    def __init__(self):
        super().__init__('explorer')

        scan_qos = QoSProfile(depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        map_qos = QoSProfile(depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(LaserScan,     '/scan', self._scan_cb,  scan_qos)
        self.create_subscription(OccupancyGrid, '/map',  self._map_cb,   map_qos)
        self.create_subscription(Odometry,      '/odom', self._odom_cb,
            QoSProfile(depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE))
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.ranges     = None
        self.angle_min  = 0.0
        self.angle_inc  = 0.0
        self.map_grid   = None
        self.map_info   = None
        self._odom_yaw  = None   # from /odom for gyro-confirmed turns

        self._cell      = (0, 0)
        self._facing    = 0
        self._stack     = [(0, 0)]
        self._tried     = {(0, 0): set()}
        self._path_log  = []

        self.get_logger().info("Explorer ready — waiting for lidar...")
        threading.Thread(target=self._run, daemon=True).start()

    def _scan_cb(self, msg):
        self.ranges    = list(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment

    def _map_cb(self, msg):
        try:
            self.map_info = msg.info
            self.map_grid = np.array(msg.data, dtype=np.int8).reshape(
                (msg.info.height, msg.info.width))
        except Exception:
            pass

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self._odom_yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z))

    def _run(self):
        while self.ranges is None:
            time.sleep(0.1)
        self.get_logger().info(
            f"Lidar ready — front={self._front():.2f}m  "
            f"gyro={'yes' if self._odom_yaw is not None else 'no'}")
        self.get_logger().info("Starting DFS...")
        while rclpy.ok():
            self._step()

    # ── DFS step ─────────────────────────────────────────────

    def _step(self):
        cx, cy = self._cell
        tried  = self._tried.setdefault((cx, cy), set())

        # Read all 4 directions simultaneously — no movement needed
        snapshot = {}
        for f in range(4):
            rel = FACING_REL_DEG[f]
            snapshot[f] = self._sector_min(rel - FRONT_DEG, rel + FRONT_DEG)

        self.get_logger().info(
            f"CELL ({cx},{cy}) facing={FACING_NAME[self._facing]} "
            f"stack={len(self._stack)} "
            f"tried={[FACING_NAME[f] for f in tried]}")
        self.get_logger().info(
            "  LIDAR: " + "  ".join(
                f"{FACING_NAME[f]}:{snapshot[f]:.2f}m" for f in range(4)))

        # Hard-block only clearly solid walls (below 60% of threshold)
        # Borderline must be verified by turning — drift makes pre-turn unreliable
        for f in range(4):
            if f not in tried and snapshot[f] < HARD_BLOCK:
                tried.add(f)
                self.get_logger().info(
                    f"  {FACING_NAME[f]}: solid wall({snapshot[f]:.2f}m) — skip")

        # Try untried directions, most open first
        untried = [f for f in range(4) if f not in tried]
        untried.sort(key=lambda f: snapshot[f], reverse=True)

        for target_f in untried:
            # Turn first — post-turn reading is authoritative
            turned = self._turn_to(target_f)
            time.sleep(0.3)

            dist_now = self._front()
            tried.add(target_f)  # mark tried AFTER turning, not before

            self.get_logger().info(
                f"  {FACING_NAME[target_f]}: "
                f"pre={snapshot[target_f]:.2f}m "
                f"post-turn={dist_now:.2f}m "
                f"(turned {turned:+.1f}deg)")

            if dist_now < OBSTACLE_DIST:
                self.get_logger().info(f"  Blocked — skip")
                continue

            moved = self._drive_forward()
            if moved:
                dc, dr = FACING_DELTA[target_f]
                self._cell = (cx + dc, cy + dr)
                self._stack.append(self._cell)
                self._tried.setdefault(self._cell, set())
                self._path_log.append({
                    "cell": list(self._cell),
                    "facing": target_f
                })
                self.get_logger().info(f"  Moved to {self._cell}")
                return
            else:
                self.get_logger().warn(f"  Obstacle mid-drive — try next")
                continue

        # All directions exhausted — backtrack
        if len(self._stack) <= 1:
            self.get_logger().info("DFS complete!")
            self.stop()
            self._save_path()
            _save_map(MAP_SAVE_PATH, self.map_grid, self.map_info)
            rclpy.shutdown()
            return

        self._stack.pop()
        prev = self._stack[-1]
        self.get_logger().info(
            f"BACKTRACK ({cx},{cy}) -> {prev}")
        self._go_to(prev)
        self._cell = prev
        # Wait for robot to fully stop + lidar to settle
        # before reading directions at this cell
        time.sleep(0.5)

    # ── Motion ───────────────────────────────────────────────

    def _turn_to(self, target_facing) -> float:
        steps = (target_facing - self._facing) % 4
        if steps == 0:
            return 0.0

        if steps == 1:    deg, direction = +90.0, +1
        elif steps == 3:  deg, direction = -90.0, -1
        else:             deg, direction = +180.0, +1

        if USE_GYRO and self._odom_yaw is not None:
            actual = self._gyro_turn(math.radians(deg), direction)
        else:
            duration = (abs(deg) / 90.0) * TURN_90_TIME
            t = time.time()
            while time.time() - t < duration:
                self.move(0.0, TURN_SPEED * direction)
                time.sleep(0.05)
            self.stop()
            actual = deg

        self._facing = target_facing
        return actual

    def _gyro_turn(self, target_rad: float, direction: int) -> float:
        """Turn using /odom yaw feedback. Returns actual degrees turned."""
        start_yaw   = self._odom_yaw
        accumulated = 0.0
        prev_yaw    = self._odom_yaw
        t_start     = time.time()
        target_abs  = abs(target_rad) - COAST_RAD

        self.move(0.0, TURN_SPEED * direction)

        while accumulated < target_abs:
            if time.time() - t_start > TURN_TIMEOUT:
                break
            time.sleep(0.01)
            if self._odom_yaw is None:
                continue
            dyaw = _wrap(self._odom_yaw - prev_yaw)
            accumulated += direction * dyaw
            prev_yaw = self._odom_yaw

        self.stop()
        actual_rad = abs(_wrap(self._odom_yaw - start_yaw))
        return math.degrees(actual_rad) * direction

    def _drive_forward(self) -> bool:
        t = time.time()
        while time.time() - t < DRIVE_TIME:
            if self._front() < OBSTACLE_DIST:
                self.get_logger().warn(
                    f"  Obstacle {self._front():.2f}m — backing up")
                self.stop()
                time.sleep(0.1)
                tb = time.time()
                while time.time() - tb < BACKUP_TIME:
                    self.move(-BACKUP_SPEED, 0.0)
                    time.sleep(0.05)
                self.stop()
                time.sleep(0.2)
                return False
            self.move(DRIVE_SPEED, 0.0)
            time.sleep(0.05)
        self.stop()
        return True

    def _go_to(self, target_cell):
        """Backtrack to adjacent cell."""
        cx, cy = self._cell
        tx, ty = target_cell
        dc, dr = tx - cx, ty - cy

        target_f = None
        for f, (d1, d2) in FACING_DELTA.items():
            if d1 == dc and d2 == dr:
                target_f = f; break

        if target_f is None:
            self.get_logger().warn(f"Backtrack: not adjacent!")
            return

        self._turn_to(target_f)
        time.sleep(0.4)

        # Drive at 70% speed — less overshoot = less drift
        t = time.time()
        while time.time() - t < DRIVE_TIME:
            self.move(DRIVE_SPEED * 0.7, 0.0)
            time.sleep(0.05)
        self.stop()
        time.sleep(0.4)

    # ── Lidar ────────────────────────────────────────────────

    def _front(self) -> float:
        return self._sector_min(-FRONT_DEG, FRONT_DEG)

    def _sector_min(self, a_deg, b_deg) -> float:
        if not self.ranges:
            return float('inf')
        a = a_deg + LIDAR_OFFSET
        b = b_deg + LIDAR_OFFSET
        n    = len(self.ranges)
        inc  = self.angle_inc
        amin = self.angle_min
        i0   = int(round((math.radians(a) - amin) / inc)) % n
        i1   = int(round((math.radians(b) - amin) / inc)) % n
        if i0 <= i1:
            idx = range(i0, i1+1)
        else:
            idx = list(range(i0, n)) + list(range(0, i1+1))
        vals = [self.ranges[i] for i in idx
                if math.isfinite(self.ranges[i]) and self.ranges[i] > 0.05]
        return min(vals) if vals else float('inf')

    def _save_path(self):
        try:
            with open(PATH_FILE, "w") as f:
                json.dump(self._path_log, f, indent=2)
            self.get_logger().info(
                f"Path saved ({len(self._path_log)} steps)")
        except Exception as e:
            self.get_logger().warn(f"Path save failed: {e}")

    def move(self, lin, ang):
        msg = Twist()
        msg.linear.x  = float(lin)
        msg.angular.z = float(ang)
        self._pub.publish(msg)

    def stop(self):
        self.move(0.0, 0.0)


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
            time.sleep(0.2)
        except Exception:
            pass
        try:
            with open(PATH_FILE, "w") as f:
                json.dump(node._path_log, f, indent=2)
            print(f"Path saved ({len(node._path_log)} steps)")
        except Exception:
            pass
        print("Saving map...")
        _save_map(MAP_SAVE_PATH, node.map_grid, node.map_info)
        try: node.destroy_node()
        except: pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()