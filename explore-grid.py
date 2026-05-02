#!/usr/bin/env python3
"""
DFS Room Explorer — 4 directions (N/E/S/W)
This is the version that successfully generated the first map.

Calibrated for this robot:
  - 31cm per step in 0.80s at 0.5m/s
  - 90deg turn in 0.585s at 1.2rad/s
  - Lidar 0deg points backward: LIDAR_OFFSET=180
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist

import math, time, struct, os, threading, json
import numpy as np

# ══════════════════════════════════════════════════════════════
# CALIBRATION
# ══════════════════════════════════════════════════════════════
DRIVE_SPEED    = 0.50
TURN_SPEED     = 1.20
DRIVE_TIME     = 0.80   # s — moves ~31cm
TURN_90_TIME   = 0.585  # s — turns ~90deg
BACKUP_TIME    = 0.50   # s

# ══════════════════════════════════════════════════════════════
# LIDAR
# ══════════════════════════════════════════════════════════════
LIDAR_OFFSET   = 180    # lidar 0deg points backward
OBSTACLE_DIST  = 0.25   # m
FRONT_DEG      = 20     # +/- degrees

# ══════════════════════════════════════════════════════════════
# DFS — 4 directions
# 0=N 1=E 2=S 3=W
# ══════════════════════════════════════════════════════════════
FACING_NAME  = {0:"N", 1:"E", 2:"S", 3:"W"}
FACING_DELTA = {0:(0,1), 1:(1,0), 2:(0,-1), 3:(-1,0)}

PATH_FILE     = "/root/NAMI/dfs_path.json"
MAP_SAVE_PATH = "/root/NAMI/apartment_map"
# ══════════════════════════════════════════════════════════════


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

        self.create_subscription(LaserScan, '/scan', self._scan_cb, scan_qos)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.ranges     = None
        self.angle_min  = 0.0
        self.angle_inc  = 0.0
        self.map_grid   = None
        self.map_info   = None

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

    def _run(self):
        while self.ranges is None:
            time.sleep(0.1)
        self.get_logger().info(
            f"Lidar ready — front={self._front():.2f}m")
        self.get_logger().info("Starting DFS...")
        while rclpy.ok():
            self._step()

    def _step(self):
        cx, cy = self._cell
        tried  = self._tried.setdefault((cx, cy), set())

        self.get_logger().info(
            f"CELL ({cx},{cy}) facing={FACING_NAME[self._facing]} "
            f"tried={[FACING_NAME[f] for f in tried]} "
            f"stack={len(self._stack)} front={self._front():.2f}m")

        # Try all 4 directions in order N->E->S->W
        for target_facing in [0, 1, 2, 3]:
            if target_facing in tried:
                continue

            tried.add(target_facing)

            # Angle to check relative to current facing
            turn_steps = (target_facing - self._facing) % 4
            turn_deg = turn_steps * 90
            if turn_deg == 270:
                turn_deg = -90

            dist = self._sector_min(turn_deg - FRONT_DEG, turn_deg + FRONT_DEG)

            self.get_logger().info(
                f"  Try {FACING_NAME[target_facing]} "
                f"(turn {turn_deg:+d}deg) lidar={dist:.2f}m")

            if dist < OBSTACLE_DIST:
                self.get_logger().info(f"  BLOCKED — skip")
                continue

            self._turn_to(target_facing)
            time.sleep(0.3)

            dist_now = self._front()
            self.get_logger().info(f"  Post-turn front={dist_now:.2f}m")

            if dist_now < OBSTACLE_DIST:
                self.get_logger().info(f"  BLOCKED post-turn — skip")
                continue

            moved = self._drive_forward()
            if moved:
                dc, dr = FACING_DELTA[target_facing]
                self._cell = (cx + dc, cy + dr)
                self._stack.append(self._cell)
                self._tried.setdefault(self._cell, set())
                self._path_log.append({
                    "cell": list(self._cell),
                    "facing": target_facing
                })
                self.get_logger().info(f"  Moved to {self._cell}")
                return
            else:
                self.get_logger().warn(f"  Obstacle mid-drive — try next")
                continue

        # All 4 tried — backtrack
        if len(self._stack) <= 1:
            self.get_logger().info("DFS complete!")
            self.stop()
            self._save_path()
            _save_map(MAP_SAVE_PATH, self.map_grid, self.map_info)
            rclpy.shutdown()
            return

        self._stack.pop()
        prev = self._stack[-1]
        self.get_logger().info(f"BACKTRACK -> {prev}")
        self._go_to(prev)
        self._cell = prev

    def _turn_to(self, target_facing):
        steps = (target_facing - self._facing) % 4
        if steps == 0:
            return
        if steps == 1:
            self._do_turn(+1, TURN_90_TIME)
        elif steps == 3:
            self._do_turn(-1, TURN_90_TIME)
        elif steps == 2:
            self._do_turn(+1, TURN_90_TIME * 2)
        self._facing = target_facing

    def _do_turn(self, direction, duration):
        t = time.time()
        while time.time() - t < duration:
            self.move(0.0, TURN_SPEED * direction)
            time.sleep(0.05)
        self.stop()

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
                    self.move(-DRIVE_SPEED, 0.0)
                    time.sleep(0.05)
                self.stop()
                time.sleep(0.2)
                return False
            self.move(DRIVE_SPEED, 0.0)
            time.sleep(0.05)
        self.stop()
        return True

    def _go_to(self, target_cell):
        cx, cy = self._cell
        tx, ty = target_cell
        dc, dr = tx - cx, ty - cy

        target_facing = None
        for f, (d1, d2) in FACING_DELTA.items():
            if d1 == dc and d2 == dr:
                target_facing = f
                break

        if target_facing is None:
            self.get_logger().warn(f"Backtrack: {target_cell} not adjacent!")
            return

        self._turn_to(target_facing)
        time.sleep(0.3)

        t = time.time()
        while time.time() - t < DRIVE_TIME:
            self.move(DRIVE_SPEED, 0.0)
            time.sleep(0.05)
        self.stop()
        time.sleep(0.3)

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
                f"Path saved: {PATH_FILE} ({len(self._path_log)} steps)")
        except Exception as e:
            self.get_logger().warn(f"Could not save path: {e}")

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
            print(f"Path saved: {PATH_FILE} ({len(node._path_log)} steps)")
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