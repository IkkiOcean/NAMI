#!/usr/bin/env python3
"""
replay.py — Replays a previously explored path from explore_dfs.py

On first run: explore_dfs.py saves path to /root/NAMI/dfs_path.json
On replay:    replay.py loads that file and drives the same path again

Path file format:
  [{"cell": [cx,cy], "facing": f}, ...]
  — one entry per cell visited, in order
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist

import math, time, struct, os, threading, json
import numpy as np

# ── Same calibration as explore_dfs.py ──────────────────────
DRIVE_SPEED   = 0.50
TURN_SPEED    = 1.20
DRIVE_TIME    = 0.80
TURN_90_TIME  = 0.585
OBSTACLE_DIST = 0.25
FRONT_DEG     = 20
LIDAR_OFFSET  = 180

FACING_DELTA  = {0:(0,1), 1:(1,0), 2:(0,-1), 3:(-1,0)}
FACING_NAME   = {0:"N", 1:"E", 2:"S", 3:"W"}

PATH_FILE     = "/root/NAMI/dfs_path.json"
MAP_SAVE_PATH = "/root/NAMI/apartment_map"
# ────────────────────────────────────────────────────────────


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
                else:         px = max(0, int(254 - v/100.0*254))
                f.write(struct.pack("B", px))
    with open(path+".yaml","w") as f:
        f.write(f"image: {os.path.basename(pgm)}\n"
                f"resolution: {info.resolution}\n"
                f"origin: [{info.origin.position.x},"
                f"{info.origin.position.y},0.0]\n"
                f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    print(f"Map saved: {pgm}")


class Replayer(Node):

    def __init__(self, path_data):
        super().__init__('replayer')

        map_qos = QoSProfile(depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        scan_qos = QoSProfile(depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(LaserScan, '/scan', self._scan_cb, scan_qos)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.ranges    = None
        self.angle_min = 0.0
        self.angle_inc = 0.0
        self.map_grid  = None
        self.map_info  = None

        self._path     = path_data   # list of {cell, facing}
        self._facing   = 0           # current robot facing

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
            f"Replaying {len(self._path)} steps...")

        for i, step in enumerate(self._path):
            cx, cy   = step["cell"]
            target_f = step["facing"]

            self.get_logger().info(
                f"Step {i+1}/{len(self._path)}: "
                f"cell=({cx},{cy}) facing={FACING_NAME[target_f]} "
                f"front={self._front():.2f}m")

            # Check for unexpected obstacle
            if self._front() < OBSTACLE_DIST:
                self.get_logger().warn(
                    f"Unexpected obstacle {self._front():.2f}m — stopping")
                self.stop()
                break

            # Turn to required facing
            self._turn_to(target_f)
            time.sleep(0.3)

            # Drive one cell
            t = time.time()
            while time.time() - t < DRIVE_TIME:
                if self._front() < OBSTACLE_DIST:
                    self.get_logger().warn("Obstacle during replay — stopping")
                    self.stop()
                    _save_map(MAP_SAVE_PATH, self.map_grid, self.map_info)
                    return
                self.move(DRIVE_SPEED, 0.0)
                time.sleep(0.05)
            self.stop()

        self.get_logger().info("Replay complete!")
        _save_map(MAP_SAVE_PATH, self.map_grid, self.map_info)
        rclpy.shutdown()

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

    def _front(self):
        return self._sector_min(-FRONT_DEG, FRONT_DEG)

    def _sector_min(self, a_deg, b_deg):
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

    def move(self, lin, ang):
        msg = Twist()
        msg.linear.x  = float(lin)
        msg.angular.z = float(ang)
        self._pub.publish(msg)

    def stop(self):
        self.move(0.0, 0.0)


def main():
    if not os.path.exists(PATH_FILE):
        print(f"No path file found at {PATH_FILE}")
        print("Run explore_dfs.py first to generate the path.")
        return

    with open(PATH_FILE) as f:
        path_data = json.load(f)

    print(f"Loaded {len(path_data)} steps from {PATH_FILE}")

    rclpy.init()
    node = Replayer(path_data)
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
        print("Saving map...")
        _save_map(MAP_SAVE_PATH, node.map_grid, node.map_info)
        try: node.destroy_node()
        except: pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
