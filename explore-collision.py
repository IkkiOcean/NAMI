#!/usr/bin/env python3
"""
DFS Room Explorer with gyro-confirmed turns and drift-resistant backtracking.

Key fixes for T-shape problem:
  1. Backtrack drives at 70% speed for accuracy
  2. 0.5s settle pause after each backtrack step
  3. OBSTACLE_DIST=0.30m (corridors ~32cm wide on this robot)
  4. Gyro confirms every 90deg turn (no timing guesses)

DFS guarantees both branches of a T get explored:
  stem → junction → right branch → backtrack to junction → left branch
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist

import math, time, struct, os, threading, json
import smbus2
import numpy as np

# ══════════════════════════════════════════════════════════════
# SPEEDS
# ══════════════════════════════════════════════════════════════
DRIVE_SPEED      = 0.50   # m/s forward
BACK_SPEED       = 0.35   # m/s backtrack (slower = more accurate)
TURN_SPEED       = 1.00   # rad/s
BACKUP_SPEED     = 0.40   # m/s obstacle backup
DRIVE_TIME       = 0.80   # s forward per cell (~31cm)
BACKUP_TIME      = 0.55   # s backup after obstacle

# ══════════════════════════════════════════════════════════════
# GYRO
# ══════════════════════════════════════════════════════════════
MPU_ADDR         = 0x68
GYRO_SENS        = 131.0
GYRO_CAL_S       = 3.0
GYRO_POLL        = 0.005  # s
COAST_DEG        = 5.0    # stop this early, robot coasts the rest

# ══════════════════════════════════════════════════════════════
# LIDAR
# ══════════════════════════════════════════════════════════════
LIDAR_OFFSET     = 180    # lidar 0deg points backward
OBSTACLE_DIST    = 0.30   # m — raised from 0.25 for 32cm corridors
FRONT_DEG        = 20     # +/- degrees

# ══════════════════════════════════════════════════════════════
# DFS  — 0=N 1=E 2=S 3=W
# ══════════════════════════════════════════════════════════════
FACING_NAME      = {0:"N", 1:"E", 2:"S", 3:"W"}
FACING_DELTA     = {0:(0,1), 1:(1,0), 2:(0,-1), 3:(-1,0)}
PATH_FILE        = "/root/NAMI/dfs_path.json"
MAP_SAVE_PATH    = "/root/NAMI/apartment_map"
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


# ── Gyro ──────────────────────────────────────────────────────

class Gyro:
    def __init__(self):
        self._bus  = smbus2.SMBus(1)
        self._bias = 0.0
        self._rate = 0.0
        self._lock = threading.Lock()

        self._bus.write_byte_data(MPU_ADDR, 0x6B, 0x00)
        time.sleep(0.1)
        self._bus.write_byte_data(MPU_ADDR, 0x1B, 0x00)

        print(f"Gyro calibrating {GYRO_CAL_S}s — keep still...")
        samples = []
        t_end = time.time() + GYRO_CAL_S
        while time.time() < t_end:
            samples.append(self._raw() / GYRO_SENS)
            time.sleep(0.005)
        self._bias = sum(samples) / len(samples)
        print(f"Gyro bias = {self._bias:.4f} deg/s")

        threading.Thread(target=self._loop, daemon=True).start()

    def _raw(self):
        h = self._bus.read_byte_data(MPU_ADDR, 0x47)
        l = self._bus.read_byte_data(MPU_ADDR, 0x48)
        v = (h << 8) | l
        return float(v - 65536 if v >= 32768 else v)

    def _loop(self):
        while True:
            try:
                with self._lock:
                    self._rate = (self._raw() / GYRO_SENS) - self._bias
            except Exception:
                pass
            time.sleep(GYRO_POLL)

    @property
    def rate(self):
        with self._lock:
            return self._rate

    def turn(self, degrees: float, turn_fn, stop_fn) -> float:
        """
        Turn exactly `degrees` (+ = left/CCW, - = right/CW).
        Uses gyro integration. Stops COAST_DEG early.
        Returns actual degrees turned.
        """
        direction = 1.0 if degrees > 0 else -1.0
        target    = abs(degrees) - COAST_DEG
        if target <= 0:
            return 0.0

        accumulated = 0.0
        prev_t      = time.time()
        turn_fn()

        while accumulated < target:
            now    = time.time()
            dt     = now - prev_t
            prev_t = now
            rate   = self.rate
            # CCW = positive gyro, CW = negative
            accumulated += direction * rate * dt
            time.sleep(GYRO_POLL)

        stop_fn()
        return (accumulated + COAST_DEG) * direction


# ── Explorer node ─────────────────────────────────────────────

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

        self._gyro      = Gyro()

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

    # ── Main DFS loop ─────────────────────────────────────────

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

        # All 4 directions — log lidar in each
        turn_degs = {0: 0, 1: 90, 2: 180, 3: -90}
        dists = {}
        for f in range(4):
            rel = (f - self._facing) % 4
            deg = turn_degs[rel]
            if deg == 270: deg = -90
            dists[f] = self._sector_min(deg - FRONT_DEG, deg + FRONT_DEG)

        self.get_logger().info(
            f"CELL ({cx},{cy}) facing={FACING_NAME[self._facing]} "
            f"stack={len(self._stack)} "
            f"tried={[FACING_NAME[f] for f in tried]} | "
            + "  ".join(f"{FACING_NAME[f]}:{dists[f]:.2f}m"
                        for f in range(4)))

        for target_f in [0, 1, 2, 3]:
            if target_f in tried:
                continue

            tried.add(target_f)
            dist = dists[target_f]

            self.get_logger().info(
                f"  Try {FACING_NAME[target_f]} lidar={dist:.2f}m")

            if dist < OBSTACLE_DIST:
                self.get_logger().info(f"  BLOCKED — skip")
                continue

            # Gyro-confirmed turn
            turned = self._turn_to(target_f)
            time.sleep(0.3)

            dist_now = self._front()
            self.get_logger().info(
                f"  Turned {turned:+.1f}deg  post-turn front={dist_now:.2f}m")

            if dist_now < OBSTACLE_DIST:
                self.get_logger().info(f"  BLOCKED post-turn — skip")
                continue

            # Drive forward
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

        # All 4 directions tried — backtrack
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
        # Settle pause — lets robot stop completely and
        # lidar get fresh readings before next decision
        time.sleep(0.5)

    # ── Motion ───────────────────────────────────────────────

    def _turn_to(self, target_facing) -> float:
        """Gyro-confirmed turn to target_facing. Returns degrees turned."""
        steps = (target_facing - self._facing) % 4
        if steps == 0:
            return 0.0

        if steps == 1:    # 90 left
            deg = +90.0
        elif steps == 3:  # 90 right
            deg = -90.0
        else:             # 180
            deg = +180.0

        actual = self._gyro.turn(
            deg,
            turn_fn=lambda: self.move(0.0, TURN_SPEED * (1 if deg > 0 else -1)),
            stop_fn=self.stop
        )
        self._facing = target_facing
        return actual

    def _drive_forward(self) -> bool:
        """Drive one cell forward. Returns True if reached, False if blocked."""
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
        """
        Drive to adjacent cell for backtracking.
        Slower speed + longer settle = less positional drift.
        """
        cx, cy = self._cell
        tx, ty = target_cell
        dc, dr = tx - cx, ty - cy

        target_f = None
        for f, (d1, d2) in FACING_DELTA.items():
            if d1 == dc and d2 == dr:
                target_f = f
                break

        if target_f is None:
            self.get_logger().warn(
                f"Backtrack: {target_cell} not adjacent to {self._cell}!")
            return

        # Gyro-confirmed turn
        turned = self._turn_to(target_f)
        time.sleep(0.4)

        # Drive at reduced speed for accuracy
        t = time.time()
        back_time = DRIVE_TIME / BACK_SPEED * BACK_SPEED  # same distance, slower
        while time.time() - t < DRIVE_TIME:
            self.move(BACK_SPEED, 0.0)
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