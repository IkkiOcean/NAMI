root@ikkiocean:~/NAMI# python3 explore_dfs.py
[INFO] [1773596327.249418054] [explorer]: Explorer ready — waiting for sensors...
[INFO] [1773596327.352592298] [explorer]: Sensors ready. Front=2.24m. Starting DFS...
[INFO] [1773596327.354850810] [explorer]: CELL (0,0) facing=N stack=1 tried=[]
[INFO] [1773596327.356850859] [explorer]:   LIDAR: N=CLEAR(2.24m)  E=CLEAR(0.33m)  S=CLEAR(0.39m)  W=CLEAR(0.69m)
[INFO] [1773596327.358760409] [explorer]:   Chose N (2.24m clear) — turning
[INFO] [1773596327.663183434] [explorer]:   Post-turn front=2.24m (turned +0.0deg)
[INFO] [1773596328.514594732] [explorer]:   Moved to (0, 1)
[INFO] [1773596328.516670466] [explorer]: CELL (0,1) facing=N stack=2 tried=[]
[INFO] [1773596328.518599886] [explorer]:   LIDAR: N=CLEAR(1.99m)  E=CLEAR(0.57m)  S=CLEAR(0.49m)  W=CLEAR(0.71m)
[INFO] [1773596328.523763371] [explorer]:   Chose N (1.99m clear) — turning
[INFO] [1773596328.825959624] [explorer]:   Post-turn front=1.90m (turned +0.0deg)
[INFO] [1773596329.650586614] [explorer]:   Moved to (0, 2)
[INFO] [1773596329.653563790] [explorer]: CELL (0,2) facing=N stack=3 tried=[]
[INFO] [1773596329.655694820] [explorer]:   LIDAR: N=CLEAR(1.65m)  E=CLEAR(0.58m)  S=CLEAR(0.67m)  W=CLEAR(0.72m)
[INFO] [1773596329.657890647] [explorer]:   Chose N (1.65m clear) — turning
[INFO] [1773596329.962808318] [explorer]:   Post-turn front=1.56m (turned +0.0deg)
[INFO] [1773596330.779572794] [explorer]:   Moved to (0, 3)
[INFO] [1773596330.784191854] [explorer]: CELL (0,3) facing=N stack=4 tried=[]
[INFO] [1773596330.790415576] [explorer]:   LIDAR: N=CLEAR(1.33m)  E=CLEAR(0.58m)  S=CLEAR(0.98m)  W=CLEAR(0.73m)
[INFO] [1773596330.792831180] [explorer]:   Chose N (1.33m clear) — turning
[INFO] [1773596331.095494043] [explorer]:   Post-turn front=1.23m (turned +0.0deg)
[INFO] [1773596331.935890261] [explorer]:   Moved to (0, 4)
[INFO] [1773596331.940125619] [explorer]: CELL (0,4) facing=N stack=5 tried=[]
[INFO] [1773596331.950725716] [explorer]:   LIDAR: N=CLEAR(0.99m)  E=CLEAR(0.57m)  S=CLEAR(1.33m)  W=CLEAR(0.74m)
[INFO] [1773596331.952658951] [explorer]:   Chose S (1.33m clear) — turning





#!/usr/bin/env python3
"""
DFS Explorer — Lidar-aware + Gyro-confirmed turns via /odom topic

Key principle: lidar is 360 degrees.
  BEFORE moving anywhere, read all 4 directions simultaneously.
  Only attempt a direction if lidar already shows it's clear.
  No more "turn then discover wall" — we know before we turn.

Gyro via /odom topic (from odom_publisher.py):
  DON'T run odom_publisher.py and this simultaneously if both
  try to read MPU hardware. Instead, subscribe to /odom which
  already has gyro yaw integrated.

  For turn confirmation: record yaw before turn, keep turning
  until yaw changes by target degrees.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import math, time, struct, os, threading, json
import numpy as np

# ══════════════════════════════════════════════════════════════
# TUNING
# ══════════════════════════════════════════════════════════════
DRIVE_SPEED    = 0.50   # m/s
BACK_SPEED     = 0.35   # m/s (backtrack slower = less drift)
TURN_SPEED     = 1.00   # rad/s
BACKUP_SPEED   = 0.40   # m/s
DRIVE_TIME     = 0.80   # s per cell forward
BACKUP_TIME    = 0.55   # s

# ══════════════════════════════════════════════════════════════
# LIDAR — 360 degree awareness
# ══════════════════════════════════════════════════════════════
LIDAR_OFFSET   = 180    # your lidar 0deg points backward
OBSTACLE_DIST  = 0.30   # m — block direction if closer than this
FRONT_DEG      = 22     # +/- degrees per direction check

# How many degrees offset each facing is from robot front (N=0):
#   N = 0deg (straight ahead)
#   E = 90deg right = -90 relative
#   S = 180deg behind
#   W = 90deg left = +90 relative
# These are RELATIVE to robot's current facing
FACING_REL_DEG = {0: 0, 1: -90, 2: 180, 3: 90}
FACING_NAME    = {0:"N", 1:"E", 2:"S", 3:"W"}
FACING_DELTA   = {0:(0,1), 1:(1,0), 2:(0,-1), 3:(-1,0)}

# ══════════════════════════════════════════════════════════════
# GYRO — from /odom topic published by odom_publisher.py
# ══════════════════════════════════════════════════════════════
# Set USE_GYRO=True if odom_publisher.py is running
# Set USE_GYRO=False to use timed turns (less accurate)
USE_GYRO       = True
TURN_90_TIME   = 0.585  # s fallback if gyro not available
# How close to target yaw counts as "done" (degrees)
GYRO_TOLERANCE = 3.0
# Coast: stop this many degrees early, robot coasts the rest
COAST_DEG      = 4.0

# ══════════════════════════════════════════════════════════════
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
        odom_qos = QoSProfile(depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(LaserScan,     '/scan', self._scan_cb,  scan_qos)
        self.create_subscription(OccupancyGrid, '/map',  self._map_cb,   map_qos)
        self.create_subscription(Odometry,      '/odom', self._odom_cb,  odom_qos)
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.ranges     = None
        self.angle_min  = 0.0
        self.angle_inc  = 0.0
        self.map_grid   = None
        self.map_info   = None
        self._yaw       = None   # current yaw from /odom (radians)

        self._cell      = (0, 0)
        self._facing    = 0
        self._stack     = [(0, 0)]
        self._tried     = {(0, 0): set()}
        self._path_log  = []

        self.get_logger().info("Explorer ready — waiting for sensors...")
        threading.Thread(target=self._run, daemon=True).start()

    # ── Callbacks ────────────────────────────────────────────

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
        self._yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z))

    # ── Main DFS loop ────────────────────────────────────────

    def _run(self):
        while self.ranges is None:
            time.sleep(0.1)
        if USE_GYRO:
            while self._yaw is None:
                time.sleep(0.1)
                self.get_logger().info(
                    "Waiting for /odom (start odom_publisher.py)...",
                    throttle_duration_sec=2.0)
        self.get_logger().info(
            f"Sensors ready. Front={self._front():.2f}m. Starting DFS...")
        while rclpy.ok():
            self._step()

    def _step(self):
        cx, cy = self._cell
        tried  = self._tried.setdefault((cx, cy), set())

        # ── LIDAR SNAPSHOT — read ALL 4 directions at once ───
        # This is the key: we know what's in every direction
        # BEFORE making any physical movement decision
        snapshot = self._lidar_snapshot()

        # Log the full picture
        self.get_logger().info(
            f"CELL ({cx},{cy}) facing={FACING_NAME[self._facing]} "
            f"stack={len(self._stack)} "
            f"tried={[FACING_NAME[f] for f in tried]}")
        self.get_logger().info(
            "  LIDAR: " +
            "  ".join(
                f"{FACING_NAME[f]}={'CLEAR' if snapshot[f]>=OBSTACLE_DIST else 'WALL '}"
                f"({snapshot[f]:.2f}m)"
                for f in range(4)
            )
        )

        # ── DFS: pick best untried direction ─────────────────
        # Among untried directions, prefer ones with most clearance
        untried_clear = [
            f for f in range(4)
            if f not in tried and snapshot[f] >= OBSTACLE_DIST
        ]
        untried_blocked = [
            f for f in range(4)
            if f not in tried and snapshot[f] < OBSTACLE_DIST
        ]

        # Mark blocked ones as tried immediately (no point attempting)
        for f in untried_blocked:
            tried.add(f)
            self.get_logger().info(
                f"  {FACING_NAME[f]}: WALL at {snapshot[f]:.2f}m "
                f"— skipped (no turn needed)")

        if untried_clear:
            # Sort by distance — try most open direction first
            untried_clear.sort(key=lambda f: snapshot[f], reverse=True)
            target_f = untried_clear[0]
            tried.add(target_f)

            self.get_logger().info(
                f"  Chose {FACING_NAME[target_f]} "
                f"({snapshot[target_f]:.2f}m clear) — turning")

            # Turn to face it
            turned = self._turn_to(target_f)
            time.sleep(0.3)

            # Final check after turning (map may have updated)
            dist_now = self._front()
            self.get_logger().info(
                f"  Post-turn front={dist_now:.2f}m "
                f"(turned {turned:+.1f}deg)")

            if dist_now < OBSTACLE_DIST:
                self.get_logger().warn(
                    f"  Obstacle appeared post-turn — skip")
                return   # re-enter _step, same cell, will try next direction

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
                self.get_logger().info(
                    f"  Moved to {self._cell}")
            else:
                self.get_logger().warn(
                    f"  Obstacle mid-drive — will retry other directions")
            return

        # ── All clear directions exhausted — backtrack ───────
        if len(self._stack) <= 1:
            self.get_logger().info("DFS complete — all reachable cells visited!")
            self.stop()
            self._save_path()
            _save_map(MAP_SAVE_PATH, self.map_grid, self.map_info)
            rclpy.shutdown()
            return

        self._stack.pop()
        prev = self._stack[-1]
        self.get_logger().info(
            f"BACKTRACK ({cx},{cy}) -> {prev}  "
            f"(all directions exhausted: "
            f"tried={[FACING_NAME[f] for f in tried]})")
        self._go_to(prev)
        self._cell = prev
        time.sleep(0.5)   # full stop + lidar settle before next step

    # ── Lidar snapshot — read all 4 directions at once ───────

    def _lidar_snapshot(self) -> dict:
        """
        Read lidar in all 4 compass directions simultaneously.
        Returns {facing: distance_metres} for all 4 facings.
        No physical movement required.
        """
        result = {}
        for f in range(4):
            rel_deg = FACING_REL_DEG[f]
            result[f] = self._sector_min(
                rel_deg - FRONT_DEG,
                rel_deg + FRONT_DEG)
        return result

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

    # ── Gyro-confirmed turn via /odom yaw ────────────────────

    def _turn_to(self, target_facing) -> float:
        """
        Turn to face target_facing.
        If USE_GYRO=True: uses /odom yaw to confirm actual rotation.
        If USE_GYRO=False: falls back to timed turn.
        Returns actual degrees turned.
        """
        steps = (target_facing - self._facing) % 4
        if steps == 0:
            return 0.0

        if steps == 1:    deg = +90.0   # left
        elif steps == 3:  deg = -90.0   # right
        else:             deg = +180.0  # 180

        direction = 1.0 if deg > 0 else -1.0

        if USE_GYRO and self._yaw is not None:
            actual = self._gyro_turn(deg)
        else:
            # Timed fallback
            duration = (abs(deg) / 90.0) * TURN_90_TIME
            t = time.time()
            while time.time() - t < duration:
                self.move(0.0, TURN_SPEED * direction)
                time.sleep(0.05)
            self.stop()
            actual = deg

        self._facing = target_facing
        return actual

    def _gyro_turn(self, target_deg: float) -> float:
        """
        Turn using /odom yaw for feedback.
        target_deg: + = left (CCW), - = right (CW)
        """
        if self._yaw is None:
            return 0.0

        direction    = 1.0 if target_deg > 0 else -1.0
        target_abs   = abs(target_deg)
        effective    = target_abs - COAST_DEG

        start_yaw    = self._yaw
        accumulated  = 0.0
        prev_yaw     = self._yaw

        self.move(0.0, TURN_SPEED * direction)

        while accumulated < effective:
            time.sleep(0.01)
            if self._yaw is None:
                continue
            dyaw = _wrap(self._yaw - prev_yaw)
            accumulated += direction * dyaw
            prev_yaw = self._yaw

        self.stop()

        # Measure actual turn
        actual = abs(_wrap(self._yaw - start_yaw))
        return actual * direction

    # ── Drive + backtrack ────────────────────────────────────

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
        """Backtrack to adjacent cell. Slower + gyro-confirmed turn."""
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
                f"Backtrack: {target_cell} not adjacent!")
            return

        turned = self._turn_to(target_f)
        time.sleep(0.4)

        # Slower backtrack speed = less drift
        t = time.time()
        while time.time() - t < DRIVE_TIME:
            self.move(BACK_SPEED, 0.0)
            time.sleep(0.05)
        self.stop()
        time.sleep(0.4)

    # ── Save + utils ─────────────────────────────────────────

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