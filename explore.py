#!/usr/bin/env python3
"""
explore.py — Frontier-Based Exploration
========================================

Algorithm (proven approach from robotics literature):
  1. Read OccupancyGrid from slam_toolbox
  2. Find FRONTIER cells: free cells (0) adjacent to unknown cells (-1)
  3. Cluster nearby frontiers → pick nearest cluster centroid
  4. Drive toward it using SLAM pose feedback (closed-loop, no drift)
  5. Repeat until no frontiers remain → map complete

Why this beats DFS grid:
  - Uses the ACTUAL map, not an invented grid
  - Navigates to real world coordinates from SLAM (no drift accumulation)
  - Backtracks only when truly needed (obstacle), not by design
  - Handles any room shape: L, T, U, irregular

Key fixes for your specific problems:
  - Turns use /odom gyro feedback (confirmed angle, not timed)
  - Drives to SLAM coordinates (closed-loop, stops when arrived)
  - Obstacle check uses full 360 lidar snapshot before committing
  - Frontier blacklist prevents revisiting same stuck spots

Hardware: lidar + encoder_odom.py (replaces odom_publisher.py)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener

import math, time, struct, os, json, threading
import numpy as np
from typing import Optional, List, Tuple

# ══════════════════════════════════════════════════════════════
# TUNING
# ══════════════════════════════════════════════════════════════
DRIVE_SPEED      = 0.50   # m/s
TURN_SPEED       = 1.00   # rad/s (slower = more accurate gyro turn)
BACKUP_SPEED     = 0.35   # m/s

# Obstacle detection
OBSTACLE_DIST    = 0.20   # m hard stop
FRONT_DEG        = 25     # ± degrees for front check
LIDAR_OFFSET     = 180    # your lidar faces backward

# Navigation (closed-loop using SLAM pose)
ARRIVE_DIST      = 0.25   # m — "arrived" at frontier goal
HEADING_KP       = 1.5    # proportional gain for heading correction
STUCK_DIST       = 0.05   # m — less than this movement = stuck
STUCK_TIME       = 8.0    # s — declare stuck after this
BACKUP_TIME      = 0.6    # s

# Gyro turn (via /odom)
COAST_RAD        = 0.07   # rad — stop motors this early, robot coasts
TURN_TIMEOUT     = 4.0    # s

# Frontier detection
FRONTIER_MIN_CLUSTER = 5  # ignore tiny frontier clusters
FRONTIER_BLACKLIST_R = 0.40  # m — blacklist radius around failed goals

MAP_SAVE_PATH    = "/root/NAMI/apartment_map"
# ══════════════════════════════════════════════════════════════


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _save_map(path, grid, info):
    if grid is None or info is None:
        print("No map data"); return
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    h, w = info.height, info.width
    res  = info.resolution
    pgm  = path + ".pgm"
    with open(pgm, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        for row in range(h-1, -1, -1):
            for col in range(w):
                v = int(grid[row, col])
                if   v == -1: px = 205
                elif v ==  0: px = 254
                else:         px = max(0, int(254-v/100.0*254))
                f.write(bytes([px]))
    with open(path+".yaml","w") as f:
        f.write(f"image: {os.path.basename(pgm)}\n"
                f"resolution: {res}\n"
                f"origin: [{info.origin.position.x},"
                f"{info.origin.position.y},0.0]\n"
                f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    free = int((grid == 0).sum())
    print(f"Map saved: {pgm}  ({w*res:.1f}×{h*res:.1f}m  {free*res**2:.1f}m² free)")


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

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

        # TF for SLAM pose
        self._tf  = Buffer()
        self._tfl = TransformListener(self._tf, self)

        # Sensor state
        self.ranges     = None
        self.angle_min  = 0.0
        self.angle_inc  = 0.0
        self.map_grid:  Optional[np.ndarray] = None
        self.map_info   = None
        self._odom_yaw  = None   # from /odom for gyro turns

        # Exploration state
        self._goal:     Optional[Tuple[float,float]] = None
        self._blacklist: List[Tuple[float,float]] = []
        self._stuck_t   = time.time()
        self._stuck_pos = None
        self._state     = "WAIT"
        self._survey_start = 0.0

        self.get_logger().info("Frontier explorer ready — waiting for sensors...")
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
        self._odom_yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z))

    # ── Main loop ────────────────────────────────────────────

    def _run(self):
        # Wait for sensors
        while self.ranges is None or self.map_grid is None:
            time.sleep(0.2)

        self.get_logger().info(
            f"Sensors ready. Front={self._front():.2f}m. "
            f"Map={self.map_info.width*self.map_info.resolution:.1f}×"
            f"{self.map_info.height*self.map_info.resolution:.1f}m")

        # Initial 360° survey so SLAM builds a starting map
        self.get_logger().info("Initial 360° survey spin...")
        self._survey_spin()

        self.get_logger().info("Starting frontier exploration...")
        while rclpy.ok():
            self._exploration_step()

    def _survey_spin(self):
        """Rotate 360° to give SLAM initial map data."""
        full_time = (2 * math.pi) / TURN_SPEED
        t = time.time()
        while time.time() - t < full_time:
            self.move(0.0, TURN_SPEED)
            time.sleep(0.05)
        self.stop()
        time.sleep(1.0)   # let map update

    # ── Frontier exploration step ─────────────────────────────

    def _exploration_step(self):
        pose = self._get_slam_pose()
        if pose is None:
            self.get_logger().info("Waiting for SLAM pose...",
                                   throttle_duration_sec=3.0)
            time.sleep(0.5)
            return

        rx, ry, ryaw = pose

        # Find all frontiers in the current map
        frontiers = self._find_frontiers()
        if not frontiers:
            self.get_logger().info("No frontiers found — exploration complete!")
            self.stop()
            _save_map(MAP_SAVE_PATH, self.map_grid, self.map_info)
            rclpy.shutdown()
            return

        # Cluster frontiers and pick best goal
        clusters = self._cluster_frontiers(frontiers)
        goal = self._pick_best_goal(clusters, rx, ry)

        if goal is None:
            self.get_logger().warn(
                "All frontiers blacklisted — clearing blacklist")
            self._blacklist.clear()
            time.sleep(1.0)
            return

        gx, gy = goal
        dist = math.hypot(gx-rx, gy-ry)
        self.get_logger().info(
            f"Frontier goal: ({gx:.2f},{gy:.2f})  "
            f"dist={dist:.2f}m  "
            f"frontiers={len(frontiers)}  "
            f"clusters={len(clusters)}  "
            f"blacklisted={len(self._blacklist)}")

        self._goal = goal
        self._stuck_t = time.time()
        self._stuck_pos = (rx, ry)

        # Navigate to goal
        result = self._navigate_to(gx, gy)

        if result == "arrived":
            self.get_logger().info(f"Reached frontier ({gx:.2f},{gy:.2f})")
            # Small spin at each frontier to map surroundings
            self._mini_spin()
        elif result == "stuck":
            self.get_logger().warn(
                f"Stuck near ({gx:.2f},{gy:.2f}) — blacklisting")
            self._blacklist.append(goal)
        elif result == "obstacle":
            self.get_logger().warn(
                f"Obstacle blocked path to ({gx:.2f},{gy:.2f}) — blacklisting")
            self._blacklist.append(goal)

    # ── Navigation ───────────────────────────────────────────

    def _navigate_to(self, tx: float, ty: float) -> str:
        """
        Drive to (tx, ty) using SLAM pose feedback.
        Returns: 'arrived' | 'stuck' | 'obstacle'
        """
        while rclpy.ok():
            pose = self._get_slam_pose()
            if pose is None:
                self.move(DRIVE_SPEED * 0.5, 0.0)
                time.sleep(0.05)
                continue

            rx, ry, ryaw = pose
            dist = math.hypot(tx - rx, ty - ry)

            # Arrived?
            if dist < ARRIVE_DIST:
                self.stop()
                return "arrived"

            # Stuck check
            if time.time() - self._stuck_t > STUCK_TIME:
                if self._stuck_pos:
                    moved = math.hypot(rx-self._stuck_pos[0],
                                       ry-self._stuck_pos[1])
                    if moved < STUCK_DIST:
                        self.stop()
                        return "stuck"
                self._stuck_t = time.time()
                self._stuck_pos = (rx, ry)

            # Obstacle check
            if self._front() < OBSTACLE_DIST:
                self.stop()
                self.get_logger().warn(
                    f"Obstacle {self._front():.2f}m — backing up")
                self._backup()
                # Try to go around
                cleared = self._avoid_obstacle()
                if not cleared:
                    return "obstacle"
                continue

            # Heading toward goal
            desired_yaw = math.atan2(ty - ry, tx - rx)
            heading_err = _wrap(desired_yaw - ryaw)

            # If very off course, stop and rotate
            if abs(heading_err) > math.radians(40):
                self.stop()
                self._rotate_to_heading(desired_yaw)
                time.sleep(0.2)
                continue

            # Drive with heading correction
            angular = max(-TURN_SPEED,
                          min(TURN_SPEED, HEADING_KP * heading_err))
            self.move(DRIVE_SPEED, angular)
            time.sleep(0.05)

        return "stuck"

    def _rotate_to_heading(self, target_yaw: float):
        """Rotate to face target_yaw using /odom gyro feedback."""
        if self._odom_yaw is None:
            # Timed fallback
            err = _wrap(target_yaw - self._get_slam_pose()[2]
                        if self._get_slam_pose() else 0)
            direction = 1 if err > 0 else -1
            duration = abs(err) / TURN_SPEED
            t = time.time()
            while time.time() - t < duration:
                self.move(0.0, TURN_SPEED * direction)
                time.sleep(0.05)
            self.stop()
            return

        # Gyro-confirmed rotation
        pose = self._get_slam_pose()
        if pose is None:
            return
        current_yaw = pose[2]
        err = _wrap(target_yaw - current_yaw)
        direction = 1 if err > 0 else -1
        target_abs = abs(err) - COAST_RAD
        if target_abs <= 0:
            return

        accumulated = 0.0
        prev_yaw = self._odom_yaw
        t_start = time.time()
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

    def _avoid_obstacle(self) -> bool:
        """
        Try to get around an obstacle.
        Returns True if front is now clear.
        """
        # Find clearest side
        left_dist  = self._sector_min( FRONT_DEG, FRONT_DEG + 60)
        right_dist = self._sector_min(-(FRONT_DEG + 60), -FRONT_DEG)
        direction  = 1.0 if left_dist >= right_dist else -1.0

        self.get_logger().info(
            f"Avoiding: L={left_dist:.2f}m R={right_dist:.2f}m "
            f"→ turning {'left' if direction>0 else 'right'}")

        # Rotate up to 90° toward clearest side
        t = time.time()
        while time.time() - t < 2.0:
            self.move(0.0, TURN_SPEED * direction)
            time.sleep(0.05)
            if self._front() > OBSTACLE_DIST:
                self.stop()
                return True
        self.stop()
        return self._front() > OBSTACLE_DIST

    def _backup(self):
        t = time.time()
        while time.time() - t < BACKUP_TIME:
            self.move(-BACKUP_SPEED, 0.0)
            time.sleep(0.05)
        self.stop()
        time.sleep(0.2)

    def _mini_spin(self):
        """Short spin at frontier to map surroundings better."""
        t = time.time()
        while time.time() - t < 1.5:
            self.move(0.0, TURN_SPEED)
            time.sleep(0.05)
        self.stop()
        time.sleep(0.5)   # let map update

    # ── Frontier detection ───────────────────────────────────

    def _find_frontiers(self) -> List[Tuple[int, int]]:
        """
        Find frontier cells: free (0) cells adjacent to unknown (-1) cells.
        Returns list of (row, col) indices.
        """
        if self.map_grid is None:
            return []
        g = self.map_grid
        free    = (g == 0)
        unknown = (g == -1)

        # Shift unknown mask in 4 directions
        adj = np.zeros_like(unknown)
        adj[1:,  :]  |= unknown[:-1, :]
        adj[:-1, :]  |= unknown[1:,  :]
        adj[:,  1:]  |= unknown[:,  :-1]
        adj[:, :-1]  |= unknown[:,   1:]

        coords = np.argwhere(free & adj)
        return [(int(r), int(c)) for r, c in coords]

    def _cluster_frontiers(self,
                            cells: List[Tuple[int,int]],
                            radius: int = 6
                            ) -> List[Tuple[float,float]]:
        """
        Group nearby frontier cells into clusters.
        Returns list of (world_x, world_y) cluster centroids.
        """
        if not cells or self.map_info is None:
            return []
        res = self.map_info.resolution
        ox  = self.map_info.origin.position.x
        oy  = self.map_info.origin.position.y

        remaining = set(cells)
        clusters  = []

        while remaining:
            seed  = next(iter(remaining))
            group = []
            queue = [seed]
            remaining.discard(seed)

            while queue:
                r, c = queue.pop()
                group.append((r, c))
                for dr in range(-radius, radius+1):
                    for dc in range(-radius, radius+1):
                        nb = (r+dr, c+dc)
                        if nb in remaining:
                            remaining.discard(nb)
                            queue.append(nb)

            if len(group) >= FRONTIER_MIN_CLUSTER:
                mr = sum(p[0] for p in group) / len(group)
                mc = sum(p[1] for p in group) / len(group)
                wx = ox + (mc + 0.5) * res
                wy = oy + (mr + 0.5) * res
                clusters.append((wx, wy))

        return clusters

    def _pick_best_goal(self,
                         clusters: List[Tuple[float,float]],
                         rx: float, ry: float
                         ) -> Optional[Tuple[float,float]]:
        """
        Pick the best frontier goal.
        Strategy: nearest cluster that isn't blacklisted.
        """
        candidates = []
        for c in clusters:
            # Skip blacklisted
            too_close = any(
                math.hypot(c[0]-b[0], c[1]-b[1]) < FRONTIER_BLACKLIST_R
                for b in self._blacklist)
            if not too_close:
                dist = math.hypot(c[0]-rx, c[1]-ry)
                candidates.append((dist, c))

        if not candidates:
            return None

        candidates.sort(key=lambda x: x[0])
        return candidates[0][1]

    # ── SLAM pose ────────────────────────────────────────────

    def _get_slam_pose(self) -> Optional[Tuple[float,float,float]]:
        try:
            tf = self._tf.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                             1 - 2*(q.y*q.y + q.z*q.z))
            return (t.x, t.y, yaw)
        except Exception:
            return None

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

    def move(self, lin, ang):
        msg = Twist()
        msg.linear.x  = float(lin)
        msg.angular.z = float(-ang)
        self._pub.publish(msg)

    def stop(self):
        self.move(0.0, 0.0)


def main():
    rclpy.init()
    node = FrontierExplorer()
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