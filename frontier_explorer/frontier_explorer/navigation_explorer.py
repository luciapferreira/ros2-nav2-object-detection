#!/usr/bin/env python3
import sys
import math
import time
import random
import subprocess
from collections import deque
from enum import Enum
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import quaternion_from_euler

# --- Parameters ---
OCC_THRESHOLD = 10
MIN_FRONTIER_SIZE = 5
SAFE_ZONE = 4
PROGRESS_TIMEOUT = 5.0     # seconds before declaring "stuck"
MIN_PROGRESS = 0.5        # meters moved before considering progress
RECOVERY_RADIUS = 0.5      # radius around robot for recovery goal search
MIN_GOAL_SEPARATION = 0.1  # meters
MAP_SAVE_RETRIES = 3
# ------------------

class OccupancyGrid2d:
    class CostValues(Enum):
        FreeSpace = 0
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map_msg: OccupancyGrid):
        self.map = map_msg

    def get_cost(self, mx, my):
        return self.map.data[my * self.map.info.width + mx]

    def map_to_world(self, mx, my) -> Tuple[float, float]:
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution
        return wx, wy

    def world_to_map(self, wx, wy) -> Tuple[int, int]:
        ox, oy = self.map.info.origin.position.x, self.map.info.origin.position.y
        if wx < ox or wy < oy:
            raise ValueError("World coordinates out of bounds")
        mx = int((wx - ox) / self.map.info.resolution)
        my = int((wy - oy) / self.map.info.resolution)
        if mx >= self.map.info.width or my >= self.map.info.height:
            raise ValueError("World coordinates out of bounds")
        return mx, my

    def size_x(self): return self.map.info.width
    def size_y(self): return self.map.info.height

class FrontierPoint:
    def __init__(self, x, y):
        self.mapX = x
        self.mapY = y
        self.classification = 0

class FrontierCache:
    def __init__(self): self.cache = {}
    def get_point(self, x, y): 
        key = (x, y)
        if key not in self.cache: self.cache[key] = FrontierPoint(x, y)
        return self.cache[key]
    def clear(self): self.cache.clear()

class PointClassification(Enum):
    MapOpen = 1
    MapClosed = 2
    FrontierOpen = 4
    FrontierClosed = 8

def centroid(points: List[Tuple[float, float]]) -> Tuple[float, float]:
    arr = np.array(points)
    return float(arr[:, 0].mean()), float(arr[:, 1].mean())

def get_neighbors(pt: FrontierPoint, grid: OccupancyGrid2d, cache: FrontierCache):
    for dx in range(-1, 2):
        for dy in range(-1, 2):
            nx, ny = pt.mapX + dx, pt.mapY + dy
            if 0 <= nx < grid.size_x() and 0 <= ny < grid.size_y():
                yield cache.get_point(nx, ny)

def is_frontier(pt: FrontierPoint, grid: OccupancyGrid2d, cache: FrontierCache) -> bool:
    if grid.get_cost(pt.mapX, pt.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
        return False
    has_free = False
    for n in get_neighbors(pt, grid, cache):
        cost = grid.get_cost(n.mapX, n.mapY)
        if cost > OCC_THRESHOLD:
            return False
        if cost == OccupancyGrid2d.CostValues.FreeSpace.value:
            has_free = True
    return has_free

def find_free(mx, my, grid: OccupancyGrid2d):
    cache = FrontierCache()
    bfs = deque([cache.get_point(mx, my)])
    while bfs:
        loc = bfs.popleft()
        if grid.get_cost(loc.mapX, loc.mapY) == 0:
            return loc.mapX, loc.mapY
        for n in get_neighbors(loc, grid, cache):
            if n.classification == 0:
                n.classification = 1
                bfs.append(n)
    return mx, my

def detect_frontiers(pose, grid: OccupancyGrid2d):
    cache = FrontierCache()
    mx, my = grid.world_to_map(pose.position.x, pose.position.y)
    free_x, free_y = find_free(mx, my, grid)
    start = cache.get_point(free_x, free_y)
    start.classification = PointClassification.MapOpen.value
    map_queue = deque([start])
    frontiers = []

    while map_queue:
        p = map_queue.popleft()
        if p.classification & PointClassification.MapClosed.value:
            continue
        if is_frontier(p, grid, cache):
            p.classification |= PointClassification.FrontierOpen.value
            frontier_queue = deque([p])
            new_frontier = []
            while frontier_queue:
                q = frontier_queue.popleft()
                if q.classification & (PointClassification.MapClosed.value | PointClassification.FrontierClosed.value):
                    continue
                if is_frontier(q, grid, cache):
                    new_frontier.append(q)
                    for w in get_neighbors(q, grid, cache):
                        if not w.classification:
                            w.classification = PointClassification.FrontierOpen.value
                            frontier_queue.append(w)
                q.classification |= PointClassification.FrontierClosed.value
            if len(new_frontier) >= MIN_FRONTIER_SIZE:
                points_world = [grid.map_to_world(f.mapX, f.mapY) for f in new_frontier]
                frontiers.append(centroid(points_world))
        for v in get_neighbors(p, grid, cache):
            if not v.classification:
                v.classification = PointClassification.MapOpen.value
                map_queue.append(v)
        p.classification |= PointClassification.MapClosed.value
    return frontiers

def is_goal_safe(goal_xy, grid: OccupancyGrid2d, safe_distance=SAFE_ZONE, occ_threshold=OCC_THRESHOLD):
    try:
        mx, my = grid.world_to_map(goal_xy[0], goal_xy[1])
    except ValueError:
        return False
    for dx in range(-safe_distance, safe_distance + 1):
        for dy in range(-safe_distance, safe_distance + 1):
            nx, ny = mx + dx, my + dy
            if 0 <= nx < grid.size_x() and 0 <= ny < grid.size_y():
                if grid.get_cost(nx, ny) > occ_threshold:
                    return False
    return True

class WaypointExplorer(Node):
    def __init__(self):
        super().__init__('waypoint_explorer')
        self.map = None
        self.current_pose = None
        self.failed_frontiers = []
        self.last_goal = None

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info("Waiting for NavigateToPose server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Explorer ready.")

    def map_callback(self, msg): self.map = OccupancyGrid2d(msg)
    def odom_callback(self, msg): self.current_pose = msg.pose.pose

    def publish_markers(self, frontiers):
        array = MarkerArray()
        for i, (x, y) in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "frontiers"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.r = 1.0; m.color.a = 1.0
            array.markers.append(m)
        self.marker_pub.publish(array)

    def save_map_cli(self, retries=MAP_SAVE_RETRIES):
        for i in range(retries):
            try:
                subprocess.run(
                    ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", "auto_map", "-t", "/map"],
                    check=True
                )
                self.get_logger().info("Map saved as auto_map.")
                return True
            except subprocess.CalledProcessError:
                self.get_logger().warn(f"Map save failed (attempt {i+1}/{retries}), retrying...")
                time.sleep(2)
        self.get_logger().error("Failed to save map after retries.")
        return False

    def find_recovery_goal(self, start_pos):
        if not self.map: return None
        for _ in range(50):
            angle = random.uniform(0, 2 * math.pi)
            dist = random.uniform(0.3, RECOVERY_RADIUS)
            x = start_pos[0] + dist * math.cos(angle)
            y = start_pos[1] + dist * math.sin(angle)
            try:
                mx, my = self.map.world_to_map(x, y)
                cost = self.map.get_cost(mx, my)
                if cost == 0 and is_goal_safe((x, y), self.map, safe_distance=2):
                    return (x, y)
            except ValueError:
                continue
        return None

    def move_to_frontiers(self):
        tried_relaxed_search = False  # Track if we did the "remove all constraints" attempt

        while rclpy.ok():
            if not self.map or not self.current_pose:
                rclpy.spin_once(self, timeout_sec=0.2)
                continue

            # --- Detect frontiers with normal constraints ---
            frontiers = detect_frontiers(self.current_pose, self.map)
            frontiers = [
                f for f in frontiers
                if f not in self.failed_frontiers
                and is_goal_safe(f, self.map)
                and (self.last_goal is None or math.hypot(f[0] - self.last_goal[0], f[1] - self.last_goal[1]) > MIN_GOAL_SEPARATION)
            ]

            # --- First-time relaxed search if no frontiers found ---
            if not frontiers and not tried_relaxed_search:
                self.get_logger().info("No frontiers found. Trying one-time relaxed search without constraints...")
                tried_relaxed_search = True

                frontiers = detect_frontiers(self.current_pose, self.map)
                frontiers = [
                    f for f in frontiers
                    if f not in self.failed_frontiers
                    and is_goal_safe(f, self.map, safe_distance=0)
                ]

                if not frontiers:
                    self.get_logger().info("Still no frontiers even with relaxed search. Saving map and shutting down...")
                    self.save_map_cli()
                    break
                else:
                    self.get_logger().info(f"Relaxed frontier found: {frontiers[0]}")

            elif not frontiers:
                # Normal shutdown path
                self.get_logger().info("No more frontiers, saving map...")
                self.save_map_cli()
                break

            # --- Pick nearest frontier ---
            target = min(frontiers, key=lambda f: math.hypot(
                f[0] - self.current_pose.position.x,
                f[1] - self.current_pose.position.y
            ))
            self.last_goal = target
            self.publish_markers(frontiers)

            # --- Send goal ---
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.pose.position.x = target[0]
            goal_msg.pose.pose.position.y = target[1]
            yaw = math.atan2(target[1] - self.current_pose.position.y, target[0] - self.current_pose.position.x)
            qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
            goal_msg.pose.pose.orientation.x, goal_msg.pose.pose.orientation.y = qx, qy
            goal_msg.pose.pose.orientation.z, goal_msg.pose.pose.orientation.w = qz, qw

            send_future = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f"Goal {target} rejected.")
                self.failed_frontiers.append(target)
                continue

            self.get_logger().info(f"Sent goal {target}")
            start_pos = (self.current_pose.position.x, self.current_pose.position.y)
            start_time = time.time()
            result_future = goal_handle.get_result_async()
            stuck_recovery_done = False

            # --- Wait for goal completion ---
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.2)
                if result_future.done():
                    break

                now = time.time()
                dist_moved = math.hypot(
                    self.current_pose.position.x - start_pos[0],
                    self.current_pose.position.y - start_pos[1]
                )

                # --- Check if the current window has passed ---
                if now - start_time > PROGRESS_TIMEOUT:
                    if not stuck_recovery_done and dist_moved < MIN_PROGRESS:
                        self.get_logger().warn("Robot seems stuck. Attempting recovery goal...")
                        recovery_goal = self.find_recovery_goal(start_pos)
                        if recovery_goal:
                            self.get_logger().info(f"Recovery goal chosen: {recovery_goal}")
                            cancel_future = goal_handle.cancel_goal_async()
                            rclpy.spin_until_future_complete(self, cancel_future)
                            self.send_recovery_goal(recovery_goal)
                            stuck_recovery_done = True
                        else:
                            self.get_logger().warn("Could not find recovery goal.")
                            stuck_recovery_done = True

                    start_pos = (self.current_pose.position.x, self.current_pose.position.y)
                    start_time = now

            rclpy.spin_once(self, timeout_sec=0.1)

    def send_recovery_goal(self, goal_xy):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = goal_xy[0]
        goal.pose.pose.position.y = goal_xy[1]
        yaw = 0.0
        qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
        goal.pose.pose.orientation.x, goal.pose.pose.orientation.y = qx, qy
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = qz, qw

        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().warn("Recovery goal rejected.")
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info("Recovery goal attempt finished.")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointExplorer()
    try:
        node.move_to_frontiers()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down explorer.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
