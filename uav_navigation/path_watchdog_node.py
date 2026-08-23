#!/usr/bin/env python3
"""
path_watchdog_node.py

Continuously monitors that cells on the active path remain free in the
obstacle layer. If an obstacle is detected on the path, publishes a
WatchdogAlert so the exploration_manager can trigger replanning.

Subscribes:
  /current_path   (nav_msgs/Path)
  /obstacle_grid  (nav_msgs/OccupancyGrid)

Publishes:
  /watchdog_alert (uav_navigation_msgs/msg/WatchdogAlert)

Author: Dorian Veloso
Capstone B - UTS 2026
Supervisor: Dr. Graeme Best
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path, OccupancyGrid
from uav_navigation_msgs.msg import WatchdogAlert
import numpy as np


class PathWatchdogNode(Node):

    def __init__(self):
        super().__init__('path_watchdog_node')

        # --- Grid parameters (must match terrain_scoring_node) ---
        self.resolution = 0.5
        self.grid_size = 200
        self.grid_origin_x = -50.0
        self.grid_origin_y = -50.0

        # --- Parameters ---
        # Check rate (Hz). Higher = more reactive, more CPU.
        self.check_rate = 5.0

        # --- State ---
        self.current_path = None    # list of (x, y) world coordinates
        self.obstacle_grid = None   # (grid_size, grid_size) int8
        self.last_alert_blocked = False

        # --- QoS ---
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Subscribers ---
        self.path_sub = self.create_subscription(
            Path, '/current_path',
            self.path_callback, reliable_qos
        )
        self.obstacle_sub = self.create_subscription(
            OccupancyGrid, '/obstacle_grid',
            self.obstacle_callback, reliable_qos
        )

        # --- Publisher ---
        self.alert_pub = self.create_publisher(
            WatchdogAlert, '/watchdog_alert', reliable_qos
        )

        # --- Check timer ---
        self.create_timer(1.0 / self.check_rate, self.check_path)

        self.get_logger().info(
            f'PathWatchdogNode started | check_rate={self.check_rate}Hz'
        )

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def path_callback(self, msg: Path):
        """Store active path as list of (x, y) world coordinates."""
        self.current_path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]

    def obstacle_callback(self, msg: OccupancyGrid):
        """Store obstacle grid as numpy array."""
        self.obstacle_grid = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

    # -------------------------------------------------------------------------
    # Watchdog check
    # -------------------------------------------------------------------------

    def check_path(self):
        """Check all path cells against obstacle grid and publish alert."""
        if self.current_path is None or self.obstacle_grid is None:
            return

        if len(self.current_path) == 0:
            return

        first_blocked_index = -1
        obstacle_x = 0.0
        obstacle_y = 0.0
        blocked_count = 0

        for idx, (wx, wy) in enumerate(self.current_path):
            gi = int((wx - self.grid_origin_x) / self.resolution)
            gj = int((wy - self.grid_origin_y) / self.resolution)

            # Skip out-of-bounds waypoints
            if not (0 <= gi < self.grid_size and 0 <= gj < self.grid_size):
                continue

            if self.obstacle_grid[gj, gi] == 100:
                blocked_count += 1
                if first_blocked_index == -1:
                    first_blocked_index = idx
                    obstacle_x = wx
                    obstacle_y = wy

        path_blocked = blocked_count > 0

        # Only log on state change to avoid spam
        if path_blocked and not self.last_alert_blocked:
            self.get_logger().warn(
                f'Path blocked: {blocked_count} cells | '
                f'first at index {first_blocked_index} '
                f'({obstacle_x:.1f}, {obstacle_y:.1f})'
            )
        elif not path_blocked and self.last_alert_blocked:
            self.get_logger().info('Path cleared')

        self.last_alert_blocked = path_blocked

        # Always publish so manager has current state
        alert = WatchdogAlert()
        alert.path_blocked = path_blocked
        alert.first_blocked_index = first_blocked_index if path_blocked else -1
        alert.obstacle_x = obstacle_x
        alert.obstacle_y = obstacle_y
        alert.blocked_cell_count = blocked_count
        self.alert_pub.publish(alert)


def main(args=None):
    rclpy.init(args=args)
    node = PathWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
