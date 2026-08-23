#!/usr/bin/env python3
"""
terrain_follower_node.py

Terrain following via downward LiDAR rays (-90 to 0 deg elevation).
Measures ground height directly below the drone in real time and publishes
a target NED altitude so the drone maintains a constant height above ground.

The waypoint_node consumes /cruise_altitude and injects it into TrajectorySetpoint.

Subscribes:
  /lidar/points                          (sensor_msgs/PointCloud2)
  /fmu/out/vehicle_local_position_v1     (px4_msgs/VehicleLocalPosition)

Publishes:
  /cruise_altitude  (std_msgs/Float32)   - target NED z (negative = above ground)

Author: Dorian Veloso
Capstone B - UTS 2026
Supervisor: Dr. Graeme Best
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
import numpy as np
import math


class TerrainFollowerNode(Node):

    def __init__(self):
        super().__init__('terrain_follower_node')

        # --- Parameters ---
        # Target height above ground (metres, positive = above)
        self.target_height_m = 8.0

        # Only downward rays used: elevation angle strictly below this threshold.
        # Elevation angle = arctan2(z_sensor, sqrt(x^2 + y^2))
        # 0 deg = horizontal, -90 deg = straight down.
        self.max_elevation_deg = 0.0

        # Cone radius around nadir (metres in XY sensor frame).
        # Only points within this radius are considered "directly below".
        # At 8m altitude, a 2m radius cone = ~14 deg half-angle, well within
        # the downward FOV and avoids slanted near-ground returns.
        self.nadir_radius_m = 2.0

        # Low-pass filter coefficient for ground height estimate.
        # 0.0 = no filtering (raw), 1.0 = never updates.
        # 0.85 gives ~6 scan smoothing at 8 Hz, fast enough for terrain following.
        self.filter_alpha = 0.85

        # Publish rate (Hz)
        self.publish_rate = 10.0

        # Safety limits: if ground estimate is outside these bounds,
        # ignore the measurement (sensor noise or out-of-range return).
        self.ground_z_min = -5.0   # world frame (below sea level tolerated)
        self.ground_z_max = 20.0   # world frame

        # --- State ---
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0        # NED: negative when airborne
        self.drone_heading = 0.0
        self.position_valid = False

        # Filtered ground height estimate (world frame, metres above origin)
        self.ground_z_estimate = None

        # Current cruise altitude target (NED frame, negative = above ground)
        self.cruise_altitude_ned = None

        # --- QoS ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Subscribers ---
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.pointcloud_callback,
            sensor_qos
        )

        try:
            from px4_msgs.msg import VehicleLocalPosition
            self.pos_sub = self.create_subscription(
                VehicleLocalPosition,
                '/fmu/out/vehicle_local_position_v1',
                self.position_callback,
                sensor_qos
            )
            self.get_logger().info('Subscribed to VehicleLocalPosition')
        except ImportError:
            self.get_logger().warn('px4_msgs not available')
            self.pos_sub = None

        # --- Publisher ---
        self.altitude_pub = self.create_publisher(
            Float32, '/cruise_altitude', reliable_qos
        )

        # --- Publish timer ---
        self.create_timer(1.0 / self.publish_rate, self.publish)

        self.scan_count = 0
        self.get_logger().info(
            f'TerrainFollowerNode started | '
            f'target height {self.target_height_m}m AGL | '
            f'nadir cone radius {self.nadir_radius_m}m | '
            f'filter alpha {self.filter_alpha}'
        )

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def position_callback(self, msg):
        """Store drone pose from PX4 VehicleLocalPosition (NED)."""
        if msg.xy_valid and msg.z_valid:
            self.drone_x = msg.x
            self.drone_y = msg.y
            self.drone_z = msg.z
            self.drone_heading = msg.heading
            self.position_valid = True

    def pointcloud_callback(self, msg: PointCloud2):
        """
        Extract downward LiDAR points, find ground height below drone,
        update filtered estimate and compute target NED altitude.
        """
        self.scan_count += 1

        if not self.position_valid:
            if self.scan_count % 50 == 0:
                self.get_logger().warn('Waiting for valid drone position...')
            return

        points = self._extract_xyz_vectorised(msg)
        if points is None or len(points) == 0:
            return

        # --- Filter: keep only downward rays (elevation < 0 deg) ---
        xy_dist = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        elevation_angle = np.degrees(np.arctan2(points[:, 2], xy_dist))
        downward_mask = elevation_angle < self.max_elevation_deg
        points = points[downward_mask]
        if len(points) == 0:
            return

        # --- Filter: nadir cone only (directly below the drone) ---
        xy_dist_filtered = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        nadir_mask = xy_dist_filtered <= self.nadir_radius_m
        points = points[nadir_mask]
        if len(points) == 0:
            return

        # --- Transform to world frame (same convention as terrain_scoring_node) ---
        cos_h = math.cos(self.drone_heading)
        sin_h = math.sin(self.drone_heading)
        wx = points[:, 0] * cos_h - points[:, 1] * sin_h + self.drone_x
        wy = points[:, 0] * sin_h + points[:, 1] * cos_h + self.drone_y
        wz = -self.drone_z + points[:, 2]

        # --- Sanity filter on world z ---
        valid = (wz >= self.ground_z_min) & (wz <= self.ground_z_max)
        wz = wz[valid]
        if len(wz) == 0:
            return

        # Ground estimate = median of downward returns (robust to outliers)
        ground_z_raw = float(np.median(wz))

        # --- Low-pass filter ---
        if self.ground_z_estimate is None:
            self.ground_z_estimate = ground_z_raw
        else:
            self.ground_z_estimate = (
                self.filter_alpha * self.ground_z_estimate +
                (1.0 - self.filter_alpha) * ground_z_raw
            )

        # --- Compute target NED altitude ---
        # NED z is negative when above ground.
        # target_ned_z = -(ground_z_world + target_height_m)
        # Because: drone_z_ned = -drone_z_world, so
        #          target_z_world = ground_z_world + target_height_m
        #          target_z_ned   = -(ground_z_world + target_height_m)
        self.cruise_altitude_ned = -(self.ground_z_estimate + self.target_height_m)

        if self.scan_count % 50 == 0:
            drone_height_agl = -self.drone_z - self.ground_z_estimate
            self.get_logger().info(
                f'Scan {self.scan_count} | '
                f'ground z {self.ground_z_estimate:.2f}m | '
                f'drone AGL {drone_height_agl:.2f}m | '
                f'target NED z {self.cruise_altitude_ned:.2f}'
            )

    # -------------------------------------------------------------------------
    # Publish
    # -------------------------------------------------------------------------

    def publish(self):
        """Publish current cruise altitude target."""
        if self.cruise_altitude_ned is None:
            return

        msg = Float32()
        msg.data = float(self.cruise_altitude_ned)
        self.altitude_pub.publish(msg)

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------

    def _extract_xyz_vectorised(self, msg: PointCloud2):
        """Extract XYZ from PointCloud2 using numpy (vectorised)."""
        x_off = y_off = z_off = None
        for field in msg.fields:
            if field.name == 'x':
                x_off = field.offset
            elif field.name == 'y':
                y_off = field.offset
            elif field.name == 'z':
                z_off = field.offset

        if None in (x_off, y_off, z_off):
            return None

        n_points = msg.width * msg.height
        if n_points == 0:
            return None

        data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        step = msg.point_step
        x = np.frombuffer(data[x_off::step][:n_points * 4].copy(), dtype=np.float32)[:n_points]
        y = np.frombuffer(data[y_off::step][:n_points * 4].copy(), dtype=np.float32)[:n_points]
        z = np.frombuffer(data[z_off::step][:n_points * 4].copy(), dtype=np.float32)[:n_points]

        points = np.column_stack([x, y, z])
        return points[np.isfinite(points).all(axis=1)]


def main(args=None):
    rclpy.init(args=args)
    node = TerrainFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
