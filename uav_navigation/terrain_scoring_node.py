#!/usr/bin/env python3
"""
terrain_scoring_node.py

Subscribes to /lidar/points (PointCloud2) and /fmu/out/vehicle_local_position_v1,
transforms points to world frame using drone position, accumulates into a 2D
elevation grid, and computes per-cell slope, roughness, and safety scores.

Methodology follows Beycimen et al. (2026) for geometric-based terrain
traversability, and Fankhauser et al. (2018) for probabilistic terrain
mapping with uncertain localization.

Publishes:
  /terrain/safety_map    (OccupancyGrid)  - safety score 0-100
  /terrain/elevation_map (OccupancyGrid)  - elevation visualization
  /terrain/markers       (MarkerArray)    - colored 3D grid for RViz2

Author: Dorian Veloso
Capstone A - UTS 2026
Supervisor: Dr. Graeme Best
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Header
import numpy as np
import struct
import math


class TerrainScoringNode(Node):
    def __init__(self):
        super().__init__('terrain_scoring_node')

        # --- Grid parameters ---
        self.resolution = 0.5
        self.grid_size = 200
        self.grid_origin_x = -50.0
        self.grid_origin_y = -50.0

        # --- Terrain scoring thresholds (Beycimen et al.) ---
        self.slope_critical = 25.0
        self.roughness_critical = 0.3
        self.step_critical = 0.5

        # --- Weights for combined traversability (Beycimen et al.) ---
        self.w_slope = 0.4
        self.w_roughness = 0.35
        self.w_step = 0.25

        # --- Minimum points per cell for confident assessment ---
        self.min_points_confidence = 5

        # --- Drone position (from PX4, NED frame) ---
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.drone_heading = 0.0
        self.position_valid = False

        # --- Elevation grid storage ---
        self.z_sum = np.zeros((self.grid_size, self.grid_size), dtype=np.float64)
        self.z_sq_sum = np.zeros((self.grid_size, self.grid_size), dtype=np.float64)
        self.z_count = np.zeros((self.grid_size, self.grid_size), dtype=np.int32)
        self.z_min = np.full((self.grid_size, self.grid_size), np.inf)
        self.z_max = np.full((self.grid_size, self.grid_size), -np.inf)

        # --- Derived maps ---
        self.elevation = np.zeros((self.grid_size, self.grid_size), dtype=np.float64)
        self.slope = np.zeros((self.grid_size, self.grid_size), dtype=np.float64)
        self.roughness = np.zeros((self.grid_size, self.grid_size), dtype=np.float64)
        self.step_height = np.zeros((self.grid_size, self.grid_size), dtype=np.float64)
        self.safety_score = np.full((self.grid_size, self.grid_size), -1.0)

        # --- QoS ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
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
            self.get_logger().warn('px4_msgs not found')
            self.pos_sub = None

        # --- Publishers ---
        self.safety_pub = self.create_publisher(OccupancyGrid, '/terrain/safety_map', 10)
        self.elev_pub = self.create_publisher(OccupancyGrid, '/terrain/elevation_map', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/terrain/markers', 10)
        self.zcount_pub = self.create_publisher(OccupancyGrid, '/terrain/z_count', 10)

        # --- Timer ---
        self.create_timer(1.0, self.compute_and_publish)

        self.get_logger().info('Terrain scoring node started')
        self.get_logger().info(f'Grid: {self.grid_size}x{self.grid_size} cells, '
                               f'{self.resolution}m resolution')

        self.scan_count = 0

    def position_callback(self, msg):
        """Update drone position from PX4 VehicleLocalPosition (NED)."""
        if msg.xy_valid and msg.z_valid:
            self.drone_x = msg.x
            self.drone_y = msg.y
            self.drone_z = msg.z
            self.drone_heading = msg.heading
            self.position_valid = True

    def pointcloud_callback(self, msg: PointCloud2):
        """Extract XYZ points, transform to world frame, accumulate."""
        self.scan_count += 1

        if not self.position_valid:
            if self.scan_count % 100 == 0:
                self.get_logger().warn('Waiting for drone position...')
            return

        points = self.extract_xyz(msg)
        if points is None or len(points) == 0:
            return

        cos_h = math.cos(self.drone_heading)
        sin_h = math.sin(self.drone_heading)

        # Rotate by heading and translate by drone position
        wx = points[:, 0] * cos_h - points[:, 1] * sin_h + self.drone_x
        wy = points[:, 0] * sin_h + points[:, 1] * cos_h + self.drone_y
        # PX4 NED: drone_z is negative when above ground
        # LiDAR z: negative = below sensor
        wz = -self.drone_z + points[:, 2]

        # Convert to grid indices
        gi = ((wx - self.grid_origin_x) / self.resolution).astype(np.int32)
        gj = ((wy - self.grid_origin_y) / self.resolution).astype(np.int32)

        valid = ((gi >= 0) & (gi < self.grid_size) &
                 (gj >= 0) & (gj < self.grid_size) &
                 (wz > -10.0) & (wz < 20.0))

        gi = gi[valid]
        gj = gj[valid]
        wz = wz[valid]

        if len(gi) == 0:
            return

        for k in range(len(gi)):
            i, j, z = gi[k], gj[k], wz[k]
            self.z_sum[j, i] += z
            self.z_sq_sum[j, i] += z * z
            self.z_count[j, i] += 1
            if z < self.z_min[j, i]:
                self.z_min[j, i] = z
            if z > self.z_max[j, i]:
                self.z_max[j, i] = z

        if self.scan_count % 50 == 0:
            observed = int(np.sum(self.z_count > 0))
            confident = int(np.sum(self.z_count >= self.min_points_confidence))
            self.get_logger().info(
                f'Scan {self.scan_count}: {observed} observed, '
                f'{confident} confident | '
                f'Drone: ({self.drone_x:.1f}, {self.drone_y:.1f}, {self.drone_z:.1f})'
            )

    def extract_xyz(self, msg: PointCloud2):
        """Extract XYZ from PointCloud2."""
        x_off = y_off = z_off = None
        for field in msg.fields:
            if field.name == 'x':
                x_off = field.offset
            elif field.name == 'y':
                y_off = field.offset
            elif field.name == 'z':
                z_off = field.offset

        if x_off is None or y_off is None or z_off is None:
            return None

        point_step = msg.point_step
        data = bytes(msg.data)
        n_points = msg.width * msg.height

        if n_points == 0:
            return None

        points = np.zeros((n_points, 3), dtype=np.float32)
        for i in range(n_points):
            offset = i * point_step
            try:
                points[i, 0] = struct.unpack_from('f', data, offset + x_off)[0]
                points[i, 1] = struct.unpack_from('f', data, offset + y_off)[0]
                points[i, 2] = struct.unpack_from('f', data, offset + z_off)[0]
            except struct.error:
                break

        valid = np.isfinite(points).all(axis=1)
        return points[valid]

    def compute_and_publish(self):
        """Compute terrain features and publish maps."""
        observed = self.z_count > 0
        confident = self.z_count >= self.min_points_confidence

        if not np.any(observed):
            return

        # 1. Elevation
        with np.errstate(divide='ignore', invalid='ignore'):
            self.elevation = np.where(observed,
                                       self.z_sum / np.maximum(self.z_count, 1), 0.0)

        # 2. Roughness (Beycimen et al.: std_dev of z)
        with np.errstate(divide='ignore', invalid='ignore'):
            mean_z = self.z_sum / np.maximum(self.z_count, 1)
            var_z = np.maximum((self.z_sq_sum / np.maximum(self.z_count, 1)) - mean_z**2, 0.0)
            self.roughness = np.where(confident, np.sqrt(var_z), 0.0)

        # 3. Step height (Beycimen et al.: max - min)
        self.step_height = np.where(
            confident & np.isfinite(self.z_max) & np.isfinite(self.z_min),
            self.z_max - self.z_min, 0.0)

        # 4. Slope (gradient of elevation)
        if np.any(confident):
            dy, dx = np.gradient(self.elevation, self.resolution)
            self.slope = np.where(confident,
                                   np.degrees(np.arctan(np.sqrt(dx**2 + dy**2))), 0.0)

        # 5. Safety score (Beycimen et al. weighted traversability)
        t_slope = np.where(self.slope <= self.slope_critical,
                            1.0 - self.slope / self.slope_critical, 0.0)
        t_roughness = np.where(self.roughness <= self.roughness_critical,
                                1.0 - self.roughness / self.roughness_critical, 0.0)
        t_step = np.where(self.step_height <= self.step_critical,
                           1.0 - self.step_height / self.step_critical, 0.0)

        combined = (self.w_slope * t_slope +
                     self.w_roughness * t_roughness +
                     self.w_step * t_step)

        self.safety_score = np.where(confident,
                                      np.clip(combined * 100, 0, 100), -1.0)

        stamp = self.get_clock().now().to_msg()
        self.publish_occupancy_grid(self.safety_pub, self.safety_score, stamp)
        self.publish_elevation_grid(self.elev_pub, self.elevation, observed, stamp)
        self.publish_markers(stamp)
        self.publish_z_count(stamp)

    def publish_occupancy_grid(self, publisher, data, stamp):
        msg = OccupancyGrid()
        msg.header = Header(stamp=stamp, frame_id='lidar_3d_link')
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = self.grid_origin_x
        msg.info.origin.position.y = self.grid_origin_y
        flat = data.flatten()
        msg.data = np.where(flat < 0, -1, np.clip(flat, 0, 100)).astype(np.int8).tolist()
        publisher.publish(msg)

    def publish_z_count(self, stamp):
        """Publish raw observation count per cell for frontier detection."""
        msg = OccupancyGrid()
        msg.header = Header(stamp=stamp, frame_id='lidar_3d_link')
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = self.grid_origin_x
        msg.info.origin.position.y = self.grid_origin_y
        # Clamp to 0-100 for OccupancyGrid (100+ points all map to 100)
        clamped = np.clip(self.z_count, 0, 100).astype(np.int8)
        msg.data = clamped.flatten().tolist()
        self.zcount_pub.publish(msg)

    def publish_elevation_grid(self, publisher, elevation, observed, stamp):
        msg = OccupancyGrid()
        msg.header = Header(stamp=stamp, frame_id='lidar_3d_link')
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = self.grid_origin_x
        msg.info.origin.position.y = self.grid_origin_y
        e_min = np.min(elevation[observed]) if np.any(observed) else 0
        e_max = np.max(elevation[observed]) if np.any(observed) else 1
        e_range = max(e_max - e_min, 0.01)
        normalized = np.where(observed,
                               ((elevation - e_min) / e_range * 100).astype(np.int8), -1)
        msg.data = normalized.flatten().tolist()
        publisher.publish(msg)

    def publish_markers(self, stamp):
        marker_array = MarkerArray()
        clear = Marker()
        clear.header = Header(stamp=stamp, frame_id='lidar_3d_link')
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        marker = Marker()
        marker.header = Header(stamp=stamp, frame_id='lidar_3d_link')
        marker.ns = 'terrain_safety'
        marker.id = 1
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.resolution * 0.9
        marker.scale.y = self.resolution * 0.9
        marker.scale.z = 0.1

        confident = self.z_count >= self.min_points_confidence

        for j in range(self.grid_size):
            for i in range(self.grid_size):
                if not confident[j, i]:
                    continue
                score = self.safety_score[j, i]
                if score < 0:
                    continue

                wx = self.grid_origin_x + (i + 0.5) * self.resolution
                wy = self.grid_origin_y + (j + 0.5) * self.resolution
                wz = self.elevation[j, i]

                marker.points.append(Point(x=wx, y=wy, z=wz))

                c = ColorRGBA(a=0.8)
                if score > 70:
                    c.r, c.g, c.b = 0.1, 0.8, 0.1
                elif score > 30:
                    t = (score - 30) / 40.0
                    c.r, c.g, c.b = 1.0 - 0.5*t, 0.5 + 0.3*t, 0.1
                else:
                    c.r, c.g, c.b = 0.9, 0.1, 0.1
                marker.colors.append(c)

        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainScoringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
