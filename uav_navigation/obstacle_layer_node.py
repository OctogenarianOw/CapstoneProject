#!/usr/bin/env python3
"""
obstacle_layer_node.py
Builds a 2D obstacle navigation layer from LiDAR lateral rays (elevation > 0 deg).
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
import math


class ObstacleLayerNode(Node):

    def __init__(self):
        super().__init__('obstacle_layer_node')

        self.resolution = 0.5
        self.grid_size = 200
        self.grid_origin_x = -50.0
        self.grid_origin_y = -50.0
        self.min_elevation_deg = 0.0
        self.min_obstacle_height_m = 0.3
        self.max_obstacle_height_m = 30.0
        self.publish_rate = 2.0

        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.drone_heading = 0.0
        self.position_valid = False

        self.obs_z_max = np.full((self.grid_size, self.grid_size), -np.inf, dtype=np.float32)
        self.obs_count = np.zeros((self.grid_size, self.grid_size), dtype=np.int32)

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=5)
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                  history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pc_sub = self.create_subscription(PointCloud2, '/lidar/points',
                                               self.pointcloud_callback, sensor_qos)

        try:
            from px4_msgs.msg import VehicleLocalPosition
            self.pos_sub = self.create_subscription(
                VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                self.position_callback, sensor_qos)
            self.get_logger().info('Subscribed to VehicleLocalPosition')
        except ImportError:
            self.get_logger().warn('px4_msgs not available')
            self.pos_sub = None

        self.grid_pub = self.create_publisher(OccupancyGrid, '/obstacle_grid', reliable_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_grid/markers', reliable_qos)
        self.create_timer(1.0 / self.publish_rate, self.publish)

        self.scan_count = 0
        self.get_logger().info(
            f'ObstacleLayerNode started | '
            f'grid {self.grid_size}x{self.grid_size} @ {self.resolution}m | '
            f'lateral rays only (elevation > {self.min_elevation_deg} deg)')

    def position_callback(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.drone_x = msg.x
            self.drone_y = msg.y
            self.drone_z = msg.z
            self.drone_heading = msg.heading
            self.position_valid = True

    def pointcloud_callback(self, msg: PointCloud2):
        self.scan_count += 1
        if not self.position_valid:
            if self.scan_count % 50 == 0:
                self.get_logger().warn('Waiting for valid drone position...')
            return

        points = self._extract_xyz_vectorised(msg)
        if points is None or len(points) == 0:
            return

        xy_dist = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        elevation_angle = np.degrees(np.arctan2(points[:, 2], xy_dist))
        points = points[elevation_angle > self.min_elevation_deg]
        if len(points) == 0:
            return

        cos_h = math.cos(self.drone_heading)
        sin_h = math.sin(self.drone_heading)
        wx = points[:, 0] * cos_h - points[:, 1] * sin_h + self.drone_x
        wy = points[:, 0] * sin_h + points[:, 1] * cos_h + self.drone_y
        wz = -self.drone_z + points[:, 2]

        height_mask = (wz >= self.min_obstacle_height_m) & (wz <= self.max_obstacle_height_m)
        wx, wy, wz = wx[height_mask], wy[height_mask], wz[height_mask]
        if len(wx) == 0:
            return

        gi = ((wx - self.grid_origin_x) / self.resolution).astype(np.int32)
        gj = ((wy - self.grid_origin_y) / self.resolution).astype(np.int32)
        in_bounds = ((gi >= 0) & (gi < self.grid_size) &
                     (gj >= 0) & (gj < self.grid_size))
        gi, gj, wz = gi[in_bounds], gj[in_bounds], wz[in_bounds]
        if len(gi) == 0:
            return

        np.maximum.at(self.obs_z_max, (gj, gi), wz)
        np.add.at(self.obs_count, (gj, gi), 1)

        if self.scan_count % 50 == 0:
            occupied = int(np.sum(np.isfinite(self.obs_z_max)))
            self.get_logger().info(
                f'Scan {self.scan_count}: {len(points)} lateral pts | '
                f'{occupied} obstacle cells | '
                f'drone ({self.drone_x:.1f}, {self.drone_y:.1f}, {-self.drone_z:.1f}m)')

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        self._publish_occupancy_grid(stamp)
        self._publish_markers(stamp)

    def _publish_occupancy_grid(self, stamp):
        msg = OccupancyGrid()
        msg.header = Header(stamp=stamp, frame_id='map')
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = self.grid_origin_x
        msg.info.origin.position.y = self.grid_origin_y
        msg.info.origin.position.z = 0.0
        occupied = np.isfinite(self.obs_z_max)
        grid_data = np.where(occupied, 100, -1).astype(np.int8)
        msg.data = grid_data.flatten().tolist()
        self.grid_pub.publish(msg)

    def _publish_markers(self, stamp):
        marker_array = MarkerArray()
        clear = Marker()
        clear.header = Header(stamp=stamp, frame_id='map')
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        marker = Marker()
        marker.header = Header(stamp=stamp, frame_id='map')
        marker.ns = 'obstacle_layer'
        marker.id = 1
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.resolution * 0.9
        marker.scale.y = self.resolution * 0.9
        marker.scale.z = 0.5
        marker.color = ColorRGBA(r=1.0, g=0.2, b=0.0, a=0.7)

        js, is_ = np.where(np.isfinite(self.obs_z_max))
        for j, i in zip(js, is_):
            cx = self.grid_origin_x + (i + 0.5) * self.resolution
            cy = self.grid_origin_y + (j + 0.5) * self.resolution
            cz = float(self.obs_z_max[j, i])
            marker.points.append(Point(x=cx, y=cy, z=cz))

        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def _extract_xyz_vectorised(self, msg: PointCloud2):
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
    node = ObstacleLayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
