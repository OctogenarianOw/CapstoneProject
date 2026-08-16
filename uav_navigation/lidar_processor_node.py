#!/usr/bin/env python3
"""
lidar_processor_node.py

Simulates a rotating LiDAR by republishing point cloud sectors
sequentially, giving the visual impression of a spinning scanner.

Subscribes:  /lidar/points        (PointCloud2) - full scan
Publishes:   /lidar/rotating      (PointCloud2) - one sector at a time

Author: Dorian Veloso
Capstone A - UTS 2026
Supervisor: Dr. Graeme Best
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
import math


class LidarProcessorNode(Node):
    def __init__(self):
        super().__init__('lidar_processor_node')

        # Number of sectors to split each full scan into
        self.n_sectors = 10
        self.current_sector = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.callback,
            sensor_qos
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/lidar/rotating',
            10
        )

        self.get_logger().info('LiDAR processor node started')
        self.get_logger().info(f'Splitting each scan into {self.n_sectors} sectors')

    def callback(self, msg: PointCloud2):
        """Split full scan into sectors and publish one per callback."""

        points = self.extract_xyz(msg)
        if points is None or len(points) == 0:
            return

        # Compute azimuth angle for each point
        angles = np.arctan2(points[:, 1], points[:, 0])  # -pi to pi

        # Define sector boundaries
        sector_width = 2 * math.pi / self.n_sectors
        sector_min = -math.pi + self.current_sector * sector_width
        sector_max = sector_min + sector_width

        # Select points in current sector
        if self.current_sector == self.n_sectors - 1:
            # Last sector wraps around to pi
            mask = angles >= sector_min
        else:
            mask = (angles >= sector_min) & (angles < sector_max)

        sector_points = points[mask]

        if len(sector_points) > 0:
            out_msg = self.build_pointcloud2(sector_points, msg.header)
            self.pub.publish(out_msg)

        # Advance to next sector
        self.current_sector = (self.current_sector + 1) % self.n_sectors

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

    def build_pointcloud2(self, points, header):
        """Build a PointCloud2 message from Nx3 numpy array."""
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 12  # 3 x float32
        msg.row_step = msg.point_step * msg.width

        msg.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]

        msg.data = points.astype(np.float32).tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
