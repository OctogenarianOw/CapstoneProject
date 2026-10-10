import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
    qos_profile_sensor_data,
)

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude


def quat_to_matrix(q):
    """Rotation FRD corps vers NED terre. q est [w, x, y, z]."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class CloudWorld(Node):
    def __init__(self):
        super().__init__('cloud_world')

        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('max_usable_range', 20.0)
        self.declare_parameter('sensor_offset_down', 0.15)

        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_usable_range').value
        self.sensor_offset = np.array(
            [0.0, 0.0, self.get_parameter('sensor_offset_down').value])

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.position = None
        self.attitude = None

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.on_position, px4_qos)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.on_attitude, px4_qos)
        self.create_subscription(
            PointCloud2, '/lidar/points',
            self.on_cloud, qos_profile_sensor_data)

        self.world_pub = self.create_publisher(
            PointCloud2, '/cloud_world', 10)

        self.log_counter = 0

    def on_position(self, msg):
        self.position = np.array([msg.x, msg.y, msg.z])

    def on_attitude(self, msg):
        self.attitude = np.array(msg.q)

    def extract_xyz(self, msg):
        offsets = {f.name: f.offset for f in msg.fields}
        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)

        def column(name):
            o = offsets[name]
            return buf[:, o:o + 4].copy().view(np.float32).ravel()

        return np.stack([column('x'), column('y'), column('z')], axis=1)

    def on_cloud(self, msg):
        if self.position is None or self.attitude is None:
            return

        pts = self.extract_xyz(msg)

        norms = np.linalg.norm(pts, axis=1)
        keep = (norms > self.min_range) & (norms < self.max_range)
        pts = pts[keep]
        if pts.shape[0] == 0:
            return

        # FLU capteur vers FRD corps : y et z changent de signe
        pts_frd = pts * np.array([1.0, -1.0, -1.0])
        pts_frd = pts_frd + self.sensor_offset

        rot = quat_to_matrix(self.attitude)
        pts_ned = pts_frd @ rot.T + self.position

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map_ned'
        self.world_pub.publish(
            point_cloud2.create_cloud_xyz32(header, pts_ned.tolist()))

        self.log_counter += 1
        if self.log_counter % 20 == 0:
            z = pts_ned[:, 2]
            self.get_logger().info(
                f'{pts_ned.shape[0]} pts | z moyen {z.mean():+.3f} '
                f'| min {z.min():+.3f} | max {z.max():+.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = CloudWorld()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()