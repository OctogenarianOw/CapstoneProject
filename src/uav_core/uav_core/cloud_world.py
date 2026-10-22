"""
Lidar ingestion: frame change and fusion into the elevation grid (S5).

Role: take each downward lidar scan, put its points in the NED world
frame with the attitude and position of the aircraft, and fuse them
into the elevation grid, saved atomically every 5 s and published on
/map/elevation at 1 Hz for the exploration manager and the display.
The publication is reliable and transient local, so a node started
later receives the latest map at once; 1 Hz matches the decision rate
of the manager (S13 asks the invariant to be checked at 1 Hz at least).

Frame chain: sensor FLU to body FRD by (x, -y, -z), plus the sensor
offset, then body FRD to NED by the attitude quaternion, then the NED
position. Rejected: skipping the rotation. A point 15 m ahead then lands
at the wrong height as soon as the aircraft tilts; in the old stack this
turned the whole terrain red.

Filtering on the norm of each point, between min_range and
max_usable_range: Gazebo encodes rays without return as zeros, not
infinities, so a finiteness test lets them through (spec trap 8).

Per-point variance sigma = sigma_range + sigma_angular * distance, as
S5 prescribes. Spec 6.3 records this model as a negative result (map
error 0.1209 to 0.1433 m); the contradiction is open, to settle by a
measurement on one map.

Analysis runs on this map, never on the raw cloud, which is mostly
noise: that was the supervisor's explicit request.

Pose at the time of the scan (pose_at_scan_time, default true), not the
latest received. Measured (experiments/2026-10-21_p5_latency): with the
latest pose the edges of raised structures spilled half a cell along
the flight, removed by taking the pose 0.110 to 0.119 s earlier, the
origin of the one-cell offset P5. Of that lag, 0.03 s is the latest
pose being received after the scan, and varies with the load; the rest
is a constant between time bases, the PX4 estimate being ahead of the
true position by 0.09 to 0.11 s against Gazebo's truth. So the pose is
interpolated at the scan stamp, put on the PX4 clock through offsets
estimated online from /clock and from the poses (uav_core/pose_buffer),
minus scan_time_offset_s, a temporal calibration term like the one
between any sensor and an estimator: 0.085 s, the map lag minus the
stamp lag on the two flights with /clock (0.082 and 0.087 s). When the
buffer does not cover that time, or /clock is silent, the latest pose
is used and counted, never silently. pose_at_scan_time false gives the
previous behaviour, which is how the change is measured.

No fusion below min_fuse_height_m, 2 m above the EKF origin. From the
ground, at 0.37 m, the lidar sees the terrain at grazing angles: on the
survey of 19 Oct the landing cells fell from 1620 to 1304 once the
aircraft had landed, and the 165 cells seen before take-off came from
the same geometry (experiments/2026-10-19_live_map).
"""

import os

import numpy as np
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data, QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from uav_core.elevation_grid import ElevationGrid
from uav_core.map_msg import grid_to_msg
from uav_core.pose_buffer import ClockOffset, PoseBuffer
from uav_interfaces.msg import ElevationMap


def quat_to_matrix(q):
    """Rotation from body FRD to NED earth, q is [w, x, y, z]."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class CloudWorld(Node):
    """Lidar scans in, elevation grid out."""

    def __init__(self):
        super().__init__('cloud_world')

        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('max_usable_range', 20.0)
        self.declare_parameter('sensor_offset_down', 0.15)
        self.declare_parameter('grid_size', 100.0)
        self.declare_parameter('grid_resolution', 1.0)
        self.declare_parameter('sigma_range_m', 0.02)
        self.declare_parameter('sigma_angular_rad', 0.009)
        self.declare_parameter('map_path', '/tmp/elevation_map.npz')
        self.declare_parameter('map_publish_period_s', 1.0)
        self.declare_parameter('min_fuse_height_m', 2.0)
        self.declare_parameter('pose_at_scan_time', True)
        self.declare_parameter('scan_time_offset_s', 0.085)
        self.min_fuse_height = self.get_parameter('min_fuse_height_m').value
        self.pose_at_scan = self.get_parameter('pose_at_scan_time').value
        self.scan_time_offset = self.get_parameter('scan_time_offset_s').value

        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_usable_range').value
        self.sensor_offset = np.array(
            [0.0, 0.0, self.get_parameter('sensor_offset_down').value])
        self.sigma_range = self.get_parameter('sigma_range_m').value
        self.sigma_angular = self.get_parameter('sigma_angular_rad').value
        self.map_path = self.get_parameter('map_path').value

        self.grid = ElevationGrid(
            size_m=self.get_parameter('grid_size').value,
            resolution=self.get_parameter('grid_resolution').value,
        )

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.position = None
        self.attitude = None
        self.poses = PoseBuffer()
        # System minus simulation clock: minimum over the last ~0.3 s of
        # /clock (about 250 Hz), so it follows a step of the system
        # clock at once. System minus PX4 clock: median over ~2 s of
        # poses (50 Hz), both clocks stepping together (pose_buffer).
        self.sim_offset = ClockOffset(window=75)
        self.px4_offset = ClockOffset(window=100, mode='median')
        self.scans_at_stamp = 0
        self.scans_latest = 0
        self.fallback_why = {}
        self.last_margins = None

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.on_position, px4_qos)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.on_attitude, px4_qos)
        self.create_subscription(
            PointCloud2, '/lidar/points',
            self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(
            Clock, '/clock', self.on_clock, qos_profile_sensor_data)

        self.world_pub = self.create_publisher(
            PointCloud2, '/cloud_world', 10)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_pub = self.create_publisher(
            ElevationMap, '/map/elevation', map_qos)

        self.log_counter = 0
        self.save_timer = self.create_timer(5.0, self.save_map)
        self.map_timer = self.create_timer(
            self.get_parameter('map_publish_period_s').value,
            self.publish_map)

    def received_s(self, info):
        """
        Return the reception time of a message by the middleware, s.

        Not the time its callback runs: the fusion of a scan holds this
        single-threaded node for tens of milliseconds, the poses and
        /clock wait behind it, and an offset taken at callback time was
        biased by that wait. Measured on 21 Oct (2026-10-21_p5_fix,
        first version): 46 % of the scans fell back to the latest pose,
        in blocks of 16 s beating with the lidar rate. The middleware
        stamps a message when it arrives; the node clock is the fallback
        if it does not.
        """
        t = info.get('received_timestamp', 0) if info else 0
        return t * 1e-9 if t else self.get_clock().now().nanoseconds * 1e-9

    def on_position(self, msg, info):
        """Keep the latest NED position and buffer it on the PX4 clock."""
        self.position = np.array([msg.x, msg.y, msg.z])
        t = msg.timestamp_sample * 1e-6
        self.px4_offset.update(self.received_s(info), msg.timestamp * 1e-6)
        self.poses.add_position(t, self.position)

    def on_attitude(self, msg):
        """Keep the latest attitude, [w, x, y, z], and buffer it."""
        self.attitude = np.array(msg.q)
        self.poses.add_attitude(msg.timestamp_sample * 1e-6, self.attitude)

    def on_clock(self, msg, info):
        """Track the offset between the system and simulation clocks."""
        self.sim_offset.update(self.received_s(info),
                               msg.clock.sec + msg.clock.nanosec * 1e-9)

    def scan_pose(self, msg):
        """Position and attitude for a scan: at its stamp if possible."""
        if self.pose_at_scan and self.sim_offset.ready \
                and self.px4_offset.ready:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            t_px4 = (stamp + self.sim_offset.value - self.px4_offset.value
                     - self.scan_time_offset)
            pose = self.poses.at(t_px4)
            if pose is not None:
                self.scans_at_stamp += 1
                return pose
            self.diagnose(t_px4)
        elif self.pose_at_scan:
            self.fallback_why['no offset'] = (
                self.fallback_why.get('no offset', 0) + 1)
        self.scans_latest += 1
        return self.position, self.attitude

    def diagnose(self, t_px4):
        """Count why a scan time was not covered, keep the margins."""
        p, a = self.poses.pos_t, self.poses.att_t
        if not p or not a:
            why = 'empty buffer'
        elif t_px4 > p[-1] + self.poses.max_extrapolation_s:
            why = 'after last position'
        elif t_px4 > a[-1] + self.poses.max_extrapolation_s:
            why = 'after last attitude'
        else:
            why = 'before the buffer'
        self.fallback_why[why] = self.fallback_why.get(why, 0) + 1
        if p and a:
            self.last_margins = (round(t_px4 - p[-1], 3),
                                 round(t_px4 - a[-1], 3),
                                 round(t_px4 - p[0], 3), len(p), len(a))

    def extract_xyz(self, msg):
        """Decode x, y, z from a PointCloud2, offsets read from its fields."""
        offsets = {f.name: f.offset for f in msg.fields}
        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            -1, msg.point_step)

        def column(name):
            o = offsets[name]
            # A slice is not contiguous: copy before the float view.
            return buf[:, o:o + 4].copy().view(np.float32).ravel()

        return np.stack([column('x'), column('y'), column('z')], axis=1)

    def on_cloud(self, msg):
        """Transform one scan to NED and fuse it into the grid."""
        if self.position is None or self.attitude is None:
            return
        if -self.position[2] < self.min_fuse_height:
            return

        pts = self.extract_xyz(msg)

        norms = np.linalg.norm(pts, axis=1)
        keep = (norms > self.min_range) & (norms < self.max_range)
        pts = pts[keep]
        dist = norms[keep]
        if pts.shape[0] == 0:
            return

        # Sensor FLU to body FRD: y and z change sign.
        pts_frd = pts * np.array([1.0, -1.0, -1.0])
        pts_frd = pts_frd + self.sensor_offset

        position, attitude = self.scan_pose(msg)
        rot = quat_to_matrix(attitude)
        pts_ned = pts_frd @ rot.T + position

        # Vertical uncertainty: sensor noise plus angular error, which
        # grows with the observation distance.
        sigma = self.sigma_range + self.sigma_angular * dist
        variances = (sigma ** 2).astype(np.float32)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map_ned'
        self.world_pub.publish(
            point_cloud2.create_cloud_xyz32(header, pts_ned.tolist()))

        self.grid.update(pts_ned, variances)

        self.log_counter += 1
        if self.log_counter % 20 == 0:
            known = self.grid.known_cells()
            valid = self.grid.elevation[self.grid.count > 0]
            self.get_logger().info(
                f'{pts_ned.shape[0]} pts | {known} cells | '
                f'mean map z {valid.mean():+.3f} '
                f'| sd {valid.std():.3f} | pose at stamp '
                f'{self.scans_at_stamp}, latest {self.scans_latest} '
                f'| fallbacks {self.fallback_why} | margins t-last pos, '
                f't-last att, t-first, n pos, n att {self.last_margins} '
                f'| offsets sim {self.sim_offset.value} px4 '
                f'{self.px4_offset.value}')

    def publish_map(self):
        """Publish the whole grid with its geometry."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map_ned'
        self.map_pub.publish(grid_to_msg(
            self.grid.elevation, self.grid.variance, self.grid.count,
            self.grid.resolution, (self.grid.origin_x, self.grid.origin_y),
            header))

    def save_map(self):
        """Write the grid to map_path atomically, temp file then rename."""
        if self.grid.known_cells() == 0:
            return
        tmp = self.map_path + '.tmp'
        np.savez(
            tmp,
            elevation=self.grid.elevation,
            variance=self.grid.variance,
            count=self.grid.count,
            resolution=self.grid.resolution,
            origin_x=self.grid.origin_x,
            origin_y=self.grid.origin_y,
        )
        os.replace(tmp + '.npz', self.map_path)


def main(args=None):
    """Run the node until interrupted."""
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
