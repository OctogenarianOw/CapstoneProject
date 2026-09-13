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
from scipy.spatial.transform import Rotation


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
        self.min_obstacle_hits = 3       # minimum obstacle-height detections,
                                          # so one stray point never counts
        self.min_obstacle_ratio = 0.5    # of all times a cell was looked at
                                          # (any lateral ray landing there),
                                          # this fraction must be obstacle-
                                          # height. A cell seen 50 times and
                                          # positive 3 times by noise reads
                                          # very differently from one seen
                                          # 5 times and positive all 5.
        self.safety_radius_m = 1.5       # never mark the drone's own
                                          # current position as an obstacle

        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.drone_heading = 0.0
        self.position_valid = False

        # --- Orientation (from PX4 VehicleAttitude, FRD body -> NED world) ---
        self.drone_rotation = None
        self.attitude_valid = False

        self.obs_z_max = np.full((self.grid_size, self.grid_size), -np.inf, dtype=np.float32)
        self.obs_hits = np.zeros((self.grid_size, self.grid_size), dtype=np.int32)
        self.obs_seen = np.zeros((self.grid_size, self.grid_size), dtype=np.int32)

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

        try:
            from px4_msgs.msg import VehicleAttitude
            self.att_sub = self.create_subscription(
                VehicleAttitude, '/fmu/out/vehicle_attitude',
                self.attitude_callback, sensor_qos)
            self.get_logger().info('Subscribed to VehicleAttitude')
        except ImportError:
            self.get_logger().warn('px4_msgs VehicleAttitude not found')
            self.att_sub = None

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

    def attitude_callback(self, msg):
        """Update drone orientation from PX4 VehicleAttitude (FRD body -> NED world)."""
        w, x, y, z = msg.q  # PX4 gives [w, x, y, z]
        self.drone_rotation = Rotation.from_quat([x, y, z, w])  # scipy wants [x, y, z, w]
        self.attitude_valid = True

    def pointcloud_callback(self, msg: PointCloud2):
        self.scan_count += 1
        if not self.position_valid or not self.attitude_valid:
            if self.scan_count % 50 == 0:
                self.get_logger().warn('Waiting for valid drone position/attitude...')
            return

        points = self._extract_xyz_vectorised(msg)
        if points is None or len(points) == 0:
            return

        xy_dist = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        elevation_angle = np.degrees(np.arctan2(points[:, 2], xy_dist))
        points = points[elevation_angle > self.min_elevation_deg]
        if len(points) == 0:
            return

        # Full roll/pitch/yaw correction (not yaw-only); same FLU-body ->
        # FRD -> NED-rotate -> world convention as terrain_scoring_node.
        points_frd = points * np.array([1.0, -1.0, -1.0], dtype=np.float32)
        points_ned = self.drone_rotation.apply(points_frd)
        wx = points_ned[:, 0] + self.drone_x
        wy = points_ned[:, 1] + self.drone_y
        wz = -(points_ned[:, 2] + self.drone_z)

        gi = ((wx - self.grid_origin_x) / self.resolution).astype(np.int32)
        gj = ((wy - self.grid_origin_y) / self.resolution).astype(np.int32)
        in_bounds = ((gi >= 0) & (gi < self.grid_size) &
                     (gj >= 0) & (gj < self.grid_size))
        gi, gj, wz = gi[in_bounds], gj[in_bounds], wz[in_bounds]
        if len(gi) == 0:
            return

        # Every lateral ray landing in a cell counts as one "look", whether
        # or not it was tall enough to be an obstacle -- this is the
        # denominator for the ratio computed in _compute_occupied_mask.
        np.add.at(self.obs_seen, (gj, gi), 1)

        height_mask = (wz >= self.min_obstacle_height_m) & (wz <= self.max_obstacle_height_m)
        gi_hit, gj_hit, wz_hit = gi[height_mask], gj[height_mask], wz[height_mask]
        if len(gi_hit) > 0:
            np.maximum.at(self.obs_z_max, (gj_hit, gi_hit), wz_hit)
            np.add.at(self.obs_hits, (gj_hit, gi_hit), 1)

        if self.scan_count % 50 == 0:
            occupied = int(np.sum(self._compute_occupied_mask()))
            self.get_logger().info(
                f'Scan {self.scan_count}: {len(points)} lateral pts | '
                f'{occupied} obstacle cells | '
                f'drone ({self.drone_x:.1f}, {self.drone_y:.1f}, {-self.drone_z:.1f}m)')

    def _compute_occupied_mask(self):
        """Cells counted as obstacles: enough obstacle-height detections
        (not a single stray/quantised point) AND a high enough fraction
        of all the times the cell was looked at, so a cell that's mostly
        clear with a few noisy hits mixed in over many passes doesn't
        get promoted to a permanent obstacle just from repeated exposure.
        Never the drone's own current position (a small radius around it
        is always cleared, regardless of what the accumulated grid says).
        """
        with np.errstate(divide='ignore', invalid='ignore'):
            hit_ratio = np.where(self.obs_seen > 0,
                                  self.obs_hits / np.maximum(self.obs_seen, 1), 0.0)

        occupied = (np.isfinite(self.obs_z_max) &
                    (self.obs_hits >= self.min_obstacle_hits) &
                    (hit_ratio >= self.min_obstacle_ratio))

        gi_drone = int((self.drone_x - self.grid_origin_x) / self.resolution)
        gj_drone = int((self.drone_y - self.grid_origin_y) / self.resolution)
        radius_cells = int(np.ceil(self.safety_radius_m / self.resolution))
        i0 = max(0, gi_drone - radius_cells)
        i1 = min(self.grid_size, gi_drone + radius_cells + 1)
        j0 = max(0, gj_drone - radius_cells)
        j1 = min(self.grid_size, gj_drone + radius_cells + 1)
        if i1 > i0 and j1 > j0:
            occupied[j0:j1, i0:i1] = False

        return occupied

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
        occupied = self._compute_occupied_mask()
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

        occupied = self._compute_occupied_mask()
        js, is_ = np.where(occupied)
        for j, i in zip(js, is_):
            cx = self.grid_origin_x + (i + 0.5) * self.resolution
            cy = self.grid_origin_y + (j + 0.5) * self.resolution
            cz = float(self.obs_z_max[j, i])
            marker.points.append(Point(x=cx, y=cy, z=cz))

        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def _extract_xyz_vectorised(self, msg: PointCloud2):
        """Extract XYZ from PointCloud2, rejecting sentinel/garbage returns.

        Some rays that hit nothing within range come back as a huge
        sentinel value instead of NaN/Inf. np.isfinite() alone lets those
        through, which caused the overflow warnings and the invalid casts
        into grid indices downstream. MAX_SENSOR_RANGE rejects those here,
        via abs() only (no squaring), so a huge value can never overflow
        before rejection.

        Also rejects two other invalid-return patterns found by
        inspection: points essentially at the sensor origin (0,0,0), and
        points at the LiDAR's max-range "no echo" distance (observed as
        a repeated ~24-26m value) -- neither is a real obstacle hit. Note
        this checks total range, not z specifically: unlike the downward
        nodes, a real lateral obstacle can legitimately sit at close to
        the sensor's own height (z near zero), so z alone can't be used
        to spot a bad return here.
        """
        MAX_SENSOR_RANGE = 200.0  # metres, garbage/sentinel-value safety net
        MIN_VALID_RANGE = 0.3    # metres, rejects near-(0,0,0) empty returns
        MAX_VALID_RANGE = 22.0   # metres, rejects near-max-range no-echo sentinel

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
        prefilter = np.isfinite(points).all(axis=1) & (np.abs(points) < MAX_SENSOR_RANGE).all(axis=1)
        total_range = np.full(len(points), np.inf, dtype=np.float64)
        safe_points = points[prefilter].astype(np.float64)
        if len(safe_points) > 0:
            total_range[prefilter] = np.sqrt(np.sum(safe_points**2, axis=1))
        valid = (prefilter &
                 (total_range > MIN_VALID_RANGE) &
                 (total_range < MAX_VALID_RANGE))
        return points[valid]


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