#!/usr/bin/env python3
"""
frontier_detector_node.py

Detects exploration frontiers from the terrain observation grid and obstacle layer.
A frontier cell is a known free cell (z_count >= threshold) adjacent to an unknown
cell (z_count == 0) that is also navigable (no obstacle detected).

Frontier cells are grouped into clusters via flood-fill (8-connectivity).
Clusters smaller than min_cluster_size are discarded.
Each cluster centroid is scored by information gain: number of unknown cells
within LiDAR range from the centroid.

Subscribes:
  /terrain/z_count   (nav_msgs/OccupancyGrid) - raw observation count per cell
  /obstacle_grid     (nav_msgs/OccupancyGrid) - 100=occupied, -1=unknown, 0=free
  /fmu/out/vehicle_local_position_v1 (px4_msgs/VehicleLocalPosition)

Publishes:
  /frontiers         (uav_navigation_msgs/FrontierArray)
  /frontiers/markers (visualization_msgs/MarkerArray) - RViz2 visualisation

Author: Dorian Veloso
Capstone B - UTS 2026
Supervisor: Dr. Graeme Best
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Header
from uav_navigation_msgs.msg import FrontierCluster, FrontierArray
import numpy as np
from collections import deque


class FrontierDetectorNode(Node):

    def __init__(self):
        super().__init__('frontier_detector_node')

        # --- Grid parameters (must match terrain_scoring_node) ---
        self.resolution = 0.5
        self.grid_size = 200
        self.grid_origin_x = -50.0
        self.grid_origin_y = -50.0

        # --- Detection parameters ---
        # Minimum z_count for a cell to be considered known
        self.known_threshold = 3

        # Minimum number of cells in a cluster to be published
        self.min_cluster_size = 5

        # LiDAR range for information gain estimation (metres)
        self.lidar_range_m = 50.0
        self.lidar_range_cells = int(self.lidar_range_m / self.resolution)

        # Detect and publish rate (Hz)
        self.detect_rate = 1.0

        # --- State ---
        self.z_count_grid = None    # (grid_size, grid_size) int
        self.obstacle_grid = None   # (grid_size, grid_size) int8
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.position_valid = False

        # --- QoS ---
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Subscribers ---
        self.zcount_sub = self.create_subscription(
            OccupancyGrid, '/terrain/z_count',
            self.zcount_callback, reliable_qos
        )
        self.obstacle_sub = self.create_subscription(
            OccupancyGrid, '/obstacle_grid',
            self.obstacle_callback, reliable_qos
        )

        try:
            from px4_msgs.msg import VehicleLocalPosition
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5
            )
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

        # --- Publishers ---
        self.frontier_pub = self.create_publisher(
            FrontierArray, '/frontiers', reliable_qos
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/frontiers/markers', reliable_qos
        )

        # --- Detection timer ---
        self.create_timer(1.0 / self.detect_rate, self.detect_and_publish)

        self.get_logger().info(
            f'FrontierDetectorNode started | '
            f'known_threshold={self.known_threshold} | '
            f'min_cluster_size={self.min_cluster_size} | '
            f'lidar_range={self.lidar_range_m}m'
        )

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def position_callback(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.drone_x = msg.x
            self.drone_y = msg.y
            self.position_valid = True

    def zcount_callback(self, msg: OccupancyGrid):
        """Store z_count grid as numpy array."""
        data = np.array(msg.data, dtype=np.int32).reshape(
            (msg.info.height, msg.info.width)
        )
        self.z_count_grid = data

    def obstacle_callback(self, msg: OccupancyGrid):
        """Store obstacle grid as numpy array."""
        data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )
        self.obstacle_grid = data

    # -------------------------------------------------------------------------
    # Detection
    # -------------------------------------------------------------------------

    def detect_and_publish(self):
        """Main detection loop: find frontiers, cluster, score, publish."""
        if self.z_count_grid is None:
            self.get_logger().warn('Waiting for /terrain/z_count...', once=True)
            return
        if self.obstacle_grid is None:
            self.get_logger().warn('Waiting for /obstacle_grid...', once=True)
            return
        if not self.position_valid:
            self.get_logger().warn('Waiting for drone position...', once=True)
            return

        # --- Step 1: identify frontier cells ---
        # Known free: z_count >= threshold AND no obstacle
        known = self.z_count_grid >= self.known_threshold
        occupied = self.obstacle_grid == 100
        known_free = known & ~occupied

        # Unknown: z_count == 0
        unknown = self.z_count_grid == 0

        # Frontier: known_free cell adjacent to at least one unknown cell
        frontier_mask = self._compute_frontier_mask(known_free, unknown)

        if not np.any(frontier_mask):
            self.get_logger().info('No frontiers detected', once=True)
            self._publish_empty()
            return

        # --- Step 2: cluster frontier cells (flood-fill, 8-connectivity) ---
        clusters = self._cluster_frontiers(frontier_mask)

        if len(clusters) == 0:
            self._publish_empty()
            return

        # --- Step 3: score each cluster ---
        drone_gi = int((self.drone_x - self.grid_origin_x) / self.resolution)
        drone_gj = int((self.drone_y - self.grid_origin_y) / self.resolution)

        frontier_msgs = []
        for cells in clusters:
            cells_arr = np.array(cells)  # shape (N, 2): [[j, i], ...]

            # Mean position of the cluster
            mean_j = float(np.mean(cells_arr[:, 0]))
            mean_i = float(np.mean(cells_arr[:, 1]))

            # Snap to the actual cluster member closest to the mean. The
            # raw mean can land on a cell that was never part of the
            # cluster at all (e.g. an elongated or L-shaped cluster
            # wrapping around an obstacle), including an occupied one,
            # which astar_planner_node then rightly refuses forever since
            # nothing here would change on the next cycle. Every cluster
            # member is, by construction, known_free, so snapping to one
            # guarantees a valid target.
            dists_to_mean = (cells_arr[:, 0] - mean_j)**2 + (cells_arr[:, 1] - mean_i)**2
            nearest_idx = int(np.argmin(dists_to_mean))
            cj = float(cells_arr[nearest_idx, 0])
            ci = float(cells_arr[nearest_idx, 1])

            # Centroid in world coordinates
            cx = self.grid_origin_x + (ci + 0.5) * self.resolution
            cy = self.grid_origin_y + (cj + 0.5) * self.resolution

            # Information gain: unknown cells within LiDAR range of centroid
            info_gain = self._estimate_information_gain(int(cj), int(ci), unknown)

            # Distance from drone to centroid
            dist = float(np.sqrt(
                (drone_gi - ci)**2 + (drone_gj - cj)**2
            ) * self.resolution)

            # Filter: exclude frontiers at grid boundary (artefacts)
            margin_m = 2.0
            boundary = (self.grid_size * self.resolution) / 2.0 - margin_m
            if abs(cx) > boundary or abs(cy) > boundary:
                continue

            # Filter: exclude frontiers over water (no terrain returns at centroid)
            terrain_ci = int(ci)
            terrain_cj = int(cj)
            terrain_ci = np.clip(terrain_ci, 0, self.grid_size - 1)
            terrain_cj = np.clip(terrain_cj, 0, self.grid_size - 1)
            if self.z_count_grid[terrain_cj, terrain_ci] < self.known_threshold:
                continue

            fc = FrontierCluster()
            fc.centroid_x = float(cx)
            fc.centroid_y = float(cy)
            fc.cluster_size = int(len(cells))
            fc.information_gain = int(info_gain)
            fc.distance_to_drone = float(dist)
            frontier_msgs.append(fc)

        # --- Step 4: publish ---
        stamp = self.get_clock().now().to_msg()

        msg = FrontierArray()
        msg.header = Header(stamp=stamp, frame_id='map')
        msg.clusters = frontier_msgs
        self.frontier_pub.publish(msg)

        self._publish_markers(frontier_msgs, stamp)

        self.get_logger().info(
            f'{len(frontier_msgs)} frontier clusters | '
            f'best gain: {max(f.information_gain for f in frontier_msgs)}'
        )

    def _compute_frontier_mask(self, known_free, unknown):
        """
        A cell is a frontier if it is known_free AND has at least one
        unknown neighbour (8-connectivity).
        Uses numpy roll for efficient neighbour checking without loops.
        """
        # Dilate unknown by 1 cell in all 8 directions
        unknown_dilated = np.zeros_like(unknown)
        for dj in [-1, 0, 1]:
            for di in [-1, 0, 1]:
                if dj == 0 and di == 0:
                    continue
                unknown_dilated |= np.roll(np.roll(unknown, dj, axis=0), di, axis=1)

        return known_free & unknown_dilated

    def _cluster_frontiers(self, frontier_mask):
        """
        Group frontier cells into clusters via BFS flood-fill (8-connectivity).
        Returns list of clusters, each cluster is a list of (j, i) tuples.
        Clusters smaller than min_cluster_size are discarded.
        """
        visited = np.zeros_like(frontier_mask, dtype=bool)
        clusters = []

        js, is_ = np.where(frontier_mask)

        for start_j, start_i in zip(js, is_):
            if visited[start_j, start_i]:
                continue

            # BFS from this seed cell
            cluster = []
            queue = deque()
            queue.append((start_j, start_i))
            visited[start_j, start_i] = True

            while queue:
                j, i = queue.popleft()
                cluster.append((j, i))

                # 8-connected neighbours
                for dj in [-1, 0, 1]:
                    for di in [-1, 0, 1]:
                        if dj == 0 and di == 0:
                            continue
                        nj, ni = j + dj, i + di
                        if (0 <= nj < self.grid_size and
                                0 <= ni < self.grid_size and
                                not visited[nj, ni] and
                                frontier_mask[nj, ni]):
                            visited[nj, ni] = True
                            queue.append((nj, ni))

            if len(cluster) >= self.min_cluster_size:
                clusters.append(cluster)

        return clusters

    def _estimate_information_gain(self, cj, ci, unknown):
        """
        Count unknown cells within LiDAR range of centroid (ci, cj).
        Uses a bounding box + circular mask for efficiency.
        """
        r = self.lidar_range_cells

        j_min = max(0, cj - r)
        j_max = min(self.grid_size, cj + r + 1)
        i_min = max(0, ci - r)
        i_max = min(self.grid_size, ci + r + 1)

        sub = unknown[j_min:j_max, i_min:i_max]

        # Build circular mask relative to centroid
        js_local = np.arange(j_min, j_max) - cj
        is_local = np.arange(i_min, i_max) - ci
        JJ, II = np.meshgrid(js_local, is_local, indexing='ij')
        circle_mask = (JJ**2 + II**2) <= r**2

        return int(np.sum(sub & circle_mask))

    def _publish_empty(self):
        """Publish empty FrontierArray when no frontiers found."""
        msg = FrontierArray()
        msg.header = Header(
            stamp=self.get_clock().now().to_msg(), frame_id='map'
        )
        msg.clusters = []
        self.frontier_pub.publish(msg)

    # -------------------------------------------------------------------------
    # Visualisation
    # -------------------------------------------------------------------------

    def _publish_markers(self, clusters, stamp):
        """Publish frontier centroids as spheres + text labels for RViz2."""
        marker_array = MarkerArray()

        clear = Marker()
        clear.header = Header(stamp=stamp, frame_id='map')
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        for idx, fc in enumerate(clusters):
            # Sphere at centroid
            sphere = Marker()
            sphere.header = Header(stamp=stamp, frame_id='map')
            sphere.ns = 'frontier_centroids'
            sphere.id = idx
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = fc.centroid_x
            sphere.pose.position.y = fc.centroid_y
            sphere.pose.position.z = 2.0
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 1.0
            sphere.scale.y = 1.0
            sphere.scale.z = 1.0

            # Colour by information gain: blue (low) -> green (high)
            max_gain = 5000
            t = min(fc.information_gain / max_gain, 1.0)
            sphere.color = ColorRGBA(
                r=0.0, g=float(t), b=float(1.0 - t), a=0.9
            )
            marker_array.markers.append(sphere)

            # Text label: gain / size
            text = Marker()
            text.header = Header(stamp=stamp, frame_id='map')
            text.ns = 'frontier_labels'
            text.id = idx
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = fc.centroid_x
            text.pose.position.y = fc.centroid_y
            text.pose.position.z = 3.5
            text.pose.orientation.w = 1.0
            text.scale.z = 0.8
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = f'g={fc.information_gain} s={fc.cluster_size}'
            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()