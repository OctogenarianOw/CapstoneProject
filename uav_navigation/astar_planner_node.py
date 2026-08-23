#!/usr/bin/env python3
"""
astar_planner_node.py

Implements the PlanPath action server. Computes a 2D A* path on the obstacle
grid from current drone position to a frontier centroid, up to a sliding
planning horizon derived from current drone speed.

The internal planning algorithm is fully encapsulated behind the PlanPath
action interface. Replacing A* with another planner requires only modifying
this file.

Action server:
  /plan_path  (uav_navigation_msgs/action/PlanPath)

Subscribes:
  /obstacle_grid                     (nav_msgs/OccupancyGrid)
  /fmu/out/vehicle_local_position_v1 (px4_msgs/VehicleLocalPosition)

Publishes:
  /current_path  (nav_msgs/Path) - active path for watchdog and waypoint_node

Author: Dorian Veloso
Capstone B - UTS 2026
Supervisor: Dr. Graeme Best
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from uav_navigation_msgs.action import PlanPath
import numpy as np
import heapq
import math


class AStarPlannerNode(Node):

    def __init__(self):
        super().__init__('astar_planner_node')

        # --- Grid parameters (must match terrain_scoring_node) ---
        self.resolution = 0.5
        self.grid_size = 200
        self.grid_origin_x = -50.0
        self.grid_origin_y = -50.0

        # --- Planning parameters ---
        # Maximum drone deceleration (m/s²), matches PX4 MPC_ACC_HOR default
        self.a_max = 3.0

        # Minimum planning horizon (metres) regardless of speed
        self.min_horizon_m = 5.0

        # Safety margin around obstacles (cells). Inflates obstacle cells
        # before planning to keep the drone away from edges.
        self.obstacle_inflation_cells = 1

        # --- State ---
        self.obstacle_grid = None   # (grid_size, grid_size) int8
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_vx = 0.0
        self.drone_vy = 0.0
        self.position_valid = False

        # --- QoS ---
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # --- Subscribers ---
        self.obstacle_sub = self.create_subscription(
            OccupancyGrid, '/obstacle_grid',
            self.obstacle_callback, reliable_qos
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
        self.path_pub = self.create_publisher(Path, '/current_path', reliable_qos)

        # --- Action server ---
        self.action_server = ActionServer(
            self,
            PlanPath,
            '/plan_path',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info(
            f'AStarPlannerNode started | '
            f'inflation={self.obstacle_inflation_cells} cells | '
            f'a_max={self.a_max} m/s²'
        )

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def position_callback(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.drone_x = msg.x
            self.drone_y = msg.y
            self.drone_vx = msg.vx
            self.drone_vy = msg.vy
            self.position_valid = True

    def obstacle_callback(self, msg: OccupancyGrid):
        data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )
        self.obstacle_grid = data

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f'Goal received: target ({goal_request.target_x:.1f}, '
            f'{goal_request.target_y:.1f}) '
            f'horizon {goal_request.planning_horizon_m:.1f}m'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Plan cancelled')
        return CancelResponse.ACCEPT

    # -------------------------------------------------------------------------
    # Action execution
    # -------------------------------------------------------------------------

    async def execute_callback(self, goal_handle):
        """Execute A* planning and return result."""
        goal = goal_handle.request

        # --- Validate inputs ---
        if self.obstacle_grid is None:
            self.get_logger().warn('No obstacle grid available')
            goal_handle.abort()
            result = PlanPath.Result()
            result.success = False
            result.message = 'no_obstacle_grid'
            return result

        if not self.position_valid:
            self.get_logger().warn('No drone position available')
            goal_handle.abort()
            result = PlanPath.Result()
            result.success = False
            result.message = 'no_position'
            return result

        # --- Compute planning horizon ---
        v = math.sqrt(self.drone_vx**2 + self.drone_vy**2)
        d_braking = max(v**2 / (2.0 * self.a_max), self.min_horizon_m)

        # Use goal horizon if provided, otherwise use braking distance
        horizon_m = goal.planning_horizon_m if goal.planning_horizon_m > 0 else d_braking
        horizon_cells = int(horizon_m / self.resolution)

        self.get_logger().info(
            f'Planning | v={v:.1f}m/s | horizon={horizon_m:.1f}m '
            f'({horizon_cells} cells)'
        )

        # --- Convert world coordinates to grid indices ---
        start_gi = int((goal.start_x - self.grid_origin_x) / self.resolution)
        start_gj = int((goal.start_y - self.grid_origin_y) / self.resolution)
        target_gi = int((goal.target_x - self.grid_origin_x) / self.resolution)
        target_gj = int((goal.target_y - self.grid_origin_y) / self.resolution)

        # Clamp to grid bounds
        start_gi = np.clip(start_gi, 0, self.grid_size - 1)
        start_gj = np.clip(start_gj, 0, self.grid_size - 1)
        target_gi = np.clip(target_gi, 0, self.grid_size - 1)
        target_gj = np.clip(target_gj, 0, self.grid_size - 1)

        # --- Check start and target validity ---
        inflated = self._inflate_obstacles(self.obstacle_grid)

        if inflated[start_gj, start_gi] == 100:
            self.get_logger().warn('Start cell occupied, searching nearest free cell...')
            found = False
            for radius in range(1, 6):
                for dj in range(-radius, radius + 1):
                    for di in range(-radius, radius + 1):
                        nj = np.clip(start_gj + dj, 0, self.grid_size - 1)
                        ni = np.clip(start_gi + di, 0, self.grid_size - 1)
                        if inflated[nj, ni] != 100:
                            start_gj, start_gi = nj, ni
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
            if not found:
                goal_handle.abort()
                result = PlanPath.Result()
                result.success = False
                result.message = 'start_invalid'
                return result
            self.get_logger().info(f'Nearest free start: ({start_gi}, {start_gj})')

        if inflated[target_gj, target_gi] == 100:
            self.get_logger().warn('Target cell is occupied')
            goal_handle.abort()
            result = PlanPath.Result()
            result.success = False
            result.message = 'target_invalid'
            return result

        # --- Publish feedback: planning started ---
        feedback = PlanPath.Feedback()
        feedback.progress = 0.1
        goal_handle.publish_feedback(feedback)

        # --- Run A* ---
        path_cells = self._astar(
            inflated,
            (start_gj, start_gi),
            (target_gj, target_gi),
            horizon_cells
        )

        feedback.progress = 0.9
        goal_handle.publish_feedback(feedback)

        if path_cells is None:
            self.get_logger().warn('A* found no path')
            goal_handle.abort()
            result = PlanPath.Result()
            result.success = False
            result.message = 'no_path_found'
            return result

        # --- Convert path cells to nav_msgs/Path ---
        ros_path = self._cells_to_path(path_cells)

        # Publish on /current_path for watchdog and waypoint_node
        self.path_pub.publish(ros_path)

        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()

        result = PlanPath.Result()
        result.path = ros_path
        result.success = True
        result.message = 'ok'

        self.get_logger().info(
            f'Path found: {len(path_cells)} waypoints'
        )
        return result

    # -------------------------------------------------------------------------
    # A* algorithm
    # -------------------------------------------------------------------------

    def _astar(self, grid, start, target, horizon_cells):
        """
        A* on 2D grid. start and target are (j, i) tuples.
        Stops when target is reached OR when path length exceeds horizon_cells.
        Returns list of (j, i) tuples from start to furthest reachable point
        within horizon, or None if no path exists.

        Uses 8-connectivity with diagonal cost = sqrt(2).
        """
        gj, gi = start
        tj, ti = target

        # Priority queue: (f_cost, g_cost, j, i)
        open_set = []
        heapq.heappush(open_set, (0.0, 0.0, gj, gi))

        came_from = {}
        g_cost = {start: 0.0}
        best_node = start
        best_g = 0.0

        while open_set:
            f, g, j, i = heapq.heappop(open_set)

            # Track furthest reachable node within horizon
            if g <= horizon_cells and g > best_g:
                best_g = g
                best_node = (j, i)

            # Goal reached
            if (j, i) == (tj, ti):
                return self._reconstruct_path(came_from, (tj, ti))

            # Beyond horizon: stop expanding but keep best node
            if g > horizon_cells:
                continue

            # Expand neighbours (8-connectivity)
            for dj in [-1, 0, 1]:
                for di in [-1, 0, 1]:
                    if dj == 0 and di == 0:
                        continue

                    nj, ni = j + dj, i + di

                    if not (0 <= nj < self.grid_size and 0 <= ni < self.grid_size):
                        continue

                    if grid[nj, ni] == 100:
                        continue

                    # Diagonal moves cost sqrt(2), cardinal cost 1
                    move_cost = math.sqrt(2) if (dj != 0 and di != 0) else 1.0
                    new_g = g + move_cost

                    neighbour = (nj, ni)
                    if new_g < g_cost.get(neighbour, float('inf')):
                        g_cost[neighbour] = new_g
                        came_from[neighbour] = (j, i)
                        h = math.sqrt((nj - tj)**2 + (ni - ti)**2)
                        heapq.heappush(open_set, (new_g + h, new_g, nj, ni))

        # No path to target: return path to furthest reachable node
        if best_node != start:
            return self._reconstruct_path(came_from, best_node)

        return None

    def _reconstruct_path(self, came_from, current):
        """Reconstruct path from came_from dict."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------

    def _inflate_obstacles(self, grid):
        """
        Inflate obstacle cells by obstacle_inflation_cells using binary dilation.
        Returns inflated grid copy.
        """
        inflated = grid.copy()
        occupied = grid == 100
        r = self.obstacle_inflation_cells

        for dj in range(-r, r + 1):
            for di in range(-r, r + 1):
                if dj == 0 and di == 0:
                    continue
                shifted = np.roll(np.roll(occupied, dj, axis=0), di, axis=1)
                inflated[shifted] = 100

        return inflated

    def _cells_to_path(self, cells):
        """Convert list of (j, i) grid cells to nav_msgs/Path in world frame."""
        path = Path()
        path.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id='map'
        )
        for j, i in cells:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = self.grid_origin_x + (i + 0.5) * self.resolution
            pose.pose.position.y = self.grid_origin_y + (j + 0.5) * self.resolution
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
