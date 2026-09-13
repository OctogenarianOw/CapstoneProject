#!/usr/bin/env python3
"""
exploration_manager_node.py

Orchestrates autonomous exploration via a finite state machine.
Selects frontier targets, sends planning goals, monitors execution,
and triggers replanning on obstacle detection.

States: IDLE -> SELECTING -> NAVIGATING -> REPLANNING -> SELECTING

Subscribes:
  /frontiers                         (uav_navigation_msgs/FrontierArray)
  /watchdog_alert                    (uav_navigation_msgs/WatchdogAlert)
  /waypoint_reached                  (std_msgs/Bool)
  /fmu/out/vehicle_local_position_v1 (px4_msgs/VehicleLocalPosition)

Action client:
  /plan_path  (uav_navigation_msgs/action/PlanPath)

Publishes:
  /arrival_threshold  (std_msgs/Float32)

Author: Dorian Veloso
Capstone B - UTS 2026
Supervisor: Dr. Graeme Best
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from uav_navigation_msgs.msg import FrontierArray, WatchdogAlert
from uav_navigation_msgs.action import PlanPath
from std_msgs.msg import Float32, Bool
from enum import Enum, auto
import math


class State(Enum):
    IDLE        = auto()
    SELECTING   = auto()
    NAVIGATING  = auto()
    REPLANNING  = auto()


class ExplorationManagerNode(Node):

    def __init__(self):
        super().__init__('exploration_manager_node')

        # --- Parameters ---
        self.arrival_threshold_exploration = 2.0  # metres
        self.replan_cooldown_s = 2.0  # minimum seconds between replan attempts
        self.frontier_blacklist_duration_s = 30.0  # cooldown for a target
                                                    # that failed to plan,
                                                    # so it isn't retried
                                                    # every single tick

        # --- State machine ---
        self.state = State.IDLE
        self.current_goal_handle = None
        self.current_frontier = None
        self.last_replan_time = 0.0
        self.frontiers = []
        self.blacklisted_frontiers = {}  # (x,y) rounded -> blacklisted_until (s)
        self.frontier_too_close_counts = {}  # (x,y) rounded -> consecutive
                                              # times revisited without ever
                                              # resolving (still too close)
        self.frontier_giveup_threshold = 2       # after this many revisits
        self.frontier_giveup_duration_s = 300.0  # give up on it for 5 min,
                                                  # likely unreachable by the
                                                  # terrain sensor from any
                                                  # position the drone can
                                                  # safely reach nearby

        # --- Drone position ---
        self.drone_x = 0.0
        self.drone_y = 0.0
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
        self.frontier_sub = self.create_subscription(
            FrontierArray, '/frontiers',
            self.frontier_callback, reliable_qos
        )
        self.watchdog_sub = self.create_subscription(
            WatchdogAlert, '/watchdog_alert',
            self.watchdog_callback, reliable_qos
        )
        self.reached_sub = self.create_subscription(
            Bool, '/waypoint_reached',
            self.reached_callback, reliable_qos
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

        # --- Publishers ---
        self.threshold_pub = self.create_publisher(
            Float32, '/arrival_threshold', reliable_qos
        )

        # --- Action client ---
        self.plan_client = ActionClient(self, PlanPath, '/plan_path')

        # --- State machine timer: 1Hz ---
        self.create_timer(1.0, self.state_machine_tick)

        self.get_logger().info('ExplorationManagerNode started | state: IDLE')

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def position_callback(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.drone_x = msg.x
            self.drone_y = msg.y
            self.position_valid = True

    def frontier_callback(self, msg: FrontierArray):
        """Store latest frontier list."""
        self.frontiers = msg.clusters

    def watchdog_callback(self, msg: WatchdogAlert):
        """React to obstacle on path."""
        if not msg.path_blocked:
            return

        if self.state != State.NAVIGATING:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_replan_time < self.replan_cooldown_s:
            return

        self.get_logger().warn(
            f'Watchdog: path blocked at ({msg.obstacle_x:.1f}, '
            f'{msg.obstacle_y:.1f}) | {msg.blocked_cell_count} cells | '
            f'triggering replan'
        )

        self.last_replan_time = now
        self._cancel_current_goal()
        self.state = State.REPLANNING

    def reached_callback(self, msg: Bool):
        """React to the current partial path's final waypoint being reached.

        astar_planner_node plans with a receding horizon (~5m per plan,
        seen as 'horizon=5.0m' in its logs) even when a full-distance goal
        is requested. So this final waypoint may only be an intermediate
        point on the way to the frontier, not the frontier itself. Check
        the real remaining distance before declaring victory, otherwise
        the manager abandons a frontier it never actually reached and it
        just keeps reappearing in the frontier list.
        """
        if not msg.data:
            return

        if self.state != State.NAVIGATING:
            return

        if self.current_frontier is None:
            self.state = State.SELECTING
            return

        dist = math.sqrt(
            (self.drone_x - self.current_frontier.centroid_x)**2 +
            (self.drone_y - self.current_frontier.centroid_y)**2
        )

        if dist < self.arrival_threshold_exploration:
            self.get_logger().info(
                f'Frontier ({self.current_frontier.centroid_x:.1f}, '
                f'{self.current_frontier.centroid_y:.1f}) reached | '
                f'returning to SELECTING'
            )
            self.current_frontier = None
            self.state = State.SELECTING
        else:
            self.get_logger().info(
                f'Horizon segment complete, {dist:.1f}m still to frontier '
                f'({self.current_frontier.centroid_x:.1f}, '
                f'{self.current_frontier.centroid_y:.1f}) | continuing'
            )
            self._send_plan_goal(self.current_frontier)

    # -------------------------------------------------------------------------
    # State machine
    # -------------------------------------------------------------------------

    def state_machine_tick(self):
        """1Hz state machine tick."""

        if self.state == State.IDLE:
            self._tick_idle()

        elif self.state == State.SELECTING:
            self._tick_selecting()

        elif self.state == State.NAVIGATING:
            self._tick_navigating()

        elif self.state == State.REPLANNING:
            self._tick_replanning()

    def _tick_idle(self):
        if not self.position_valid:
            self.get_logger().info('IDLE: waiting for drone position...', once=True)
            return

        if len(self.frontiers) == 0:
            self.get_logger().info('IDLE: waiting for frontiers...', once=True)
            return

        self.get_logger().info('IDLE -> SELECTING')
        self.state = State.SELECTING

    def _tick_selecting(self):
        if len(self.frontiers) == 0:
            self.get_logger().info('SELECTING: no frontiers available, back to IDLE')
            self.state = State.IDLE
            return

        frontier = self._select_best_frontier()
        if frontier is None:
            self.get_logger().warn('SELECTING: no valid frontier found')
            return

        self.current_frontier = frontier
        self.get_logger().info(
            f'SELECTING: target frontier '
            f'({frontier.centroid_x:.1f}, {frontier.centroid_y:.1f}) | '
            f'gain={frontier.information_gain} | '
            f'dist={frontier.distance_to_drone:.1f}m'
        )

        # Set arrival threshold for exploration
        threshold_msg = Float32()
        threshold_msg.data = self.arrival_threshold_exploration
        self.threshold_pub.publish(threshold_msg)

        # Send planning goal
        self._send_plan_goal(frontier)
        self.state = State.NAVIGATING

    def _tick_navigating(self):
        """Check if current frontier is still in frontier list."""
        if self.current_frontier is None:
            self.state = State.SELECTING
            return

        # Periodic log
        if self.position_valid:
            dist = math.sqrt(
                (self.drone_x - self.current_frontier.centroid_x)**2 +
                (self.drone_y - self.current_frontier.centroid_y)**2
            )
            self.get_logger().info(
                f'NAVIGATING: {dist:.1f}m to frontier | '
                f'state={self.state.name}'
            )

    def _tick_replanning(self):
        """Replan from current position toward same frontier."""
        if self.current_frontier is None:
            self.state = State.SELECTING
            return

        self.get_logger().info(
            f'REPLANNING: re-sending goal to '
            f'({self.current_frontier.centroid_x:.1f}, '
            f'{self.current_frontier.centroid_y:.1f})'
        )
        self._send_plan_goal(self.current_frontier)
        self.state = State.NAVIGATING

    # -------------------------------------------------------------------------
    # Frontier selection
    # -------------------------------------------------------------------------

    def _frontier_key(self, fc):
        """Stable key for a frontier location, for blacklist lookups."""
        return (round(fc.centroid_x, 1), round(fc.centroid_y, 1))

    def _blacklist_frontier(self, fc, duration=None):
        """Exclude this frontier from selection for a cooldown period,
        so a target that fails to plan (e.g. it turned out to be
        occupied) isn't retried every single tick forever."""
        if duration is None:
            duration = self.frontier_blacklist_duration_s
        now = self.get_clock().now().nanoseconds / 1e9
        key = self._frontier_key(fc)
        self.blacklisted_frontiers[key] = now + duration
        self.get_logger().warn(
            f'Blacklisting frontier ({fc.centroid_x:.1f}, {fc.centroid_y:.1f}) '
            f'for {duration:.0f}s'
        )

    def _select_best_frontier(self):
        """
        Select frontier with best ratio: information_gain / distance_to_drone.
        Ignores frontiers with zero information gain or currently
        blacklisted (after a planning failure, or after being too close
        to the drone -- see below).

        A frontier within arrival_threshold_exploration is one the drone
        has already effectively visited. If frontier_detector_node hasn't
        cleared it from the list (e.g. the unknown cell that makes it a
        frontier sits just past what the terrain LiDAR's cone can reach
        even from directly overhead), it would otherwise be silently
        skipped every single tick forever with nothing else to select --
        blacklisting it instead gives it a cooldown, so this loops at most
        once per blacklist duration rather than once per tick.
        """
        now = self.get_clock().now().nanoseconds / 1e9
        best = None
        best_score = -1.0

        for fc in self.frontiers:
            key = self._frontier_key(fc)
            blacklisted_until = self.blacklisted_frontiers.get(key)
            if blacklisted_until is not None and now < blacklisted_until:
                continue

            if fc.distance_to_drone < self.arrival_threshold_exploration:
                key = self._frontier_key(fc)
                count = self.frontier_too_close_counts.get(key, 0) + 1
                self.frontier_too_close_counts[key] = count
                if count > self.frontier_giveup_threshold:
                    self.get_logger().warn(
                        f'Frontier ({fc.centroid_x:.1f}, {fc.centroid_y:.1f}) '
                        f'revisited {count} times without ever resolving -- '
                        f'likely unreachable by the terrain sensor from any '
                        f'nearby position, giving up on it for longer'
                    )
                    self._blacklist_frontier(fc, duration=self.frontier_giveup_duration_s)
                else:
                    self._blacklist_frontier(fc)
                continue
            if fc.information_gain <= 0:
                continue

            score = fc.information_gain / fc.distance_to_drone
            if score > best_score:
                best_score = score
                best = fc

        if best is None and len(self.frontiers) > 0:
            self._log_frontier_rejections(now)

        return best

    def _log_frontier_rejections(self, now):
        """Diagnostic: explain why every current frontier was rejected,
        so a prolonged 'no valid frontier found' can be traced to too-close
        / zero-gain / blacklisted rather than guessed at."""
        for fc in self.frontiers:
            reasons = []
            if fc.distance_to_drone < self.arrival_threshold_exploration:
                reasons.append(f'too close ({fc.distance_to_drone:.1f}m)')
            if fc.information_gain <= 0:
                reasons.append(f'zero gain ({fc.information_gain})')
            blacklisted_until = self.blacklisted_frontiers.get(self._frontier_key(fc))
            if blacklisted_until is not None and now < blacklisted_until:
                reasons.append(f'blacklisted, {blacklisted_until - now:.0f}s left')
            if reasons:
                self.get_logger().warn(
                    f'  rejected ({fc.centroid_x:.1f}, {fc.centroid_y:.1f}): '
                    f'{", ".join(reasons)}'
                )

    # -------------------------------------------------------------------------
    # Action client
    # -------------------------------------------------------------------------

    def _send_plan_goal(self, frontier):
        """Send PlanPath goal to astar_planner_node."""
        if not self.plan_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('PlanPath action server not available')
            self.state = State.IDLE
            return

        goal = PlanPath.Goal()
        goal.start_x = self.drone_x
        goal.start_y = self.drone_y
        goal.target_x = frontier.centroid_x
        goal.target_y = frontier.centroid_y
        goal.planning_horizon_m = 0.0  # 0 = use braking distance

        send_goal_future = self.plan_client.send_goal_async(
            goal,
            feedback_callback=self._plan_feedback_callback
        )
        send_goal_future.add_done_callback(self._plan_goal_response_callback)

    def _plan_feedback_callback(self, feedback_msg):
        progress = feedback_msg.feedback.progress
        self.get_logger().info(f'Planning progress: {progress*100:.0f}%', once=False)

    def _plan_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('PlanPath goal rejected')
            if self.current_frontier is not None:
                self._blacklist_frontier(self.current_frontier)
            self.state = State.SELECTING
            return

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._plan_result_callback)

    def _plan_result_callback(self, future):
        result = future.result().result
        if not result.success:
            self.get_logger().warn(f'Planning failed: {result.message}')
            if self.current_frontier is not None:
                self._blacklist_frontier(self.current_frontier)
            self.state = State.SELECTING
            return

        self.get_logger().info(
            f'Path received: {len(result.path.poses)} waypoints'
        )

    def _cancel_current_goal(self):
        """Cancel active planning goal if any."""
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()