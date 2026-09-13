#!/usr/bin/env python3
"""
waypoint_node.py

Handles PX4 arm/offboard sequence and executes the active path from
/current_path waypoint by waypoint. Altitude is driven by /cruise_altitude
(terrain follower). Arrival threshold is configurable per context
(exploration vs SLZ approach).

Subscribes:
  /current_path                      (nav_msgs/Path)
  /cruise_altitude                   (std_msgs/Float32)  NED z target
  /arrival_threshold                 (std_msgs/Float32)  metres
  /fmu/out/vehicle_local_position_v1 (px4_msgs/VehicleLocalPosition)

Publishes:
  /fmu/in/offboard_control_mode  (px4_msgs/OffboardControlMode)
  /fmu/in/trajectory_setpoint    (px4_msgs/TrajectorySetpoint)
  /fmu/in/vehicle_command        (px4_msgs/VehicleCommand)
  /waypoint_reached              (std_msgs/Bool) - true when current wp reached

Author: Dorian Veloso
Capstone B - UTS 2026
Supervisor: Dr. Graeme Best
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition)
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Bool
import math


class WaypointNode(Node):

    def __init__(self):
        super().__init__('waypoint_node')

        # --- Default parameters ---
        # Arrival threshold for exploration (frontier navigation)
        self.arrival_threshold_exploration = 2.0

        # Arrival threshold for SLZ approach
        self.arrival_threshold_slz = 0.5

        # Current active threshold (set by manager via /arrival_threshold)
        self.arrival_threshold = self.arrival_threshold_exploration

        # Default cruise altitude NED (used before terrain follower sends data)
        # -8.0 = 8m above origin in NED frame
        self.cruise_altitude_ned = -8.0

        # The altitude setpoint actually sent to PX4, rate-limited so a
        # noisy terrain estimate can't command an abrupt multi-metre climb
        # or dive from one 10Hz cycle to the next.
        self.commanded_altitude_ned = -8.0
        self.max_altitude_rate_mps = 2.0  # metres/second, generous but bounded

        # --- State ---
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.px4_ready = False
        self.armed = False
        self.count = 0

        # Active path: list of (x, y) world coordinates
        self.path = []
        self.current_wp_index = 0

        # --- QoS ---
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- PX4 Publishers ---
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos
        )

        # --- Navigation publisher ---
        self.reached_pub = self.create_publisher(
            Bool, '/waypoint_reached', reliable_qos
        )

        # --- Subscribers ---
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            px4_qos
        )
        self.path_sub = self.create_subscription(
            Path, '/current_path',
            self.path_callback, reliable_qos
        )
        self.altitude_sub = self.create_subscription(
            Float32, '/cruise_altitude',
            self.altitude_callback, reliable_qos
        )
        self.threshold_sub = self.create_subscription(
            Float32, '/arrival_threshold',
            self.threshold_callback, reliable_qos
        )

        # --- Timer: 10Hz control loop ---
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'WaypointNode started | '
            f'default altitude {self.cruise_altitude_ned}m NED | '
            f'exploration threshold {self.arrival_threshold_exploration}m | '
            f'SLZ threshold {self.arrival_threshold_slz}m | '
            f'max altitude rate {self.max_altitude_rate_mps}m/s'
        )

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def position_callback(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.drone_x = msg.x
            self.drone_y = msg.y
            self.drone_z = msg.z
            if not self.px4_ready:
                self.px4_ready = True
                self.count = 0
                self.get_logger().info('PX4 ready, starting offboard sequence...')

    def path_callback(self, msg: Path):
        """Receive new path and reset waypoint index."""
        new_path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]
        if len(new_path) == 0:
            return

        self.path = new_path
        self.current_wp_index = 0
        self.get_logger().info(
            f'New path received: {len(self.path)} waypoints'
        )

    def altitude_callback(self, msg: Float32):
        """Update cruise altitude from terrain follower."""
        self.cruise_altitude_ned = msg.data

    def threshold_callback(self, msg: Float32):
        """Update arrival threshold from exploration manager."""
        self.arrival_threshold = msg.data
        self.get_logger().info(
            f'Arrival threshold updated: {self.arrival_threshold}m'
        )

    # -------------------------------------------------------------------------
    # Control loop
    # -------------------------------------------------------------------------

    def control_loop(self):
        """10Hz loop: arm/offboard sequence then waypoint following."""
        if not self.px4_ready:
            return

        # --- Always publish OffboardControlMode to keep offboard active ---
        ocm = OffboardControlMode()
        ocm.position = True
        ocm.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(ocm)

        # --- Arm and switch to offboard mode ---
        self.count += 1
        if self.count == 20:
            self.get_logger().info('Switching to offboard mode...')
            self.send_command(176, 1.0, 6.0)
        if self.count == 30:
            self.get_logger().info('Arming...')
            self.send_command(400, 1.0, 0.0)
            self.armed = True

        # --- Waypoint following ---
        target_x, target_y = self._get_current_target()

        # Step the commanded altitude toward the raw terrain-follower
        # target by at most max_altitude_rate_mps, so a noisy estimate
        # can't demand an abrupt climb/dive in a single 10Hz tick.
        max_step = self.max_altitude_rate_mps * 0.1  # dt = 1/10Hz
        delta = self.cruise_altitude_ned - self.commanded_altitude_ned
        if delta > max_step:
            self.commanded_altitude_ned += max_step
        elif delta < -max_step:
            self.commanded_altitude_ned -= max_step
        else:
            self.commanded_altitude_ned = self.cruise_altitude_ned

        self._send_trajectory_setpoint(target_x, target_y)

        # --- Check arrival ---
        if self.armed and len(self.path) > 0:
            self._check_arrival()

    def _get_current_target(self):
        """Return current waypoint coordinates, or hold position if no path."""
        if len(self.path) == 0 or self.current_wp_index >= len(self.path):
            # Hold current position
            return self.drone_x, self.drone_y

        return self.path[self.current_wp_index]

    def _send_trajectory_setpoint(self, target_x, target_y):
        """Send TrajectorySetpoint to PX4."""
        wp = TrajectorySetpoint()
        wp.position = [target_x, target_y, self.commanded_altitude_ned]
        wp.yaw = float('nan')   # Let PX4 handle yaw
        wp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(wp)

    def _check_arrival(self):
        """Check if drone reached current waypoint, advance to next if so."""
        if self.current_wp_index >= len(self.path):
            return

        wp_x, wp_y = self.path[self.current_wp_index]
        dist = math.sqrt(
            (self.drone_x - wp_x)**2 +
            (self.drone_y - wp_y)**2
        )

        if dist < self.arrival_threshold:
            self.current_wp_index += 1

            if self.current_wp_index >= len(self.path):
                # Final waypoint reached
                self.get_logger().info('Final waypoint reached')
                reached = Bool()
                reached.data = True
                self.reached_pub.publish(reached)
            else:
                self.get_logger().info(
                    f'Waypoint {self.current_wp_index}/{len(self.path)} reached'
                )

    # -------------------------------------------------------------------------
    # PX4 helpers
    # -------------------------------------------------------------------------

    def send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)


def main():
    rclpy.init()
    node = WaypointNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()