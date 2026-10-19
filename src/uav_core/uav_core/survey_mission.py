"""Boustrophedon survey over the validation world.

Flies parallel lines at a fixed altitude, alternating direction, so the
lidar sweeps every test zone. Line spacing is set so neighbouring
swaths overlap: cells seen from two different lines get observations
from two different angles, which is what actually reduces map error.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
    TimesyncStatus,
)


def build_survey(length, spacing, n_lines, altitude):
    """Lawnmower pattern centred on the origin, lines along north.

    Returns NED waypoints: x north, y east, z down (negative up).
    """
    half = length / 2.0
    span = (n_lines - 1) * spacing
    waypoints = []

    for i in range(n_lines):
        y = -span / 2.0 + i * spacing
        if i % 2 == 0:
            waypoints.append((-half, y, -altitude))
            waypoints.append((+half, y, -altitude))
        else:
            waypoints.append((+half, y, -altitude))
            waypoints.append((-half, y, -altitude))

    waypoints.append((0.0, 0.0, -altitude))   # home before landing
    return waypoints


class SurveyMission(Node):
    def __init__(self):
        super().__init__('survey_mission')

        self.declare_parameter('accept_radius_xy', 1.5)
        self.declare_parameter('accept_radius_z', 0.8)
        self.declare_parameter('altitude', 8.0)
        self.declare_parameter('line_length', 50.0)
        self.declare_parameter('line_spacing', 15.0)
        self.declare_parameter('n_lines', 4)

        self.radius_xy = self.get_parameter('accept_radius_xy').value
        self.radius_z = self.get_parameter('accept_radius_z').value

        self.waypoints = build_survey(
            self.get_parameter('line_length').value,
            self.get_parameter('line_spacing').value,
            self.get_parameter('n_lines').value,
            self.get_parameter('altitude').value,
        )
        self.index = 0
        self.get_logger().info(f'{len(self.waypoints)} waypoints planned')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.create_subscription(
            TimesyncStatus, '/fmu/out/timesync_status', self.on_timesync, qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4', self.on_status, qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.on_position, qos)

        self.px4_offset_us = None
        self.position = None
        self.armed = False
        self.offboard = False

        self.state = 'ARMING'
        self.counter = 0
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timesync(self, msg):
        # timesync_status only arrives at 1 Hz, so publishing its raw
        # timestamp would stamp ten setpoints with the same value and
        # PX4 would treat the stream as stale. Store the offset instead
        # and extrapolate with the local clock between messages.
        local_us = self.get_clock().now().nanoseconds // 1000
        self.px4_offset_us = msg.timestamp - local_us

    def px4_now(self):
        if self.px4_offset_us is None:
            return 0
        return int(self.get_clock().now().nanoseconds // 1000
                   + self.px4_offset_us)

    def on_status(self, msg):
        self.armed = (msg.arming_state == 2)
        self.offboard = (msg.nav_state == 14)

    def on_position(self, msg):
        self.position = (msg.x, msg.y, msg.z)

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.px4_now()
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.velocity = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.jerk = [float('nan')] * 3
        msg.yaw = 0.0
        msg.yawspeed = float('nan')
        msg.timestamp = self.px4_now()
        self.setpoint_pub.publish(msg)

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
        msg.timestamp = self.px4_now()
        self.command_pub.publish(msg)

    def reached(self, target):
        if self.position is None:
            return False
        dx = self.position[0] - target[0]
        dy = self.position[1] - target[1]
        dz = self.position[2] - target[2]
        return (math.hypot(dx, dy) < self.radius_xy
                and abs(dz) < self.radius_z)

    def on_timer(self):
        self.counter += 1

        if self.state == 'LANDING':
            return

        self.publish_offboard_mode()
        self.publish_setpoint(*self.waypoints[self.index])

        if self.state == 'ARMING':
            if self.armed and self.offboard:
                self.state = 'FLYING'
                self.get_logger().info('Offboard active, starting survey')
            elif self.counter > 20 and self.counter % 10 == 0:
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.send_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            return

        if self.reached(self.waypoints[self.index]):
            if self.index + 1 < len(self.waypoints):
                self.index += 1
                wp = self.waypoints[self.index]
                self.get_logger().info(
                    f'waypoint {self.index}/{len(self.waypoints) - 1} '
                    f'-> N {wp[0]:+.0f} E {wp[1]:+.0f} '
                    f'| at N {self.position[0]:+.1f} '
                    f'E {self.position[1]:+.1f} '
                    f'D {self.position[2]:+.1f}')
            else:
                self.get_logger().info('Survey complete, landing')
                self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.state = 'LANDING'


def main(args=None):
    rclpy.init(args=args)
    node = SurveyMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()