"""Minimal test of the offboard control chain: arm, offboard, hold 5 m.

Kept as the reference for the PX4 interface (spec S3): if this node
cannot hold offboard, nothing built on top of it can.

Every message is stamped with px4_now(), the local clock plus the
offset to the PX4 clock measured at each timesync_status. The rejected
alternative, copying the raw timesync timestamp, stamps ten setpoints
in a row with the same value because timesync only arrives at 1 Hz, and
PX4 then treats the stream as stale. Measured on 17 Oct 2026 with that
alternative: 177 failsafe activations in a 218 s hover, offboard
flipping to RTL about once per second
(experiments/2026-10-17_energy_signal, run1).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    TimesyncStatus,
)


class OffboardDemo(Node):
    def __init__(self):
        super().__init__('offboard_demo')

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

        self.px4_offset_us = None
        self.timesync_sub = self.create_subscription(
            TimesyncStatus, '/fmu/out/timesync_status', self.on_timesync, qos)

        self.counter = 0
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timesync(self, msg):
        # timesync_status only arrives at 1 Hz: store the offset to the
        # PX4 clock and extrapolate with the local clock in between.
        local_us = self.get_clock().now().nanoseconds // 1000
        self.px4_offset_us = msg.timestamp - local_us

    def px4_now(self):
        if self.px4_offset_us is None:
            return 0
        return int(self.get_clock().now().nanoseconds // 1000
                   + self.px4_offset_us)

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
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
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

    def on_timer(self):
        self.publish_offboard_mode()
        self.publish_setpoint(0.0, 0.0, -5.0)

        if self.counter == 20:
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info('Offboard requested, arming')

        if self.counter < 1000:
            self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = OffboardDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()