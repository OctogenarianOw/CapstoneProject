#!/usr/bin/env python3
"""Scratch probe: take off, hold (0, 0, -ALT) for HOLD seconds, land.

Same PX4 interface as survey_mission (timesync offset, NaN fields), so the
offboard stream stays valid. Not part of the package.

    python3 hover_hold.py ALT HOLD
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleStatus,
                          VehicleLocalPosition, TimesyncStatus)

ALT = float(sys.argv[1])
HOLD = float(sys.argv[2])


class Hover(Node):
    def __init__(self):
        super().__init__('hover_hold')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.create_subscription(TimesyncStatus, '/fmu/out/timesync_status',
                                 self.on_timesync, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4',
                                 self.on_status, qos)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_position, qos)
        self.offset = None
        self.armed = self.offboard = False
        self.z = None
        self.state = 'ARMING'
        self.n = 0
        self.hold_ticks = 0
        self.create_timer(0.1, self.tick)

    def on_timesync(self, m):
        self.offset = m.timestamp - self.get_clock().now().nanoseconds // 1000

    def now(self):
        if self.offset is None:
            return 0
        return int(self.get_clock().now().nanoseconds // 1000 + self.offset)

    def on_status(self, m):
        self.armed = m.arming_state == 2
        self.offboard = m.nav_state == 14

    def on_position(self, m):
        self.z = m.z

    def cmd(self, c, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command, m.param1, m.param2 = c, p1, p2
        m.target_system = m.target_component = 1
        m.source_system = m.source_component = 1
        m.from_external = True
        m.timestamp = self.now()
        self.cmd_pub.publish(m)

    def tick(self):
        self.n += 1
        if self.state == 'LANDING':
            return
        m = OffboardControlMode()
        m.position = True
        m.timestamp = self.now()
        self.mode_pub.publish(m)
        sp = TrajectorySetpoint()
        sp.position = [0.0, 0.0, -ALT]
        sp.velocity = [float('nan')] * 3
        sp.acceleration = [float('nan')] * 3
        sp.jerk = [float('nan')] * 3
        sp.yaw = 0.0
        sp.yawspeed = float('nan')
        sp.timestamp = self.now()
        self.sp_pub.publish(sp)

        if self.state == 'ARMING':
            if self.armed and self.offboard:
                self.state = 'CLIMB'
            elif self.n > 20 and self.n % 10 == 0:
                self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        elif self.state == 'CLIMB':
            if self.z is not None and abs(self.z + ALT) < 0.3:
                self.state = 'HOLD'
                self.get_logger().info('holding')
        elif self.state == 'HOLD':
            self.hold_ticks += 1
            if self.hold_ticks >= HOLD * 10:
                self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.state = 'LANDING'
                self.get_logger().info('landing')


rclpy.init()
rclpy.spin(Hover())
