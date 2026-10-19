"""
Trajectory executor: the only node that drives PX4 in offboard (spec S4).

Role: take off, fly the paths the exploration manager sends, hold, and
land, publishing its state at 10 Hz. It decides nothing: where to go and
when to land belong to the manager (spec 3.3).

Choices:
  - paths are flown on the calibrated rest-to-rest trapezoid
    (trajectory_profile.py, 5 m/s, 3 m/s2), position, velocity and
    acceleration together. The energy model holds only for this profile
  - a new path replaces the current one at once, mid-leg if need be
    (preemption, S4): it starts from the current setpoint, so the
    position setpoint does not jump; PX4 brakes within its own limits
  - arrival is the real horizontal distance to the end of the path,
    under 0.5 m, and a horizontal speed under 0.3 m/s (S4). Confusing
    the end of a profile with arrival made the old stack declare itself
    arrived without being there
  - the PX4 interface follows the trap catalogue: setpoints stream
    before offboard is requested, unused fields are NaN (trap 1),
    timestamps are the extrapolated PX4 clock (trap 3, 177 -> 0
    failsafes on 19 Oct), no arming before a valid pose (S3 rule 6),
    the land command is resent at 1 Hz until PX4 leaves offboard
"""

import math

from px4_msgs.msg import (OffboardControlMode, TimesyncStatus,
                          TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Header
from uav_core.trajectory_profile import TrapezoidPath
from uav_interfaces.msg import ExecutorCommand, ExecutorStatus, PathCommand

NAN3 = [float('nan')] * 3
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
S = ExecutorStatus


class TrajectoryExecutor(Node):
    """Offboard setpoints from paths and mode commands."""

    def __init__(self):
        super().__init__('trajectory_executor')
        self.declare_parameter('cruise_speed_ms', 5.0)
        self.declare_parameter('accel_ms2', 3.0)
        self.declare_parameter('arrive_xy_m', 0.5)
        self.declare_parameter('arrive_speed_ms', 0.3)
        self.declare_parameter('arrive_timeout_s', 15.0)
        self.v = self.get_parameter('cruise_speed_ms').value
        self.a = self.get_parameter('accel_ms2').value
        self.arrive_xy = self.get_parameter('arrive_xy_m').value
        self.arrive_speed = self.get_parameter('arrive_speed_ms').value
        self.arrive_timeout = self.get_parameter('arrive_timeout_s').value

        self.mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)
        self.status_pub = self.create_publisher(ExecutorStatus,
                                                '/executor/status', 10)
        self.create_subscription(TimesyncStatus, '/fmu/out/timesync_status',
                                 self.on_timesync, PX4_QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4',
                                 self.on_status, PX4_QOS)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_position, PX4_QOS)
        self.create_subscription(ExecutorCommand, '/executor/command',
                                 self.on_command, 10)
        self.create_subscription(PathCommand, '/executor/path',
                                 self.on_path, 10)

        self.px4_offset_us = None
        self.armed = self.offboard = False
        self.pos = None
        self.state = S.IDLE
        self.altitude = 8.0
        self.hold_ne = None
        self.path = None
        self.path_id = 0
        self.arrived = False
        self.t_state = self.clock_s()
        self.n = 0
        self.create_timer(0.1, self.tick)

    # ------------------------------------------------------------ inputs
    def on_timesync(self, msg):
        """Store the offset to the PX4 clock (1 Hz), extrapolate later."""
        local_us = self.get_clock().now().nanoseconds // 1000
        self.px4_offset_us = msg.timestamp - local_us

    def on_status(self, msg):
        """Track arming and offboard."""
        self.armed = msg.arming_state == 2
        self.offboard = msg.nav_state == 14

    def on_position(self, msg):
        """Keep the latest local position."""
        self.pos = msg

    def on_command(self, msg):
        """Mode commands from the manager."""
        if msg.command == ExecutorCommand.TAKEOFF and self.state in (
                S.IDLE, S.LANDED):
            if self.pos is None:
                self.get_logger().warn('takeoff refused: no pose yet')
                return
            self.altitude = float(msg.altitude_m)
            self.hold_ne = (self.pos.x, self.pos.y)
            self.set_state(S.TAKING_OFF)
        elif msg.command == ExecutorCommand.HOLD and self.pos is not None:
            if self.state in (S.FOLLOWING, S.HOLDING):
                self.hold_ne = (self.pos.x, self.pos.y)
                self.path = None
                self.set_state(S.HOLDING)
        elif msg.command == ExecutorCommand.LAND and self.state in (
                S.HOLDING, S.FOLLOWING, S.TAKING_OFF):
            if self.pos is not None:
                self.hold_ne = (self.pos.x, self.pos.y)
            self.path = None
            self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.set_state(S.LANDING)

    def on_path(self, msg):
        """Start a new path at once, replacing any current one."""
        if self.state not in (S.HOLDING, S.FOLLOWING):
            self.get_logger().warn(f'path {msg.id} refused in state '
                                   f'{self.state}')
            return
        pts = list(zip(msg.north, msg.east))
        if not pts:
            return
        start = self.current_setpoint_ne()
        pts = [start] + pts[1:] if len(pts) > 1 else [start, pts[0]]
        self.path = TrapezoidPath(pts, self.v, self.a)
        self.altitude = float(msg.altitude_m)
        self.path_id = msg.id
        self.arrived = False
        self.set_state(S.FOLLOWING)
        self.get_logger().info(
            f'path {msg.id}: {len(self.path.legs)} legs, '
            f'{self.path.length:.1f} m, {self.path.duration:.1f} s')

    # ------------------------------------------------------------ helpers
    def clock_s(self):
        """Node clock in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def px4_now(self):
        """Extrapolated PX4 time in microseconds, 0 before timesync."""
        if self.px4_offset_us is None:
            return 0
        return int(self.get_clock().now().nanoseconds // 1000
                   + self.px4_offset_us)

    def set_state(self, state):
        """Change state and restart the state clock."""
        self.state = state
        self.t_state = self.clock_s()

    def current_setpoint_ne(self):
        """Where the setpoint stands now, the start of a new path."""
        if self.state == S.FOLLOWING and self.path is not None:
            return self.path.at(self.clock_s() - self.t_state)[0]
        return self.hold_ne

    def cmd(self, command, p1=0.0, p2=0.0):
        """Send a VehicleCommand."""
        m = VehicleCommand()
        m.command, m.param1, m.param2 = command, p1, p2
        m.target_system = m.target_component = 1
        m.source_system = m.source_component = 1
        m.from_external = True
        m.timestamp = self.px4_now()
        self.cmd_pub.publish(m)

    def publish_setpoint(self, ne, vel=None, acc=None):
        """Position setpoint; NE feedforward or NaN, never zero (trap 1)."""
        mode = OffboardControlMode()
        mode.position = True
        mode.timestamp = self.px4_now()
        self.mode_pub.publish(mode)
        sp = TrajectorySetpoint()
        sp.position = [float(ne[0]), float(ne[1]), -self.altitude]
        sp.velocity = [vel[0], vel[1], 0.0] if vel is not None else NAN3
        sp.acceleration = [acc[0], acc[1], 0.0] if acc is not None else NAN3
        sp.jerk = NAN3
        sp.yaw = 0.0
        sp.yawspeed = float('nan')
        sp.timestamp = self.px4_now()
        self.sp_pub.publish(sp)

    def horizontal_distance(self, ne):
        """Horizontal distance from the aircraft to a point."""
        return math.hypot(self.pos.x - ne[0], self.pos.y - ne[1])

    def at_rest_over(self, ne):
        """Arrival test of S4: real distance and speed."""
        return (self.horizontal_distance(ne) < self.arrive_xy
                and math.hypot(self.pos.vx, self.pos.vy) < self.arrive_speed)

    # ------------------------------------------------------------ cycle
    def tick(self):
        """Stream setpoints, advance the state machine, publish status."""
        self.n += 1
        elapsed = self.clock_s() - self.t_state
        if self.state == S.TAKING_OFF:
            self.publish_setpoint(self.hold_ne)
            ready = (self.pos is not None and self.pos.xy_valid
                     and self.pos.z_valid and self.px4_offset_us is not None)
            if not (self.armed and self.offboard):
                if ready and elapsed > 2.0 and self.n % 10 == 0:
                    self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self.cmd(
                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            elif (abs(self.pos.z + self.altitude) < 0.3
                  and abs(self.pos.vz) < 0.2
                  and self.at_rest_over(self.hold_ne)):
                self.set_state(S.HOLDING)
                self.get_logger().info(f'holding at {self.altitude} m')
        elif self.state == S.HOLDING:
            self.publish_setpoint(self.hold_ne)
        elif self.state == S.FOLLOWING:
            ne, vel, acc = self.path.at(elapsed)
            if elapsed < self.path.duration:
                self.publish_setpoint(ne, vel, acc)
            else:
                self.publish_setpoint(self.path.end)
                timed_out = elapsed - self.path.duration > self.arrive_timeout
                if self.at_rest_over(self.path.end) or timed_out:
                    self.arrived = not timed_out
                    self.hold_ne = self.path.end
                    self.set_state(S.HOLDING)
                    if timed_out:
                        self.get_logger().warn(
                            f'path {self.path_id}: arrival timeout')
        elif self.state == S.LANDING:
            if not self.armed:
                self.set_state(S.LANDED)
            elif self.offboard:
                # PX4 has not taken the land command yet: keep streaming
                # so it does not fail over, and resend at 1 Hz.
                self.publish_setpoint(self.hold_ne)
                if self.n % 10 == 0:
                    self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.publish_status(elapsed)

    def publish_status(self, elapsed):
        """Publish the executor state."""
        msg = ExecutorStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.path_id = self.path_id
        msg.arrived = self.arrived
        msg.armed, msg.offboard = self.armed, self.offboard
        if self.path is not None and self.pos is not None:
            msg.distance_to_goal_m = self.horizontal_distance(self.path.end)
            msg.legs = len(self.path.legs)
            msg.leg = (self.path.leg_index(elapsed)
                       if self.state == S.FOLLOWING else msg.legs)
        self.status_pub.publish(msg)


def main(args=None):
    """Run the node until interrupted."""
    rclpy.init(args=args)
    node = TrajectoryExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
