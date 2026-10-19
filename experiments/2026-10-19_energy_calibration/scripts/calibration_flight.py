#!/usr/bin/env python3
"""Calibration flight for the energy model of spec S10.

Takes off to ALT, holds HOLD seconds, flies one closed polygon, holds
POST seconds, and lands. Every phase change is published on
/calibration/phase, so the probe stamps phases, rotor power and
position on one clock.

Speed: each leg follows a trapezoidal profile, ACCEL up to SPEED,
cruise, ACCEL down to rest at the corner, sent as position, velocity
and acceleration together so PX4 tracks it instead of chasing it. The
cruise speed is then fixed by construction, the flown path matches the
planned one, and the planned time of a leg is exact from the plan,
L / v + v / a when the leg is long enough to reach v. That last point
is what C1 needs: a time it can compute before flying.

Rejected, measured in pilot1_carrot_L10_h10: a carrot advancing at
constant SPEED with velocity feedforward only. The aircraft starts
behind it, overtakes to catch up and overshoots every 90 degree turn:
4.3 to 7.9 m/s during cruise, 3 m past each corner, 56.3 m flown on a
40 m square. Neither the fixed speed of S10 nor the 5 % agreement
between planned and flown length of S12 holds that way.

Hover holds of different length in each repetition are part of the
design, not padding. At a single speed, transit time follows from
distance, so transit segments alone make the two regressors nearly
collinear and only k_eff = P_hover / v + k_move is identifiable.
Hover segments, with time and no distance, separate P_hover from
k_move.

Arrival at the end of the path is the real horizontal distance to the
final point and the horizontal speed (spec S4), so the final settling
belongs to the transit segment, as it would in a real transit.

    python3 calibration_flight.py --path "0,0;10,0;10,10;0,10;0,0" \
        --speed 5 --accel 3 --hold 20 --post 5 --alt 8
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from std_msgs.msg import String
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleStatus,
                          VehicleLocalPosition, TimesyncStatus)

NAN3 = [float('nan')] * 3
ARRIVE_XY_M = 0.5
ARRIVE_SPEED_MS = 0.3
CLIMB_TOL_M = 0.3
CLIMB_VZ_MS = 0.2
ARRIVE_TIMEOUT_S = 15.0


def parse_path(text):
    pts = []
    for pair in text.split(';'):
        n, e = pair.split(',')
        pts.append((float(n), float(e)))
    return pts


def leg_profile(length, v, a):
    """Peak speed, ramp time and total time of a rest-to-rest leg."""
    v_peak = min(v, math.sqrt(a * length))
    t_ramp = v_peak / a
    d_ramp = 0.5 * v_peak * t_ramp
    t_cruise = (length - 2.0 * d_ramp) / v_peak if v_peak > 0 else 0.0
    return v_peak, t_ramp, 2.0 * t_ramp + t_cruise


def planned_transit_time(pts, v, a):
    total = 0.0
    for (n0, e0), (n1, e1) in zip(pts[:-1], pts[1:]):
        total += leg_profile(math.hypot(n1 - n0, e1 - e0), v, a)[2]
    return total


class TrapezoidPath:
    """Rest-to-rest trapezoidal profile on each leg of a polyline."""

    def __init__(self, pts, v, a):
        self.pts = pts
        self.a = a
        self.legs = []
        t0 = 0.0
        for (n0, e0), (n1, e1) in zip(pts[:-1], pts[1:]):
            length = math.hypot(n1 - n0, e1 - e0)
            if length == 0.0:
                continue
            v_peak, t_ramp, t_leg = leg_profile(length, v, a)
            u = ((n1 - n0) / length, (e1 - e0) / length)
            self.legs.append(dict(start=(n0, e0), u=u, length=length,
                                  v=v_peak, t_ramp=t_ramp, t0=t0,
                                  t_leg=t_leg))
            t0 += t_leg
        self.duration = t0
        self.length = sum(leg['length'] for leg in self.legs)

    def at(self, t):
        """Position, velocity and acceleration (NE) at time t."""
        if t >= self.duration:
            last = self.legs[-1]
            n, e = (last['start'][0] + last['u'][0] * last['length'],
                    last['start'][1] + last['u'][1] * last['length'])
            return (n, e), (0.0, 0.0), (0.0, 0.0)
        leg = next(g for g in self.legs if t < g['t0'] + g['t_leg'])
        tau = t - leg['t0']
        a, vp, tr, tl = self.a, leg['v'], leg['t_ramp'], leg['t_leg']
        if tau < tr:
            s, sv, sa = 0.5 * a * tau * tau, a * tau, a
        elif tau < tl - tr:
            s, sv, sa = 0.5 * vp * tr + vp * (tau - tr), vp, 0.0
        else:
            rem = tl - tau
            s, sv, sa = leg['length'] - 0.5 * a * rem * rem, a * rem, -a
        un, ue = leg['u']
        return ((leg['start'][0] + un * s, leg['start'][1] + ue * s),
                (un * sv, ue * sv), (un * sa, ue * sa))


class CalibrationFlight(Node):
    def __init__(self, args):
        super().__init__('calibration_flight')
        self.args = args
        pts = parse_path(args.path)
        self.path = TrapezoidPath(pts, args.speed, args.accel)
        self.start = pts[0]
        self.end = pts[-1]

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.phase_pub = self.create_publisher(
            String, '/calibration/phase', 10)
        self.create_subscription(TimesyncStatus, '/fmu/out/timesync_status',
                                 self.on_timesync, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4',
                                 self.on_status, qos)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_position, qos)

        self.px4_offset_us = None
        self.armed = self.offboard = False
        self.pos = None
        self.state = 'WAIT_POSE'
        self.n = 0
        self.t_state = self.clock_s()
        self.done = False
        self.get_logger().info(
            f'path {args.path} length {self.path.length:.1f} m, '
            f'speed {args.speed} m/s, accel {args.accel} m/s2, '
            f'planned transit {self.path.duration:.2f} s, '
            f'hold {args.hold} s, post {args.post} s, alt {args.alt} m')
        self.create_timer(0.1, self.tick)

    # PX4 clock: offset measured at each 1 Hz timesync, extrapolated.
    def on_timesync(self, msg):
        local_us = self.get_clock().now().nanoseconds // 1000
        self.px4_offset_us = msg.timestamp - local_us

    def px4_now(self):
        if self.px4_offset_us is None:
            return 0
        return int(self.get_clock().now().nanoseconds // 1000
                   + self.px4_offset_us)

    def clock_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, m):
        self.armed = m.arming_state == 2
        self.offboard = m.nav_state == 14

    def on_position(self, m):
        self.pos = m

    def set_state(self, state):
        self.state = state
        self.t_state = self.clock_s()
        self.phase_pub.publish(String(data=state))
        self.get_logger().info(f'phase {state}')

    def cmd(self, c, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command, m.param1, m.param2 = c, p1, p2
        m.target_system = m.target_component = 1
        m.source_system = m.source_component = 1
        m.from_external = True
        m.timestamp = self.px4_now()
        self.cmd_pub.publish(m)

    def publish(self, n, e, vel=None, acc=None):
        """Position setpoint; vel and acc are NE feedforward or None.

        Unused fields are NaN, never zero (spec trap 1). With a
        feedforward, the vertical component is 0.0 on purpose: the
        altitude is held, so zero vertical velocity and acceleration
        are the setpoint, not an absence of one.
        """
        m = OffboardControlMode()
        m.position = True
        m.timestamp = self.px4_now()
        self.mode_pub.publish(m)
        sp = TrajectorySetpoint()
        sp.position = [n, e, -self.args.alt]
        sp.velocity = [vel[0], vel[1], 0.0] if vel is not None else NAN3
        sp.acceleration = [acc[0], acc[1], 0.0] if acc is not None else NAN3
        sp.jerk = NAN3
        sp.yaw = 0.0
        sp.yawspeed = float('nan')
        sp.timestamp = self.px4_now()
        self.sp_pub.publish(sp)

    def arrived(self, n, e):
        p = self.pos
        return (math.hypot(p.x - n, p.y - e) < ARRIVE_XY_M
                and math.hypot(p.vx, p.vy) < ARRIVE_SPEED_MS)

    def tick(self):
        self.n += 1
        elapsed = self.clock_s() - self.t_state

        if self.state == 'LANDED':
            return
        if self.state == 'LAND':
            if not self.armed:
                self.set_state('LANDED')
                self.done = True
            elif self.offboard:
                # PX4 has not taken the land command yet: keep the stream
                # alive so it does not fail over, and resend at 1 Hz.
                self.publish(*self.end)
                if self.n % 10 == 0:
                    self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            return

        if self.state == 'WAIT_POSE':
            # Stream setpoints from the start: PX4 refuses offboard unless
            # they already flow. Spec S3 rule 6: no arming before a pose.
            self.publish(*self.start)
            if (self.pos is not None and self.pos.xy_valid
                    and self.pos.z_valid and self.px4_offset_us is not None):
                self.set_state('ARMING')
            return

        if self.state == 'TRANSIT':
            pos, vel, acc = self.path.at(elapsed)
            if elapsed < self.path.duration:
                self.publish(*pos, vel, acc)
            else:
                self.publish(*pos)
                if self.arrived(*self.end):
                    self.set_state('HOVER_POST')
                elif elapsed - self.path.duration > ARRIVE_TIMEOUT_S:
                    self.get_logger().warn('arrival timeout')
                    self.set_state('HOVER_POST')
            return

        hold = self.end if self.state == 'HOVER_POST' else self.start
        self.publish(*hold)

        if self.state == 'ARMING':
            if self.armed and self.offboard:
                self.set_state('CLIMB')
            elif elapsed > 2.0 and self.n % 10 == 0:
                self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        elif self.state == 'CLIMB':
            p = self.pos
            if (abs(p.z + self.args.alt) < CLIMB_TOL_M
                    and abs(p.vz) < CLIMB_VZ_MS
                    and self.arrived(*hold)):
                self.set_state('HOVER_PRE')
        elif self.state == 'HOVER_PRE':
            if elapsed >= self.args.hold:
                self.set_state('TRANSIT')
        elif self.state == 'HOVER_POST':
            if elapsed >= self.args.post:
                self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.set_state('LAND')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--path', required=True,
                   help='closed polygon "n0,e0;n1,e1;...", NED metres')
    p.add_argument('--speed', type=float, default=5.0)
    p.add_argument('--accel', type=float, default=3.0)
    p.add_argument('--hold', type=float, default=20.0)
    p.add_argument('--post', type=float, default=5.0)
    p.add_argument('--alt', type=float, default=8.0)
    args = p.parse_args()

    rclpy.init()
    node = CalibrationFlight(args)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    # Let the LANDED phase message reach the probe before exiting.
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
