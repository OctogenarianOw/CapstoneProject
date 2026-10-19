#!/usr/bin/env python3
"""
Scripted driver for a live test of the trajectory executor.

Plays the manager's part: take off to 8 m, hover 5 s, fly a 20 m square,
hover 5 s, start a 40 m line east and preempt it after 4 s with a path
to (20, 20) then home, hover 5 s, land. Publishes its phases on
/calibration/phase for energy_probe_v2, and prints, per path, the energy
the model predicted from the plan against the energy the monitor
measured from the path command to arrival.

    python3 drive.py
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String
from uav_core.energy_model import CALIBRATION_2026_10_19 as MODEL
from uav_interfaces.msg import (EnergyState, ExecutorCommand,
                                ExecutorStatus, PathCommand)

S = ExecutorStatus
ALT = 8.0
SQUARE = [(0, 0), (20, 0), (20, 20), (0, 20), (0, 0)]
LINE = [(0, 0), (0, 40)]
DETOUR = [(20, 20), (0, 0)]


class Driver(Node):
    """Script of the test, one step per executor event."""

    def __init__(self):
        super().__init__('executor_driver')
        self.cmd_pub = self.create_publisher(ExecutorCommand,
                                             '/executor/command', 10)
        self.path_pub = self.create_publisher(PathCommand,
                                              '/executor/path', 10)
        self.phase_pub = self.create_publisher(String, '/calibration/phase',
                                               10)
        self.create_subscription(ExecutorStatus, '/executor/status',
                                 self.on_status, 10)
        self.create_subscription(EnergyState, '/energy/state',
                                 self.on_energy, 10)
        self.status = None
        self.energy = None
        self.step = 'wait'
        self.t_step = self.now()
        self.e_start = None
        self.predicted = None
        self.done = False
        self.create_timer(0.1, self.tick)

    def now(self):
        """Node clock in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, msg):
        """Keep the latest executor status."""
        self.status = msg

    def on_energy(self, msg):
        """Keep the latest energy state."""
        self.energy = msg

    def go(self, step):
        """Enter a step and publish it as a phase."""
        self.step, self.t_step = step, self.now()
        self.phase_pub.publish(String(data=step.upper()))
        self.get_logger().info(f'step {step}')

    def send_path(self, pid, pts):
        """Publish a path and remember its predicted energy."""
        msg = PathCommand(header=Header(), id=pid, altitude_m=ALT)
        msg.north = [float(p[0]) for p in pts]
        msg.east = [float(p[1]) for p in pts]
        self.path_pub.publish(msg)
        self.e_start = self.energy.consumed_j
        self.predicted = MODEL.path_energy(pts)

    def report(self, label, pts):
        """Print predicted against measured energy for one path."""
        measured = self.energy.consumed_j - self.e_start
        err = 100.0 * (self.predicted - measured) / measured
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                     for a, b in zip(pts[:-1], pts[1:]))
        self.get_logger().info(
            f'{label}: {length:.1f} m planned, predicted {self.predicted:.0f}'
            f' J, measured {measured:.0f} J, error {err:+.2f} %')

    def arrived(self, pid):
        """Tell whether the executor reports arrival at path pid."""
        s = self.status
        return (s.state == S.HOLDING and s.path_id == pid and s.arrived)

    def tick(self):
        """Advance the script."""
        if self.status is None or self.energy is None:
            return
        dt = self.now() - self.t_step
        if self.step == 'wait' and self.status.state == S.IDLE and dt > 3:
            self.cmd_pub.publish(ExecutorCommand(
                header=Header(), command=ExecutorCommand.TAKEOFF,
                altitude_m=ALT))
            self.go('climb')
        elif self.step == 'climb' and self.status.state == S.HOLDING:
            self.go('hover_1')
        elif self.step == 'hover_1' and dt > 5:
            self.send_path(1, SQUARE)
            self.go('square')
        elif self.step == 'square' and self.arrived(1):
            self.report('square 20 m', SQUARE)
            self.go('hover_2')
        elif self.step == 'hover_2' and dt > 5:
            self.send_path(2, LINE)
            self.go('line')
        elif self.step == 'line' and dt > 4:
            # The executor replaces the first point by its own setpoint,
            # mid-line here, so the start given is a placeholder.
            self.send_path(3, [(0.0, 0.0)] + DETOUR)
            self.go('preempted')
        elif self.step == 'preempted' and self.arrived(3):
            self.get_logger().info('arrived after preemption')
            self.go('hover_3')
        elif self.step == 'hover_3' and dt > 5:
            self.cmd_pub.publish(ExecutorCommand(
                header=Header(), command=ExecutorCommand.LAND))
            self.go('land')
        elif self.step == 'land' and self.status.state == S.LANDED:
            self.phase_pub.publish(String(data='LANDED'))
            self.get_logger().info('landed')
            self.done = True


def main():
    """Run the script to the end."""
    rclpy.init()
    node = Driver()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
