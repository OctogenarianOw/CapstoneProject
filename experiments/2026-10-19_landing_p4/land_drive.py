#!/usr/bin/env python3
"""
One landing as the exploration manager commands it: spec S16 (P4), S10.

Plays the manager's part at the end of a mission: take off to 8 m,
hover 2 s, fly a 15 m out-and-back return path (0, 0) -> (15, 0) ->
(0, 0) on the calibrated profile, and command LAND the moment the
executor reports arrival, as MissionFSM._commit does. Records the
setpoint, the position where PX4 disarms, the energy from the LAND
command to the disarm (the definition of E_land in energy_model), and
its duration, as one JSON line written to OUT_FILE.

Why a dedicated driver and not the calibration descents of S10: those
gave E_land = 3.20 kJ, but every autonomous mission logged on 19 Oct
spent 3.30 to 4.14 kJ from LAND to disarm. The landing C1 relies on is
this one, after a path, commanded at arrival.

    python3 land_drive.py OUT_FILE
"""

import json
import math
import sys

from px4_msgs.msg import VehicleLocalPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Header
from uav_interfaces.msg import (EnergyState, ExecutorCommand,
                                ExecutorStatus, PathCommand)

S = ExecutorStatus
ALT = 8.0
PATH = [(0.0, 0.0), (15.0, 0.0), (0.0, 0.0)]
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)


class LandDriver(Node):
    """Script of one landing, one step per executor event."""

    def __init__(self, out):
        super().__init__('landing_driver')
        self.out = out
        self.cmd_pub = self.create_publisher(ExecutorCommand,
                                             '/executor/command', 10)
        self.path_pub = self.create_publisher(PathCommand,
                                              '/executor/path', 10)
        self.create_subscription(ExecutorStatus, '/executor/status',
                                 self.on_status, 10)
        self.create_subscription(EnergyState, '/energy/state',
                                 self.on_energy, 10)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_position, PX4_QOS)
        self.status = self.energy = self.pos = None
        self.step, self.t_step = 'wait', self.now()
        self.land = None
        self.done = False
        self.create_timer(0.05, self.tick)

    def now(self):
        """Node clock in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, msg):
        """Keep the latest executor status."""
        self.status = msg

    def on_energy(self, msg):
        """Keep the latest energy state."""
        self.energy = msg

    def on_position(self, msg):
        """Keep the latest local position."""
        self.pos = msg

    def go(self, step):
        """Enter a step."""
        self.step, self.t_step = step, self.now()
        self.get_logger().info(f'step {step}')

    def tick(self):
        """Advance the script."""
        if None in (self.status, self.energy, self.pos):
            return
        dt = self.now() - self.t_step
        s = self.status
        if self.step == 'wait' and s.state == S.IDLE and dt > 3:
            self.cmd_pub.publish(ExecutorCommand(
                header=Header(), command=ExecutorCommand.TAKEOFF,
                altitude_m=ALT))
            self.go('climb')
        elif self.step == 'climb' and s.state == S.HOLDING:
            self.go('hover')
        elif self.step == 'hover' and dt > 2:
            msg = PathCommand(header=Header(), id=1, altitude_m=ALT)
            msg.north = [p[0] for p in PATH]
            msg.east = [p[1] for p in PATH]
            self.path_pub.publish(msg)
            self.go('return')
        elif (self.step == 'return' and s.state == S.HOLDING
              and s.path_id == 1 and s.arrived):
            self.cmd_pub.publish(ExecutorCommand(
                header=Header(), command=ExecutorCommand.LAND))
            self.land = (self.now(), self.energy.consumed_j, self.pos.x,
                         self.pos.y, -self.pos.z)
            self.go('land')
        elif self.step == 'land' and s.state == S.LANDED:
            t0, e0, x0, y0, z0 = self.land
            row = {
                'setpoint_ne': PATH[-1],
                'at_land_ne': (x0, y0), 'at_land_alt_m': z0,
                'touchdown_ne': (self.pos.x, self.pos.y),
                'radial_error_m': math.hypot(self.pos.x - PATH[-1][0],
                                             self.pos.y - PATH[-1][1]),
                'drift_during_landing_m': math.hypot(self.pos.x - x0,
                                                     self.pos.y - y0),
                'landing_energy_j': self.energy.consumed_j - e0,
                'landing_time_s': self.now() - t0,
            }
            with open(self.out, 'w') as f:
                f.write(json.dumps(row) + '\n')
            self.get_logger().info(f'landed: {row}')
            self.done = True


def main():
    """Run one landing to the end."""
    rclpy.init()
    node = LandDriver(sys.argv[1])
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
