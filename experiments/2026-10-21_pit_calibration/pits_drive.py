#!/usr/bin/env python3
"""
Scripted flight of the pit calibration: far lines first, then closer.

Plays the manager's part, as 2026-10-19_executor/drive.py does: take off
to 8 m at the origin, fly five north-south lines past the row of plates
of worlds/pits.sdf (tools/make_world_pits.py), then come home and land.
The lines run at east 58.5, 54.5, 50.5, 46.5 and 42.5 m, so the nearer
column of pits (east 42.5) is passed at 16, 12, 8, 4 and 0 m and the
farther one (east 38.5) at 20, 16, 12, 8 and 4 m. Far to near is the
order of a mission that confirms a pad from a distance and then
approaches it. Transits run along north -60 and +60, 22.5 m from the
nearest pit, beyond the 18 m the lidar reaches at 8 m.

After each line the aircraft holds 7 s, longer than the 5 s save period
of cloud_world, and the map is copied to snap_lineK.npz. The vehicle
position is written to track.csv at 10 Hz and each phase to phases.csv,
so the distance at which each pit was passed is measured from the
flight, not taken from the plan. Writes OUT/stop at the end, which ends
the session of sim_session.sh.

    python3 pits_drive.py OUT_DIR MAP_PATH
"""

import csv
from pathlib import Path
import shutil
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Header
from px4_msgs.msg import VehicleLocalPosition
from uav_interfaces.msg import ExecutorCommand, ExecutorStatus, PathCommand

S = ExecutorStatus
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
ALT = 8.0
HOLD_S = 7.0
N_END = 60.0
LINES_E = [58.5, 54.5, 50.5, 46.5, 42.5]


def line_paths():
    """One path per line, from the end of the previous one (NED n, e)."""
    paths = []
    paths.append([(0.0, 0.0), (-N_END, 0.0), (-N_END, LINES_E[0]),
                  (N_END, LINES_E[0])])
    n = N_END
    for e_prev, e in zip(LINES_E[:-1], LINES_E[1:]):
        paths.append([(n, e_prev), (n, e), (-n, e)])
        n = -n
    return paths, (n, LINES_E[-1])


class Driver(Node):
    """Script of the calibration flight, one step per executor event."""

    def __init__(self, out, map_path):
        super().__init__('pits_driver')
        self.out, self.map_path = out, Path(map_path)
        self.cmd_pub = self.create_publisher(ExecutorCommand,
                                             '/executor/command', 10)
        self.path_pub = self.create_publisher(PathCommand,
                                              '/executor/path', 10)
        self.create_subscription(ExecutorStatus, '/executor/status',
                                 self.on_status, 10)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_position, PX4_QOS)
        self.track_file = (out / 'track.csv').open('w', newline='')
        self.track = csv.writer(self.track_file)
        self.track.writerow(['t', 'north', 'east', 'down'])
        self.phases_file = (out / 'phases.csv').open('w', newline='')
        self.phases = csv.writer(self.phases_file)
        self.phases.writerow(['t', 'phase'])
        self.paths, self.last_point = line_paths()
        self.status = None
        self.position = None
        self.step, self.line = 'wait', 0
        self.t_step = self.now()
        self.done = False
        self.create_timer(0.1, self.tick)

    def now(self):
        """Node clock in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, msg):
        """Keep the latest executor status."""
        self.status = msg

    def on_position(self, msg):
        """Keep the latest NED position."""
        self.position = (msg.x, msg.y, msg.z)

    def go(self, step):
        """Enter a step and log it as a phase."""
        self.step, self.t_step = step, self.now()
        self.phases.writerow([f'{self.t_step:.3f}', step])
        self.phases_file.flush()
        self.track_file.flush()
        self.get_logger().info(f'step {step}')

    def send_path(self, pid, pts):
        """Publish a path; the executor replaces its first point."""
        msg = PathCommand(header=Header(), id=pid, altitude_m=ALT)
        msg.north = [float(p[0]) for p in pts]
        msg.east = [float(p[1]) for p in pts]
        self.path_pub.publish(msg)

    def arrived(self, pid):
        """Tell whether the executor reports arrival at path pid."""
        s = self.status
        return s.state == S.HOLDING and s.path_id == pid and s.arrived

    def tick(self):
        """Log the position and advance the script."""
        if self.position is not None:
            self.track.writerow([f'{self.now():.3f}']
                                + [f'{v:.3f}' for v in self.position])
        if self.status is None:
            return
        dt = self.now() - self.t_step
        if self.step == 'wait' and self.status.state == S.IDLE and dt > 3:
            self.cmd_pub.publish(ExecutorCommand(
                header=Header(), command=ExecutorCommand.TAKEOFF,
                altitude_m=ALT))
            self.go('climb')
        elif self.step == 'climb' and self.status.state == S.HOLDING:
            self.line = 1
            self.send_path(1, self.paths[0])
            self.go('line1')
        elif self.step.startswith('line') and self.arrived(self.line):
            self.go(f'hold{self.line}')
        elif self.step.startswith('hold') and dt > HOLD_S:
            shutil.copy(self.map_path, self.out / f'snap_line{self.line}.npz')
            self.phases.writerow([f'{self.now():.3f}', f'snap{self.line}'])
            if self.line < len(self.paths):
                self.line += 1
                self.send_path(self.line, self.paths[self.line - 1])
                self.go(f'line{self.line}')
            else:
                n, e = self.last_point
                self.send_path(99, [(n, e), (n, 0.0), (0.0, 0.0)])
                self.go('home')
        elif self.step == 'home' and self.arrived(99):
            self.cmd_pub.publish(ExecutorCommand(
                header=Header(), command=ExecutorCommand.LAND))
            self.go('land')
        elif self.step == 'land' and self.status.state == S.LANDED:
            self.go('landed')
            (self.out / 'stop').touch()
            self.done = True


def main():
    """Run the script to the end."""
    out = Path(sys.argv[1]).resolve()
    rclpy.init()
    node = Driver(out, sys.argv[2])
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.phases_file.close()
    node.track_file.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
