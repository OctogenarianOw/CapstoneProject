#!/usr/bin/env python3
"""
Second pit calibration flight: far lines, then a hover over each plate.

As 2026-10-21_pit_calibration/pits_drive.py, on worlds/pits2.sdf
(tools/make_world_pits2.py): take off at the origin, five north-south
lines at east 59, 55, 51, 47 and 43 m past the row of plates, so the
pit column centred at east 43 (2 m pits) or 43.5 (3 m pits) is passed
at 16, 12, 8, 4 and 0 m and the one at 37 or 36.5 at 22 to 6 m. Then a
hover of 8 s at 8 m over the centre of each plate, north 36, 12, -12,
-36: the case of an aircraft stopped over a pad before landing on it,
which W3 relies on and the first calibration did not fly. Transits run
along north -64 and +64, 21 m from the plates, beyond the lidar's 18 m.

The map is copied after each line (snap_lineK.npz) and after each hover
(snap_hoverP.npz, P the plate index) once the hold exceeds the 5 s save
period of cloud_world; the position is logged at 10 Hz (track.csv) and
the phases with their times (phases.csv). Writes OUT/stop at the end.

    python3 pits2_drive.py OUT_DIR MAP_PATH
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
HOVER_S = 8.0
N_END = 64.0
LINES_E = [59.0, 55.0, 51.0, 47.0, 43.0]
PLATE_E = 40.0
HOVERS = [(3, 36.0), (2, 12.0), (1, -12.0), (0, -36.0)]


def plan():
    """Legs as (kind, label, path NED), each path from the previous end."""
    legs = [('line', 'line1', [(0.0, 0.0), (-N_END, 0.0),
                               (-N_END, LINES_E[0]), (N_END, LINES_E[0])])]
    n = N_END
    for k, (e_prev, e) in enumerate(zip(LINES_E[:-1], LINES_E[1:])):
        legs.append(('line', f'line{k + 2}', [(n, e_prev), (n, e), (-n, e)]))
        n = -n
    here = (n, LINES_E[-1])
    for p, cn in HOVERS:
        legs.append(('hover', f'hover{p}', [here, (cn, PLATE_E)]))
        here = (cn, PLATE_E)
    return legs, here


class Driver(Node):
    """Script of the flight, one step per executor event."""

    def __init__(self, out, map_path):
        """Open the logs and the topics."""
        super().__init__('pits2_driver')
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
        self.legs, self.last_point = plan()
        self.leg = -1
        self.status = None
        self.position = None
        self.step = 'wait'
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

    def log(self, phase):
        """Write a phase with the current time."""
        self.phases.writerow([f'{self.now():.3f}', phase])
        self.phases_file.flush()
        self.track_file.flush()

    def go(self, step):
        """Enter a step and log it as a phase."""
        self.step, self.t_step = step, self.now()
        self.log(step)
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

    def next_leg(self):
        """Send the next leg, or the way home when none is left."""
        self.leg += 1
        if self.leg < len(self.legs):
            _, label, path = self.legs[self.leg]
            self.send_path(self.leg + 1, path)
            self.go(label)
        else:
            n, e = self.last_point
            self.send_path(99, [(n, e), (n, 0.0), (0.0, 0.0)])
            self.go('home')

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
            self.next_leg()
        elif self.step.startswith(('line', 'hover')) \
                and self.arrived(self.leg + 1):
            self.go('hold_' + self.step)
        elif self.step.startswith('hold_'):
            label = self.step[5:]
            if dt > (HOVER_S if label.startswith('hover') else HOLD_S):
                shutil.copy(self.map_path, self.out / f'snap_{label}.npz')
                self.log(f'snap_{label}')
                self.next_leg()
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
