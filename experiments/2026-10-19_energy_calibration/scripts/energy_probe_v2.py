#!/usr/bin/env python3
"""Record rotor power inputs, position and flight phases on one clock.

Derived from 2026-10-17_energy_signal/scripts/energy_probe.py, the
instrument validated on hover (805.5 rad/s measured against 806.0
predicted). Three changes, all about time bases:

  - every row carries t_wall, time.monotonic() at reception, shared by
    the Gazebo thread and the ROS thread of this one process. The 17 Oct
    probe stamped state with the PX4 clock only, and analyse_energy.py
    had to align it on the rotor spin-up, an alignment whose error was
    never measured; cutting segments at phase changes needs better,
  - the phases published by calibration_flight.py are recorded,
  - recording stops 2 s after the LANDED phase, DURATION_S is a cap.

CSV files are written with LF line ends.

  rotors.csv   t_sim, t_wall, w0..w3, rotor speed commands (rad/s)
  state.csv    t_wall, t_px4, position, velocity, arming, nav state
  phases.csv   t_wall, phase

    python3 energy_probe_v2.py OUT_DIR GZ_MOTOR_TOPIC DURATION_S
"""

import csv
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from gz.transport13 import Node as GzNode
from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.clock_pb2 import Clock
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

out = Path(sys.argv[1])
motor_topic = sys.argv[2]
duration = float(sys.argv[3])

lock = threading.Lock()
rotor_rows = []
sim_clock = [float('nan')]


def on_clock(msg):
    sim_clock[0] = msg.sim.sec + msg.sim.nsec * 1e-9


def on_rotors(msg):
    t_wall = time.monotonic()
    with lock:
        rotor_rows.append([sim_clock[0], t_wall] + list(msg.velocity))


gz = GzNode()
if not gz.subscribe(Clock, '/clock', on_clock):
    raise SystemExit('cannot subscribe to /clock')
if not gz.subscribe(Actuators, motor_topic, on_rotors):
    raise SystemExit(f'cannot subscribe to {motor_topic}')


class Probe(Node):
    def __init__(self):
        super().__init__('energy_probe_v2')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.state = []
        self.phases = []
        self.status = (0, 0)
        self.landed_at = None
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_position, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4',
                                 self.on_status, qos)
        self.create_subscription(String, '/calibration/phase',
                                 self.on_phase, 10)

    def on_status(self, m):
        self.status = (m.arming_state, m.nav_state)

    def on_position(self, m):
        self.state.append([time.monotonic(), m.timestamp * 1e-6, m.x, m.y,
                           m.z, m.vx, m.vy, m.vz, self.status[0],
                           self.status[1]])

    def on_phase(self, m):
        t = time.monotonic()
        self.phases.append([t, m.data])
        if m.data == 'LANDED':
            self.landed_at = t


rclpy.init()
node = Probe()
end = time.monotonic() + duration
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
    if node.landed_at is not None and time.monotonic() > node.landed_at + 2:
        break


def dump(name, header, rows):
    with open(out / name, 'w', newline='') as f:
        w = csv.writer(f, lineterminator='\n')
        w.writerow(header)
        w.writerows(rows)
    print(f'{name}: {len(rows)} rows')


with lock:
    n = max((len(r) for r in rotor_rows), default=2) - 2
    dump('rotors.csv', ['t_sim', 't_wall'] + [f'w{i}' for i in range(n)],
         rotor_rows)
dump('state.csv', ['t_wall', 't_px4', 'x', 'y', 'z', 'vx', 'vy', 'vz',
                   'arming', 'nav'], node.state)
dump('phases.csv', ['t_wall', 'phase'], node.phases)
print('landed' if node.landed_at is not None else 'stopped on duration cap')
node.destroy_node()
rclpy.shutdown()
