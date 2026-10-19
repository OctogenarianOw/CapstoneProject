#!/usr/bin/env python3
"""Record what the simulator offers as an energy signal, for one flight.

  rotors.csv   rotor speed commands PX4 sends to Gazebo (rad/s), stamped
               with the latest Gazebo sim clock
  battery.csv  PX4 battery_status as published over DDS
  state.csv    local position, velocity, arming and nav state

    python3 energy_probe.py OUT_DIR GZ_MOTOR_TOPIC DURATION_S
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
from px4_msgs.msg import BatteryStatus, VehicleLocalPosition, VehicleStatus

out = Path(sys.argv[1])
motor_topic = sys.argv[2]
duration = float(sys.argv[3])

lock = threading.Lock()
rotor_rows = []
sim_clock = [float('nan')]


def on_clock(msg):
    sim_clock[0] = msg.sim.sec + msg.sim.nsec * 1e-9


def on_rotors(msg):
    stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    with lock:
        rotor_rows.append([sim_clock[0], stamp, time.monotonic()]
                          + list(msg.velocity))


gz = GzNode()
if not gz.subscribe(Clock, '/clock', on_clock):
    raise SystemExit('cannot subscribe to /clock')
if not gz.subscribe(Actuators, motor_topic, on_rotors):
    raise SystemExit(f'cannot subscribe to {motor_topic}')


class Probe(Node):
    def __init__(self):
        super().__init__('energy_probe')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.battery = []
        self.state = []
        self.status = (0, 0)
        self.create_subscription(BatteryStatus, '/fmu/out/battery_status_v1',
                                 self.on_battery, qos)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_position, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4',
                                 self.on_status, qos)

    def on_battery(self, m):
        self.battery.append([m.timestamp * 1e-6, m.voltage_v, m.current_a,
                             m.remaining, m.discharged_mah])

    def on_status(self, m):
        self.status = (m.arming_state, m.nav_state)

    def on_position(self, m):
        self.state.append([m.timestamp * 1e-6, m.x, m.y, m.z, m.vx, m.vy,
                           m.vz, self.status[0], self.status[1]])


rclpy.init()
node = Probe()
end = time.monotonic() + duration
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)


def dump(name, header, rows):
    with open(out / name, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    print(f'{name}: {len(rows)} rows')


with lock:
    n = max((len(r) for r in rotor_rows), default=3) - 3
    dump('rotors.csv', ['t_sim', 't_stamp', 't_wall']
         + [f'w{i}' for i in range(n)], rotor_rows)
dump('battery.csv', ['t_px4', 'voltage_v', 'current_a', 'remaining',
                     'discharged_mah'], node.battery)
dump('state.csv', ['t_px4', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'arming',
                   'nav'], node.state)
node.destroy_node()
rclpy.shutdown()
