#!/usr/bin/env python3
"""
Where does a decision cycle of the manager spend its time? Offline.

Feeds map snapshots of a mission, in order, to the real
ExplorationManager node (no simulator, no RViz), forces a selection on
each, and times the four parts of on_map: world model, state machine
(registry, invariant, C1 and C2), display messages, log. Also profiles
the slowest part.

    python3 profile_cycle.py MISSION_DIR T1 [T2 ...]
"""

import cProfile
import glob
import io
import json
import pstats
import sys
import tempfile
import time

import numpy as np
import rclpy
from std_msgs.msg import Header
from uav_core import exploration_manager as em
from uav_core.map_msg import grid_to_msg
from uav_interfaces.msg import EnergyState

run = sys.argv[1]
times = [float(x) for x in sys.argv[2:]]
cycles = [json.loads(line) for line in open(f'{run}/cycles.jsonl')]
snaps = sorted(glob.glob(f'{run}/maps/map_*.npz'))


def nearest(items, key, t):
    return min(items, key=lambda x: abs(key(x) - t))


rclpy.init(args=['--ros-args', '-p', f'log_root:={tempfile.mkdtemp()}',
                 '-p', 'mode:=explore', '-p', 'area_half_m:=95.0'])
node = em.ExplorationManager()
node.pose_valid = True
node.fsm.registry.add_provisional((0.0, 0.0), 0.0)
parts = {}
for name in ('build_world', 'display', 'record'):
    original = getattr(em, name, None) or getattr(node, name)


def timed(fn, key):
    def wrapper(*a, **k):
        t0 = time.perf_counter()
        out = fn(*a, **k)
        parts[key] = parts.get(key, 0.0) + 1e3 * (time.perf_counter() - t0)
        return out
    return wrapper


em.build_world = timed(em.build_world, 'world model')
node.fsm.step = timed(node.fsm.step, 'state machine, C1, C2')
node.display = timed(node.display, 'display messages')
node.record = timed(node.record, 'log')

for t in times:
    snap = nearest(snaps, lambda p: float(p.split('_')[-1][:-4]), t)
    c = nearest(cycles, lambda c: c['t'], t)
    m = np.load(snap)
    msg = grid_to_msg(m['elevation'], m['variance'], m['count'],
                      float(m['resolution']),
                      (float(m['origin_north']), float(m['origin_east'])),
                      Header())
    node.vehicle = tuple(c['vehicle_ned'])
    node.energy = EnergyState(capacity_j=200000.0,
                              remaining_j=float(c['remaining_j']))
    node.energy_rx = node.now_s()
    node.fsm.state = 'SELECTING'
    parts.clear()
    prof = cProfile.Profile()
    t0 = time.perf_counter()
    prof.enable()
    node.on_map(msg)
    prof.disable()
    total = 1e3 * (time.perf_counter() - t0)
    d = node.fsm.last_decision
    print(f'== snapshot {snap.split("/")[-1]}: {total:.0f} ms, '
          f'{len(d.candidates) if d else 0} candidates')
    for k, v in parts.items():
        print(f'   {k:<24} {v:7.0f} ms')
    s = io.StringIO()
    pstats.Stats(prof, stream=s).sort_stats('cumulative').print_stats(14)
    lines = [ln for ln in s.getvalue().splitlines() if 'uav_core' in ln]
    print('\n'.join('   ' + ln.strip()[:150] for ln in lines[:10]))
node.destroy_node()
rclpy.shutdown()
