#!/usr/bin/env python3
"""
Scripted invalidation of the zone a return flies to, on W3 (spec S9.5).

Role: follow a running mission through its own log and, when the
aircraft starts its return to a confirmed zone, place a field of crates
over that zone in the Gazebo world, so that the zone the return relies
on is invalidated by the aircraft's own observation during the
transit. Like ../w3_depot.py it stays outside the system under test:
it reads the mission log, acts on the Gazebo world only, and sends
nothing to the aircraft or the manager.

Why the return, measured (2026-10-23_s95_depot, README.md): the design
of 22 Oct placed a depot on pad Z at a frontier transit relying on Z
with Z in view. Flown, 0 placements in 26 missions; on the logs of the
50 C1 missions of the final W3 campaign, 4 to 5 missions would have
fired. A frontier transit rarely relies on Z, and when it does it flies
away from Z. A return always relies on its target and flies to it:
every C1 mission has one, the target is never the take-off point on W3
(0 of 50), and it is the transit where the energy is lowest, the
hardest case for the guarantee.

Conditions (v2, 24 Oct 10:55), checked every cycle of a return, on its
current target (the last 'return' event, so a switch to a cheaper zone
moves the target):
  zone    the target is a confirmed, non-provisional zone of at most
          MAX_AREA_M2, so that the field covers it whole, and the field
          would not cover the take-off point (TAKEOFF_CLEAR_M)
  arm     the aircraft is within ARM_M of the target's touchdown point
          and at least MIN_LENGTH_M from it, so that the crates are
          placed on the zone it is about to reach and seen before the
          arrival: once the landing is committed the machine
          re-evaluates nothing (S15.2), and a field placed under a
          committed landing would test that rule, not S9.5. Assumed
          detection delay 2.5 s (22 Oct design); 20 m leave about 4 s
Why v2 (runs_v1/, README.md): v1 fired once, at the start of the
return, on the take-off yard; 3 s later the return switched to a
cheaper zone and never saw the crates. And a field over the take-off
point covers the provisional zone, which the system assumes free for
the whole mission (S15.3) and no observation can retire: that is an
assumption to declare, not what S9.5 tests.
The field: crates of 2 x 2 x 2 m on a 3.5 m pitch over a 20 m square
centred on the zone's touchdown point, at the height of the truth
surface there (a zone may be a roof). The 1.5 m gaps hold no 1.5 m
footprint of 3 x 3 cells, so no landing cell survives under it.

    python3 return_depot.py LOG_ROOT --out OUT_JSONL
"""

import argparse
import glob
import json
import math
import os
from pathlib import Path
import subprocess
import time

import numpy as np

TRUTH = Path.home() / 'ros2_ws/droneAfter/worlds/w3_truth.npz'
WORLD = 'w3'
MAX_AREA_M2 = 250.0
MIN_LENGTH_M = 20.0
ARM_M = 35.0
SIDE_M = 20.0
CRATE_M = 2.0
PITCH_M = 3.5


def read_lines(path, offset):
    """New complete JSON lines of a growing file from a byte offset."""
    try:
        with open(path) as fh:
            fh.seek(offset)
            data = fh.read()
    except FileNotFoundError:
        return [], offset
    end = data.rfind('\n') + 1
    lines = [json.loads(x) for x in data[:end].splitlines() if x.strip()]
    return lines, offset + len(data[:end].encode())


def surface_height(north, east):
    """Height of the W3 truth surface under a NED point."""
    t = np.load(TRUTH)
    c = t['centres']
    i = int(np.argmin(np.abs(c - north)))
    j = int(np.argmin(np.abs(c - east)))
    return float(t['truth'].T[i, j])


def crates(north, east, height):
    """World poses (x east, y north, z centre) of the field's crates."""
    half = SIDE_M / 2.0 - CRATE_M / 2.0
    offsets = np.arange(-half, half + 1e-6, PITCH_M)
    return [(east + dx, north + dy, height + CRATE_M / 2.0)
            for dx in offsets for dy in offsets]


def field_sdf(poses):
    """The field as one static model, box poses in world coordinates."""
    boxes = []
    for k, (x, y, z) in enumerate(poses):
        pose = f'{x:.3f} {y:.3f} {z:.3f} 0 0 0'
        size = f'{CRATE_M} {CRATE_M} {CRATE_M}'
        boxes.append(
            f'      <collision name="crate_{k}_collision"><pose>{pose}</pose>'
            f'<geometry><box><size>{size}</size></box></geometry></collision>\n'
            f'      <visual name="crate_{k}_visual"><pose>{pose}</pose>'
            f'<geometry><box><size>{size}</size></box></geometry>'
            f'<material><diffuse>0.85 0.6 0.3 1</diffuse></material></visual>\n')
    return ('<?xml version="1.0" ?>\n<sdf version="1.9">\n'
            '  <model name="return_depot">\n    <static>true</static>\n'
            '    <link name="crates">\n' + ''.join(boxes)
            + '    </link>\n  </model>\n</sdf>\n')


def spawn(sdf_path):
    """Place the field in the running world; True if Gazebo accepted."""
    req = f'sdf_filename: "{sdf_path}" name: "return_depot"'
    out = subprocess.run(['gz', 'service', '-s', f'/world/{WORLD}/create',
                          '--reqtype', 'gz.msgs.EntityFactory',
                          '--reptype', 'gz.msgs.Boolean',
                          '--timeout', '3000', '--req', req],
                         capture_output=True, text=True)
    return 'data: true' in out.stdout


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('log_root')
    ap.add_argument('--out', default='return_depot.jsonl')
    args = ap.parse_args()
    log = open(args.out, 'a')

    def write(rec):
        log.write(json.dumps(rec) + '\n')
        log.flush()

    write({'wall': time.time(), 'kind': 'start', 'version': 2,
           'max_area_m2': MAX_AREA_M2, 'min_length_m': MIN_LENGTH_M,
           'arm_m': ARM_M, 'side_m': SIDE_M})
    mdir = None
    while mdir is None:
        found = sorted(glob.glob(os.path.join(args.log_root, '*',
                                              'config.json')))
        if found:
            mdir = os.path.dirname(found[-1])
        else:
            time.sleep(0.5)
    cycles, oc, oe = [], 0, 0
    target = None          # the last 'return' event: the current target
    fired = False
    skipped = set()
    while True:
        new_c, oc = read_lines(os.path.join(mdir, 'cycles.jsonl'), oc)
        new_e, oe = read_lines(os.path.join(mdir, 'events.jsonl'), oe)
        cycles += new_c
        for e in new_e:
            if e.get('kind') == 'state' and e.get('new') == 'LANDED':
                write({'wall': time.time(), 'kind': 'end', 'fired': fired})
                return
            if e.get('kind') == 'return':
                target = e
                write({'wall': time.time(), 't': e['t'], 'kind': 'target',
                       'zone_id': e['zone_id'], 'reason': e['reason'],
                       'length_m': e['length_m']})
            if e.get('kind') == 'state' and e.get('new') in (
                    'COMMIT_LANDING', 'EMERGENCY'):
                target = None      # nothing is re-evaluated after it
        if fired or target is None or not new_c:
            time.sleep(0.1)
            continue
        c = cycles[-1]
        zone = next((z for z in c['zones']
                     if z['id'] == target['zone_id']), None)
        if c['state'] != 'RETURNING' or zone is None:
            time.sleep(0.1)
            continue
        n, east = zone['touchdown_ne']
        d = math.dist(c['vehicle_ned'][:2], (n, east))
        half = SIDE_M / 2.0 + 1.0
        covers_takeoff = abs(n) <= half and abs(east) <= half
        eligible = (not zone['provisional'] and zone['state'] == 'CONFIRMED'
                    and zone['area_m2'] <= MAX_AREA_M2 and not covers_takeoff)
        if not eligible:
            if target['zone_id'] not in skipped:
                skipped.add(target['zone_id'])
                write({'wall': time.time(), 't': c['t'], 'kind': 'skip',
                       'zone_id': target['zone_id'],
                       'area_m2': zone['area_m2'],
                       'provisional': zone['provisional'],
                       'covers_takeoff': covers_takeoff})
            time.sleep(0.1)
            continue
        if not MIN_LENGTH_M <= d <= ARM_M:
            time.sleep(0.1)
            continue
        h = surface_height(n, east)
        poses = crates(n, east, h)
        path = os.path.join(os.path.dirname(os.path.abspath(args.out)),
                            'return_depot.sdf')
        with open(path, 'w') as fh:
            fh.write(field_sdf(poses))
        fired = True
        write({'wall': time.time(), 't': c['t'], 'kind': 'fire',
               'zone_id': target['zone_id'], 'reason': target['reason'],
               'length_m': d, 'touchdown_ne': [n, east],
               'area_m2': zone['area_m2'], 'landing_m2': zone['landing_m2'],
               'height_m': h, 'crates': len(poses), 'side_m': SIDE_M,
               'spawned': spawn(path), 'spawn_wall': time.time()})

if __name__ == '__main__':
    main()
