#!/usr/bin/env python3
"""
Scripted invalidation of a confirmed zone on W3 (spec S9.5, gate B).

Role: follow a running mission through its own log and, when the
critical case of S9.5 holds, place the depot of crates on pad Z
(tools/make_world_w3.py), so that the zone the aircraft's guarantee
rests on is invalidated by an observation during a transit. It stays
outside the system under test: it reads the mission log, acts only on
the Gazebo world, and sends nothing to the aircraft or the manager.

Why a trigger that waits, measured first (2026-10-22_w3_design): on W2
the case arose by itself in 1 of 50 C1 missions. With Z gone, a target
f decided at p needs E_go + E(f -> T) + E_land <= B - R to stay
admissible through the fallback T, while the aircraft can still reach
T from p if E(p -> T) + E_land <= B - R. By the triangle inequality the
band between the two is at most 2 E(p -> f) wide: a few kJ, about ten
seconds of flight. Outside it, losing Z either changes nothing or
leaves no zone reachable, whatever the system does.

Conditions, on each new frontier decision (first cycle of a new chosen
target, read from cycles.jsonl and the path from events.jsonl):
  relies  the return zone of the target is Z (logged zone_id)
  rests   without Z the target is not admissible:
          E_go + E(f -> fallback) + overhead + E_land > B - R + margin.
          Constant along the transit to first order (B and E(p -> f)
          fall together), so taken at the decision
  view    all of Z will be in view: the aircraft at the decision, or
          the first point of its planned path, within r_view of the
          centre of Z; a part left unseen stays flat in the map and
          keeps the zone confirmed
  reach   the fallback stays reachable where the registry will react:
          the observation point moved on along the path by an assumed
          detection delay at cruise speed, energy flown deducted,
          E(here -> fallback) + overhead + E_land <= B - R - margin
Energies with the manager's planning model, in straight legs (the
planner smooths its paths into straight legs on a field without
obstacles), to the nearest landable cell of the fallback pad. The
detection delay is assumed (2.5 s) until measured; every quantity is
logged for every decision, so a flight measures how often each holds.

Modes: critical (relies, rests, view, reach), first (relies, view,
reach: the first transit on Z, whatever the energy), dry (log only).

    python3 w3_depot.py live LOG_ROOT [--mode critical] [--world w3]
    python3 w3_depot.py replay MISSION_DIR [--pads w2]
"""

import argparse
import glob
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]
                       / 'src' / 'uav_core'))
from uav_core.energy_model import EnergyModel  # noqa: E402

DEPOT_SDF = Path(__file__).resolve().parents[1] / 'worlds' / 'w3_depot.sdf'
# Pads in NED: centre (north, east) and half extent of the landable
# cell centres (north, east); from the truths of make_world_w2/_w3.
PADS = {
    # Only the yard and Z: in the mixed W3 the other zones (road,
    # parking lot, roof, clearing, meadow) are fallbacks too, and are
    # not described here yet, so the trigger under-counts them
    'w3': {'home': ((0.0, 0.0), (6.5, 6.5)),
           'pad_z': ((-28.0, 28.0), (4.5, 2.5))},
    'w2': {'home': ((0.0, 0.0), (5.5, 6.5)),
           'pad_ne': ((40.0, 48.0), (5.5, 6.5)),
           'pad_sw': ((-48.0, -40.0), (5.5, 6.5))},
}
TARGET = {'w3': ('pad_z',), 'w2': ('pad_ne', 'pad_sw')}
R_VIEW_M = {'pad_z': 9.0, 'pad_ne': 5.0, 'pad_sw': 5.0}
DETECTION_S = 2.5          # assumed, to be measured in flight
MARGIN_J = 200.0           # straight legs against planned paths


def pad_of(ne, pads):
    """Name of the pad whose landable core holds a point, or None."""
    for name, (c, h) in pads.items():
        if abs(ne[0] - c[0]) <= h[0] + 1.0 and abs(ne[1] - c[1]) <= h[1] + 1.0:
            return name
    return None


def nearest(p, pad):
    """Nearest landable cell centre of a pad to p."""
    (c, h) = pad
    return (float(np.clip(p[0], c[0] - h[0], c[0] + h[0])),
            float(np.clip(p[1], c[1] - h[1], c[1] + h[1])))


def along(path, s):
    """Point at arc length s along a polyline, and the polyline to it."""
    out = [tuple(path[0])]
    for a, b in zip(path[:-1], path[1:]):
        seg = math.dist(a, b)
        if s <= seg and seg > 0:
            t = s / seg
            p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            return p, out + [p]
        s -= seg
        out.append(tuple(b))
    return tuple(path[-1]), out


def first_within(path, centre, r):
    """Arc length of the first point of a polyline within r of centre."""
    s0 = 0.0
    for a, b in zip(path[:-1], path[1:]):
        a, b, c = np.asarray(a), np.asarray(b), np.asarray(centre)
        d, seg = b - a, float(np.linalg.norm(b - a))
        if seg > 0:
            # Solve |a + t d - c| = r for the smallest t in [0, 1].
            f = a - c
            qa, qb, qc = d @ d, 2 * f @ d, f @ f - r * r
            disc = qb * qb - 4 * qa * qc
            if qc <= 0:
                return s0
            if disc >= 0:
                t = (-qb - math.sqrt(disc)) / (2 * qa)
                if 0.0 <= t <= 1.0:
                    return s0 + t * seg
        s0 += seg
    return None


def evaluate(cycle, chosen, path, config, pads, targets):
    """All conditions of one decision, as a JSON-ready dict."""
    model = EnergyModel(**config['energy_model'])
    zones = {z['id']: z for z in cycle['zones']}
    z = zones.get(chosen['zone_id'])
    out = {'t': cycle['t'], 'zone_id': chosen['zone_id'],
           'viewpoint_ne': chosen['viewpoint_ne']}
    pad = None if z is None or z['provisional'] else pad_of(
        z['touchdown_ne'], pads)
    out['pad'] = pad
    out['relies'] = pad in targets
    if not out['relies']:
        return out
    # What the return field would seed without Z: the landable cells of
    # every other confirmed zone, the take-off point alone for the
    # provisional zone (it has no cells).
    fallbacks = []
    for o in cycle['zones']:
        if o['state'] != 'CONFIRMED' or o['id'] == z['id']:
            continue
        name = None if o['provisional'] else pad_of(o['touchdown_ne'], pads)
        if o['provisional']:
            fallbacks.append(('take-off', (o['touchdown_ne'], (0.0, 0.0))))
        elif name is not None and name != pad:
            fallbacks.append((name, pads[name]))
    b = cycle['remaining_j']
    capacity = b + cycle['consumed_j']
    reserve = config['reserve_fraction'] * capacity
    hover = model.hover_energy(cycle['overhead_s'])
    p = tuple(cycle['vehicle_ned'][:2])
    f = tuple(chosen['viewpoint_ne'])

    def to_fallback(q):
        if not fallbacks:
            return math.inf
        return min(model.path_energy([q, nearest(q, box)])
                   for _, box in fallbacks) + hover + model.e_land_j

    need_alt = chosen['e_go'] + to_fallback(f)
    out.update(b_j=b, reserve_j=reserve, e_total_j=chosen['e_total'],
               need_without_z_j=need_alt,
               fallbacks=sorted({n for n, _ in fallbacks}))
    out['rests'] = need_alt > b - reserve + MARGIN_J
    centre = pads[pad][0]
    path = [p] + [tuple(w) for w in path] if path else [p, f]
    s_obs = first_within(path, centre, R_VIEW_M[pad])
    out['d_start_m'] = math.dist(p, centre)
    out['view'] = s_obs is not None
    if not out['view']:
        return out
    s_react = s_obs + model.cruise_speed_ms * DETECTION_S
    q, flown = along(path, s_react)
    b_react = b - model.path_energy(flown)
    out.update(s_obs_m=s_obs, react_ne=q, b_react_j=b_react,
               i1_without_z_j=b_react - reserve - to_fallback(q))
    out['reach'] = out['i1_without_z_j'] >= MARGIN_J
    return out


def fires(r, mode):
    """Whether a decision record meets the conditions of a mode."""
    if mode == 'dry' or not r.get('relies') or not r.get('view'):
        return False
    if not r.get('reach'):
        return False
    return mode == 'first' or r.get('rests', False)


def decisions(cycles, events):
    """First cycle of each new chosen target, with its planned path."""
    paths = [e for e in events if e.get('kind') == 'action'
             and e.get('action') == 'path']
    last = None
    for cy in cycles:
        ch = [c for c in cy.get('candidates', []) if c.get('chosen')]
        if not ch:
            last = None
            continue
        key = (tuple(ch[0]['viewpoint_ne']), ch[0]['e_go'])
        if key == last:
            continue
        last = key
        after = [e for e in paths if e['t'] >= cy['t']]
        yield cy, ch[0], (after[0]['waypoints'] if after else None)


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


def spawn(world):
    """Place the depot in the running world; True if Gazebo accepted."""
    req = f'sdf_filename: "{DEPOT_SDF}" name: "w3_depot"'
    out = subprocess.run(['gz', 'service', '-s', f'/world/{world}/create',
                          '--reqtype', 'gz.msgs.EntityFactory',
                          '--reptype', 'gz.msgs.Boolean',
                          '--timeout', '3000', '--req', req],
                         capture_output=True, text=True)
    return 'data: true' in out.stdout


def live(args):
    """Follow the newest mission under LOG_ROOT, fire once."""
    log = open(args.out, 'a')

    def write(rec):
        log.write(json.dumps(rec) + '\n')
        log.flush()

    write({'wall': time.time(), 'kind': 'start', 'mode': args.mode,
           'world': args.world, 'detection_s': DETECTION_S,
           'margin_j': MARGIN_J, 'r_view_m': R_VIEW_M})
    mdir = None
    while mdir is None:
        found = sorted(glob.glob(os.path.join(args.log_root, '*',
                                              'config.json')))
        if found:
            mdir = os.path.dirname(found[-1])
        else:
            time.sleep(0.5)
    config = json.load(open(os.path.join(mdir, 'config.json')))
    pads, targets = PADS[args.world], TARGET[args.world]
    cycles, events, oc, oe = [], [], 0, 0
    seen = 0
    fired = False
    while True:
        new_c, oc = read_lines(os.path.join(mdir, 'cycles.jsonl'), oc)
        new_e, oe = read_lines(os.path.join(mdir, 'events.jsonl'), oe)
        cycles += new_c
        events += new_e
        if any(e.get('kind') == 'state' and e.get('new') == 'LANDED'
               for e in new_e):
            write({'wall': time.time(), 'kind': 'end', 'fired': fired})
            return
        decs = list(decisions(cycles, events))
        # A decision is complete once its path action is logged.
        while seen < len(decs) and decs[seen][2] is not None:
            cy, ch, path = decs[seen]
            seen += 1
            r = evaluate(cy, ch, path, config, pads, targets)
            r.update(wall=time.time(), kind='decision')
            if not fired and fires(r, args.mode):
                fired = True
                r['spawned'] = spawn(args.world)
                r['spawn_wall'] = time.time()
                r['kind'] = 'fire'
            write(r)
        time.sleep(0.1)


def replay(args):
    """Evaluate every decision of a finished mission, print a table."""
    mdir = Path(args.mission)
    config = json.load(open(mdir / 'config.json'))
    load = [json.loads(x) for x in (mdir / 'cycles.jsonl').read_text()
            .splitlines() if x.strip()]
    ev = [json.loads(x) for x in (mdir / 'events.jsonl').read_text()
          .splitlines() if x.strip()]
    pads, targets = PADS[args.pads], TARGET[args.pads]
    for cy, ch, path in decisions(load, ev):
        r = evaluate(cy, ch, path, config, pads, targets)
        if not r['relies']:
            continue
        print(f'{r["t"]:7.1f} {r["pad"]:>7} B {r["b_j"]:7.0f} '
              f'need-B+R {r["need_without_z_j"] - r["b_j"] + r["reserve_j"]:7.0f}'
              f' rests {r["rests"]!s:5} d0 {r["d_start_m"]:5.1f} '
              f'view {r["view"]!s:5} reach {r.get("reach")!s:5} '
              f'I1_alt {r.get("i1_without_z_j", float("nan")):7.0f} '
              f'critical {fires(r, "critical")!s:5} '
              f'first {fires(r, "first")!s:5}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    sub = ap.add_subparsers(dest='cmd', required=True)
    lv = sub.add_parser('live')
    lv.add_argument('log_root')
    lv.add_argument('--mode', default='critical',
                    choices=('critical', 'first', 'dry'))
    lv.add_argument('--world', default='w3', choices=tuple(PADS))
    lv.add_argument('--out', default='depot.jsonl')
    rp = sub.add_parser('replay')
    rp.add_argument('mission')
    rp.add_argument('--pads', default='w3', choices=tuple(PADS))
    args = ap.parse_args()
    (live if args.cmd == 'live' else replay)(args)


if __name__ == '__main__':
    main()
