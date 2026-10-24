#!/usr/bin/env python3
"""
S9.5 provoked on W3 at the return: the sequence of every mission where
the field of crates was placed, checked against the criterion of
README.md. Derived from 2026-10-23_s95_depot/analyse.py.

Per mission with the field placed (depot.jsonl, kind 'fire', spawned):
  - the return that fired it: its time, its target zone Z, its length,
    the zone's area and the height of the field
  - when Z left CONFIRMED, and the reason the registry gave: the delay
    from the start of the return to the invalidation, an upper bound of
    the detection delay (the trigger reads the log every 0.1 s)
  - the S9.5 re-evaluation at that cycle ('invariant' event with Z in
    its 'retired' list), its margin against the shrunk registry, and
    what the machine did within 0.5 s
  - the expected branch of S9.5: margin >= 0, I1 holds with another
    zone (steps 3 and 4), and no violation may follow; margin < 0, no
    zone within the guarantee (step 5), and the violation must be
    logged at that cycle, the aircraft still returning to the nearest
    reachable zone
  - I1 violations within 15 s after, and in the whole mission
  - the landing: end state, zone, whether it lies inside the field, and
    whether its truth cell is severe in the W3 truth
Missions where the field was not placed are counted, not analysed.

    python3 analyse.py        # ROS and the workspace sourced
"""

import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TRUTH = Path.home() / 'ros2_ws/droneAfter/worlds/w3_truth.npz'


def load(path):
    return [json.loads(x) for x in path.open() if x.strip()]


def severe_cell():
    """Function of a NED point: is its cell severe in the W3 truth."""
    from uav_core.terrain_analysis import analyse
    t = np.load(TRUTH)
    ned = -t['truth'].T
    lay = analyse(ned, np.ones(ned.shape, np.int32), 1.0)
    centres = t['centres']

    def severe(n, e):
        i = int(np.argmin(np.abs(centres - n)))
        j = int(np.argmin(np.abs(centres - e)))
        return bool(lay['slope'][i, j] > 12.0 or lay['step'][i, j] > 0.48)
    return severe


def reaction(events, t):
    """What the machine did within 0.5 s of the re-evaluation."""
    for e in events:
        if not t <= e['t'] <= t + 0.5:
            continue
        if e['kind'] == 'return':
            return f'return to zone {e["zone_id"]} ({e["length_m"]:.0f} m)'
        if e['kind'] == 'state' and e['new'] == 'EMERGENCY':
            return 'emergency'
    return 'continue'


def main():
    severe = severe_cell()
    runs = sorted(r for r in (HERE / 'runs').iterdir()
                  if r.is_dir() and not r.name.startswith('.'))
    placed = []
    for run in runs:
        recs = load(run / 'depot.jsonl') if (run / 'depot.jsonl').exists() \
            else []
        fire = next((r for r in recs if r.get('kind') == 'fire'
                     and r.get('spawned')), None)
        if fire is not None:
            placed.append((run, fire))
    print(f'{len(runs)} missions flown, field placed in {len(placed)}')
    ok_all = True
    for run, fire in placed:
        m = next((run / 'missions').iterdir())
        events = load(m / 'events.jsonl')
        cycles = load(m / 'cycles.jsonl')
        z, t0 = fire['zone_id'], fire['t']
        print(f'\n== {run.name}: field placed at {t0:.1f} s on zone {z}, '
              f'{fire["length_m"]:.0f} m ahead, zone {fire["area_m2"]:.0f} m2, '
              f'field at {fire["height_m"]:.1f} m ({fire["reason"]})')
        lost = next((e for e in events if e['kind'] == 'zone'
                     and e['zone_id'] == z and e['old'] == 'CONFIRMED'
                     and e['t'] >= t0), None)
        if lost is None:
            print('   Z never left CONFIRMED after the placement')
            ok_all = False
            continue
        inv = next((e for e in events if e['kind'] == 'invariant'
                    and z in e['retired'] and abs(e['t'] - lost['t']) < 0.05),
                   None)
        viol = [e for e in events if e['kind'] == 'invariant'
                and e['violated']]
        after = [v for v in viol if 0 < v['t'] - lost['t'] <= 15.0]
        end = next((e for e in reversed(events) if e['kind'] == 'state'
                    and e['new'] in ('COMMIT_LANDING', 'EMERGENCY')), None)
        v = cycles[-1]['vehicle_ned']
        landing_zone = None
        if end is not None and end['new'] == 'COMMIT_LANDING':
            landing_zone = int(end['why'].split()[2].rstrip(','))
        sev = severe(v[0], v[1])
        cn, ce = fire['touchdown_ne']
        half = fire['side_m'] / 2.0
        in_field = abs(v[0] - cn) <= half and abs(v[1] - ce) <= half
        print(f'   Z left CONFIRMED at {lost["t"]:.1f} s '
              f'({lost["t"] - t0:.1f} s after the placement): '
              f'{lost["new"]}, {lost["reason"]}')
        branch_ok = False
        if inv is not None:
            react = reaction(events, inv['t'])
            holds = inv['margin_j'] >= 0
            print(f'   I1 re-evaluated at that cycle: margin '
                  f'{inv["margin_j"]:.0f} J against zone {inv["zone_id"]}, '
                  f'violated {inv["violated"]}; reaction: {react}')
            if holds:
                branch_ok = not after
                print(f'   branch: I1 holds with another zone, violations '
                      f'within 15 s {len(after)} (0 expected)')
            else:
                branch_ok = inv['violated'] and react.startswith('return')
                print(f'   branch: no zone within the guarantee, violation '
                      f'logged {inv["violated"]}, return {react}')
        else:
            print('   no re-evaluation event at that cycle')
        print(f'   I1 violations in the mission: {len(viol)}')
        print(f'   landing: {end["new"] if end else "none"}, zone '
              f'{landing_zone}, on Z {landing_zone == z}, in the field '
              f'{in_field}, severe {sev}')
        ok = (branch_ok and end is not None
              and end['new'] == 'COMMIT_LANDING' and landing_zone != z
              and not in_field and not sev)
        print(f'   criterion met: {ok}')
        ok_all = ok_all and ok
    print(f'\nall placed missions meet the criterion: '
          f'{ok_all and bool(placed)}')


if __name__ == '__main__':
    main()
