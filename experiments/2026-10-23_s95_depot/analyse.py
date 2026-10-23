#!/usr/bin/env python3
"""
S9.5 provoked on W3: the sequence of every mission where the depot was
placed, checked against the criterion of README.md.

Per mission with the depot placed (depot.jsonl, kind 'fire', spawned):
  - the decision that fired it: its time, the zone Z relied on, whether
    the guarantee also rested on Z alone (rests, the critical case)
  - when Z left CONFIRMED, and the reason the registry gave: the delay
    from the decision to the invalidation, an upper bound of the
    detection delay (the trigger reads the log every 0.1 s)
  - the S9.5 re-evaluation at that cycle ('invariant' event with Z in
    its 'retired' list) and its margin against the shrunk registry
  - what the machine did within 0.5 s: continue (I1 holds with another
    zone, S9.5 step 3) or return (step 4) or emergency (step 5)
  - I1 violations within 15 s after, and in the whole mission
  - the landing: end state, zone, and whether its truth cell is severe
    in the truth with the depot (worlds/w3_depot_truth.npz)
Missions where the depot was not placed are counted, not analysed.

    python3 analyse.py        # ROS and the workspace sourced
"""

import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TRUTH = Path.home() / 'ros2_ws/droneAfter/worlds/w3_depot_truth.npz'


def load(path):
    return [json.loads(x) for x in path.open() if x.strip()]


def severe_cell():
    """Function of a NED point: is its cell severe in the depot truth."""
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
        if e['kind'] == 'abort':
            return f'abort ({e["reason"]})'
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
    print(f'{len(runs)} missions flown, depot placed in {len(placed)}')
    ok_all = True
    for run, fire in placed:
        m = next((run / 'missions').iterdir())
        events = load(m / 'events.jsonl')
        cycles = load(m / 'cycles.jsonl')
        z, t0 = fire['zone_id'], fire['t']
        lost = next((e for e in events if e['kind'] == 'zone'
                     and e['zone_id'] == z and e['old'] == 'CONFIRMED'
                     and e['t'] >= t0), None)
        print(f'\n== {run.name}: decision at {t0:.1f} s on zone {z}, '
              f'rests on Z alone {fire.get("rests")}, I1 without Z at the '
              f'decision {fire.get("i1_without_at_decision_j", math.nan):.0f} J')
        if lost is None:
            print('   Z never left CONFIRMED after the placement')
            ok_all = False
            continue
        inv = next((e for e in events if e['kind'] == 'invariant'
                    and z in e['retired'] and abs(e['t'] - lost['t']) < 0.05),
                   None)
        viol = [e for e in events if e['kind'] == 'invariant'
                and e['violated']]
        after = [v for v in viol if 0 <= v['t'] - lost['t'] <= 15.0]
        end = next((e for e in reversed(events) if e['kind'] == 'state'
                    and e['new'] in ('COMMIT_LANDING', 'EMERGENCY')), None)
        v = cycles[-1]['vehicle_ned']
        landing_zone = None
        if end is not None and end['new'] == 'COMMIT_LANDING':
            landing_zone = int(end['why'].split()[2].rstrip(','))
        sev = severe(v[0], v[1])
        print(f'   Z left CONFIRMED at {lost["t"]:.1f} s '
              f'({lost["t"] - t0:.1f} s after the decision): '
              f'{lost["new"]}, {lost["reason"]}')
        if inv is not None:
            print(f'   I1 re-evaluated at that cycle: margin '
                  f'{inv["margin_j"]:.0f} J against zone {inv["zone_id"]}, '
                  f'violated {inv["violated"]}; reaction: '
                  f'{reaction(events, inv["t"])}')
        else:
            print('   no re-evaluation event at that cycle')
        print(f'   I1 violations within 15 s: {len(after)}, in the '
              f'mission: {len(viol)}')
        print(f'   landing: {end["new"] if end else "none"}, zone '
              f'{landing_zone}, on Z {landing_zone == z}, severe {sev}')
        ok = (inv is not None and not after and end is not None
              and end['new'] == 'COMMIT_LANDING' and landing_zone != z
              and not sev)
        print(f'   criterion met: {ok}')
        ok_all = ok_all and ok
    print(f'\nall placed missions meet the criterion: '
          f'{ok_all and bool(placed)}')


if __name__ == '__main__':
    main()
