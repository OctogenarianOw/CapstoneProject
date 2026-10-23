#!/usr/bin/env python3
"""
Registry shrinks met in the W3 campaign without a script (S9.5).

S9.5 is to be provoked on purpose (the depot trigger of
experiments/w3_depot.py), but a large mixed world also shrinks the
registry on its own: a confirmed zone demoted or retired as the map
improves. This script lists those events in the C1 missions, from the
logs only (S17), so the deliberate experiment can be read against what
happens unprovoked.

An S9.5 event is an 'invariant' event whose 'retired' list is not empty:
the machine re-evaluated I1 at the registry update (S9.5 steps 1 and 2).
For each one it reports
  - the state, and whether the lost zone was the one in use: the target
    of the return under way, or the zone the last guarantee rested on
    (the 'guarantee' event of the target being flown, S9.5 logging)
  - the reason the registry gave, per lost zone
  - the I1 margin before (last cycle before the event) and at the event
  - what the machine did within 0.5 s: return, abort, emergency, nothing
  - the smallest margin and the cycles in violation over the next 15 s

and, over all the targets selected, how many had a guarantee resting on
one zone (rests_on_zone: without that zone the need exceeds the budget).

Measured, not judged: whether the reaction is the one S9.5 asks for is
read from the table. The margin after the event is the one the
machine computed against the shrunk registry; a violation that follows
it is a model shortfall of the flight that follows, not of the event.

    python3 s95_natural.py [CAMPAIGN_DIR]
"""

import json
import math
from pathlib import Path
import sys

HERE = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent
WINDOW_S = 15.0


def load(path):
    return [json.loads(x) for x in path.open()]


def mission_events(run):
    mission = next((run / 'missions').iterdir())
    config = json.loads((mission / 'config.json').read_text())
    return (config, load(mission / 'events.jsonl'),
            load(mission / 'cycles.jsonl'))


def in_use(events, t, state):
    """Zone the aircraft relied on just before t, and how."""
    before = [e for e in events if e['t'] < t]
    if state in ('RETURNING', 'COMMIT_LANDING'):
        ret = next((e for e in reversed(before) if e['kind'] == 'return'),
                   None)
        return (ret['zone_id'], 'return target') if ret else (None, '')
    g = next((e for e in reversed(before) if e['kind'] == 'guarantee'),
             None)
    if g is None:
        return None, ''
    how = 'guarantee rests on it' if g['rests_on_zone'] else 'guarantee zone'
    return g['zone_id'], how


def reaction(events, t):
    """What the machine did within 0.5 s of the event."""
    after = [e for e in events if t <= e['t'] <= t + 0.5]
    for e in after:
        if e['kind'] == 'return':
            return f'return to zone {e["zone_id"]} ({e["length_m"]:.0f} m)'
        if e['kind'] == 'state' and e['new'] == 'EMERGENCY':
            return 'emergency'
        if e['kind'] == 'abort':
            return f'abort ({e["reason"]})'
    return 'continue'


def main():
    rows = []
    decisions = resting = 0
    for run in sorted((HERE / 'runs').iterdir()):
        if not run.is_dir() or run.name.startswith('.'):
            continue
        config, events, cycles = mission_events(run)
        if config['policy'] not in ('c1_gain', 'c1_c2'):
            continue
        g = [e for e in events if e['kind'] == 'guarantee']
        decisions += len(g)
        resting += sum(e['rests_on_zone'] for e in g)
        for e in events:
            if e['kind'] != 'invariant' or not e['retired']:
                continue
            t = e['t']
            # A cycle record is stamped at its input, up to 0.3 s before
            # the events it produced: the cycle of the event is the last
            # one before it, the one before is the previous registry.
            prev = [c for c in cycles if c['t'] < t]
            state = prev[-1]['state'] if prev else ''
            before = prev[-2]['i1_margin_j'] if len(prev) > 1 else None
            zid, how = in_use(events, t, state)
            reasons = sorted({x['reason'] for x in events
                              if x['kind'] == 'zone'
                              and x['zone_id'] in e['retired']
                              and x['old'] == 'CONFIRMED'
                              and abs(x['t'] - t) < 0.05})
            nxt = [c for c in cycles if t <= c['t'] <= t + WINDOW_S
                   and c['i1_margin_j'] is not None]
            low = min((c['i1_margin_j'] for c in nxt), default=math.nan)
            bad = sum(1 for x in events if x['kind'] == 'invariant'
                      and x['violated'] and t <= x['t'] <= t + WINDOW_S)
            rows.append({
                'run': run.name, 't': t, 'state': state,
                'lost': e['retired'], 'reasons': '; '.join(reasons),
                'in_use': zid in e['retired'], 'how': how if zid in
                e['retired'] else '',
                'before_j': before, 'at_j': e['margin_j'],
                'reaction': reaction(events, t),
                'low_j': low, 'violations': bad})

    print(f'{decisions} targets selected in the C1 missions, {resting} of '
          f'them with a guarantee resting on one zone (rests_on_zone)')
    print(f'{len(rows)} registry shrinks in the C1 missions of {HERE.name}')
    used = [r for r in rows if r['in_use']]
    print(f'{len(used)} of them took the zone in use')
    print(f'{sum(r["violations"] > 0 for r in rows)} followed by an I1 '
          f'violation within {WINDOW_S:.0f} s\n')
    hdr = (f'{"mission":<11} {"t s":>6} {"state":<14} {"lost":<10} '
           f'{"in use":<22} {"before J":>9} {"at J":>9} '
           f'{"low 15 s J":>10} {"viol":>4}  reaction; reasons')
    print(hdr)
    for r in rows:
        fmt = lambda v: f'{v:9.0f}' if v is not None and math.isfinite(v) \
            else f'{"n/a":>9}'
        print(f'{r["run"]:<11} {r["t"]:6.1f} {r["state"]:<14} '
              f'{",".join(map(str, r["lost"])):<10} '
              f'{r["how"] or "no":<22} {fmt(r["before_j"])} '
              f'{fmt(r["at_j"])} {fmt(r["low_j"]):>10} '
              f'{r["violations"]:4d}  {r["reaction"]}; {r["reasons"]}')


if __name__ == '__main__':
    main()
