#!/usr/bin/env python3
"""
What becomes of a confirmed zone demoted on the pass count (I4), W3.

Written on 23 Oct, after the README of 22 Oct read the 20 demotions of
this campaign as splits (the identity going to the larger part, the
cells passed twice to the other). This script tests that reading on
the logs (S17), demotion by demotion, from the cycle before and the
cycle of the demotion:
  - split: a zone newly detected in the same cycle has 2 passes, so the
    cells passed twice went to it;
  - fragmented: new zones in the same cycle, none with 2 passes;
  - shrunk: no new zone; the zone lost cells, and with them its landing
    cells (whole footprint passed twice).
For each: area and landing area before and after, and the time since
the zone was last confirmed.

    python3 demotions.py [CAMPAIGN_DIR]
"""

import json
import math
from pathlib import Path
import sys

HERE = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent


def load(path):
    return [json.loads(x) for x in path.open()]


def main():
    rows = []
    for run in sorted((HERE / 'runs').iterdir()):
        if not run.is_dir() or run.name.startswith('.'):
            continue
        mission = next((run / 'missions').iterdir())
        events = load(mission / 'events.jsonl')
        cycles = load(mission / 'cycles.jsonl')
        since = {}
        for e in events:
            if e['kind'] != 'zone':
                continue
            if e['new'] == 'CONFIRMED':
                since[e['zone_id']] = e['t']
            if not (e['old'] == 'CONFIRMED' and e['new'] == 'CANDIDATE'
                    and 'pass' in e['reason']):
                continue
            t = e['t']
            # A cycle record is stamped up to 0.3 s before its events.
            before = [c for c in cycles if c['t'] < t - 0.5][-1]
            at = min((c for c in cycles if c['t'] > t - 0.5),
                     key=lambda c: c['t'])
            old = next(z for z in before['zones'] if z['id'] == e['zone_id'])
            now = next((z for z in at['zones'] if z['id'] == e['zone_id']),
                       None)
            new_ids = {x['zone_id'] for x in events
                       if x['kind'] == 'zone' and abs(x['t'] - t) < 0.05
                       and x['old'] in ('', 'RETIRED')
                       and x['new'] == 'CANDIDATE'}
            news = [z for z in at['zones'] if z['id'] in new_ids]
            best = max(news, key=lambda z: (z['passes'], z['area_m2']),
                       default=None)
            kind = ('split' if best and best['passes'] >= 2
                    else 'fragmented' if news else 'shrunk')
            rows.append({
                'run': run.name, 't': t, 'zone': e['zone_id'], 'kind': kind,
                'area': (old['area_m2'], now['area_m2'] if now else 0.0),
                'landing': (old['landing_m2'],
                            now['landing_m2'] if now else 0.0),
                'age': t - since.get(e['zone_id'], math.nan),
                'to_new_m': (math.dist(best['touchdown_ne'],
                                       old['touchdown_ne'])
                             if best else math.nan)})

    print(f'{"mission":<11} {"t s":>6} {"zone":>4} {"kind":<10} '
          f'{"area m2":>11} {"landing m2":>11} {"age s":>6}')
    for r in rows:
        print(f'{r["run"]:<11} {r["t"]:6.1f} {r["zone"]:4d} {r["kind"]:<10} '
              f'{r["area"][0]:5.0f}>{r["area"][1]:5.0f} '
              f'{r["landing"][0]:5.0f}>{r["landing"][1]:5.0f} '
              f'{r["age"]:6.1f}')
    n = len(rows)
    for kind in ('split', 'fragmented', 'shrunk'):
        print(f'{kind:<10} {sum(r["kind"] == kind for r in rows)} of {n}')
    for limit in (5.0, 10.0):
        print(f'within {limit:.0f} s of the confirmation: '
              f'{sum(r["age"] <= limit for r in rows)} of {n}')
    print(f'landing area 30 m2 or less before: '
          f'{sum(r["landing"][0] <= 30 for r in rows)} of {n}')


if __name__ == '__main__':
    main()
