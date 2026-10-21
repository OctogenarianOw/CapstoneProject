#!/usr/bin/env python3
"""
How often does the critical case of S9.5 arise by itself on W2?

S9.5: the aircraft flies to a frontier on a guarantee that rests on a
zone Z, and an observation on the way invalidates Z. W3 has to make it
happen. Before choosing how, this replays the 50 C1 missions of
2026-10-21_campaign_w2 (B3, every alpha) from their logs alone, with
the conditions of the W3 trigger (../w3_depot.py, the same code), over
the two remote pads of W2:

  confirmed  missions in which a remote pad was ever CONFIRMED
  relies     frontier decisions whose return zone is a remote pad
  rests      ... and not admissible without that pad
  view       ... and the whole pad in view during the transit
  reach      ... and a zone still reachable once the pad is gone

    python3 critical_case_w2.py [campaign_dir]
"""

import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import w3_depot  # noqa: E402

CAMPAIGN = Path(sys.argv[1] if len(sys.argv) > 1 else Path.home()
                / 'ros2_ws/droneAfter/experiments/2026-10-21_campaign_w2')
PADS, TARGETS = w3_depot.PADS['w2'], w3_depot.TARGET['w2']


def load(path):
    """Records of one JSON lines file."""
    return [json.loads(x) for x in path.read_text().splitlines()
            if x.strip()]


def main():
    mdirs = sorted(CAMPAIGN.glob('runs/B3a*_r*/missions/*'))
    confirmed, recs = {}, []
    for m in mdirs:
        run = m.parents[1].name
        cycles, events = load(m / 'cycles.jsonl'), load(m / 'events.jsonl')
        config = json.load(open(m / 'config.json'))
        for cy in cycles:
            for z in cy['zones']:
                pad = (None if z['provisional']
                       else w3_depot.pad_of(z['touchdown_ne'], PADS))
                if z['state'] == 'CONFIRMED' and pad in TARGETS:
                    confirmed.setdefault((run, pad),
                                         (cy['t'], cy['remaining_j']))
        for cy, ch, path in w3_depot.decisions(cycles, events):
            r = w3_depot.evaluate(cy, ch, path, config, PADS, TARGETS)
            if r['relies']:
                r['run'] = run
                recs.append(r)
    print(f'{len(mdirs)} C1 missions on W2')
    print(f'remote pad ever confirmed: {len(confirmed)} (mission, pad):')
    for (run, pad), (t, b) in sorted(confirmed.items()):
        print(f'  {run} {pad} at {t:.0f} s, {b / 1e3:.1f} kJ left')
    steps = [('relies', lambda r: True),
             ('rests', lambda r: r['rests']),
             ('view', lambda r: r['rests'] and r['view']),
             ('reach', lambda r: r['rests'] and r['view'] and r['reach'])]
    for name, keep in steps:
        sel = [r for r in recs if keep(r)]
        print(f'decisions, {name:6s}: {len(sel):3d} in '
              f'{len({r["run"] for r in sel})} missions')
    print('\nrun          t      pad      B kJ  need-B+R  d0 m  view  '
          'reach  I1 without pad')
    for r in recs:
        print(f'{r["run"]:<12} {r["t"]:6.1f} {r["pad"]:>7} '
              f'{r["b_j"] / 1e3:5.1f} '
              f'{r["need_without_z_j"] - r["b_j"] + r["reserve_j"]:9.0f} '
              f'{r["d_start_m"]:5.1f}  {r["view"]!s:5} '
              f'{r.get("reach")!s:5}  {r.get("i1_without_z_j", "")}')


if __name__ == '__main__':
    main()
