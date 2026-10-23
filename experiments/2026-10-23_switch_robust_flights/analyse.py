#!/usr/bin/env python3
"""
Robust return switch on W3: the flights after against those before.

A copy of 2026-10-23_return_switch_flights/analyse.py. Before: the 10
C1 missions of rounds 1 and 2 of 2026-10-23_campaign_w3 (B3, alpha 0
to 1), flown with the switch but without the robustness condition.
After: the missions of runs/ here, same protocol, built from the
branch switch-robust. One count more, the switches refused because the
loss of their zone would have broken I1 ('switch_refused' events).
Reads only the mission logs (S17) and the W3 truth.

Per mission:
  switches   'return' events turning to a cheaper zone (or away from a
             target no longer reachable), with the saving logged
  ratio      energy spent from the first return to the commit, over
             the path energy planned at that first return: under 1
             when a switch shortened the return, about 1 otherwise,
             over 1 when the registry sent it farther (S9.5)
  above R    energy left over the reserve once landed (under 0: the
             reserve was used)
  viol       I1 violations, and those within 15 s after a switch
  landing    end state, and whether the truth cell under it is severe

    python3 analyse.py        # ROS and the workspace sourced
"""

import json
import math
from pathlib import Path
import re

import numpy as np

HERE = Path(__file__).resolve().parent
BEFORE = HERE.parent / '2026-10-23_campaign_w3' / 'runs'
TRUTH = Path.home() / 'ros2_ws/droneAfter/worlds/w3_truth.npz'
SWITCH = re.compile(r'cheaper by (\d+) J|no longer reachable')


def load(path):
    return [json.loads(x) for x in path.open()]


def severe_cell():
    """Function of a NED point: is its truth cell severe (S8)."""
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


def mission(run, severe):
    m = next((run / 'missions').iterdir())
    config = json.loads((m / 'config.json').read_text())
    events = load(m / 'events.jsonl')
    cycles = load(m / 'cycles.jsonl')
    reserve = config['reserve_fraction'] * (cycles[0]['remaining_j']
                                            + cycles[0]['consumed_j'])
    returns = [e for e in events if e['kind'] == 'return']
    refused = [e for e in events if e['kind'] == 'switch_refused']
    switches = [e for e in returns if SWITCH.search(e['reason'])]
    savings = [int(SWITCH.search(e['reason']).group(1))
               for e in switches if SWITCH.search(e['reason']).group(1)]
    commit = next((c for c in cycles if c['state'] == 'COMMIT_LANDING'),
                  None)
    ratio = math.nan
    if returns and commit is not None:
        at = [c for c in cycles if c['t'] <= returns[0]['t']]
        if at and returns[0]['energy_j'] > 0:
            ratio = ((at[-1]['remaining_j'] - commit['remaining_j'])
                     / returns[0]['energy_j'])
    viol = [e for e in events if e['kind'] == 'invariant' and e['violated']]
    after_switch = sum(1 for v in viol if any(
        0 <= v['t'] - s['t'] <= 15.0 for s in switches))
    end = next((e for e in reversed(events) if e['kind'] == 'state'
                and e['new'] in ('COMMIT_LANDING', 'EMERGENCY')), None)
    v = cycles[-1]['vehicle_ned']
    return {'run': run.name, 'switches': len(switches),
            'refused': len(refused), 'savings': savings, 'returns': len(returns), 'ratio': ratio,
            'above_r': cycles[-1]['remaining_j'] - reserve,
            'viol': len(viol), 'viol_after_switch': after_switch,
            'end': end['new'] if end else '',
            'severe': severe(v[0], v[1]),
            'switch_reasons': [e['reason'] for e in switches]}


def table(rows, title):
    print(f'\n== {title}: {len(rows)} missions')
    print(f'{"mission":<11} {"returns":>7} {"switches":>8} {"ratio":>6} '
          f'{"above R kJ":>10} {"viol":>4} {"after sw":>8} '
          f'{"end":<15} {"severe":>6}  savings J')
    for r in rows:
        print(f'{r["run"]:<11} {r["returns"]:7d} {r["switches"]:8d} '
              f'{r["ratio"]:6.2f} {r["above_r"] / 1000:10.2f} '
              f'{r["viol"]:4d} {r["viol_after_switch"]:8d} {r["end"]:<15} '
              f'{r["severe"]!s:>6}  {r["savings"]}')


def main():
    severe = severe_cell()
    before = [mission(r, severe) for r in sorted(BEFORE.glob('B3a*'))]
    after = [mission(r, severe) for r in sorted((HERE / 'runs').glob('B3a*'))
             if not r.name.startswith('.')]
    table(before, 'before, 2026-10-23_campaign_w3 rounds 1 and 2')
    table(after, 'after, robust return switch')
    for name, rows in (('before', before), ('after', after)):
        if not rows:
            continue
        sw = [r for r in rows if r['switches']]
        ratio = [r['ratio'] for r in rows if np.isfinite(r['ratio'])]
        print(f'\n{name}: switches {sum(r["switches"] for r in rows)} in '
              f'{len(sw)} missions, refused '
              f'{sum(r["refused"] for r in rows)}; return ratio median '
              f'{np.median(ratio):.2f} (min {min(ratio):.2f}, '
              f'max {max(ratio):.2f}); in the missions with a switch '
              f'{[round(r["ratio"], 2) for r in sw]}')
        print(f'{name}: above the reserve median '
              f'{np.median([r["above_r"] for r in rows]) / 1000:.2f} kJ, '
              f'reserve used in {sum(r["above_r"] < 0 for r in rows)}; '
              f'missions with an I1 violation '
              f'{sum(r["viol"] > 0 for r in rows)}, violations within '
              f'15 s after a switch {sum(r["viol_after_switch"] for r in rows)}'
              f'; emergencies {sum(r["end"] == "EMERGENCY" for r in rows)}, '
              f'severe landings {sum(r["severe"] for r in rows)}')


if __name__ == '__main__':
    main()
