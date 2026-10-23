#!/usr/bin/env python3
"""
What a logged I1 violation means in energy, mission by mission.

I1 is evaluated every cycle as B - R - need, need being the energy of a
return planned from the vehicle cell (mission_policy.invariant_margin):
rest-to-rest legs (energy_model, the calibrated profile), the hover C1
charges per visit, and E_land. Two situations make that need larger
than what the aircraft will spend, and both occur at the end of a
mission, where the margin is smallest by design:
  - moving: a return planned from a moving aircraft pays an acceleration
    ramp it will not fly (about v / 2a = 0.83 s of hover at 5 m/s and
    3 m/s2, 240 J, plus the leg's fixed 65 J);
  - over the zone: if the cell under the aircraft has stopped being a
    landing cell (they are recomputed every cycle, I4 per cell), the
    plan adds a leg to the next one, about 411 J for one cell, which
    the aircraft does not fly: COMMIT_LANDING lands where it is.
Neither is a shortfall of energy; both are counted as violations. The
outcome that matters for safety is measured directly: the energy above
the reserve once landed.

Per mission:
  - violations and the smallest logged margin;
  - the leg charged at the first COMMIT_LANDING cycle, need - E_land -
    hover charge, 0 when the aircraft is over a landing cell;
  - the energy above the reserve once landed (the last cycle logged);
  - each violating cycle, one where the machine's violation count rose
    (after the commit the margin is no longer evaluated), classed as
    moving (over 1 m/s between cycles), commit (the first commit cycle)
    or other.

Reads only the mission logs (S17).

    python3 margin_check.py [CAMPAIGN_DIR]
"""

import json
import math
from pathlib import Path
import sys

HERE = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent
P_HOVER_W = 285.636
E_LAND_PLAN_J = 3387.0     # energy_model.PLANNING_2026_10_20
MOVING_MS = 1.0


def load(path):
    return [json.loads(x) for x in path.open()]


def mission(run):
    m = next((run / 'missions').iterdir())
    return (json.loads((m / 'config.json').read_text()),
            load(m / 'cycles.jsonl'))


def main():
    print(f'{"mission":<11} {"viol":>4} {"min J":>7} {"commit leg J":>12} '
          f'{"above R J":>9}  violating cycles')
    total = {'moving': 0, 'commit': 0, 'other': 0}
    for run in sorted((HERE / 'runs').iterdir()):
        if not run.is_dir() or run.name.startswith('.'):
            continue
        config, cycles = mission(run)
        reserve = config['reserve_fraction'] * (cycles[0]['remaining_j']
                                                + cycles[0]['consumed_j'])
        margins = [c['i1_margin_j'] for c in cycles
                   if c['i1_margin_j'] is not None]
        commit = next((i for i, c in enumerate(cycles)
                       if c['state'] == 'COMMIT_LANDING'), None)
        leg = math.nan
        if commit is not None and cycles[commit]['i1_margin_j'] is not None:
            c = cycles[commit]
            need = c['remaining_j'] - reserve - c['i1_margin_j']
            leg = need - E_LAND_PLAN_J - P_HOVER_W * c['overhead_s']
        classes = []
        for i, c in enumerate(cycles):
            if i == 0 or c['violations'] <= cycles[i - 1]['violations']:
                continue
            if i == commit:
                kind = 'commit'
            elif math.dist(
                    c['vehicle_ned'][:2], cycles[i - 1]['vehicle_ned'][:2]) \
                    / max(c['t'] - cycles[i - 1]['t'], 1e-3) > MOVING_MS:
                kind = 'moving'
            else:
                kind = 'other'
            classes.append(f'{c["t"]:.0f}:{kind}:{c["i1_margin_j"]:.0f}')
            if config['policy'] in ('c1_gain', 'c1_c2'):
                total[kind] += 1
        n_viol = len(classes)
        above = cycles[-1]['remaining_j'] - reserve
        shown = ' '.join(classes[:6]) + (' ...' if n_viol > 6 else '')
        print(f'{run.name:<11} {n_viol:4d} {min(margins):7.0f} {leg:12.0f} '
              f'{above:9.0f}  {shown}')
    print(f'\nviolating cycles of the C1 missions by class: {total}')


if __name__ == '__main__':
    main()
