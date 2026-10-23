#!/usr/bin/env python3
"""
Return switches of this campaign and what followed them, per mission.

Written after round 2, when B3a075_r01 showed the switch flapping on a
zone confirmed and lost within 1 to 2 s, and landing into its reserve.
Per mission: switches to a cheaper zone, returns sent back by the loss
of the zone switched to ('retired en route'), whether the first I1
violation came within 15 s after a switch, and the energy above the
reserve once landed. Reads only the mission logs (S17).

    python3 switch_check.py [CAMPAIGN_DIR]
"""

import json
from pathlib import Path
import re
import sys

HERE = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent
SWITCH = re.compile(r'cheaper by|no longer reachable')


def load(path):
    return [json.loads(x) for x in path.open()]


for run in sorted((HERE / 'runs').iterdir()):
    if not run.is_dir() or run.name.startswith('.'):
        continue
    m = next((run / 'missions').iterdir())
    config = json.loads((m / 'config.json').read_text())
    events = load(m / 'events.jsonl')
    cycles = load(m / 'cycles.jsonl')
    reserve = config['reserve_fraction'] * (cycles[0]['remaining_j']
                                            + cycles[0]['consumed_j'])
    switches = [e for e in events if e['kind'] == 'return'
                and SWITCH.search(e['reason'])]
    back = [e for e in events if e['kind'] == 'return'
            and 'retired en route' in e['reason']]
    viol = [e for e in events if e['kind'] == 'invariant' and e['violated']]
    after = bool(viol) and any(0 <= viol[0]['t'] - s['t'] <= 15.0
                               for s in switches)
    print(f'{run.name:<11} {config["policy"]:<7} violations {len(viol):3d} '
          f'switches {len(switches)} sent back {len(back)} first violation '
          f'after a switch {after!s:<5} above R '
          f'{(cycles[-1]["remaining_j"] - reserve) / 1000:6.2f} kJ')
