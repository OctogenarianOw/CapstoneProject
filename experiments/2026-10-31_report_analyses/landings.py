"""Where did every C1 mission land, against the truth? (claim check)"""
from collections import Counter

from common import ALPHAS, CAMPAIGNS, missions_csv

for world in CAMPAIGNS:
    rows = [r for r in missions_csv(world).values() if r['config'] in ALPHAS]
    safe = sum(r['landed_safe'] == 'True' for r in rows)
    severe = sum(r.get('landed_severe') == 'True' for r in rows)
    reg = sum(r['landed_on_registry'] == 'True' for r in rows)
    zones = Counter(r['landing_truth_zone'] for r in rows
                    if r['landed_safe'] != 'True')
    print(f'{world}: {len(rows)} C1 missions, on registry {reg}, on ground '
          f'landable by design {safe}, on a dangerous cell {severe} '
          f'(column present: {"landed_severe" in rows[0]}); not landable '
          f'by design: {dict(zones)}')
