"""
B5, follow-up. Where are the dangerous false accepts that the guard
leaves on the final W1 maps, and could the vehicle have used them?

Role: b5_guard_final.sh finds 12 dangerous landable cells on 10 of the
70 last maps of the final W1 campaign, after the confirmation guard.
For each one this prints the mission, the cell (north, east), the true
slope and step that make it dangerous, the truth zone, the distance to
the nearest truth zone boundary, and whether the cell lies in a
landable component that held a CONFIRMED zone of the registry at the
end of the mission (so C1 could have sent the vehicle there). It also
reads missions.csv for any landing on a dangerous cell.

Same measurement code as measure.py (2026-10-20_min_zone), so the
cells are those it counts.
"""
import importlib.util
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
# both experiment scripts are called measure.py: load the min_zone one
# under its own name; it imports the footprint_p4 one as 'measure'
_spec = importlib.util.spec_from_file_location(
    'mz_measure', ROOT / 'experiments' / '2026-10-20_min_zone' / 'measure.py')
mz = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mz)

from common import load, missions_csv  # noqa: E402
from style import zone_components  # noqa: E402

FP = mz.fp
RUNS = ROOT / 'experiments' / '2026-10-23_campaign_w1_final' / 'runs'
TRUTH = np.load(ROOT / 'worlds' / 'validation_truth.npz')
NAMES = [str(n) for n in TRUTH['zone_names']]
ZONE = TRUTH['zone'].T


def main():
    rows = missions_csv('W1')
    total = 0
    in_confirmed = 0
    for name, snap in FP.last_snapshots(RUNS):
        d = FP.load_map(snap)
        truth, found = mz.detections(d)
        kept = mz.holds_footprint(found)
        severe, _, _, _ = mz.v.severity(d, kept & ~truth['candidate'], 2.0)
        if not severe.any():
            continue
        rec = load(RUNS / name)
        last = [c for c in rec['cycles'] if c['state'] not in
                ('LANDED',)][-1]
        conf = [z['touchdown_ne'] for z in last['zones']
                if z['state'] == 'CONFIRMED']
        res = d['res']
        origin = d['sub_origin']
        comp = zone_components(kept, conf, res, origin)
        for i, j in zip(*np.nonzero(severe)):
            n = origin[0] + (i + 0.5) * res
            e = origin[1] + (j + 0.5) * res
            zi = ZONE[i, j]
            total += 1
            in_confirmed += bool(comp[i, j])
            print(f'{name:12s} N {n:6.1f} E {e:6.1f}  true slope '
                  f'{d["truth_layers"]["slope"][i, j]:5.1f} deg step '
                  f'{d["truth_layers"]["step"][i, j]:4.2f} m  truth zone '
                  f'{NAMES[zi]:9s}  map slope '
                  f'{d["map_layers"]["slope"][i, j]:4.1f} step '
                  f'{d["map_layers"]["step"][i, j]:4.2f}  in confirmed '
                  f'component: {bool(comp[i, j])}  landed_safe '
                  f'{rows[name]["landed_safe"]} '
                  f'({rows[name]["landing_truth_zone"]})')
    print(f'\n{total} dangerous cells after the guard, {in_confirmed} in a '
          f'component holding a CONFIRMED zone at the end of the mission')
    landed = [(k, r['landing_truth_zone']) for k, r in rows.items()
              if r['landed_safe'] != 'True']
    print(f'missions of the final W1 campaign not landed on ground '
          f'landable in the truth: {landed}')


if __name__ == '__main__':
    main()
