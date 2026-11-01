"""
Was the cell under the landing commit still landable when the land
command was sent? Measured on every C1 mission of the final campaigns.

Role: filming a provoked loss of zone on 31 Oct (2026-10-31_video,
w3_depot_4) showed a commit sent over a cell that the current map
refused: the crate field covered only part of the target zone, the zone
stayed CONFIRMED through its other cells, and the return, planned before
the crates fell, flew on to its original end and committed there
(mission_fsm._returning commits on arrival without checking the cell).
This script measures whether that ever happened in the 150 C1 missions
the report rests on.

Method: for each mission, the position at the COMMIT_LANDING transition
and the last map snapshot logged before it (at most 10 s older), replayed
through the flight perception code (build_world): is the commit cell a
landing candidate of that map? A snapshot up to 10 s old can miss a
refusal of the last seconds, so the test is lenient towards the code;
the truth zone under the landing is printed as a second check.

    bash run.sh commit_cell.py
"""
import numpy as np

from common import ALPHAS, CAMPAIGNS, load, missions_csv, runs
from style import perceive, snapshots


def check(rec):
    ev = rec['events']
    commit = next((e for e in ev if e['kind'] == 'state'
                   and e['new'] == 'COMMIT_LANDING'), None)
    if commit is None:
        return None
    cyc = rec['cycles']
    ts = np.array([c['t'] for c in cyc])
    c = cyc[int(np.argmin(np.abs(ts - commit['t'])))]
    n, e = c['ned'][:2]
    snaps = [(t, p) for t, p in snapshots(rec['mission_dir'])
             if t <= commit['t']]
    t_map, path = snaps[-1]
    cls, land, w, ext, res, origin = perceive(path, (n, e), rec['config'])
    i = int((n - origin[0]) / res)
    j = int((e - origin[1]) / res)
    near = land[max(i - 1, 0):i + 2, max(j - 1, 0):j + 2]
    return {'t_commit': commit['t'], 'map_age': commit['t'] - t_map,
            'landable': bool(land[i, j]), 'landable_3x3': bool(near.any())}


def main():
    bad = []
    for world in CAMPAIGNS:
        rows = missions_csv(world)
        n = ok = 0
        for cfg, r, d in runs(world):
            if cfg not in ALPHAS:
                continue
            res = check(load(d))
            if res is None:
                continue
            n += 1
            ok += res['landable']
            if not res['landable']:
                bad.append((world, d.name, res,
                            rows[d.name]['landing_truth_zone']))
        print(f'{world}: {ok} of {n} commits over a cell the map still '
              f'held landable')
    print('commits over a cell the map no longer held landable:')
    for world, name, res, truth in bad:
        print(f'  {world} {name}: map {res["map_age"]:.1f} s old, landable '
              f'within one cell {res["landable_3x3"]}, truth under the '
              f'landing: {truth}')


if __name__ == '__main__':
    main()
