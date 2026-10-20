#!/usr/bin/env python3
"""
Metrics of spec 5.3 for every mission of the W2 campaign, then the tables.

A copy of 2026-10-20_campaign_w1_fix/summarise.py with the W2 truth
(safe ground: the flat pads) and the seven configurations flown on W2
(B2 is not flown, B3a100 ranks as it does: run.sh). The metrics and
the statistics are unchanged, so W1 and W2 tables read the same.

Reads only the mission logs (S17): runs/<CONFIG>_rNN/ with config.json,
events.jsonl, cycles.jsonl and the map snapshots, plus px4.log and the
ground truth of W1 for the landing cell. Writes missions.csv, one row
per mission, and prints per configuration the median and interquartile
range of each metric, with the counts for the yes/no ones.

Statistics, chosen before the results came in:
  - median and IQR, not mean (spec 5.4, ten runs, no normality)
  - Mann-Whitney U, two sided, with the rank-biserial correlation as the
    effect size. Not Wilcoxon signed-rank: spec 5.4 asks for paired
    runs on shared seeds, but a SITL run is not seeded, its variation
    comes from process scheduling, so run k of one configuration has
    nothing in common with run k of another. Pairing them would invent
    a structure the data does not have
  - gate F: Spearman correlation of alpha with each metric over the 50
    B3 missions, and the medians per alpha to see monotonicity

    python3 summarise.py [CAMPAIGN_DIR]
"""

import csv
import json
import math
from pathlib import Path
import re
import sys

import numpy as np
from scipy import stats

HERE = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent
TRUTH = Path.home() / 'ros2_ws/droneAfter/worlds/w2_truth.npz'
SAFE_TRUTH_ZONES = ('flat',)
P_HOVER_W = 285.636
ORDER = ['B0', 'B1', 'B3a000', 'B3a025', 'B3a050', 'B3a075', 'B3a100']


def num(v):
    """Read a logged float, None as nan."""
    return float(v) if v is not None else math.nan


def truth_zone(n, e):
    """Name of the truth zone under a NED point, None outside the truth."""
    t = np.load(TRUTH)
    centres = t['centres']
    zone_ned = t['zone'].T
    if not (abs(n) <= centres.max() + 0.5 and abs(e) <= centres.max() + 0.5):
        return None
    i = int(np.argmin(np.abs(centres - n)))
    j = int(np.argmin(np.abs(centres - e)))
    return str(t['zone_names'][zone_ned[i, j]])


def observed_fraction(mission, half_m):
    """Fraction of the area OBSERVED (S6) on the last map snapshot."""
    from uav_core.observation import classify, OBSERVED
    from uav_core.world_model import area_mask
    snaps = sorted((mission / 'maps').glob('map_*.npz'))
    if not snaps:
        return math.nan
    m = np.load(snaps[-1])
    state = classify(m['count'], m['variance'])
    area = area_mask(state.shape, float(m['resolution']),
                     (float(m['origin_north']), float(m['origin_east'])),
                     half_m)
    return float((state[area] == OBSERVED).mean())


def metrics(run):
    """One row of missions.csv from one mission directory."""
    mission = next((run / 'missions').iterdir())
    config = json.loads((mission / 'config.json').read_text())
    events = [json.loads(x) for x in (mission / 'events.jsonl').open()]
    cycles = [json.loads(x) for x in (mission / 'cycles.jsonl').open()]
    last = cycles[-1]
    states = [e for e in events if e['kind'] == 'state']
    t_takeoff = next(e['t'] for e in states if e['new'] == 'TAKEOFF')
    t_landed = next((e['t'] for e in states if e['new'] == 'LANDED'),
                    math.nan)
    end = next((e for e in reversed(states)
                if e['new'] in ('COMMIT_LANDING', 'EMERGENCY')), None)
    first_slz = next((e['t'] for e in events if e['kind'] == 'zone'
                      and e['new'] == 'CONFIRMED'
                      and not e['reason'].startswith('take-off')), math.nan)
    margins = [num(c['i1_margin_j']) for c in cycles
               if c['i1_margin_j'] is not None]
    v = last['vehicle_ned']
    zones = [z for z in last['zones'] if z['state'] == 'CONFIRMED']
    near = min((math.hypot(v[0] - z['touchdown_ne'][0],
                           v[1] - z['touchdown_ne'][1]) for z in zones),
               default=math.inf)
    # Distance to the nearest confirmed zone, averaged over the flight.
    dists = []
    for c in cycles:
        if c['state'] in ('INIT', 'TAKEOFF', 'LANDED') or not c['zones']:
            continue
        p = c['vehicle_ned']
        conf = [z['touchdown_ne'] for z in c['zones']
                if z['state'] == 'CONFIRMED']
        if conf:
            dists.append(min(math.hypot(p[0] - a, p[1] - b)
                             for a, b in conf))
    # Cause of the first violation: a confirmed zone leaving the registry
    # in the 3 s before it (S9.5, the registry shrinks under the
    # guarantee; the margin can stay just positive at the cycle of the
    # loss and cross zero one cycle later, while the aircraft brakes),
    # with the reason the registry gave, or none; and whether the zone
    # was CONFIRMED again within 5 s.
    first_v = next((e for e in events if e['kind'] == 'invariant'
                    and e['violated']), None)
    cause, back = '', ''
    if first_v is not None:
        losses = [e for e in events if e['kind'] == 'zone'
                  and e['old'] == 'CONFIRMED' and e['new'] != 'CONFIRMED'
                  and -3.0 <= e['t'] - first_v['t'] <= 0.5]
        lost = sorted({e['zone_id'] for e in losses})
        reasons = sorted({e['reason'] for e in losses})
        cause = (f'registry shrink ({"; ".join(reasons)})' if lost
                 else 'no zone lost')
        if lost:
            back = all(any(e['kind'] == 'zone' and e['zone_id'] == z
                           and e['new'] == 'CONFIRMED'
                           and 0 < e['t'] - first_v['t'] <= 5.0
                           for e in events) for z in lost)
    pass_demotions = sum(1 for e in events if e['kind'] == 'zone'
                         and e['old'] == 'CONFIRMED'
                         and e['new'] == 'CANDIDATE' and 'pass' in e['reason'])
    area = last['seen_in_area'] + last['unknown_in_area']
    consumed = num(last['consumed_j'])
    px4 = (run / 'px4.log').read_text(errors='replace')
    zone_name = truth_zone(v[0], v[1])
    return {
        'config': run.name.rsplit('_r', 1)[0],
        'run': run.name.rsplit('_r', 1)[1],
        'policy': config['policy'],
        'alpha': config['alpha'],
        'git': config.get('git_head', '')[:7],
        'violations': int(last['violations']),
        'min_margin_kj': min(margins) / 1000 if margins else math.nan,
        'coverage_seen': last['seen_in_area'] / area,
        'coverage_observed': observed_fraction(mission,
                                               config['area_half_m']),
        'energy_left_kj': num(last['remaining_j']) / 1000,
        'consumed_kj': consumed / 1000,
        'cells_per_kj': last['seen_in_area'] / (consumed / 1000),
        'end_state': end['new'] if end else '',
        'end_why': end['why'] if end else '',
        'emergency': bool(end and end['new'] == 'EMERGENCY'),
        # I5: a commit is a landing over a registry zone by construction
        'landed_on_registry': bool(end and end['new'] == 'COMMIT_LANDING'),
        'landing_to_zone_m': near,
        'landing_truth_zone': zone_name or 'outside the truth',
        'landed_safe': zone_name in SAFE_TRUTH_ZONES,
        't_first_slz_s': first_slz - t_takeoff,
        'mean_dist_to_slz_m': float(np.mean(dists)) if dists else math.nan,
        'duration_s': t_landed - t_takeoff,
        'overruns': sum(1 for e in events if e['kind'] == 'overrun'),
        'first_violation_cause': cause,
        'lost_zone_back_within_5s': back,
        'pass_demotions': pass_demotions,
        'reserve_used_kj': max(0.0, config['reserve_fraction']
                               * (num(cycles[0]['remaining_j'])
                                  + num(cycles[0]['consumed_j']))
                               - num(last['remaining_j'])) / 1000,
        'failsafes': len(re.findall('Failsafe activated', px4)),
    }


def iqr(x):
    """Median [q1, q3] of the finite values, as text."""
    x = np.asarray([a for a in x if np.isfinite(a)], dtype=float)
    if not x.size:
        return 'n/a'
    q1, med, q3 = np.percentile(x, [25, 50, 75])
    return f'{med:.2f} [{q1:.2f}, {q3:.2f}]'


def mann_whitney(a, b):
    """U test two sided and the rank-biserial correlation of a over b."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    if np.all(a == a[0]) and np.all(b == a[0]):
        return math.nan, 0.0
    u, p = stats.mannwhitneyu(a, b, alternative='two-sided')
    return p, 2.0 * u / (len(a) * len(b)) - 1.0


rows = [metrics(r) for r in sorted((HERE / 'runs').iterdir())
        if r.is_dir() and not r.name.startswith('.')]
with open(HERE / 'missions.csv', 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)
failed = sorted(p.name for p in (HERE / 'failed').iterdir()) if (
    HERE / 'failed').exists() else []
by = {c: [r for r in rows if r['config'] == c] for c in ORDER}
print(f'{len(rows)} missions, {len(failed)} technical failures relaunched'
      f'{": " + ", ".join(failed) if failed else ""}')
print(f'git {sorted({r["git"] for r in rows})}')

cols = [('min_margin_kj', 'min I1 margin kJ'),
        ('reserve_used_kj', 'reserve used kJ'),
        ('pass_demotions', 'zone demotions on the pass count'),
        ('coverage_seen', 'coverage seen'),
        ('coverage_observed', 'coverage OBSERVED'),
        ('energy_left_kj', 'energy left kJ'),
        ('cells_per_kj', 'cells per kJ'),
        ('t_first_slz_s', 'first SLZ s'),
        ('mean_dist_to_slz_m', 'mean dist to SLZ m'),
        ('duration_s', 'duration s')]
print('\n== safety')
print('landed on registry: I5; landed safe: truth zone under the final '
      'position flat or 3 deg ramp')
print(f'{"config":<8} {"n":>3} {"missions with I1 violation":>27} '
      f'{"violations median [IQR]":>24} {"emergency":>9} '
      f'{"on registry":>11} {"landed safe":>11} {"overruns":>8}')
for c, rs in by.items():
    if not rs:
        continue
    print(f'{c:<8} {len(rs):3d} {sum(r["violations"] > 0 for r in rs):>27d} '
          f'{iqr([r["violations"] for r in rs]):>24} '
          f'{sum(r["emergency"] for r in rs):9d} '
          f'{sum(r["landed_on_registry"] for r in rs):11d} '
          f'{sum(r["landed_safe"] for r in rs):11d} '
          f'{sum(r["overruns"] for r in rs):8d}')
print('\n== C1 missions with an I1 violation: cause of the first one')
for c, rs in by.items():
    hit = [r for r in rs if r['violations'] > 0
           and r['policy'] in ('c1_gain', 'c1_c2')]
    if hit:
        print(f'{c:<8} ' + ', '.join(
            f'r{r["run"]} {r["first_violation_cause"]}'
            + (', zone back within 5 s' if r['lost_zone_back_within_5s']
               else '') for r in hit))
for key, label in cols:
    print(f'\n== {label}, median [IQR]')
    for c, rs in by.items():
        if rs:
            print(f'{c:<8} {iqr([r[key] for r in rs])}')

print('\n== gate E: C1 policies against the baselines, Mann-Whitney U')
print('effect r: rank-biserial, > 0 when the first is larger')
for key in ('violations', 'min_margin_kj', 'coverage_seen',
            'energy_left_kj'):
    for a in ('B3a050', 'B3a100'):
        for b in ('B0', 'B1'):
            if by[a] and by[b]:
                p, r = mann_whitney([x[key] for x in by[a]],
                                    [x[key] for x in by[b]])
                print(f'{key:<16} {a:>6} vs {b:<3} p {p:.4f}  r {r:+.2f}')

print('\n== gate F: alpha over the B3 missions, Spearman')
b3 = [r for r in rows if r['policy'] == 'c1_c2']
if len({r['alpha'] for r in b3}) > 1:
    for key, label in [('violations', 'violations')] + cols:
        x = [r['alpha'] for r in b3 if np.isfinite(r[key])]
        y = [r[key] for r in b3 if np.isfinite(r[key])]
        rho, p = stats.spearmanr(x, y)
        meds = ' '.join(f'{np.median([r[key] for r in b3 if r["alpha"] == a and np.isfinite(r[key])]):.2f}'
                        for a in sorted({r['alpha'] for r in b3}))
        print(f'{label:<20} rho {rho:+.2f}  p {p:.4f}  medians by alpha '
              f'{meds}')
