"""
B7. A mission-oriented metric: how fast and how cheaply does each
configuration find landing zones?

Role: coverage measures the means, not the mission. The mission is to
find safe landing zones, so this script measures, per mission, from the
registry logged at every cycle:
  - E_first: energy consumed when the first non-provisional zone becomes
    CONFIRMED with a usable touchdown area (landing_m2 > 0);
  - A_end: confirmed landing area (landing_m2 summed over the CONFIRMED,
    non-provisional zones) in the last cycle before touchdown, i.e. the
    registry the mission leaves behind;
  - A_end per kJ consumed, and the number of distinct zones ever
    confirmed.

Design choice: landing_m2 rather than area_m2. area_m2 counts every
landable cell of a connected zone; landing_m2 counts only the cells
whose whole footprint was seen on two passes, which is what C1 can use
as a touchdown point. Rejected: the peak area over the mission, which
rewards zones that were confirmed then demoted; the end area is what a
second mission or an operator would inherit. The peak is printed for
reference only.

Statistics as in the report: median [IQR], Mann-Whitney U two-sided,
Spearman over the five alphas.
"""
import numpy as np
from scipy import stats

from common import (ALPHAS, BUDGET_KJ, CAMPAIGNS, CONFIGS, fmt, load, runs)

FLYING = {'BOOTSTRAP', 'SELECTING', 'NAVIGATING', 'RETURNING',
          'COMMIT_LANDING', 'EMERGENCY', 'TAKEOFF'}


def confirmed_landing_m2(cycle):
    return sum(z['landing_m2'] or 0.0 for z in cycle['zones']
               if z['state'] == 'CONFIRMED' and not z['provisional'])


def mission_metrics(rec):
    cyc = [c for c in rec['cycles'] if c['state'] in FLYING]
    e_first = np.nan
    t_first = np.nan
    ever = set()
    peak = 0.0
    for c in cyc:
        a = confirmed_landing_m2(c)
        peak = max(peak, a)
        for z in c['zones']:
            if (z['state'] == 'CONFIRMED' and not z['provisional']
                    and (z['landing_m2'] or 0) > 0):
                ever.add(z['id'])
        if np.isnan(e_first) and a > 0:
            e_first = c['consumed_j'] / 1000.0
            t_first = c['t']
    last = cyc[-1]
    a_end = confirmed_landing_m2(last)
    consumed = last['consumed_j'] / 1000.0
    return {'e_first_kj': e_first, 't_first_s': t_first, 'a_end_m2': a_end,
            'a_peak_m2': peak, 'a_per_kj': a_end / consumed,
            'n_zones': len(ever), 'consumed_kj': consumed}


def main():
    allrows = []
    for world in CAMPAIGNS:
        per = {k: [] for k in CONFIGS}
        for cfg, r, d in runs(world):
            m = mission_metrics(load(d))
            m.update(world=world, config=cfg, run=r)
            per[cfg].append(m)
            allrows.append(m)
        print(f'\n== {world}, budget {BUDGET_KJ[world]:.0f} kJ: median [IQR]')
        print(f'{"config":8s} {"no zone":>7s}  {"E_first kJ":>20s}  '
              f'{"A_end m2":>22s}  {"A_end/kJ m2":>20s}  {"zones":>16s}  '
              f'{"A_peak m2":>22s}')
        for k in CONFIGS:
            ms = per[k]
            none = sum(np.isnan(m['e_first_kj']) for m in ms)
            print(f'{k:8s} {none:7d}  '
                  f'{fmt([m["e_first_kj"] for m in ms]):>20s}  '
                  f'{fmt([m["a_end_m2"] for m in ms], 0):>22s}  '
                  f'{fmt([m["a_per_kj"] for m in ms]):>20s}  '
                  f'{fmt([m["n_zones"] for m in ms], 1):>16s}  '
                  f'{fmt([m["a_peak_m2"] for m in ms], 0):>22s}')
        b3 = [k for k in CONFIGS if k in ALPHAS]
        for metric in ('e_first_kj', 'a_end_m2', 'a_per_kj', 'n_zones'):
            x = np.concatenate([[ALPHAS[k]] * len(per[k]) for k in b3])
            y = np.concatenate([[m[metric] for m in per[k]] for k in b3])
            ok = np.isfinite(y)
            rho, p = stats.spearmanr(x[ok], y[ok])
            line = f'  {metric:10s} Spearman alpha rho {rho:+.2f} p {p:.4f}'
            for a, b in (('B3a075', 'B3a000'), ('B3a075', 'B3a100'),
                         ('B3a050', 'B3a000'), ('B3a050', 'B3a100'),
                         ('B3a075', 'B1'), ('B3a050', 'B1')):
                ya = np.array([m[metric] for m in per[a]], float)
                yb = np.array([m[metric] for m in per[b]], float)
                ya, yb = ya[np.isfinite(ya)], yb[np.isfinite(yb)]
                _, pp = stats.mannwhitneyu(ya, yb, alternative='two-sided')
                u, _ = stats.mannwhitneyu(ya, yb, alternative='two-sided')
                r_ = 2 * u / (len(ya) * len(yb)) - 1
                line += f' | {a[3:] if a != "B1" else a}-{b[3:] if b.startswith("B3") else b} p {pp:.4f} r {r_:+.2f}'
            print(line)
    import csv
    with open('b7_mission_metric.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(allrows[0].keys()))
        w.writeheader()
        w.writerows(allrows)


if __name__ == '__main__':
    main()
