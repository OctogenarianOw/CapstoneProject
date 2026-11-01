"""
r7_analyses.png: four analyses of the final campaigns, from the logs.

  (a) B1: energy C1 charged against energy measured, per frontier transit
      (E_go with the latency charge) and per final return leg (E_ret
      with the latency charge), as the relative error of the charge;
  (b) B4: share of the evaluated frontiers that C1 rejects at each
      decision of the C1 missions, against the share of the budget
      already spent;
  (c) B6: computation time of one decision cycle against the number of
      observed cells, three worlds;
  (d) B7: confirmed landing area at the end of the mission against
      alpha, with B1 for reference, per world.

Design choice: one figure with four panels rather than four figures,
because each answers one question with one plot and the report has a
word and page budget. Every number is read from the npz/csv files the
analysis scripts write (b1_energy_prediction.py, b3_b4_b6_decisions.py,
b7_mission_metric.py), so figure and text cannot disagree.

    python3 fig_analyses.py   (after the three scripts above)
"""
import csv

import matplotlib.pyplot as plt
import numpy as np

from common import ALPHAS
from style import (ACCENT, BLUE, BLUE_DARK, GREEN, GRID, HERE, INK, INK2,
                   MUTED, RED, SEQ, setup)

WCOL = {'W1': SEQ[0], 'W2': SEQ[1], 'W3': SEQ[3]}


def main():
    setup()
    fig, axes = plt.subplots(2, 2, figsize=(10.2, 7.0))
    fig.subplots_adjust(hspace=0.42, wspace=0.26, left=0.07, right=0.98,
                        top=0.95, bottom=0.08)

    # (a) energy charged by C1 against measured
    b1 = np.load(HERE / 'b1_energy_prediction.npz')
    ax = axes[0, 0]
    go_err = (b1['go_meas'] - b1['go_pred']) / b1['go_pred'] * 100
    ret_c = b1['ret_pred'] + b1['ret_tau']
    ret_err = (b1['ret_meas'] - ret_c) / ret_c * 100
    bins = np.linspace(-60, 20, 41)
    ax.hist(np.clip(go_err, -60, 20), bins=bins, color=BLUE, alpha=0.85,
            label=f'transit to a frontier, E_go (n = {len(go_err)})',
            density=True)
    ax.hist(np.clip(ret_err, -60, 20), bins=bins, color=ACCENT, alpha=0.6,
            label=f'final return leg, E_ret (n = {len(ret_err)})',
            density=True)
    ax.axvline(0, color=INK, lw=1.0)
    top = ax.get_ylim()[1]
    ax.text(1.5, top * 0.62, 'under-charged:\nmeasured above\nthe charge',
            fontsize=7, color=INK, va='top')
    ax.text(-38, top * 0.97, 'conservative: charge above the measurement',
            fontsize=7, color=INK, va='top', ha='center')
    clip = ret_err >= 20
    n_clip = int(clip.sum())
    if n_clip:
        lmax = float(np.max(b1['ret_len'][clip]))
        ax.annotate(f'{n_clip} return legs of {lmax:.0f} m or less\n'
                    f'(+{ret_err.max():.0f} % at most), clipped',
                    xy=(19.5, 0.012),
                    xytext=(4, top * 0.3), fontsize=6.8, color=INK,
                    arrowprops={'arrowstyle': '-|>', 'color': INK2,
                                'lw': 0.8})
    ax.set_xlabel('measured − charged by C1, % of the charge')
    ax.set_ylabel('density')
    ax.legend(loc='center left', fontsize=7, bbox_to_anchor=(0.0, 0.62))
    ax.set_title(f'(a) C1 charge against measurement: under in '
                 f'{int(np.sum(go_err > 0))}/{len(go_err)} transits, '
                 f'{int(np.sum(ret_err > 0))}/{len(ret_err)} returns',
                 loc='left', fontsize=8.5, color=INK)
    print('return under-charges (pct, length m):',
          [(round(float(e), 1), round(float(l), 1)) for e, l in
           zip(ret_err[ret_err > 0], b1['ret_len'][ret_err > 0])])

    # (b) rejection by C1 against budget spent
    b4 = np.load(HERE / 'b4_rejection.npz')
    ax = axes[0, 1]
    sel = np.isin(b4['config'], list(ALPHAS))
    spent, rej = b4['spent'][sel] * 100, b4['rej'][sel] * 100
    world = b4['world'][sel]
    edges = np.arange(0, 91, 10)
    for w in ('W1', 'W2', 'W3'):
        m = world == w
        ax.scatter(spent[m], rej[m], s=4, color=WCOL[w], alpha=0.25, lw=0)
    centres, med, q1, q3 = [], [], [], []
    for lo in edges[:-1]:
        m = (spent >= lo) & (spent < lo + 10)
        if m.sum() >= 10:
            centres.append(lo + 5)
            med.append(np.median(rej[m]))
            q1.append(np.percentile(rej[m], 25))
            q3.append(np.percentile(rej[m], 75))
    ax.fill_between(centres, q1, q3, color=INK, alpha=0.12, lw=0)
    ax.plot(centres, med, color=INK, lw=1.6, marker='o', ms=3,
            label='median and IQR, all worlds')
    for w in ('W1', 'W2', 'W3'):
        ax.scatter([], [], s=12, color=WCOL[w], label=w)
    ax.set_xlabel('budget already spent at the decision (%)')
    ax.set_ylabel('frontiers rejected by C1 (%)')
    ax.set_xlim(0, 90)
    ax.set_ylim(-3, 103)
    ax.legend(loc='upper left', fontsize=7)
    ax.set_title(f'(b) frontiers rejected by C1, {len(rej)} decisions of '
                 f'the C1 missions', loc='left', fontsize=8.5, color=INK)

    # (c) computation time against map size
    b6 = np.load(HERE / 'b6_timing.npz')
    ax = axes[1, 0]
    for w in ('W1', 'W2', 'W3'):
        m = b6['world'] == w
        idx = np.flatnonzero(m)
        if idx.size > 3000:
            idx = np.random.default_rng(0).choice(idx, 3000, replace=False)
        ax.scatter(b6['seen'][idx], b6['total'][idx], s=3, color=WCOL[w],
                   alpha=0.3, lw=0, label=f'{w} ({m.sum()} cycles)')
    ax.axhline(1000, color=RED, lw=1.0)
    ax.text(200, 1010, 'one map cycle (1 Hz)', color=RED, fontsize=7,
            va='bottom')
    ax.set_xlabel('observed cells in the map')
    ax.set_ylabel('computation per cycle (ms)')
    ax.set_ylim(0, 1100)
    ax.legend(loc='center left', fontsize=7, markerscale=3,
              bbox_to_anchor=(0.0, 0.62))
    ax.set_title(f'(c) decision cycle: max {b6["total"].max():.0f} ms on the '
                 f'240 m world', loc='left', fontsize=8.5, color=INK)

    # (d) confirmed landing area against alpha
    ax = axes[1, 1]
    rows = list(csv.DictReader(open(HERE / 'b7_mission_metric.csv')))
    for w in ('W1', 'W2', 'W3'):
        xs, meds, lo, hi = [], [], [], []
        for cfg, a in ALPHAS.items():
            v = [float(r['a_end_m2']) for r in rows
                 if r['world'] == w and r['config'] == cfg]
            ax.scatter([a + {'W1': -0.03, 'W2': 0, 'W3': 0.03}[w]] * len(v),
                       v, s=7, color=WCOL[w], alpha=0.45, lw=0)
            xs.append(a)
            meds.append(np.median(v))
        ax.plot(xs, meds, color=WCOL[w], lw=1.6, marker='o', ms=4,
                label=f'{w}, B3')
        b1v = [float(r['a_end_m2']) for r in rows
               if r['world'] == w and r['config'] == 'B1']
        ax.scatter([1.13], [np.median(b1v)], marker='D', s=26,
                   color=WCOL[w], edgecolor=INK, lw=0.5)
    ax.scatter([], [], marker='D', s=26, color=MUTED, edgecolor=INK,
               lw=0.5, label='B1 (median)')
    ax.set_xticks([0, 0.25, 0.5, 0.75, 1.0, 1.13])
    ax.set_xticklabels(['0', '0.25', '0.5', '0.75', '1', 'B1'])
    ax.set_xlabel('α (weight of information gain in C2)')
    ax.set_ylabel('confirmed landing area at the end (m²)')
    ax.legend(loc='upper left', fontsize=7)
    ax.set_title('(d) landing area found rises with α, unlike coverage',
                 loc='left', fontsize=8.5, color=INK)
    out = HERE / 'r7_analyses.png'
    fig.savefig(out, dpi=200)
    print(out)


if __name__ == '__main__':
    main()
