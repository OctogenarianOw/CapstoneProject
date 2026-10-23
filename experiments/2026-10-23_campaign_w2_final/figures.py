#!/usr/bin/env python3
"""
Figures of the W2 campaign, read from the mission logs only (S17).

  f12_trajectories.png    spec F12: the flights of every configuration
                          over the W2 truth, landings and first I1
                          violations marked
  observed_frequency.png  share of the missions of a configuration whose
                          last map holds each cell OBSERVED (S6)
  f11_metrics.png         spec F11 on W2: coverage, minimum I1 margin,
                          energy left and cells per kJ, one dot per
                          mission, the median as a bar
  f9_f10_chronogram.png   spec F9 and F10: energy left against the energy
                          a return and a landing need plus the reserve,
                          for one B1 and one B3 mission

Design choice: small multiples, one panel per configuration on the same
axes, rather than all configurations overlaid on one map. Seventy
trajectories on one map hide each other, and the spec F12 comparison,
B1 against B3, reads panel to panel. The metrics are dot strips with
the median, not bars of the median or box plots: with ten missions per
configuration every mission can be shown, and a bar would hide the
spread the IQR tables report. Colour follows the policy (B0, B1, B3),
never the rank, so a configuration keeps its colour in every figure.
Chronogram missions, fixed before the results: the B1 mission with the
most I1 violations (F10 shows the limit crossed), and the B3 mission at
alpha 0.75, the coverage optimum of gate F on W1, whose minimum margin
is the median of its configuration (a typical flight, not the best).
Run after summarise.py, whose missions.csv it reads.

    python3 figures.py [CAMPAIGN_DIR]
"""

import csv
import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import BoundaryNorm, ListedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402
import numpy as np  # noqa: E402

from uav_core.observation import classify, OBSERVED  # noqa: E402

HERE = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent
TRUTH = Path.home() / 'ros2_ws/droneAfter/worlds/w2_truth.npz'
OUT = HERE / 'figures'
ORDER = ['B0', 'B1', 'B3a000', 'B3a025', 'B3a050', 'B3a075', 'B3a100']
TITLE = {'B0': 'B0, nearest frontier',
         'B1': 'B1, gain, return at 30 %',
         'B3a000': 'B3, alpha 0', 'B3a025': 'B3, alpha 0.25',
         'B3a050': 'B3, alpha 0.5', 'B3a075': 'B3, alpha 0.75',
         'B3a100': 'B3, alpha 1 (B2 ranking)'}
TICK = {'B0': 'B0', 'B1': 'B1', 'B3a000': '0', 'B3a025': '0.25',
        'B3a050': '0.5', 'B3a075': '0.75', 'B3a100': '1'}
CHRONO_B3 = 'B3a075'
STOPPED = ('INIT', 'TAKEOFF', 'COMMIT_LANDING', 'LANDED', 'EMERGENCY')
HALF_VIEW_M = 64.0         # area half side 60 m, plus room for the edge

# Reference palette of the dataviz method, light mode (printed thesis).
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK2 = '#52514e'
MUTED = '#898781'
GRID = '#e1e0d9'
CRITICAL = '#d03b3b'
POLICY_COLOR = {'B0': '#1baf7a', 'B1': '#eb6834', 'B3': '#2a78d6'}
BLUE_RAMP = ['#cde2fb', '#b7d3f6', '#9ec5f4', '#86b6ef', '#6da7ec',
             '#5598e7', '#3987e5', '#2a78d6', '#1c5cab', '#0d366b']

plt.rcParams.update({
    'font.family': 'sans-serif', 'font.size': 8,
    'axes.edgecolor': GRID, 'axes.linewidth': 0.8,
    'axes.labelcolor': INK2, 'axes.titlesize': 8.5,
    'axes.titlecolor': INK, 'xtick.color': MUTED, 'ytick.color': MUTED,
    'xtick.labelcolor': INK2, 'ytick.labelcolor': INK2,
    'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE,
    'savefig.facecolor': SURFACE, 'legend.frameon': False,
})


def colour(config):
    """Series colour of a configuration, by its policy."""
    return POLICY_COLOR[config[:2]]


def load_truth():
    """Flat-pad mask in NED (north rows, east columns) and its extent."""
    t = np.load(TRUTH)
    names = list(t['zone_names'])
    flat = t['zone'].T == names.index('flat')
    c = t['centres']
    half = float(t['resolution']) / 2.0
    return flat, (c[0] - half, c[-1] + half, c[0] - half, c[-1] + half)


def load_mission(run):
    """Cycles and events of the one mission of a run directory."""
    mission = next((run / 'missions').iterdir())
    cycles = [json.loads(x) for x in (mission / 'cycles.jsonl').open()]
    events = [json.loads(x) for x in (mission / 'events.jsonl').open()]
    return mission, cycles, events


def first_violation_ne(cycles, events):
    """(north, east) of the vehicle at the first I1 violation, or None."""
    v = next((e for e in events if e['kind'] == 'invariant'
              and e['violated']), None)
    if v is None:
        return None
    c = min(cycles, key=lambda x: abs(x['t'] - v['t']))
    return c['vehicle_ned'][0], c['vehicle_ned'][1]


def draw_world(ax, flat, extent):
    """W2 truth: slope field in grey, flat pads in surface colour."""
    ax.imshow(np.where(flat, 0, 1), origin='lower', extent=extent,
              cmap=ListedColormap([SURFACE, GRID]), vmin=0, vmax=1,
              interpolation='nearest', zorder=0)
    e = np.linspace(extent[0], extent[1], flat.shape[1] + 1)
    n = np.linspace(extent[2], extent[3], flat.shape[0] + 1)
    ec, nc = (e[:-1] + e[1:]) / 2, (n[:-1] + n[1:]) / 2
    # Outlines above the observation layer, below the flights.
    ax.contour(ec, nc, flat.astype(float), levels=[0.5], colors=INK2,
               linewidths=0.6, zorder=1.8)
    a = 60.0
    ax.plot([-a, a, a, -a, -a], [-a, -a, a, a, -a], color=MUTED, lw=0.6,
            zorder=1.8)
    ax.set_xlim(-HALF_VIEW_M, HALF_VIEW_M)
    ax.set_ylim(-HALF_VIEW_M, HALF_VIEW_M)
    ax.set_aspect('equal')
    ax.set_xticks([-60, -30, 0, 30, 60])
    ax.set_yticks([-60, -30, 0, 30, 60])
    ax.tick_params(length=0, labelsize=7)


def fig_trajectories(runs, rows, flat, extent):
    """F12: every flight of every configuration over the truth."""
    fig, axes = plt.subplots(2, 4, figsize=(10.0, 5.6), sharex=True,
                             sharey=True)
    for ax, config in zip(axes.flat, ORDER):
        draw_world(ax, flat, extent)
        mine = [r for r in runs if r.name.rsplit('_r', 1)[0] == config]
        for run in mine:
            _, cycles, events = load_mission(run)
            ne = np.array([c['vehicle_ned'][:2] for c in cycles
                           if c['vehicle_ned'] is not None])
            ax.plot(ne[:, 1], ne[:, 0], color=colour(config), lw=0.9,
                    alpha=0.6, solid_joinstyle='round', zorder=2)
            row = rows.get(run.name)
            safe = row is not None and row['landed_safe'] == 'True'
            ax.scatter(ne[-1, 1], ne[-1, 0], s=22, zorder=4,
                       color=INK if safe else CRITICAL,
                       marker='o' if safe else 'v',
                       edgecolors=SURFACE, linewidths=1.0)
            fv = first_violation_ne(cycles, events)
            if fv is not None:
                ax.scatter(fv[1], fv[0], s=30, marker='X', color=CRITICAL,
                           edgecolors=SURFACE, linewidths=0.8, zorder=5)
        mine_rows = [rows[r.name] for r in mine if r.name in rows]
        hit = sum(int(r['violations']) > 0 for r in mine_rows)
        cov = np.median([float(r['coverage_observed']) for r in mine_rows]) \
            if mine_rows else math.nan
        ax.set_title(f'{TITLE[config]}\ncoverage {cov:.0%}, I1 violated '
                     f'in {hit}/{len(mine_rows)}', loc='left')
    legend_ax = axes.flat[-1]
    legend_ax.axis('off')
    handles = [
        Line2D([], [], color=POLICY_COLOR['B0'], lw=2, label='B0 flight'),
        Line2D([], [], color=POLICY_COLOR['B1'], lw=2, label='B1 flight'),
        Line2D([], [], color=POLICY_COLOR['B3'], lw=2,
               label='B3 flight (C1 + C2)'),
        Line2D([], [], ls='', marker='o', color=INK, markersize=6,
               label='landing on a flat pad'),
        Line2D([], [], ls='', marker='v', color=CRITICAL, markersize=6,
               label='landing off the pads'),
        Line2D([], [], ls='', marker='X', color=CRITICAL, markersize=7,
               label='first I1 violation'),
        Patch(facecolor=SURFACE, edgecolor=INK2, lw=0.6,
              label='flat pad (truth), 16 m'),
        Patch(facecolor=GRID, edgecolor=GRID, label='20 deg field (truth)'),
        Line2D([], [], color=MUTED, lw=0.6, label='explored area, 120 m'),
    ]
    legend_ax.legend(handles=handles, loc='center left', fontsize=7.5,
                     labelcolor=INK2, handlelength=1.6)
    for ax in axes[1]:
        ax.set_xlabel('east (m)')
    for ax in axes[:, 0]:
        ax.set_ylabel('north (m)')
    fig.suptitle('W2, 48 kJ: every flight of the campaign, take-off at '
                 'the centre pad', x=0.01, ha='left', color=INK, fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT / 'f12_trajectories.png', dpi=200)
    plt.close(fig)


def fig_observed(runs, flat, extent):
    """Share of the missions whose last map has each cell OBSERVED."""
    fig, axes = plt.subplots(2, 4, figsize=(10.0, 5.6), sharex=True,
                             sharey=True)
    cmap = ListedColormap(BLUE_RAMP)
    cmap.set_bad(alpha=0.0)
    norm = BoundaryNorm(np.linspace(0.0, 1.0, 11), cmap.N)
    image = None
    for ax, config in zip(axes.flat, ORDER):
        draw_world(ax, flat, extent)
        mine = [r for r in runs if r.name.rsplit('_r', 1)[0] == config]
        total, map_extent = None, None
        for run in mine:
            mission, _, _ = load_mission(run)
            snap = sorted((mission / 'maps').glob('map_*.npz'))[-1]
            m = np.load(snap)
            seen = classify(m['count'], m['variance']) == OBSERVED
            total = seen.astype(float) if total is None else total + seen
            on, oe = float(m['origin_north']), float(m['origin_east'])
            res = float(m['resolution'])
            map_extent = (oe, oe + seen.shape[1] * res,
                          on, on + seen.shape[0] * res)
        if total is not None:
            share = np.ma.masked_equal(total / len(mine), 0.0)
            image = ax.imshow(share, origin='lower', extent=map_extent,
                              cmap=cmap, norm=norm, interpolation='nearest',
                              zorder=1.5)
        ax.set_title(f'{TITLE[config]}, {len(mine)} missions', loc='left')
    axes.flat[-1].axis('off')
    if image is not None:
        bar = fig.colorbar(image, ax=axes.flat[-1], location='left',
                           fraction=0.5, shrink=0.8)
        bar.set_label('share of the missions with the cell OBSERVED',
                      color=INK2)
        bar.outline.set_visible(False)
        bar.ax.tick_params(length=0, labelsize=7)
    for ax in axes[1]:
        ax.set_xlabel('east (m)')
    for ax in axes[:, 0]:
        ax.set_ylabel('north (m)')
    fig.suptitle('W2, 48 kJ: where each configuration looks, last map of '
                 'each mission', x=0.01, ha='left', color=INK, fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT / 'observed_frequency.png', dpi=200)
    plt.close(fig)


def fig_metrics(rows):
    """F11 on W2: four metrics, one dot per mission, median as a bar."""
    metrics = [('coverage_observed', 'coverage OBSERVED (%)', 100.0),
               ('min_margin_kj', 'minimum I1 margin (kJ)', 1.0),
               ('energy_left_kj', 'energy left at landing (kJ)', 1.0),
               ('cells_per_kj', 'cells seen per kJ', 1.0)]
    configs = [c for c in ORDER if any(r['config'] == c for r in rows)]
    fig, axes = plt.subplots(2, 2, figsize=(8.0, 5.6))
    for ax, (key, label, scale) in zip(axes.flat, metrics):
        for x, config in enumerate(configs):
            y = np.sort([float(r[key]) * scale for r in rows
                         if r['config'] == config])
            y = y[np.isfinite(y)]
            if not y.size:
                continue
            dx = np.linspace(-0.2, 0.2, y.size) if y.size > 1 else [0.0]
            ax.scatter(x + np.asarray(dx), y, s=26, color=colour(config),
                       edgecolors=SURFACE, linewidths=1.0, zorder=3)
            med = float(np.median(y))
            ax.plot([x - 0.3, x + 0.3], [med, med], color=INK, lw=2,
                    solid_capstyle='round', zorder=4)
        if key == 'min_margin_kj':
            ax.axhline(0.0, color=CRITICAL, lw=0.8, zorder=2)
            ax.text(-0.55, 0.0, 'I1 limit', color=INK2, fontsize=7,
                    ha='left', va='bottom')
        ax.set_xticks(range(len(configs)))
        ax.set_xticklabels([TICK[c] for c in configs])
        ax.set_xlim(-0.6, len(configs) - 0.4)
        ax.grid(axis='y', color=GRID, lw=0.6)
        ax.set_axisbelow(True)
        ax.tick_params(length=0)
        for side in ('top', 'right', 'left'):
            ax.spines[side].set_visible(False)
        ax.set_title(label, loc='left')
        if 'B3a000' in configs:
            ax.set_xlabel('baselines, then B3 at alpha', color=INK2)
    handles = [Line2D([], [], ls='', marker='o', color=POLICY_COLOR[p],
                      markersize=6, label=lab)
               for p, lab in (('B0', 'B0, nearest frontier'),
                              ('B1', 'B1, gain, return at 30 %'),
                              ('B3', 'B3, C1 + C2'))]
    handles.append(Line2D([], [], color=INK, lw=2, label='median'))
    fig.legend(handles=handles, loc='upper right', ncol=4, fontsize=7.5,
               labelcolor=INK2)
    fig.suptitle('W2, 48 kJ: one dot per mission', x=0.01, ha='left',
                 color=INK, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(OUT / 'f11_metrics.png', dpi=200)
    plt.close(fig)


def pick_chronogram_runs(rows):
    """The B1 run with the most violations, the median-margin B3 run."""
    b1 = [r for r in rows if r['config'] == 'B1']
    b3 = [r for r in rows if r['config'] == CHRONO_B3
          and np.isfinite(float(r['min_margin_kj']))]
    picks = []
    if b1:
        worst = max(b1, key=lambda r: (int(r['violations']),
                                       -float(r['min_margin_kj'])))
        picks.append(f'B1_r{worst["run"]}')
    if b3:
        b3 = sorted(b3, key=lambda r: float(r['min_margin_kj']))
        picks.append(f'{CHRONO_B3}_r{b3[(len(b3) - 1) // 2]["run"]}')
    return picks


def fig_chronogram(campaign, rows):
    """F9 and F10: energy left against what a return needs."""
    picks = pick_chronogram_runs(rows)
    if not picks:
        return
    fig, axes = plt.subplots(len(picks), 1, figsize=(8.0, 2.6 * len(picks)),
                             sharey=True, squeeze=False)
    for ax, name in zip(axes[:, 0], picks):
        mission, cycles, events = load_mission(campaign / 'runs' / name)
        config = json.loads((mission / 'config.json').read_text())
        t0 = next(e['t'] for e in events if e['kind'] == 'state'
                  and e['new'] == 'TAKEOFF')
        t = np.array([c['t'] - t0 for c in cycles])
        left = np.array([c['remaining_j'] for c in cycles], float) / 1000
        # The machine stops evaluating I1 once the descent is committed
        # (S15.2) and the log repeats the last margin: keep the cycle of
        # the commit, which evaluated it (B1_r08: -71 J there), drop the
        # repeats after it.
        end = next((k for k, c in enumerate(cycles)
                    if c['state'] in STOPPED[2:]), len(cycles))
        margin = np.array([np.nan if c['i1_margin_j'] is None or k > end
                           else c['i1_margin_j']
                           for k, c in enumerate(cycles)], float) / 1000
        need = left - margin
        reserve = config['reserve_fraction'] * (
            cycles[0]['remaining_j'] + cycles[0]['consumed_j']) / 1000
        config = name.rsplit('_r', 1)[0]
        ax.plot(t, left, color=colour(config), lw=2, zorder=3)
        ax.plot(t, need, color=INK2, lw=1.4, zorder=3)
        ax.axhline(reserve, color=MUTED, lw=0.6, zorder=1)
        ax.text(1.0, reserve, 'reserve', color=INK2, fontsize=7,
                ha='left', va='bottom')
        # One band per violated cycle, 1 s wide (the cycle period), so a
        # single cycle shows.
        bad = np.flatnonzero(np.nan_to_num(margin, nan=1.0) < 0)
        for k in bad:
            ax.axvspan(t[k] - 0.5, t[k] + 0.5, color=CRITICAL, alpha=0.25,
                       lw=0, zorder=0)
        marks = [(e['t'] - t0, f'zone {e["zone_id"]} confirmed')
                 for e in events if e['kind'] == 'zone'
                 and e['new'] == 'CONFIRMED'
                 and not e['reason'].startswith('take-off')]
        marks += [(e['t'] - t0, e['new'].lower().replace('_', ' '))
                  for e in events if e['kind'] == 'state'
                  and e['new'] in ('RETURNING', 'COMMIT_LANDING', 'EMERGENCY')]
        for tm, text in marks:
            ax.axvline(tm, color=GRID, lw=0.8, zorder=1)
            ax.text(tm, 0.98, f' {text}', transform=ax.get_xaxis_transform(),
                    rotation=90, va='top', ha='left', fontsize=6.5,
                    color=INK2)
        row = next((r for r in rows if f'{r["config"]}_r{r["run"]}' == name),
                   None)
        n_v = int(row['violations']) if row else 0
        extra = (f', {n_v} cycle{"" if n_v == 1 else "s"} with I1 violated'
                 if row else '')
        ax.set_title(f'{TITLE[config]}, run {name.rsplit("_r", 1)[1]}{extra}',
                     loc='left')
        ax.set_ylim(bottom=0)
        ax.set_xlim(0, t[-1])
        ax.grid(axis='y', color=GRID, lw=0.6)
        ax.set_axisbelow(True)
        ax.tick_params(length=0)
        for side in ('top', 'right', 'left'):
            ax.spines[side].set_visible(False)
        ax.set_ylabel('kJ')
    axes[-1, 0].set_xlabel('time since take-off (s)')
    handles = [Line2D([], [], color=colour(n.rsplit('_r', 1)[0]), lw=2,
                      label=f'energy left, {n.rsplit("_r", 1)[0]}')
               for n in picks]
    handles += [Line2D([], [], color=INK2, lw=1.4,
                       label='needed to return and land, plus the reserve'),
                Patch(facecolor=CRITICAL, alpha=0.25, label='I1 violated')]
    fig.legend(handles=handles, loc='upper right', ncol=2, fontsize=7.5,
               labelcolor=INK2)
    fig.suptitle('W2, 48 kJ: the I1 guarantee over one flight', x=0.01,
                 ha='left', color=INK, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(OUT / 'f9_f10_chronogram.png', dpi=200)
    plt.close(fig)


def main():
    OUT.mkdir(exist_ok=True)
    with open(HERE / 'missions.csv') as f:
        table = list(csv.DictReader(f))
    rows = {f'{r["config"]}_r{r["run"]}': r for r in table}
    runs = sorted(r for r in (HERE / 'runs').iterdir()
                  if r.is_dir() and not r.name.startswith('.'))
    flat, extent = load_truth()
    fig_trajectories(runs, rows, flat, extent)
    fig_observed(runs, flat, extent)
    fig_metrics(table)
    fig_chronogram(HERE, table)
    for p in sorted(OUT.glob('*.png')):
        print(p)


if __name__ == '__main__':
    main()
