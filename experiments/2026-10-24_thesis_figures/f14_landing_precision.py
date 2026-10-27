#!/usr/bin/env python3
"""
Spec F14: the landing precision, with the footprint radius retained.

Data: experiments/2026-10-19_landing_p4/summary.txt, the 15 landings
flown on 19 Oct (executor and monitor only, headless), radial error of
the touchdown to the setpoint and drift during the descent, as
summarise.py wrote them; the airframe radius 0.386 m measured in the
PX4 x500_base model (blade tip), same README.

Left: the 15 radial errors, one dot each, stacked in 0.02 m bins (a
histogram with 15 values says less than the values), the mean and the
p95, against the error the footprint leaves room for: half the
footprint minus the airframe radius, 0.364 m at the 1.5 m in use, and
0.214 m at the 1.2 m P4 computed (2 (0.386 + 0.180), rounded up),
which broke the S8 barrier on mission maps on 20 Oct
(experiments/2026-10-20_footprint_p4) and was not adopted. Right: the
same seen from above, the 1.5 m footprint, the airframe drawn at the
largest error measured, so the reader sees the clearance left.

    python3 f14_landing_precision.py
"""

from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
SUMMARY = HERE.parent / '2026-10-19_landing_p4' / 'summary.txt'
OUT = HERE / 'f14_landing_precision.png'
AIRFRAME_R = 0.386
FOOTPRINT, FOOTPRINT_P4 = 1.5, 1.2
BIN = 0.02

SURFACE, INK, INK2, MUTED, GRID = ('#fcfcfb', '#0b0b0b', '#52514e',
                                   '#898781', '#e1e0d9')
BLUE, ORANGE, AQUA = '#2a78d6', '#eb6834', '#1baf7a'


def landings():
    lines = SUMMARY.read_text().splitlines()
    k = next(i for i, ln in enumerate(lines) if ln.startswith('per landing'))
    rows = []
    for ln in lines[k + 1:]:
        parts = ln.split()
        if len(parts) != 4:
            break
        rows.append([float(p) for p in parts])
    return np.array(rows)


def main():
    rows = landings()
    err = rows[:, 0]
    mean, p95 = float(err.mean()), float(np.percentile(err, 95))
    room = FOOTPRINT / 2 - AIRFRAME_R
    room_p4 = FOOTPRINT_P4 / 2 - AIRFRAME_R
    print(f'{len(err)} landings, radial error mean {mean:.3f} p95 {p95:.3f} '
          f'max {err.max():.3f} m; room {room:.3f} m at {FOOTPRINT} m, '
          f'{room_p4:.3f} m at {FOOTPRINT_P4} m')

    plt.rcParams.update({'font.family': 'sans-serif', 'font.size': 8,
                         'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                         'xtick.color': MUTED, 'ytick.color': MUTED,
                         'xtick.labelcolor': INK2, 'ytick.labelcolor': INK2,
                         'figure.facecolor': SURFACE,
                         'axes.facecolor': SURFACE,
                         'savefig.facecolor': SURFACE,
                         'legend.frameon': False})
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(9.6, 4.0),
                                 gridspec_kw={'width_ratios': [1.55, 1.0]})

    # Left: dots stacked per bin.
    bins = np.floor(err / BIN).astype(int)
    for b in np.unique(bins):
        sel = np.sort(err[bins == b])
        for h, v in enumerate(sel):
            ax.scatter((b + 0.5) * BIN, h + 1, s=70, color=BLUE,
                       edgecolors=SURFACE, linewidths=1.0, zorder=3)
    top = int(np.bincount(bins).max()) + 1.6
    ax.axvspan(0, room, color='#eef4fc', zorder=0)
    for x, colour, label, style in (
            (mean, INK2, f'mean {mean:.3f} m', ':'),
            (p95, INK, f'p95 {p95:.3f} m', '-'),
            (room_p4, ORANGE, f'room at {FOOTPRINT_P4} m: {room_p4:.3f} m, '
             'not adopted', '--'),
            (room, AQUA, f'room at {FOOTPRINT} m, in use: {room:.3f} m', '-')):
        ax.axvline(x, color=colour, lw=1.3, ls=style, zorder=2)
        ax.text(x + 0.004, top - 0.15, label, rotation=90, va='top',
                ha='left', fontsize=7, color=colour if colour != INK2
                else INK2)
    ax.set_xlim(0, 0.42)
    ax.set_ylim(0, top)
    ax.set_yticks(range(0, int(top) + 1))
    ax.set_xlabel('radial error of the touchdown to the setpoint (m)')
    ax.set_ylabel(f'landings per {BIN} m bin')
    ax.tick_params(length=0)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    ax.set_title(f'{len(err)} landings, 19 Oct; shaded: error the 1.5 m '
                 'footprint absorbs', loc='left', color=INK, fontsize=8.5)
    # The drift as summarise.py computed it, from the unrounded values.
    line = next(ln for ln in SUMMARY.read_text().splitlines()
                if ln.startswith('drift during the descent'))
    words = line.split()
    d_mean, d_max = words[words.index('mean') + 1], words[-2]
    ax.text(0.236, top - 0.25, f'drift during the descent:\nmean {d_mean} m,'
            f' max {d_max} m;\nmost of the error is\nalready there when\n'
            'LAND is sent', ha='left', va='top', fontsize=7, color=INK2)

    # Right: from above.
    half = FOOTPRINT / 2
    for c in range(-1, 2):
        for d in range(-1, 2):
            bx.add_patch(plt.Rectangle((c - 0.5, d - 0.5), 1, 1,
                                       facecolor='none', edgecolor=GRID,
                                       lw=0.8))
    bx.add_patch(Circle((0, 0), half, facecolor='#eef4fc', edgecolor=AQUA,
                        lw=1.6, zorder=1))
    bx.add_patch(Circle((0, 0), FOOTPRINT_P4 / 2, facecolor='none',
                        edgecolor=ORANGE, lw=1.2, ls='--', zorder=2))
    bx.add_patch(Circle((0, 0), p95, facecolor='none', edgecolor=INK,
                        lw=1.0, zorder=3))
    worst = float(err.max())
    ang = np.radians(35)
    cx, cy = worst * np.cos(ang), worst * np.sin(ang)
    bx.add_patch(Circle((cx, cy), AIRFRAME_R, facecolor='none',
                        edgecolor=BLUE, lw=1.6, zorder=4))
    bx.scatter([0], [0], marker='+', s=60, color=INK, zorder=5)
    bx.scatter([cx], [cy], s=14, color=BLUE, zorder=5)
    bx.text(0, -half - 0.08, f'footprint {FOOTPRINT} m, in use', ha='center',
            va='top', fontsize=7, color=AQUA)
    bx.text(0, -FOOTPRINT_P4 / 2 + 0.05, f'{FOOTPRINT_P4} m', ha='center',
            va='bottom', fontsize=6.5, color=ORANGE)
    bx.text(0, half + 0.1, f'airframe, {AIRFRAME_R} m to the blade tip,\n'
            f'at the largest error, {worst:.3f} m', fontsize=6.8,
            color=BLUE, va='bottom', ha='center')
    bx.text(-1.45, -1.45, '1 m cells', fontsize=6.5, color=MUTED,
            va='bottom')
    bx.set_xlim(-1.5, 1.5)
    bx.set_ylim(-1.5, 1.5)
    bx.set_aspect('equal')
    bx.set_axis_off()
    bx.set_title('seen from above: the setpoint +, the p95 circle',
                 loc='left', color=INK, fontsize=8.5)
    fig.suptitle('Landing precision against the footprint (spec P4, S8)',
                 x=0.01, ha='left', color=INK, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(OUT, dpi=200)
    print(OUT)


if __name__ == '__main__':
    main()
