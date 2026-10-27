#!/usr/bin/env python3
"""
Spec F7: the landable cells the detector finds on the map, laid over
what is landable in the truth, false accepts benign and severe in
distinct colours, on the reference map v6 of W1.

Method of tools/validate_slz.py, unchanged: detect (slz_detection) run
on the map layers and on the truth layers, same 1.5 m footprint and
thresholds, no margin on the truth (it defines what is safe); a cell
is correct when both accept it, a false accept when only the map does,
a false reject when only the truth does; a false accept is severe when
the real terrain under it exceeds twice a limit (slope over 12 deg or
step over 0.48 m), benign otherwise (a gentle slope a little over the
limit, say). Zone labels play no part in the verdict, only in the
outlines.

Choices: one map, five classes, the two kinds of false accept in the
only warm colours so they stand out against the blues of agreement;
severe in the critical status colour, kept for it alone.

    python3 f7_zones_vs_truth.py        # ROS and the workspace sourced
"""

from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402
import numpy as np  # noqa: E402

from uav_core.slz_detection import detect  # noqa: E402
from uav_core.terrain_analysis import analyse  # noqa: E402
from uav_core.world_model import THRESHOLDS  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / 'data_survey_v6.npz'
TRUTH = ROOT / 'worlds' / 'validation_truth.npz'
OUT = Path(__file__).resolve().parent / 'f7_zones_vs_truth.png'
FOOTPRINT, FACTOR = 1.5, 2.0

SURFACE, INK, INK2, MUTED, GRID = ('#fcfcfb', '#0b0b0b', '#52514e',
                                   '#898781', '#e1e0d9')
CLASSES = [('not landable, refused by both', '#e7e6df'),
           ('landable, accepted by both', '#2a78d6'),
           ('landable, refused by the map (false reject)', '#b7d3f6'),
           ('benign false accept', '#f0a400'),
           ('severe false accept', '#d03b3b')]
LABEL = {'ramp_ok': 'ramp 3 deg', 'ramp_bad': 'ramp 10 deg',
         'rough': 'rough', 'obstacle': 'cube'}


def main():
    m = np.load(MAP)
    t = np.load(TRUTH)
    res = float(m['resolution'])
    names = [str(n) for n in t['zone_names']]
    truth = -t['truth'].T
    zone = t['zone'].T
    n = truth.shape[0]
    half = n * res / 2.0
    i0 = int(round((-half + res / 2.0 - float(m['origin_x'])) / res))
    j0 = int(round((-half + res / 2.0 - float(m['origin_y'])) / res))
    elev = m['elevation'][i0:i0 + n, j0:j0 + n]
    count = m['count'][i0:i0 + n, j0:j0 + n]
    origin = (-half, -half)
    map_lay = analyse(elev, count, res)
    truth_lay = analyse(truth, np.ones_like(count), res)
    mapped = detect(map_lay, elev, res, origin, footprint_m=FOOTPRINT,
                    safety_margin_m=0.0, **THRESHOLDS)['candidate']
    real = detect(truth_lay, truth, res, origin, footprint_m=FOOTPRINT,
                  safety_margin_m=0.0, **THRESHOLDS)['candidate']
    with np.errstate(invalid='ignore'):
        hazard = ((truth_lay['slope'] > FACTOR * THRESHOLDS['max_slope_deg'])
                  | (truth_lay['step'] > FACTOR * THRESHOLDS['max_step_m']))
    fa = mapped & ~real
    cls = np.zeros(zone.shape, int)
    cls[mapped & real] = 1
    cls[real & ~mapped] = 2
    cls[fa & ~hazard] = 3
    cls[fa & hazard] = 4
    recall = 100.0 * (mapped & real).sum() / real.sum()
    stats = (f'recall {recall:.1f} % of the truly landable cells; false '
             f'accepts {int(fa.sum())}: benign {int((fa & ~hazard).sum())}, '
             f'severe {int((fa & hazard).sum())}')
    print(stats)

    plt.rcParams.update({'font.family': 'sans-serif', 'font.size': 8,
                         'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                         'xtick.color': MUTED, 'ytick.color': MUTED,
                         'xtick.labelcolor': INK2, 'ytick.labelcolor': INK2,
                         'figure.facecolor': SURFACE,
                         'axes.facecolor': SURFACE,
                         'savefig.facecolor': SURFACE,
                         'legend.frameon': False})
    fig, ax = plt.subplots(figsize=(7.4, 5.2))
    c = t['centres']
    extent = (c[0] - res / 2, c[-1] + res / 2, c[0] - res / 2,
              c[-1] + res / 2)
    ax.imshow(cls, origin='lower', extent=extent, interpolation='nearest',
              cmap=ListedColormap([col for _, col in CLASSES]), vmin=0,
              vmax=len(CLASSES) - 1)
    flat = names.index('flat')
    for code, name in enumerate(names):
        if code == flat:
            continue
        ax.contour(c, c, (zone == code).astype(float), levels=[0.5],
                   colors=INK2, linewidths=0.7)
        ii, jj = np.nonzero(zone == code)
        ax.text(c[jj].mean(), c[ii].max() + 1.5, LABEL.get(name, name),
                ha='center', va='bottom', fontsize=7, color=INK)
    ax.set_aspect('equal')
    ax.set_xlabel('east (m)')
    ax.set_ylabel('north (m)')
    ax.tick_params(length=0, labelsize=7)
    ax.legend(handles=[Patch(facecolor=col, label=lab)
                       for lab, col in CLASSES],
              loc='upper left', bbox_to_anchor=(1.01, 1.0), fontsize=7,
              labelcolor=INK2)
    ax.text(1.02, 0.02, stats.replace('; ', ';\n'), transform=ax.transAxes,
            fontsize=7, color=INK2, va='bottom')
    fig.suptitle('W1 reference map v6: landing cells found against the truth '
                 f'(footprint {FOOTPRINT} m)', x=0.01, ha='left', color=INK,
                 fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(OUT, dpi=200)
    print(OUT)


if __name__ == '__main__':
    main()
