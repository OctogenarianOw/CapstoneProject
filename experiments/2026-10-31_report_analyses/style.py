"""
Palette and helpers shared by the figures of 31 Oct, identical to the
report figures of r_report_figures.py so that old and new figures read
as one set. Also the perception replay used by the map panels: the
flight code's own build_world run on a logged map snapshot.
"""
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

SURFACE, INK, INK2, MUTED, GRID = ('#fcfcfb', '#0b0b0b', '#52514e',
                                   '#898781', '#e1e0d9')
ACCENT, ACCENT_FACE = '#eb6834', '#fdf0ea'
BLUE, GREEN, RED = '#2a78d6', '#1baf7a', '#d03b3b'
BLUE_DARK = '#184f93'
LANDABLE, GROUND, OBSTACLE = '#9ec5f4', '#e7e6df', '#52514e'
CRITICAL = '#f2b8b5'
SEQ = ['#0b3d78', '#2a78d6', '#7fb2ec', '#eb6834', '#b34a1d']

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
WORLDS = ROOT / 'worlds'


def setup():
    plt.rcParams.update({'font.family': 'sans-serif', 'font.size': 8,
                         'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                         'xtick.color': MUTED, 'ytick.color': MUTED,
                         'xtick.labelcolor': INK2, 'ytick.labelcolor': INK2,
                         'figure.facecolor': SURFACE,
                         'axes.facecolor': SURFACE,
                         'savefig.facecolor': SURFACE,
                         'legend.frameon': False,
                         'axes.spines.top': False,
                         'axes.spines.right': False})


def scale_bar(ax, length_m, x0, y0, label=None):
    ax.plot([x0, x0 + length_m], [y0, y0], color=INK, lw=2.2,
            solid_capstyle='butt', zorder=9)
    ax.text(x0 + length_m / 2, y0, label or f'{length_m:g} m', ha='center',
            va='bottom', fontsize=7, color=INK, zorder=9,
            bbox={'facecolor': SURFACE, 'edgecolor': 'none', 'pad': 0.6,
                  'alpha': 0.85})


def snapshots(mission_dir):
    """(time, path) of every logged map snapshot, sorted."""
    snaps = sorted(Path(mission_dir, 'maps').glob('map_*.npz'),
                   key=lambda p: float(p.stem.split('_')[1]))
    return [(float(p.stem.split('_')[1]), p) for p in snaps]


def world_params(config):
    from uav_core.world_model import WorldParams
    import dataclasses
    names = {f.name for f in dataclasses.fields(WorldParams)}
    return WorldParams(**{k: v for k, v in config.items() if k in names})


def perceive(path, vehicle_ne, config):
    """Run the flight perception chain on one logged map.

    Returns the classes (0 unknown, 1 observed, 2 above the flight floor,
    3 landable), the landable mask, the world model and the image extent
    (east, north). Class 2 is the planner's height test alone (no
    inflation, unknown not blocked): what the map says is too high to
    overfly, not the inflated no-fly mask, whose rim along the unknown
    would read as an obstacle.
    """
    from uav_core.world_model import build_world
    from uav_core.observation import classify, OBSERVED
    from uav_core.planner import blocked_mask
    m = np.load(path)
    res = float(m['resolution'])
    origin = (float(m['origin_north']), float(m['origin_east']))
    params = world_params(config)
    w = build_world(m['elevation'], m['variance'], m['count'], res, origin,
                    vehicle_ne, params)
    seen = classify(m['count'], m['variance']) == OBSERVED
    high = blocked_mask(m['elevation'], w.state, params.altitude_m,
                        params.vertical_margin_m, False)
    cls = np.zeros(seen.shape, int)
    cls[seen] = 1
    cls[seen & high] = 2
    land = np.asarray(w.landing['candidate'], bool)
    cls[land] = 3
    n0, e0 = origin
    extent = (e0, e0 + cls.shape[1] * res, n0, n0 + cls.shape[0] * res)
    return cls, land, w, extent, res, origin


def zone_components(land, touchdowns_ne, res, origin):
    """Landable components (8-connected) holding a confirmed touchdown."""
    from scipy import ndimage
    lab, _ = ndimage.label(land, structure=np.ones((3, 3), int))
    keep = np.zeros(land.shape, bool)
    for n, e in touchdowns_ne:
        i = int((n - origin[0]) / res)
        j = int((e - origin[1]) / res)
        if 0 <= i < lab.shape[0] and 0 <= j < lab.shape[1] and lab[i, j]:
            keep |= lab == lab[i, j]
    return keep
