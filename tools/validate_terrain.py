"""Compare terrain layers computed on the map against ground truth.

The same analysis runs on both grids, so any difference comes from the
map, not from the method. Per zone, the table answers two questions:

  does the map recover the real slope?
  does it tell roughness apart from slope?

The second matters most. A rubble field and a smooth ramp both break a
landing, but for different reasons, and a detector that confuses them
would reject good ground and accept bad.

Zone masks are eroded by the analysis radius before any statistic is
taken. A cell at the foot of a ramp has ramp cells inside its window,
so its slope is genuinely high even though the cell itself sits on
flat ground. Counting it as flat would report terrain structure as
measurement noise.
"""

import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'uav_core'))
from uav_core.terrain_analysis import analyse

# Defaults: the reference map and the W1 truth of this repository. The
# default used to be /tmp/elevation_map.npz, which any cloud_world run
# overwrites, so a figure could silently be measured on the last flight.
ROOT = Path(__file__).resolve().parents[1]
MAP_PATH = (sys.argv[1] if len(sys.argv) > 1
            else str(ROOT / 'data_survey_v6.npz'))
TRUTH_PATH = (sys.argv[2] if len(sys.argv) > 2
              else str(ROOT / 'worlds' / 'validation_truth.npz'))

def zone_names(truth):
    """Zone names as the truth file stores them, code = index.

    Hard-coded names used to label code 1 'ramp_10' and code 2
    'ramp_25' after the truth was redrawn with a 3 deg and a 10 deg
    ramp, so every row of the table was mislabelled.
    """
    if 'zone_names' in truth.files:
        return [str(n) for n in truth['zone_names']]
    print('warning: truth without zone_names, legacy labels used')
    return ['flat', 'ramp_10', 'ramp_25', 'rough', 'obstacle']

RADIUS = 1

# ------------------------------------------------------------------ load

m = np.load(MAP_PATH)
elev = m['elevation']
count = m['count']
res = float(m['resolution'])
ox, oy = float(m['origin_x']), float(m['origin_y'])

t = np.load(TRUTH_PATH)
ZONE_NAMES = zone_names(t)
truth_ned = -t['truth'].T          # ENU up to NED down, axes swapped
zone_ned = t['zone'].T

n_t = truth_ned.shape[0]
half_t = n_t * res / 2.0

i0 = int(round((-half_t + res / 2.0 - ox) / res))
j0 = int(round((-half_t + res / 2.0 - oy) / res))

sub_elev = elev[i0:i0 + n_t, j0:j0 + n_t]
sub_count = count[i0:i0 + n_t, j0:j0 + n_t]

# --------------------------------------------------------------- analyse

map_layers = analyse(sub_elev, sub_count, res, radius=RADIUS)
truth_layers = analyse(truth_ned, np.ones_like(sub_count), res, radius=RADIUS)


def interior(mask, radius):
    """Cells whose full window stays inside the mask.

    Shifting the mask by every window offset and taking the logical AND
    drops exactly the cells that touch a different zone.
    """
    out = mask.copy()
    h, w = mask.shape
    for di in range(-radius, radius + 1):
        for dj in range(-radius, radius + 1):
            if di == 0 and dj == 0:
                continue
            shifted = np.zeros_like(mask)
            src_i = slice(max(0, -di), h - max(0, di))
            src_j = slice(max(0, -dj), w - max(0, dj))
            dst_i = slice(max(0, di), h - max(0, -di))
            dst_j = slice(max(0, dj), w - max(0, -dj))
            shifted[dst_i, dst_j] = mask[src_i, src_j]
            out &= shifted
    return out

# ---------------------------------------------------------------- report

print(f'map    {MAP_PATH}')
print(f'truth  {TRUTH_PATH}')
print(f'zone masks eroded by {RADIUS} cell so windows stay inside a zone')
print()
print(f'{"zone":<10} {"cells":>6} '
      f'{"slope true":>11} {"slope map":>10} '
      f'{"rough true":>11} {"rough map":>10} '
      f'{"step map":>9}')

for code, name in enumerate(ZONE_NAMES):
    core = interior(zone_ned == code, RADIUS)
    sel = core & map_layers['valid'] & truth_layers['valid']
    if sel.sum() == 0:
        print(f'{name:<10} {0:>6}   no valid interior cell')
        continue

    print(f'{name:<10} {sel.sum():>6} '
          f'{np.median(truth_layers["slope"][sel]):>10.1f}° '
          f'{np.median(map_layers["slope"][sel]):>9.1f}° '
          f'{np.median(truth_layers["roughness"][sel]):>10.3f}m '
          f'{np.median(map_layers["roughness"][sel]):>9.3f}m '
          f'{np.median(map_layers["step"][sel]):>8.3f}m')

# The noise floor sets every threshold downstream: a slope limit below
# what flat ground already reads cannot discriminate anything. Measured
# on interior flat cells only, where the true slope is exactly zero.
flat = (interior(zone_ned == ZONE_NAMES.index('flat'), RADIUS)
        & map_layers['valid'])
print()
print(f'noise floor on flat interior ({flat.sum()} cells):')
for q in (50, 90, 95, 99):
    print(f'  p{q:<3} slope {np.percentile(map_layers["slope"][flat], q):>6.2f}° '
          f'| roughness {np.percentile(map_layers["roughness"][flat], q):>6.3f} m '
          f'| step {np.percentile(map_layers["step"][flat], q):>6.3f} m')

# Separation is what a classifier actually needs: how far apart the
# flat distribution and the 10 deg ramp distribution are.
# The ramp over the 6 deg threshold, found by name: by code it was
# the 3 deg ramp of the current truth, which reported a separation
# that does not exist.
STEEP = next((n for n in ('ramp_bad', 'ramp_10') if n in ZONE_NAMES), None)
ramp = (interior(zone_ned == ZONE_NAMES.index(STEEP), RADIUS)
        & map_layers['valid']) if STEEP else np.zeros_like(flat)
if ramp.sum():
    print()
    print(f'separation flat vs {STEEP} (the ramp over the threshold):')
    for q in (95, 99):
        print(f'  flat     p{q} {np.percentile(map_layers["slope"][flat], q):.2f}°')
    print(f'  {STEEP:<8} p01 {np.percentile(map_layers["slope"][ramp], 1):.2f}°')

# ------------------------------------------------------------------ plot

fig, ax = plt.subplots(2, 3, figsize=(17, 10))
extent = [-half_t, half_t, -half_t, half_t]

rows = [('truth', truth_layers), ('map', map_layers)]
cols = [('slope', 'slope (deg)', 0, 30, 'viridis'),
        ('roughness', 'roughness (m)', 0, 0.3, 'magma'),
        ('step', 'step (m)', 0, 0.6, 'cividis')]

for r, (row_name, layers) in enumerate(rows):
    for c, (key, label, lo, hi, cmap) in enumerate(cols):
        im = ax[r, c].imshow(layers[key], origin='lower', extent=extent,
                             cmap=cmap, vmin=lo, vmax=hi)
        ax[r, c].set_title(f'{row_name}: {label}')
        ax[r, c].set_xlabel('y east (m)')
        ax[r, c].set_ylabel('x north (m)')
        fig.colorbar(im, ax=ax[r, c])

plt.tight_layout()
plt.savefig('/tmp/terrain.png', dpi=100)
print('\nwrote /tmp/terrain.png')