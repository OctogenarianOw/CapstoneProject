"""Compare a produced elevation map against the analytic ground truth.

The truth grid comes from make_world.py, so every cell has an exact
expected height. Two things get measured:

  accuracy  - how close the mapped height is to the true height
  slope     - whether the map reproduces the real terrain gradient,
              which is what landing-site detection actually consumes

Frames: Gazebo is ENU (x east, y north, z up), the map is NED
(x north, y east, z down). The truth grid is indexed [x_gz, y_gz] and
has to be transposed and negated to be comparable.
"""

import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

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


# ------------------------------------------------------------------ load

m = np.load(MAP_PATH)
elev = m['elevation']
count = m['count']
res = float(m['resolution'])
ox, oy = float(m['origin_x']), float(m['origin_y'])

t = np.load(TRUTH_PATH)
truth_gz = t['truth']          # indexed [x_east, y_north], z up
zone_gz = t['zone']
centres = t['centres']
t_res = float(t['resolution'])
ZONE_NAMES = zone_names(t)

if abs(t_res - res) > 1e-6:
    raise SystemExit(f'resolution mismatch: map {res}, truth {t_res}')

# Gazebo ENU to NED: north = gazebo y, east = gazebo x, down = -up.
# Transposing swaps the two horizontal axes.
truth_ned = -truth_gz.T
zone_ned = zone_gz.T

# ------------------------------------------------------- align the grids

n_t = truth_ned.shape[0]
half_t = n_t * res / 2.0

# Map index of the truth grid's first cell centre.
i0 = int(round((-half_t + res / 2.0 - ox) / res))
j0 = int(round((-half_t + res / 2.0 - oy) / res))

if i0 < 0 or j0 < 0 or i0 + n_t > elev.shape[0] or j0 + n_t > elev.shape[1]:
    raise SystemExit('truth grid does not fit inside the map')

sub_elev = elev[i0:i0 + n_t, j0:j0 + n_t]
sub_count = count[i0:i0 + n_t, j0:j0 + n_t]

known = sub_count > 0

# The EKF origin sits on base_link at spawn, so the whole map carries a
# constant offset against the world ground plane. Remove it using the
# flat reference zone, then every residual is real error.
ref = known & (zone_ned == ZONE_NAMES.index('flat'))
if ref.sum() < 50:
    raise SystemExit('not enough flat cells observed to calibrate the offset')
offset = np.median(sub_elev[ref] - truth_ned[ref])

err = np.where(known, sub_elev - truth_ned - offset, np.nan)

# --------------------------------------------------------- slope per zone

def mean_slope(height_ned, mask):
    """Mean terrain slope in degrees over the masked cells."""
    gx, gy = np.gradient(height_ned, res)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    sel = mask & np.isfinite(slope)
    return np.nanmedian(slope[sel]) if sel.sum() else np.nan


filled = np.where(known, sub_elev - offset, np.nan)

# -------------------------------------------------------------- report

print(f'map      {MAP_PATH}')
print(f'truth    {TRUTH_PATH}')
print(f'offset   {offset:+.3f} m removed, from {ref.sum()} flat cells')
print()
print(f'{"zone":<10} {"cells":>6} {"cover":>7} {"bias":>8} {"RMS":>8} '
      f'{"slope true":>11} {"slope map":>10}')

for code, name in enumerate(ZONE_NAMES):
    in_zone = zone_ned == code
    seen = in_zone & known
    if seen.sum() == 0:
        print(f'{name:<10} {0:>6} {"0%":>7}')
        continue

    e = err[seen]
    cover = 100.0 * seen.sum() / in_zone.sum()
    s_map = mean_slope(filled, seen)
    # The true slope by the same estimator on the truth, not a number
    # typed in (they were those of an older truth, 10 and 25 deg).
    s_true_txt = f'{mean_slope(truth_ned, seen):>10.1f}°'

    print(f'{name:<10} {seen.sum():>6} {cover:>6.0f}% '
          f'{np.nanmean(e):>+8.3f} {np.sqrt(np.nanmean(e ** 2)):>8.3f} '
          f'{s_true_txt} {s_map:>9.1f}°')

# ----------------------------------------------------------------- plot

fig, ax = plt.subplots(1, 3, figsize=(18, 5.5))
extent = [-half_t, half_t, -half_t, half_t]

im = ax[0].imshow(truth_ned, origin='lower', extent=extent, cmap='terrain')
ax[0].set_title('Ground truth (m, NED)')
fig.colorbar(im, ax=ax[0])

im = ax[1].imshow(filled, origin='lower', extent=extent, cmap='terrain')
ax[1].set_title('Mapped elevation (m, NED)')
fig.colorbar(im, ax=ax[1])

im = ax[2].imshow(err, origin='lower', extent=extent, cmap='coolwarm',
                  vmin=-0.5, vmax=0.5)
ax[2].set_title('Error (m)')
fig.colorbar(im, ax=ax[2])

for a in ax:
    a.set_xlabel('y east (m)')
    a.set_ylabel('x north (m)')

plt.tight_layout()
plt.savefig('/tmp/validation.png', dpi=110)
print('\nwrote /tmp/validation.png')