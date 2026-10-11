"""Measure the safety of SLZ detection against ground truth.

Two error types, and they are not symmetric:

  false accept  a zone declared landable where the real terrain is not
                -> a destroyed aircraft. This must be zero.
  false reject  safe ground declared unlandable
                -> lost opportunity. Tolerable, and the price of
                   refusing to decide on poorly observed terrain.

Truth runs through the identical pipeline, so any difference comes
from the map rather than from the thresholds.
"""

import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, '/home/doria/ros2_ws/droneAfter/src/uav_core')
from uav_core.terrain_analysis import analyse
from uav_core.slz_detection import detect

MAP_PATH = sys.argv[1] if len(sys.argv) > 1 else '/tmp/elevation_map.npz'
TRUTH_PATH = (sys.argv[2] if len(sys.argv) > 2
              else '/home/doria/ros2_ws/droneAfter/worlds/validation_truth.npz')

ZONE_NAMES = ['flat', 'ramp_10', 'ramp_25', 'rough', 'obstacle']

THRESHOLDS = dict(max_slope_deg=6.0, max_roughness_m=0.05, max_step_m=0.24)
FOOTPRINT_M = 3.0
SAFETY_MARGIN_M = 1.0

# ------------------------------------------------------------------ load

m = np.load(MAP_PATH)
elev = m['elevation']
count = m['count']
res = float(m['resolution'])
ox, oy = float(m['origin_x']), float(m['origin_y'])

t = np.load(TRUTH_PATH)
truth_ned = -t['truth'].T
zone_ned = t['zone'].T

n_t = truth_ned.shape[0]
half_t = n_t * res / 2.0
i0 = int(round((-half_t + res / 2.0 - ox) / res))
j0 = int(round((-half_t + res / 2.0 - oy) / res))

sub_elev = elev[i0:i0 + n_t, j0:j0 + n_t]
sub_count = count[i0:i0 + n_t, j0:j0 + n_t]

sub_origin = (-half_t, -half_t)

# ---------------------------------------------------------------- detect

map_layers = analyse(sub_elev, sub_count, res)
truth_layers = analyse(truth_ned, np.ones_like(sub_count), res)

map_res = detect(map_layers, sub_elev, res, sub_origin,
                 footprint_m=FOOTPRINT_M,
                 safety_margin_m=SAFETY_MARGIN_M, **THRESHOLDS)
truth_res = detect(truth_layers, truth_ned, res, sub_origin,
                   footprint_m=FOOTPRINT_M,
                   safety_margin_m=0.0, **THRESHOLDS)

mapped = map_res['candidate']
real = truth_res['candidate']

# ---------------------------------------------------------------- report

print(f'map        {MAP_PATH}')
print(f'thresholds slope<={THRESHOLDS["max_slope_deg"]}deg '
      f'roughness<={THRESHOLDS["max_roughness_m"]}m '
      f'step<={THRESHOLDS["max_step_m"]}m')
print(f'footprint  {FOOTPRINT_M} m '
      f'({2 * map_res["footprint_radius"] + 1} cells square)')
print(f'margin     {SAFETY_MARGIN_M} m ({map_res["margin_cells"]} cells)')
print()

false_accept = mapped & ~real
false_reject = real & ~mapped
correct = mapped & real

print(f'truth says landable   {real.sum():>6} cells')
print(f'map says landable     {mapped.sum():>6} cells')
print(f'  correct             {correct.sum():>6}')
print(f'  FALSE ACCEPT        {false_accept.sum():>6}   <- must be 0')
print(f'  false reject        {false_reject.sum():>6}')
if real.sum():
    print(f'  recall              {100.0 * correct.sum() / real.sum():>5.1f}%')
print()

print(f'{"zone":<10} {"cells":>6} {"truth ok":>9} {"map ok":>8} '
      f'{"f.accept":>9} {"f.reject":>9}')
for code, name in enumerate(ZONE_NAMES):
    sel = zone_ned == code
    print(f'{name:<10} {sel.sum():>6} {(real & sel).sum():>9} '
          f'{(mapped & sel).sum():>8} '
          f'{(false_accept & sel).sum():>9} '
          f'{(false_reject & sel).sum():>9}')

if false_accept.sum():
    print()
    print('false accepts by true zone:')
    for code, name in enumerate(ZONE_NAMES):
        n = (false_accept & (zone_ned == code)).sum()
        if n:
            print(f'  {name}: {n} cells')

print()
print(f'zones found in map: {len(map_res["zones"])}')
for z in map_res['zones'][:5]:
    print(f'  #{z["id"]:<3} {z["area_m2"]:>7.0f} m2 '
          f'at N {z["centre_x"]:+6.1f} E {z["centre_y"]:+6.1f}')

# ------------------------------------------------------------------ plot

from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch

fig, ax = plt.subplots(1, 3, figsize=(18, 6))
extent = [-half_t, half_t, -half_t, half_t]

binary = ListedColormap(['#e8e8e8', '#2a7a3a'])
ax[0].imshow(real, origin='lower', extent=extent, cmap=binary,
             vmin=0, vmax=1)
ax[0].set_title('Ground truth')
ax[0].legend(handles=[Patch(facecolor='#2a7a3a', label='landable'),
                      Patch(facecolor='#e8e8e8', label='not landable')],
             loc='lower left', fontsize=8)

binary_map = ListedColormap(['#e8e8e8', '#1f4e79'])
ax[1].imshow(mapped, origin='lower', extent=extent, cmap=binary_map,
             vmin=0, vmax=1)
ax[1].set_title('Detected from map')
ax[1].legend(handles=[Patch(facecolor='#1f4e79', label='landable'),
                      Patch(facecolor='#e8e8e8', label='not landable')],
             loc='lower left', fontsize=8)

overlay = np.zeros(real.shape + (3,))
overlay[...] = [0.10, 0.10, 0.10]
overlay[correct] = [0.20, 0.70, 0.30]
overlay[false_reject] = [1.00, 0.70, 0.20]
overlay[false_accept] = [1.00, 0.00, 0.00]
ax[2].imshow(overlay, origin='lower', extent=extent)
ax[2].set_title('Agreement')
ax[2].legend(handles=[
    Patch(facecolor=(0.20, 0.70, 0.30), label='correct'),
    Patch(facecolor=(1.00, 0.70, 0.20), label='false reject (safe)'),
    Patch(facecolor=(1.00, 0.00, 0.00), label='FALSE ACCEPT (unsafe)'),
    Patch(facecolor=(0.10, 0.10, 0.10), label='refused by both'),
], loc='lower left', fontsize=8)

for a in ax:
    a.set_xlabel('y east (m)')
    a.set_ylabel('x north (m)')

plt.tight_layout()
plt.savefig('/tmp/slz.png', dpi=110)
print('\nwrote /tmp/slz.png')