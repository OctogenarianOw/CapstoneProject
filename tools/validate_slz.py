"""Measure the safety of SLZ detection against ground truth.

Two error types, and they are not symmetric:

  false accept  a zone declared landable where the real terrain is not
                -> a destroyed aircraft. This must be zero on hazardous
                   ground; on genuinely flat ground near a boundary it
                   costs nothing, so the two are reported apart.
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
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch

sys.path.insert(0, '/home/doria/ros2_ws/droneAfter/src/uav_core')
from uav_core.terrain_analysis import analyse
from uav_core.slz_detection import detect

MAP_PATH = sys.argv[1] if len(sys.argv) > 1 else '/tmp/elevation_map.npz'
TRUTH_PATH = (sys.argv[2] if len(sys.argv) > 2
              else '/home/doria/ros2_ws/droneAfter/worlds/validation_truth.npz')

ZONE_NAMES = ['flat', 'ramp_10', 'ramp_25', 'rough', 'obstacle']
HAZARD_CODES = [1, 2, 3, 4]

THRESHOLDS = dict(max_slope_deg=6.0, max_roughness_m=0.05, max_step_m=0.24)
FOOTPRINT_M = 1.5
SAFETY_MARGIN_M = 0.0

# ------------------------------------------------------------------ load

m = np.load(MAP_PATH)
elev = m['elevation']
count = m['count']
res = float(m['resolution'])
ox, oy = float(m['origin_x']), float(m['origin_y'])

t = np.load(TRUTH_PATH)
truth_ned = -t['truth'].T          # ENU up to NED down, axes swapped
zone_ned = t['zone'].T

n_t = truth_ned.shape[0]
half_t = n_t * res / 2.0
i0 = int(round((-half_t + res / 2.0 - ox) / res))
j0 = int(round((-half_t + res / 2.0 - oy) / res))

if i0 < 0 or j0 < 0 or i0 + n_t > elev.shape[0] or j0 + n_t > elev.shape[1]:
    raise SystemExit('truth grid does not fit inside the map')

sub_elev = elev[i0:i0 + n_t, j0:j0 + n_t]
sub_count = count[i0:i0 + n_t, j0:j0 + n_t]
sub_origin = (-half_t, -half_t)
known = sub_count > 0

# ---------------------------------------------------------------- detect

map_layers = analyse(sub_elev, sub_count, res)
truth_layers = analyse(truth_ned, np.ones_like(sub_count), res)

map_res = detect(map_layers, sub_elev, res, sub_origin,
                 footprint_m=FOOTPRINT_M,
                 safety_margin_m=SAFETY_MARGIN_M, **THRESHOLDS)

# The truth gets no margin: it defines what is genuinely safe, and the
# margin is a precaution only an imperfect map needs to take.
truth_res = detect(truth_layers, truth_ned, res, sub_origin,
                   footprint_m=FOOTPRINT_M,
                   safety_margin_m=0.0, **THRESHOLDS)

mapped = map_res['candidate']
real = truth_res['candidate']

false_accept = mapped & ~real
false_reject = real & ~mapped
correct = mapped & real

# ---------------------------------------------------------------- report

print(f'map        {MAP_PATH}')
print(f'thresholds slope<={THRESHOLDS["max_slope_deg"]}deg '
      f'roughness<={THRESHOLDS["max_roughness_m"]}m '
      f'step<={THRESHOLDS["max_step_m"]}m')
print(f'footprint  {FOOTPRINT_M} m '
      f'({2 * map_res["footprint_radius"] + 1} cells square)')
print(f'margin     {SAFETY_MARGIN_M} m ({map_res["margin_cells"]} cells)')
print()

print(f'truth says landable   {real.sum():>6} cells')
print(f'map says landable     {mapped.sum():>6} cells')
print(f'  correct             {correct.sum():>6}')
print(f'  FALSE ACCEPT        {false_accept.sum():>6}')
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

# The safety metric. A false accept on flat ground near a boundary is
# a labelling artefact; one on a slope is a destroyed aircraft. Only
# the second has to be zero.
hazard = np.isin(zone_ned, HAZARD_CODES)
danger = false_accept & hazard
benign = false_accept & ~hazard

print()
print(f'false accepts: {benign.sum()} benign (flat ground), '
      f'{danger.sum()} on hazardous terrain  <- must be 0')

if danger.sum():
    print()
    print(f'{"N":>7} {"E":>7} {"zone":<9} {"true":>8} {"map":>8} '
          f'{"step":>8} {"obs":>6}')
    ii, jj = np.nonzero(danger)
    order = np.argsort(-truth_layers['slope'][danger])
    for k in order[:20]:
        i, j = ii[k], jj[k]
        print(f'{-half_t + (i + 0.5) * res:>7.1f} '
              f'{-half_t + (j + 0.5) * res:>7.1f} '
              f'{ZONE_NAMES[zone_ned[i, j]]:<9} '
              f'{truth_layers["slope"][i, j]:>7.1f}° '
              f'{map_layers["slope"][i, j]:>7.1f}° '
              f'{map_layers["step"][i, j]:>7.3f}m '
              f'{sub_count[i, j]:>6}')

    # The slope figures disagree by tens of degrees, so the heights
    # themselves have to be inspected before designing any fix.
    ground = np.median(sub_elev[(zone_ned == 0) & known])
    k = int(np.argmax(truth_layers['slope'][danger]))
    ci, cj = int(ii[k]), int(jj[k])
    lo_i, hi_i = max(0, ci - 2), min(n_t - 1, ci + 2)
    lo_j, hi_j = max(0, cj - 2), min(n_t - 1, cj + 2)
    print()
    print(f'heights around N {-half_t + (ci + 0.5) * res:+.1f} '
          f'E {-half_t + (cj + 0.5) * res:+.1f}, north up, '
          f'map offset {ground:+.3f} removed')
    for label, grid in (('truth', truth_ned), ('map', sub_elev - ground)):
        print(f'  {label}:')
        for i in range(hi_i, lo_i - 1, -1):
            print('    ' + ''.join(
                f'{grid[i, j]:>9.2f}' for j in range(lo_j, hi_j + 1)))

print()
print(f'zones found in map: {len(map_res["zones"])}')
for z in map_res['zones'][:5]:
    print(f'  #{z["id"]:<3} {z["area_m2"]:>7.0f} m2 '
          f'at N {z["centre_x"]:+6.1f} E {z["centre_y"]:+6.1f}')

# ------------------------------------------------------------------ plot

fig, ax = plt.subplots(1, 3, figsize=(18, 6))
extent = [-half_t, half_t, -half_t, half_t]

truth_cmap = ListedColormap(['#e8e8e8', '#2a7a3a'])
ax[0].imshow(real, origin='lower', extent=extent, cmap=truth_cmap,
             vmin=0, vmax=1)
ax[0].set_title('Ground truth')
ax[0].legend(handles=[Patch(facecolor='#2a7a3a', label='landable'),
                      Patch(facecolor='#e8e8e8', label='not landable')],
             loc='lower left', fontsize=8)

map_cmap = ListedColormap(['#e8e8e8', '#1f4e79'])
ax[1].imshow(mapped, origin='lower', extent=extent, cmap=map_cmap,
             vmin=0, vmax=1)
ax[1].set_title('Detected from map')
ax[1].legend(handles=[Patch(facecolor='#1f4e79', label='landable'),
                      Patch(facecolor='#e8e8e8', label='not landable')],
             loc='lower left', fontsize=8)

overlay = np.zeros(real.shape + (3,))
overlay[...] = [0.10, 0.10, 0.10]
overlay[correct] = [0.20, 0.70, 0.30]
overlay[false_reject] = [1.00, 0.70, 0.20]
overlay[benign] = [1.00, 0.85, 0.00]
overlay[danger] = [1.00, 0.00, 0.00]
ax[2].imshow(overlay, origin='lower', extent=extent)
ax[2].set_title('Agreement')
ax[2].legend(handles=[
    Patch(facecolor=(0.20, 0.70, 0.30), label='correct'),
    Patch(facecolor=(1.00, 0.70, 0.20), label='false reject (safe)'),
    Patch(facecolor=(1.00, 0.85, 0.00), label='false accept, flat ground'),
    Patch(facecolor=(1.00, 0.00, 0.00), label='FALSE ACCEPT on hazard'),
    Patch(facecolor=(0.10, 0.10, 0.10), label='refused by both'),
], loc='lower left', fontsize=8)

for a in ax:
    a.set_xlabel('y east (m)')
    a.set_ylabel('x north (m)')

plt.tight_layout()
plt.savefig('/tmp/slz.png', dpi=110)
print('\nwrote /tmp/slz.png')