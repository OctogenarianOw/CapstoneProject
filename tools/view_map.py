import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/elevation_map.npz'
d = np.load(path)

elev = d['elevation']
count = d['count']
res = float(d['resolution'])
ox, oy = float(d['origin_x']), float(d['origin_y'])

known = count > 0
ground = np.median(elev[known])

print(f'{known.sum()} cellules connues')
print(f'sol estime a z = {ground:+.4f}')
print(f'z moyen {elev[known].mean():+.4f} | ecart-type {elev[known].std():.4f}')
print(f'observations par cellule : min {count[known].min()} '
      f'| median {int(np.median(count[known]))} | max {count[known].max()}')

rows = np.any(known, axis=1)
cols = np.any(known, axis=0)
i0, i1 = np.where(rows)[0][[0, -1]]
j0, j1 = np.where(cols)[0][[0, -1]]

extent = [oy + j0 * res, oy + j1 * res, ox + i0 * res, ox + i1 * res]

fig, ax = plt.subplots(1, 2, figsize=(14, 6))

im0 = ax[0].imshow(elev[i0:i1 + 1, j0:j1 + 1], origin='lower',
                   extent=extent, cmap='terrain')
ax[0].set_title('Elevation (m, NED)')
ax[0].set_xlabel('y east (m)')
ax[0].set_ylabel('x north (m)')
fig.colorbar(im0, ax=ax[0])

err = np.where(known, np.abs(elev - ground), np.nan)
im1 = ax[1].imshow(err[i0:i1 + 1, j0:j1 + 1], origin='lower',
                   extent=extent, cmap='inferno', vmin=0.0, vmax=0.4)
ax[1].set_title('Ecart au sol estime (m)')
ax[1].set_xlabel('y east (m)')
ax[1].set_ylabel('x north (m)')
fig.colorbar(im1, ax=ax[1])

square = [(0, 0), (10, 0), (10, 10), (0, 10), (0, 0)]
sx = [p[0] for p in square]
sy = [p[1] for p in square]
for a in ax:
    a.plot(sy, sx, 'w--', linewidth=1.5)

plt.tight_layout()
plt.savefig('/tmp/map.png', dpi=110)
print('ecrit dans /tmp/map.png')