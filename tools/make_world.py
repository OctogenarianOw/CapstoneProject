"""Generate a validation world for elevation mapping, plus its exact
ground truth. Both come from the same analytic height function, so the
SDF geometry and the truth grid cannot drift apart.

Layout, in Gazebo world coordinates (x east, y north, z up), drone
spawns at the origin:

    flat        around (0, 0)     reference, noise floor
    ramp 10 deg at (20, 0)        just at a typical landing threshold
    ramp 25 deg at (-20, 0)       clearly unlandable
    rough       at (0, 20)        scattered blocks, roughness not slope
    obstacle    at (0, -20)       3 m vertical block
"""

import os
import numpy as np

# ----------------------------------------------------------------- config

PX4_DIR = os.path.expanduser('~/PX4-Autopilot')
BASE_WORLD = os.path.join(PX4_DIR, 'Tools/simulation/gz/worlds/default.sdf')

OUT_DIR = os.path.expanduser('~/ros2_ws/droneAfter/worlds')
OUT_SDF = os.path.join(OUT_DIR, 'validation.sdf')
OUT_TRUTH = os.path.join(OUT_DIR, 'validation_truth.npz')

WORLD_NAME = 'validation'

GRID_SIZE = 60.0        # metres, square, centred on the origin
GRID_RES = 1.0          # metres per truth cell
SUBSAMPLE = 10          # truth samples per cell edge

RAMP_LEN = 15.0         # along-slope length
RAMP_WIDTH = 15.0
RAMP_THICK = 0.2

ROUGH_SIZE = 12.0       # square side of the rough patch
ROUGH_PITCH = 0.6       # spacing between blocks
ROUGH_BLOCK = 0.45      # block footprint
ROUGH_MIN_H = 0.05
ROUGH_MAX_H = 0.30

OBSTACLE_SIZE = 4.0
OBSTACLE_HEIGHT = 3.0

RAMPS = [
    {'name': 'ramp_10', 'cx': 20.0, 'cy': 0.0, 'deg': 10.0},
    {'name': 'ramp_25', 'cx': -20.0, 'cy': 0.0, 'deg': 25.0},
]
ROUGH_CX, ROUGH_CY = 0.0, 20.0
OBST_CX, OBST_CY = 0.0, -20.0

rng = np.random.default_rng(42)   # fixed seed: SDF and truth must match

# ------------------------------------------------------------ rough patch

def build_rough_blocks():
    """Return a list of (x, y, height) for the scattered blocks."""
    n = int(ROUGH_SIZE / ROUGH_PITCH)
    start = -ROUGH_SIZE / 2.0 + ROUGH_PITCH / 2.0
    blocks = []
    for i in range(n):
        for j in range(n):
            x = ROUGH_CX + start + i * ROUGH_PITCH
            y = ROUGH_CY + start + j * ROUGH_PITCH
            h = rng.uniform(ROUGH_MIN_H, ROUGH_MAX_H)
            blocks.append((x, y, h))
    return blocks


ROUGH_BLOCKS = build_rough_blocks()

# -------------------------------------------------------- height function

def ramp_top(x, ramp):
    """Top surface height of a ramp at world x.

    A box of length L tilted by theta about the y axis, placed so its
    low edge sits on the ground, has a top surface at
        z(x) = (L/2) sin(theta) + (x - cx) tan(theta)
    which is 0 at the low edge and L sin(theta) at the high edge.
    """
    th = np.radians(ramp['deg'])
    return RAMP_LEN / 2.0 * np.sin(th) + (x - ramp['cx']) * np.tan(th)


def height_at(x, y):
    """Exact terrain height at a world point. Scalars or arrays."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    z = np.zeros_like(x)

    for ramp in RAMPS:
        th = np.radians(ramp['deg'])
        half_x = RAMP_LEN * np.cos(th) / 2.0
        inside = ((np.abs(x - ramp['cx']) <= half_x)
                  & (np.abs(y - ramp['cy']) <= RAMP_WIDTH / 2.0))
        z = np.where(inside, np.maximum(z, ramp_top(x, ramp)), z)

    half = ROUGH_BLOCK / 2.0
    for bx, by, bh in ROUGH_BLOCKS:
        inside = (np.abs(x - bx) <= half) & (np.abs(y - by) <= half)
        z = np.where(inside, np.maximum(z, bh), z)

    inside = ((np.abs(x - OBST_CX) <= OBSTACLE_SIZE / 2.0)
              & (np.abs(y - OBST_CY) <= OBSTACLE_SIZE / 2.0))
    z = np.where(inside, np.maximum(z, OBSTACLE_HEIGHT), z)

    return z

# ------------------------------------------------------------- truth grid

def build_truth():
    """Mean terrain height per cell, sampled on a finer subgrid.

    Averaging matters for the rough patch: a 1 m cell holds several
    blocks and gaps, and the map can only ever represent their mean.
    """
    n = int(GRID_SIZE / GRID_RES)
    half = GRID_SIZE / 2.0
    centres = -half + GRID_RES / 2.0 + np.arange(n) * GRID_RES

    step = GRID_RES / SUBSAMPLE
    offs = -GRID_RES / 2.0 + step / 2.0 + np.arange(SUBSAMPLE) * step

    fine_x = (centres[:, None] + offs[None, :]).ravel()
    fine_y = (centres[:, None] + offs[None, :]).ravel()
    gx, gy = np.meshgrid(fine_x, fine_y, indexing='ij')

    fine_z = height_at(gx, gy)

    truth = fine_z.reshape(n, SUBSAMPLE, n, SUBSAMPLE).mean(axis=(1, 3))

    # Zone labels, same indexing as the truth grid.
    cx, cy = np.meshgrid(centres, centres, indexing='ij')
    zone = np.zeros_like(truth, dtype=np.int8)      # 0 = flat reference

    for code, ramp in zip((1, 2), RAMPS):
        th = np.radians(ramp['deg'])
        half_x = RAMP_LEN * np.cos(th) / 2.0
        zone[(np.abs(cx - ramp['cx']) <= half_x)
             & (np.abs(cy - ramp['cy']) <= RAMP_WIDTH / 2.0)] = code

    zone[(np.abs(cx - ROUGH_CX) <= ROUGH_SIZE / 2.0)
         & (np.abs(cy - ROUGH_CY) <= ROUGH_SIZE / 2.0)] = 3
    zone[(np.abs(cx - OBST_CX) <= OBSTACLE_SIZE / 2.0)
         & (np.abs(cy - OBST_CY) <= OBSTACLE_SIZE / 2.0)] = 4

    return centres, truth, zone

# -------------------------------------------------------------- sdf parts

def box_geometry(sx, sy, sz):
    return (f'<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}'
            f'</size></box></geometry>')


def box_pair(name, px, py, pz, sx, sy, sz, rot='0 0 0', rgba='0.6 0.6 0.6 1'):
    """One collision and one visual, both inside the same link."""
    pose = f'<pose>{px:.4f} {py:.4f} {pz:.4f} {rot}</pose>'
    geom = box_geometry(sx, sy, sz)
    return (
        f'      <collision name="{name}_col">{pose}{geom}</collision>\n'
        f'      <visual name="{name}_vis">{pose}{geom}'
        f'<material><ambient>{rgba}</ambient>'
        f'<diffuse>{rgba}</diffuse></material></visual>\n'
    )


def build_model():
    """Everything in a single static link: one body, many shapes.

    Separate models would mean hundreds of links for the physics engine
    to track. Collisions inside one static link cost far less.
    """
    parts = []

    for ramp in RAMPS:
        th = np.radians(ramp['deg'])
        # Centre height so the low edge rests on the ground.
        cz = RAMP_LEN / 2.0 * np.sin(th) - RAMP_THICK / 2.0 * np.cos(th)
        # Negative pitch about y lifts the +x end.
        parts.append(box_pair(
            ramp['name'], ramp['cx'], ramp['cy'], cz,
            RAMP_LEN, RAMP_WIDTH, RAMP_THICK,
            rot=f'0 {-th:.6f} 0', rgba='0.55 0.45 0.35 1'))

    for k, (bx, by, bh) in enumerate(ROUGH_BLOCKS):
        parts.append(box_pair(
            f'rock_{k}', bx, by, bh / 2.0,
            ROUGH_BLOCK, ROUGH_BLOCK, bh, rgba='0.45 0.42 0.40 1'))

    parts.append(box_pair(
        'obstacle', OBST_CX, OBST_CY, OBSTACLE_HEIGHT / 2.0,
        OBSTACLE_SIZE, OBSTACLE_SIZE, OBSTACLE_HEIGHT,
        rgba='0.30 0.30 0.35 1'))

    return (
        '    <model name="validation_terrain">\n'
        '      <static>true</static>\n'
        '      <link name="terrain_link">\n'
        + ''.join(parts) +
        '      </link>\n'
        '    </model>\n'
    )

# ------------------------------------------------------------------- main

def main():
    with open(BASE_WORLD, 'r') as f:
        base = f.read()

    # PX4 waits for a world whose name matches PX4_GZ_WORLD, so the
    # name inside the SDF has to match the file name.
    base = base.replace('<world name="default">',
                        f'<world name="{WORLD_NAME}">', 1)

    cut = base.rfind('</world>')
    if cut < 0:
        raise SystemExit(f'no </world> tag found in {BASE_WORLD}')

    world = base[:cut] + build_model() + base[cut:]

    os.makedirs(OUT_DIR, exist_ok=True)
    with open(OUT_SDF, 'w') as f:
        f.write(world)

    centres, truth, zone = build_truth()
    np.savez(
        OUT_TRUTH,
        centres=centres,
        truth=truth,
        zone=zone,
        resolution=GRID_RES,
        zone_names=np.array(
            ['flat', 'ramp_10', 'ramp_25', 'rough', 'obstacle']),
    )

    print(f'wrote {OUT_SDF}')
    print(f'wrote {OUT_TRUTH}')
    print(f'{len(ROUGH_BLOCKS)} rough blocks, '
          f'truth grid {truth.shape[0]}x{truth.shape[1]} '
          f'at {GRID_RES} m')
    for code, name in enumerate(
            ['flat', 'ramp_10', 'ramp_25', 'rough', 'obstacle']):
        sel = zone == code
        print(f'  {name:<10} {sel.sum():5d} cells | '
              f'mean height {truth[sel].mean():+.3f} m')


if __name__ == '__main__':
    main()