#!/usr/bin/env python3
"""Generate the validation world and its exact ground truth.

Both outputs come from the same height function, so the geometry handed to
Gazebo and the truth used for validation cannot drift apart.

Outputs
    worlds/validation.sdf        PX4 default world with the terrain injected
    worlds/validation_truth.npz  per cell ground truth

Design rules
    1. Nothing floats. Every structure is solid from its top surface down to
       below the ground plane, so no LiDAR ray can enter a cavity from the
       side and report ground where the truth says terrain.
    2. Every footprint is aligned on the truth grid, so no cell is half
       inside and half outside a zone.
    3. The rough zone is quantised to the truth resolution: one cell is one
       flat tile, so the truth of that cell is exact instead of being an
       average over structure the grid cannot represent.

Run from the workspace root:
    python3 tools/make_world.py
"""

from pathlib import Path

import numpy as np

# ----------------------------------------------------------------- locations

PX4_WORLDS = Path.home() / 'PX4-Autopilot' / 'Tools' / 'simulation' / 'gz' / 'worlds'
ROOT = Path(__file__).resolve().parents[1]
OUT_SDF = ROOT / 'worlds' / 'validation.sdf'
OUT_TRUTH = ROOT / 'worlds' / 'validation_truth.npz'

# ---------------------------------------------------------------- parameters

WORLD_NAME = 'validation'
WORLD_SIZE = 60.0          # square side, metres, centred on the origin
RESOLUTION = 1.0           # truth cell size, metres
SUBSAMPLES = 10            # height samples per cell and per axis
SEED = 42

# Ramps. Footprint 16 m so that both edges land on cell borders.
# Each ramp rises towards +x, from z = 0 at its west edge.
RAMP_LEN = 16.0
RAMP_WIDTH = 16.0
RAMP_THICK = 8.0           # see ramp geometry below: thick enough to bury
RAMPS = [
    # zone id, name, centre x, centre y, slope in degrees
    (1, 'ramp_ok', -20.0, 0.0, 3.0),
    (2, 'ramp_bad', 20.0, 0.0, 10.0),
]

# Rough zone: one flat tile per truth cell, random heights.
ROUGH_ID = 3
ROUGH_CX, ROUGH_CY = 0.0, 20.0
ROUGH_SIZE = 12.0
ROUGH_MIN, ROUGH_MAX = 0.10, 0.80

# Solid cube.
CUBE_ID = 4
CUBE_CX, CUBE_CY = 0.0, -20.0
CUBE_SIDE = 4.0

ZONE_NAMES = ['flat', 'ramp_ok', 'ramp_bad', 'rough', 'obstacle']

COLOUR = {
    'ramp': '0.55 0.47 0.37 1',
    'rough': '0.45 0.45 0.45 1',
    'cube': '0.70 0.25 0.20 1',
}

# ------------------------------------------------------------- height field


def rough_tiles():
    """Tile heights of the rough zone, indexed [ix, iy]."""
    k = int(round(ROUGH_SIZE / RESOLUTION))
    rng = np.random.default_rng(SEED)
    return rng.uniform(ROUGH_MIN, ROUGH_MAX, size=(k, k))


def height_field(x, y, tiles):
    """Top surface height and zone id at points (x, y), Gazebo ENU.

    x and y are arrays of the same shape. Zones never overlap, so the blocks
    below can assign rather than take a maximum.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    h = np.zeros(x.shape, dtype=float)
    zone = np.zeros(x.shape, dtype=int)

    for zid, _name, cx, cy, deg in RAMPS:
        inside = ((np.abs(x - cx) <= RAMP_LEN / 2.0)
                  & (np.abs(y - cy) <= RAMP_WIDTH / 2.0))
        rise = (x - (cx - RAMP_LEN / 2.0)) * np.tan(np.radians(deg))
        h = np.where(inside, rise, h)
        zone = np.where(inside, zid, zone)

    k = tiles.shape[0]
    rx0 = ROUGH_CX - ROUGH_SIZE / 2.0
    ry0 = ROUGH_CY - ROUGH_SIZE / 2.0
    inside = ((np.abs(x - ROUGH_CX) <= ROUGH_SIZE / 2.0)
              & (np.abs(y - ROUGH_CY) <= ROUGH_SIZE / 2.0))
    ix = np.clip(((x - rx0) / RESOLUTION).astype(int), 0, k - 1)
    iy = np.clip(((y - ry0) / RESOLUTION).astype(int), 0, k - 1)
    h = np.where(inside, tiles[ix, iy], h)
    zone = np.where(inside, ROUGH_ID, zone)

    inside = ((np.abs(x - CUBE_CX) <= CUBE_SIDE / 2.0)
              & (np.abs(y - CUBE_CY) <= CUBE_SIDE / 2.0))
    h = np.where(inside, CUBE_SIDE, h)
    zone = np.where(inside, CUBE_ID, zone)

    return h, zone


# ---------------------------------------------------------------------- SDF


def box_pair(name, pose, size, rgba):
    """One collision and one visual sharing the same box."""
    px, py, pz, roll, pitch, yaw = pose
    sx, sy, sz = size
    p = f'{px:.4f} {py:.4f} {pz:.4f} {roll:.5f} {pitch:.5f} {yaw:.5f}'
    s = f'{sx:.4f} {sy:.4f} {sz:.4f}'
    return (
        f'        <collision name="{name}_collision">\n'
        f'          <pose>{p}</pose>\n'
        f'          <geometry><box><size>{s}</size></box></geometry>\n'
        f'        </collision>\n'
        f'        <visual name="{name}_visual">\n'
        f'          <pose>{p}</pose>\n'
        f'          <geometry><box><size>{s}</size></box></geometry>\n'
        f'          <material>\n'
        f'            <ambient>{rgba}</ambient>\n'
        f'            <diffuse>{rgba}</diffuse>\n'
        f'            <specular>0.1 0.1 0.1 1</specular>\n'
        f'          </material>\n'
        f'        </visual>\n')


def build_parts(tiles):
    """Every box of the terrain, as SDF fragments."""
    parts = []

    for _zid, name, cx, cy, deg in RAMPS:
        th = np.radians(deg)
        top = RAMP_LEN * np.tan(th)
        # The ramp is one box pitched by -th, so its +x axis rises. Its upper
        # face is the ramp surface: centred on (cx, cy) at z = top / 2, with a
        # horizontal extent of exactly RAMP_LEN. The box centre sits
        # RAMP_THICK / 2 below that face along the surface normal
        # (-sin th, 0, cos th). With RAMP_THICK > top / cos th the underside
        # stays below z = 0 over the whole footprint, so there is no cavity.
        parts.append(box_pair(
            name,
            (cx + RAMP_THICK / 2.0 * np.sin(th),
             cy,
             top / 2.0 - RAMP_THICK / 2.0 * np.cos(th),
             0.0, -th, 0.0),
            (RAMP_LEN / np.cos(th), RAMP_WIDTH, RAMP_THICK),
            COLOUR['ramp']))

    k = tiles.shape[0]
    rx0 = ROUGH_CX - ROUGH_SIZE / 2.0
    ry0 = ROUGH_CY - ROUGH_SIZE / 2.0
    for i in range(k):
        for j in range(k):
            h = float(tiles[i, j])
            # Each tile is solid from the ground up and abuts its neighbours,
            # so the gaps between rocks of the previous world are gone.
            parts.append(box_pair(
                f'rough_{i:02d}_{j:02d}',
                (rx0 + (i + 0.5) * RESOLUTION,
                 ry0 + (j + 0.5) * RESOLUTION,
                 h / 2.0, 0.0, 0.0, 0.0),
                (RESOLUTION, RESOLUTION, h),
                COLOUR['rough']))

    parts.append(box_pair(
        'obstacle',
        (CUBE_CX, CUBE_CY, CUBE_SIDE / 2.0, 0.0, 0.0, 0.0),
        (CUBE_SIDE, CUBE_SIDE, CUBE_SIDE),
        COLOUR['cube']))

    return parts


def build_sdf(parts):
    """PX4's default world with the terrain injected before </world>."""
    src = (PX4_WORLDS / 'default.sdf').read_text()
    tag = '<world name="default">'
    if tag not in src:
        raise SystemExit(f'world tag not found in {PX4_WORLDS / "default.sdf"}')
    src = src.replace(tag, f'<world name="{WORLD_NAME}">', 1)

    block = ('    <model name="validation_terrain">\n'
             '      <static>true</static>\n'
             '      <link name="terrain">\n'
             + ''.join(parts) +
             '      </link>\n'
             '    </model>\n')

    cut = src.rindex('</world>')
    return src[:cut] + block + src[cut:]


# -------------------------------------------------------------------- truth


def build_truth(tiles):
    """Per cell ground truth, axis 0 east, axis 1 north, Gazebo ENU."""
    n = int(round(WORLD_SIZE / RESOLUTION))
    m = SUBSAMPLES
    edge = -WORLD_SIZE / 2.0

    s = edge + (np.arange(n * m) + 0.5) * (RESOLUTION / m)
    sx, sy = np.meshgrid(s, s, indexing='ij')
    h, _ = height_field(sx, sy, tiles)
    h = h.reshape(n, m, n, m)

    # Mean is what a cell averaging estimator can reach, max is what matters
    # for safety at a discontinuity. Both are stored, neither is guessed.
    truth_mean = h.mean(axis=(1, 3))
    truth_max = h.max(axis=(1, 3))

    centres = edge + (np.arange(n) + 0.5) * RESOLUTION
    cx, cy = np.meshgrid(centres, centres, indexing='ij')
    _, zone = height_field(cx, cy, tiles)

    return centres, truth_mean, truth_max, zone.astype(np.int8)


# --------------------------------------------------------------------- main


def main():
    tiles = rough_tiles()

    OUT_SDF.parent.mkdir(parents=True, exist_ok=True)
    parts = build_parts(tiles)
    OUT_SDF.write_text(build_sdf(parts))

    centres, truth_mean, truth_max, zone = build_truth(tiles)
    np.savez(OUT_TRUTH,
             centres=centres,
             truth=truth_mean,
             truth_mean=truth_mean,
             truth_max=truth_max,
             zone=zone,
             resolution=RESOLUTION,
             zone_names=np.array(ZONE_NAMES))

    print(f'{OUT_SDF}  ({len(parts)} boxes)')
    print(f'{OUT_TRUTH}  ({zone.shape[0]} x {zone.shape[1]} cells)')
    print()
    print(f'{"zone":10s} {"cells":>6s} {"h min":>8s} {"h max":>8s}')
    for zid, name in enumerate(ZONE_NAMES):
        mask = zone == zid
        if not mask.any():
            print(f'{name:10s} {0:6d}')
            continue
        print(f'{name:10s} {int(mask.sum()):6d} '
              f'{truth_mean[mask].min():8.3f} {truth_max[mask].max():8.3f}')


if __name__ == '__main__':
    main()