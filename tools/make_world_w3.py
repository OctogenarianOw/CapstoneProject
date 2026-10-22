#!/usr/bin/env python3
"""Generate W3, a large mixed world, and its exact truth.

W3 (spec 5.1) is the world of the critical case S9.5 and, as Dorian
asked on 22 Oct, a world of diverse obstacles: a road, so that landing
zones exist for sure, other open landing zones, and rock, vegetation
and buildings over the rest, on a large extent. Each obstacle family
puts a different descriptor of S7 to work:

  step       buildings 3 to 7 m, rooftop units, cars, low walls, the
             0.15 m kerb of the sidewalk (below the 0.24 m threshold, so
             it must be ACCEPTED, the step counterpart of ramp_ok)
  slope      a 4 deg meadow (must be accepted, like ramp_ok), a 12 deg
             smooth slope (refused by slope alone), gable roofs, hills
  roughness  rock ground and boulders, scrub, tussock meadow

Layout, Gazebo ENU (x east, y north), 240 x 240 m, take-off at the
origin; the exploration area proposed is 160 x 160 m (area_half_m 80),
so the edge of the world stays beyond sensor reach from the area:

  road      east-west, 10 m wide, y 14 to 24, the whole width; a
            north-south branch, x 40 to 50, from y 24 to the north edge
  yard      take-off pad, 16 x 16 m at the origin, in the scrub, behind
            a hedge that keeps it apart from the road zone
  town      north of the road, west of the branch: plots of 20 m and
            alleys of 4 m (too narrow to land: an alley keeps 2 cells
            once its edges are refused, a footprint needs 3); a parking
            lot (20 x 20 m) and a flat warehouse roof (20 x 20 m, 4.5 m
            high, an elevated landing zone) are the open plots; other
            large roofs carry units or a gable, so they are not landable
  forest    east of the branch: trees on a jittered 5 m grid, with a
            clearing of 20 x 20 m
  rock      south-west: rocky hills and boulders
  scrub     south-centre: bushes on rough ground around the yard and
            pad Z, 8 x 12 m, 40 m from the take-off, the zone the depot
            of S9.5 invalidates (worlds/w3_depot.sdf, four vans, spawned
            during a mission by experiments/w3_depot.py)
  hills     south-east: tussock meadow, smooth hills, the 4 deg meadow
            and the 12 deg slope

Height cap. Every obstacle stays below 7 m, under the cruise altitude
of 8 m. The lidar looks down from 8 m and never sees above itself: a
tree or a building taller than the flight is seen only on its sides,
and a thin one (a trunk, a canopy) can be estimated well below its top
in the 2.5D map, flown over, and hit. Obstacles of 6 to 7 m are blocked
by I3 (altitude minus the 2 m margin) and flown around; lower ones are
flown over. Taller obstacles are a limit to declare (6.4), or need a
higher cruise altitude and a new energy calibration.

Construction. The terrain is one triangulated grid at 1 m, vertices on
the cell corners, flat zones exactly flat; obstacles are triangle
meshes (boxes, prisms, low-poly spheres, cylinders). Everything is
merged into one OBJ per material, about fifteen draw calls for the
software-rendered lidar of WSL2 (W2 rejected 30976 boxes, one draw call
each, as too heavy). The truth is the top surface of those very
triangles, rasterised on 10 x 10 subsamples per cell, so the world and
its truth cannot diverge (S2); per cell the mean and the maximum, and a
label, the majority of the subsamples. Seeded, so W3 is reproducible.
Rejected: a Gazebo heightmap, whose interpolation the truth would have
to reproduce; one primitive per object (draw calls); a truth from the
analytic spheres the meshes approximate (centimetres apart).

Outputs, from the workspace root:
    python3 tools/make_world_w3.py
    worlds/w3.sdf, worlds/w3_meshes/*.obj, worlds/w3_truth.npz,
    worlds/w3_depot.sdf, worlds/w3_depot_truth.npz, and a check of the
    truth through the landing detection chain: every landable component
    with a whole footprint, the zone it lies in, and its size
"""

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]
                       / 'src' / 'uav_core'))
from make_world import box_pair, PX4_WORLDS  # noqa: E402
from uav_core.slz_detection import detect, footprint_ok  # noqa: E402
from uav_core.slz_detection import label_zones  # noqa: E402
from uav_core.terrain_analysis import analyse  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
WORLD_NAME = 'w3'
OUT_SDF = ROOT / 'worlds' / 'w3.sdf'
MESH_DIR = ROOT / 'worlds' / 'w3_meshes'
OUT_TRUTH = ROOT / 'worlds' / 'w3_truth.npz'
OUT_DEPOT = ROOT / 'worlds' / 'w3_depot.sdf'
OUT_DEPOT_TRUTH = ROOT / 'worlds' / 'w3_depot_truth.npz'

HALF = 120.0               # half side of the world, m
RES = 1.0                  # truth cell and terrain vertex spacing, m
SUB = 10                   # truth subsamples per cell side
SEED = 20261022
CAP = 7.0                  # highest obstacle, below the 8 m cruise
FOOTPRINT_M = 1.5
THRESHOLDS = {'max_slope_deg': 6.0, 'max_roughness_m': 0.05,
              'max_step_m': 0.24}

# Ground zones, (x0, x1, y0, y1), Gazebo ENU; later entries win.
ROCK = (-HALF, -24.0, -HALF, 14.0)
SCRUB = (-24.0, 60.0, -HALF, 14.0)
HILLS = (60.0, HALF, -HALF, 14.0)
TOWN = (-HALF, 40.0, 24.0, HALF)
FOREST = (50.0, HALF, 24.0, HALF)
ROAD_EW = (-HALF, HALF, 14.0, 24.0)
ROAD_NS = (40.0, 50.0, 24.0, HALF)
YARD = (-8.0, 8.0, -8.0, 8.0)
PAD_Z = (24.0, 32.0, -34.0, -22.0)
PARKING = (-46.0, -26.0, 28.0, 48.0)
ROOF = (-22.0, -2.0, 52.0, 72.0)
ROOF_H = 4.5
CLEARING = (74.0, 94.0, 56.0, 76.0)
SLOPE_OK = (66.0, 96.0, -56.0, -32.0)      # rises north at 4 deg
SLOPE_BAD = (66.0, 96.0, -24.0, -6.0)      # rises north at 12 deg
SIDEWALK = (-HALF, 40.0, 24.0, 26.0)       # kerb 0.15 m, town side
KERB = 0.15

NAMES = ['road', 'yard', 'parking', 'roof_flat', 'clearing', 'pad_z',
         'slope_ok', 'slope_bad', 'sidewalk', 'town_ground',
         'forest_floor', 'meadow', 'rock', 'scrub', 'building',
         'roof_gable', 'roof_unit', 'car', 'wall', 'trunk', 'tree',
         'bush', 'boulder', 'van']
ID = {n: k for k, n in enumerate(NAMES)}
# Landing zones by design, for the check
SLZ = ['road', 'yard', 'parking', 'roof_flat', 'clearing', 'pad_z',
       'slope_ok', 'sidewalk']

COLOURS = {
    'asphalt': '0.18 0.18 0.19 1', 'concrete': '0.62 0.61 0.58 1',
    'dirt': '0.52 0.42 0.30 1', 'grass': '0.33 0.50 0.24 1',
    'rough_grass': '0.40 0.48 0.26 1', 'rock_ground': '0.47 0.45 0.42 1',
    'walls': '0.78 0.74 0.66 1', 'roofs': '0.55 0.30 0.24 1',
    'flat_roofs': '0.50 0.50 0.50 1', 'units': '0.70 0.72 0.74 1',
    'cars': '0.25 0.35 0.55 1', 'garden_walls': '0.66 0.60 0.50 1',
    'trunks': '0.35 0.25 0.16 1', 'foliage': '0.16 0.38 0.14 1',
    'bushes': '0.24 0.42 0.18 1', 'boulders': '0.42 0.41 0.39 1',
    'kerb': '0.70 0.70 0.68 1',
}


def inside(box, x, y, margin=0.0):
    """Mask of points inside a zone rectangle, grown by margin."""
    x0, x1, y0, y1 = box
    return ((x >= x0 - margin) & (x <= x1 + margin)
            & (y >= y0 - margin) & (y <= y1 + margin))


# ------------------------------------------------------------- terrain
def ground_zone(x, y):
    """Ground zone id at points (x, y), by priority."""
    z = np.full(np.shape(x), ID['scrub'], dtype=np.int8)
    for box, name in ((ROCK, 'rock'), (SCRUB, 'scrub'), (HILLS, 'meadow'),
                      (TOWN, 'town_ground'), (FOREST, 'forest_floor'),
                      (ROAD_EW, 'road'), (ROAD_NS, 'road'), (YARD, 'yard'),
                      (PAD_Z, 'pad_z'), (PARKING, 'parking'),
                      (CLEARING, 'clearing'), (SLOPE_OK, 'slope_ok'),
                      (SLOPE_BAD, 'slope_bad')):
        z[inside(box, x, y)] = ID[name]
    return z


def taper(x, y, box, width):
    """0 on the edge of a zone rectangle, 1 deeper than width inside."""
    x0, x1, y0, y1 = box
    d = np.minimum.reduce([x - x0, x1 - x, y - y0, y1 - y])
    return np.clip(d / width, 0.0, 1.0)


def vertex_heights(rng):
    """Terrain height at the grid vertices (cell corners)."""
    v = np.arange(-HALF, HALF + RES / 2, RES)
    x, y = np.meshgrid(v, v, indexing='ij')
    h = np.zeros_like(x)
    noise = np.abs(rng.normal(0.0, 1.0, x.shape))

    rock = inside(ROCK, x, y)
    hills = 1.8 * (np.sin(2 * np.pi * x / 41 + 0.5) + 1) \
        * (np.sin(2 * np.pi * y / 33 + 1.1) + 1) / 2
    h[rock] = (taper(x, y, ROCK, 8.0) * (hills + 0.3) + 0.25 * noise)[rock]

    scrub = inside(SCRUB, x, y)
    wave = 0.25 * (1 + np.sin(2 * np.pi * x / 23) * np.sin(2 * np.pi * y / 19))
    h[scrub] = (wave + 0.12 * noise)[scrub]

    meadow = inside(HILLS, x, y)
    bumps = sum(a * np.exp(-((x - cx) ** 2 + (y - cy) ** 2) / (2 * s * s))
                for cx, cy, a, s in ((104, -80, 5.0, 12), (75, -95, 4.0, 10),
                                     (110, -20, 4.5, 9)))
    # Tussock meadow: at 0.12 m of vertex noise, patches of the meadow
    # passed the cell test and joined the 4 deg meadow to the road
    h[meadow] = (taper(x, y, HILLS, 6.0) * bumps + 0.3 * noise)[meadow]

    town = inside(TOWN, x, y) | inside(FOREST, x, y)
    h[town] = (0.02 * noise)[town]
    # Smooth slopes: planes, a hair of noise, rising north from 0
    for box, deg in ((SLOPE_OK, 4.0), (SLOPE_BAD, 12.0)):
        m = inside(box, x, y)
        h[m] = (np.tan(np.radians(deg)) * (y - box[2]) + 0.01 * noise)[m]
    # Exactly flat zones, every vertex on or inside the edge at 0
    for box in (ROAD_EW, ROAD_NS, YARD, PAD_Z, PARKING, CLEARING):
        h[inside(box, x, y)] = 0.0
    return v, np.maximum(h, 0.0)


def terrain_at(v, hv, x, y):
    """Height of the triangulated terrain at points (x, y)."""
    fx = (np.asarray(x) - v[0]) / RES
    fy = (np.asarray(y) - v[0]) / RES
    i = np.clip(np.floor(fx).astype(int), 0, len(v) - 2)
    j = np.clip(np.floor(fy).astype(int), 0, len(v) - 2)
    u, w = fx - i, fy - j
    h00, h10 = hv[i, j], hv[i + 1, j]
    h01, h11 = hv[i, j + 1], hv[i + 1, j + 1]
    # Each square split along (i, j) -> (i + 1, j + 1), as in the OBJ
    lower = h00 + u * (h10 - h00) + w * (h11 - h10)
    upper = h00 + w * (h01 - h00) + u * (h11 - h01)
    return np.where(u >= w, lower, upper)


def terrain_mesh(v, hv):
    """Triangles of the terrain grid, and the ground zone of each."""
    n = len(v)
    x, y = np.meshgrid(v, v, indexing='ij')
    verts = np.stack([x.ravel(), y.ravel(), hv.ravel()], axis=1)
    idx = np.arange(n * n).reshape(n, n)
    a, b = idx[:-1, :-1].ravel(), idx[1:, :-1].ravel()
    c, d = idx[1:, 1:].ravel(), idx[:-1, 1:].ravel()
    faces = np.concatenate([np.stack([a, b, c], 1), np.stack([a, c, d], 1)])
    cx = (v[:-1, None] + RES / 2) * np.ones((1, n - 1))
    cy = (v[None, :-1] + RES / 2) * np.ones((n - 1, 1))
    zone = ground_zone(cx, cy).ravel()
    return verts, faces, np.concatenate([zone, zone])


GROUND_MATERIAL = {'road': 'asphalt', 'parking': 'asphalt',
                   'yard': 'concrete', 'pad_z': 'dirt', 'scrub': 'dirt',
                   'rock': 'rock_ground', 'meadow': 'rough_grass',
                   'clearing': 'grass', 'slope_ok': 'grass',
                   'slope_bad': 'grass', 'town_ground': 'concrete',
                   'forest_floor': 'grass'}


# ------------------------------------------------------------- shapes
def orient(verts, faces, centre):
    """Wind every face counter-clockwise seen from outside a convex body."""
    t = verts[faces]
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    out = (t.mean(axis=1) - centre)
    flip = (n * out).sum(axis=1) < 0
    faces = faces.copy()
    faces[flip] = faces[flip][:, ::-1]
    return faces


def box_mesh(cx, cy, z0, sx, sy, sz, yaw=0.0):
    """Closed box, its faces apart so each is shaded flat."""
    hx, hy = sx / 2, sy / 2
    corners = np.array([[sxg * hx, syg * hy, zg]
                        for zg in (0.0, sz) for syg in (-1, 1)
                        for sxg in (-1, 1)])
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    p = corners @ rot.T + [cx, cy, z0]
    quads = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6),
             (0, 2, 6, 4), (1, 3, 7, 5)]
    verts, faces = [], []
    for q in quads:
        k = len(verts)
        verts.extend(p[list(q)])
        faces += [(k, k + 1, k + 2), (k, k + 2, k + 3)]
    verts = np.array(verts)
    return verts, orient(verts, np.array(faces),
                         np.array([cx, cy, z0 + sz / 2]))


def gable_mesh(x0, x1, y0, y1, z0, rise):
    """Gable roof on a rectangle, ridge along its longer side."""
    along_x = (x1 - x0) >= (y1 - y0)
    if along_x:
        ym = (y0 + y1) / 2
        base = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        ridge = [(x0, ym), (x1, ym)]
        slopes = [(0, 1, 5, 4), (3, 2, 5, 4)]
        ends = [(0, 3, 4), (1, 2, 5)]
    else:
        xm = (x0 + x1) / 2
        base = [(x0, y0), (x0, y1), (x1, y1), (x1, y0)]
        ridge = [(xm, y0), (xm, y1)]
        slopes = [(0, 1, 5, 4), (3, 2, 5, 4)]
        ends = [(0, 3, 4), (1, 2, 5)]
    p = np.array([(a, b, z0) for a, b in base]
                 + [(a, b, z0 + rise) for a, b in ridge])
    verts, faces = [], []
    for q in slopes:
        k = len(verts)
        verts.extend(p[list(q)])
        faces += [(k, k + 1, k + 2), (k, k + 2, k + 3)]
    for tri in ends:
        k = len(verts)
        verts.extend(p[list(tri)])
        faces.append((k, k + 1, k + 2))
    verts = np.array(verts)
    centre = np.array([(x0 + x1) / 2, (y0 + y1) / 2, z0 + rise / 3])
    return verts, orient(verts, np.array(faces), centre)


_ICO = {}


def icosphere(level):
    """Unit icosphere, vertices and faces, cached."""
    if level in _ICO:
        return _ICO[level]
    t = (1 + 5 ** 0.5) / 2
    v = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t),
         (0, 1, t), (0, -1, -t), (0, 1, -t), (t, 0, -1), (t, 0, 1),
         (-t, 0, -1), (-t, 0, 1)]
    f = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
         (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
         (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
         (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    v = [np.array(p, float) / np.linalg.norm(p) for p in v]
    for _ in range(level):
        mid, nf = {}, []

        def m(a, b):
            key = (min(a, b), max(a, b))
            if key not in mid:
                p = v[a] + v[b]
                v.append(p / np.linalg.norm(p))
                mid[key] = len(v) - 1
            return mid[key]
        for a, b, c in f:
            ab, bc, ca = m(a, b), m(b, c), m(c, a)
            nf += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        f = nf
    verts = np.array(v)
    _ICO[level] = (verts, orient(verts, np.array(f), np.zeros(3)))
    return _ICO[level]


def ellipsoid_mesh(cx, cy, cz, rx, ry, rz, level=1, jitter=0.0, rng=None):
    """Low-poly ellipsoid, optionally roughened (boulders)."""
    v, f = icosphere(level)
    s = np.ones(len(v))
    if jitter and rng is not None:
        s = 1.0 + rng.uniform(-jitter, jitter, len(v))
    p = v * s[:, None] * [rx, ry, rz] + [cx, cy, cz]
    return p, orient(p, f, np.array([cx, cy, cz]))


def cylinder_mesh(cx, cy, z0, r, h, n=8):
    """Open vertical cylinder, the sides only (a trunk)."""
    a = 2 * np.pi * np.arange(n) / n
    ring = np.stack([cx + r * np.cos(a), cy + r * np.sin(a)], 1)
    verts = np.concatenate([np.column_stack([ring, np.full(n, z0)]),
                            np.column_stack([ring, np.full(n, z0 + h)])])
    faces = []
    for k in range(n):
        k1 = (k + 1) % n
        faces += [(k, k1, n + k1), (k, n + k1, n + k)]
    return verts, orient(verts, np.array(faces),
                         np.array([cx, cy, z0 + h / 2]))


# ------------------------------------------------------------- scene
class Scene:
    """Triangles by material, each with a truth label."""

    def __init__(self):
        self.parts = {}

    def add(self, material, label, mesh):
        verts, faces = mesh
        self.parts.setdefault(material, []).append((verts, faces, label))

    def merged(self, material):
        """One mesh of a material: vertices, faces, per face label."""
        vs, fs, ls, k = [], [], [], 0
        for verts, faces, label in self.parts[material]:
            vs.append(verts)
            fs.append(faces + k)
            lab = np.broadcast_to(np.asarray(label, dtype=np.int8),
                                  (len(faces),))
            ls.append(lab)
            k += len(verts)
        return np.concatenate(vs), np.concatenate(fs), np.concatenate(ls)


def jittered(box, pitch, rng, jitter):
    """Points on a jittered grid over a rectangle."""
    x0, x1, y0, y1 = box
    xs = np.arange(x0 + pitch / 2, x1, pitch)
    ys = np.arange(y0 + pitch / 2, y1, pitch)
    x, y = np.meshgrid(xs, ys, indexing='ij')
    x = x.ravel() + rng.uniform(-jitter, jitter, x.size)
    y = y.ravel() + rng.uniform(-jitter, jitter, y.size)
    return x, y


def build_town(scene, rng):
    """Plots of 20 m, alleys of 4 m, one parking lot, one flat roof."""
    plots = []
    for x0 in np.arange(-118.0, 36.0, 24.0):
        for y0 in (28.0, 52.0, 76.0, 100.0):
            plots.append((x0, min(x0 + 20.0, 38.0), y0, y0 + 20.0))
    for p in plots:
        if (p[0], p[2]) == (PARKING[0], PARKING[2]):
            continue
        if (p[0], p[2]) == (ROOF[0], ROOF[2]):
            scene.add('flat_roofs', ID['roof_flat'],
                      box_mesh((p[0] + p[1]) / 2, (p[2] + p[3]) / 2, 0.0,
                               p[1] - p[0], p[3] - p[2], ROOF_H))
            continue
        layout = rng.integers(0, 3)
        w, d = p[1] - p[0], p[3] - p[2]
        if layout == 0 or w < 16:      # houses of 8 x 8, gaps of 4 m
            cells = [(p[0] + i * 12.0, p[2] + j * 12.0)
                     for i in range(int((w + 4) // 12))
                     for j in range(int((d + 4) // 12))]
            boxes = [(cx, cx + 8.0, cy, cy + 8.0) for cx, cy in cells]
        elif layout == 1:              # two blocks, 3 m apart
            cut = p[0] + rng.integers(7, 11)
            boxes = [(p[0], cut, p[2], p[3]), (cut + 3.0, p[1], p[2], p[3])]
        else:                          # one block
            boxes = [(p[0] + 1.0, p[1] - 1.0, p[2] + 1.0, p[3] - 1.0)]
        for bx in boxes:
            add_building(scene, bx, rng)
    # Cars in the alleys, low walls along some plots
    for _ in range(40):
        p = plots[rng.integers(len(plots))]
        if rng.random() < 0.5:         # alley east of the plot, along y
            cx, cy, yaw = p[1] + 2.0, rng.uniform(p[2] + 3, p[3] - 3), \
                np.pi / 2
        else:                          # alley north of the plot, along x
            cx, cy, yaw = rng.uniform(p[0] + 3, p[1] - 3), p[3] + 2.0, 0.0
        if inside(ROAD_NS, np.array(cx), np.array(cy), 1.0) or cy > 118:
            continue
        scene.add('cars', ID['car'], box_mesh(cx, cy, 0.0, 4.5, 1.8, 1.5,
                                              yaw))
    for p in plots[::5]:
        if (p[0], p[2]) in ((PARKING[0], PARKING[2]), (ROOF[0], ROOF[2])):
            continue
        scene.add('garden_walls', ID['wall'],
                  box_mesh((p[0] + p[1]) / 2, p[2] + 0.2, 0.0,
                           p[1] - p[0], 0.4, 1.2))
    # A walled parking lot: open, the flat alleys along it joined it to
    # the road zone; walled, it is a zone of its own
    x0, x1, y0, y1 = PARKING
    for cx, cy, sx, sy in (((x0 + x1) / 2, y0 + 0.2, x1 - x0, 0.4),
                           ((x0 + x1) / 2, y1 - 0.2, x1 - x0, 0.4),
                           (x0 + 0.2, (y0 + y1) / 2, 0.4, y1 - y0),
                           (x1 - 0.2, (y0 + y1) / 2, 0.4, y1 - y0)):
        scene.add('garden_walls', ID['wall'],
                  box_mesh(cx, cy, 0.0, sx, sy, 1.2))


def add_building(scene, bx, rng):
    """One building: walls, and a roof that is not a landing zone."""
    x0, x1, y0, y1 = bx
    w, d = x1 - x0, y1 - y0
    # A flat roof of 8 m keeps 6 x 6 cells once its edges are refused:
    # a landing zone. Small buildings are always gabled; large flat
    # roofs carry units on a 6 m pitch, which leave at most 2 clear
    # cells between their refused rings, and stay low enough for them.
    gable = min(w, d) < 10 or rng.random() < 0.3
    top = CAP - 1.5 if gable else CAP - 1.0
    h = float(np.round(rng.uniform(3.0, top) * 2) / 2)
    scene.add('walls', ID['building'],
              box_mesh((x0 + x1) / 2, (y0 + y1) / 2, 0.0, w, d, h))
    if gable:
        rise = min(CAP - h, 0.35 * min(w, d))
        scene.add('roofs', ID['roof_gable'],
                  gable_mesh(x0, x1, y0, y1, h, rise))
        return
    for ux in np.arange(x0 + 3.0, x1 - 1.0, 6.0):
        for uy in np.arange(y0 + 3.0, y1 - 1.0, 6.0):
            scene.add('units', ID['roof_unit'],
                      box_mesh(ux, uy, h, 1.6, 1.6, 1.0))


def build_vegetation(scene, rng, v, hv):
    """Forest with a clearing, scrub, hedge, a few trees on the hills."""
    def ground(x, y):
        return float(terrain_at(v, hv, x, y))

    x, y = jittered((FOREST[0] + 4.0, FOREST[1], FOREST[2] + 2.0, FOREST[3]),
                    5.0, rng, 1.0)
    for tx, ty in zip(x, y):
        r = rng.uniform(1.6, 2.6)
        if inside(CLEARING, np.array(tx), np.array(ty), r + 1.0):
            continue
        add_tree(scene, tx, ty, ground(tx, ty), r, rng)
    for tx, ty in zip(*jittered(HILLS, 17.0, rng, 4.0)):
        if any(inside(b, np.array(tx), np.array(ty), 5.0)
               for b in (SLOPE_OK, SLOPE_BAD)):
            continue
        add_tree(scene, tx, ty, ground(tx, ty), rng.uniform(1.6, 2.4), rng)

    x, y = jittered(SCRUB, 3.5, rng, 0.8)
    hedge_x = np.arange(-20.0, 20.1, 1.6)
    x = np.concatenate([x, hedge_x])
    y = np.concatenate([y, np.full(hedge_x.size, 11.0)])
    for bx, by in zip(x, y):
        r = rng.uniform(0.5, 1.3)
        if (inside(YARD, np.array(bx), np.array(by), r + 1.0)
                or inside(PAD_Z, np.array(bx), np.array(by), r + 1.5)
                or not inside(SCRUB, np.array(bx), np.array(by), -r)):
            continue
        rz = r * rng.uniform(0.6, 1.0)
        scene.add('bushes', ID['bush'],
                  ellipsoid_mesh(bx, by, ground(bx, by) + 0.4 * rz, r,
                                 r * rng.uniform(0.8, 1.2), rz))

    x, y = jittered(ROCK, 7.0, rng, 2.5)
    for bx, by in zip(x, y):
        r = rng.uniform(0.5, 1.6)
        if not inside(ROCK, np.array(bx), np.array(by), -r):
            continue
        rz = r * rng.uniform(0.6, 0.9)
        scene.add('boulders', ID['boulder'],
                  ellipsoid_mesh(bx, by, ground(bx, by) + 0.3 * rz, r,
                                 r * rng.uniform(0.7, 1.3), rz,
                                 jitter=0.15, rng=rng))


def add_tree(scene, tx, ty, z0, r, rng):
    """Trunk and canopy, the top at most CAP in absolute height: the
    aircraft flies at 8 m above the take-off, not above the ground."""
    room = CAP - z0
    if room < r + 1.5:
        return
    top = rng.uniform(min(4.0, room), room)
    cz = z0 + top - r
    scene.add('trunks', ID['trunk'],
              cylinder_mesh(tx, ty, z0, 0.2, max(cz - z0, 0.5)))
    scene.add('foliage', ID['tree'],
              ellipsoid_mesh(tx, ty, cz, r, r, r, level=1))


def build_kerb(scene):
    """Sidewalk on the town side of the road, 0.15 m high."""
    x0, x1, y0, y1 = SIDEWALK
    for a, b in ((x0, ROAD_NS[0]),):
        scene.add('kerb', ID['sidewalk'],
                  box_mesh((a + b) / 2, (y0 + y1) / 2, 0.0, b - a,
                           y1 - y0, KERB))


# ------------------------------------------------------------- truth
def rasterise(zbuf, lab, verts, faces, labels):
    """Top surface of triangles into the subsample buffers."""
    step = RES / SUB
    n = zbuf.shape[0]
    t = verts[faces]
    nz = ((t[:, 1, 0] - t[:, 0, 0]) * (t[:, 2, 1] - t[:, 0, 1])
          - (t[:, 1, 1] - t[:, 0, 1]) * (t[:, 2, 0] - t[:, 0, 0]))
    keep = np.abs(nz) > 1e-9
    for tri, label in zip(t[keep], labels[keep]):
        (xa, ya, za), (xb, yb, zb), (xc, yc, zc) = tri
        i0 = max(int(np.ceil((min(xa, xb, xc) + HALF) / step - 0.5)), 0)
        i1 = min(int(np.floor((max(xa, xb, xc) + HALF) / step - 0.5)), n - 1)
        j0 = max(int(np.ceil((min(ya, yb, yc) + HALF) / step - 0.5)), 0)
        j1 = min(int(np.floor((max(ya, yb, yc) + HALF) / step - 0.5)), n - 1)
        if i1 < i0 or j1 < j0:
            continue
        px = (-HALF + (np.arange(i0, i1 + 1) + 0.5) * step)[:, None]
        py = (-HALF + (np.arange(j0, j1 + 1) + 0.5) * step)[None, :]
        det = (yb - yc) * (xa - xc) + (xc - xb) * (ya - yc)
        l1 = ((yb - yc) * (px - xc) + (xc - xb) * (py - yc)) / det
        l2 = ((yc - ya) * (px - xc) + (xa - xc) * (py - yc)) / det
        l3 = 1.0 - l1 - l2
        ins = (l1 >= -1e-9) & (l2 >= -1e-9) & (l3 >= -1e-9)
        z = l1 * za + l2 * zb + l3 * zc
        zs = zbuf[i0:i1 + 1, j0:j1 + 1]
        up = ins & (z > zs)
        zs[up] = z[up]
        lab[i0:i1 + 1, j0:j1 + 1][up] = label


def truth_from(zbuf, lab):
    """Per cell mean, maximum and majority label."""
    n = int(round(2 * HALF / RES))
    z = zbuf.reshape(n, SUB, n, SUB)
    counts = np.stack([(lab == k).reshape(n, SUB, n, SUB).sum(axis=(1, 3))
                       for k in range(len(NAMES))])
    return z.mean(axis=(1, 3)), z.max(axis=(1, 3)), counts.argmax(axis=0)


# ------------------------------------------------------------- outputs
def write_obj(path, verts, faces):
    """Wavefront OBJ with smooth vertex normals (faces kept apart where
    flat shading is wanted)."""
    t = verts[faces]
    fn = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    vn = np.zeros_like(verts)
    for k in range(3):
        np.add.at(vn, faces[:, k], fn)
    vn /= np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-12)
    lines = [f'v {a:.3f} {b:.3f} {c:.3f}' for a, b, c in verts]
    lines += [f'vn {a:.4f} {b:.4f} {c:.4f}' for a, b, c in vn]
    lines += [f'f {a}//{a} {b}//{b} {c}//{c}' for a, b, c in faces + 1]
    path.write_text('\n'.join(lines) + '\n')


def visual(name, uri, rgba, collide):
    """SDF visual (and collision) of one mesh file."""
    geo = f'<geometry><mesh><uri>{uri}</uri></mesh></geometry>'
    out = (f'        <visual name="{name}_visual">\n'
           f'          {geo}\n'
           f'          <material><ambient>{rgba}</ambient>'
           f'<diffuse>{rgba}</diffuse>'
           f'<specular>0.05 0.05 0.05 1</specular></material>\n'
           f'        </visual>\n')
    if collide:
        out += (f'        <collision name="{name}_collision">\n'
                f'          {geo}\n'
                f'        </collision>\n')
    return out


def build_sdf(blocks):
    """PX4's default world with W3 injected before </world>."""
    src = (PX4_WORLDS / 'default.sdf').read_text()
    tag = '<world name="default">'
    if tag not in src:
        raise SystemExit(f'world tag not found in {PX4_WORLDS / "default.sdf"}')
    src = src.replace(tag, f'<world name="{WORLD_NAME}">', 1)
    block = (f'    <model name="{WORLD_NAME}_scene">\n'
             '      <static>true</static>\n'
             '      <link name="scene">\n'
             + ''.join(blocks) +
             '      </link>\n'
             '    </model>\n')
    cut = src.rindex('</world>')
    return src[:cut] + block + src[cut:]


def depot_boxes():
    """Four vans of 2 x 4 x 2 m over pad Z: each refuses itself and a
    ring of one cell, and the 4 x 6 blocks tile the 8 x 12 m pad."""
    x0, _, y0, _ = PAD_Z
    return [(x0 + 2.0 + 4.0 * i, y0 + 3.0 + 6.0 * j, 2.0, 4.0, 2.0)
            for i in range(2) for j in range(2)]


def build_depot_sdf():
    """The depot as a static model, box poses in world coordinates."""
    parts = [box_pair(f'van_{k}', (x, y, h / 2, 0, 0, 0), (sx, sy, h),
                      '0.92 0.92 0.90 1')
             for k, (x, y, sx, sy, h) in enumerate(depot_boxes())]
    return ('<?xml version="1.0" ?>\n<sdf version="1.9">\n'
            '  <model name="w3_depot">\n    <static>true</static>\n'
            '    <link name="vans">\n' + ''.join(parts)
            + '    </link>\n  </model>\n</sdf>\n')


def check(mean, label):
    """Landable components of the truth with a whole footprint."""
    count = np.ones(mean.shape, dtype=np.int32)
    layers = analyse(mean, count, RES)
    land = detect(layers, mean, RES, (-HALF, -HALF),
                  footprint_m=FOOTPRINT_M, **THRESHOLDS)
    core = footprint_ok(land['candidate'], land['footprint_offsets'])
    labels, n = label_zones(land['candidate'])
    rows = []
    for k in range(1, n + 1):
        m = labels == k
        if not core[m].any():
            continue
        names, counts = np.unique(label[m], return_counts=True)
        main = NAMES[names[counts.argmax()]]
        ii, jj = np.nonzero(m)
        rows.append((main, int(m.sum()), -HALF + ii.mean() + 0.5,
                     -HALF + jj.mean() + 0.5))
    return rows


def main():
    rng = np.random.default_rng(SEED)
    v, hv = vertex_heights(rng)
    scene = Scene()
    tv, tf, tz = terrain_mesh(v, hv)
    by_material = {}
    for zid in np.unique(tz):
        by_material.setdefault(GROUND_MATERIAL[NAMES[zid]], []).append(zid)
    for material, zids in by_material.items():
        sel = np.isin(tz, zids)
        scene.add(material, tz[sel], (tv, tf[sel]))
    build_town(scene, rng)
    build_vegetation(scene, rng, v, hv)
    build_kerb(scene)

    MESH_DIR.mkdir(parents=True, exist_ok=True)
    n = int(round(2 * HALF / RES)) * SUB
    s = -HALF + (np.arange(n) + 0.5) * (RES / SUB)
    sx, sy = np.meshgrid(s, s, indexing='ij')
    zbuf = terrain_at(v, hv, sx, sy).astype(np.float32)
    lab = ground_zone(np.floor(sx) + 0.5, np.floor(sy) + 0.5)
    del sx, sy
    blocks, tris = [], 0
    for material in sorted(scene.parts):
        verts, faces, labels = scene.merged(material)
        used, inv = np.unique(faces, return_inverse=True)
        verts, faces = verts[used], inv.reshape(faces.shape)
        write_obj(MESH_DIR / f'{material}.obj', verts, faces)
        uri = f'file://{MESH_DIR / (material + ".obj")}'
        blocks.append(visual(material, uri, COLOURS[material],
                             collide=material not in ('foliage', 'bushes')))
        tris += len(faces)
        if material not in GROUND_MATERIAL.values():
            rasterise(zbuf, lab, verts, faces, labels)
        print(f'{material:12s} {len(faces):7d} triangles')
    OUT_SDF.write_text(build_sdf(blocks))
    centres = -HALF + (np.arange(int(round(2 * HALF / RES))) + 0.5) * RES
    mean, top, label = truth_from(zbuf, lab)
    np.savez(OUT_TRUTH, centres=centres, truth=mean, truth_mean=mean,
             truth_max=top, zone=label.astype(np.int8), resolution=RES,
             zone_names=np.array(NAMES))
    OUT_DEPOT.write_text(build_depot_sdf())
    for x, y, bx, by, h in depot_boxes():
        i = slice(int((x - bx / 2 + HALF) * SUB), int((x + bx / 2 + HALF) * SUB))
        j = slice(int((y - by / 2 + HALF) * SUB), int((y + by / 2 + HALF) * SUB))
        zbuf[i, j] = np.maximum(zbuf[i, j], h)
        lab[i, j] = ID['van']
    d_mean, d_top, d_label = truth_from(zbuf, lab)
    np.savez(OUT_DEPOT_TRUTH, centres=centres, truth=d_mean,
             truth_mean=d_mean, truth_max=d_top,
             zone=d_label.astype(np.int8), resolution=RES,
             zone_names=np.array(NAMES))
    print(f'{OUT_SDF}: {len(blocks)} meshes, {tris} triangles')
    print(f'highest top {top.max():.2f} m (cap {CAP} m)')
    for name, truth_mean, truth_label in (('W3', mean, label),
                                          ('W3 with the depot', d_mean,
                                           d_label)):
        rows = check(truth_mean, truth_label)
        print(f'{name}: {len(rows)} landable components with a whole '
              'footprint (truth, detection chain)')
        for main_zone, cells, cx, cy in sorted(rows, key=lambda r: -r[1]):
            flag = '' if main_zone in SLZ else '   <- not a landing zone'
            print(f'  {main_zone:12s} {cells:6d} cells, centre E {cx:+7.1f}'
                  f' N {cy:+7.1f}{flag}')


if __name__ == '__main__':
    main()
