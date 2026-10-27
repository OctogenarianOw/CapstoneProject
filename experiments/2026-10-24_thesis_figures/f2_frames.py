#!/usr/bin/env python3
"""
Spec F2: the four frames and the conversions between them, as the code
applies them (cloud_world.py, elevation_grid.py, the validation tools).

A chain read left to right: the lidar returns in FLU, turned into the
body FRD, then into the world NED where the map lives; the Gazebo
world, ENU, in which the truth is drawn, joins NED from the right.
Every frame is drawn in the same physical scene, a ground compass N
and E repeated in each panel, the aircraft heading 30 deg east of
north, so the reader sees which axes coincide and which flip. The
x, y, z axes keep one colour each across the four panels.

Choices: a drawing of the frames rather than a table (Annexe A.4 is
the table); a non-zero heading, since at zero heading and level
flight FRD and NED coincide and the rotation, the source of the old
"terrain turns red" bug, would not show; the formulas under the
panels, numbered on the arrows, written as the code computes them.

    python3 f2_frames.py
"""

from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch  # noqa: E402
import numpy as np  # noqa: E402

OUT = Path(__file__).resolve().parent / 'f2_frames.png'
YAW_DEG = 30.0

SURFACE, INK, INK2, MUTED, GRID = ('#fcfcfb', '#0b0b0b', '#52514e',
                                   '#898781', '#e1e0d9')
AXIS = ('#2a78d6', '#eb6834', '#1baf7a')       # x, y, z

# Physical directions in plot coordinates (east, north, up).
psi = np.radians(YAW_DEG)
FWD = np.array([np.sin(psi), np.cos(psi), 0.0])
RIGHT = np.array([np.cos(psi), -np.sin(psi), 0.0])
UP = np.array([0.0, 0.0, 1.0])
NORTH, EAST = np.array([0.0, 1.0, 0.0]), np.array([1.0, 0.0, 0.0])

FRAMES = [
    ('lidar, FLU', 'sensor frame lidar_link,\n/lidar/points',
     [(FWD, 'x forward'), (-RIGHT, 'y left'), (UP, 'z up')], True),
    ('body, FRD', 'PX4 body frame,\nvehicle_attitude',
     [(FWD, 'x forward'), (RIGHT, 'y right'), (-UP, 'z down')], True),
    ('world, NED', 'PX4 local frame, origin the EKF\norigin; '
     'the map and every decision',
     [(NORTH, 'x north'), (EAST, 'y east'), (-UP, 'z down')], False),
    ('Gazebo, ENU', 'the world file and the\ntruth maps',
     [(EAST, 'x east'), (NORTH, 'y north'), (UP, 'z up')], False),
]

NOTES = [
    ('1  lidar to body',
     'p_FRD = (x, -y, -z)_FLU + (0, 0, 0.15 m)\n'
     'y and z change sign; the lidar sits\n0.15 m under the body origin.'),
    ('2  body to world',
     'p_NED = R(q) p_FRD + t_NED\n'
     'q the attitude quaternion (w, x, y, z), last\nreceived; t the NED '
     'position at the scan\nstamp - 0.060 s (P5). Without R a point\n'
     '15 m ahead is misplaced in height as\nsoon as the aircraft tilts.'),
    ('3  Gazebo to world',
     'north = y_gz, east = x_gz, down = -z_gz\n'
     'on a grid: truth_ned = -truth_gz.T;\nplus the height of the EKF origin, '
     'removed\nby the median over flat ground (+0.37 m\non the map v6).'),
    ('the grid, in NED',
     'cell (i, j) covers north\n[o_x + i r, o_x + (i + 1) r), east\n'
     '[o_y + j r, o_y + (j + 1) r); r = 1 m,\no = -50 m, a 100 x 100 '
     'grid, heights\nstored down positive.'),
]


def draw_frame(ax, axes, quad):
    ax.set_axis_off()
    lim = 1.15
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    ax.set_box_aspect((1, 1, 1), zoom=1.45)
    ax.view_init(elev=24, azim=-58)
    # Ground compass, the same in every panel, in the south-west corner
    # under the frame so it never meets an axis label.
    base = -0.75 * NORTH - 0.75 * EAST - 1.0 * UP
    for vec, name in ((NORTH, 'N'), (EAST, 'E')):
        end = base + 0.55 * vec
        ax.plot(*zip(base, end), color=MUTED, lw=1.0, ls=(0, (3, 2)))
        ax.text(*(end + 0.12 * vec), name, color=MUTED, fontsize=8,
                ha='center', va='center')
    if quad:
        # Four rotors on the diagonals of the body, in the horizontal
        # plane, so the heading reads at a glance.
        t = np.linspace(0, 2 * np.pi, 40)
        for a in (45, 135, 225, 315):
            d = np.cos(np.radians(a)) * FWD + np.sin(np.radians(a)) * RIGHT
            c = 0.42 * d
            ax.plot(*zip(np.zeros(3), c), color=MUTED, lw=1.0)
            ring = (c[:, None] + 0.13 * (np.cos(t) * FWD[:, None]
                                         + np.sin(t) * RIGHT[:, None]))
            ax.plot(ring[0], ring[1], ring[2], color=MUTED, lw=0.8)
    for k, (vec, label) in enumerate(axes):
        ax.quiver(0, 0, 0, *vec, color=AXIS[k], lw=2.0,
                  arrow_length_ratio=0.16)
        ax.text(*(1.36 * vec), label, color=AXIS[k], fontsize=8,
                ha='center', va='center', fontweight='bold')


def main():
    plt.rcParams.update({'font.family': 'sans-serif', 'font.size': 8,
                         'figure.facecolor': SURFACE,
                         'savefig.facecolor': SURFACE})
    fig = plt.figure(figsize=(11.0, 5.4))
    left, width, gap = 0.02, 0.205, 0.04
    xs = [left + k * (width + gap) for k in range(4)]
    for x, (title, where, axes, quad) in zip(xs, FRAMES):
        ax = fig.add_axes((x, 0.40, width, 0.47), projection='3d')
        ax.set_facecolor(SURFACE)
        draw_frame(ax, axes, quad)
        fig.text(x + width / 2, 0.86, title, ha='center', color=INK,
                 fontsize=10, fontweight='bold')
        fig.text(x + width / 2, 0.385, where, ha='center', va='top',
                 color=INK2, fontsize=7.5)

    # Arrows between panels: 1 and 2 left to right, 3 from Gazebo into
    # NED, right to left.
    for k, (a, b) in enumerate(((0, 1), (1, 2), (3, 2))):
        x0 = xs[min(a, b)] + width
        x1 = xs[max(a, b)]
        start, end = ((x0 - 0.005, x1 + 0.005) if a < b
                      else (x1 + 0.005, x0 - 0.005))
        fig.add_artist(FancyArrowPatch(
            (start, 0.64), (end, 0.64), transform=fig.transFigure,
            arrowstyle='-|>', mutation_scale=14, color=INK, lw=1.2))
        fig.text((x0 + x1) / 2, 0.615, str(k + 1), ha='center',
                 va='top', color=INK, fontsize=10, fontweight='bold')

    for x, (head, body) in zip(xs, NOTES):
        fig.text(x + 0.01, 0.27, head, color=INK, fontsize=8.5,
                 fontweight='bold', va='top')
        fig.text(x + 0.01, 0.235, body, color=INK2, fontsize=7.5,
                 va='top', linespacing=1.35)
    fig.text(0.01, 0.975, 'The four frames and the conversions between '
             f'them, aircraft heading {YAW_DEG:.0f} deg east of north',
             color=INK, fontsize=11, va='top')
    fig.text(0.01, 0.935, 'x blue, y orange, z green in every frame; '
             'dashed, the ground directions north and east',
             color=MUTED, fontsize=8, va='top')
    fig.savefig(OUT, dpi=200)
    print(OUT)


if __name__ == '__main__':
    main()
