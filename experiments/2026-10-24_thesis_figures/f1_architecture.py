#!/usr/bin/env python3
"""
Spec F1: the architecture of the system, nodes and flows, the
contributions in dashed lines and the state of the art in solid ones.

What is drawn is what runs (src/uav_core/launch/exploration.launch.py
and the publishers and subscriptions of each node), not the target
table of spec 3.3: the simulation on the left (Gazebo Harmonic and
PX4 SITL with their two bridges), the four ROS nodes of uav_core in
the middle, the exploration manager on the right with the pure
modules it calls. Arrows carry the real topic names.

Dashed and in the accent colour: the three contributions of spec 2,
the dynamic landing zone registry, C1 (the invariant I1 kept on every
choice) and C2 (the utility with an energy margin). Solid: what spec
2.5 declares state of the art or infrastructure, the elevation
mapping, the terrain analysis, the frontiers, A*, and the energy
model, which C1 needs but which is a calibration, not a claim.

Choices: drawn with matplotlib patches rather than a diagram tool, so
the figure is regenerated from the repository like every other; one
flow per arrow, the PX4 topics shortened (the /fmu/in and /fmu/out
prefixes and the version suffixes dropped, given in the caption).

    python3 f1_architecture.py
"""

from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch  # noqa: E402

OUT = Path(__file__).resolve().parent / 'f1_architecture.png'

SURFACE, INK, INK2, MUTED, GRID = ('#fcfcfb', '#0b0b0b', '#52514e',
                                   '#898781', '#e1e0d9')
SIM_FACE, NODE_FACE = '#f0efeb', '#ffffff'
ACCENT, ACCENT_FACE = '#eb6834', '#fdf0ea'


def box(ax, x, y, w, h, title, body='', contribution=False, face=NODE_FACE,
        title_size=8.5, body_size=7.0, edge=INK2, lw=1.1, top=False):
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h, boxstyle='round,pad=0,rounding_size=0.9',
        facecolor=ACCENT_FACE if contribution else face,
        edgecolor=ACCENT if contribution else edge,
        linestyle=(0, (4, 2.5)) if contribution else '-',
        linewidth=1.6 if contribution else lw, zorder=2))
    if top:
        ax.text(x + 1.0, y + h - 1.0, title, ha='left', va='top',
                fontsize=title_size, fontweight='bold', color=INK, zorder=3)
        return
    ty = y + h / 2 + (1.4 if body else 0)
    ax.text(x + w / 2, ty, title, ha='center', va='center',
            fontsize=title_size, fontweight='bold',
            color=ACCENT if contribution else INK, zorder=3)
    if body:
        ax.text(x + w / 2, ty - 1.6, body, ha='center', va='top',
                fontsize=body_size, color=INK2, linespacing=1.3, zorder=3)


def strip(ax, x, y, w, h, label):
    ax.add_patch(FancyBboxPatch((x, y), w, h,
                                boxstyle='round,pad=0,rounding_size=0.5',
                                facecolor=GRID, edgecolor=INK2, lw=0.8,
                                zorder=2))
    ax.text(x + w / 2, y + h / 2, label, rotation=90, ha='center',
            va='center', fontsize=6.8, color=INK, zorder=3)


def arrow(ax, p0, p1, label='', at=0.5, dy=0.9, path=None):
    if path is None:
        ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle='-|>',
                                     mutation_scale=9, color=INK2, lw=1.0,
                                     zorder=2.5, shrinkA=0, shrinkB=0))
        lx = p0[0] + at * (p1[0] - p0[0])
        ly = p0[1] + at * (p1[1] - p0[1]) + dy
    else:
        pts = [p0] + path + [p1]
        for a, b in zip(pts[:-2], pts[1:-1]):
            ax.plot(*zip(a, b), color=INK2, lw=1.0, zorder=2.5)
        ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle='-|>',
                                     mutation_scale=9, color=INK2, lw=1.0,
                                     zorder=2.5, shrinkA=0, shrinkB=0))
        lx, ly = (pts[1][0] + pts[2][0]) / 2, pts[1][1] + dy
    if label:
        ax.text(lx, ly, label, ha='center', va='bottom', fontsize=6.4,
                color=INK2, family='monospace', zorder=4,
                bbox={'facecolor': SURFACE, 'edgecolor': 'none',
                      'pad': 0.6})


def main():
    plt.rcParams.update({'font.family': 'sans-serif',
                         'figure.facecolor': SURFACE,
                         'savefig.facecolor': SURFACE})
    fig, ax = plt.subplots(figsize=(11.0, 7.2))
    fig.subplots_adjust(0.005, 0.005, 0.995, 0.93)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 66)
    ax.set_axis_off()

    # Simulation, left.
    ax.text(2, 57.6, 'simulation', fontsize=8, color=MUTED, va='bottom')
    box(ax, 2, 35, 14.5, 22, 'Gazebo Harmonic',
        'validation world W1,\nW2 or W3; x500_slz\nwith a lidar under\n'
        'the body', face=SIM_FACE)
    strip(ax, 17.0, 35, 3.0, 22, 'ros_gz_bridge')
    box(ax, 2, 5, 14.5, 24, 'PX4 SITL',
        'EKF2 on GPS, no SLAM;\nposition and attitude\ncontrol; offboard\n'
        'mode', face=SIM_FACE)
    strip(ax, 17.0, 5, 3.0, 24, 'Micro XRCE-DDS agent')

    # ROS nodes, middle.
    ax.text(31.5, 57.6, 'ROS 2 nodes, package uav_core', fontsize=8,
            color=MUTED, va='bottom')
    box(ax, 31.5, 47, 18, 10, 'energy_monitor',
        'rotor shaft power from\nthe motor speeds, integrated')
    box(ax, 31.5, 29, 18, 13, 'cloud_world',
        'lidar to NED (F2), Kalman\nfusion per cell, elevation\n'
        'grid 100 x 100 m, 1 m')
    box(ax, 31.5, 5, 18, 13, 'trajectory_executor',
        'follows the path: 5 m/s,\n3 m/s² ramps, stop at each\n'
        'vertex; take-off, landing')

    # Exploration manager, right.
    box(ax, 59, 3, 39, 54, 'exploration_manager, sole decision '
        'authority (S15)', face=SURFACE, top=True, title_size=8.5, lw=1.3)
    box(ax, 61, 39, 35, 13, 'world_model',
        'terrain analysis: slope, roughness, step (Beycimen 2025);\n'
        'landing cells and footprint; frontiers (Yamauchi 1997);\n'
        'obstacle map and A* planner', face=NODE_FACE)
    box(ax, 61, 25, 17, 11.5, 'slz_registry',
        'zones confirmed after\ntwo passes (I4);\nloss detected (S9.5)',
        contribution=True)
    box(ax, 79.5, 25, 16.5, 11.5, 'energy_model',
        'E = P t + k d + c n,\ncalibrated (F8)')
    box(ax, 61, 5, 35, 17.5, 'mission_fsm, mission_policy', face=NODE_FACE,
        top=True, title_size=8)
    box(ax, 62.5, 6.5, 15.5, 11.5, 'C1',
        'keeps I1 on every\nchoice: B - R - need\n>= 0 to a zone',
        contribution=True)
    box(ax, 79.5, 6.5, 15, 11.5, 'C2',
        'utility with an\nenergy margin,\nweight alpha',
        contribution=True)
    # Inside the manager: what feeds the decision.
    arrow(ax, (69.5, 39), (69.5, 36.5))
    arrow(ax, (78.75, 39), (78.75, 22.5))
    arrow(ax, (69.5, 25), (69.5, 22.5))
    arrow(ax, (87.75, 25), (87.75, 22.5))

    # Flows.
    arrow(ax, (20.0, 52), (31.5, 52), 'motor_speed')
    arrow(ax, (20.0, 39), (31.5, 39), '/lidar/points\n/clock')
    arrow(ax, (20.0, 25), (31.5, 31.5), 'local_position\nattitude', at=0.4,
          dy=0.4)
    arrow(ax, (31.5, 12.5), (20.0, 12.5), 'trajectory_setpoint\n'
          'offboard_control_mode\nvehicle_command', dy=0.8)
    arrow(ax, (20.0, 8.5), (31.5, 8.5), 'vehicle_status\ntimesync_status',
          dy=-4.4)
    arrow(ax, (49.5, 52), (59, 52), '/energy/state\n10 Hz')
    arrow(ax, (49.5, 35.5), (59, 35.5), '/map/elevation\n1 Hz')
    arrow(ax, (59, 13.5), (49.5, 13.5), '/executor/path\n/executor/command')
    arrow(ax, (49.5, 8.5), (59, 8.5), '/executor/status', dy=-2.9)
    arrow(ax, (18.5, 5), (75, 3), 'local_position, to the manager',
          path=[(18.5, 1.2), (75, 1.2)], dy=0.3)

    # Display and live parameters, above the manager.
    box(ax, 59, 60.5, 18, 4.5, 'RViz', '', title_size=7.5)
    box(ax, 80, 60.5, 18, 4.5, 'demo panel, rqt_reconfigure', '',
        title_size=7.5)
    arrow(ax, (68, 57), (68, 60.5))
    ax.text(68.8, 58.75, '/viz/*', fontsize=6.4, color=INK2,
            family='monospace', va='center')
    arrow(ax, (89, 60.5), (89, 57))
    ax.text(88.2, 58.75, 'alpha, reserve_fraction', fontsize=6.4,
            color=INK2, family='monospace', va='center', ha='right')

    # Legend.
    ax.add_patch(FancyBboxPatch((2, 61.5), 5, 2.6,
                                boxstyle='round,pad=0,rounding_size=0.5',
                                facecolor=ACCENT_FACE, edgecolor=ACCENT,
                                linestyle=(0, (4, 2.5)), lw=1.4))
    ax.text(8, 62.8, 'contribution (spec 2)', fontsize=7.2, color=INK2,
            va='center')
    ax.add_patch(FancyBboxPatch((25, 61.5), 5, 2.6,
                                boxstyle='round,pad=0,rounding_size=0.5',
                                facecolor=NODE_FACE, edgecolor=INK2, lw=1.1))
    ax.text(31, 62.8, 'state of the art or infrastructure (spec 2.5)',
            fontsize=7.2, color=INK2, va='center')
    fig.suptitle('Architecture: what runs, the flows between nodes, and '
                 'where the contributions sit', x=0.01, ha='left',
                 color=INK, fontsize=11)
    fig.savefig(OUT, dpi=200)
    print(OUT)


if __name__ == '__main__':
    main()
