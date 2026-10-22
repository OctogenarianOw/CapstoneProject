#!/usr/bin/env python3
"""
P5 on W1 itself: the pose at the scan time, swept offline on a survey.

The fix v3 (pose at the scan stamp minus 0.085 s, position and
attitude) removed P5 on W1's ramps but raised the flat RMS (0.25 ->
0.30 m) and cut the recall (94 -> 61 %) on a stable clock
(2026-10-22_p5_stable_clock). On the plates (2026-10-21_p5_latency,
bag r2, clean clock) the replay showed the attitude is not the cause
and the term overshoots: A crosses zero near 0.05 s, not 0.085 s; and
the plate flight left the flat ground intact for every variant, so the
recall loss belongs to the W1 survey, flown by another node
(survey_mission). This rebuilds W1 maps from a survey bag, the whole
grid, with the chain of cloud_world, and scores each with the tools
that defined P5 and S8 (check_alignment, validate_map, validate_slz,
through 2026-10-21_p5_fix/analyse.part_b):

  latest            last position and attitude received (dronePropre)
  term_T            both at the scan time minus T, T in TERMS
  term_T_att_latest position at the scan time minus T, attitude latest

Clock offsets as v3 estimates them (variants.py of this diagnosis).

    python3 sweep.py BAG_DIR RUN_DIR OUT_DIR
"""

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path.home() / 'ros2_ws/droneAfter/experiments'
                       / '2026-10-21_p5_fix'))
import variants  # noqa: E402
from analyse import part_b  # noqa: E402
from uav_core.cloud_world import quat_to_matrix  # noqa: E402
from uav_core.elevation_grid import ElevationGrid  # noqa: E402

TERMS = (0.0, 0.03, 0.05, 0.068, 0.085)
GRID = 100.0
MIN_FUSE_HEIGHT = 2.0
SIGMA_RANGE, SIGMA_ANGULAR = 0.02, 0.009


def poses(clouds, pos, att, clock):
    """Pose of every cloud for every variant name."""
    from collections import deque
    out = {'latest': []}
    for term in TERMS:
        out[f'term_{term:.3f}'] = []
        out[f'term_{term:.3f}_att_latest'] = []
    sim, px4 = deque(), deque()
    ci = pi = 0
    for rx, stamp, _ in clouds:
        while ci < len(clock) and clock[ci, 0] <= rx:
            sim.append((clock[ci, 0], clock[ci, 0] - clock[ci, 1]))
            ci += 1
        while sim and sim[0][0] < rx - 0.3:
            sim.popleft()
        while pi < len(pos) and pos[pi, 0] <= rx:
            px4.append((pos[pi, 0], pos[pi, 0] - pos[pi, 2]))
            pi += 1
        while px4 and px4[0][0] < rx - 2.0:
            px4.popleft()
        last_p = pos[max(0, pi - 1), 3:6]
        ka = max(0, int(np.searchsorted(att[:, 0], rx, side='right')) - 1)
        last_q = att[ka, 2:6] / np.linalg.norm(att[ka, 2:6])
        out['latest'].append((last_p, last_q))
        ok = bool(sim) and bool(px4)
        t = (stamp + min(s for _, s in sim)
             - float(np.median([o for _, o in px4]))) if ok else None
        for term in TERMS:
            p = q = None
            if ok and pos[0, 1] <= t - term <= pos[-1, 1]:
                p = np.array([np.interp(t - term, pos[:, 1], pos[:, k])
                              for k in (3, 4, 5)])
                q = variants.nlerp(att, t - term)
            out[f'term_{term:.3f}'].append(
                None if p is None or q is None else (p, q))
            out[f'term_{term:.3f}_att_latest'].append(
                None if p is None else (p, last_q))
    return out


def build(clouds, plist):
    """Whole W1 grid from the clouds with the given poses."""
    grid = ElevationGrid(size_m=GRID, resolution=1.0)
    for k, (_, _, (pts, norms)) in enumerate(clouds):
        if plist[k] is None:
            continue
        p, q = plist[k]
        if -p[2] < MIN_FUSE_HEIGHT:
            continue
        ned = pts @ quat_to_matrix(q).T + p
        sigma = SIGMA_RANGE + SIGMA_ANGULAR * norms
        grid.update(ned, (sigma ** 2).astype(np.float32))
    return grid


def save(grid, path):
    """Write a map as cloud_world does."""
    np.savez(path, elevation=grid.elevation, variance=grid.variance,
             count=grid.count, resolution=grid.resolution,
             origin_x=grid.origin_x, origin_y=grid.origin_y)


def main():
    bag, run, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
    out.mkdir(parents=True, exist_ok=True)
    clouds, pos, att, clock = variants.read(bag)
    print(f'{bag.name}: {len(clouds)} clouds, {len(clock)} clock messages')
    plists = poses(clouds, pos, att, clock)
    live = part_b(run / 'elevation_map.npz')
    print(f'{"variant":<26} {"covered":>8} {"ramp RMS (0,0)":>14} '
          f'{"best shift":>10} {"flat RMS":>8} {"recall":>7} {"severe":>6}')
    print(f'{"live map":<26} {"":>8} {live["ramp_rms_00"]:14.3f} '
          f'{str(live["best_shift"]):>10} {live["flat_rms"]:8.3f} '
          f'{live["recall"]:6.1f}% {live["severe"]:6d}')
    for name, pl in plists.items():
        covered = sum(p is not None for p in pl) / len(pl)
        path = out / f'{name}.npz'
        save(build(clouds, pl), path)
        b = part_b(path)
        print(f'{name:<26} {covered:8.1%} {b["ramp_rms_00"]:14.3f} '
              f'{str(b["best_shift"]):>10} {b["flat_rms"]:8.3f} '
              f'{b["recall"]:6.1f}% {b["severe"]:6d}', flush=True)


if __name__ == '__main__':
    main()
