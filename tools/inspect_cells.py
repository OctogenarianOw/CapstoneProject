"""Print a neighbourhood of the map cell by cell: truth, height, error,
observation count, variance, and the gain the next observation would get.

The gain is the number that matters. A cell whose gain has collapsed can
no longer be corrected by any observation, however good: it is frozen on
whatever surface it saw first. A cell with a healthy gain that still holds
a wrong height is averaging two surfaces instead, which is a different
failure needing a different fix.

    python3 tools/inspect_cells.py data_survey_v4.npz --north 8.5 --east 26.5
"""

import argparse

import numpy as np

DEFAULT_TRUTH = '/home/doria/ros2_ws/droneAfter/worlds/validation_truth.npz'


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('map_file')
    p.add_argument('--truth', default=DEFAULT_TRUTH)
    p.add_argument('--north', type=float, required=True)
    p.add_argument('--east', type=float, required=True)
    p.add_argument('--radius', type=int, default=2)
    p.add_argument('--sigma', type=float, default=0.02,
                   help='measurement standard deviation used by cloud_world')
    p.add_argument('--truth-layer', default='truth')
    return p.parse_args()


def grid_block(grid, i, j, radius, fmt):
    """Rows printed north up, columns east increasing."""
    lo_i, hi_i = max(0, i - radius), min(grid.shape[0] - 1, i + radius)
    lo_j, hi_j = max(0, j - radius), min(grid.shape[1] - 1, j + radius)
    lines = []
    for r in range(hi_i, lo_i - 1, -1):
        lines.append('    ' + ''.join(
            f'{grid[r, c]:{fmt}}' for c in range(lo_j, hi_j + 1)))
    return '\n'.join(lines)


def main():
    args = parse_args()

    m = np.load(args.map_file)
    res = float(m['resolution'])
    ox, oy = float(m['origin_x']), float(m['origin_y'])
    elev = m['elevation']
    count = m['count']
    variance = m['variance'] if 'variance' in m.files else None

    t = np.load(args.truth)
    layer = args.truth_layer if args.truth_layer in t.files else 'truth'
    truth_ned = -t[layer].T
    n_t = truth_ned.shape[0]
    half_t = n_t * res / 2.0

    # Index of the requested cell in each grid.
    i_map = int(round((args.north - ox) / res))
    j_map = int(round((args.east - oy) / res))
    i_tru = int(round((args.north + half_t - res / 2.0) / res))
    j_tru = int(round((args.east + half_t - res / 2.0) / res))

    i0 = int(round((-half_t + res / 2.0 - ox) / res))
    j0 = int(round((-half_t + res / 2.0 - oy) / res))
    sub_elev = elev[i0:i0 + n_t, j0:j0 + n_t]
    known = count[i0:i0 + n_t, j0:j0 + n_t] > 0
    flat_like = known & (np.abs(truth_ned) < 1e-6)
    offset = float(np.median(sub_elev[flat_like])) if flat_like.any() else 0.0

    print(f'map    {args.map_file}')
    print(f'truth  {args.truth} [{layer}]')
    print(f'centre N {args.north:+.1f} E {args.east:+.1f}, '
          f'radius {args.radius} cells')
    print(f'map offset {offset:+.3f} m removed, from '
          f'{int(flat_like.sum())} flat cells')
    print()

    print('  truth:')
    print(grid_block(truth_ned, i_tru, j_tru, args.radius, '>9.2f'))
    print('  map:')
    print(grid_block(elev - offset, i_map, j_map, args.radius, '>9.2f'))
    print('  error (map minus truth):')
    print(grid_block((elev - offset)[i0:i0 + n_t, j0:j0 + n_t] - truth_ned,
                     i_tru, j_tru, args.radius, '>9.2f'))
    print('  observations:')
    print(grid_block(count, i_map, j_map, args.radius, '>9.0f'))

    if variance is None:
        print()
        print('no variance array in this map file. cloud_world saves only '
              'elevation and count, so add variance to the npz: the gate in '
              'classify_cells needs it too.')
        return

    print('  variance (m2):')
    print(grid_block(variance, i_map, j_map, args.radius, '>9.5f'))

    # Scalar Kalman gain the next observation would receive:
    #   k = p / (p + r),  r = sigma squared
    r = args.sigma ** 2
    gain = variance / (variance + r)
    print(f'  gain for the next observation (sigma {args.sigma} m):')
    print(grid_block(gain, i_map, j_map, args.radius, '>9.4f'))

    centre_gain = float(gain[i_map, j_map])
    shift = centre_gain * 1.0
    print()
    print(f'centre cell: gain {centre_gain:.4f}, so a measurement 1.00 m '
          f'away from the current estimate moves it by {shift:.3f} m')
    if centre_gain < 0.01:
        print('that cell is frozen: no observation can correct it any more')


if __name__ == '__main__':
    main()