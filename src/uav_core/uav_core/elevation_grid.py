"""
2.5D elevation grid fused cell by cell (spec S5), the map everything uses.

Role: turn a stream of noisy lidar points, already in NED, into one
elevation estimate per cell with its variance and observation count.

Choice: a scalar Kalman filter per cell, k = p / (p + r), with a floor
on the variance. Without the floor the variance decays as 1/n without
limit: after a few hundred observations the gain falls under 0.002 and
the cell is frozen on its value, right or wrong. A cell under the landed
aircraft had gathered 120 000 observations with an absurd confidence,
and frozen cells drew 4 m deep ghost trenches at ramp edges. No later
observation, even close and from a better angle, could correct them.

Rejected: a Mahalanobis gate on outliers at discontinuities, written
then removed (spec 6.3): severe false accepts went from 26 to 138, as
the gate locked a cell onto the first surface it saw. One Gaussian per
cell cannot hold two surfaces (P2); a fix needs two hypotheses per cell.

Pending (S5): a process noise term, growing the variance with time
since the last observation, is the principled form of the floor and is
to be compared against it.
"""

import numpy as np


class ElevationGrid:
    """
    Elevation, variance and count per cell, NED frame, z down.

    Cell (i, j) covers north [origin_x + i res, origin_x + (i + 1) res)
    and east [origin_y + j res, origin_y + (j + 1) res).
    """

    def __init__(self, size_m=100.0, resolution=1.0, origin_xy=(0.0, 0.0),
                 min_sigma_m=0.02):
        self.resolution = resolution
        self.n = int(size_m / resolution)
        self.origin_x = origin_xy[0] - size_m / 2.0
        self.origin_y = origin_xy[1] - size_m / 2.0

        # Smallest uncertainty the map allows itself to claim.
        self.min_variance = float(min_sigma_m) ** 2

        self.elevation = np.full((self.n, self.n), np.nan, dtype=np.float32)
        self.variance = np.full((self.n, self.n), np.inf, dtype=np.float32)
        self.count = np.zeros((self.n, self.n), dtype=np.int32)

    def to_index(self, x, y):
        """Map NED north and east, numpy arrays, to cell indices."""
        ix = ((x - self.origin_x) / self.resolution).astype(np.int32)
        iy = ((y - self.origin_y) / self.resolution).astype(np.int32)
        return ix, iy

    def update(self, points, variances):
        """
        Fuse points into the grid, return how many fell inside it.

        points    : (N, 3) array in NED
        variances : (N,) array or scalar, in m^2
        """
        if points.shape[0] == 0:
            return 0

        variances = np.asarray(variances, dtype=np.float32)
        if variances.ndim == 0:
            variances = np.full(points.shape[0], float(variances),
                                dtype=np.float32)

        ix, iy = self.to_index(points[:, 0], points[:, 1])

        inside = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        ix, iy = ix[inside], iy[inside]
        z = points[inside, 2]
        r = variances[inside]
        if z.size == 0:
            return 0

        floor = self.min_variance

        for i, j, zm, rm in zip(ix, iy, z, r):
            if self.count[i, j] == 0:
                self.elevation[i, j] = zm
                self.variance[i, j] = max(rm, floor)
            else:
                p = self.variance[i, j]
                k = p / (p + rm)
                self.elevation[i, j] += k * (zm - self.elevation[i, j])
                self.variance[i, j] = max((1.0 - k) * p, floor)
            self.count[i, j] += 1

        return z.size

    def known_cells(self):
        """Return the number of cells observed at least once."""
        return int(np.count_nonzero(self.count))
