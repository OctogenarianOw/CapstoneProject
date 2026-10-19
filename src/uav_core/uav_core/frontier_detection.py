"""
Frontier catalogue on the elevation map (spec S11), input of the manager.

Role: list the places worth flying to, each with what C1 and C2 need,
computed on the map and never on the raw cloud. The catalogue makes no
choice: the exploration manager alone ranks and decides (spec 3.3), so
C1 and C2 can be switched off cleanly for the baselines.

Pipeline, adapted from the frontier explorer of the 49274 project:
  1. frontier cells: seen cells (observation.SPARSE or OBSERVED) with an
     UNKNOWN 8-neighbour inside the exploration area
  2. clusters: 8-connected groups, the small ones are noise; a cluster
     wider than max_piece_m is cut into pieces on a max_piece_m grid
  3. one viewpoint per cluster: the reachable OBSERVED cell within
     viewpoint_radius_m of the cluster that is closest to its centroid
  4. information gain: UNKNOWN cells of the area within the sensor
     footprint radius of the viewpoint, by counting (S11.5)
  5. path distance from the vehicle, read off the planner's field

Changes for an aerial vehicle, each a choice:
  - seen, not OBSERVED, on the known side of a frontier: on map v6 the
    S11 wording found 53 of 404 boundary cells (see observation.py)
  - large clusters cut into pieces. A downward sensor sees a disc, so
    the first frontier is a whole circle: on v6 cut to the take-off
    footprint it was one cluster of 140 cells whose centroid was the
    take-off point itself, leaving one arbitrary goal. Indoor frontiers
    of the 49274 project are short segments and never met this case.
    Pieces of at most max_piece_m keep their centroid on the frontier
    and give C1 and C2 a choice of direction
  - no heading penalty: a multirotor is holonomic, and its turning cost
    is the per-leg energy of energy_model.py, counted where paths are
  - no score and no hysteresis bonus here. The bonus kept a ground robot
    from oscillating under continuous reselection; the manager selects
    only on arrival, and a bonus would also alter U and confound the
    alpha study of C2
  - the low-density filter for water (S11.4) is left out: the simulated
    worlds hold no water, so it could not be tested

Blacklist (S11): a frontier visited without its state changing is set
aside 30 s, then 300 s once it has failed three times. Without it the
old stack cycled between two or three points for 65 % of its flight.
"""

from dataclasses import dataclass, field
import math

import numpy as np

from uav_core.observation import classify, grow8, OBSERVED, UNKNOWN
from uav_core.slz_detection import label_zones


@dataclass
class Frontier:
    """One frontier cluster and what the manager needs to judge it."""

    cluster: int                 # label in this cycle, not stable
    cells: np.ndarray            # (N, 2) cell indices (i north, j east)
    size: int                    # number of frontier cells
    centroid_ne: tuple           # metres, NED north and east
    viewpoint_ij: tuple = None   # cell the vehicle flies to
    viewpoint_ne: tuple = None
    gain: int = 0                # UNKNOWN cells in the footprint
    path_dist_m: float = math.inf
    rejected: str = ''           # reason, empty when kept


@dataclass
class Catalogue:
    """All frontiers of one cycle, kept and rejected, plus the masks."""

    frontiers: list              # kept, in cluster order
    rejected: list               # with a reason, for display and logs
    frontier_mask: np.ndarray
    state: np.ndarray
    unknown_in_area: int         # 0 means exploration is complete


@dataclass
class Blacklist:
    """Escalating exclusion of frontiers whose visit changed nothing."""

    radius_m: float = 3.0
    first_s: float = 30.0
    long_s: float = 300.0
    long_after: int = 3
    entries: list = field(default_factory=list)  # [n, e, failures, until]

    def _find(self, ne):
        for entry in self.entries:
            if math.hypot(entry[0] - ne[0], entry[1] - ne[1]) < self.radius_m:
                return entry
        return None

    def record_failure(self, ne, now):
        """Register a visit that left the frontier in place."""
        entry = self._find(ne)
        if entry is None:
            entry = [ne[0], ne[1], 0, now]
            self.entries.append(entry)
        entry[2] += 1
        hold = self.long_s if entry[2] >= self.long_after else self.first_s
        entry[3] = now + hold
        return hold

    def blocked(self, ne, now):
        """Tell whether a viewpoint is excluded at time now."""
        entry = self._find(ne)
        return entry is not None and now < entry[3]


def disc_offsets(radius_cells):
    """Cell offsets whose centre lies within radius_cells of the origin."""
    r = int(math.floor(radius_cells))
    di, dj = np.mgrid[-r:r + 1, -r:r + 1]
    keep = di * di + dj * dj <= radius_cells * radius_cells
    return di[keep], dj[keep]


def count_in_disc(mask, i, j, offsets):
    """Count true cells of mask in a disc, cells off the grid count 0."""
    di, dj = offsets
    ii, jj = i + di, j + dj
    ok = (ii >= 0) & (ii < mask.shape[0]) & (jj >= 0) & (jj < mask.shape[1])
    return int(mask[ii[ok], jj[ok]].sum())


def split_clusters(labels, n_clusters, resolution, max_piece_m):
    """
    Cut clusters wider than max_piece_m into pieces on a square grid.

    A piece is the part of a cluster inside one grid square. Clusters
    that already fit in max_piece_m along both axes are left whole.
    Returns new labels and their count.
    """
    out = np.zeros_like(labels)
    tile = max(1, int(round(max_piece_m / resolution)))
    n_out = 0
    for k in range(1, n_clusters + 1):
        ci, cj = np.nonzero(labels == k)
        if (ci.max() - ci.min() + 1 <= tile
                and cj.max() - cj.min() + 1 <= tile):
            n_out += 1
            out[ci, cj] = n_out
            continue
        keys = (ci // tile) * 100003 + (cj // tile)
        for key in np.unique(keys):
            sel = keys == key
            n_out += 1
            out[ci[sel], cj[sel]] = n_out
    return out, n_out


def detect_frontiers(count, variance, resolution, origin_ne, vehicle_ne,
                     traversable=None, dist_field=None, explore_mask=None,
                     blacklist=None, now=0.0, min_cluster_cells=3,
                     viewpoint_radius_m=5.0, sensor_radius_m=18.0,
                     min_goal_dist_m=2.0, max_piece_m=10.0, n_min=None,
                     var_max=None):
    """
    Build the frontier catalogue of one cycle.

    count, variance : fused map layers, indexed [i north, j east]
    origin_ne       : NED north and east of the corner of cell (0, 0)
    traversable     : cells the planner may fly over; default OBSERVED
    dist_field      : path distance in metres from the vehicle, inf when
                      unreachable; default straight line over traversable
    explore_mask    : area of interest; default the whole grid
    sensor_radius_m : footprint radius of the downward lidar at cruise,
                      sqrt(20^2 - 8^2) = 18.3 m at 8 m with 20 m range
    """
    kwargs = {}
    if n_min is not None:
        kwargs['n_min'] = n_min
    if var_max is not None:
        kwargs['var_max'] = var_max
    state = classify(count, variance, **kwargs)
    h, w = state.shape
    if explore_mask is None:
        explore_mask = np.ones((h, w), dtype=bool)
    unknown = (state == UNKNOWN) & explore_mask
    seen = state != UNKNOWN
    frontier_mask = seen & grow8(unknown)

    ii, jj = np.indices((h, w))
    centres_n = origin_ne[0] + (ii + 0.5) * resolution
    centres_e = origin_ne[1] + (jj + 0.5) * resolution
    if traversable is None:
        traversable = state == OBSERVED
    if dist_field is None:
        dist_field = np.where(traversable,
                              np.hypot(centres_n - vehicle_ne[0],
                                       centres_e - vehicle_ne[1]), np.inf)
    candidates = traversable & (state == OBSERVED) & np.isfinite(dist_field)

    labels, n_clusters = label_zones(frontier_mask)
    labels, n_clusters = split_clusters(labels, n_clusters, resolution,
                                        max_piece_m)
    vp_reach = viewpoint_radius_m / resolution
    vp_offsets = disc_offsets(vp_reach)
    gain_offsets = disc_offsets(sensor_radius_m / resolution)
    kept, rejected = [], []
    for k in range(1, n_clusters + 1):
        ci, cj = np.nonzero(labels == k)
        f = Frontier(cluster=k, cells=np.stack([ci, cj], axis=1),
                     size=len(ci),
                     centroid_ne=(float(centres_n[ci, cj].mean()),
                                  float(centres_e[ci, cj].mean())))
        if f.size < min_cluster_cells:
            continue
        near = np.zeros((h, w), dtype=bool)
        di, dj = vp_offsets
        for a, b in zip(ci, cj):
            pi, pj = a + di, b + dj
            ok = (pi >= 0) & (pi < h) & (pj >= 0) & (pj < w)
            near[pi[ok], pj[ok]] = True
        near &= candidates
        if not near.any():
            f.rejected = 'unreachable'
            rejected.append(f)
            continue
        vi, vj = np.nonzero(near)
        d2 = ((centres_n[vi, vj] - f.centroid_ne[0]) ** 2
              + (centres_e[vi, vj] - f.centroid_ne[1]) ** 2)
        best = int(np.argmin(d2))
        f.viewpoint_ij = (int(vi[best]), int(vj[best]))
        f.viewpoint_ne = (float(centres_n[f.viewpoint_ij]),
                          float(centres_e[f.viewpoint_ij]))
        f.path_dist_m = float(dist_field[f.viewpoint_ij])
        f.gain = count_in_disc(unknown, *f.viewpoint_ij, gain_offsets)
        if math.hypot(f.viewpoint_ne[0] - vehicle_ne[0],
                      f.viewpoint_ne[1] - vehicle_ne[1]) < min_goal_dist_m:
            f.rejected = 'too_close'
        elif blacklist is not None and blacklist.blocked(f.viewpoint_ne,
                                                         now):
            f.rejected = 'blacklisted'
        elif f.gain == 0:
            f.rejected = 'no_gain'
        (rejected if f.rejected else kept).append(f)

    return Catalogue(frontiers=kept, rejected=rejected,
                     frontier_mask=frontier_mask, state=state,
                     unknown_in_area=int(unknown.sum()))
