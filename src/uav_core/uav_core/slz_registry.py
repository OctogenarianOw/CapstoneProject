"""
Dynamic registry of safe landing zones (spec S9), the core of C1.

Role: keep the list of landing zones found online, give each a stable
identity across cycles, and say at every cycle which ones C1 may rely
on. Unlike a predefined return base, the registry grows when zones are
confirmed and shrinks when an observation invalidates one (S9.5).

Lifecycle, criteria of S9:
  CANDIDATE  detected, not usable by C1
  CONFIRMED  inscribed radius >= footprint / 2, one whole footprint
             element of landable cells inside the zone, every cell
             OBSERVED
             (count >= n_min, variance <= var_confirm), and one whole
             footprint of landable cells each observed from at least two
             distinct passes (invariant I4); C1 targets only such cells
  RETIRED    no longer detected; reported at once as an event, so the
             manager re-evaluates I1 at the registry update, not at the
             next frontier selection (S9.5)
A confirmed zone that stops meeting the criteria goes back to CANDIDATE.

Matching between cycles: zones are detected afresh on every map, so each
new zone takes the identity of the previous zone it overlaps most, if
the overlap is at least min_overlap of the smaller of the two. A split
leaves the identity to the larger part, a merge to the zone with the
larger overlap. Without matching, identities change every cycle and the
pass count means nothing.

Choices:
  - retirement is immediate, with no tolerance of missed cycles: a
    tolerance would delay exactly the case S9.5 is about
  - a retired zone detected again within revive_s at the same place
    takes back its identity and its passes, as a CANDIDATE that must meet
    the criteria again. Flicker then does not erase the history of a
    zone, and safety is not weakened, since confirmation is re-checked
  - passes are counted per cell (I4, 22 Oct): a cell is passed when the
    aircraft enters within r_in = 0.8 sensor radius of it after having
    been beyond the sensor radius; the hysteresis keeps a vehicle
    hovering at the edge from counting passes. Counted from the first
    cycle, not from the detection: a zone only appears once its cells
    are well observed, so counting from detection drops exactly the
    passes that built the evidence (a 391 m2 zone of the 19 Oct survey
    had 0 passes after two overflights). Per cell a count only grows,
    which keeps Dorian's decision of 19 Oct that passes flown stay
    flown. A zone's n_passes is the passes of its best-observed
    footprint: the largest, over its cells, of the fewest passes among
    the cells of the footprint centred there. Its landing cells, the
    only ones C1 may target (mission_policy.return_field), are the cells
    whose whole footprint was passed min_passes times. Replaced: passes
    counted around one touchdown point per zone. On the first W3 flight
    the road's point fell 50 m west and was flown near once, so the road
    was never confirmed; over the 60 C1 missions of the W1 campaign,
    two passes near the point confirmed whole zones of which 538 m2 had
    been passed twice, out of 2465 (medians, 2026-10-22_i4_cells)
  - the touchdown point is the zone cell farthest from refused ground,
    not the centroid, which can fall outside a concave zone; once set,
    it moves only if another cell is clearer by more than one cell
    (inscribed_radius). Passes were counted around it until 22 Oct; it
    now gives the inscribed radius and the zone's marker
  - a zone is confirmed only if it holds one whole footprint element
    of landable cells, the footprint test applied to the landable
    cells themselves. Without it, islands of 1 to 8 landable cells
    inside the rough ground of W1 were confirmed in 15 of the 80
    missions of 2026-10-20_campaign_w1_fix, and the return field
    targets the nearest landable cell of any confirmed zone. The map
    smooths 0.5 m steps there, and such islands held the severe
    false accepts of 14 and 17 of the 80 last maps of the two W1
    campaigns; with the rule, 0 and 0, recall unchanged
    (experiments/2026-10-20_min_zone). Price: the smallest isolated
    platform that can be confirmed grows from 5 to 7 m at the 1.5 m
    footprint. Rejected: a minimum count of cells, chosen at 7 on
    the first campaign, which let 2 severe cells through on the
    second, and says nothing about shape (a one-cell-wide strip)
  - the take-off point enters as a provisional CONFIRMED zone (S15.3,
    policy a): the aircraft took off from it, so it can land there
"""

from dataclasses import dataclass, field
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from uav_core.observation import N_MIN, VAR_MAX
from uav_core.slz_detection import footprint_ok

CANDIDATE = 'CANDIDATE'
CONFIRMED = 'CONFIRMED'
RETIRED = 'RETIRED'


@dataclass
class Zone:
    """One landing zone and the evidence behind its state."""

    zone_id: int
    state: str
    cells: np.ndarray = None           # (N, 2) cell indices
    centroid_ne: tuple = None
    touchdown_ij: tuple = None
    touchdown_ne: tuple = None
    area_m2: float = 0.0
    inscribed_radius_m: float = 0.0
    holds_footprint: bool = False      # one whole element of landable
    n_observations: int = 0            # minimum over the cells
    max_variance: float = math.inf     # maximum over the cells
    n_passes: int = 0                  # passes of its best footprint
    landing_cells: np.ndarray = None   # whole footprint passed twice
    provisional: bool = False
    first_seen: float = 0.0
    last_updated: float = 0.0
    retired_at: float = None
    reason: str = ''


@dataclass
class Event:
    """A state transition, for the log and for the S9.5 reaction."""

    zone_id: int
    old: str
    new: str
    reason: str


def refused_distance(refused):
    """
    Euclidean distance, in cells, from every cell centre to refused ground.

    One exact distance transform of the whole grid per cycle (scipy,
    compiled). It replaced a pairwise distance between each zone cell and
    each refused cell nearby, quadratic in zone size: on the 200 m grid
    of 19 Oct, with zones of thousands of cells, that took up to 6.3 s
    per cycle and stretched the decision period to 20 s
    (experiments/2026-10-19_big_area). No refused cell: infinity.
    """
    if not refused.any():
        return np.full(refused.shape, np.inf)
    return distance_transform_edt(~refused)


def inscribed_radius(zone_cells, distance, resolution, window=12,
                     previous_ij=None):
    """
    Largest distance from a zone cell centre to refused ground.

    distance    : refused_distance() of the cycle
    previous_ij : touchdown cell of the zone at the previous cycle
    Returns the radius in metres, from the cell centre to the nearest
    edge of a refused cell, capped at window cells, and the touchdown
    cell. A new zone takes the cell where the uncapped distance is
    largest. A known zone keeps the cell nearest its previous touchdown
    among those within one cell of the best capped distance: in a large
    flat zone hundreds of cells lie beyond the cap, and the farthest one
    moves with every change of the map edge. On 19 Oct the touchdown of
    such a zone moved by up to 64 m between two cycles on W1 and 160 m
    on the 190 m flight, its pass count with it, and the zone fell back
    to CANDIDATE 2 to 5 times per mission; in the W1 campaign each of
    the 31 C1 missions that broke I1 did so within 3 s of such a
    demotion (experiments/2026-10-19_campaign_w1). One cell is the
    resolution the distance is known to. Rejected as the fix of these
    jumps: a pass count that never decreases, which would have hidden
    the cause. It was adopted afterwards for what remains, a growing
    zone whose touchdown genuinely moves (module docstring).
    """
    raw = distance[zone_cells[:, 0], zone_cells[:, 1]]
    capped = np.minimum(raw, float(window))
    best = int(np.argmax(raw))
    if previous_ij is not None:
        near = np.flatnonzero(capped >= capped[best] - 1.0)
        d2 = ((zone_cells[near, 0] - previous_ij[0]) ** 2
              + (zone_cells[near, 1] - previous_ij[1]) ** 2)
        best = int(near[np.argmin(d2)])
    radius = float(capped.max())
    return max(0.0, (radius - 0.5) * resolution), tuple(
        int(v) for v in zone_cells[best])


class PassGrid:
    """Passes per cell of the map: I4 applied to every cell."""

    def __init__(self, shape, resolution, origin_ne, r_in, r_out):
        i, j = np.indices(shape)
        self.north = origin_ne[0] + (i + 0.5) * resolution
        self.east = origin_ne[1] + (j + 0.5) * resolution
        self.r_in, self.r_out = r_in, r_out
        self.inside = np.zeros(shape, dtype=bool)
        self.passes = np.zeros(shape, dtype=np.int32)

    def add(self, vehicle_ne):
        """Count an entry within r_in of each cell left beyond r_out."""
        d = np.hypot(self.north - vehicle_ne[0], self.east - vehicle_ne[1])
        enter = ~self.inside & (d < self.r_in)
        self.passes[enter] += 1
        self.inside = (self.inside | enter) & ~(d > self.r_out)


def footprint_min(values, offsets):
    """
    Per cell, the smallest value over the footprint centred on it.

    Cells outside the grid count as 0, as footprint_ok counts them as
    failures.
    """
    out = values.copy()
    h, w = values.shape
    for di, dj in offsets:
        if di == 0 and dj == 0:
            continue
        shifted = np.zeros_like(values)
        shifted[max(0, di):h - max(0, -di), max(0, dj):w - max(0, -dj)] = \
            values[max(0, -di):h - max(0, di), max(0, -dj):w - max(0, dj)]
        out = np.minimum(out, shifted)
    return out


@dataclass
class SlzRegistry:
    """Zones with stable identities, updated once per cycle."""

    footprint_m: float = 1.5
    var_confirm: float = VAR_MAX
    n_min: int = N_MIN
    min_passes: int = 2
    min_overlap: float = 0.3
    sensor_radius_m: float = 18.0
    pass_hysteresis: float = 0.8
    revive_s: float = 30.0
    zones: dict = field(default_factory=dict)
    next_id: int = 1
    grid: object = None                          # PassGrid, from 1st cycle

    def add_provisional(self, ne, t):
        """Register the take-off point as a provisional confirmed zone."""
        z = Zone(zone_id=self.next_id, state=CONFIRMED,
                 centroid_ne=tuple(ne), touchdown_ne=tuple(ne),
                 provisional=True, first_seen=t, last_updated=t,
                 reason='take-off point, S15.3 policy a')
        self.zones[z.zone_id] = z
        self.next_id += 1
        return z

    def confirmed(self):
        """Zones C1 may rely on, provisional take-off point included."""
        return [z for z in self.zones.values() if z.state == CONFIRMED]

    def active(self):
        """Zones currently detected, provisional excluded."""
        return [z for z in self.zones.values()
                if z.state != RETIRED and not z.provisional]

    def _criteria(self, z):
        if z.inscribed_radius_m < self.footprint_m / 2.0:
            return 'inscribed radius below half the footprint'
        if not z.holds_footprint:
            return 'no whole footprint of landable cells'
        if z.n_observations < self.n_min or z.max_variance > self.var_confirm:
            return 'cells not all OBSERVED'
        if z.n_passes < self.min_passes:
            return f'{z.n_passes} pass, {self.min_passes} needed'
        return ''

    def _match(self, new_masks, t):
        """Map each new zone index to an old zone id, or None."""
        old = {z.zone_id: z for z in self.zones.values()
               if not z.provisional and z.cells is not None
               and (z.state != RETIRED
                    or (z.retired_at is not None
                        and t - z.retired_at <= self.revive_s))}
        old_sets = {zid: set(map(tuple, z.cells)) for zid, z in old.items()}
        pairs = []
        for k, cells in enumerate(new_masks):
            new_set = set(map(tuple, cells))
            for zid, old_set in old_sets.items():
                inter = len(new_set & old_set)
                if inter == 0:
                    continue
                frac = inter / min(len(new_set), len(old_set))
                if frac >= self.min_overlap:
                    alive = old[zid].state != RETIRED
                    pairs.append((alive, inter, k, zid))
        # Live zones before retired ones, larger overlaps first.
        pairs.sort(reverse=True)
        taken_new, taken_old, out = set(), set(), {}
        for _, _, k, zid in pairs:
            if k in taken_new or zid in taken_old:
                continue
            out[k] = zid
            taken_new.add(k)
            taken_old.add(zid)
        return out

    def update(self, landing, variance, count, resolution, origin_ne,
               vehicle_ne, t):
        """
        Fold one cycle of landing detection into the registry.

        landing : output of slz_detection.detect on the current map
        Returns the list of Events of this cycle.
        """
        labels = landing['labels']
        cand = landing['candidate']
        if self.grid is None or self.grid.passes.shape != cand.shape:
            self.grid = PassGrid(cand.shape, resolution, origin_ne,
                                 self.pass_hysteresis * self.sensor_radius_m,
                                 self.sensor_radius_m)
        self.grid.add(vehicle_ne)
        distance = refused_distance(~landing['cell_ok'])
        core = footprint_ok(cand, landing['footprint_offsets'])
        # Passes of the footprint centred on each cell: the fewest among
        # its cells, 0 where it leaves the landable cells
        fp_passes = footprint_min(np.where(cand, self.grid.passes, 0),
                                  landing['footprint_offsets'])
        new_cells = [np.argwhere(labels == k)
                     for k in range(1, int(labels.max()) + 1)]
        new_cells = [c for c in new_cells if len(c)]
        matches = self._match(new_cells, t)
        events = []
        seen_ids = set()

        for k, cells in enumerate(new_cells):
            zid = matches.get(k)
            if zid is None:
                z = Zone(zone_id=self.next_id, state=CANDIDATE,
                         first_seen=t)
                self.zones[z.zone_id] = z
                self.next_id += 1
                events.append(Event(z.zone_id, '', CANDIDATE,
                                    'new detection'))
            else:
                z = self.zones[zid]
                if z.state == RETIRED:
                    events.append(Event(z.zone_id, RETIRED, CANDIDATE,
                                        'detected again'))
                    z.state, z.retired_at = CANDIDATE, None
            seen_ids.add(z.zone_id)
            self._measure(z, cells, distance, variance, count, resolution,
                          origin_ne, t)
            z.holds_footprint = bool(core[cells[:, 0], cells[:, 1]].any())
            fp = fp_passes[cells[:, 0], cells[:, 1]]
            z.n_passes = int(fp.max())
            z.landing_cells = cells[fp >= self.min_passes]

        for z in self.zones.values():
            if (z.provisional or z.state == RETIRED
                    or z.zone_id in seen_ids):
                continue
            events.append(Event(z.zone_id, z.state, RETIRED,
                                'no longer detected'))
            z.state, z.retired_at, z.reason = RETIRED, t, 'no longer detected'

        for z in self.active():
            missing = self._criteria(z)
            if z.state == CANDIDATE and not missing:
                events.append(Event(z.zone_id, CANDIDATE, CONFIRMED,
                                    'criteria met'))
                z.state = CONFIRMED
            elif z.state == CONFIRMED and missing:
                events.append(Event(z.zone_id, CONFIRMED, CANDIDATE,
                                    missing))
                z.state = CANDIDATE
            z.reason = missing
        return events

    def _measure(self, z, cells, distance, variance, count, resolution,
                 origin_ne, t):
        z.cells = cells
        z.area_m2 = len(cells) * resolution ** 2
        centres = np.asarray(origin_ne) + (cells + 0.5) * resolution
        z.centroid_ne = (float(centres[:, 0].mean()),
                         float(centres[:, 1].mean()))
        z.inscribed_radius_m, z.touchdown_ij = inscribed_radius(
            cells, distance, resolution, previous_ij=z.touchdown_ij)
        z.touchdown_ne = (origin_ne[0] + (z.touchdown_ij[0] + 0.5)
                          * resolution,
                          origin_ne[1] + (z.touchdown_ij[1] + 0.5)
                          * resolution)
        z.n_observations = int(count[cells[:, 0], cells[:, 1]].min())
        z.max_variance = float(variance[cells[:, 0], cells[:, 1]].max())
        z.last_updated = t
