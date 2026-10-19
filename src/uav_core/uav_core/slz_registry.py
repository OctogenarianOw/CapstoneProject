"""
Dynamic registry of safe landing zones (spec S9), the core of C1.

Role: keep the list of landing zones found online, give each a stable
identity across cycles, and say at every cycle which ones C1 may rely
on. Unlike a predefined return base, the registry grows when zones are
confirmed and shrinks when an observation invalidates one (S9.5).

Lifecycle, criteria of S9:
  CANDIDATE  detected, not usable by C1
  CONFIRMED  inscribed radius >= footprint / 2, every cell OBSERVED
             (count >= n_min, variance <= var_confirm), and observed
             from at least two distinct passes (invariant I4)
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
  - a pass is an entry of the vehicle within r_in = 0.8 sensor radius of
    the touchdown point after having been beyond the sensor radius. The
    hysteresis keeps a vehicle hovering at the edge from counting passes.
    Passes are counted over the whole trajectory history, recomputed at
    every cycle, not from the detection onwards. Replaying the survey of
    19 Oct with the latter, the 391 m2 zone first detected at 67 s had 0
    passes although the aircraft had flown over it twice: a zone only
    appears once its cells are well observed, so counting from detection
    drops exactly the passes that built the evidence
  - the touchdown point is the zone cell farthest from refused ground,
    not the centroid, which can fall outside a concave zone
  - the take-off point enters as a provisional CONFIRMED zone (S15.3,
    policy a): the aircraft took off from it, so it can land there
"""

from dataclasses import dataclass, field
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from uav_core.observation import N_MIN, VAR_MAX

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
    n_observations: int = 0            # minimum over the cells
    max_variance: float = math.inf     # maximum over the cells
    n_passes: int = 0
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


def inscribed_radius(zone_cells, distance, resolution, window=12):
    """
    Largest distance from a zone cell centre to refused ground.

    distance : refused_distance() of the cycle
    Returns the radius in metres, from the cell centre to the nearest
    edge of a refused cell, capped at window cells, and the cell where
    the uncapped distance is largest, the touchdown point.
    """
    d = distance[zone_cells[:, 0], zone_cells[:, 1]]
    best = int(np.argmax(d))
    radius = min(float(d[best]), float(window))
    return max(0.0, (radius - 0.5) * resolution), tuple(
        int(v) for v in zone_cells[best])


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
    track: list = field(default_factory=list)   # vehicle (n, e), 1 per cycle

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
        self.track.append((float(vehicle_ne[0]), float(vehicle_ne[1])))
        labels = landing['labels']
        distance = refused_distance(~landing['cell_ok'])
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

        for z in self.zones.values():
            if (z.provisional or z.state == RETIRED
                    or z.zone_id in seen_ids):
                continue
            events.append(Event(z.zone_id, z.state, RETIRED,
                                'no longer detected'))
            z.state, z.retired_at, z.reason = RETIRED, t, 'no longer detected'

        track = np.asarray(self.track)
        for z in self.active():
            z.n_passes = self.count_passes(track, z.touchdown_ne)
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
            cells, distance, resolution)
        z.touchdown_ne = (origin_ne[0] + (z.touchdown_ij[0] + 0.5)
                          * resolution,
                          origin_ne[1] + (z.touchdown_ij[1] + 0.5)
                          * resolution)
        z.n_observations = int(count[cells[:, 0], cells[:, 1]].min())
        z.max_variance = float(variance[cells[:, 0], cells[:, 1]].max())
        z.last_updated = t

    def count_passes(self, track, point_ne):
        """Count entries within r_in after leaving r_out, over a track."""
        if len(track) == 0:
            return 0
        d = np.hypot(track[:, 0] - point_ne[0], track[:, 1] - point_ne[1])
        r_in = self.pass_hysteresis * self.sensor_radius_m
        inside, passes = False, 0
        for dk in d:
            if not inside and dk < r_in:
                inside, passes = True, passes + 1
            elif inside and dk > self.sensor_radius_m:
                inside = False
        return passes
