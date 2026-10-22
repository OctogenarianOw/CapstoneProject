"""
Unit tests for slz_registry, on scripted sequences of detections.

Each test scripts what the vehicle sees cycle after cycle and checks the
lifecycle of S9: a zone needs two distinct passes to be confirmed, keeps
its identity while it is detected, is retired at once when it vanishes
(S9.5), and comes back under its old identity if it reappears.
"""

import numpy as np
import pytest
from uav_core.slz_detection import footprint_offsets, label_zones
from uav_core.slz_registry import (CANDIDATE, CONFIRMED, inscribed_radius,
                                   PassGrid, refused_distance,
                                   RETIRED, SlzRegistry)

N = 60
RES = 1.0
ORIGIN = (-30.0, -30.0)
GOOD_COUNT = np.full((N, N), 100, dtype=np.int32)
GOOD_VAR = np.full((N, N), 4e-4, dtype=np.float32)


def landing(*boxes):
    """Return a detect() output with one zone per box (i0, i1, j0, j1)."""
    cand = np.zeros((N, N), dtype=bool)
    for i0, i1, j0, j1 in boxes:
        cand[i0:i1, j0:j1] = True
    labels, _ = label_zones(cand)
    ok = np.zeros((N, N), dtype=bool)
    for i0, i1, j0, j1 in boxes:
        ok[max(0, i0 - 1):i1 + 1, max(0, j0 - 1):j1 + 1] = True
    return {'labels': labels, 'cell_ok': ok, 'candidate': cand,
            'footprint_offsets': footprint_offsets(1.5, RES)}


def step(reg, det, vehicle, t, var=GOOD_VAR):
    """Run one registry update and return its events."""
    return reg.update(det, var, GOOD_COUNT, RES, ORIGIN, vehicle, t)


ZONE = (28, 34, 28, 34)          # 6 x 6 cells around the origin
NEAR, FAR = (0.0, 0.0), (40.0, 0.0)


def test_two_distinct_passes_confirm_a_zone():
    """In range, out of range, in range again: CONFIRMED on the second."""
    reg = SlzRegistry()
    ev = step(reg, landing(ZONE), NEAR, 0.0)
    assert [(e.new, e.reason) for e in ev] == [(CANDIDATE, 'new detection')]
    (z,) = reg.active()
    assert z.state == CANDIDATE and z.n_passes == 1
    step(reg, landing(ZONE), NEAR, 1.0)
    assert z.n_passes == 1
    step(reg, landing(ZONE), FAR, 2.0)
    ev = step(reg, landing(ZONE), NEAR, 3.0)
    assert z.n_passes == 2 and z.state == CONFIRMED
    assert [(e.old, e.new) for e in ev] == [(CANDIDATE, CONFIRMED)]
    assert reg.confirmed() == [z]


def test_hysteresis_does_not_count_edge_hovering():
    """
    Count a pass only on entry within r_in after leaving the radius.

    One cell at the origin. Enter at 10 m, wander up to 17.9 m: one
    pass. Leave beyond 18 m, come back to 15 and 16 m, above r_in =
    14.4 m: still one. Cross r_in at 14 m: second pass.
    """
    grid = PassGrid((1, 1), 1.0, (-0.5, -0.5), 14.4, 18.0)
    passes = []
    for n in [10.0, 17.9, 13.0, 18.5, 15.0, 16.0, 14.0]:
        grid.add((n, 0.0))
        passes.append(int(grid.passes[0, 0]))
    assert passes == [1, 1, 1, 1, 1, 1, 2]


def test_passes_flown_before_detection_count():
    """Two passes before the zone appears: confirmed when it appears."""
    reg = SlzRegistry()
    step(reg, landing(), NEAR, 0.0)
    step(reg, landing(), FAR, 1.0)
    ev = step(reg, landing(ZONE), NEAR, 2.0)
    expected = [('', CANDIDATE), (CANDIDATE, CONFIRMED)]
    assert [(e.old, e.new) for e in ev] == expected
    assert reg.active()[0].n_passes == 2


def test_high_variance_blocks_confirmation():
    """Two passes but cells not all OBSERVED: stays CANDIDATE."""
    reg = SlzRegistry()
    var = GOOD_VAR.copy()
    var[30, 30] = 1e-2
    for t, v in enumerate([NEAR, FAR, NEAR]):
        step(reg, landing(ZONE), v, float(t), var)
    z = reg.active()[0]
    assert z.n_passes == 2 and z.state == CANDIDATE
    assert z.reason == 'cells not all OBSERVED'


def test_vanishing_confirmed_zone_is_retired_at_once():
    """S9.5: the update that loses the zone reports CONFIRMED -> RETIRED."""
    reg = SlzRegistry()
    for t, v in enumerate([NEAR, FAR, NEAR]):
        step(reg, landing(ZONE), v, float(t))
    zid = reg.confirmed()[0].zone_id
    ev = step(reg, landing(), NEAR, 3.0)
    assert [(e.zone_id, e.old, e.new) for e in ev] == [(zid, CONFIRMED,
                                                        RETIRED)]
    assert reg.confirmed() == []


def test_reappearing_zone_keeps_identity_and_passes():
    """A flicker revives the old identity, criteria checked again."""
    reg = SlzRegistry()
    for t, v in enumerate([NEAR, FAR, NEAR]):
        step(reg, landing(ZONE), v, float(t))
    zid = reg.confirmed()[0].zone_id
    step(reg, landing(), NEAR, 3.0)
    ev = step(reg, landing(ZONE), NEAR, 4.0)
    z = reg.zones[zid]
    expected = [(RETIRED, CANDIDATE), (CANDIDATE, CONFIRMED)]
    assert [(e.old, e.new) for e in ev] == expected
    assert z.n_passes == 2 and len(reg.zones) == 1


def test_part_of_a_zone_seen_once_is_not_targeted():
    """
    I4 per cell: C1 targets only the part of a zone passed twice.

    A zone grown where it was seen once stays confirmed on the part
    passed twice. Confirmed with two passes, the zone grows a wide part east, out to
    28 m, and its touchdown moves there; the aircraft at 19.5 m east
    passes that part once. The zone keeps its two passes and stays
    CONFIRMED, with landing cells within 14.4 m of where it flew twice.
    The other criteria still apply: a cell short of OBSERVED demotes it.
    """
    reg = SlzRegistry()
    for t, v in enumerate([NEAR, FAR, NEAR]):
        step(reg, landing(ZONE), v, float(t))
    (z,) = reg.confirmed()
    first = z.touchdown_ne
    grown = landing(ZONE, (28, 34, 34, 40), (16, 44, 40, 58))
    ev = step(reg, grown, (0.0, 19.5), 3.0)
    assert z.touchdown_ne[1] - first[1] > 10.0
    assert z.n_passes == 2 and z.state == CONFIRMED and ev == []
    east = ORIGIN[1] + z.landing_cells[:, 1] + 0.5
    assert 0 < len(z.landing_cells) < len(z.cells) and east.max() < 14.4
    var = GOOD_VAR.copy()
    var[30, 30] = 1e-2
    step(reg, grown, (0.0, 19.5), 4.0, var)
    assert z.state == CANDIDATE and z.reason == 'cells not all OBSERVED'


def test_long_zone_is_confirmed_where_it_was_passed_twice():
    """
    The road of the first W3 flight: a strip flown near twice at one end.

    A 6 x 50 cell strip, east -25 to +25 m; the aircraft comes within
    reach of its west end, leaves, comes back. Wherever its touchdown
    falls along it, the strip is confirmed, and its landing cells are
    the west end only, where every cell of a footprint was passed twice.
    """
    reg = SlzRegistry()
    strip = (27, 33, 5, 55)
    for t, v in enumerate([(0.0, -25.0), (0.0, -60.0), (0.0, -25.0)]):
        step(reg, landing(strip), v, float(t))
    (z,) = reg.active()
    assert z.state == CONFIRMED and z.n_passes == 2
    east = ORIGIN[1] + z.landing_cells[:, 1] + 0.5
    assert east.min() < -20.0 and east.max() < -10.0


def test_island_without_a_whole_footprint_is_never_confirmed():
    """
    A 2 x 3 island, passes and OBSERVED cells: stays CANDIDATE.

    No cell of it has its whole 3 x 3 footprint in landable cells, the
    case of the rough ground of W1 (2026-10-20_min_zone). A 3 x 3
    island next to it, the smallest that holds one, is confirmed.
    """
    reg = SlzRegistry()
    island, block = (30, 32, 30, 33), (30, 33, 40, 43)
    for t, v in enumerate([NEAR, FAR, NEAR]):
        step(reg, landing(island, block), v, float(t))
    by_size = {len(z.cells): z for z in reg.active()}
    assert by_size[6].state == CANDIDATE
    assert by_size[6].reason == 'no whole footprint of landable cells'
    assert by_size[9].state == CONFIRMED


def test_split_leaves_identity_to_the_larger_part():
    """A zone cut in two: the larger part keeps the id, the other is new."""
    reg = SlzRegistry()
    step(reg, landing((20, 40, 28, 34)), NEAR, 0.0)
    zid = reg.active()[0].zone_id
    step(reg, landing((20, 33, 28, 34), (35, 40, 28, 34)), NEAR, 1.0)
    sizes = {z.zone_id: len(z.cells) for z in reg.active()}
    assert sizes[zid] == 13 * 6
    assert len(sizes) == 2


def test_take_off_point_is_a_provisional_confirmed_zone():
    """S15.3 policy a: usable at once, never retired by detection."""
    reg = SlzRegistry()
    home = reg.add_provisional((0.0, 0.0), 0.0)
    step(reg, landing(), NEAR, 1.0)
    assert reg.confirmed() == [home] and home.provisional


def test_inscribed_radius_and_touchdown_point():
    """A 7 x 7 free square: centre cell, 3.5 m to the refused edge."""
    refused = np.ones((20, 20), dtype=bool)
    refused[5:12, 5:12] = False
    cells = np.argwhere(~refused)
    radius, ij = inscribed_radius(cells, refused_distance(refused), 1.0)
    assert ij == (8, 8)
    assert radius == 3.5


def test_touchdown_of_a_large_zone_stays_put_when_the_zone_grows():
    """
    Beyond the cap, a known zone keeps the touchdown nearest its last one.

    A 60 x 60 free square has hundreds of cells past the 12-cell cap.
    Grown to 80 x 80, its farthest cell moves 10 cells along each axis;
    the touchdown of a known zone does not, a new zone takes the centre.
    """
    refused = np.ones((100, 100), dtype=bool)
    refused[10:70, 10:70] = False
    cells = np.argwhere(~refused)
    radius, first = inscribed_radius(cells, refused_distance(refused), 1.0)
    assert first == (39, 39) and radius == 11.5
    refused[10:90, 10:90] = False
    grown = np.argwhere(~refused)
    _, fresh = inscribed_radius(grown, refused_distance(refused), 1.0)
    _, kept = inscribed_radius(grown, refused_distance(refused), 1.0,
                               previous_ij=first)
    assert fresh == (49, 49)
    assert kept == first


def pairwise_reference(zone_cells, refused, resolution, window=12):
    """Return the previous pairwise implementation, the reference."""
    i0, j0 = zone_cells.min(axis=0) - window
    i1, j1 = zone_cells.max(axis=0) + window + 1
    h, w = refused.shape
    sub = refused[max(0, i0):min(h, i1), max(0, j0):min(w, j1)]
    ri, rj = np.nonzero(sub)
    ri, rj = ri + max(0, i0), rj + max(0, j0)
    if ri.size == 0:
        return (window - 0.5) * resolution, None, np.inf
    d2 = ((zone_cells[:, 0, None] - ri[None, :]) ** 2
          + (zone_cells[:, 1, None] - rj[None, :]) ** 2)
    nearest = np.sqrt(d2.min(axis=1))
    best = int(np.argmax(nearest))
    radius = min(float(nearest[best]), float(window))
    return (max(0.0, (radius - 0.5) * resolution),
            tuple(int(v) for v in zone_cells[best]), float(nearest[best]))


def test_distance_transform_matches_the_pairwise_reference():
    """
    Same radius on 200 random masks, same touchdown below the cap.

    Above the 12-cell cap the reference only saw refused cells within
    the window, so its touchdown among far-from-refused cells could
    differ; the radius, capped, is equal in every case.
    """
    rng = np.random.default_rng(1)
    for _ in range(200):
        refused = rng.random((40, 40)) < rng.uniform(0.02, 0.4)
        free = np.argwhere(~refused)
        if free.size == 0:
            continue
        zone_cells = free[rng.random(len(free)) < 0.5]
        if zone_cells.size == 0:
            continue
        r_new, ij_new = inscribed_radius(zone_cells,
                                         refused_distance(refused), 1.0)
        r_ref, ij_ref, uncapped = pairwise_reference(zone_cells, refused,
                                                     1.0)
        assert r_new == pytest.approx(r_ref)
        if uncapped <= 12:
            assert ij_new == ij_ref
