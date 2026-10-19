"""
Unit tests for slz_registry, on scripted sequences of detections.

Each test scripts what the vehicle sees cycle after cycle and checks the
lifecycle of S9: a zone needs two distinct passes to be confirmed, keeps
its identity while it is detected, is retired at once when it vanishes
(S9.5), and comes back under its old identity if it reappears.
"""

import numpy as np
from uav_core.slz_detection import label_zones
from uav_core.slz_registry import (CANDIDATE, CONFIRMED, inscribed_radius,
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
    return {'labels': labels, 'cell_ok': ok, 'candidate': cand}


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

    Enter at 10 m, wander up to 17.9 m: one pass. Leave beyond 18 m,
    come back to 15 and 16 m, above r_in = 14.4 m: still one. Cross
    r_in at 14 m: second pass.
    """
    reg = SlzRegistry()
    passes = []
    for t, n in enumerate([10.0, 17.9, 13.0, 18.5, 15.0, 16.0, 14.0]):
        step(reg, landing(ZONE), (n, 0.0), float(t))
        passes.append(reg.active()[0].n_passes)
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
    """A 7 x 7 free square: centre cell, 3.0 m to the refused edge."""
    refused = np.ones((20, 20), dtype=bool)
    refused[5:12, 5:12] = False
    cells = np.argwhere(~refused)
    radius, ij = inscribed_radius(cells, refused, 1.0)
    assert ij == (8, 8)
    assert radius == 3.5
