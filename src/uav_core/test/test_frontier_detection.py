"""
Unit tests for observation and frontier_detection, on synthetic maps.

Each test is a property S11 requires or a choice the module header
makes: a frontier exists while reachable unknown ground remains, the
catalogue is empty at full coverage, a SPARSE ring does not hide the
boundary, and the blacklist escalates as specified.
"""

import numpy as np
import pytest
from uav_core.frontier_detection import Blacklist, detect_frontiers
from uav_core.observation import classify, OBSERVED, SPARSE, UNKNOWN

RES = 1.0
ORIGIN = (0.0, 0.0)


def layers(observed, sparse=None, n=40):
    """Return count and variance with OBSERVED and SPARSE regions."""
    count = np.zeros((n, n), dtype=np.int32)
    variance = np.full((n, n), np.inf, dtype=np.float32)
    count[observed] = 100
    variance[observed] = 4e-4
    if sparse is not None:
        count[sparse] = 3
        variance[sparse] = 1e-2
    return count, variance


def test_classify_thresholds():
    """Count and variance gates give the three states."""
    count = np.array([[0, 3, 10, 10]])
    var = np.array([[np.inf, 1e-3, 1e-3, 5e-3]])
    assert classify(count, var).tolist() == [[UNKNOWN, SPARSE, OBSERVED,
                                              SPARSE]]


def test_half_known_map_has_frontiers_on_the_boundary():
    """A 40 m straight boundary gives four 10 m pieces on that boundary."""
    obs = np.zeros((40, 40), dtype=bool)
    obs[:, :20] = True
    cat = detect_frontiers(*layers(obs), RES, ORIGIN, vehicle_ne=(20, 5))
    assert len(cat.frontiers) == 4
    for f in cat.frontiers:
        assert set(f.cells[:, 1].tolist()) == {19}
        assert f.size == 10
        assert f.viewpoint_ij[1] <= 19
        assert cat.state[f.viewpoint_ij] == OBSERVED
        assert f.gain > 0
        assert f.path_dist_m == pytest.approx(
            np.hypot(f.viewpoint_ne[0] - 20, f.viewpoint_ne[1] - 5))


def test_take_off_disc_gives_pieces_all_around():
    """The first frontier is a circle: pieces in every direction."""
    ii, jj = np.indices((60, 60))
    obs = np.hypot(ii - 29.5, jj - 29.5) <= 18
    cat = detect_frontiers(*layers(obs, n=60), RES, ORIGIN,
                           vehicle_ne=(30, 30))
    assert len(cat.frontiers) >= 8
    angles = [np.arctan2(f.viewpoint_ne[1] - 30, f.viewpoint_ne[0] - 30)
              for f in cat.frontiers]
    sectors = {int((a + np.pi) // (np.pi / 2)) % 4 for a in angles}
    assert sectors == {0, 1, 2, 3}
    vps = {f.viewpoint_ij for f in cat.frontiers}
    assert len(vps) == len(cat.frontiers)


def test_full_coverage_gives_empty_catalogue():
    """No unknown cell in the area, no frontier: exploration ends."""
    obs = np.ones((40, 40), dtype=bool)
    cat = detect_frontiers(*layers(obs), RES, ORIGIN, vehicle_ne=(20, 20))
    assert cat.frontiers == [] and cat.unknown_in_area == 0


def test_unknown_outside_the_area_is_ignored():
    """Unknown ground outside the exploration area makes no frontier."""
    obs = np.zeros((40, 40), dtype=bool)
    obs[:, :20] = True
    area = obs.copy()
    cat = detect_frontiers(*layers(obs), RES, ORIGIN, vehicle_ne=(20, 5),
                           explore_mask=area)
    assert cat.frontiers == [] and cat.unknown_in_area == 0


def test_sparse_ring_does_not_hide_the_boundary():
    """A SPARSE ring between OBSERVED and UNKNOWN, as measured on v6."""
    obs = np.zeros((40, 40), dtype=bool)
    obs[:, :18] = True
    ring = np.zeros((40, 40), dtype=bool)
    ring[:, 18:20] = True
    cat = detect_frontiers(*layers(obs, ring), RES, ORIGIN,
                           vehicle_ne=(20, 5))
    assert len(cat.frontiers) == 4
    for f in cat.frontiers:
        assert set(f.cells[:, 1].tolist()) == {19}
        assert cat.state[f.viewpoint_ij] == OBSERVED


def test_small_clusters_are_noise():
    """A single unknown cell inside known ground is below the size floor."""
    obs = np.ones((40, 40), dtype=bool)
    obs[20, 20] = False
    cat = detect_frontiers(*layers(obs), RES, ORIGIN, vehicle_ne=(5, 5),
                           min_cluster_cells=9)
    assert cat.frontiers == [] and cat.rejected == []


def test_frontier_without_reachable_viewpoint_is_rejected():
    """No traversable OBSERVED cell near the cluster: unreachable."""
    obs = np.zeros((40, 40), dtype=bool)
    obs[:, :20] = True
    trav = np.zeros((40, 40), dtype=bool)
    trav[:, :5] = True
    cat = detect_frontiers(*layers(obs), RES, ORIGIN, vehicle_ne=(20, 2),
                           traversable=trav)
    assert cat.frontiers == []
    assert [f.rejected for f in cat.rejected] == ['unreachable'] * 4


def test_blacklisted_viewpoint_is_rejected_then_released():
    """A blacklisted viewpoint is set aside until its hold expires."""
    obs = np.zeros((40, 40), dtype=bool)
    obs[:, :20] = True
    count, var = layers(obs)
    first = detect_frontiers(count, var, RES, ORIGIN, vehicle_ne=(20, 5))
    bl = Blacklist(radius_m=1.0)
    bl.record_failure(first.frontiers[0].viewpoint_ne, now=0.0)
    held = detect_frontiers(count, var, RES, ORIGIN, vehicle_ne=(20, 5),
                            blacklist=bl, now=10.0)
    assert [f.rejected for f in held.rejected] == ['blacklisted']
    assert len(held.frontiers) == 3
    freed = detect_frontiers(count, var, RES, ORIGIN, vehicle_ne=(20, 5),
                             blacklist=bl, now=31.0)
    assert len(freed.frontiers) == 4


def test_blacklist_escalates_after_three_failures():
    """30 s holds for the first two failures, 300 s from the third."""
    bl = Blacklist()
    holds = [bl.record_failure((10.0, 10.0), now=t) for t in (0, 40, 80)]
    assert holds == [30.0, 30.0, 300.0]
    assert bl.blocked((11.0, 10.0), now=300.0)
    assert not bl.blocked((11.0, 10.0), now=381.0)
    assert not bl.blocked((20.0, 10.0), now=81.0)
