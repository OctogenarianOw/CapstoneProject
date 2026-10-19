"""
Unit tests for planner, on synthetic elevation maps.

Checked: the field is the octile distance on open ground, it routes
around walls, several sources give the distance to the nearest one,
simplification leaves straight legs that never cross a blocked cell,
and traversability follows I3 and the inflation radius.
"""

import math

import numpy as np
import pytest
from uav_core.observation import OBSERVED, UNKNOWN
from uav_core.planner import (distance_field, extract_path, line_of_sight,
                              nearest_traversable, plan_path, simplify,
                              traversable_mask)

RES = 1.0
ORIGIN = (0.0, 0.0)


def octile(a, b):
    """Return the octile distance between two cells."""
    di, dj = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(di, dj) + (math.sqrt(2.0) - 1.0) * min(di, dj)


def test_open_field_is_octile_distance():
    """With nothing in the way the field is the octile metric."""
    free = np.ones((30, 30), dtype=bool)
    dist = distance_field(free, [(3, 4)])
    for cell in [(3, 4), (20, 4), (25, 27), (0, 29)]:
        assert dist[cell] == pytest.approx(octile((3, 4), cell))


def test_open_path_is_one_straight_leg():
    """Simplification leaves a single leg on open ground."""
    free = np.ones((30, 30), dtype=bool)
    dist = distance_field(free, [(2, 2)])
    plan = plan_path(free, dist, (25, 17), RES, ORIGIN)
    assert plan.cells == [(2, 2), (25, 17)]
    assert plan.length_m == pytest.approx(math.hypot(23, 15))


def test_path_goes_through_the_gap_and_never_crosses_the_wall():
    """
    A wall with one gap: few legs, all free, near the shortest length.

    Greedy simplification keeps vertices of the raw grid path only, so
    it may keep one leg more than an any-angle planner: 4 here, where 3
    would do. The length stays within 5 % of the two straight legs
    through the gap centre.
    """
    free = np.ones((30, 30), dtype=bool)
    free[15, :] = False
    free[15, 25:27] = True
    dist = distance_field(free, [(2, 2)])
    plan = plan_path(free, dist, (28, 2), RES, ORIGIN)
    assert 2 <= len(plan.cells) - 1 <= 4
    for a, b in zip(plan.cells[:-1], plan.cells[1:]):
        assert line_of_sight(free, a, b)
    via_gap = 2 * math.hypot(13.0, 23.5)
    assert abs(plan.length_m - via_gap) / via_gap < 0.05


def test_several_sources_give_distance_to_the_nearest():
    """The field seeded on two zones is the distance to the closer one."""
    free = np.ones((30, 30), dtype=bool)
    dist = distance_field(free, [(5, 5), (25, 25)])
    assert dist[6, 6] == pytest.approx(octile((5, 5), (6, 6)))
    assert dist[24, 25] == pytest.approx(1.0)
    assert dist[15, 15] == pytest.approx(octile((5, 5), (15, 15)))


def test_unreachable_goal_gives_no_plan():
    """A goal walled off from the source has no path."""
    free = np.ones((20, 20), dtype=bool)
    free[10, :] = False
    dist = distance_field(free, [(2, 2)])
    assert math.isinf(dist[15, 15])
    assert plan_path(free, dist, (15, 15), RES, ORIGIN) is None


def test_traversability_follows_altitude_margin_and_inflation():
    """Tall cells block (I3), unknown cells block, inflation grows them."""
    elevation = np.zeros((20, 20), dtype=np.float32)
    state = np.full((20, 20), OBSERVED, dtype=np.int8)
    elevation[10, 10] = -6.5      # 6.5 m high: above 8 - 2 m
    elevation[3, 3] = -5.5        # 5.5 m high: below the limit
    state[18, 18] = UNKNOWN
    free = traversable_mask(elevation, state, RES, altitude_m=8.0,
                            vertical_margin_m=2.0, inflate_m=1.25)
    assert not free[10, 10] and not free[12, 12] and free[13, 10]
    assert free[3, 3]
    assert not free[18, 18] and not free[16, 16] and free[15, 18]


def loop_reference(cells, traversable):
    """Return the previous one-by-one simplification, the reference."""
    if len(cells) <= 2:
        return list(cells)
    out, k = [cells[0]], 0
    while k < len(cells) - 1:
        far = k + 1
        for m in range(len(cells) - 1, k, -1):
            if line_of_sight(traversable, cells[k], cells[m]):
                far = m
                break
        out.append(cells[far])
        k = far
    return out


def test_vectorised_simplify_matches_the_loop_on_random_fields():
    """Same vertices as the one-by-one loop on 100 random obstacle maps."""
    rng = np.random.default_rng(3)
    checked = 0
    for _ in range(100):
        free = rng.random((60, 60)) > rng.uniform(0.0, 0.35)
        free[0, :] = free[:, 0] = True
        dist = distance_field(free, [(0, 0)])
        goals = np.argwhere(np.isfinite(dist))
        goal = tuple(goals[rng.integers(len(goals))])
        raw = extract_path(dist, goal)[::-1]
        assert simplify(raw, free) == loop_reference(raw, free)
        checked += 1
    assert checked == 100


def test_start_inside_inflation_moves_to_nearest_free_cell():
    """A start cell inside the inflation is moved, but not too far."""
    free = np.ones((20, 20), dtype=bool)
    free[8:13, 8:13] = False
    assert nearest_traversable(free, (10, 10), max_cells=3) in {
        (7, 10), (13, 10), (10, 7), (10, 13)}
    assert nearest_traversable(free, (10, 10), max_cells=2) is None
