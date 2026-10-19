"""
World model of one decision cycle: everything the manager knows.

Role: run, on one map, the whole perception chain in a fixed order, and
hand the exploration manager a single consistent snapshot. Pure numpy,
no ROS, so a cycle can be replayed offline on any saved map.

Order, each step reading only the steps before it:
  1. observation states (observation.py, S6)
  2. terrain layers, slope, roughness, step, validity (terrain_analysis)
  3. candidate landing cells and zones (slz_detection, S8)
  4. traversable cells at cruise altitude (planner, S12, I3)
  5. path distance field from the vehicle (planner)
  6. frontier catalogue on that field (frontier_detection, S11)

Choice: one function, recomputed from scratch on every map, rather than
incremental updates. At 1 Hz on a 100 x 100 grid the whole chain runs in
tens of milliseconds (measured in the live test of this step), and a
stateless cycle cannot drift from the map it claims to describe. State
that must persist across cycles, the landing zone registry and the
blacklist, lives in its own objects, passed in.
"""

from dataclasses import dataclass
import time

import numpy as np

from uav_core.frontier_detection import detect_frontiers
from uav_core.observation import classify
from uav_core.planner import (distance_field, nearest_traversable,
                              traversable_mask)
from uav_core.slz_detection import detect
from uav_core.terrain_analysis import analyse

THRESHOLDS = {'max_slope_deg': 6.0, 'max_roughness_m': 0.05,
              'max_step_m': 0.24}


@dataclass
class WorldParams:
    """Parameters of the perception chain, with their origin."""

    altitude_m: float = 8.0           # survey altitude, 4 m above the cube
    vertical_margin_m: float = 2.0    # chosen, map error and P2 shrink
    inflate_m: float = 1.25           # half-span + 0.90 m tracking, measured
    footprint_m: float = 1.5          # estimated, P4 open
    sensor_radius_m: float = 18.0     # sqrt(20^2 - 8^2), geometry
    area_half_m: float = 30.0         # exploration area, validation world


@dataclass
class WorldModel:
    """Snapshot of one cycle."""

    state: np.ndarray
    layers: dict
    landing: dict            # output of slz_detection.detect
    traversable: np.ndarray
    vehicle_ij: tuple        # cell used as field source, None if none
    dist_m: np.ndarray       # path distance from the vehicle, metres
    catalogue: object        # frontier_detection.Catalogue
    explore_mask: np.ndarray
    timings_ms: dict


def area_mask(shape, resolution, origin_ne, half_m):
    """Cells whose centre lies in the square of half side half_m."""
    i, j = np.indices(shape)
    north = origin_ne[0] + (i + 0.5) * resolution
    east = origin_ne[1] + (j + 0.5) * resolution
    return (np.abs(north) <= half_m) & (np.abs(east) <= half_m)


def build_world(elevation, variance, count, resolution, origin_ne,
                vehicle_ne, params, blacklist=None, now=0.0):
    """Run the perception chain on one map."""
    timings = {}
    t0 = time.perf_counter()

    def lap(name):
        nonlocal t0
        t1 = time.perf_counter()
        timings[name] = 1e3 * (t1 - t0)
        t0 = t1

    state = classify(count, variance)
    layers = analyse(elevation, count, resolution)
    lap('analysis')
    landing = detect(layers, elevation, resolution, origin_ne,
                     footprint_m=params.footprint_m, **THRESHOLDS)
    lap('landing')
    traversable = traversable_mask(elevation, state, resolution,
                                   params.altitude_m,
                                   params.vertical_margin_m,
                                   params.inflate_m)
    vi = int((vehicle_ne[0] - origin_ne[0]) / resolution)
    vj = int((vehicle_ne[1] - origin_ne[1]) / resolution)
    start = nearest_traversable(traversable, (vi, vj),
                                max_cells=int(np.ceil(2.0 / resolution)))
    if start is None:
        dist = np.full(state.shape, np.inf)
    else:
        dist = distance_field(traversable, [start]) * resolution
    lap('field')
    explore = area_mask(state.shape, resolution, origin_ne,
                        params.area_half_m)
    catalogue = detect_frontiers(count, variance, resolution, origin_ne,
                                 vehicle_ne, traversable=traversable,
                                 dist_field=dist, explore_mask=explore,
                                 blacklist=blacklist, now=now,
                                 sensor_radius_m=params.sensor_radius_m)
    lap('frontiers')
    return WorldModel(state=state, layers=layers, landing=landing,
                      traversable=traversable, vehicle_ij=start,
                      dist_m=dist, catalogue=catalogue,
                      explore_mask=explore, timings_ms=timings)
