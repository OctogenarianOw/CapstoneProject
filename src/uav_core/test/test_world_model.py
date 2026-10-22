"""
World model on the reference map v6, and the mission log, offline.

The world model must reproduce on v6 what the separate measurements of
19 Oct found: the ramp shadow as the only unknown ground of the
validation world, four frontier pieces there, landing cells present,
one 1 Hz cycle well under a second.
"""

import json
from pathlib import Path

import numpy as np
import pytest
from uav_core.mission_log import MissionLog
from uav_core.world_model import build_world, WorldParams

V6 = Path(__file__).resolve().parents[3] / 'data_survey_v6.npz'


@pytest.mark.skipif(not V6.exists(), reason='reference map not found')
def test_world_model_on_v6():
    """Frontiers, landing cells and cycle time on the reference map."""
    m = np.load(V6)
    origin = (float(m['origin_x']), float(m['origin_y']))
    world = build_world(m['elevation'], m['variance'], m['count'],
                        float(m['resolution']), origin, (0.0, 0.0),
                        WorldParams())
    assert world.catalogue.unknown_in_area == 15
    assert len(world.catalogue.frontiers) == 4
    assert world.landing['candidate'].sum() > 1000
    assert world.vehicle_ij is not None
    assert sum(world.timings_ms.values()) < 1000.0


def test_mission_log_writes_config_cycles_events(tmp_path):
    """Config, JSON lines with non-finite floats, snapshot throttling."""
    log = MissionLog(str(tmp_path), {'altitude_m': 8.0})
    log.cycle(1.0, {'dist': float('inf'), 'gain': np.int64(3)})
    log.event(1.5, 'state', to='SELECTING')
    log.maybe_snapshot(2.0, count=np.zeros((2, 2)))
    log.maybe_snapshot(5.0, count=np.zeros((2, 2)))
    log.close()
    run = Path(log.dir)
    assert json.loads((run / 'config.json').read_text())['altitude_m'] == 8
    cycle = json.loads((run / 'cycles.jsonl').read_text())
    assert cycle == {'t': 1.0, 'dist': 'inf', 'gain': 3}
    event = json.loads((run / 'events.jsonl').read_text())
    assert event['kind'] == 'state' and event['to'] == 'SELECTING'
    assert len(list((run / 'maps').iterdir())) == 1


def roof_world(roof_half_m):
    """
    Return the world of an aircraft over the middle of a blocked roof.

    A flat observed square, a roof at 6.5 m around the aircraft, above
    the 6 m ceiling of I3.
    """
    n, res, origin = 60, 1.0, (-30.0, -30.0)
    ii, jj = np.indices((n, n))
    north, east = origin[0] + ii + 0.5, origin[1] + jj + 0.5
    roof = (np.abs(north) <= roof_half_m) & (np.abs(east) <= roof_half_m)
    elev = np.where(roof, -6.5, 0.0).astype(np.float32)
    count = np.full((n, n), 100, dtype=np.int32)
    var = np.full((n, n), 4e-4, dtype=np.float32)
    return build_world(elev, var, count, res, origin, (0.0, 0.0),
                       WorldParams())


def test_aircraft_over_a_blocked_roof_still_has_a_start_cell():
    """
    Find a free start cell off a roof mapped above the ceiling.

    Over a W3 roof whose units were mapped at 6.3 m on approach, the
    nearest free cell was 3.0 m away and the search, 2 m then, found
    none: an emergency landing. Within 10 m it is found, past the roof
    edge and its inflation; a roof 24 m wide still has none.
    """
    world = roof_world(4.0)
    assert world.vehicle_ij is not None
    i, j = world.vehicle_ij
    assert np.hypot(i - 30, j - 30) >= 5.0
    assert np.isfinite(world.dist_m).any()
    assert roof_world(12.0).vehicle_ij is None
