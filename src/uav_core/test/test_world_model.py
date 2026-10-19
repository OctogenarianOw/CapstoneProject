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
