"""
Smoke test of the exploration manager node, without simulator.

The first autonomous mission of 19 Oct stopped right after take-off on
a TypeError in the action log, a path no unit test of the pure modules
went through. This test builds the node, sends every kind of action,
and runs one full map cycle (world model, decision, display, log) on a
synthetic map, so the glue code is exercised before any flight.
"""

import json
from pathlib import Path

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Header
from uav_core.exploration_manager import ExplorationManager
from uav_core.map_msg import grid_to_msg
from uav_core.mission_fsm import Action


def test_manager_acts_and_runs_a_map_cycle(tmp_path):
    """Every action is sent and logged; one map cycle is logged."""
    rclpy.init(args=['--ros-args', '-p', f'log_root:={tmp_path}', '-p',
                     'mode:=explore'])
    try:
        node = ExplorationManager()
        node.act([Action('takeoff'),
                  Action('path', waypoints_ne=[(0.0, 0.0), (5.0, 5.0)],
                         path_id=1),
                  Action('hold'), Action('land')])
        n = 100
        ii, jj = np.indices((n, n))
        disc = np.hypot(-50 + ii + 0.5, -50 + jj + 0.5) <= 18.0
        msg = grid_to_msg(np.where(disc, 0.0, np.nan),
                          np.where(disc, 4e-4, np.inf),
                          np.where(disc, 100, 0), 1.0, (-50.0, -50.0),
                          Header())
        node.vehicle = (0.0, 0.0, -8.0)
        node.pose_valid = True
        node.on_map(msg)
        # alpha: declared range for the rqt_reconfigure slider, and a
        # change reaches the state machine and the log.
        rng = node.describe_parameter('alpha').floating_point_range[0]
        assert (rng.from_value, rng.to_value) == (0.0, 1.0)
        node.set_parameters([Parameter('alpha', value=0.25)])
        assert node.fsm.params.alpha == 0.25
        node.log.close()
        run = Path(node.log.dir)
        events = [json.loads(line) for line in
                  (run / 'events.jsonl').read_text().splitlines()]
        kinds = [e['action'] for e in events if e['kind'] == 'action']
        assert kinds == ['takeoff', 'path', 'hold', 'land']
        assert [e['value'] for e in events if e['kind'] == 'alpha'] == [0.25]
        cycles = (run / 'cycles.jsonl').read_text().splitlines()
        assert len(cycles) == 1
        assert json.loads(cycles[0])['unknown_in_area'] > 0
        # The whole cycle is timed, decision and display included.
        timings = json.loads(cycles[0])['timings_ms']
        assert {'decode', 'decision', 'display'} <= set(timings)
        # No injected latency unless asked, and the value is logged.
        config = json.loads((run / 'config.json').read_text())
        assert config['inject_delay_s'] == 0.0
        node.destroy_node()
    finally:
        rclpy.shutdown()
