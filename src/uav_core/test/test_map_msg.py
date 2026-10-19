"""Round trip of the elevation grid through the ElevationMap message."""

import numpy as np
from std_msgs.msg import Header
from uav_core.map_msg import grid_to_msg, msg_to_layers


def test_round_trip_keeps_layers_geometry_and_unknown_markers():
    """Layers, NaN and inf markers, resolution and origin survive."""
    rng = np.random.default_rng(0)
    elev = rng.normal(size=(7, 5)).astype(np.float32)
    var = rng.uniform(1e-4, 1e-2, size=(7, 5)).astype(np.float32)
    cnt = rng.integers(0, 50, size=(7, 5)).astype(np.int32)
    elev[2, 3], var[2, 3], cnt[2, 3] = np.nan, np.inf, 0
    msg = grid_to_msg(elev, var, cnt, 0.5, (-50.0, -25.0), Header())
    e2, v2, c2, res, origin = msg_to_layers(msg)
    assert (msg.rows, msg.cols) == (7, 5)
    np.testing.assert_array_equal(e2, elev)
    np.testing.assert_array_equal(v2, var)
    np.testing.assert_array_equal(c2, cnt)
    assert res == 0.5 and origin == (-50.0, -25.0)
    assert e2[2, 3] != e2[2, 3] and np.isinf(v2[2, 3])
