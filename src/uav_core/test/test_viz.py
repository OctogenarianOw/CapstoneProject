"""
The classification layer: one box per seen cell, one colour per class.

Built after the elevation layer, coloured by height, was read as a
safety map on 19 Oct. Checked on a synthetic map with a step: candidate,
margin, refused, uncertain and unseen cells get the right colour, and
the message holds exactly one 16-byte point per seen cell.
"""

import numpy as np
from std_msgs.msg import Header
from uav_core.viz import CLASS_RGB, classification_cloud, ratio_colour
from uav_core.world_model import build_world, WorldParams


def rgb(name):
    """Return the packed colour of a class."""
    r, g, b = CLASS_RGB[name]
    return (r << 16) | (g << 8) | b


def test_every_seen_cell_gets_its_class_colour():
    """Flat ground green, step red, sparse grey, unseen absent."""
    n = 40
    count = np.full((n, n), 100, dtype=np.int32)
    var = np.full((n, n), 4e-4, dtype=np.float32)
    elev = np.zeros((n, n), dtype=np.float32)
    elev[:, 30:] = -2.0          # a 2 m step along j = 30
    count[5, 5] = 3              # seen but not OBSERVED
    count[0:2, :] = 0            # two unseen rows
    var[0:2, :] = np.inf
    elev[0:2, :] = np.nan
    w = build_world(elev, var, count, 1.0, (0.0, 0.0), (20.0, 10.0),
                    WorldParams())
    msg = classification_cloud(elev, w.state, w.layers, w.landing, 1.0,
                               (0.0, 0.0), Header())
    seen = int((count > 0).sum())
    assert msg.width == seen and len(msg.data) == 16 * seen
    cloud = np.frombuffer(bytes(msg.data), dtype=[
        ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    colour = {(int(p['y']), int(p['x'])): int(p['rgb']) for p in cloud}
    # Step between columns 29 and 30: both refused by the step test,
    # 28 sound but its 3 x 3 footprint touches 29, 27 a candidate.
    assert colour[(20, 10)] == rgb('candidate')
    assert colour[(20, 27)] == rgb('candidate')
    assert colour[(20, 28)] == rgb('margin')
    assert colour[(20, 29)] == rgb('refused')
    assert colour[(20, 30)] == rgb('refused')
    assert colour[(5, 5)] == rgb('uncertain')
    assert (0, 10) not in colour


def test_score_colours_green_yellow_red_around_the_threshold():
    """Half the threshold green, the threshold yellow, twice red."""
    green, yellow, red = (int(c) for c in ratio_colour([0.2, 1.0, 3.0]))
    assert green == (0 << 16) | (210 << 8) | 40
    assert (yellow >> 16) & 255 == 230 and (yellow >> 8) & 255 == 210
    assert (red >> 16) & 255 == 230 and (red >> 8) & 255 == 0
