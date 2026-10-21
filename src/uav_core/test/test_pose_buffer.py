"""Tests of the pose interpolation at the scan time (pose_buffer)."""

import numpy as np

from uav_core.pose_buffer import ClockOffset, PoseBuffer


def filled(rate=50.0, duration=2.0, speed=5.0):
    """Return a buffer flying north at speed, yawing slowly, from 100 s."""
    b = PoseBuffer()
    for k in range(int(rate * duration)):
        t = 100.0 + k / rate
        b.add_position(t, (speed * (t - 100.0), 0.0, -8.0))
        yaw = 0.1 * (t - 100.0)
        b.add_attitude(t, (np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)))
    return b


def test_position_interpolated_between_samples():
    b = filled()
    p, _ = b.at(100.511)
    assert abs(p[0] - 5.0 * 0.511) < 1e-9


def test_attitude_interpolated_and_unit():
    b = filled()
    _, q = b.at(100.511)
    assert abs(np.linalg.norm(q) - 1.0) < 1e-12
    yaw = 2.0 * np.arctan2(q[3], q[0])
    assert abs(yaw - 0.0511) < 1e-4


def test_outside_the_buffer_is_none():
    b = filled()
    assert b.at(99.0) is None
    assert b.at(102.5) is None


def test_short_extrapolation_takes_the_last_sample():
    b = filled()
    last = 100.0 + 99 / 50.0
    p, _ = b.at(last + 0.01)
    assert abs(p[0] - 5.0 * (last - 100.0)) < 1e-9


def test_out_of_order_samples_are_dropped():
    b = filled()
    n = len(b.pos_t)
    b.add_position(100.1, (0.0, 0.0, 0.0))
    assert len(b.pos_t) == n


def test_quaternion_sign_flip_does_not_cancel():
    b = PoseBuffer()
    b.add_position(0.0, (0, 0, 0))
    b.add_position(1.0, (0, 0, 0))
    q = np.array([0.6, 0.0, 0.0, 0.8])
    b.add_attitude(0.0, q)
    b.add_attitude(1.0, -q)
    _, out = b.at(0.5)
    assert abs(abs(np.dot(out, q)) - 1.0) < 1e-12


def test_clock_offset_is_the_shortest_transport():
    c = ClockOffset()
    assert c.value is None
    for k, delay in enumerate([0.12, 0.10, 0.15, 0.11]):
        c.update(1000.0 + k + delay, 900.0 + k)
    assert abs(c.value - (100.0 + 0.10)) < 1e-9


def test_scan_time_through_two_clocks():
    """A scan stamped in simulation time, poses in PX4 time."""
    sim_to_sys = ClockOffset()
    px4_to_sys = ClockOffset()
    sim_to_sys.update(5000.002, 10.0)     # /clock, 2 ms transport
    px4_to_sys.update(5000.005, 7000.0)   # pose, 5 ms transport
    stamp = 12.0
    t_px4 = stamp + sim_to_sys.value - px4_to_sys.value
    assert abs(t_px4 - (7002.0 - 0.003)) < 1e-9
