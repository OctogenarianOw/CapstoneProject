"""
Pose at the time of a lidar scan, from the PX4 pose stream (fix of P5).

Role: give cloud_world the position and attitude of the aircraft at the
instant a scan was taken, not at the instant it was received. Measured
on 21 Oct (experiments/2026-10-21_p5_latency): with the latest pose, the
edges of the plates of the pit worlds spill half a cell along the
flight, and rebuilding the map with the pose taken 0.110 to 0.119 s
earlier removes the direction-dependent part. Of that, 0.03 s is the
pose arriving after the scan, which this module corrects through the
stamps; the rest, a constant between the PX4 and simulation time bases,
is a calibration term cloud_world subtracts (scan_time_offset_s).

Choice: interpolate the pose stream at the scan stamp. Position linear,
attitude by normalised linear interpolation of the quaternion (the
attitude moves a fraction of a degree between two PX4 samples at 50 Hz,
where nlerp and slerp agree). The scan stamp is simulation time and the
PX4 stamps another clock (the system clock in this SITL, measured, but
microseconds since boot on a real PX4), so each is related to the
system clock by its own offset, estimated as the lower envelope of
reception time minus stamp over a sliding window: the shortest transport
seen. Nothing assumes the two clocks agree.

Rejected: shifting the latest pose back by the measured lag times the
velocity. It freezes a latency measured under one load, and the lag
grows with the load of the machine; the stamp follows it by itself.

Pure module, no ROS: tested in test/test_pose_buffer.py.
"""

from collections import deque

import numpy as np


class ClockOffset:
    """System time minus stamp time, lower envelope over a window."""

    def __init__(self, window=500):
        self.samples = deque(maxlen=window)

    def update(self, received_s, stamp_s):
        """Add one message: when it arrived and what it was stamped."""
        self.samples.append(received_s - stamp_s)

    @property
    def ready(self):
        """Tell whether there is at least one sample."""
        return len(self.samples) > 0

    @property
    def value(self):
        """Offset in seconds, None before the first sample."""
        return min(self.samples) if self.samples else None


class PoseBuffer:
    """Recent positions and attitudes on the PX4 clock, interpolated."""

    def __init__(self, horizon=400, max_extrapolation_s=0.03):
        self.pos_t = deque(maxlen=horizon)
        self.pos = deque(maxlen=horizon)
        self.att_t = deque(maxlen=horizon)
        self.att = deque(maxlen=horizon)
        self.max_extrapolation_s = max_extrapolation_s

    def add_position(self, t_s, xyz):
        """Add a NED position sampled at t_s (PX4 clock, seconds)."""
        if self.pos_t and t_s <= self.pos_t[-1]:
            return
        self.pos_t.append(t_s)
        self.pos.append(np.asarray(xyz, dtype=float))

    def add_attitude(self, t_s, q):
        """Add a quaternion [w, x, y, z] sampled at t_s."""
        if self.att_t and t_s <= self.att_t[-1]:
            return
        self.att_t.append(t_s)
        self.att.append(np.asarray(q, dtype=float))

    def _bracket(self, times, t):
        """Return the indices and weight of the samples around t, or None."""
        n = len(times)
        if n == 0 or t < times[0]:
            return None
        if t >= times[-1]:
            if t - times[-1] > self.max_extrapolation_s:
                return None
            return n - 1, n - 1, 0.0
        arr = np.fromiter(times, dtype=float, count=n)
        k = int(np.searchsorted(arr, t, side='right'))
        a = (t - arr[k - 1]) / (arr[k] - arr[k - 1])
        return k - 1, k, a

    def at(self, t_s):
        """Position and attitude at t_s, or None if not covered."""
        bp = self._bracket(self.pos_t, t_s)
        ba = self._bracket(self.att_t, t_s)
        if bp is None or ba is None:
            return None
        i, j, a = bp
        p = (1.0 - a) * self.pos[i] + a * self.pos[j]
        i, j, a = ba
        q0, q1 = self.att[i], self.att[j]
        if np.dot(q0, q1) < 0.0:
            q1 = -q1
        q = (1.0 - a) * q0 + a * q1
        return p, q / np.linalg.norm(q)
