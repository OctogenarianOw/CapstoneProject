"""
Rest-to-rest trapezoidal profile along a polyline (spec S4, S10).

Role: give, at any time since the start of a path, the position,
velocity and acceleration setpoints the executor sends to PX4, so that
the aircraft flies at a fixed cruise speed and stops at every vertex.

Choice: on each leg, accelerate at a up to v, cruise, decelerate to rest
at the vertex; a leg too short to reach v is a triangle with peak
sqrt(a L). The planned duration of a leg, L / v + v / a, is exactly the
time energy_model.EnergyModel.leg_time uses, which is the point: the
energy model was calibrated on this profile (19 Oct, 16 flights, flown
length within +0.9 to +3.8 % of planned, cruise speed median 4.99 to
5.10 m/s) and holds only if the executor flies it.

Rejected, measured in the calibration pilot: a carrot advancing at
constant speed with velocity feedforward only, which overshot every
corner by 3 m and flew 56.3 m for a 40 m square.

Moved here unchanged from
experiments/2026-10-19_energy_calibration/scripts/calibration_flight.py,
which keeps its own copy as the record of what was flown.
"""

import math


def leg_profile(length, v, a):
    """Return peak speed, ramp time and total time of one leg."""
    v_peak = min(v, math.sqrt(a * length))
    t_ramp = v_peak / a
    d_ramp = 0.5 * v_peak * t_ramp
    t_cruise = (length - 2.0 * d_ramp) / v_peak if v_peak > 0 else 0.0
    return v_peak, t_ramp, 2.0 * t_ramp + t_cruise


class TrapezoidPath:
    """Rest-to-rest trapezoidal profile on each leg of a polyline."""

    def __init__(self, pts, v, a):
        self.pts = [tuple(p) for p in pts]
        self.a = a
        self.legs = []
        t0 = 0.0
        for (n0, e0), (n1, e1) in zip(self.pts[:-1], self.pts[1:]):
            length = math.hypot(n1 - n0, e1 - e0)
            if length < 1e-9:
                continue
            v_peak, t_ramp, t_leg = leg_profile(length, v, a)
            u = ((n1 - n0) / length, (e1 - e0) / length)
            self.legs.append({'start': (n0, e0), 'u': u, 'length': length,
                              'v': v_peak, 't_ramp': t_ramp, 't0': t0,
                              't_leg': t_leg})
            t0 += t_leg
        self.duration = t0
        self.length = sum(leg['length'] for leg in self.legs)
        self.end = self.pts[-1]

    def leg_index(self, t):
        """Return the index of the leg flown at time t."""
        for k, leg in enumerate(self.legs):
            if t < leg['t0'] + leg['t_leg']:
                return k
        return max(0, len(self.legs) - 1)

    def at(self, t):
        """Position, velocity and acceleration (north, east) at time t."""
        if not self.legs or t >= self.duration:
            return self.end, (0.0, 0.0), (0.0, 0.0)
        leg = self.legs[self.leg_index(t)]
        tau = max(0.0, t - leg['t0'])
        a, vp, tr, tl = self.a, leg['v'], leg['t_ramp'], leg['t_leg']
        if tau < tr:
            s, sv, sa = 0.5 * a * tau * tau, a * tau, a
        elif tau < tl - tr:
            s, sv, sa = 0.5 * vp * tr + vp * (tau - tr), vp, 0.0
        else:
            rem = tl - tau
            s, sv, sa = leg['length'] - 0.5 * a * rem * rem, a * rem, -a
        un, ue = leg['u']
        return ((leg['start'][0] + un * s, leg['start'][1] + ue * s),
                (un * sv, ue * sv), (un * sa, ue * sa))
