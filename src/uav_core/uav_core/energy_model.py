"""
Energy prediction for planned flights (spec S10), input of C1 and C2.

Role: turn a planned path, a hover duration or a landing into the rotor
shaft energy it will cost, before flying it.

Model: E = P_hover * t + k_move * d + c_leg * n_legs, with t taken from
the rest-to-rest trapezoidal profile flown in calibration, which the
trajectory executor must fly too, so a planned path has an exact
planned time.

Chosen over the two-term S10 form E = P_hover * t + k_move * d, on the
calibration of 19 Oct 2026 (experiments/2026-10-19_energy_calibration).
The two-term fit was optimistic on short legs, -4.2 % on 10 m transits,
and on a 220 m zigzag with 8 legs, -1.00 % from the plan. Every leg pays
the same two ramps, about 65 J; with that term the transit errors fall
within +/-0.16 % and the zigzag, outside the fit, to -0.14 %. n_legs is
known from the plan, so C1 can use it. An optimistic model is the wrong
direction for a safety guarantee (spec 5.3), which settled the choice.

Rejected: E = k d, refuted on 17 Oct (48 J/m predicted 77 J for a 60 s
hover that used 21 kJ).

Validity: the profile the model was fitted on, 5 m/s cruise, 3 m/s2
ramps, stop at every vertex, 8 m altitude, no wind. Another profile
needs another calibration. Units are rotor shaft joules, the quantity
the instrument of 17 Oct measures, not battery joules.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class EnergyModel:
    """Calibrated coefficients and the predictions built on them."""

    p_hover_w: float        # power while holding position
    k_move_j_per_m: float   # extra energy per metre at cruise, may be < 0
    c_leg_j: float          # fixed energy of one rest-to-rest leg
    e_land_j: float         # LAND command to disarm, from cruise altitude
    cruise_speed_ms: float  # profile the coefficients belong to
    accel_ms2: float
    settle_s: float         # mean arrival settling after the profile ends

    def leg_time(self, length_m):
        """Duration of one rest-to-rest trapezoidal leg."""
        if length_m <= 0.0:
            return 0.0
        v, a = self.cruise_speed_ms, self.accel_ms2
        v_peak = min(v, math.sqrt(a * length_m))
        t_ramp = v_peak / a
        return 2.0 * t_ramp + (length_m - v_peak * t_ramp) / v_peak

    @staticmethod
    def leg_lengths(path):
        """Lengths of the non-degenerate legs of a polyline of (n, e)."""
        out = []
        for (n0, e0), (n1, e1) in zip(path[:-1], path[1:]):
            length = math.hypot(n1 - n0, e1 - e0)
            if length > 1e-9:
                out.append(length)
        return out

    def path_time(self, path):
        """Planned duration of a polyline, settling included."""
        legs = self.leg_lengths(path)
        if not legs:
            return 0.0
        return sum(self.leg_time(length) for length in legs) + self.settle_s

    def path_energy(self, path):
        """Energy of flying a polyline, stopping at each vertex."""
        legs = self.leg_lengths(path)
        if not legs:
            return 0.0
        return (self.p_hover_w * self.path_time(path)
                + self.k_move_j_per_m * sum(legs)
                + self.c_leg_j * len(legs))

    def hover_energy(self, seconds):
        """Energy of holding position for a given time."""
        return self.p_hover_w * max(0.0, seconds)


# Calibration of 19 Oct 2026, 36 segments of 12 flights, 95 % intervals
# P_hover +/- 0.12 W, k_move +/- 0.042 J/m, c_leg +/- 1.3 J; E_land sd 15 J
# over 12 descents. The intervals reflect model misfit, not noise: the
# simulation repeats to 0.02 %.
CALIBRATION_2026_10_19 = EnergyModel(
    p_hover_w=285.636,
    k_move_j_per_m=-0.400,
    c_leg_j=65.223,
    e_land_j=3199.5,
    cruise_speed_ms=5.0,
    accel_ms2=3.0,
    settle_s=0.058,
)
