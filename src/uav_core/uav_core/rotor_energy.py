"""
Rotor shaft power and its integral, the energy signal of the budget (S10).

Role: turn the rotor speed commands PX4 sends to Gazebo into the shaft
power of the four rotors, and integrate it into energy consumed.

Model of the Gazebo MulticopterMotorModel, constants from the x500
model.sdf: thrust = k_f w^2, drag torque = k_m thrust, so the shaft
power of one rotor is k_m k_f w^3. Validated on hover on 17 Oct 2026:
805.5 rad/s measured against 806.0 predicted from m g = 4 k_f w^2, and
285.9 W against 286.5 W (experiments/2026-10-17_energy_signal).

Rejected as the budget signal: PX4 battery_status in SITL, whose
current is -1, discharged_mah 0, and remaining a ramp in armed time
(same campaign). Regressing it on distance would only measure time.

Limit to declare: this is the mechanical energy of the simulator's
rotor model, without motor and ESC efficiency or voltage sag. It is
consistent and predictive within the simulation, which the study needs,
and does not transfer as such to a real aircraft (S10).

Integration: trapezoid between consecutive samples, on the time of
reception. The header stamp of the Gazebo message is zero (17 Oct), and
the simulation runs at a real-time factor of 1.0, so reception time is
sim time to within the transport delay. A gap longer than max_gap_s is
not integrated across but counted, so a stalled bridge shows up instead
of being bridged by a straight line.
"""

from dataclasses import dataclass

import numpy as np

K_F = 8.54858e-06    # motorConstant, N / (rad/s)^2, x500 model.sdf
K_M = 0.016          # momentConstant, m, x500 model.sdf


def shaft_power(speeds):
    """Shaft power in watts of rotors at the given speeds in rad/s."""
    w = np.abs(np.asarray(speeds, dtype=np.float64))
    return float(K_M * K_F * np.sum(w ** 3))


@dataclass
class EnergyIntegrator:
    """Trapezoid integral of power samples, with gap accounting."""

    max_gap_s: float = 0.5
    consumed_j: float = 0.0
    samples: int = 0
    gaps: int = 0
    last_t: float = None
    last_p: float = None

    def add(self, t, power_w):
        """Fold one power sample taken at time t, in seconds."""
        if self.last_t is not None:
            dt = t - self.last_t
            if 0.0 < dt <= self.max_gap_s:
                self.consumed_j += 0.5 * (power_w + self.last_p) * dt
            elif dt > self.max_gap_s:
                self.gaps += 1
        self.last_t, self.last_p = t, power_w
        self.samples += 1
