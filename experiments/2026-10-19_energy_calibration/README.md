# Energy model calibration, 19 Oct 2026 (spec S10, gate C)

Question: does E = P_hover t + k_move d, fitted on rotor shaft energy at a
fixed and logged cruise speed, predict trajectories it was not fitted on
to better than 10 %, and does its P_hover agree with the 285.9 W of pure
hover?

Setup: PX4 v1.17.0-alpha1, Gazebo world `validation`, airframe
4030_gz_x500_slz, HEADLESS=1, fresh PX4 and fresh Gazebo for every flight
(session_cal.sh refuses a surviving server or a reused world). Altitude
8 m, clear of the highest relief (the 4 m cube). Energy is the rotor
shaft energy of the 17 Oct instrument, P = k_m k_f sum(w^3), integrated
on the probe clock.

## Design

Chosen: cruise speed 5 m/s, the PX4 default MPC_XY_CRUISE; ramps 3 m/s2,
the PX4 default MPC_ACC_HOR. Each leg is a rest-to-rest trapezoid sent as
position, velocity and acceleration (calibration_flight.py), so the speed
is fixed by construction and the planned leg time is exact, L / v + v / a.

Rejected, measured on pilot1_carrot_L10_h10: a carrot at constant speed
with velocity feedforward only. 56.3 m flown for a 40 m square, speed 4.3
to 7.9 m/s during cruise, 3 m past each corner. pilot2_trapezoid_L10_h10
flies the same square at 41.5 m and 4.63 to 5.42 m/s.

Chosen: a hover of 10, 20 or 30 s before each lap, every square size
seeing all three. At one speed transit time follows from distance, so
transit segments alone would leave only k_eff identifiable. Over the fit
rows corr(t, d) is 0.851, not 1.

Plan, scripts/run_campaign.sh: squares of 10, 20, 40, 60 m, 3 repetitions
each, rotated order. Validation, never in the fit: 30 m square after 60 s
of hover (the regime that refuted E = k d), and a 220 m zigzag with 8 legs
against 4 for every square. 16 flights, 16 succeeded at the first
attempt, 0 failsafes (runs/campaign.log).

## Results

### Tracking (scripts/tracking.py, runs/tracking.txt)

| quantity | all 16 flights |
|---|---|
| flown / planned length | +0.9 % to +3.8 % (S12 asks for 5 %) |
| position error to profile | max 0.90 m, p95 at most 0.82 m |
| speed while the profile cruises | median 4.99 to 5.10 m/s |
| measured transit time - planned | 0.06 s mean, sd 0.05 s |

### Two-term fit, the S10 form (scripts/analyse_calibration.py, runs/analysis.txt)

36 segments (hover_pre, transit, hover_post) of the 12 calibration flights,
least squares without intercept, 95 % intervals.

| parameter | value |
|---|---|
| P_hover | 286.43 +/- 2.03 W |
| k_move | 0.885 +/- 0.563 J/m |
| k_eff = P_hover / v + k_move at 5 m/s | 58.17 +/- 0.31 J/m |

Consistency check of S10: P_hover is 286.4 W against 285.9 W for the pure
hover of 17 Oct, +0.19 %. The 24 hover segments alone give 285.8 W, sd
0.4 W.

Prediction error per transit segment, (predicted - measured) / measured,
by square size: 10 m -4.2 %, 20 m -2.2 %, 40 m -0.2 %, 60 m +0.6 %, each
identical across the three repetitions to 0.02 %. Hover segments stay
within 0.5 %. The error is structured, not noise: the model is optimistic
on short legs.

E_land, from LAND to disarm, 12 descents from 8 m: 3199 J, sd 15 J, 14.3 s
mean. The climb to 8 m costs about 1.94 kJ in 8.8 s; it is outside the
model, as the explorer climbs once.

### Validation, never in the fit

Window from the start of the hover to the land command; "with land" adds
E_land to the prediction and the descent to the measurement.

| run | from measured t, d | from the plan only | plan, with land |
|---|---|---|---|
| val_sq30_h60_r1 | -0.17 % | -0.37 % | -0.37 % |
| val_sq30_h60_r2 | -0.18 % | -0.37 % | -0.26 % |
| val_zig_h10_r1 | -0.68 % | -0.99 % | -0.87 % |
| val_zig_h10_r2 | -0.69 % | -1.00 % | -0.78 % |

"From the plan only" uses d = path length and t = hold + post + planned
profile time + 0.06 s, the inputs C1 has before flying.

### Exploratory, not adopted: a cost per leg (scripts/fit_per_leg.py, runs/fit_per_leg.txt)

Every leg carries the same two ramps whatever its length, so a fixed
energy per leg is the candidate for the structured error:
E = P_hover t + k_move d + c n_legs.

| | two terms | three terms |
|---|---|---|
| P_hover | 286.43 +/- 2.03 W | 285.64 +/- 0.12 W |
| k_move | 0.885 +/- 0.563 J/m | -0.400 +/- 0.042 J/m |
| c per leg | n/a | 65.2 +/- 1.3 J |
| transit errors, calibration | -4.2 % to +0.6 % | -0.13 % to +0.16 % |
| square 30 m, from plan | -0.37 % | -0.26 % |
| zigzag, 8 legs, from plan | -1.00 % | -0.14 % |

Every calibration square has 4 legs, so inside the fit c cannot be told
from a constant per transit. The zigzag, with 8 legs and outside the fit,
separates the two: a per transit constant, with the same fitted values,
would predict it 4 x 65 J lower, about 1.1 % low from measured t and d. Its error drops from -1.00 % to -0.14 %: the cost is per leg.

k_move turns negative once the ramps are accounted for: cruising at 5 m/s
draws 2.0 W less than hovering in this simulator. Measured, origin not
established.

## Conclusions

Measured. Gate C passes: the S10 two-term model predicts trajectories it
was not fitted on within 1.0 %, against the 10 % bar, from the plan
alone, including the 60 s hover that refuted E = k d. P_hover matches the
pure hover within 0.19 %.

Measured. Its bias is optimistic, the wrong direction for a guarantee
(spec 5.3): -0.37 % to -1.00 % on validation, -4.2 % on 10 m transit
segments. It grows with the number of legs, about 65 J per rest-to-rest
leg, which the third term removes.

Measured. In this simulator the energy is almost all time: k_move d is
1.4 % of a 60 m square transit (two-term fit). Distance costs energy
mainly through the time spent covering it.

Assumed. The 95 % intervals come from ordinary least squares, which
assumes independent errors. Repetitions agree to 0.02 %, the simulation
being close to deterministic, so the intervals measure the misfit of the
model form, not measurement noise, and three repetitions are not three
independent samples.

Not tested. Other speeds and other ramps: the model holds for the profile
it was fitted on, 5 m/s and 3 m/s2, rest to rest at every vertex. The
trajectory executor of the explorer must fly that profile, or the model
must be calibrated again. Speed dependence (S10) is not measured.

To decide. Adopt the per-leg term in S10 and C1, n_legs being known from
the plan, or keep the two-term form and cover its optimistic bias with
the reserve R.

Figure F8: runs/calibration_fit.png and .pdf (scripts/plot_calibration.py).
Left, mean power against mean ground speed per segment with the two-term
fit and its 95 % band; the three repetitions overlap exactly. Right,
prediction error per segment against duration, with the 10 % bar.

## Files

MANIFEST.tsv lists every file with its size and SHA-256. Check with:

    tail -n +2 MANIFEST.tsv | awk -F'\t' '{print $1 "  " $3}' | sha256sum -c

Reproduce, from a shell whose command line contains none of the cleanup
patterns of session_cal.sh:

    cd ~/ros2_ws/droneAfter/experiments/2026-10-19_energy_calibration
    bash scripts/run_campaign.sh
    python3 scripts/analyse_calibration.py runs
    python3 scripts/tracking.py runs/cal_* runs/val_*
    python3 scripts/fit_per_leg.py runs
    python3 scripts/plot_calibration.py runs

Note: runs/ already holds the flights; run_campaign.sh skips any flight
whose directory exists, so move runs/ aside to fly the campaign again.
