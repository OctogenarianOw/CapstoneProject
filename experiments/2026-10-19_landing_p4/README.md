# Landing precision (S16, P4) and the landing energy in closed loop (S10), 19 Oct 2026

Two questions, one experiment:

1. P4: how far from its setpoint does the aircraft touch down? The
   1.5 m footprint of the detector rested on an estimate (0.70 m span,
   0.40 m of landing dispersion each side), never measured.
2. Does E_land, 3200 J calibrated on the descents of the energy
   calibration, hold for the landing the manager actually commands? The
   autonomous missions had logged more.

## Protocol

`land_drive.py` plays the manager's part at the end of a mission: take
off to 8 m, hover 2 s, fly a 15 m out-and-back return path (0, 0) ->
(15, 0) -> (0, 0) on the calibrated profile, and command LAND the
moment the executor reports arrival, as `MissionFSM._commit` does. It
records the setpoint, the position at the LAND command and at the
disarm, the energy between the two from the energy monitor, and the
time. 15 landings, one fresh PX4 session each (`sim_session.sh`,
parameters reset), headless, only the executor and the energy monitor
running. 0 failed sessions, 0 failsafes. Code c7fc2aa (14 sessions
recorded HEAD 1b3df10, a documentation commit with the same `src/`).

    bash run.sh 15
    python3 summarise.py > summary.txt

## Precision

| quantity | mean | sd | p95 | max |
|---|---|---|---|---|
| radial touchdown error to the setpoint | 0.124 m | 0.045 m | 0.180 m | 0.195 m |
| error at the LAND command, from the executor arrival test | 0.128 m | 0.021 m | 0.158 m | 0.165 m |
| drift during the descent | 0.055 m | 0.038 m | 0.113 m | 0.166 m |

Most of the error is already there when LAND is sent: the executor
declares arrival within 0.5 m and below 0.3 m/s, and stops at about
0.13 m. PX4 then descends nearly vertically, 5 cm of drift on average.

Airframe radius, measured in the PX4 `x500_base` model: rotor hubs at
(+-0.174, +-0.174) m, 0.246 m from the centre, blade collision box
0.279 m long, so 0.386 m to the blade tip (the spec estimated 0.35 m).

    footprint = 2 (airframe radius + p95 error) = 2 (0.386 + 0.180)
              = 1.131 m, rounded up to 1.2 m, against 1.5 m in use

The footprint in use is conservative by 0.3 m. Changing it changes the
detector and every figure measured with it (recall 94.2 % and 0 severe
false accepts on v6, the minimum detectable zone of S8); that is
Dorian's decision, so it is not changed here. The next step if it is
taken: rerun `tools/validate_slz.py` on v6 with 1.2 m and report recall,
false accepts and the minimum zone size against 1.5 m.

Limits: 15 landings on flat ground, no wind, all from the same 15 m
return; SITL position from the EKF with simulated GPS. The p95 of 15
values is the 14th or 15th of them, so 0.180 m is itself uncertain by a
few centimetres; the rounding up to 0.1 m covers it.

## Landing energy

| landings | n | mean | sd | p95 | max | against E_land 3200 J |
|---|---|---|---|---|---|---|
| this experiment, headless, executor and monitor only | 15 | 3194 J | 17 J | 3218 J | 3224 J | -0.2 % mean, +0.6 % p95 |
| W1 campaign, committed landings, headless | 70 | 3296 J | 63 J | 3379 J | 3460 J | +3.0 % mean, +5.6 % p95 |
| registry flights, headless | 4 | 3285 J | 72 J | 3345 J | 3346 J | +2.7 % mean |
| sessions with RViz and the Gazebo window, or slow decisions | 10 | 3603 J | 215 J | 3958 J | 4139 J | +12.6 % mean, +23.7 % p95 |

`mission_landings.py` measures the mission landings from the logs: the
land action and the LANDED state in `events.jsonl`, the energy left
interpolated between cycles; the energy left does not change after the
disarm (checked up to 716 s past it). Emergency landings are excluded
(B0_r02 touched down on the 10 deg ramp, 2.25 kJ in 10.9 s).

E_land holds for the landing as the calibration flew it, here
reproduced within 0.2 %. The same landing inside a mission costs 3 %
more on a headless run, and 13 % more on a loaded one; its duration
grows too (14.3 s here, 14.6 s in the campaign, 16 to 17 s loaded). The
landing itself is PX4's (NAV_LAND), so the extra time is most likely
between the LAND command and the start of the descent, which grows
with the CPU load of the session (the executor resends the command at
1 Hz until PX4 leaves offboard). That cause is a hypothesis, not
measured; PX4's own log (vehicle_command against vehicle_status) can
settle it.

What it means for C1: E_land is optimistic by 3 % on average, 5.6 % at
the p95, in the conditions of the campaign, about 100 to 180 J, inside
the 2 kJ reserve; on a loaded machine up to 0.94 kJ. A conservative
E_land, the p95 of the landings flown in the conditions of the
experiment, is the candidate correction; it is a change of the energy
model, to be made and measured on its own.

## Files

    land_drive.py          one landing as the manager commands it
    run.sh                 N sessions, failed ones flown again
    summarise.py           precision, footprint, energy
    summary.txt            its output for the 15 landings
    mission_landings.py    landing energy of logged missions
    mission_landings.txt   its output by kind of session
    sessions/land_NN/      setup.txt, px4.log, node logs, landing.json
