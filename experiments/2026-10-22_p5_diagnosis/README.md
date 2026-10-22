# P5 diagnosis: which part of the pose at the scan time fails? 22 Oct 2026

`2026-10-22_p5_stable_clock`: the fix v3 (branch p5-interp, 548aa15,
pose interpolated at the scan stamp minus a calibration term of
0.085 s, for the position and the attitude) fails on a stable clock:
on the plates the spill changes side (A +0.44 -> -0.25), on W1 the flat
RMS rises (0.25 -> 0.30 m) and the recall falls (94 -> 61 %), while P5
and the severe false accepts go. Hypothesis written there, not
measured: the term, measured on the position, is applied to the
attitude too.

## 1. Plates, offline (`variants.py`)

The bags of `2026-10-21_p5_latency` that hold /clock (r2 on a clean
clock, r3 recorded while the WSL clock stepped) rebuilt with the chain
of cloud_world, the pose taken five ways, each differing from the one
before by one thing, the clock offsets estimated as v3 does
(`variants_r2.txt`, `variants_r3.txt`).

| r2 | A | ground RMS |
|---|---|---|
| latest (dronePropre) | +0.458 | 0.027 m |
| v3, both at stamp - 0.085 s | -0.276 | 0.027 m |
| attitude without the term | -0.298 | 0.026 m |
| attitude latest | -0.339 | 0.027 m |
| both at the stamp, no term | +0.391 | 0.025 m |

The replay of latest equals the live map (median difference 0.0 mm);
v3 offline gives the A of v3 in flight. **The hypothesis is refuted**:
A follows the term on the position and nothing else, and no variant
spoils the flat ground of the plates. **The term overshoots**: between
no term (+0.39) and 0.085 s (-0.28), A crosses zero near 0.05 s; from
the lags of 2026-10-21_p5_latency (pose received 0.115 s after the scene,
a cloud 0.047 s after its stamp, a pose 5 ms after its own) the term
should be about 0.068 s. r3, recorded during the clock steps, shows the
flat ground spoilt by the interpolated attitude (0.075 against 0.030 m),
which a stable clock does not: not used.

What the plates do not explain: the loss of recall on W1. The plate
flight is flown by trajectory_executor, the W1 survey by survey_mission.

## 2. W1, offline on a survey bag (`w1bag/`)

`w1bag/run.sh`: the W1 survey of 2026-10-22_p5_stable_clock (survey
_mission, cloud_world of dronePropre, /clock bridged) with a rosbag of
the lidar, the PX4 pose and /clock (out of the repository, SHA-256 in
the run directory). `w1bag/sweep.py` rebuilds the whole W1 grid from
it: latest, and the pose at the scan time minus T for T in 0, 0.03,
0.05, 0.068, 0.085 s, with the attitude at the same time or the latest
received; each map scored as in 2026-10-22_p5_stable_clock (ramp RMS
and best shift, flat RMS, recall, severe false accepts).

Criterion, fixed before the flight: a variant is a candidate fix if, on
the survey bag, the best ramp shift is (0, 0), the ramp RMS at (0, 0)
is lower than latest's, the flat RMS is under 0.25 m, the recall no
lower than latest's minus 2 points, and no severe false accept. The
candidate with the smallest ramp RMS is then built and flown as in
2026-10-22_p5_stable_clock (plates and W1, 3 before and 3 after), with
that experiment's criterion, before anything is merged.

## Results

Two W1 survey bags (`w1bag/runs/r1`, `r2`, 18:58 and 19:03, no clock
step; bags out of the repository, SHA-256 in the run directories),
swept offline (`w1bag/sweep_r1.txt`, `sweep_r2.txt`). The replay of
latest equals the live map (same ramp RMS, shift, flat RMS, recall,
severe count).

| r1 / r2 | ramp RMS (0,0) | best shift | flat RMS | recall | severe |
|---|---|---|---|---|---|
| latest (dronePropre) | 0.341 / 0.340 | (-1, 0) | 0.246 / 0.245 | 93.4 / 92.9 % | 1 / 3 |
| 0.085 s, attitude interpolated (v3) | 0.190 / 0.189 | (0, 0) | 0.311 / 0.311 | **60.7 / 60.6 %** | 0 / 0 |
| 0.050 s, attitude interpolated | 0.182 / 0.181 | (0, 0) | 0.253 / 0.257 | 79.7 / 83.1 % | 0 / 0 |
| **0.050 s, attitude latest** | **0.176 / 0.177** | **(0, 0)** | **0.237 / 0.238** | **94.8 / 95.8 %** | **0 / 0** |
| 0.068 s, attitude latest | 0.163 / 0.165 | (0, 0) | 0.234 / 0.242 | 94.5 / 95.0 % | 0 / 0 |
| 0.085 s, attitude latest | 0.154 / 0.154 | (0, 0) | 0.240 / 0.248 | 94.8 / 95.5 % | 0 / 0 |

**On the W1 survey the interpolated attitude is what destroys the
recall**: offline, v3 gives the 60.6 to 60.7 % it gave in flight (60 to
64 %), and the same term with the attitude last received keeps the
recall at 94.5 to 95.8 % and the flat RMS under 0.25 m, while P5 goes
(shift (0, 0)) and the severe false accepts with it (1 and 3 -> 0). The
plates, flown by another node (trajectory_executor), did not show it.
Cause of the attitude effect not measured.

**Choice of the term, a deviation from the rule above, stated.** The
rule fixed before the flight picks the smallest ramp RMS, 0.085 s. But
the flight validation that follows also asks |A| < 0.1 on the plates,
and the plate bags predict, by the linear trend between no term
(+0.391) and 0.085 s with the attitude last received (-0.339), A about
-0.34 at 0.085 s and -0.19 at 0.068 s: both would fail, and A crosses
zero near 0.045 s. At 0.050 s (predicted A about -0.04) W1 still passes
every test of the rule on both bags. The plate spill measures the
along-track shift directly; the ramp RMS mixes it with the shrinking of
edges (P2). **Candidate v4: position at the scan stamp minus 0.050 s,
attitude last received**, to be flown with the protocol and criterion
of 2026-10-22_p5_stable_clock.
