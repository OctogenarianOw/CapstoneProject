# P5 fix, before and after, 21 Oct 2026

`2026-10-21_p5_latency` measured why the map is smeared along the
flight: cloud_world fuses each scan with a pose 0.110 to 0.119 s ahead
of the scene, 0.03 s because the latest pose arrives after the scan and
0.09 to 0.11 s because the PX4 estimate is ahead of the true position.
Branch `p5-interp` of `~/uav_dev` (not merged) takes the pose at the
scan time: interpolated at the scan stamp through /clock, minus a
temporal calibration term of 0.085 s (`uav_core/pose_buffer.py`,
`cloud_world.py`, tests 106 passed). This measures it.

## Set-up

One change: the parameter `pose_at_scan_time` of the branch's
cloud_world, false (the behaviour of dronePropre) before, true after.
Both run with /clock bridged and the same code, through PYTHONPATH over
the droneAfter install; each run directory holds the branch commit.
Three runs of each of four sessions, interleaved (`run.sh`):

- A, world `pits`: the flight of the first pit calibration, plate
  edges after line 4 (flown south) and line 5 (flown north).
- B, world `validation` (W1): the survey of the reference map v6, then
  the tools that measured P5 and the safety of the detector.

## Criterion, fixed before the flights

The fix works if, in every flight after, the direction-dependent spill
|A| is under 0.1 (about +0.45 before) with more than 95 % of the scans
fused at the scan time, and if on W1 the best ramp shift of
`check_alignment.py` is (0, 0) after, with a ramp RMS at (0, 0) lower
than before. The flat RMS (S5, 0.25 m) and the severe false accepts
(S8, barrier 0) must not get worse. Otherwise it is reported as it is,
and nothing is merged in any case: merging is Dorian's decision.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-21_p5_fix
    bash run.sh 3
    python3 analyse.py > results.txt     # ROS and the workspace sourced

## Results

18 sessions, all at the first attempt (`flights*.log`, `results_v1.txt`,
`results_v2.txt`, runs listed by tag).

### Before (pose_at_scan_time false, the behaviour of dronePropre)

| | value, 3 runs |
|---|---|
| plates, A | +0.41, +0.43, +0.41 |
| W1 ramp RMS at (0, 0), best shift | 0.34 m, (-1, 0) in 3 of 3: **P5 reproduced on fresh surveys** |
| W1 flat RMS, recall | 0.22 to 0.25 m, 93 to 94 % |
| W1 severe false accepts | **2, 3 and 4**, against 0 on v6 |

The S8 barrier fails on the three fresh surveys flown today with the
code of dronePropre: zero severe false accepts was a property of v6,
not of the chain. Where they sit is not yet measured.

### After: the fix does not pass, and the reason is the clock of WSL

| version | scans at the scan time | plates, A | W1 best shift | W1 recall | W1 severe |
|---|---|---|---|---|---|
| v1 (e92a3e1) | 54 to 74 % | +0.25, +0.05, +0.31 | (-1,0), (0,0), (-1,0) | 63 to 77 % | 0, 1, 0 |
| v2 (27a94ac) | 44 to 71 % | +0.03, +0.26, +0.32 | (-1,0), (-1,0), (0,-1) | 63 to 82 % | 0, 0, 0 |

Neither meets the criterion, and the recall falls: a map mixing scans
placed with two different lags is worse than a map with one. v2 was a
guess (the callback waiting behind the fusion of a scan) made before
measuring, and it changed nothing. The diagnosis (`0a0d5ed`, runs
`*_diag_r1`) logged every fallback: all were "after the last position",
by 0.53 to 0.60 s with a full buffer, and the offsets stepped. Then
measured directly in WSL, outside any simulation:

    13.8 s: realtime - monotonic stepped by -0.686 s
    45.7 s: realtime - monotonic stepped by -0.706 s
    total over 70 s: -1.392 s

**The system clock of WSL is stepped back by about 0.7 s every 32 s**
(the virtual machine's clock runs about 2 % fast and the host resets
it). It started between 14:53 (bag r2 of the latency flights: no step)
and 14:57 (r3: steps). PX4 SITL stamps on that clock and the simulation
does not, so the offset between them jumps at every step, a message
stamped before a step and received after it seems to come 0.7 s from
the future, and the minimum over a window keeps that outlier for the
length of the window: the fallbacks come in blocks of about 16 s.

What it touched: this measurement of the fix, and the tail of 2 to 3 m
errors of the ground-truth flights g1 and g2 (the medians stand). Not
the W2 campaign (09:41 to 13:16): no cycle of its 8724 consumed the
energy of a 1.7 s real cycle. Not the latency flights r1 and r2, whose
lags agree with r3's. Not the old cloud_world, which never compares two
clocks.

What it means: the fix needs estimators that stand clock steps (a
median over a short window rather than a minimum, and a pose buffer
that restarts when the PX4 stamps go back), and every flight should
record whether the clock stepped while it flew, since the decision
latency that C1 charges is measured on that clock too. A backward step
during a visit would make C1 charge 0.7 s too little.

### v3: estimators that stand the steps (548aa15)

Median over 2 s for the PX4 offset, minimum over 0.3 s for the
simulation offset, buffer restarted when PX4 stamps go back. Replayed
offline on bag r3 (steps present): 2487 of 2557 scans at the scan time
against 2259 for the first version. In flight (`results_v3.txt`):

| | scans at the scan time | plates, A | W1 ramp RMS (0,0), best shift | W1 flat RMS | W1 recall | severe |
|---|---|---|---|---|---|---|
| before | 0 % | +0.41 to +0.43 | 0.34 m, (-1,0) x 3 | 0.22 to 0.25 m | 93 to 94 % | 2, 3, 4 |
| v3 | 94.7 to 97.4 % | -0.01, -0.49, -0.67 | 0.18 to 0.22 m, (0,0) x 2, (0,-1) | 0.37 to 0.55 m | 51 to 56 % | 0, 0, 0 |

**P5 goes with the pose taken at the scan time**: on the ramps of W1
the best shift becomes (0, 0) and the RMS at (0, 0) falls from 0.34 to
0.18-0.22 m, which confirms the mechanism on the map that defined P5.
**But the map gets worse elsewhere and the fix fails its criterion**:
the flat RMS rises, the recall halves, and on the plates the spill
turns to the other side of the flight and grows. Some scans are now
placed with a pose too early, which the steps of the clock (and the 2 %
drift between them) can explain: the 0.085 s term and the clock
relation were measured under another clock regime. Not measured
further. A fair test of the design needs a stable clock (WSL restarted,
or the native Linux machine of the final demonstration). Not merged.
