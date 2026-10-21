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

To be filled after the flights.
