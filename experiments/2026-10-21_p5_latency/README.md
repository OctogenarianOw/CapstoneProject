# P5: a pose taken after the scan? 21 Oct 2026

P5 (spec section 10): a one-cell offset to the north between the map and
the truth on the ramps of W1, never explained. `cloud_world` takes, for
each lidar scan, the latest pose received, not the pose at the time of
the scan, and its header names that as a candidate cause, untested.

What the pit calibrations showed on the way (their maps, 6 flights):
the sharp north and south edges of the plates spill into the first row
of ground past them, and only on one side, the side the aircraft was
flying towards on its last line:

| map after | flying | north spill | south spill |
|---|---|---|---|
| line 3 | north | 0.35 to 0.36 | 0 |
| line 4 | south | 0 | 0.34 to 0.37 |
| line 5 | north | 0.45 to 0.50 | 0 |

(fraction of the plate height, mean of the four plates, flights r1 to r3
of `2026-10-21_pit_calibration`; the same on r1 of the second one.)
A pose ahead of the scan moves every point forward along the flight:
half a cell at 5 m/s is a pose about 0.1 s late, one lidar period.

## Measurement

`run.sh`: the flight of the first pit calibration, with a rosbag of the
lidar and the PX4 pose (bags out of the repository, their SHA-256 in
each run directory). `remap.py`: the map rebuilt offline with the chain
of `cloud_world`, the pose taken at the reception time of the scan
minus a lag D, one map per direction of flight, and the
direction-dependent part A(D) of the edge spill; plus the lag read on
the stamps, sample time of the pose against stamp of the scan.

## Criterion, fixed before the flights

The hypothesis stands if the replay at D = 0 reproduces the map
`cloud_world` wrote, A(0) is positive in every flight, a lag D* in
[0, 0.4] s brings A to zero, and D* agrees with the lag read on the
stamps within one lidar period (0.1 s). It is refuted if no lag of the
sweep brings A to zero. No fix is made here: a change of the map changes
every result measured with it, and is Dorian's decision.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-21_p5_latency
    bash run.sh 3
    python3 remap.py ~/uav_bags/2026-10-21_p5_latency/rN runs/rN    # ROS sourced

## Results

To be filled after the flights.
