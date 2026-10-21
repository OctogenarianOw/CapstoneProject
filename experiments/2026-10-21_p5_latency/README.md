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

Three flights at the first good attempt (`runs/r1` to `r3`; the three
first attempts of r1 hung on a recorder stopped with SIGINT, kept as
`failed/*_sigint`). Bags of 0.94 GB each under
`~/uav_bags/2026-10-21_p5_latency`, hashes in `runs/rN/bag.sha256`.

| flight | replay at lag 0 vs cloud_world, median / p95 | A(0) | D* | pose used - scan stamp (`stamp_lag.py`) |
|---|---|---|---|---|
| r1 | 6.9 / 18.1 mm | +0.48 | 0.118 s | no /clock |
| r2 | 2.4 / 15.8 mm | +0.47 | 0.110 s | 0.028 s |
| r3 | 5.0 / 66.5 mm | +0.45 | 0.119 s | 0.032 s |

**The map is smeared along the flight by a late pose.** The replay
reproduces the live map, every flight shows the direction-dependent
spill (A about +0.47 of the plate height, north edges when flying north,
south edges when flying south), and taking the pose 0.110 to 0.119 s
earlier removes it in every flight. At 5 m/s that is 0.55 to 0.60 m,
half a cell, the order of P5. Measured too: PX4 stamps its poses on the
system clock in this SITL (a pose arrives 4 to 8 ms after its stamp, at
50 Hz), and a scan reaches cloud_world 0.047 s after its stamp, with the
simulation at real time (factor 1.000 and 1.007).

**The stamps account for a quarter of it.** The latest pose received
is 0.028 to 0.032 s after the scan stamp, not 0.11 s. The criterion
fixed before the flights (agreement within one lidar period, 0.1 s) is
met on r2 and r3 as written, but it was too loose to tell: about
0.08 s of the lag lies between the geometry of a scan and its stamp, or
between the PX4 estimate and the true pose, and these flights cannot
say which. Consequence for the fix: interpolating the pose at the scan
stamp alone would remove about a quarter of the smear. remap.py's
single clock offset fails when the simulation pauses (r3: the offset
jumped once by about 0.85 s); `stamp_lag.py` takes it within 2 s of each
scan.

Next measurement, not flown: the true pose of the aircraft from Gazebo
against the PX4 estimate, to place the 0.08 s.
