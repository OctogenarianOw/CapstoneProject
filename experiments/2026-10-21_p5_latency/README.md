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

### Where the other 0.08 s is: the PX4 estimate against the truth

Two ground-truth flights (`run_gt.sh`, `gt_lag.py`), the same flight
with the true pose of the aircraft from Gazebo: g1 through the ROS
bridge, which drops the names and stamps of the poses (timed by their
reception), g2 through Gazebo's own echo with the simulation stamps
(`gt_echo.sh`), put on the system clock by the local /clock offset.

| flight | truth timed by | shift at the median error minimum | lead from the along-track error, north / south legs |
|---|---|---|---|
| g1 | reception | -0.108 s (0.14 m) | 0.091 s (+0.54 / +0.45 m) |
| g2 | simulation stamp | -0.108 s (0.11 m) | 0.091 s (+0.46 / +0.49 m) |

**The PX4 position, on the system clock, is ahead of the true position
by 0.09 to 0.11 s.** Hence the map lag: the pose used is 0.03 s after
the scan stamp (`stamp_lag.py`) and the estimate a further 0.09 to
0.11 s ahead, 0.12 to 0.14 s in all, against D* = 0.110 to 0.119 s
found on the map. What is left for a scene older than its stamp is
about zero: the simulated lidar is on time. This split does not depend
on the clock offset estimated from /clock: an error there moves the two
terms by the same amount in opposite directions. Not measured, the
reason of the lead; a likely one is the offset PX4 SITL takes between
the simulation clock and its own when it starts.

The RMS of the horizontal error put its minimum at the wrong sign
(+0.12 s): about 10 % of the samples err by 2 to 3 m where the clock
offset jumps as the simulation pauses. The median is used.

### For the fix (Dorian's decision, nothing merged)

Interpolating the pose at the scan stamp corrects the 0.03 s that
varies with the load of the machine. The 0.09 to 0.11 s is a constant
between two time bases, the same in both ground-truth flights and
consistent with the three map lags; it is a temporal calibration term,
as between any sensor and an estimator, to be measured once in SITL
and declared. A fix with both, and its measurement before and after on
the plates and on W1's ramps (P5), is prepared on branch `p5-interp`
of the worktree `~/uav_dev`, not merged: it changes the map, so every
result measured with the old map would have to be taken again, or
declared as measured before the fix.
