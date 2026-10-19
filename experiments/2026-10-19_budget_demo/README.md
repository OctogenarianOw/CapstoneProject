# Policies under a budget that bites, W1, 19 Oct 2026 (S13, 5.2, 5.5 point 3)

run.sh flies one autonomous mission per policy on the validation world,
budget 20 kJ (full coverage cost 32.4 kJ in 2026-10-19_first_autonomous,
so about 60 %, inside the half to two thirds S10 asks for), reserve 10 %
of the capacity (2 kJ), alpha 0.5. Fresh PX4 world per mission, 0
failsafes. Analysis of each with experiments/analyse_mission.py
(analysis.txt in each directory).

| policy | I1 violations | min margin | coverage | end | energy left |
|---|---|---|---|---|---|
| B3 c1_c2 | 0 | +1.60 kJ | 93.2 % | none admissible, return, landed on the take-off zone 0.6 m from its point | +3.5 kJ |
| B1 gain, return at 30 % | 12 | -2.13 kJ | 95.5 % | threshold reached, landed on the take-off zone | -0.2 kJ |
| B0 nearest | 25 | -7.85 kJ | 97.2 % | budget depleted, emergency landing 30.8 m from home, outside the registry (I5 exception) | -3.7 kJ |

With C1 the aircraft never entered a state where the confirmed zone was
out of reach; both baselines did, B1 for 12 cycles before and during its
return, B0 until its budget ran out. C1 costs coverage: 93.2 % against
95.5 and 97.2 %. This is the shape of point 3 of spec 5.5, on one flight
per policy: not yet the campaign of 5.4 (10 runs per configuration,
median and IQR, Wilcoxon).

failed/c1_c2_20kJ_takeoff_lost: the first c1_c2 flight never took off.
The manager had a valid pose before the executor, which refused the
take-off; sent once, the command was lost. Fixed in mission_fsm.py
(commands resent until the executor confirms them); the flight above is
the re-flight, where no retry was needed.

In all three missions the registry held the take-off point and a single
3 m2 confirmed zone: C1 rested on the take-off point (see
2026-10-19_first_autonomous, finding on I4).

    cd ~/ros2_ws/droneAfter/experiments/2026-10-19_budget_demo
    bash run.sh
