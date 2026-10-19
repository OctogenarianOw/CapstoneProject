# Trajectory executor, live, 19 Oct 2026 (S4)

live_drive: trajectory_executor, energy_monitor and energy_probe_v2,
driven by drive.py in the manager's place: take off to 8 m, hover 5 s,
20 m square, hover 5 s, 40 m line preempted after 4 s by a path to
(20, 20) then home, hover 5 s, land. Fresh PX4 world, 0 failsafes.

Per phase (analyse.py, live_drive/analyse.txt):

| phase | planned | flown | peak speed | peak accel | altitude |
|---|---|---|---|---|---|
| square | 80.0 m, 22.7 s | 82.8 m (+3.5 %), 22.8 s | 5.59 m/s | 3.89 m/s2 | 7.98 to 8.05 m |
| preempted path | 48.6 m, 13.1 s | 52.8 m (+8.6 %), 13.2 s | 5.37 m/s | 7.48 m/s2 | 7.98 to 8.18 m |

Square energy, predicted by energy_model from the plan against the
monitor from path command to arrival: 6720 J against 6742 J, -0.33 %,
in line with the calibration.

Arrival was declared on the real distance to the end of each path (S4),
and the preempted path started from the setpoint mid-line, as designed.
Known limit: the new path starts from rest while the aircraft flies at
5 m/s, so PX4 brakes at up to 7.5 m/s2 and overshoots; the flown length
exceeds the plan by 8.6 % and the energy of a preempted path is
underestimated. Preemption is rare by design (S9.5 reactions); a start
at the current velocity is the fix if it matters.

    cd ~/ros2_ws/droneAfter/experiments
    bash sim_session.sh 2026-10-19_executor/live_drive 200 \
      mon="ros2 run uav_core energy_monitor" \
      exe="ros2 run uav_core trajectory_executor" \
      probe="GZ_IP=127.0.0.1 python3 CAL/energy_probe_v2.py . /x500_slz_0/command/motor_speed 220" \
      drive="sleep 6; python3 2026-10-19_executor/drive.py; sleep 3; touch stop"
    python3 2026-10-19_executor/analyse.py 2026-10-19_executor/live_drive
