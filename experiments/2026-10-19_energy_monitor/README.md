# Online energy monitor against the offline probe, 19 Oct 2026 (S10)

live_check: one controlled flight (climb to 8 m, 30 s hover, 20 m
square at 5 m/s, land) with energy_monitor (rotor commands bridged as
actuator_msgs/Actuators, budget 200 kJ) and energy_probe_v2 (the
calibration instrument, gz transport directly) running side by side.
Fresh PX4 world, 0 failsafes.

| quantity | value |
|---|---|
| probe, integrated offline with rotor_energy | 21946.6 J, 23556 samples, 0 gaps |
| monitor, last /energy/state | 21946.6 J, 24078 samples, 0 gaps |
| difference | 0.000 % |
| hover power over 30.1 s | 285.7 W, calibrated P_hover 285.6 W |

The monitor counts more samples because it ran longer, on the ground
with the rotors stopped, which adds no energy.

    cd ~/ros2_ws/droneAfter/experiments
    bash sim_session.sh 2026-10-19_energy_monitor/live_check 150 \
      mon="ros2 run uav_core energy_monitor" \
      rec="sleep 2; ros2 topic echo --csv /energy/state" \
      probe="GZ_IP=127.0.0.1 python3 CAL/energy_probe_v2.py . /x500_slz_0/command/motor_speed 200" \
      fly="sleep 8; python3 CAL/calibration_flight.py --path '0,0;20,0;20,20;0,20;0,0' --hold 30; sleep 4; touch stop"
    python3 2026-10-19_energy_monitor/compare.py 2026-10-19_energy_monitor/live_check

with CAL = experiments/2026-10-19_energy_calibration/scripts.
