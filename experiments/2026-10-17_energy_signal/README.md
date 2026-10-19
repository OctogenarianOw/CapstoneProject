# Energy signal available in SITL, 17 Oct 2026

Question: before running the S10 calibration campaign, does the simulator
provide an energy signal that depends on what the aircraft does?

Setup: PX4 v1.17.0-alpha1-1603-g242d63f437, Gazebo world `validation`,
airframe 4030_gz_x500_slz (SIM_BAT_DRAIN 3000, SIM_BAT_MIN_PCT 95),
HEADLESS=1, real time factor 1.0 (sim span equals wall span).

## Runs

| run | node | what it flies |
|---|---|---|
| run1_offboard_demo_hover5m | uav_core offboard_demo | hold 5 m, 218 s armed |
| run2_survey_mission | uav_core survey_mission | default survey, 8 m, 324 m flown |
| run3_hover_hold_8m | scripts/hover_hold.py | climb to 8 m, hold 60 s, land |

Each run directory holds rotors.csv (rotor speed commands from
`/x500_slz_0/command/motor_speed`, gz.msgs.Actuators, rad/s, stamped with
the Gazebo `/clock`), battery.csv (`/fmu/out/battery_status_v1`),
state.csv (`/fmu/out/vehicle_local_position_v1` plus arming and nav state),
and the topic lists seen during the run.

MANIFEST.tsv lists every file of this campaign with its size and SHA-256,
raw data included. The raw files are stored byte for byte (see
experiments/.gitattributes); the CSV files keep the CRLF line ends that
Python's csv module wrote. Check the copy with:

    tail -n +2 MANIFEST.tsv | awk -F'\t' '{print $1 "  " $3}' | sha256sum -c

Reproduce one run, then analyse it:

    bash scripts/hover_energy_check.sh OUT_DIR 110 hover_hold.py "8.0 60"
    python3 scripts/analyse_energy.py OUT_DIR

The session script kills every process it started on exit, with pkill -f
patterns. Launch it from a shell whose own command line does not contain
those patterns, or the cleanup kills the caller.

## Results

PX4 battery telemetry, measured in all three runs and explained by
`src/modules/simulation/battery_simulator/BatterySimulator.cpp`:

| field | value while armed |
|---|---|
| current_a | -1.0 always, the simulator has no current sensor |
| discharged_mah | 0.0 always, PX4 skips integration when current is -1 |
| remaining | linear in armed time, slope -3.40e-4 per s, flat at 0.9501 after about 150 s |

Rotor shaft power, P = k_m * k_f * sum(w^3), with k_f = 8.54858e-06 and
k_m = 0.016 from the x500 model:

| quantity | predicted from m g = 4 k_f w^2, m = 2.264 kg | measured, run3 hover |
|---|---|---|
| hover rotor speed | 806.0 rad/s | 805.5 rad/s, sd 0.6 |
| hover shaft power | 286.5 W | 285.9 W |

| run | airborne | horizontal distance | shaft energy | hover power x airborne time |
|---|---|---|---|---|
| run2 survey | 47.4 s | 324.0 m | 15.56 kJ | 13.55 kJ, 87 % |
| run3 hover | 74.0 s | 1.6 m | 21.04 kJ | 21.16 kJ, 101 % |

## Conclusions

Measured. PX4 battery_status in this SITL carries armed time only. A
regression of it on distance would recover armed time divided by mean
speed, not a consumption law.

Measured. Rotor shaft power computed from the Gazebo rotor commands is
load dependent and matches the hover force balance to 0.2 %.

Measured, one flight each. A pure E = k d model is falsified: k fitted on
the survey is 48 J/m and predicts 77 J for the hover run, which used
21 kJ. Hover power times time explains 87 % of the survey energy.

Assumed, to be tested by the calibration campaign. E = P_hover t + k_move d
predicts unseen trajectories within 10 %.

Not decided. Whether the thesis energy budget is defined as rotor shaft
energy. Shaft energy differs from battery energy by the motor and ESC
efficiency, which is not modelled here.

Update, 19 Oct 2026. Decided since, in the specification, section S10:
the budget is rotor mechanical energy, converted to electrical energy by
one declared constant efficiency of 0.70 to 0.75, with a sensitivity
analysis. The campaign moved from ~/ros2_ws/experiments into the
droneAfter repository on the same day, data and scripts unchanged.

Update, 19 Oct 2026. run3_hover_hold_8m, the source of the 285.9 W hover
reference, ran on a Gazebo server left over from run2: its px4.log reads
"gazebo already running world: validation". The cleanup patterns of
hover_energy_check.sh miss the server, whose command line is "gz sim ...".
run1 and run2 started a fresh world. The fresh-world sessions of
2026-10-19 check the hover power again.
