# Logical energy budget of the mission, published by energy_monitor (S10).
#
# Units are rotor shaft joules, the quantity the energy model predicts.
# The budget is logical, held in software, not the simulated PX4 battery,
# whose telemetry in SITL only measures armed time.

std_msgs/Header header       # stamp of publication

float64 capacity_j           # budget at the start of the mission
float64 consumed_j           # shaft energy integrated since start
float64 remaining_j          # capacity_j - consumed_j
float64 power_w              # latest shaft power
uint32 samples               # rotor messages integrated
uint32 gaps                  # gaps longer than max_gap_s, not integrated
float64 last_sample_age_s    # time since the latest rotor message
