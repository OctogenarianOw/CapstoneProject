#!/bin/bash
# B5. The confirmation guard of 20 Oct (a zone is confirmed only if it
# holds one whole footprint element of landable cells) was chosen on the
# 80 last maps of 2026-10-19_campaign_w1 and checked on the 80 of
# 2026-10-20_campaign_w1_fix. This replays the same measurement, with
# the same script, on a third sample flown later with the final code:
# the 70 last maps of 2026-10-23_campaign_w1_final (all configurations).
# Read-only on data_survey_v6.npz.
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/droneAfter/install/setup.bash
cd ~/ros2_ws/droneAfter
python3 experiments/2026-10-20_min_zone/measure.py data_survey_v6.npz \
  --runs experiments/2026-10-23_campaign_w1_final/runs \
  > experiments/2026-10-31_report_analyses/b5_guard_final.txt 2>&1
cat experiments/2026-10-31_report_analyses/b5_guard_final.txt
