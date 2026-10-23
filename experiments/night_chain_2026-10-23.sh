#!/bin/bash
# After the final W3 and W2 campaigns (final_chain_2026-10-23.sh): the
# S9.5 depot flights, then the final W1 campaign, then
# night_23oct.done for the keep-awake helper. Waits for
# autonomy_23oct.done, then for night_go (written once the results of
# W3 and W2 are committed, so these flights record a clean head), at
# most 20 min. Started from Windows as its own wsl.exe process
# (trap 23).
#   bash night_chain_2026-10-23.sh
EXP=$(cd "$(dirname "$0")" && pwd)
LOG="$EXP/night_chain_2026-10-23.log"
until [ -e "$EXP/autonomy_23oct.done" ]; do sleep 30; done
echo "[$(date +%T)] W3 and W2 done, waiting for night_go" >> "$LOG"
for i in $(seq 1 40); do [ -e "$EXP/night_go" ] && break; sleep 30; done
echo "[$(date +%T)] S9.5 depot flights, git $(git -C "$EXP/.." rev-parse --short HEAD)," \
  "$(python3 "$EXP/../tools/wsl_clock_tick.py")" >> "$LOG"
cd "$EXP/2026-10-23_s95_depot" && bash run.sh 30 4 >> flights.log 2>&1
source /opt/ros/jazzy/setup.bash
source "$EXP/../install/setup.bash"
python3 analyse.py > results.txt 2>&1
echo "[$(date +%T)] S9.5: $(tail -1 flights.log)" >> "$LOG"
echo "[$(date +%T)] W1 final campaign" >> "$LOG"
bash "$EXP/campaign_chain.sh" "$EXP/2026-10-23_campaign_w1_final" 10
echo "[$(date +%T)] W1 done: $(tail -1 "$EXP/2026-10-23_campaign_w1_final/chain.log")" >> "$LOG"
touch "$EXP/night_23oct.done"
echo "[$(date +%T)] end" >> "$LOG"
