#!/bin/bash
# 24 Oct, after the final W1 campaign (already running through
# campaign_chain.sh): the S9.5 return flights, their analysis, then
# day_24oct.done for the keep-awake helper. Waits for the W1 chain to
# end, then for day_go (written once the W1 results are committed, so
# these flights record a clean head), at most 15 min. Started from
# Windows as its own wsl.exe process (trap 23).
#   bash day_chain_2026-10-24.sh
EXP=$(cd "$(dirname "$0")" && pwd)
LOG="$EXP/day_chain_2026-10-24.log"
W1="$EXP/2026-10-23_campaign_w1_final/chain.log"
until grep -qs "\] end$" "$W1"; do sleep 30; done
echo "[$(date +%T)] W1 done, waiting for day_go" >> "$LOG"
for i in $(seq 1 30); do [ -e "$EXP/day_go" ] && break; sleep 30; done
echo "[$(date +%T)] S9.5 return flights, git $(git -C "$EXP/.." rev-parse --short HEAD)," \
  "$(python3 "$EXP/../tools/wsl_clock_tick.py")" >> "$LOG"
cd "$EXP/2026-10-24_s95_return" && bash run.sh 16 5 >> flights.log 2>&1
source /opt/ros/jazzy/setup.bash
source "$EXP/../install/setup.bash"
python3 analyse.py > results.txt 2>&1
echo "[$(date +%T)] S9.5: $(tail -1 flights.log)" >> "$LOG"
touch "$EXP/day_24oct.done"
echo "[$(date +%T)] end" >> "$LOG"
