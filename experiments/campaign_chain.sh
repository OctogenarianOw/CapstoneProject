#!/bin/bash
# A campaign as a chain of its own: run.sh N in the campaign directory,
# then every analysis its README lists, then DIR/campaign.done for the
# keep-awake helper. Started from Windows as its own wsl.exe process
# (trap 23). Generalises campaign_w3_chain.sh, which named its campaign.
#   bash campaign_chain.sh CAMPAIGN_DIR [N]
DIR=$(cd "$1" && pwd)
N=${2:-10}
EXP=$(cd "$(dirname "$0")" && pwd)
exec >> "$DIR/chain.log" 2>&1
echo "[$(date +%T)] start N=$N, git $(git -C "$EXP/.." rev-parse --short HEAD)," \
  "$(python3 "$EXP/../tools/wsl_clock_tick.py")"
cd "$DIR" && bash run.sh "$N" >> campaign.log 2>&1
echo "[$(date +%T)] flights: $(tail -1 campaign.log)"
source /opt/ros/jazzy/setup.bash
source "$EXP/../install/setup.bash"
for a in summarise:summary gate_f_tests:gate_f_tests s95_natural:s95_natural \
         margin_check:margin_check demotions:demotions scan_time:scan_time \
         switch_check:switch_check; do
  [ -e "${a%%:*}.py" ] && python3 "${a%%:*}.py" > "${a##*:}.txt" 2>&1
done
echo "[$(date +%T)] analyses: $(ls *.txt | tr '\n' ' ')"
touch "$DIR/campaign.done"
echo "[$(date +%T)] end"
