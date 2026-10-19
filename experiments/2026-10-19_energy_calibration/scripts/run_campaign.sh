#!/bin/bash
# Calibration and validation flights for the S10 energy model, one fresh
# PX4 and Gazebo per flight (session_cal.sh).
#
# Calibration: squares of 10, 20, 40 and 60 m, three repetitions each, at
# 5 m/s cruise and 3 m/s2 ramps. The hover before the lap is 10, 20 or
# 30 s, assigned so that every size sees all three lengths:
# hold = (10, 20, 30)[(size index + repetition) mod 3]. Repetitions run in
# a rotated size order, so neither size nor hover length is confounded
# with time of day.
#
# Validation, never used in the fit: a 30 m square after a 60 s hover,
# the prolonged stationary phase that refuted E = k d, and a 220 m zigzag
# with 8 corners against 4 for every square, which tests whether corners
# cost what the fit assumes.
#
# A flight that ends without the LANDED phase is a technical failure: its
# directory moves to runs/failed/ and the flight is flown again once.
# Every failure is logged in runs/campaign.log, none is dropped silently
# (spec 5.4).
#   bash run_campaign.sh
HERE=$(cd "$(dirname "$0")" && pwd)
RUNS=$(cd "$HERE/.." && pwd)/runs
SPEED=5
ACCEL=3
POST=5
ALT=8
mkdir -p "$RUNS/failed"
LOG="$RUNS/campaign.log"

square() { echo "0,0;$1,0;$1,$1;0,$1;0,0"; }
ZIGZAG="0,0;40,0;40,10;0,10;0,20;40,20;40,30;0,30;0,0"

PLAN=(
  "cal_L10_r1 $(square 10) 10"
  "cal_L20_r1 $(square 20) 20"
  "cal_L40_r1 $(square 40) 30"
  "cal_L60_r1 $(square 60) 10"
  "val_sq30_h60_r1 $(square 30) 60"
  "cal_L40_r2 $(square 40) 10"
  "cal_L60_r2 $(square 60) 20"
  "cal_L10_r2 $(square 10) 20"
  "cal_L20_r2 $(square 20) 30"
  "val_zig_h10_r1 $ZIGZAG 10"
  "cal_L60_r3 $(square 60) 30"
  "cal_L10_r3 $(square 10) 30"
  "cal_L20_r3 $(square 20) 10"
  "cal_L40_r3 $(square 40) 20"
  "val_sq30_h60_r2 $(square 30) 60"
  "val_zig_h10_r2 $ZIGZAG 10"
)

echo "$(date -Is) campaign start, ${#PLAN[@]} flights" >> "$LOG"
for entry in "${PLAN[@]}"; do
  read -r name poly hold <<< "$entry"
  for attempt in 1 2; do
    if [ -e "$RUNS/$name" ]; then
      echo "$(date -Is) $name exists, skipped" >> "$LOG"
      break
    fi
    bash "$HERE/session_cal.sh" "$RUNS/$name" "$poly" "$SPEED" "$hold" \
      "$POST" "$ALT" "$ACCEL" > "$RUNS/$name.session.log" 2>&1
    mv "$RUNS/$name.session.log" "$RUNS/$name/session.log" 2>/dev/null
    if grep -q ",LANDED$" "$RUNS/$name/phases.csv" 2>/dev/null; then
      fs=$(grep -c "Failsafe activated" "$RUNS/$name/px4.log")
      echo "$(date -Is) $name ok, attempt $attempt, failsafes $fs" >> "$LOG"
      break
    fi
    echo "$(date -Is) $name FAILED, attempt $attempt, moved to failed/" >> "$LOG"
    mv "$RUNS/$name" "$RUNS/failed/${name}_attempt$attempt" 2>/dev/null
  done
done
echo "$(date -Is) campaign end" >> "$LOG"
cat "$LOG"
