# alpha changed during a mission, 20 Oct 2026 (S14)

Dorian wants to change alpha while the simulation runs, as in a live
demonstration: the aircraft keeping its margin, then leaving it for
information, on the same flight. The manager declares alpha as a
dynamic ROS parameter (range 0 to 1, step 0.05, so `rqt_reconfigure`
shows a slider), applies a change at its next frontier choice and logs
it as an 'alpha' event. Never tested in flight before this.

`run.sh LABEL BUDGET A0 A1 SWITCH_S`: one session, policy c1_c2 at A0;
SWITCH_S seconds after the first path the watcher runs, as a user would,
`ros2 param set /exploration_manager alpha A1`. 16 kJ on W1, the budget
where alpha acts most in `2026-10-20_budget_sweep`. Fresh world, 0
failsafes, 0 I1 violations in all four sessions.

`alpha_live.py` lists every frontier choice with the alpha in force,
and re-ranks each decision, from the candidates logged with it, at the
starting and at the final alpha: a choice after the change that differs
from the pick of the starting alpha is the change acting.

## Sessions

`a0to1_16kJ`, `a1to0_16kJ` (ecefdde, switch 10 s after the first path,
at 30 s): the parameter was accepted ("Set parameter successful") and
logged, but at 16 kJ the three frontier choices of W1 all come before
30 s, then C1 sends the aircraft home. No decision came after the
change, so these two say nothing of its effect; they show that at
every one of their six decisions alpha 0 and alpha 1 would have picked
different frontiers.

`a0to1_16kJ_early`, `a1to0_16kJ_early` (ecefdde, switch 1 s after the
first path, at 16.9 s):

| session | decision | alpha in force | chosen | path | share of budget left | pick of the starting alpha | changed |
|---|---|---|---|---|---|---|---|
| 0 -> 1 | 13.9 s | 0 | (-11.5, -12.5) | 16.6 m | 0.369 | same | before |
| 0 -> 1 | 19.9 s | **1** | (+12.5, +11.5) | **33.9 m** | **0.152**, the lowest on offer (M 0) | (-19.5, +1.5) | **yes** |
| 1 -> 0 | 13.9 s | 1 | (-12.5, -11.5) | 17.0 m | 0.367 | same | before |
| 1 -> 0 | 19.9 s | **0** | (-19.5, +2.5) | **16.9 m** | **0.247**, the highest on offer (M 1) | (+11.5, +12.5) | **yes** |
| 1 -> 0 | 25.9 s | **0** | (-12.5, +16.5) | 16.9 m | 0.077, the highest on offer (M 1) | (+5.5, +16.5) | **yes** |

Every decision taken after the change differs from what the starting
alpha picks, 3 of 3, from the first decision after it (3 s later).
Switched to 1 the aircraft takes the far frontier with the most
unknown ground and the least margin left; switched to 0 it takes the
near ones that keep the most energy. C1 stays in force throughout:
every choice was admissible, no I1 violation, landing on a confirmed
zone.

## What it means for a live demonstration

The mechanism works: the knob acts at the next decision, and safety
does not depend on it. W1 is too small to show it to an audience: a
16 kJ mission lasts about 50 s and makes three or four choices, so a
change has to come within seconds of the first one. A demonstration
someone can follow needs missions of minutes with landing zones far
apart, where staying near a zone and leaving it are visibly different
paths: the world W2.

Files:

    run.sh             one session with a change of alpha
    alpha_live.py      choices, re-ranking at both alphas, figure
    <session>/         setup.txt, logs, missions/, analysis.txt,
                       alpha_live.txt, alpha_live.png
    <session>.out      session progress
