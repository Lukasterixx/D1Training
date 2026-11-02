#!/usr/bin/env bash
# The same mechanism policy, claw and correction with the force law off (`--no_force_law`): only the goal is
# commanded, and the policy yields to the springs (1/16 on a free door, 0/16 with any closer). Door closer 8 N·m,
# as run_mech.sh, so the two can be watched side by side.
#
# Standing dog, box on a post, one placement at a time in the viewer. R: next attempt (restarts one mid-way).
# Close the window to end. Arguments are added after these and win, e.g. --door_torque_nm 16, --seed 7,
# --stance_deg 15. Setup and logging: _standing.sh.
. "$(dirname "$0")/_standing.sh"
standing_demo --mech --no_force_law --door_torque_nm 8 "$@"
