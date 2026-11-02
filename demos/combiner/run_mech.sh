#!/usr/bin/env bash
# The mechanism-trained policy (F-108, `mech_law_v1` model_17499) with the L-lip claw, the goal correction, the
# roll command and the task layer's force law: the best controller here (F-110, F-112). Door closer 8 N·m.
#
# Standing dog, box on a post, one placement at a time in the viewer. R: next attempt (restarts one mid-way).
# Close the window to end. Arguments are added after these and win, e.g. --door_torque_nm 16, --seed 7,
# --stance_deg 15. Setup and logging: _standing.sh.
. "$(dirname "$0")/_standing.sh"
standing_demo --mech --door_torque_nm 8 "$@"
