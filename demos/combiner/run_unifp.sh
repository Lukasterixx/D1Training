#!/usr/bin/env bash
# UniFP (model_56000) alone, friction grip: the hand points wherever the goal puts it, so it rarely opens the box
# (F-095: 2/16). Free door.
#
# Standing dog, box on a post, one placement at a time in the viewer. R: next attempt (restarts one mid-way).
# Close the window to end. Arguments are added after these and win, e.g. --door_torque_nm 16, --seed 7,
# --stance_deg 15. Setup and logging: _standing.sh.
. "$(dirname "$0")/_standing.sh"
standing_demo  "$@"
