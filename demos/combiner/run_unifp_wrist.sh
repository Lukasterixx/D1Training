#!/usr/bin/env bash
# UniFP (model_56000) with the wrist roll servo and a friction grip, no claw (F-096: 11/16 with 16 environments,
# 8/16 with one). --wrist replaces two of the policy's actions, so this is not UniFP alone. Free door.
#
# Standing dog, box on a post, one placement at a time in the viewer. R: next attempt (restarts one mid-way).
# Close the window to end. Arguments are added after these and win, e.g. --door_torque_nm 16, --seed 7,
# --stance_deg 15. Setup and logging: _standing.sh.
. "$(dirname "$0")/_standing.sh"
standing_demo --wrist "$@"
