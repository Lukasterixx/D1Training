#!/usr/bin/env bash
# UniFP (model_56000) with the wrist roll servo, given the same L-lip claw, goal correction and script as the
# mechanism policy: the comparison controller of F-112 (13/16 on a free door, 6/16 at 4 N·m, none from 8).
# --wrist replaces two of the policy's actions, so this is not UniFP alone. Free door.
#
# Standing dog, box on a post, one placement at a time in the viewer. R: next attempt (restarts one mid-way).
# Close the window to end. Arguments are added after these and win, e.g. --door_torque_nm 16, --seed 7,
# --stance_deg 15. Setup and logging: _standing.sh.
. "$(dirname "$0")/_standing.sh"
standing_demo --wrist --claw --goal_correction --turn_deg 60 --lever_grasp_m 0.080 --ease_deg 60 "$@"
