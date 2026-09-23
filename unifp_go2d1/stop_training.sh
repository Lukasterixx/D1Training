#!/usr/bin/env bash
# Stop the Go2+D1 training and its supervisor on purpose.
# The STOP file is what tells the supervisor this was deliberate, so it does not resume.
set -uo pipefail
UNIFP="${UNIFP_ROOT:-$HOME/thesis_b_legacy/UniFP}"
touch "$UNIFP/logs/STOP"
pkill -f "supervise_training.sh" && echo "supervisor stopped"
pkill -f "launch_training.py" && echo "training stopped"
echo "STOP file left at $UNIFP/logs/STOP -- delete it before starting again."
