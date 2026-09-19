#!/usr/bin/env bash
# Launch the Go2+D1 UniFP training detached, so it survives the terminal.
#   ./unifp_go2d1/run_training.sh [extra launch_training.py args...]
set -euo pipefail

UNIFP="${UNIFP_ROOT:-$HOME/thesis_b_legacy/UniFP}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG="$UNIFP/logs/train_${STAMP}.log"
mkdir -p "$UNIFP/logs"

source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate unifp
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"

cd "$UNIFP"
setsid nohup python "$HERE/launch_training.py" \
    --tf32 --no-wandb --task=go2d1_pos_force --num_envs 4096 --headless "$@" \
    > "$LOG" 2>&1 < /dev/null &
PID=$!
echo "pid $PID"
echo "log $LOG"
echo "stop with: kill $PID"
