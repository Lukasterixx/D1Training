#!/usr/bin/env bash
# The key turned by the arm (arm_env.py, arm_turn.py): the head RealSense finds the box's AprilTag, the MID-360
# gives the door's heading, the dog walks to the push's stance and the D1's fingers push the key from 45 up to 45 down.
#
#   ./demos/combiner_key/run_arm_turn.sh                                   # watch one in the viewer; pictures in frames/
#   ./demos/combiner_key/run_arm_turn.sh --lock_n 10                       # a stiffer lock: 10 N where the fingers push
#   ./demos/combiner_key/run_arm_turn.sh --attempts 8 --headless --no_overview   # measure: eight placements
#
# Any argument goes to run_arm_turn.py. Isaac Sim's environment as
# the other demos use it, without ROS. The run's folder is printed at the end; record it with
# ./dashboard.py record <folder> --title "...".
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")/../.."

find_conda_base() {
  if [ -n "${CONDA_EXE:-}" ] && [ -x "$CONDA_EXE" ]; then
    dirname "$(dirname "$CONDA_EXE")"
    return
  fi
  for candidate in "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda; do
    if [ -x "$candidate/bin/conda" ]; then
      printf '%s\n' "$candidate"
      return
    fi
  done
  if command -v conda >/dev/null 2>&1; then
    dirname "$(dirname "$(command -v conda)")"
    return
  fi
  return 1
}

if ! CONDA_BASE="$(find_conda_base)"; then
  echo "[arm_turn] Conda is unavailable; set CONDA_EXE or install the Isaac environment." >&2
  exit 127
fi

# The GPU is shared with other Isaac jobs (CLAUDE.md): say so if one is already running, then carry on.
if command -v nvidia-smi >/dev/null 2>&1; then
  running="$(nvidia-smi --query-compute-apps=pid,process_name --format=csv,noheader 2>/dev/null || true)"
  if [ -n "$running" ]; then
    echo "[arm_turn] already on the GPU:"
    printf '  %s\n' "$running"
  fi
fi

# Conda's activation hooks are not required to be nounset-safe.
set +u
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate "${ISAAC_SIM_CONDA_ENV:-env_isaaclab}"
set -u
# Keep system ROS's Python out of Isaac Sim's.
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export PYTHONUNBUFFERED=1

python demos/combiner_key/run_arm_turn.py "$@"
