#!/usr/bin/env bash
# Launch the scripted cup pick (Go2 lying down, wrist RealSense, stock YOLO) in Isaac Sim.
#
# Same environment setup as run_sim.sh, minus the ROS 2 bridge: the pick publishes nothing. Do NOT source
# /opt/ros/humble/setup.bash first -- system ROS's Python 3.10 on PYTHONPATH breaks Isaac Sim's 3.11.
#
# Any arguments are forwarded to run_pick_demo.py, e.g.:
#   ./run_pick_demo.sh                           # viewer, real time; R resets with the cup somewhere new
#   ./run_pick_demo.sh --seed 7                  # a different sequence of cup positions for R
#   ./run_pick_demo.sh --cup_xy 0.38 -0.08       # first cup somewhere specific
#   ./run_pick_demo.sh --headless --episodes 10  # ten picks back to back, results in logs/pick_demo/
#   ./run_pick_demo.sh --help                    # every option
#
# Needs ultralytics in env_isaaclab, installed without its dependencies (see README), and the cup model at
# ~/Downloads/High-Resolution_3D_Cup_Model_FBX.usdz unless --cup_usdz says otherwise.
set -e

cd "$(dirname "$0")"

CONDA_ENV_NAME="${ISAAC_SIM_CONDA_ENV:-env_isaaclab}"

find_conda_base() {
  if [ -n "${CONDA_EXE:-}" ] && [ -x "$CONDA_EXE" ]; then
    dirname "$(dirname "$CONDA_EXE")"
    return 0
  fi
  for candidate in "$HOME/miniconda3" "$HOME/anaconda3" "/opt/conda"; do
    if [ -x "$candidate/bin/conda" ]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  if command -v conda >/dev/null 2>&1; then
    dirname "$(dirname "$(command -v conda)")"
    return 0
  fi
  return 1
}

if ! CONDA_BASE="$(find_conda_base)"; then
  echo "[run_pick_demo] Conda is not available. Install Miniconda or set CONDA_EXE/ISAAC_SIM_CONDA_ENV." >&2
  exit 127
fi

if [ -f "$CONDA_BASE/etc/profile.d/conda.sh" ]; then
  # shellcheck disable=SC1090
  . "$CONDA_BASE/etc/profile.d/conda.sh"
else
  export PATH="$CONDA_BASE/bin:$PATH"
  eval "$("$CONDA_BASE/bin/conda" shell.bash hook)"
fi
conda activate "$CONDA_ENV_NAME"

# Keep system ROS's Python 3.10 out of Isaac Sim's 3.11.
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH

# The GPU is shared with other Isaac Lab training. Say so rather than silently competing with it.
if command -v nvidia-smi >/dev/null 2>&1; then
  BUSY="$(nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv,noheader 2>/dev/null || true)"
  if [ -n "$BUSY" ]; then
    echo "[run_pick_demo] Note: the GPU is already running:" >&2
    echo "$BUSY" | sed 's/^/[run_pick_demo]   /' >&2
  fi
fi

# Isaac's logging drowns stdout; unbuffered, the [pick] lines appear as they happen.
export PYTHONUNBUFFERED=1

python run_pick_demo.py "$@"
