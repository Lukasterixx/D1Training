#!/usr/bin/env bash
# Close-up renders of the wrist with the RealSense body drawn, to check the assumed mount against the
# bracket on the bench.
#
# Same environment setup as run_pick_demo.sh: the Isaac conda env, with ROS kept off PYTHONPATH. No
# console, no ROS 2 bridge, no policy -- it renders four views of the wrist and exits.
#
#   ./demos/cup/run_camera_body_view.sh                                   # four views into logs/camera_body/<stamp>/
#   ./demos/cup/run_camera_body_view.sh --pose zero                       # every joint at zero, easiest to measure
#   ./demos/cup/run_camera_body_view.sh --mount_pos -0.05 0 0.04 --mount_pitch_deg 25
#   ./demos/cup/run_camera_body_view.sh --calibration demos/cup/pick_demo/assets/calibration/d435i_238222076237_640x480.json
#   ./demos/cup/run_camera_body_view.sh --help                            # every option
set -e

# Run from the repository root: the demo imports `demos.cup.…` and writes into logs/.
cd "$(dirname "$0")/../.."

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
  echo "[camera_body_view] Conda is not available. Install Miniconda or set CONDA_EXE/ISAAC_SIM_CONDA_ENV." >&2
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
    echo "[camera_body_view] Note: the GPU is already running:" >&2
    echo "$BUSY" | sed 's/^/[camera_body_view]   /' >&2
  fi
fi

export PYTHONUNBUFFERED=1

python demos/cup/run_camera_body_view.py "$@"
