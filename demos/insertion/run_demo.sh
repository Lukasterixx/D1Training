#!/usr/bin/env bash
# Watch the insertion demo on a loop: the standing Go2 + D1 pushes a key into a keyed slot on a door, attempt after
# attempt, until the window is closed.
#
# Same environment setup as demos/cup/run_pick_demo.sh: the Isaac conda env, with ROS kept off PYTHONPATH. No
# console, no ROS 2 bridge. Each attempt draws a new slot (height, side, blade level or upright) and a new 2 mm error
# in the estimate of where it is. A seated key is held home for 3 s, then the next attempt starts; one that does not
# seat runs the whole 16 s episode. The viewer runs in real time. The lock is drawn in a door (a wooden door in a wall,
# an escutcheon round the keyway, a lever handle) and the wrist RealSense and its bracket at the saved mount, all
# visual only (--no-scenery, --no-camera_body leave them out). Each attempt's outcome is printed as it ends, and the
# results go to logs/insertion_demo/<stamp>_.../ after every attempt, so closing the window keeps the finished ones.
#
# Always runs the measured-robot insertion policy (checkpoints/README.md) holding the key compliantly, as it was trained,
# with one robot in the scene. Any arguments are appended, and the last value given wins:
#   ./demos/insertion/run_demo.sh                              # loop until the window is closed
#   ./demos/insertion/run_demo.sh --lateral_mm 4               # a worse estimate
#   ./demos/insertion/run_demo.sh --blade level --seed 7       # only level blades, other draws
#   ./demos/insertion/run_demo.sh --hold_s 1 --no-realtime     # move on sooner, and as fast as the simulator goes
#   ./demos/insertion/run_demo.sh --checkpoint checkpoints/unifp_go2d1_insertion_seed3_model_21200.pt
#   ./demos/insertion/run_demo.sh --help                       # every option
#
# For measuring, run demos/insertion/run_demo.py directly, headless, without --loop (demos/insertion/README.md).
set -e

# Run from the repository root: the demo imports `demos.insertion.…` and writes into logs/.
cd "$(dirname "$0")/../.."

CONDA_ENV_NAME="${ISAAC_SIM_CONDA_ENV:-env_isaaclab}"
CHECKPOINT="checkpoints/unifp_go2d1_insertion_seed3_measured_model_21400.pt"

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
  echo "[insertion] Conda is not available. Install Miniconda or set CONDA_EXE/ISAAC_SIM_CONDA_ENV." >&2
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
    echo "[insertion] Note: the GPU is already running:" >&2
    echo "$BUSY" | sed 's/^/[insertion]   /' >&2
  fi
fi

# Isaac's logging drowns stdout; unbuffered, the [insertion demo] lines appear as they happen.
export PYTHONUNBUFFERED=1

exec python demos/insertion/run_demo.py --checkpoint "$CHECKPOINT" --compliant_key --num_envs 1 \
  --lateral_mm 2 --loop --hold_s 3 "$@"
