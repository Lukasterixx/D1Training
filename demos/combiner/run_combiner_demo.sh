#!/usr/bin/env bash
# Isaac Sim environment, matching the cup demo, without ROS.
#
# The reach console comes up beside the simulator as it does for the cup pick (demos/cup/d1_ui/beside_sim.sh):
# the arm, the legs and the wrist RealSense's view on http://localhost:8090, opened in a browser tab once the
# simulator is publishing (not for --headless), and stopped when this script exits. There is no cup, so the
# console's YOLO is off (--detect none); D1_UI_ARGS='--detect all' turns it back on. The camera window outlines
# the door's AprilTag, with its range (--apriltag). --no_console leaves the console out.
set -euo pipefail
cd "$(dirname "$0")/../.."

# With no arguments at all, the arm finds the door's AprilTag, grips the lever, turns it 45° against a 0.3 N·m
# spring and pulls the door open, at each placement, in the viewer. 0.3, not the 0.8 the lever push used: the
# grip turned 0.4 N·m and lost the bar at 0.5 (Week 1 log, 2026-09-19); `--method press` still pushes 0.8.
# Any argument hands the choice back: `--seed 42` alone is the scene without the arm moving, as before.
if [ $# -eq 0 ]; then
  set -- --turn --handle_torque_nm 0.3
  echo "[combiner] no arguments: running --turn --handle_torque_nm 0.3 (pass any argument to choose yourself)"
fi

# --no_console is this script's own; everything else goes to run_combiner_demo.py untouched.
. demos/cup/d1_ui/beside_sim.sh
console_args "$@"
set -- ${SIM_ARGS[@]+"${SIM_ARGS[@]}"}

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

if ! COMBINER_CONDA_BASE="$(find_conda_base)"; then
  echo "[combiner] Conda is unavailable; set CONDA_EXE or install the Isaac environment." >&2
  exit 127
fi
# Conda's activation hooks are not required to be nounset-safe.
set +u
source "$COMBINER_CONDA_BASE/etc/profile.d/conda.sh"
conda activate "${ISAAC_SIM_CONDA_ENV:-env_isaaclab}"
set -u
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export PYTHONUNBUFFERED=1
console_start combiner --detect none --apriltag
# Not `exec`: this shell's exit is what stops the console.
python demos/combiner/run_combiner_demo.py "$@"
