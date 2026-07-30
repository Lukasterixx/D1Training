#!/usr/bin/env bash
# Launch the flat Go2+D1 testbed.
#
# Trimmed from Rescue's run_sim.sh: no CycloneDDS, and no lidar JSON to install
# (the L1 is built from Isaac's shipped Example_Rotary asset, overridden in
# ros2.py). The ROS 2 bridge setup below is Rescue's, because the sim publishes
# its L1 cloud and joint states again -- see run_rviz.sh.
#
# The one hazard that still bites: system ROS's Python 3.10 leaking into Isaac
# Sim's 3.11 via PYTHONPATH, which fails with an opaque import error deep inside
# omni. So do NOT source /opt/ros/humble/setup.bash before this -- Isaac's own
# bundled Humble libraries are what the bridge uses.
#
# Any arguments are forwarded to main.py, e.g.:
#   ./run_sim.sh --arm_mass 6.0      # heavier base cylinder (default is the real 3.152 kg)
#   ./run_sim.sh --no_arm            # bare-Go2 baseline
#   ./run_sim.sh --no_ros2           # no lidar, no /joint_states
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
  echo "[run_sim] Conda is not available. Install Miniconda or set CONDA_EXE/ISAAC_SIM_CONDA_ENV." >&2
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

# ROS 2 bridge. Fast DDS, matching run_rviz.sh -- the two will not discover each
# other on different middleware, and the failure is silent (topics simply never
# appear). Any CycloneDDS config left in the environment would do the same.
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
unset CYCLONEDDS_URI CYCLONEDDS_HOME CYCLONEDDS_CONFIG ROS_DISCOVERY_SERVER

# Use Isaac Sim's bundled ROS 2 libraries, not /opt/ros/humble's.
ISAAC_BRIDGE_EXT="$CONDA_PREFIX/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge"
export LD_LIBRARY_PATH="$ISAAC_BRIDGE_EXT/humble/lib:${LD_LIBRARY_PATH}"

# Still needed on Ubuntu 22.04.
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

# Isaac's own logging drowns stdout; without this, buffered prints from main.py
# are lost when the app tears down and a --selftest run looks like it produced
# nothing at all.
export PYTHONUNBUFFERED=1

python main.py "$@"
