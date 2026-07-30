#!/usr/bin/env bash
# Watch the running sim in RViz 2: the welded Go2+D1 drawn from its meshes in the
# pose the sim is actually in, plus the front L1 lidar's point cloud.
#
# Run it alongside ./run_sim.sh, in a separate terminal. Order does not matter.
#
# This is the mirror image of run_sim.sh: that script must NOT see system ROS
# (Isaac Sim has its own bundled Humble), and this one needs nothing else --
# robot_state_publisher and rviz2 come from /opt/ros/humble. The two talk over
# Fast DDS on ROS_DOMAIN_ID 0, which both scripts set explicitly, because a
# mismatch shows up as topics that simply never arrive.
#
#   ./run_rviz.sh                 # robot model + L1 cloud
#   ROS_DOMAIN_ID=7 ./run_rviz.sh # ...if run_sim.sh was given the same
set -e

cd "$(dirname "$0")"
REPO="$(pwd)"

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [ ! -f "$ROS_SETUP" ]; then
  echo "[run_rviz] No ROS 2 at $ROS_SETUP. Install ros-humble-desktop, or set" >&2
  echo "[run_rviz] ROS_SETUP to your distro's setup.bash." >&2
  exit 127
fi
# shellcheck disable=SC1090
. "$ROS_SETUP"

for pkg in robot_state_publisher rviz2; do
  if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "[run_rviz] Missing package '$pkg'. sudo apt install ros-$ROS_DISTRO-$(echo "$pkg" | tr _ -)" >&2
    exit 127
  fi
done

# Match run_sim.sh's middleware exactly.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
unset CYCLONEDDS_URI CYCLONEDDS_HOME CYCLONEDDS_CONFIG ROS_DISCOVERY_SERVER

# The committed URDF addresses its meshes as package://d1_training/..., which is
# a placeholder: this repo is not an ament package, so nothing would resolve it.
# Rewrite it to absolute file:// paths against the checkout we are sitting in.
GEN_DIR="${TMPDIR:-/tmp}/d1training_rviz"
mkdir -p "$GEN_DIR"
URDF="$GEN_DIR/go2_d1.urdf"
sed "s|package://d1_training/|file://$REPO/|g" description/go2_d1.urdf > "$URDF"

# use_sim_time throughout: the sim publishes /clock off Isaac's timeline, and the
# lidar's clouds are stamped from it. On wall-clock time RViz reads every
# transform as decades old and draws nothing.
# The URDF goes in as the positional argument, not as -p robot_description:=...:
# a 700-line XML string does not survive ros2's YAML parameter parsing (it fails
# with "Failed to parse global arguments"). Humble warns that the positional
# fallback is deprecated; that one warning line is expected.
ros2 run robot_state_publisher robot_state_publisher "$URDF" \
  --ros-args -p use_sim_time:=true &
RSP_PID=$!
trap 'kill "$RSP_PID" 2>/dev/null || true' EXIT INT TERM

echo "[run_rviz] robot_state_publisher up on $URDF"
echo "[run_rviz] waiting on /joint_states and /utlidar/cloud from run_sim.sh..."

ros2 run rviz2 rviz2 -d description/go2_d1.rviz --ros-args -p use_sim_time:=true
