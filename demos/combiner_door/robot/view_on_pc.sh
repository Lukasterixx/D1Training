#!/usr/bin/env bash
# RViz on THIS PC for the combiner door's robot program (node.py on the dog), over the dog's P2Dingo zenoh link: the
# Go2 and its D1 as they are, the plan's ghost, the box where the look puts it (the door at the lidar's angle), the
# stance, the base's path round the hinge, the hand's target, the lidar's face fit and points, the head camera with
# the tag outlined, and the status. After P2Dingo's scripts/p2dingo-dock-view-robot.sh.
#
#   demos/combiner_door/robot/view_on_pc.sh              # finds the dog as P2Dingo's p2dingo-rviz.sh does
#   ROBOT_HOST=192.168.10.18 demos/combiner_door/robot/view_on_pc.sh
#   demos/combiner_door/robot/view_on_pc.sh --no-gpu     # any p2dingo-rviz.sh option passes through
#
# It is P2Dingo's p2dingo-rviz.sh (the zenoh client, the SSH tunnel through the RUT241 on a home WiFi, the image
# decoder, rviz in a container) with this directory's combiner_door_robot.rviz, this repo mounted into rviz for the
# meshes (opc-compose.yaml), the decoder on /combiner_door/debug_image, and P2Dingo's static TF helper
# (scripts/python/p2dingo-opc-static-tf.py: this PC's bridge keeps only the newest /tf_static message, so the
# robot's fixed frames are published here). The /combiner_door/ topics must be in BOTH zenoh allow-lists (P2Dingo's
# zenoh/robot_bridge.json5 on the dog, zenoh/opc_bridge.json5 here); the dog's bridge reads its file when it starts.
# Closing RViz (or Ctrl-C) stops this PC's side only. Start the robot side with run_on_dog.sh (or `d1dog`).
set -euo pipefail

HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
D1="$(cd "$HERE/../../.." && pwd)"
P2="${P2DINGO_ROOT:-$HOME/P2Dingo}"
IMAGE="p2dingo-rviz:${OPC_ROS_DISTRO:-humble}"
TF_NAME=d1t-opc-static-tf

log() { printf '[combiner-door-view] %s\n' "$*" >&2; }
die() { printf '[combiner-door-view] ERROR: %s\n' "$*" >&2; exit 1; }

case "${1:-}" in
  -h|--help) awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "${BASH_SOURCE[0]}"; exit 0 ;;
esac
command -v docker >/dev/null || die "docker is not installed."
[ -x "$P2/scripts/p2dingo-rviz.sh" ] || die "no P2Dingo checkout at $P2 (set P2DINGO_ROOT)"
grep -q 'OPC_COMPOSE_EXTRA' "$P2/scripts/p2dingo-rviz.sh" ||
  die "$P2/scripts/p2dingo-rviz.sh has no OPC_COMPOSE_EXTRA hook (it mounts this repo into rviz)"
grep -q '/combiner_door/' "$P2/zenoh/opc_bridge.json5" ||
  die "$P2/zenoh/opc_bridge.json5 does not carry the /combiner_door topics"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  log "building $IMAGE (first run: a few minutes)"
  (cd "$P2" && OPC_ROS_DISTRO="${OPC_ROS_DISTRO:-humble}" docker compose -f docker-compose-opc.yaml build rviz)
fi

cleanup() { docker rm -f "$TF_NAME" >/dev/null 2>&1 || true; }
trap cleanup EXIT

# The robot's static TF tree on this PC's DDS (domain 68), as the dock view runs it.
docker rm -f "$TF_NAME" >/dev/null 2>&1 || true
docker run -d --rm --name "$TF_NAME" --network host --ipc host \
  -e ROS_DOMAIN_ID=68 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_LOCALHOST_ONLY=1 \
  -v "$P2/scripts/python/p2dingo-opc-static-tf.py:/root/repo/scripts/python/p2dingo-opc-static-tf.py:ro" \
  --entrypoint bash "$IMAGE" \
  -lc 'source "/opt/ros/$ROS_DISTRO/setup.bash" && exec python3 /root/repo/scripts/python/p2dingo-opc-static-tf.py' \
  >/dev/null
log "the robot's static TF tree: container $TF_NAME"

# The D1 dog's own WiFi dongle first, at the address `sshuni` and `d1update` last found it (~/.cache/p2dingo/dongle-ip):
# p2dingo-rviz.sh's own search (Tailscale, the dog's WiFi, a RUT241's 2222) does not look there.
if [ -z "${ROBOT_HOST:-}" ]; then
  dongle="$(cat "$HOME/.cache/p2dingo/dongle-ip" 2>/dev/null || true)"
  if [ -n "$dongle" ] && timeout 0.8 bash -c "</dev/tcp/$dongle/7447" 2>/dev/null; then
    export ROBOT_HOST="$dongle"
    log "the dog's zenoh listener on its WiFi dongle, $dongle:7447"
  fi
fi

export D1TRAINING_ROOT="$D1"
export OPC_COMPOSE_EXTRA="$HERE/opc-compose.yaml"
export IMAGE_BASE=/combiner_door/debug_image
export RVIZ_WAIT_TOPIC=/tf
"$P2/scripts/p2dingo-rviz.sh" --config /root/d1training/demos/combiner_door/robot/combiner_door_robot.rviz "$@"
