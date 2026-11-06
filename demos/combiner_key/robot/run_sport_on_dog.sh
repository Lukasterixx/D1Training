#!/usr/bin/env bash
# The Go2's sport mode from the dog's host, in P2Dingo's ROS image (sport_mode.py): for when no program is left to hand
# the legs back. Run ON the dog, from its D1Training checkout.
#
#   demos/combiner_key/robot/run_sport_on_dog.sh                  # CheckMode only
#   demos/combiner_key/robot/run_sport_on_dog.sh --ensure mcf     # SelectMode only if sport mode is released
#   demos/combiner_key/robot/run_sport_on_dog.sh --select mcf     # SelectMode
set -euo pipefail

REPO="$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)"
P2="${P2DINGO_ROOT:-$HOME/P2Dingo}"
ROS_IMAGE="${D1T_ROS_IMAGE:-p2dingo-web:latest}"

exec docker run --rm --name d1t-sport --network host --ipc host \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///root/cyclonedds_go2.xml \
  -e PYTHONUNBUFFERED=1 -e PYTHONDONTWRITEBYTECODE=1 \
  -v "$P2/cyclonedds_go2.xml:/root/cyclonedds_go2.xml:ro" -v "$REPO:/root/D1Training" \
  --entrypoint bash "$ROS_IMAGE" \
  -lc 'source /opt/ros/humble/setup.bash && source /root/ws/install/setup.bash && cd /root/D1Training && exec python3 demos/combiner_key/robot/sport_mode.py "$@"' \
  sport "$@"
