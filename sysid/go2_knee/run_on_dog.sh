#!/usr/bin/env bash
# Log the Go2's leg torques (LowState tau_est) on the dog, READ-ONLY: nothing is sent to the robot. Run ON the dog, from its
# D1Training checkout, beside P2Dingo's containers; drive the robot from the Unitree remote as usual (sport mode).
#
#   sysid/go2_knee/run_on_dog.sh --mass_kg 18.4              # Ctrl-C to stop; the record is logs/go2_knee/<stamp>/
#   sysid/go2_knee/run_on_dog.sh --mass_kg 18.4 --topic /lf/lowstate
#
# Every argument goes to log_lowstate.py (--mass_kg, the dog weighed as it stands, is required; --topic; --seconds).
# What to do while it logs, and how to read it: log_lowstate.py's docstring.
set -euo pipefail

REPO="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
P2="${P2DINGO_ROOT:-$HOME/P2Dingo}"
ROS_IMAGE="${D1T_ROS_IMAGE:-p2dingo-web:latest}"

case "${1:-}" in
  -h|--help) awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "${BASH_SOURCE[0]}"; exit 0 ;;
esac
command -v docker >/dev/null || { echo "docker is not installed (is this the dog?)" >&2; exit 1; }
docker image inspect "$ROS_IMAGE" >/dev/null 2>&1 || { echo "no image $ROS_IMAGE: P2Dingo's split stack builds it" >&2; exit 1; }
[ -f "$P2/cyclonedds_go2.xml" ] || { echo "no $P2/cyclonedds_go2.xml (set P2DINGO_ROOT)" >&2; exit 1; }
mkdir -p "$REPO/logs/go2_knee"

TTY=(-i); [ -t 0 ] && TTY=(-it)
docker run "${TTY[@]}" --rm --name d1t-go2-knee-log --network host --ipc host \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///root/cyclonedds_go2.xml \
  -e PYTHONUNBUFFERED=1 -e PYTHONDONTWRITEBYTECODE=1 \
  -v "$P2/cyclonedds_go2.xml:/root/cyclonedds_go2.xml:ro" -v "$REPO:/root/D1Training" \
  --entrypoint bash "$ROS_IMAGE" \
  -lc 'source /opt/ros/humble/setup.bash && source /root/ws/install/setup.bash && cd /root/D1Training && exec python3 sysid/go2_knee/log_lowstate.py --run_dir /root/D1Training/logs/go2_knee "$@"' \
  log "$@"
# The container runs as root: hand the record back.
docker run --rm -v "$REPO/logs:/logs" --entrypoint chown "$ROS_IMAGE" -R "$(id -u):$(id -g)" /logs/go2_knee >/dev/null 2>&1 || true
