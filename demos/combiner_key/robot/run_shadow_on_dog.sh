#!/usr/bin/env bash
# The key-turning leg policy in SHADOW MODE on the dog: it runs on the robot's own state and sends NOTHING -- no LowCmd,
# no sport request, no arm target. Run ON the dog, from its D1Training checkout, beside P2Dingo's containers, with the
# robot standing in sport mode (the classic stand) and the Unitree remote in hand as usual.
#
#   demos/combiner_key/robot/run_shadow_on_dog.sh --policy checkpoints/leg_key_force_s2walker_model_2104.npz
#   demos/combiner_key/robot/run_shadow_on_dog.sh --policy ... --seconds 60
#   demos/combiner_key/robot/run_shadow_on_dog.sh --policy ... --no_arm          # no arm link: the arm at its rest pose
#
# It starts the arm link (demos/combiner_door/robot/arm_link.py) with --observe, which only reports the D1's angles,
# then shadow.py in P2Dingo's ROS image; every other argument goes to shadow.py (--policy, --topic, --seconds,
# --foot_goal; --help). Ctrl-C stops it. The record is logs/leg_shadow/<stamp>/ (`d1fetch` copies it back); read it on
# the PC with check_shadow.py --dog.
set -euo pipefail

REPO="$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)"
P2="${P2DINGO_ROOT:-$HOME/P2Dingo}"
ROS_IMAGE="${D1T_ROS_IMAGE:-p2dingo-web:latest}"
ARM_IMAGE="${D1T_ARM_IMAGE:-p2dingo-arm:latest}"
NODE_NAME=d1t-leg-shadow
ARM_NAME=d1t-arm-link

log() { printf '[run_shadow_on_dog] %s\n' "$*" >&2; }
die() { printf '[run_shadow_on_dog] ERROR: %s\n' "$*" >&2; exit 1; }

case "${1:-}" in
  -h|--help) awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "${BASH_SOURCE[0]}"; exit 0 ;;
esac
command -v docker >/dev/null || die "docker is not installed (is this the dog?)"
docker image inspect "$ROS_IMAGE" >/dev/null 2>&1 || die "no image $ROS_IMAGE: P2Dingo's split stack builds it"
[ -f "$P2/cyclonedds_go2.xml" ] || die "no $P2/cyclonedds_go2.xml (set P2DINGO_ROOT)"

ARM=1
args=()
for a in "$@"; do
  if [ "$a" = "--no_arm" ]; then ARM=0; else args+=("$a"); fi
done
STAMP="$(date -u +%Y%m%dT%H%M%S)"
mkdir -p "$REPO/logs/leg_shadow"

for name in "$NODE_NAME" "$ARM_NAME"; do
  docker rm -f "$name" >/dev/null 2>&1 || true
done

PY_ENV=(-e PYTHONUNBUFFERED=1 -e PYTHONDONTWRITEBYTECODE=1)
ROS_ENV=(-e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///root/cyclonedds_go2.xml
         -e TZ=Australia/Sydney "${PY_ENV[@]}")
ROS_MOUNTS=(-v "$P2/cyclonedds_go2.xml:/root/cyclonedds_go2.xml:ro" -v "$REPO:/root/D1Training")
OWNER="$(id -u):$(id -g)"

cleanup() {
  set +e
  docker stop -t 2 "$NODE_NAME" >/dev/null 2>&1
  if [ "$ARM" = 1 ]; then
    docker stop -t 3 "$ARM_NAME" >/dev/null 2>&1
    docker logs "$ARM_NAME" >"$REPO/logs/leg_shadow/${STAMP}_arm_link.log" 2>&1
    docker rm -f "$ARM_NAME" >/dev/null 2>&1
  fi
  docker run --rm -v "$REPO/logs:/logs" --entrypoint chown "$ROS_IMAGE" -R "$OWNER" /logs/leg_shadow >/dev/null 2>&1
  log "records: $REPO/logs/leg_shadow/"
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

if [ "$ARM" = 1 ]; then
  if docker image inspect "$ARM_IMAGE" >/dev/null 2>&1; then
    # --observe: the link reports the arm's angles and takes no target.
    docker run -d --name "$ARM_NAME" --network host --ipc host "${PY_ENV[@]}" \
      -v "$REPO:/root/D1Training" --entrypoint python3 "$ARM_IMAGE" \
      /root/D1Training/demos/combiner_door/robot/arm_link.py --observe \
      --log "/root/D1Training/logs/leg_shadow/${STAMP}_arm_link.jsonl" >/dev/null
    for _ in $(seq 1 60); do
      docker logs "$ARM_NAME" 2>&1 | grep -q "listening on" && break
      docker ps -q -f "name=^${ARM_NAME}$" | grep -q . || break
      sleep 0.25
    done
    docker logs "$ARM_NAME" 2>&1 | grep -q "listening on" \
      || log "the arm link did not start ($(docker logs "$ARM_NAME" 2>&1 | tail -1)): the arm is taken at its rest pose"
  else
    log "no image $ARM_IMAGE: the arm is taken at its rest pose"
  fi
else
  args+=(--arm_link_port 0)
fi

TTY=(-i); [ -t 0 ] && TTY=(-it)
log "shadow mode: nothing is sent to the robot"
docker run "${TTY[@]}" --rm --name "$NODE_NAME" --network host --ipc host "${ROS_ENV[@]}" "${ROS_MOUNTS[@]}" \
  --entrypoint bash "$ROS_IMAGE" \
  -lc 'source /opt/ros/humble/setup.bash && source /root/ws/install/setup.bash && cd /root/D1Training && exec python3 demos/combiner_key/robot/shadow.py --run_dir /root/D1Training/logs/leg_shadow "$@"' \
  shadow "${args[@]}"
