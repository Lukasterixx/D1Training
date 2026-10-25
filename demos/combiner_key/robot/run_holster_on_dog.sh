#!/usr/bin/env bash
# The holster draw's arm motion on the real D1 with no holster and no key (holster_dog.py). DRY unless --live: dry,
# the arm link only reports and the motion runs on a simulated arm; live, it moves the real arm. The legs are never
# commanded: sit (or stand) the dog with the remote first, the arm folded. Run ON the dog, from its D1Training
# checkout, beside P2Dingo's containers (`d1dogholster` on the PC does the SSH).
#
#   demos/combiner_key/robot/run_holster_on_dog.sh                  # dry: the plan, the motion on a simulated arm
#   demos/combiner_key/robot/run_holster_on_dog.sh --live           # the real arm: one Enter to start
#   demos/combiner_key/robot/run_holster_on_dog.sh --live --step    # an Enter before every move
#   demos/combiner_key/robot/run_holster_on_dog.sh --live --timing  # and, beside it, the arm's report timing (read-only,
#                                                                   # arm_feedback_timing.py) in <record>/feedback_timing/
#
# It starts the arm link (demos/combiner_door/robot/arm_link.py, P2Dingo's arm image; --observe unless --live), then
# holster_dog.py in P2Dingo's ROS image; every argument goes to holster_dog.py (--live, --step, --joint_speed; --help).
# `a` + Enter (or the remote's B) stops it and holds the arm where it is (the link never releases it); then `r` + Enter
# (or the remote's A) takes it back to where it started, the way it came. The remote is read from /lf/lowstate (ROS,
# P2Dingo's cyclonedds_go2.xml). The record is logs/holster_dog/<stamp>_{dry,live}/ (`d1fetch` copies it back).
set -euo pipefail

REPO="$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)"
P2="${P2DINGO_ROOT:-$HOME/P2Dingo}"
ROS_IMAGE="${D1T_ROS_IMAGE:-p2dingo-web:latest}"
ARM_IMAGE="${D1T_ARM_IMAGE:-p2dingo-arm:latest}"
NODE_NAME=d1t-holster
ARM_NAME=d1t-arm-link

log() { printf '[run_holster_on_dog] %s\n' "$*" >&2; }
die() { printf '[run_holster_on_dog] ERROR: %s\n' "$*" >&2; exit 1; }

case "${1:-}" in
  -h|--help) awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "${BASH_SOURCE[0]}"; exit 0 ;;
esac
command -v docker >/dev/null || die "docker is not installed (is this the dog?)"
for image in "$ROS_IMAGE" "$ARM_IMAGE"; do
  docker image inspect "$image" >/dev/null 2>&1 || die "no image $image: P2Dingo's split stack builds it"
done

LIVE=0
TIMING=0
args=()
for a in "$@"; do
  case "$a" in
    --live) LIVE=1; args+=("$a") ;;
    --timing) TIMING=1 ;;
    *) args+=("$a") ;;
  esac
done
TIMING_NAME=d1t-feedback-timing
STAMP="$(date -u +%Y%m%dT%H%M%S)"
SUFFIX=dry; [ "$LIVE" = 1 ] && SUFFIX=live
RUN_DIR="logs/holster_dog/${STAMP}_${SUFFIX}"
mkdir -p "$REPO/$RUN_DIR"

# Another robot program (the door program, the shadow runner, the foot press) shares the arm link's name: never remove
# it from under one.
others="$(docker ps -a --format '{{.Names}}' | grep -E '^d1t-' || true)"
[ -z "$others" ] || die "D1Training robot containers exist ($(echo $others)): stop them first (docker rm -f ...)"

PY_ENV=(-e PYTHONUNBUFFERED=1 -e PYTHONDONTWRITEBYTECODE=1 -e TZ=Australia/Sydney)
# The node reads the remote over ROS (as run_press_on_dog.sh): without P2Dingo's DDS config, the keyboard only.
ROS_ENV=(-e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "${PY_ENV[@]}")
ROS_MOUNTS=(-v "$REPO:/root/D1Training")
if [ -f "$P2/cyclonedds_go2.xml" ]; then
  ROS_ENV+=(-e CYCLONEDDS_URI=file:///root/cyclonedds_go2.xml)
  ROS_MOUNTS+=(-v "$P2/cyclonedds_go2.xml:/root/cyclonedds_go2.xml:ro")
else
  log "no $P2/cyclonedds_go2.xml (set P2DINGO_ROOT): the remote may not be read"
fi
OWNER="$(id -u):$(id -g)"

cleanup() {
  set +e
  docker stop -t 2 "$NODE_NAME" >/dev/null 2>&1
  # SIGTERM: the timing capture writes its summary and exits.
  [ "$TIMING" = 1 ] && docker stop -t 5 "$TIMING_NAME" >/dev/null 2>&1 && docker rm -f "$TIMING_NAME" >/dev/null 2>&1
  # SIGTERM: the arm link holds the arm where it is (it never releases it).
  docker stop -t 3 "$ARM_NAME" >/dev/null 2>&1
  docker logs "$ARM_NAME" >"$REPO/$RUN_DIR/arm_link.log" 2>&1
  docker rm -f "$ARM_NAME" "$NODE_NAME" >/dev/null 2>&1
  docker run --rm -v "$REPO/logs:/logs" --entrypoint chown "$ARM_IMAGE" -R "$OWNER" /logs/holster_dog >/dev/null 2>&1
  log "record: $REPO/$RUN_DIR"
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

ARM_ARGS=(--log "/root/D1Training/$RUN_DIR/arm_link.jsonl")
[ "$LIVE" = 1 ] || ARM_ARGS+=(--observe)
docker run -d --name "$ARM_NAME" --network host --ipc host "${PY_ENV[@]}" \
  -v "$REPO:/root/D1Training" --entrypoint python3 "$ARM_IMAGE" \
  /root/D1Training/demos/combiner_door/robot/arm_link.py "${ARM_ARGS[@]}" >/dev/null
for _ in $(seq 1 60); do
  docker logs "$ARM_NAME" 2>&1 | grep -q "listening on" && break
  if ! docker ps -q -f "name=^${ARM_NAME}$" | grep -q .; then break; fi
  sleep 0.25
done
if ! docker logs "$ARM_NAME" 2>&1 | grep -q "listening on"; then
  docker logs "$ARM_NAME" 2>&1 | tail -5 >&2
  [ "$LIVE" = 1 ] && die "the arm link did not start (above); nothing was moved"
  log "the arm link did not start (above): the dry run uses a simulated arm anyway"
fi

if [ "$TIMING" = 1 ]; then
  docker rm -f "$TIMING_NAME" >/dev/null 2>&1 || true
  docker run -d --name "$TIMING_NAME" --network host "${PY_ENV[@]}" -v "$REPO:/root/D1Training" --entrypoint python3 \
    "$ARM_IMAGE" /root/D1Training/demos/combiner_door/robot/arm_feedback_timing.py --seconds 900 \
    --out "/root/D1Training/$RUN_DIR/feedback_timing" >/dev/null
  log "the arm's report timing is recorded beside it (read-only)"
fi

TTY=(-i); [ -t 0 ] && TTY=(-it)
log "holster draw: $([ "$LIVE" = 1 ] && echo 'LIVE, the real arm' || echo 'dry run, nothing moves')"
docker run "${TTY[@]}" --rm --name "$NODE_NAME" --network host --ipc host "${ROS_ENV[@]}" "${ROS_MOUNTS[@]}" \
  --entrypoint bash "$ROS_IMAGE" \
  -lc 'source /opt/ros/humble/setup.bash && source /root/ws/install/setup.bash && exec python3 /root/D1Training/demos/combiner_key/robot/holster_dog.py "$@"' \
  holster --run_dir "/root/D1Training/$RUN_DIR" "${args[@]}"
