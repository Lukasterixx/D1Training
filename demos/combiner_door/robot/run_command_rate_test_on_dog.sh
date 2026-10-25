#!/usr/bin/env bash
# The D1 commanded its own pose at several rates, its report timing recorded beside it (arm_command_rate_test.py,
# arm_feedback_timing.py): does commanding make it pause its joint reports (F-200), and does the rate or the gripper's
# angle in each command? The arm stays where it is (folded); it is commanded, so run it with the dog still and nothing
# near the arm. Run ON the dog, from its D1Training checkout.
#
#   demos/combiner_door/robot/run_command_rate_test_on_dog.sh               # six 20 s blocks, ~2.5 min
#   demos/combiner_door/robot/run_command_rate_test_on_dog.sh --block_s 10
#
# The record is logs/arm_command_rate/<stamp>/ (rate_test.json, feedback_timing/); `d1fetch` copies it back, then
# `python demos/combiner_door/robot/arm_command_rate_test.py --analyse <dir>`.
set -euo pipefail

REPO="$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)"
ARM_IMAGE="${D1T_ARM_IMAGE:-p2dingo-arm:latest}"
TEST_NAME=d1t-rate-test
TIMING_NAME=d1t-feedback-timing

log() { printf '[run_command_rate_test] %s\n' "$*" >&2; }
die() { printf '[run_command_rate_test] ERROR: %s\n' "$*" >&2; exit 1; }

case "${1:-}" in
  -h|--help) awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "${BASH_SOURCE[0]}"; exit 0 ;;
esac
command -v docker >/dev/null || die "docker is not installed (is this the dog?)"
docker image inspect "$ARM_IMAGE" >/dev/null 2>&1 || die "no image $ARM_IMAGE"
others="$(docker ps -a --format '{{.Names}}' | grep -E '^d1t-' || true)"
[ -z "$others" ] || die "D1Training robot containers exist ($(echo $others)): stop them first (docker rm -f ...)"

STAMP="$(date -u +%Y%m%dT%H%M%S)"
RUN_DIR="logs/arm_command_rate/$STAMP"
mkdir -p "$REPO/$RUN_DIR"
PY_ENV=(-e PYTHONUNBUFFERED=1 -e PYTHONDONTWRITEBYTECODE=1 -e TZ=Australia/Sydney)
OWNER="$(id -u):$(id -g)"

cleanup() {
  set +e
  # SIGTERM: the test stops its stream (the arm holds where it is); the capture writes its summary.
  docker stop -t 3 "$TEST_NAME" >/dev/null 2>&1
  docker stop -t 5 "$TIMING_NAME" >/dev/null 2>&1
  docker rm -f "$TEST_NAME" "$TIMING_NAME" >/dev/null 2>&1
  docker run --rm -v "$REPO/logs:/logs" --entrypoint chown "$ARM_IMAGE" -R "$OWNER" /logs/arm_command_rate >/dev/null 2>&1
  log "record: $REPO/$RUN_DIR"
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

docker run -d --name "$TIMING_NAME" --network host "${PY_ENV[@]}" -v "$REPO:/root/D1Training" --entrypoint python3 \
  "$ARM_IMAGE" /root/D1Training/demos/combiner_door/robot/arm_feedback_timing.py --seconds 900 \
  --out "/root/D1Training/$RUN_DIR/feedback_timing" >/dev/null
sleep 2
TTY=(-i); [ -t 0 ] && TTY=(-it)
log "commanding the arm its own pose (it should not move); Ctrl-C stops the stream and the arm holds"
docker run "${TTY[@]}" --rm --name "$TEST_NAME" --network host "${PY_ENV[@]}" -v "$REPO:/root/D1Training" \
  --entrypoint python3 "$ARM_IMAGE" /root/D1Training/demos/combiner_door/robot/arm_command_rate_test.py \
  --out "/root/D1Training/$RUN_DIR" "$@"
