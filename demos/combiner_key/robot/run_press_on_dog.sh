#!/usr/bin/env bash
# The general foot-press policy driving the dog's front-right leg onto the operator's hand (press_live.py). DRY unless
# --live: dry, it sends NOTHING and runs the policy open loop beside sport mode; live, it lies the dog down, releases
# sport mode, stands it up and drives all twelve leg motors, then lies it down and gives sport mode back. Run ON the dog,
# from its D1Training checkout, beside P2Dingo's containers, with the robot standing in sport mode and the arm folded.
#
#   demos/combiner_key/robot/run_press_on_dog.sh --policy checkpoints/leg_press_force_r4_model_5999.npz
#   demos/combiner_key/robot/run_press_on_dog.sh --policy ... --rehearse     # dry, a pretend foot walks the sequence
#   demos/combiner_key/robot/run_press_on_dog.sh --policy ... --live         # the legs driven (typed confirmation)
#   demos/combiner_key/robot/run_press_on_dog.sh --policy ... --no_arm       # no arm link: the arm at its rest pose
#
# It starts the arm link (demos/combiner_door/robot/arm_link.py) with --observe, which only reports the D1's angles,
# then press_live.py in P2Dingo's ROS image; every other argument goes to press_live.py (--live, --plate, --force_n,
# --presses; --help). Enter steps it on, Ctrl-C or the remote's B stops it into damping. The record is
# logs/leg_press/<stamp>/ (`d1fetch` copies it back).
set -euo pipefail

REPO="$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)"
P2="${P2DINGO_ROOT:-$HOME/P2Dingo}"
ROS_IMAGE="${D1T_ROS_IMAGE:-p2dingo-web:latest}"
ARM_IMAGE="${D1T_ARM_IMAGE:-p2dingo-arm:latest}"
NODE_NAME=d1t-leg-press
ARM_NAME=d1t-arm-link

log() { printf '[run_press_on_dog] %s\n' "$*" >&2; }
die() { printf '[run_press_on_dog] ERROR: %s\n' "$*" >&2; exit 1; }

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
mkdir -p "$REPO/logs/leg_press"

# Another robot program (the door program, the shadow runner) shares the arm link's name: never remove it from under
# one, and never drive the legs beside one.
others="$(docker ps -a --format '{{.Names}}' | grep -E '^d1t-' || true)"
[ -z "$others" ] || die "D1Training robot containers exist ($(echo $others)): stop them first (docker rm -f ...)"
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
  # SIGTERM ends the node's loop and it saves the record (~0.5 s per 90 s recorded on the dog) before SIGKILL.
  docker stop -t 10 "$NODE_NAME" >/dev/null 2>&1
  if [ "$ARM" = 1 ]; then
    docker stop -t 3 "$ARM_NAME" >/dev/null 2>&1
    docker logs "$ARM_NAME" >"$REPO/logs/leg_press/${STAMP}_arm_link.log" 2>&1
    docker rm -f "$ARM_NAME" >/dev/null 2>&1
  fi
  docker run --rm -v "$REPO/logs:/logs" --entrypoint chown "$ROS_IMAGE" -R "$OWNER" /logs/leg_press >/dev/null 2>&1
  log "records: $REPO/logs/leg_press/"
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

if [ "$ARM" = 1 ]; then
  if docker image inspect "$ARM_IMAGE" >/dev/null 2>&1; then
    # --observe: the link reports the arm's angles and takes no target.
    docker run -d --name "$ARM_NAME" --network host --ipc host "${PY_ENV[@]}" \
      -v "$REPO:/root/D1Training" --entrypoint python3 "$ARM_IMAGE" \
      /root/D1Training/demos/combiner_door/robot/arm_link.py --observe \
      --log "/root/D1Training/logs/leg_press/${STAMP}_arm_link.jsonl" >/dev/null
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
case " ${args[*]} " in
  *" --live "*) log "LIVE: this program will drive the legs" ;;
  *) log "dry: nothing is sent to the robot" ;;
esac
docker run "${TTY[@]}" --rm --name "$NODE_NAME" --network host --ipc host "${ROS_ENV[@]}" "${ROS_MOUNTS[@]}" \
  --entrypoint bash "$ROS_IMAGE" \
  -lc 'source /opt/ros/humble/setup.bash && source /root/ws/install/setup.bash && cd /root/D1Training && exec python3 demos/combiner_key/robot/press_live.py --run_dir /root/D1Training/logs/leg_press "$@"' \
  press "${args[@]}"
