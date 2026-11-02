#!/usr/bin/env bash
# The combiner door demo on the real dog: run ON the dog, from its D1Training checkout (`d1dog` on the PC does the
# SSH). It starts the arm link (arm_link.py, in P2Dingo's arm image) and the robot program (node.py, in P2Dingo's
# ROS image), both on the host network beside P2Dingo's containers, and on the way out sends StopMove and stops the
# arm link, which holds the arm where it is. Watch it from the PC with view_on_pc.sh (`d1view`).
#
#   demos/combiner_door/robot/run_on_dog.sh                          # watch: where the box is; nothing moves
#   demos/combiner_door/robot/run_on_dog.sh --stage pull             # the whole attempt as a dry run
#   demos/combiner_door/robot/run_on_dog.sh --stage unfold --live    # unfold the arm and fold it again
#   demos/combiner_door/robot/run_on_dog.sh --stage grip --live      # look, unfold, walk in, reach, grip, back off, fold
#   demos/combiner_door/robot/run_on_dog.sh --task key --stage turn  # the key turned by the arm, dry (key_plan.py)
#   demos/combiner_door/robot/run_on_dog.sh --task key --stage turn --then_door pull  # then the door pulled from there
#   demos/combiner_door/robot/run_on_dog.sh --task insert --stage insert  # the key handed to the arm and inserted
#   demos/combiner_door/robot/run_on_dog.sh --stage pull --then_close  # the door pulled open, closed, then locked
#   demos/combiner_door/robot/run_on_dog.sh --task insert --stage insert --lock  # the key in to lock the door
#
# Every argument goes to node.py (--task, --stage, --then_door, --then_close, --lock, --camera, --live, --yes, --fast, --speed_scale,
# --door_deg, --odom, --no_fold, --no_return;
# --help). Once a run is over the dog walks back to where it started (not after an abort; --no_return stays).
# --loop (the whole chain, d1dogdemo): once it has locked the door and walked back, the chain runs again by itself, a
# record each time, 5 s after the last (Ctrl-C stops); any other end stops the loop.
# A live run waits for one Enter after showing its plan, then runs through; `a` + Enter stops it. Keep the Unitree remote
# in hand. The record is logs/combiner_door_robot/<run>/ (`d1fetch` on the PC copies it back).
set -euo pipefail

REPO="$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)"
P2="${P2DINGO_ROOT:-$HOME/P2Dingo}"
ROS_IMAGE="${D1T_ROS_IMAGE:-p2dingo-web:latest}"
ARM_IMAGE="${D1T_ARM_IMAGE:-p2dingo-arm:latest}"
NODE_NAME=d1t-combiner-door
ARM_NAME=d1t-arm-link

log() { printf '[run_on_dog] %s\n' "$*" >&2; }
die() { printf '[run_on_dog] ERROR: %s\n' "$*" >&2; exit 1; }

case "${1:-}" in
  -h|--help) awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "${BASH_SOURCE[0]}"; exit 0 ;;
esac
command -v docker >/dev/null || die "docker is not installed (is this the dog?)"
for image in "$ROS_IMAGE" "$ARM_IMAGE"; do
  docker image inspect "$image" >/dev/null 2>&1 || die "no image $image: P2Dingo's split stack builds it"
done
[ -f "$P2/cyclonedds_go2.xml" ] || die "no $P2/cyclonedds_go2.xml (set P2DINGO_ROOT)"

LIVE=0
STAGE=watch
TASK=""
THEN=""
CLOSE=""
CAMERA=front
FAST=""
LOOP=0
args=("$@")
for ((i = 0; i < ${#args[@]}; i++)); do
  case "${args[$i]}" in
    --live) LIVE=1 ;;
    --stage) STAGE="${args[$((i + 1))]:-}" ;;
    --stage=*) STAGE="${args[$i]#--stage=}" ;;
    --release) STAGE=release ;;
    --task) case "${args[$((i + 1))]:-}" in key|insert) TASK="${args[$((i + 1))]}_" ;; esac ;;
    --task=key|--task=insert) TASK="${args[$i]#--task=}_" ;;
    --then_door) THEN="_door_${args[$((i + 1))]:-}" ;;
    --then_door=*) THEN="_door_${args[$i]#--then_door=}" ;;
    --then_close) CLOSE="_close" ;;
    --lock) CLOSE="_lock" ;;
    --fast) FAST="_fast" ;;
    --loop) LOOP=1 ;;
    --camera) CAMERA="${args[$((i + 1))]:-}" ;;
    --camera=*) CAMERA="${args[$i]#--camera=}" ;;
  esac
done

# The look's default camera is the Go2's front camera, which only P2Dingo's front camera container puts on ROS.
if [ "$CAMERA" = front ] && [ "$STAGE" != release ] \
   && ! docker ps -q -f "name=^p2dingo-front-camera$" | grep -q .; then
  die "the look uses the Go2's front camera and p2dingo-front-camera is not running: cd $P2 && docker compose up -d front_camera (or --camera wrist)"
fi

SUFFIX=dry; [ "$LIVE" = 1 ] && SUFFIX=live
RUN_DIR=""
new_run_dir() {
  STAMP="$(date -u +%Y%m%dT%H%M%S)"
  RUN_DIR="logs/combiner_door_robot/${STAMP}_${TASK}${STAGE}${THEN}${CLOSE}${FAST}_${SUFFIX}"
  mkdir -p "$REPO/$RUN_DIR"
}

for name in "$NODE_NAME" "$ARM_NAME"; do
  docker rm -f "$name" >/dev/null 2>&1 || true
done

# The containers run as root over this checkout: no bytecode left in it, and the record handed back on the way out.
PY_ENV=(-e PYTHONUNBUFFERED=1 -e PYTHONDONTWRITEBYTECODE=1)
# The NVIDIA runtime, as P2Dingo's `p2dingo` service has it: the image's OpenCV links Jetson libraries the runtime mounts.
ROS_ENV=(--runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all
         -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///root/cyclonedds_go2.xml
         -e TZ=Australia/Sydney "${PY_ENV[@]}")
ROS_MOUNTS=(-v "$P2/cyclonedds_go2.xml:/root/cyclonedds_go2.xml:ro" -v "$REPO:/root/D1Training")
ROS_SETUP='source /opt/ros/humble/setup.bash && source /root/ws/install/setup.bash'
OWNER="$(id -u):$(id -g)"

# One run's end: the node stopped, StopMove, the arm link stopped (it holds the arm where it is) and its log kept with
# the run's record. Once per run (`RUN_OPEN`), whether the run ended or the script is on its way out.
RUN_OPEN=0
end_run() {
  [ "$RUN_OPEN" = 1 ] || return 0
  RUN_OPEN=0
  set +e                                      # every step whatever became of the others (the node already gone)
  docker stop -t 2 "$NODE_NAME" >/dev/null 2>&1
  if [ "$LIVE" = 1 ]; then
    log "StopMove"
    docker run --rm --network host --ipc host "${ROS_ENV[@]}" "${ROS_MOUNTS[@]}" --entrypoint bash "$ROS_IMAGE" \
      -lc "$ROS_SETUP && python3 /root/D1Training/demos/combiner_door/robot/node.py --stop_only" >/dev/null 2>&1
  fi
  # SIGTERM: the arm link holds the arm where it is (it never releases it).
  docker stop -t 3 "$ARM_NAME" >/dev/null 2>&1
  docker logs "$ARM_NAME" >"$REPO/$RUN_DIR/arm_link.log" 2>&1
  docker rm -f "$ARM_NAME" >/dev/null 2>&1
  docker run --rm -v "$REPO/logs:/logs" --entrypoint chown "$ARM_IMAGE" -R "$OWNER" /logs/combiner_door_robot \
    >/dev/null 2>&1
  log "record: $REPO/$RUN_DIR"
  set -e
}
cleanup() {
  set +e
  end_run
}
trap cleanup EXIT
# A signal (the SSH session dropping, Ctrl-C outside the container) exits normally, so the cleanup runs.
trap 'exit 130' INT TERM HUP

# The arm link: taking targets in a live run, only reporting in a dry one (so RViz draws the real arm either way).
start_arm_link() {
  local -a arm_args=(--log "/root/D1Training/$RUN_DIR/arm_link.jsonl")
  [ "$LIVE" = 1 ] || arm_args+=(--observe)
  docker run -d --name "$ARM_NAME" --network host --ipc host "${PY_ENV[@]}" \
    -v "$REPO:/root/D1Training" --entrypoint python3 "$ARM_IMAGE" \
    /root/D1Training/demos/combiner_door/robot/arm_link.py "${arm_args[@]}" >/dev/null
  for _ in $(seq 1 60); do
    docker logs "$ARM_NAME" 2>&1 | grep -q "listening on" && break
    if ! docker ps -q -f "name=^${ARM_NAME}$" | grep -q .; then break; fi
    sleep 0.25
  done
  if ! docker logs "$ARM_NAME" 2>&1 | grep -q "listening on"; then
    docker logs "$ARM_NAME" 2>&1 | tail -5 >&2
    [ "$LIVE" = 1 ] && die "the arm link did not start (above); nothing was started"
    log "the arm link did not start (above): the arm will not be drawn"
  fi
}

TTY=(-i); [ -t 0 ] && TTY=(-it)
node_args=("$@")
runs=0
while :; do
  runs=$((runs + 1))
  new_run_dir
  RUN_OPEN=1
  start_arm_link
  log "robot program: $([ "$LIVE" = 1 ] && echo LIVE || echo 'dry run, nothing moves'), stage $STAGE$([ "$LOOP" = 1 ] && echo ", run $runs of the loop")"
  docker run "${TTY[@]}" --rm --name "$NODE_NAME" --network host --ipc host "${ROS_ENV[@]}" "${ROS_MOUNTS[@]}" \
    --entrypoint bash "$ROS_IMAGE" \
    -lc "$ROS_SETUP && exec python3 /root/D1Training/demos/combiner_door/robot/node.py --run_dir /root/D1Training/$RUN_DIR \"\$@\"" \
    node "${node_args[@]}"
  end_run
  [ "$LOOP" = 1 ] || break
  record="$REPO/$RUN_DIR/run.json"
  if ! grep -q '"repeatable": true' "$record" 2>/dev/null; then
    outcome="$(python3 -c 'import json, sys; r = json.load(open(sys.argv[1])); print(r.get("abort") or r.get("outcome") or r.get("status"))' "$record" 2>/dev/null || echo "no record")"
    log "--loop: not again after run $runs ($outcome): the door locked and the dog back where it started is where it begins"
    break
  fi
  # The operator's one Enter was for the loop: the runs after it start by themselves.
  [ "$runs" = 1 ] && node_args+=(--yes)
  log "--loop: run $((runs + 1)) starts in 5 s (Ctrl-C stops)"
  sleep 5
done
