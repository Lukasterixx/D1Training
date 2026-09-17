#!/usr/bin/env bash
# Run the D1 reach console: next to a simulator, or on the Go2's Jetson payload.
#
# Sim or hardware is detected first. If a simulator is publishing on this PC
# (./run_pick_demo.sh or ./run_sim.sh; d1_ui/sim_feed.py), the console runs here
# in sim mode and follows it: joints, legs, and the rendered wrist camera with
# YOLO boxes. Commands to the arm are refused in that mode.
#
# Otherwise it deploys to the dog. The hardware console has to run ON the dog:
# CycloneDDS only reaches the arm from the Jetson's arm-facing NIC
# (192.168.123.x). This script copies what the server needs, starts it there,
# and prints the URL. Run it from this PC.
#
#   ./run_ui.sh                 # detect: sim here if a simulator is up, else deploy to the dog
#   ./run_ui.sh sim             # sim mode here, waiting for a simulator if none is up yet
#   ./run_ui.sh robot           # deploy to the dog even if a simulator is running here
#   ./run_ui.sh stop            # stop the server (the arm is not touched)
#   ./run_ui.sh status          # is it up? what does it see?
#   ./run_ui.sh logs            # tail the server log on the dog
#   ./run_ui.sh restart         # stop, re-deploy, start
#
# Anything after the subcommand is forwarded to server.py:
#   ./run_ui.sh start --sphere-radius 0.5 --no-legs
#   ./run_ui.sh sim --detect all                # box every COCO class, not just cups
#
# The remote copy lives under /tmp, which does NOT survive a reboot of the dog.
# Re-run this script after one; it re-deploys every time, so that is the fix for
# "it worked yesterday".
#
# The server starts in DRY RUN. Nothing reaches the arm until the LIVE switch in
# the page is turned on, and that state lives in the server, not the browser.
set -euo pipefail

cd "$(dirname "$0")"

# Same target the rest of the project uses; override with GO2_ROBOT or D1_UI_HOST.
HOST="${D1_UI_HOST:-${GO2_ROBOT:-unitree@100.99.23.36}}"
REMOTE_DIR="${D1_UI_REMOTE_DIR:-/tmp/d1train}"
PORT="${D1_UI_PORT:-8090}"
LOG="/tmp/d1_ui.log"
HOST_ADDR="${HOST#*@}"
SIM_FEED="${D1_UI_SIM_FEED:-http://127.0.0.1:8765}"
WEIGHTS="generated/yolo/yolo11s-seg.pt"

say() { printf '\033[36m==\033[0m %s\n' "$*"; }
die() { printf '\033[31m!!\033[0m %s\n' "$*" >&2; exit 1; }

# `pkill -f` from an inline `ssh '...'` matches the ssh shell running it and
# kills the connection instead of the server, so every remote action goes
# through a script file written on the dog.
remote_script() {
  local name="$1"; shift
  printf '%s\n' "$@" | ssh "$HOST" "cat > /tmp/$name && chmod +x /tmp/$name"
  ssh "$HOST" "bash /tmp/$name"
}

sim_feed_up() { curl -s --max-time 1 "$SIM_FEED/health" 2>/dev/null | grep -q '"sim": true'; }

run_local() {
  # Sim mode serves from this PC, in the Isaac conda env for numpy, torch, ultralytics and OpenCV; Isaac itself
  # is not started. No ROS in this shell, as for every Isaac script.
  local base="${CONDA_EXE:+$(dirname "$(dirname "$CONDA_EXE")")}"
  for candidate in "$base" "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda; do
    if [ -n "$candidate" ] && [ -f "$candidate/etc/profile.d/conda.sh" ]; then base="$candidate"; break; fi
  done
  [ -f "$base/etc/profile.d/conda.sh" ] || die "conda not found; the sim-mode console needs the ${ISAAC_SIM_CONDA_ENV:-env_isaaclab} env"
  set +u
  # shellcheck disable=SC1091
  . "$base/etc/profile.d/conda.sh"
  conda activate "${ISAAC_SIM_CONDA_ENV:-env_isaaclab}"
  set -u
  unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
  say "sim mode: http://localhost:$PORT  (Ctrl-C stops it; the simulator keeps running)"
  PYTHONUNBUFFERED=1 exec python d1_ui/server.py --mode sim --sim-url "$SIM_FEED" --port "$PORT" "$@"
}

need_dog() {
  ssh -o ConnectTimeout=8 -o BatchMode=yes "$HOST" true 2>/dev/null \
    || die "cannot reach $HOST over ssh. Is the dog up and on Tailscale? (try: sshuni)  For the simulator: start ./run_pick_demo.sh, then ./run_ui.sh (or ./run_ui.sh sim)."
}

deploy() {
  say "deploying to $HOST:$REMOTE_DIR"
  ssh "$HOST" "mkdir -p '$REMOTE_DIR'/{d1_ui/static/vendor,position_only,pick_demo,generated/yolo,d1_arm/meshes,description/meshes/go2}"
  # The server's import closure, plus the geometry the page draws.
  scp -q d1_ik.py d1_hardware.py "$HOST:$REMOTE_DIR/"
  scp -q position_only/__init__.py position_only/tool_point.py position_only/task_space.py \
         position_only/workspace.py "$HOST:$REMOTE_DIR/position_only/"
  scp -q d1_ui/__init__.py d1_ui/server.py d1_ui/sim_feed.py d1_ui/camera_feed.py "$HOST:$REMOTE_DIR/d1_ui/"
  # The camera window's detector: the pick demo's YOLO wrapper and weights (the weights only once).
  scp -q pick_demo/__init__.py pick_demo/camera.py pick_demo/perception.py "$HOST:$REMOTE_DIR/pick_demo/"
  if [ -f "$WEIGHTS" ] && ! ssh "$HOST" "test -f '$REMOTE_DIR/$WEIGHTS'"; then
    scp -q "$WEIGHTS" "$HOST:$REMOTE_DIR/$WEIGHTS"
  fi
  scp -q d1_ui/static/index.html d1_ui/static/app.js d1_ui/static/style.css "$HOST:$REMOTE_DIR/d1_ui/static/"
  scp -q d1_ui/static/vendor/*.js "$HOST:$REMOTE_DIR/d1_ui/static/vendor/"
  scp -q d1_arm/d1.urdf "$HOST:$REMOTE_DIR/d1_arm/"
  scp -q d1_arm/meshes/*.STL "$HOST:$REMOTE_DIR/d1_arm/meshes/"
  scp -q description/go2_d1.urdf "$HOST:$REMOTE_DIR/description/"
  scp -q description/meshes/go2/*.dae "$HOST:$REMOTE_DIR/description/meshes/go2/"
}

start() {
  remote_script d1_ui_start.sh \
    "cd '$REMOTE_DIR'" \
    "pkill -f 'python3 d1_ui/server.py' 2>/dev/null || true" \
    "sleep 0.5" \
    "nohup python3 d1_ui/server.py --port $PORT $* > $LOG 2>&1 &" \
    "sleep 4" \
    "pgrep -f 'python3 d1_ui/server.py' >/dev/null || { echo '--- server failed to start ---'; tail -20 $LOG; exit 1; }" \
    "head -1 $LOG"
  say "up at  http://$HOST_ADDR:$PORT"
  say "starts in DRY RUN; turn on LIVE in the page to command the arm"
}

stop() {
  remote_script d1_ui_stop.sh \
    "pkill -f 'python3 d1_ui/server.py' 2>/dev/null && echo stopped || echo 'not running'"
  say "the arm was not touched; it holds whatever pose it was in"
}

status() {
  remote_script d1_ui_status.sh \
    "pgrep -af 'python3 d1_ui/server.py' || { echo 'not running'; exit 0; }" \
    "curl -s --max-time 5 localhost:$PORT/state | python3 -c \"
import json,sys
s=json.load(sys.stdin)
print('  url        http://$HOST_ADDR:$PORT')
print('  mode      ', s.get('mode'), '-', s.get('mode_reason'))
print('  camera    ', (s.get('camera') or {}).get('source'), (s.get('camera') or {}).get('message') or 'streaming')
print('  live      ', s['live'], '(dry run)' if not s['live'] else '(COMMANDS REACH THE ARM)')
print('  arm       ', s['servo_deg'], 'enable', s['enable'], 'error', s['error'])
print('  feedback  ', round(s['feedback_age_s'],2), 's old')
print('  go2 legs  ', 'live' if s['legs_live'] else 'not subscribed')
print('  busy      ', s['busy'] or 'idle')
\" || echo '  (server up but /state did not answer)'"
}

logs() { ssh -t "$HOST" "tail -n 40 -f $LOG"; }

cmd="${1:-start}"; [ $# -gt 0 ] && shift || true
case "$cmd" in
  start)
    if sim_feed_up; then
      say "a simulator is publishing at $SIM_FEED: sim mode, served from this PC (./run_ui.sh robot for the dog)"
      run_local "$@"
    fi
    need_dog; deploy; start "$@" ;;
  sim)     run_local "$@" ;;
  robot)   need_dog; deploy; start "$@" ;;
  restart) need_dog; stop; deploy; start "$@" ;;
  stop)    need_dog; stop ;;
  status)  need_dog; status ;;
  logs)    need_dog; logs ;;
  deploy)  need_dog; deploy ;;
  *) die "unknown command '$cmd'. Use: start | sim | robot | restart | stop | status | logs | deploy" ;;
esac
