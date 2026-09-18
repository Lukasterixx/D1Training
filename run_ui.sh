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
#   ./run_ui.sh                 # detect where we are: on the dog, a simulator here, the arm here, else the dog
#   ./run_ui.sh sim             # sim mode here, waiting for a simulator if none is up yet
#
# The page is opened in your browser once the server is listening; D1_UI_OPEN=0 turns that off.
# `sim` and `bench` then stay in the foreground -- the terminal is the server's log, Ctrl-C stops it.
#   ./run_ui.sh robot           # deploy to the dog even if a simulator is running here
#   ./run_ui.sh bench           # hardware mode HERE: the arm on this PC's own NIC, no dog, no ssh
#   ./run_ui.sh stop            # stop the server on the dog (the arm is not touched)
#   ./run_ui.sh stop-local      # stop a sim or bench console running on THIS PC
#   ./run_ui.sh status          # is it up? what does it see?
#   ./run_ui.sh logs            # tail the server log on the dog
#   ./run_ui.sh restart         # stop, re-deploy, start
#
# Anything after the subcommand is forwarded to server.py:
#   ./run_ui.sh start --sphere-radius 0.5 --no-legs
#   ./run_ui.sh sim --detect all                # box every COCO class, not just cups
#
# `bench` is for the arm plugged into this PC rather than the Go2. Everything the dog's console does, it
# does here, with two differences: the arm's NIC is this machine's (D1_ARM_IFACE, default the first
# 192.168.123.x interface found) and there is no `rt/lowstate`, so the legs are not drawn. The scripted
# pick is available in this mode, with nothing to set up first.
#
# Double-clicking the script in a file manager runs it with no arguments, which is the detection above.
# On this PC that lands in bench mode, and the pick needs nothing further: it finds the stored camera
# calibration and wrist mount itself, and no height of anything has to be typed.
#
#   ./run_ui.sh bench --pick-calibration pick_demo/assets/calibration/d435i_238222076237_640x480.json
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

# Open the console in a browser once the server is listening. The server binds before it does anything
# slow, so a short wait is enough; the page itself then waits on the backlog until it is served. Set
# D1_UI_OPEN=0 to keep the browser out of it (a headless box, or a terminal you are watching logs in).
open_browser() {
  [ "${D1_UI_OPEN:-1}" = "1" ] || return 0
  local opener=""
  for candidate in xdg-open gio open; do
    command -v "$candidate" >/dev/null 2>&1 && { opener="$candidate"; break; }
  done
  [ -n "$opener" ] || { say "no browser opener found; open $1 yourself"; return 0; }
  [ "$opener" = "gio" ] && opener="gio open"
  # The probe has to be in a subshell: a failed /dev/tcp redirect is reported by the shell itself,
  # and only wrapping the whole thing keeps "Connection refused" out of the console's own log.
  ( for _ in $(seq 40); do
      if ( exec 3<>"/dev/tcp/127.0.0.1/$PORT" ) 2>/dev/null; then break; fi
      sleep 0.25
    done
    $opener "$1" >/dev/null 2>&1 || true ) &
}
# Launched from a file manager ("Run as a Program"), the terminal closes the instant the script exits,
# taking the reason with it -- which is what an 8-second window with nothing in it was. When stdin is
# not a terminal, hold the window open on the way out so the message can be read.
die() {
  printf '\033[31m!!\033[0m %s\n' "$*" >&2
  if [ ! -t 0 ] && [ -r /dev/tty ]; then
    printf '\n[press Enter to close]' >&2
    read -r _ < /dev/tty || sleep 20
  fi
  exit 1
}

# `pkill -f` from an inline `ssh '...'` matches the ssh shell running it and
# kills the connection instead of the server, so every remote action goes
# through a script file written on the dog.
remote_script() {
  local name="$1"; shift
  printf '%s\n' "$@" | ssh "$HOST" "cat > /tmp/$name && chmod +x /tmp/$name"
  ssh "$HOST" "bash /tmp/$name"
}

sim_feed_up() { curl -s --max-time 1 "$SIM_FEED/health" 2>/dev/null | grep -q '"sim": true'; }

# The arm lives on 192.168.123.0/24. On the dog that is enP8p1s0; on a PC it is whatever the arm is
# plugged into, so find it rather than making the user look it up.
find_arm_iface() {
  if [ -n "${D1_ARM_IFACE:-}" ]; then printf '%s\n' "$D1_ARM_IFACE"; return 0; fi
  ip -o -4 addr show 2>/dev/null | awk '$4 ~ /^192\.168\.123\./ {print $2; exit}'
}

run_bench() {
  # Hardware mode on this PC: same env as sim mode (numpy, torch, ultralytics, OpenCV), plus cyclonedds
  # for the arm and pyrealsense2 for the camera.
  local iface
  iface="$(find_arm_iface || true)"
  [ -n "$iface" ] || die "no interface on 192.168.123.0/24 here. Plug the arm in, or set D1_ARM_IFACE."
  say "bench mode: the arm on $iface, no dog (legs are not drawn)"
  local base="${CONDA_EXE:+$(dirname "$(dirname "$CONDA_EXE")")}"
  for candidate in "$base" "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda; do
    if [ -n "$candidate" ] && [ -f "$candidate/etc/profile.d/conda.sh" ]; then base="$candidate"; break; fi
  done
  [ -f "$base/etc/profile.d/conda.sh" ] || die "conda not found; the bench console needs ${ISAAC_SIM_CONDA_ENV:-env_isaaclab}"
  set +u
  # shellcheck disable=SC1091
  . "$base/etc/profile.d/conda.sh"
  conda activate "${ISAAC_SIM_CONDA_ENV:-env_isaaclab}"
  set -u
  unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
  say "http://localhost:$PORT   (DRY RUN until the LIVE switch is on)"
  open_browser "http://localhost:$PORT"
  PYTHONUNBUFFERED=1 exec python d1_ui/server.py --mode hardware --iface "$iface" --no-legs \
    --port "$PORT" "$@"
}

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
  open_browser "http://localhost:$PORT"
  PYTHONUNBUFFERED=1 exec python d1_ui/server.py --mode sim --sim-url "$SIM_FEED" --port "$PORT" "$@"
}

dog_reachable() { ssh -o ConnectTimeout=8 -o BatchMode=yes "$HOST" true 2>/dev/null; }

# Are we running ON the Go2's payload computer? Then there is nothing to deploy and nobody to ssh to:
# serve from here, with the legs, because `rt/lowstate` is on this machine's DDS.
on_the_dog() {
  [ -n "${D1_UI_ON_DOG:-}" ] && return 0
  [ -f /etc/nv_tegra_release ] && return 0                              # a Jetson
  ip -o -4 addr show 2>/dev/null | grep -q " ${HOST_ADDR}/" && return 0  # we are the host we deploy to
  return 1
}

run_dog_local() {
  # The dog runs system python3: no conda there, and the legs are drawn because rt/lowstate is local.
  local iface
  iface="$(find_arm_iface || true)"
  [ -n "$iface" ] || die "on the dog but no interface on 192.168.123.0/24; is the arm powered and plugged in?"
  say "on the dog: hardware mode here, arm on $iface, legs drawn"
  say "http://localhost:$PORT   (DRY RUN until the LIVE switch is on)"
  open_browser "http://localhost:$PORT"
  PYTHONUNBUFFERED=1 exec python3 d1_ui/server.py --mode hardware --iface "$iface" --port "$PORT" "$@"
}

need_dog() {
  dog_reachable \
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
  scp -q pick_demo/__init__.py pick_demo/camera.py pick_demo/perception.py \
         pick_demo/camera_body.py pick_demo/grasp.py "$HOST:$REMOTE_DIR/pick_demo/"
  # The mount editor's camera model. camera_body reads it with trimesh, which the dog does not carry --
  # there it costs the live clearance warning, while the page draws the mesh itself.
  ssh "$HOST" "mkdir -p '$REMOTE_DIR/pick_demo/assets/realsense'"
  scp -q pick_demo/assets/realsense/d435_housing.ply "$HOST:$REMOTE_DIR/pick_demo/assets/realsense/"
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
  if [ "${D1_UI_OPEN:-1}" = "1" ] && command -v xdg-open >/dev/null 2>&1; then
    xdg-open "http://$HOST_ADDR:$PORT" >/dev/null 2>&1 || true   # already up: no need to wait for it
  fi
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

# `sim` and `bench` run here, in the foreground, and Ctrl-C is how they stop -- but a console started in
# a terminal that has since been closed keeps the port, and the next start then fails to bind. This is
# the way out of that, and it is local only: the dog's console is `stop`.
stop_local() {
  local pids
  pids="$(ss -ltnp "sport = :$PORT" 2>/dev/null | sed -n 's/.*pid=\([0-9]*\).*/\1/p' | sort -u)"
  [ -n "$pids" ] || { say "nothing is listening on :$PORT here"; return 0; }
  for pid in $pids; do
    if ps -p "$pid" -o cmd= | grep -q 'd1_ui/server.py'; then
      say "stopping the console on :$PORT (pid $pid); the arm is not touched"
      kill "$pid" 2>/dev/null || true
    else
      die ":$PORT is held by pid $pid, which is not a d1_ui console: $(ps -p "$pid" -o cmd=)"
    fi
  done
}

cmd="${1:-start}"; [ $# -gt 0 ] && shift || true
case "$cmd" in
  start)
    # Work out where we are and what is attached, then just start the console -- a bare `./run_ui.sh`,
    # which is what a double-click runs, should not need an argument to do the obvious thing.
    #
    #   on the dog        -> serve here, with the legs. Nothing to deploy, nobody to ssh to.
    #   simulator here    -> sim mode, following it.
    #   arm on this NIC   -> bench. The arm being plugged into this PC is the signal: if it were on the
    #                        dog there would be no 192.168.123.x interface here. Checked before the dog
    #                        so a click is instant rather than waiting out an ssh timeout first.
    #   dog reachable     -> deploy and start it there, the original behaviour.
    #
    # Every path starts in DRY RUN: the console reads the arm and commands nothing until LIVE is
    # switched on in the page. `sim`, `bench` and `robot` force one of these if the guess is wrong.
    if on_the_dog; then run_dog_local "$@"; fi
    if sim_feed_up; then
      say "a simulator is publishing at $SIM_FEED: sim mode, served from this PC (./run_ui.sh robot for the dog)"
      run_local "$@"
    fi
    if [ -n "$(find_arm_iface || true)" ]; then
      say "the arm is on this PC's own NIC: bench mode (./run_ui.sh robot deploys to the dog instead)"
      run_bench "$@"
    fi
    need_dog; deploy; start "$@" ;;
  sim)     run_local "$@" ;;
  bench)   run_bench "$@" ;;
  robot)   need_dog; deploy; start "$@" ;;
  restart) need_dog; stop; deploy; start "$@" ;;
  stop)    need_dog; stop ;;
  stop-local) stop_local ;;
  status)  need_dog; status ;;
  logs)    need_dog; logs ;;
  deploy)  need_dog; deploy ;;
  *) die "unknown command '$cmd'. Use: start | sim | bench | robot | restart | stop | stop-local | status | logs | deploy" ;;
esac
