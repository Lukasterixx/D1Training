#!/usr/bin/env bash
# Deploy and run the D1 reach console on the Go2's Jetson payload.
#
# The console has to run ON the dog: CycloneDDS only reaches the arm from the
# Jetson's arm-facing NIC (192.168.123.x). This script copies what the server
# needs, starts it there, and prints the URL. Run it from this PC.
#
#   ./run_ui.sh                 # deploy and start, print the URL
#   ./run_ui.sh stop            # stop the server (the arm is not touched)
#   ./run_ui.sh status          # is it up? what does it see?
#   ./run_ui.sh logs            # tail the server log on the dog
#   ./run_ui.sh restart         # stop, re-deploy, start
#
# Anything after the subcommand is forwarded to server.py:
#   ./run_ui.sh start --sphere-radius 0.5 --no-legs
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

deploy() {
  say "deploying to $HOST:$REMOTE_DIR"
  ssh "$HOST" "mkdir -p '$REMOTE_DIR'/{d1_ui/static/vendor,position_only,d1_arm/meshes,description/meshes/go2}"
  # The server's import closure, plus the geometry the page draws.
  scp -q d1_ik.py d1_hardware.py "$HOST:$REMOTE_DIR/"
  scp -q position_only/__init__.py position_only/tool_point.py position_only/task_space.py \
         position_only/workspace.py "$HOST:$REMOTE_DIR/position_only/"
  scp -q d1_ui/__init__.py d1_ui/server.py "$HOST:$REMOTE_DIR/d1_ui/"
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
print('  live      ', s['live'], '(dry run)' if not s['live'] else '(COMMANDS REACH THE ARM)')
print('  arm       ', s['servo_deg'], 'enable', s['enable'], 'error', s['error'])
print('  feedback  ', round(s['feedback_age_s'],2), 's old')
print('  go2 legs  ', 'live' if s['legs_live'] else 'not subscribed')
print('  busy      ', s['busy'] or 'idle')
\" || echo '  (server up but /state did not answer)'"
}

logs() { ssh -t "$HOST" "tail -n 40 -f $LOG"; }

ssh -o ConnectTimeout=8 -o BatchMode=yes "$HOST" true 2>/dev/null \
  || die "cannot reach $HOST over ssh. Is the dog up and on Tailscale? (try: sshuni)"

cmd="${1:-start}"; [ $# -gt 0 ] && shift || true
case "$cmd" in
  start)   deploy; start "$@" ;;
  restart) stop; deploy; start "$@" ;;
  stop)    stop ;;
  status)  status ;;
  logs)    logs ;;
  deploy)  deploy ;;
  *) die "unknown command '$cmd'. Use: start | restart | stop | status | logs | deploy" ;;
esac
