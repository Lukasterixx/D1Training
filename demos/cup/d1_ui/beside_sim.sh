# shellcheck shell=bash
# The reach console beside a simulator, for the launchers that start one: sourced by demos/cup/run_pick_demo.sh
# and demos/combiner/run_combiner_demo.sh, from the repository root, after conda is activated.
#
# demos/cup/d1_ui/server.py runs in sim mode, following the simulator's feed (joints, legs, and the wrist camera;
# demos/cup/d1_ui/sim_feed.py) on http://localhost:8090, and is stopped when the launcher exits. A viewer run opens
# a browser tab once the simulator is publishing; a --headless one does not. --no_console leaves it out (as does
# D1_UI_CONSOLE=0), and ./demos/cup/run_ui.sh still runs one by hand. D1_UI_PORT and D1_UI_LOG move the console
# and its log; D1_UI_ARGS reaches server.py last, so it overrides a launcher's own console options.
#
#   . demos/cup/d1_ui/beside_sim.sh
#   console_args "$@"; set -- ${SIM_ARGS[@]+"${SIM_ARGS[@]}"}
#   console_start run_pick_demo [server.py options...]
#   python the_simulator.py "$@"      # not `exec`: the launcher's exit is what stops the console
#
# Written to run under `set -euo pipefail` as well as plain `set -e`.

CONSOLE="${D1_UI_CONSOLE:-1}"
CONSOLE_PORT="${D1_UI_PORT:-8090}"
CONSOLE_LOG="${D1_UI_LOG:-/tmp/d1_ui_sim.log}"
CONSOLE_TAG="console"
CONSOLE_PID=""
CONSOLE_WAITER=""
FEED_PORT=8765
HEADLESS=0
SIM_ARGS=()

# --no_console is the launcher's own; everything else goes to the simulator untouched, in SIM_ARGS. The feed port
# and --headless are read (not consumed) because the console needs the one and the browser tab depends on the other.
console_args() {
  while [ $# -gt 0 ]; do
    case "$1" in
      --no_console) CONSOLE=0 ;;
      -h|--help) CONSOLE=0; SIM_ARGS+=("$1") ;;
      --headless) HEADLESS=1; SIM_ARGS+=("$1") ;;
      --ui_feed_port) SIM_ARGS+=("$1"); if [ $# -gt 1 ]; then FEED_PORT="$2"; SIM_ARGS+=("$2"); shift; fi ;;
      --ui_feed_port=*) FEED_PORT="${1#*=}"; SIM_ARGS+=("$1") ;;
      *) SIM_ARGS+=("$1") ;;
    esac
    shift
  done
}

stop_console() {
  if [ -n "$CONSOLE_WAITER" ]; then kill "$CONSOLE_WAITER" 2>/dev/null || true; fi
  if [ -n "$CONSOLE_PID" ] && kill -0 "$CONSOLE_PID" 2>/dev/null; then
    kill "$CONSOLE_PID" 2>/dev/null || true
    wait "$CONSOLE_PID" 2>/dev/null || true
    echo "[$CONSOLE_TAG] reach console stopped."
  fi
  CONSOLE_PID=""
}

# Start the console, left waiting: `--mode sim` polls the feed rather than giving up when none answers yet, so it is
# up by the time Isaac finishes loading. Its own output would bury the simulator's lines, so it goes to a log. The
# console watches; it cannot command anything in sim mode. Arguments after the tag go to server.py.
console_start() {
  CONSOLE_TAG="$1"; shift
  trap stop_console EXIT
  trap 'stop_console; exit 130' INT TERM
  if [ "$CONSOLE" != "1" ] || [ "$FEED_PORT" = "0" ]; then
    return 0
  fi
  if curl -s --max-time 1 "http://127.0.0.1:$CONSOLE_PORT/state" >/dev/null 2>&1; then
    echo "[$CONSOLE_TAG] a console is already serving http://localhost:$CONSOLE_PORT; leaving that one alone." >&2
    return 0
  fi
  # D1_UI_ARGS is for the console's own options: the detector on the CPU when the viewer wants the VRAM
  # (D1_UI_ARGS='--yolo-device cpu'), every COCO class (--detect all), no legs.
  # shellcheck disable=SC2086
  python demos/cup/d1_ui/server.py --mode sim --sim-url "http://127.0.0.1:$FEED_PORT" --port "$CONSOLE_PORT" \
    "$@" ${D1_UI_ARGS:-} > "$CONSOLE_LOG" 2>&1 &
  CONSOLE_PID=$!
  # Give it a moment to bind, and say so plainly if it died instead: the simulator itself does not need it.
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    curl -s --max-time 1 "http://127.0.0.1:$CONSOLE_PORT/state" >/dev/null 2>&1 && break
    kill -0 "$CONSOLE_PID" 2>/dev/null || break
    sleep 1
  done
  if ! kill -0 "$CONSOLE_PID" 2>/dev/null; then
    CONSOLE_PID=""
    echo "[$CONSOLE_TAG] the reach console did not start; the simulator runs without it. Last lines of $CONSOLE_LOG:" >&2
    tail -5 "$CONSOLE_LOG" 2>/dev/null | sed "s/^/[$CONSOLE_TAG]   /" >&2 || true
    return 0
  fi
  echo "[$CONSOLE_TAG] reach console: http://localhost:$CONSOLE_PORT  (log: $CONSOLE_LOG)"
  if [ "$HEADLESS" = "0" ] && command -v xdg-open >/dev/null 2>&1; then
    # Open the tab when the simulator starts publishing, not now: Isaac takes minutes to load, and a page
    # opened before that just says it is waiting. Gives up after 10 minutes and leaves the console running.
    (
      for _ in $(seq 600); do
        if curl -s --max-time 1 "http://127.0.0.1:$FEED_PORT/health" 2>/dev/null | grep -q '"sim": *true'; then
          xdg-open "http://localhost:$CONSOLE_PORT" >/dev/null 2>&1 || true
          exit 0
        fi
        sleep 1
      done
    ) &
    CONSOLE_WAITER=$!
  fi
}
