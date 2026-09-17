#!/usr/bin/env bash
# Launch the scripted cup pick (Go2 lying down, wrist RealSense, stock YOLO) in Isaac Sim.
#
# Same environment setup as run_sim.sh, minus the ROS 2 bridge: the pick publishes nothing. Do NOT source
# /opt/ros/humble/setup.bash first -- system ROS's Python 3.10 on PYTHONPATH breaks Isaac Sim's 3.11.
#
# The reach console comes up beside the simulator: d1_ui/server.py in sim mode, following this run's feed
# (joints, legs, and the wrist camera with YOLO's boxes) on http://localhost:8090, and stopped when this
# script exits. A viewer run opens a browser tab once the simulator is publishing; a --headless one does not.
# --no_console leaves it out (as does D1_UI_CONSOLE=0), and ./run_ui.sh still runs one by hand.
#
# Any arguments are forwarded to run_pick_demo.py, e.g.:
#   ./run_pick_demo.sh                           # viewer, real time; R resets with the cup somewhere new
#   ./run_pick_demo.sh --seed 7                  # a different sequence of cup positions for R
#   ./run_pick_demo.sh --cup_xy 0.38 -0.08       # first cup somewhere specific
#   ./run_pick_demo.sh --headless --episodes 10  # ten picks back to back, results in logs/pick_demo/
#   ./run_pick_demo.sh --no_console              # simulator only, no console (a shell flag, not the demo's)
#   ./run_pick_demo.sh --help                    # every option
#
# Needs ultralytics in env_isaaclab, installed without its dependencies (see README). The cup model is in the
# repository (pick_demo/assets/, CC BY 4.0, credited in third_party/sketchfab_cup/NOTICE.md); --cup_usdz swaps it.
set -e

cd "$(dirname "$0")"

CONDA_ENV_NAME="${ISAAC_SIM_CONDA_ENV:-env_isaaclab}"

# --no_console is this script's own; everything else goes to run_pick_demo.py untouched. The feed port and
# --headless are read (not consumed) because the console needs the one and the browser tab depends on the other.
CONSOLE="${D1_UI_CONSOLE:-1}"
CONSOLE_PORT="${D1_UI_PORT:-8090}"
CONSOLE_LOG="${D1_UI_LOG:-/tmp/d1_ui_sim.log}"
FEED_PORT=8765
HEADLESS=0
ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --no_console) CONSOLE=0 ;;
    -h|--help) CONSOLE=0; ARGS+=("$1") ;;
    --headless) HEADLESS=1; ARGS+=("$1") ;;
    --ui_feed_port) ARGS+=("$1"); if [ $# -gt 1 ]; then FEED_PORT="$2"; ARGS+=("$2"); shift; fi ;;
    --ui_feed_port=*) FEED_PORT="${1#*=}"; ARGS+=("$1") ;;
    *) ARGS+=("$1") ;;
  esac
  shift
done
set -- ${ARGS[@]+"${ARGS[@]}"}

find_conda_base() {
  if [ -n "${CONDA_EXE:-}" ] && [ -x "$CONDA_EXE" ]; then
    dirname "$(dirname "$CONDA_EXE")"
    return 0
  fi
  for candidate in "$HOME/miniconda3" "$HOME/anaconda3" "/opt/conda"; do
    if [ -x "$candidate/bin/conda" ]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  if command -v conda >/dev/null 2>&1; then
    dirname "$(dirname "$(command -v conda)")"
    return 0
  fi
  return 1
}

if ! CONDA_BASE="$(find_conda_base)"; then
  echo "[run_pick_demo] Conda is not available. Install Miniconda or set CONDA_EXE/ISAAC_SIM_CONDA_ENV." >&2
  exit 127
fi

if [ -f "$CONDA_BASE/etc/profile.d/conda.sh" ]; then
  # shellcheck disable=SC1090
  . "$CONDA_BASE/etc/profile.d/conda.sh"
else
  export PATH="$CONDA_BASE/bin:$PATH"
  eval "$("$CONDA_BASE/bin/conda" shell.bash hook)"
fi
conda activate "$CONDA_ENV_NAME"

# Keep system ROS's Python 3.10 out of Isaac Sim's 3.11.
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH

# The GPU is shared with other Isaac Lab training. Say so rather than silently competing with it.
if command -v nvidia-smi >/dev/null 2>&1; then
  BUSY="$(nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv,noheader 2>/dev/null || true)"
  if [ -n "$BUSY" ]; then
    echo "[run_pick_demo] Note: the GPU is already running:" >&2
    echo "$BUSY" | sed 's/^/[run_pick_demo]   /' >&2
  fi
fi

# Isaac's logging drowns stdout; unbuffered, the [pick] lines appear as they happen.
export PYTHONUNBUFFERED=1

# The reach console, beside the simulator. It is started first and left waiting: `--mode sim` polls the feed
# rather than giving up when none answers yet, so it is up by the time Isaac finishes loading. Its own output
# would bury the [pick] lines, so it goes to a log. The console watches; it cannot command anything in sim mode.
CONSOLE_PID=""
CONSOLE_WAITER=""
stop_console() {
  [ -n "$CONSOLE_WAITER" ] && kill "$CONSOLE_WAITER" 2>/dev/null || true
  if [ -n "$CONSOLE_PID" ] && kill -0 "$CONSOLE_PID" 2>/dev/null; then
    kill "$CONSOLE_PID" 2>/dev/null || true
    wait "$CONSOLE_PID" 2>/dev/null || true
    echo "[run_pick_demo] reach console stopped."
  fi
}
trap stop_console EXIT
trap 'stop_console; exit 130' INT TERM

if [ "$CONSOLE" = "1" ] && [ "$FEED_PORT" != "0" ]; then
  if curl -s --max-time 1 "http://127.0.0.1:$CONSOLE_PORT/state" >/dev/null 2>&1; then
    echo "[run_pick_demo] a console is already serving http://localhost:$CONSOLE_PORT; leaving that one alone." >&2
  else
    # D1_UI_ARGS reaches server.py as it stands, for the console's own options: the detector on the CPU when
    # the viewer wants the VRAM (D1_UI_ARGS='--yolo-device cpu'), every COCO class (--detect all), no legs.
    # shellcheck disable=SC2086
    python d1_ui/server.py --mode sim --sim-url "http://127.0.0.1:$FEED_PORT" --port "$CONSOLE_PORT" \
      ${D1_UI_ARGS:-} > "$CONSOLE_LOG" 2>&1 &
    CONSOLE_PID=$!
    # Give it a moment to bind, and say so plainly if it died instead: the pick itself does not need it.
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      curl -s --max-time 1 "http://127.0.0.1:$CONSOLE_PORT/state" >/dev/null 2>&1 && break
      kill -0 "$CONSOLE_PID" 2>/dev/null || break
      sleep 1
    done
    if kill -0 "$CONSOLE_PID" 2>/dev/null; then
      echo "[run_pick_demo] reach console: http://localhost:$CONSOLE_PORT  (log: $CONSOLE_LOG)"
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
    else
      CONSOLE_PID=""
      echo "[run_pick_demo] the reach console did not start; the pick runs without it. Last lines of $CONSOLE_LOG:" >&2
      tail -5 "$CONSOLE_LOG" 2>/dev/null | sed 's/^/[run_pick_demo]   /' >&2
    fi
  fi
fi

python run_pick_demo.py "$@"
