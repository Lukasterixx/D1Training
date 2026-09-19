#!/usr/bin/env bash
# Show the training progress readout.
#
#   ./unifp_go2d1/watch_progress.sh          # in a new desktop terminal window if there is a
#                                            # display, otherwise right here (works over SSH)
#   ./unifp_go2d1/watch_progress.sh --here   # always in this terminal
#   ./unifp_go2d1/watch_progress.sh --once   # print once and exit
#
# The desktop window is launched with a scrubbed environment on purpose: a shell with ROS or a
# conda env sourced puts snap and ROS libraries on LD_LIBRARY_PATH, and gnome-terminal then dies
# on a symbol lookup in /snap/core20 rather than opening.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INTERVAL="${INTERVAL:-30}"

case "${1:-}" in
  --once) exec python3 "$HERE/progress.py" ;;
  --here) exec python3 "$HERE/progress.py" --watch "$INTERVAL" ;;
esac

if [ -n "${DISPLAY:-}" ] || [ -e /tmp/.X11-unix/X0 ]; then
  if command -v gnome-terminal >/dev/null; then
    setsid env -i DISPLAY="${DISPLAY:-:0}" HOME="$HOME" USER="${USER:-$(id -un)}" \
      PATH=/usr/local/bin:/usr/bin:/bin \
      DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u)/bus" \
      XDG_RUNTIME_DIR="/run/user/$(id -u)" \
      gnome-terminal --title="Go2+D1 UniFP training" --geometry=80x26 -- \
      bash -c "python3 '$HERE/progress.py' --watch $INTERVAL; exec bash" >/dev/null 2>&1 &
    sleep 3
    if pgrep -f "progress.py --watch" >/dev/null; then
      echo "Opened a terminal window on the desktop (title: Go2+D1 UniFP training)."
      exit 0
    fi
    echo "Could not open a desktop terminal; falling back to this one." >&2
  fi
fi
exec python3 "$HERE/progress.py" --watch "$INTERVAL"
