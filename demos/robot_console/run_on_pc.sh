#!/usr/bin/env bash
# The test console from the PC: sends this working tree to the dog, starts the server there on 127.0.0.1, tunnels it to
# http://localhost:8097 over the key-checked route (so it is not open on the WiFi), and opens the page.
#
#   demos/robot_console/run_on_pc.sh              # add --no_update to skip d1update, --no_browser not to open a page
#
# Closing this terminal or losing the link E-stops a live run. Ctrl-C here stops the console. The d1* helpers are
# ~/.bashrc functions, hence the interactive shells.
set -uo pipefail

UPDATE=1 BROWSER=1
for a in "$@"; do
  case "$a" in
    --no_update) UPDATE=0 ;;
    --no_browser) BROWSER=0 ;;
    *) echo "unknown argument $a" >&2; exit 2 ;;
  esac
done

if [ "$UPDATE" = 1 ]; then
  bash -ic d1update 2>&1 | grep -E 'd1update:|d1: |rror' || true
fi
target="$(bash -ic _d1_dog 2>/dev/null | tail -1)"
case "$target" in
  *@*) ;;
  *) echo "the D1 dog is not on this PC's WiFi" >&2; exit 1 ;;
esac
mapfile -t opts < <(bash -ic '_p2_robot_ssh_opts dog-wifi' 2>/dev/null)
[ "$BROWSER" = 1 ] && ( sleep 2; xdg-open http://localhost:8097 >/dev/null 2>&1 ) &
# Its own connection, ahead of the options that share one (ssh keeps the first value): a shared connection would outlive
# this terminal and hide a lost link.
exec ssh -o ControlMaster=no -o ControlPath=none "${opts[@]}" -tt -o ServerAliveInterval=2 -o ServerAliveCountMax=3 -o ExitOnForwardFailure=yes \
  -L 8097:127.0.0.1:8097 "$target" "cd D1Training && exec python3 demos/robot_console/server.py"
