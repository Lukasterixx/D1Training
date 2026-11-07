#!/usr/bin/env bash
# The test console from the PC: sends this working tree to the dog, starts the server there on 127.0.0.1, tunnels it to
# http://localhost:8097 over the key-checked route (so it is not open on the WiFi), and opens the page.
#
#   demos/robot_console/run_on_pc.sh              # add --no_update to skip d1update, --no_browser not to open a page
#   demos/robot_console/run_on_pc.sh --dog unitree@192.168.1.213 --preview    # a PC without the helpers: what would change
#   demos/robot_console/run_on_pc.sh --dog unitree@192.168.1.213              # ... then the same as above
#
# Closing this terminal or losing the link E-stops a live run. Ctrl-C here stops the console. The d1* helpers are
# ~/.bashrc functions, hence the interactive shells. --dog goes without them: this script does `d1update`'s copy itself and
# takes the address as given, so it does not look for the dog. There are two dogs, and the host key is what tells them
# apart: it has to be in known_hosts already, and a new one is refused here, never accepted.
set -uo pipefail

UPDATE=1 BROWSER=1 DOG="" PREVIEW=0
while [ $# -gt 0 ]; do
  case "$1" in
    --no_update) UPDATE=0 ;;
    --no_browser) BROWSER=0 ;;
    --preview) PREVIEW=1 ;;
    --dog)
      DOG="${2:-}"
      case "$DOG" in
        *@*) shift ;;
        *) echo "--dog takes user@address" >&2; exit 2 ;;
      esac
      ;;
    *) echo "unknown argument $1" >&2; exit 2 ;;
  esac
  shift
done
if [ "$PREVIEW" = 1 ] && [ -z "$DOG" ]; then
  echo "--preview goes with --dog" >&2
  exit 2
fi

if [ -n "$DOG" ]; then
  target="$DOG"
  opts=(-o StrictHostKeyChecking=yes)
  # `d1update`'s filters. --delete replaces the dog's tree with this one; what is excluded here stays on the dog.
  # --checksum: the two PCs' copies of a file differ in time, so only the content says what changed.
  copy=(rsync -a --checksum --delete -e "ssh ${opts[*]}"
    --exclude /.git/ --exclude /.claude/ --exclude /results/ --exclude /logs/ --exclude /generated/
    --include /checkpoints/ --include '/checkpoints/*.npz' --exclude '/checkpoints/*'
    --exclude __pycache__/ --exclude '*.py[cod]' --exclude .vscode/ --exclude graphify-out/ --exclude .pytest_cache/)
  cd "$(dirname "${BASH_SOURCE[0]}")/../.." || exit 1
  if [ "$PREVIEW" = 1 ]; then
    exec "${copy[@]}" --dry-run --itemize-changes ./ "$target:D1Training/"
  fi
  if [ "$UPDATE" = 1 ]; then
    "${copy[@]}" ./ "$target:D1Training/" || { echo "the copy to $target failed: no console started" >&2; exit 1; }
  fi
else
  if [ "$UPDATE" = 1 ]; then
    bash -ic d1update 2>&1 | grep -E 'd1update:|d1: |rror' || true
  fi
  target="$(bash -ic _d1_dog 2>/dev/null | tail -1)"
  case "$target" in
    *@*) ;;
    *) echo "the D1 dog is not on this PC's WiFi" >&2; exit 1 ;;
  esac
  mapfile -t opts < <(bash -ic '_p2_robot_ssh_opts dog-wifi' 2>/dev/null)
fi
[ "$BROWSER" = 1 ] && ( sleep 2; xdg-open http://localhost:8097 >/dev/null 2>&1 ) &
# Its own connection, ahead of the options that share one (ssh keeps the first value): a shared connection would outlive
# this terminal and hide a lost link.
exec ssh -o ControlMaster=no -o ControlPath=none "${opts[@]}" -tt -o ServerAliveInterval=2 -o ServerAliveCountMax=3 -o ExitOnForwardFailure=yes \
  -L 8097:127.0.0.1:8097 "$target" "cd D1Training && exec python3 demos/robot_console/server.py"
