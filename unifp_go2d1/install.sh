#!/usr/bin/env bash
# Install the Go2+D1 port into a UniFP checkout. Idempotent: re-run after editing the port.
#
#   ./unifp_go2d1/install.sh [<UniFP checkout>]     (default ~/thesis_b_legacy/UniFP)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIFP="${1:-$HOME/thesis_b_legacy/UniFP}"

[ -f "$UNIFP/legged_gym/envs/b2/b2z1_pos_force_config.py" ] || {
  echo "not a UniFP checkout: $UNIFP" >&2; exit 1; }

echo "== asset =="
python3 "$HERE/build_asset.py" "$UNIFP/resources/robots/go2d1"

echo "== environment =="
mkdir -p "$UNIFP/legged_gym/envs/go2d1"
touch "$UNIFP/legged_gym/envs/go2d1/__init__.py"
cp "$HERE/go2d1_pos_force_config.py" "$UNIFP/legged_gym/envs/go2d1/"
python3 "$HERE/port_env.py" "$UNIFP"

echo "== task registration =="
REG="$UNIFP/legged_gym/envs/__init__.py"
if ! grep -q "go2d1_pos_force" "$REG"; then
  cat >> "$REG" <<'PYEOF'


# --- Go2 + D1 port (D1Training/unifp_go2d1) ---
from legged_gym.envs.go2d1.go2d1_pos_force_config import Go2D1PosForceRoughCfg, Go2D1PosForceRoughCfgPPO
from .go2d1.legged_robot_go2d1_pos_force import LeggedRobot_go2d1_pos_force

task_registry.register("go2d1_pos_force", LeggedRobot_go2d1_pos_force,
                       Go2D1PosForceRoughCfg(), Go2D1PosForceRoughCfgPPO())
PYEOF
  echo "  registered go2d1_pos_force"
else
  echo "  already registered"
fi

echo "== scripts =="
for s in train play; do
  sed -e "s/b2z1_pos_force/go2d1_pos_force/g" \
      "$UNIFP/legged_gym/scripts/${s}_b2z1posforce.py" \
      > "$UNIFP/legged_gym/scripts/${s}_go2d1posforce.py"
done
echo "  wrote train_go2d1posforce.py, play_go2d1posforce.py"

echo
echo "done. Train with:"
echo "  cd $UNIFP && python legged_gym/scripts/train_go2d1posforce.py --task=go2d1_pos_force --headless"
