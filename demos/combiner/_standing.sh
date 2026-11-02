# Sourced by the standing combiner launchers (demos/combiner/run_*.sh), not run on its own.
#
# Activates env_isaaclab without ROS, then `standing_demo ARGS...` runs demos/unifp/run_demo.py on the combiner box
# with one environment in the viewer and --loop: each attempt holds its end pose until R, which starts the next
# placement (pressed mid-attempt, it restarts). Closing the window ends the run; run.json, summary.json and
# trace.csv are in logs/unifp_demos/, saved after every attempt. Record it: ./dashboard.py record <dir> --title "...".
# The GPU is shared: check nvidia-smi before launching.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."

find_conda_base() {
  if [ -n "${CONDA_EXE:-}" ] && [ -x "$CONDA_EXE" ]; then
    dirname "$(dirname "$CONDA_EXE")"
    return
  fi
  for candidate in "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda; do
    if [ -x "$candidate/bin/conda" ]; then
      printf '%s\n' "$candidate"
      return
    fi
  done
  if command -v conda >/dev/null 2>&1; then
    dirname "$(dirname "$(command -v conda)")"
    return
  fi
  return 1
}

if ! CONDA_BASE="$(find_conda_base)"; then
  echo "[combiner] Conda is unavailable; set CONDA_EXE or install the Isaac environment." >&2
  exit 127
fi
# Conda's activation hooks are not required to be nounset-safe.
set +u
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate "${ISAAC_SIM_CONDA_ENV:-env_isaaclab}"
set -u
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export PYTHONUNBUFFERED=1

standing_demo() {
  exec python demos/unifp/run_demo.py --task combiner --num_envs 1 --attempts 1 --loop "$@"
}
