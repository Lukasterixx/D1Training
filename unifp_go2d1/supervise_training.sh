#!/usr/bin/env bash
# Keep the Go2+D1 UniFP run going to TARGET iterations without a human present.
#
#   ./unifp_go2d1/supervise_training.sh --attach <pid>   # watch the run already going
#   ./unifp_go2d1/supervise_training.sh                  # start one if none is running
#
# It watches the training process, and when it is gone and the target is not reached it resumes
# from the newest checkpoint across every run directory of the experiment, asking for exactly the
# iterations still owed. That last part matters: UniFP's `learn(num_learning_iterations=N)` adds
# N to wherever it resumed, so passing the target would overshoot by however far it had got.
#
# It judges a restart by PROGRESS, not by liveness. A run that exits quickly having written a
# newer checkpoint has finished its allotted iterations, which is success; a run that exits
# without moving the checkpoint forward is a failure. Getting that backwards makes the supervisor
# fall down its own checkpoint list after a perfectly good short run -- which is what the first
# version of this script did, on its first test.
#
# Failure handling is deliberately dumb: fall back one checkpoint (in case the newest was being
# written when the process died), and after MAX_FAILS consecutive failures stop and say so rather
# than burn the GPU in a crash loop. It never edits the config or tries a different configuration.
#
# To stop deliberately, use stop_training.sh -- the STOP file is how this tells "the user stopped
# it" from "it died".
set -uo pipefail        # not -e: surviving failures is the whole job

TARGET=${TARGET:-60000}
EXP=${EXP:-go2d1_pos_force}
NUM_ENVS=${NUM_ENVS:-4096}
CHECK_S=${CHECK_S:-60}          # how often to check on a running job
SETTLE_S=${SETTLE_S:-300}       # how long to give a restart before judging it
STALL_S=${STALL_S:-3600}        # alive but no new checkpoint this long => hung, restart it
MAX_FAILS=${MAX_FAILS:-5}

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIFP="${UNIFP_ROOT:-$HOME/thesis_b_legacy/UniFP}"
LOGROOT="$UNIFP/logs/$EXP"
STOP="$UNIFP/logs/STOP"
SUP="$UNIFP/logs/supervisor.log"

PID=""
[ "${1:-}" = "--attach" ] && PID="${2:-}"

log() { echo "[$(date '+%F %T')] $*" >> "$SUP"; }

# Newest checkpoint iteration across every run directory, or -1 if there is none.
newest_iter() {
  local out; out="$(python3 "$HERE/find_checkpoint.py" "$LOGROOT" --skip 0)"
  [ -n "$out" ] && echo "${out##* }" || echo -1
}

source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate unifp
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
cd "$UNIFP"

log "supervisor up: target ${TARGET} iterations, experiment ${EXP}, ${NUM_ENVS} envs, attached pid '${PID:-none}'"
fails=0
skip=0
stall_ref=$(newest_iter)
stall_since=$(date +%s)

while true; do
  if [ -f "$STOP" ]; then
    log "STOP file present -- stopped on purpose. Supervisor exiting."
    exit 0
  fi

  NEWEST=$(newest_iter)

  # Finished? Ask the checkpoints, never the process. Checked first and always at skip=0, so a
  # fallback can never hide a newer checkpoint from this test.
  if [ "$NEWEST" -ge "$TARGET" ]; then
    log "COMPLETE: newest checkpoint is iteration ${NEWEST} of ${TARGET}. Supervisor exiting."
    exit 0
  fi

  if [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null; then
    # Alive. Make sure it is also moving: a hung process holds the GPU and writes nothing.
    if [ "$NEWEST" -gt "$stall_ref" ]; then
      stall_ref=$NEWEST; stall_since=$(date +%s)
    elif [ $(( $(date +%s) - stall_since )) -ge "$STALL_S" ]; then
      log "STALLED: pid ${PID} alive but no new checkpoint past ${stall_ref} for ${STALL_S}s. Killing it to force a resume."
      kill "$PID" 2>/dev/null; sleep 30; kill -9 "$PID" 2>/dev/null
      PID=""; stall_since=$(date +%s)
      continue
    fi
    sleep "$CHECK_S"
    continue
  fi

  read -r RUN ITER <<< "$(python3 "$HERE/find_checkpoint.py" "$LOGROOT" --skip "$skip")"
  if [ -z "${ITER:-}" ]; then
    log "FATAL: no usable checkpoint in ${LOGROOT} at skip=${skip}. Nothing to resume from."
    exit 1
  fi

  REMAIN=$((TARGET - ITER))
  log "training is not running. Resuming from ${RUN}/model_${ITER}.pt, ${REMAIN} iterations owed (skip=${skip})."
  nohup python "$HERE/launch_training.py" \
      --tf32 --no-wandb --task=go2d1_pos_force --experiment_name "$EXP" \
      --num_envs "$NUM_ENVS" --headless \
      --resume --load_run "$RUN" --checkpoint "$ITER" --max_iterations "$REMAIN" \
      >> "$UNIFP/logs/train_resumed_$(date +%Y%m%d_%H%M%S).log" 2>&1 < /dev/null &
  PID=$!
  log "  relaunched as pid ${PID}; judging it in ${SETTLE_S}s"
  sleep "$SETTLE_S"

  AFTER=$(newest_iter)
  if kill -0 "$PID" 2>/dev/null; then
    log "  restart healthy: still running (pid ${PID}, newest checkpoint ${AFTER})"
    fails=0; skip=0; stall_ref=$AFTER; stall_since=$(date +%s)
  elif [ "$AFTER" -gt "$NEWEST" ]; then
    log "  restart exited having made progress (${NEWEST} -> ${AFTER}); that is success, not a failure"
    fails=0; skip=0; stall_ref=$AFTER; stall_since=$(date +%s)
    PID=""
  else
    fails=$((fails + 1)); skip=$((skip + 1))
    log "  restart exited inside ${SETTLE_S}s with no progress (still ${AFTER}); consecutive failure ${fails}/${MAX_FAILS}, falling back one checkpoint"
    PID=""
    if [ "$fails" -ge "$MAX_FAILS" ]; then
      log "FATAL: ${MAX_FAILS} restarts failed in a row with no progress. Not restarting again -- this needs a human."
      exit 1
    fi
  fi
done
