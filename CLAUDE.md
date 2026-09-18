# D1Training

Go2 + welded D1 arm in Isaac Lab; Thesis B whole-body control work (position-only baseline, then
force-aware and tool-conditioned policies). Start from [docs/thesis_b_plan.md](docs/thesis_b_plan.md).

## The experimental record is part of every task

`results/` is Lukas's Thesis B evidence, viewed with `./dashboard.py`. Keep it current as you work,
in the same session as the work, not as a follow-up. Layout and conventions: [results/README.md](results/README.md).

- **Which week:** `results/config.json` `term_start` is Week 1's Monday (2026-09-14). Check today's date.
- **Every run you launch** (smoke, train, eval, playback), including failed or aborted ones:
  `./dashboard.py record <run_dir> --title "<short description>"`. Then add a dated entry under `## Log`
  in that week's `notes.md` with the exact command, the outcome in numbers, and what it does and does
  not show. Link the run as `[title](#/week/N/run/<run id>)`.
- **Conclusions** go in `results/findings.md` as the next `F-NNN`, with Status/Week/Date/Evidence/Scope/Implication.
  Never delete or silently rewrite a finding; mark it `superseded` or `retracted` and link the replacement.
- **Gate changes** go in `results/gates.md`. **Checklist items** in `notes.md` get ticked when done, with evidence.
- Update the week's `**Status:**` line (`not started` / `in progress` / `complete`).
- **Screenshots** in `results/week_NN/screenshots/` are added by Lukas. When new ones appear, look at them
  and describe what they show in the notes. Plots you generate go in `figures/`.
- **Scope honesty:** interface smoke ≠ reaching ability; training-time `Metrics/...` sampled at reset ≠ the
  frozen-manifest evaluation that G1a needs; the CAD pincer tip (the controlled point) ≠ a point measured on the arm. Say "not validated" where true.
- If a doc in `docs/` states something a new result contradicts, fix the doc and note it in the week log.

## Running things

- Isaac scripts: `source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab`, then
  `unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH`. Never source ROS into that shell.
- **The GPU is shared** with other Isaac Lab training (e.g. `~/IsaacLab` Rescue runs). Check `nvidia-smi`
  before launching a simulator; don't start one alongside another job without asking Lukas.
- Tests: `python -m unittest discover -s . -p "test_*.py"` runs both roots -- `tests/` for the robot and
  `demos/cup/tests/` for the cup demo (the Isaac env runs all of them; `tests/test_evidence.py`
  also runs on the system Python).
- `logs/` is gitignored; the recorded copy in `results/` is what gets committed.
- Code adapted from other repositories keeps its licence and a provenance row in `third_party/<name>/NOTICE.md`.
- Playback (`run_sim.sh`) must keep `leg_actuator="dc_motor"`: the walking checkpoint was trained against it.
- Arm drive type matters (F-010): `--arm_actuator d1_servo` uses force drives; playback and `implicit` keep the URDF
  import's acceleration drives. Say which when comparing arm behaviour across them.
- `python run_position_only.py verify --headless --num_envs 8` re-runs the deliberate G0 checks after any change
  to the task, robot config or weld.
- Motor and timing values in `motor_model.py` carry source labels (published, URDF-unverified, estimated). Keep the labels accurate; change a label only when a measurement replaces the value.
