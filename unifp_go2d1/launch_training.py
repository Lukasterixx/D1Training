"""Launcher for the Go2+D1 UniFP training run.

Same training as `legged_gym/scripts/train_go2d1posforce.py`; this only sets the knobs that
buy wall-clock time, and prints what it set so a run's log says which were on.

  --tf32        TF32 matmuls on the Ada tensor cores. The PPO update is ~45% of iteration
                time here (a 32x76 stacked observation into a 512-256-128 MLP), and TF32
                roughly halves it. This is a REDUCED-PRECISION arithmetic change, not a
                free one: results stay statistically equivalent but are not bit-identical,
                so a TF32 run and an FP32 run of the same seed diverge. Say which was used.
  --no-wandb    Sets WANDB_MODE=disabled. UniFP's runner calls wandb.init unconditionally;
                TensorBoard logging is unaffected. Purely an overhead and login change.

Everything that would change the learned policy -- num_envs, iterations, curriculum, rewards,
network -- stays on the command line and in the config, where it is visible.
"""

import argparse
import os
import sys

UNIFP = os.environ.get("UNIFP_ROOT", os.path.expanduser("~/thesis_b_legacy/UniFP"))
sys.path.insert(0, UNIFP)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def main():
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--tf32", action="store_true")
    pre.add_argument("--no-wandb", action="store_true")
    pre.add_argument("--force-start-step", type=int, default=None,
                     help="Override commands.force_start_step (iterations of position-only "
                          "training before any external force). CHANGES THE EXPERIMENT -- "
                          "upstream's value is 8000. Record it if you use it.")
    known, rest = pre.parse_known_args()

    if known.no_wandb:
        os.environ["WANDB_MODE"] = "disabled"
        print("[launch] WANDB_MODE=disabled")

    import isaacgym  # noqa: F401  (must precede torch)
    import torch

    if known.tf32:
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
        print("[launch] TF32 matmuls enabled (reduced precision; not bit-identical to FP32)")

    sys.argv = [sys.argv[0]] + rest
    import legged_gym.envs  # noqa: F401  (importing registers the tasks)
    from legged_gym.utils import get_args, task_registry

    args = get_args()
    args.headless = True
    print(f"[launch] task={args.task} num_envs={args.num_envs} max_iterations={args.max_iterations} "
          f"seed={args.seed} resume={args.resume}")

    import run_metadata  # noqa: E402  (same folder)

    env_cfg, train_cfg = task_registry.get_cfgs(name=args.task)
    if args.flat_terrain:
        env_cfg.terrain.height = [0.0, 0.0]
    if known.force_start_step is not None:
        print(f"[launch] DEVIATION: force_start_step "
              f"{env_cfg.commands.force_start_step} -> {known.force_start_step}")
        env_cfg.commands.force_start_step = known.force_start_step
    env, env_cfg = task_registry.make_env(name=args.task, args=args, env_cfg=env_cfg)
    runner, train_cfg = task_registry.make_alg_runner(env=env, name=args.task, args=args)
    print(f"[launch] log_dir={runner.log_dir}")

    # --- resume corrections -------------------------------------------------------------
    # `make_alg_runner` has already loaded the checkpoint if --resume was given, so
    # runner.current_learning_iteration is where training will pick up.
    resumed_from = runner.current_learning_iteration
    if resumed_from:
        # 1. The force curriculum is gated on env.global_steps, which _init_buffers zeroes on
        #    every launch, while the gate compares against force_start_step * num_steps_per_env.
        #    Left alone, a resume at iteration 20,000 would drop back to position-only training
        #    for another 8,000 iterations without saying so. global_steps counts one per policy
        #    step and each iteration takes num_steps_per_env of them, so this is exact.
        steps_per_iter = train_cfg.runner.num_steps_per_env
        env.global_steps = resumed_from * steps_per_iter
        forces_at = env_cfg.commands.force_start_step
        print(f"[launch] resumed at iteration {resumed_from}; global_steps set to "
              f"{env.global_steps} ({steps_per_iter}/iteration). External forces start at "
              f"iteration {forces_at}: {'ON' if resumed_from >= forces_at else 'not yet'}")
        # 2. learn() runs current_learning_iteration + num_learning_iterations, so
        #    --max_iterations is an amount to ADD, not a target. Say what will happen, loudly,
        #    because the two readings differ by however far the run had already got.
        print(f"[launch] will train {train_cfg.runner.max_iterations} more iterations, "
              f"ending at {resumed_from + train_cfg.runner.max_iterations}")

    # The experimental record needs more than TensorBoard events; see run_metadata.py.
    urdf = os.path.join(UNIFP, env_cfg.asset.file)
    meta = dict(mode="train", seed=train_cfg.seed, num_envs=env.num_envs,
                iterations=resumed_from + train_cfg.runner.max_iterations, urdf=urdf,
                command=" ".join(sys.argv),
                extra={"tf32": bool(known.tf32), "task": args.task,
                       "resumed_from_iteration": resumed_from,
                       "force_start_step": env_cfg.commands.force_start_step,
                       "resume_path": getattr(train_cfg.runner, "load_run", None) if resumed_from else None})
    run_metadata.write(runner.log_dir, status="running", **meta)
    try:
        runner.learn(num_learning_iterations=train_cfg.runner.max_iterations,
                     init_at_random_ep_len=True)
    except BaseException as exc:                      # KeyboardInterrupt counts as evidence too
        run_metadata.write(runner.log_dir, status="aborted", error=f"{type(exc).__name__}: {exc}", **meta)
        raise
    run_metadata.write(runner.log_dir, status="finished", **meta)


if __name__ == "__main__":
    main()
