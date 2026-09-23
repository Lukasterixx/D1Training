#!/usr/bin/env python3
"""Train UniFP's position/force task natively in Isaac Lab.

`run_unifp_isaaclab.py` runs a checkpoint trained on the legacy Isaac Gym stack. This trains one
here instead, which is what F-088 argues for: the Isaac Gym policy holds a stance on the Isaac Lab
model but falls within half a second when told to walk, so moving trained weights between the two
simulators is not a reliable step.

    ./run_unifp_train.py smoke --num_envs 16 --headless            # build the env and step it
    ./run_unifp_train.py play  --num_envs 16 --headless            # ... driven by model_48800
    ./run_unifp_train.py train --num_envs 4096 --iterations 60000 --headless

`smoke` and `play` step the environment directly and report shapes and reward scale. `train`
builds the rsl-rl runner around it: `unifp_train.models.UniFPActor` for the adaptation-module
actor, `unifp_train.algorithm.UniFPPPO` for PPO plus the estimator loss, and
`unifp_train.agent.make_agent_cfg` for upstream's hyperparameters.

Every run writes a `run.json` next to its checkpoints, recording the stack versions, the welded
USD's hash, the git commit and the agent config, so `./dashboard.py record` has something to read.

**The GPU is shared.** Check `nvidia-smi` before starting a long run.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=("smoke", "play", "train", "manifest", "eval"),
                        help="'play' drives the new environment with an existing UniFP checkpoint, "
                             "which checks the whole loop -- observation, action mapping, reward -- "
                             "against a policy already known to work on this robot. 'train' runs "
                             "PPO with the adaptation module. 'manifest' freezes an "
                             "evaluation set; 'eval' scores a policy against one.")
    parser.add_argument("--checkpoint", default=os.path.expanduser(
        "~/thesis_b_legacy/UniFP/logs/go2d1_pos_force/Sep18_11-10-15_/model_48800.pt"),
        help="UniFP checkpoint for 'play'.")
    parser.add_argument("--num_envs", type=int, default=16)
    parser.add_argument("--steps", type=int, default=100,
                        help="Policy steps at 50 Hz, for 'smoke' and 'play'.")
    parser.add_argument("--iterations", type=int, default=60000,
                        help="Policy updates, for 'train'. Upstream's config asks for 60000.")
    parser.add_argument("--seed", type=int, default=1, help="UniFP's config seed is 1.")
    parser.add_argument("--robot_usd", help="Existing welded USD; otherwise it is rebuilt.")
    parser.add_argument("--output", default=str(ROOT / "logs/unifp_train"),
                        help="Where 'train' writes its run directory.")
    parser.add_argument("--run_name", default="", help="Appended to the run directory's name.")
    parser.add_argument("--force_start_iteration", type=int, default=None,
                        help="Override the force curriculum. UniFP's default is 8000 iterations "
                             "of position-only training first; 0 applies forces immediately.")
    parser.add_argument("--resume_from", default=None,
                        help="Checkpoint to resume 'train' from, written by this script.")
    parser.add_argument("--manifest", default=None,
                        help="Frozen evaluation set, for 'eval'.")
    parser.add_argument("--manifest_out", default=None,
                        help="Where 'manifest' writes. Defaults under results/manifests/.")
    parser.add_argument("--role", default="validation", choices=("development", "validation", "test"),
                        help="Which frozen set 'manifest' is building.")
    parser.add_argument("--hold_command", nargs=3, type=float, metavar=("VX", "VY", "WZ"),
                        default=None,
                        help="For 'eval': hold one base velocity command for the whole episode "
                             "instead of sampling it. This deliberately overrides the frozen "
                             "schedule, so the digests will not match and the run is a "
                             "diagnostic rather than a manifest result.")
    parser.add_argument("--zero_actions", action="store_true",
                        help="Evaluate the do-nothing baseline instead of a checkpoint.")
    # Knobs for diagnosing the training instability of 2026-09-21. Each defaults to UniFP's own
    # value, so leaving them alone keeps the faithful port; naming one is a deliberate departure
    # and is recorded in the run's agent.json.
    parser.add_argument("--std_type", choices=("scalar", "log"), default=None,
                        help="Action-noise parameterisation. UniFP uses 'scalar'; 'log' makes the "
                             "updates multiplicative, so the noise cannot be driven to a value "
                             "where the KL divergence blows up.")
    parser.add_argument("--init_std", type=float, default=None,
                        help="Initial action noise. UniFP's is 1.0.")
    parser.add_argument("--estimator_learning_rate", type=float, default=None,
                        help="Rate for the adaptation module's own optimizer, which shares the "
                             "encoder with the actor. UniFP's is 1e-5 and fixed, so as PPO's "
                             "adaptive rate falls the estimator's share of the encoder rises. "
                             "0 disables it, which is a diagnosis rather than a configuration.")
    parser.add_argument("--diagnose_storage", action="store_true",
                        help="Before each update, check that the policy the update sees is the "
                             "one that produced the rollout, and print where it is not. The "
                             "cheap standing version of this check is `Loss/kl_first_minibatch`, "
                             "logged on every run; use this when that is not ~0.")
    parser.add_argument("--fixed_learning_rate", type=float, default=None,
                        help="Hold PPO's learning rate at this value, ignoring the adaptive "
                             "schedule. 0 freezes the policy entirely, which is what makes it a "
                             "null test: any KL it still reports is not produced by the "
                             "optimizer. A departure from UniFP; recorded as one.")
    parser.add_argument("--learning_rate_scale", type=float, default=None,
                        help="Multiply the optimizer's effective step by this. rsl-rl clamps the "
                             "adaptive rate to [1e-5, 1e-2] and the floor binds on this task, so "
                             "the schedule stops regulating; 0.25 moves the range down without "
                             "changing the schedule.")
    parser.add_argument("--entropy_coef", type=float, default=None,
                        help="Entropy bonus. UniFP's is 0.01, and it is the term pushing the "
                             "action noise back up once the learning rate is floored.")
    return parser


def git_output(*args: str) -> str:
    try:
        return subprocess.run(("git", *args), cwd=ROOT, capture_output=True,
                              text=True, check=True).stdout.strip()
    except Exception:
        return ""


def stack_versions() -> dict:
    """What actually got imported, so a run record says which stack produced it."""
    from importlib.metadata import PackageNotFoundError, version

    out = {}
    for package in ("isaacsim", "isaaclab", "isaaclab-rl", "rsl-rl-lib", "torch"):
        try:
            out[package] = version(package)
        except PackageNotFoundError:
            out[package] = None
    return out


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def main() -> int:
    parser = build_parser()
    if len(sys.argv) == 1 or "-h" in sys.argv or "--help" in sys.argv:
        parser.parse_args()
        return 0

    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    sys.path.insert(0, str(ROOT))
    from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR  # noqa: E402
    from weld import build_welded_robot_usd  # noqa: E402
    from unifp_isaaclab import interface, robot as robot_mod  # noqa: E402
    from unifp_train import observations, task_cfg  # noqa: E402
    from unifp_train.env import Go2D1PosForceEnv  # noqa: E402
    from unifp_train.env_cfg import Go2D1PosForceEnvCfg  # noqa: E402

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    suffix = f"_{args.run_name}" if args.run_name else ""
    run_dir = Path(args.output).resolve() / f"{stamp}_{args.mode}_seed{args.seed}{suffix}"

    if args.robot_usd:
        robot_usd = str(Path(args.robot_usd).resolve())
    else:
        robot_usd = build_welded_robot_usd(
            go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd",
            d1_urdf_path=str(ROOT / "d1_arm/d1.urdf"),
            out_usd_path=str(ROOT / "generated/go2_d1.usd"),
            mount_pos=(0.0, 0.0, 0.08), arm_mass_kg=3.152,
        ).usd_path

    cfg = Go2D1PosForceEnvCfg()
    cfg.scene.num_envs = args.num_envs
    if args.mode in ("manifest", "eval"):
        from unifp_train import eval as evaluation

        # The environment count is part of the frozen conditions: the schedule is drawn in
        # batches across environments, so the same seed with a different batch is a different
        # episode set. For 'eval' the manifest dictates it rather than the command line.
        if args.mode == "eval":
            manifest = evaluation.load(args.manifest)
            cfg.scene.num_envs = manifest["episode_count"]
            args.seed = manifest["seed"]
        # Evaluation runs with the force curriculum already open -- a policy that has never been
        # pushed is not what this task is for.
        cfg.force_start_step = 0
    cfg.seed = args.seed
    cfg.robot = robot_mod.make_robot_cfg(robot_usd, prim_path="/World/envs/env_.*/Robot")
    cfg.robot_usd = robot_usd
    if args.force_start_iteration is not None:
        cfg.force_start_step = args.force_start_iteration * task_cfg.NUM_STEPS_PER_ENV
    if not args.headless:
        # Follow the robot rather than sit in the world frame: a walking quadruped leaves a fixed
        # camera within seconds. Offset from the base, slightly above and behind the right
        # shoulder, which keeps the arm and the front feet both in frame.
        cfg.viewer.origin_type = "asset_root"
        cfg.viewer.asset_name = "robot"
        cfg.viewer.env_index = 0
        cfg.viewer.eye = (1.6, -1.8, 0.9)
        cfg.viewer.lookat = (0.0, 0.0, 0.3)

    env = Go2D1PosForceEnv(cfg)
    obs, _ = env.reset()
    if not args.headless:
        # UniFP's overlay: the goal, the force-displaced goal the reward actually scores, the
        # tool tip, the sphere the goal is drawn on, the trajectory between start and goal, and
        # the applied and commanded force arrows. Viewer only -- headless training allocates none
        # of it.
        env.set_debug_vis(True)
    report = {
        "mode": args.mode,
        "seed": args.seed,
        # Recorded so a run says what produced it without the directory name having to be parsed,
        # which is the convention run_position_only.py already follows.
        "arguments": {k: v for k, v in vars(args).items() if not k.startswith("_")},
        "num_envs": env.num_envs,
        "actor_obs": list(obs["policy"].shape),
        "critic_obs": list(obs["critic"].shape),
        "estimate_obs": list(obs["estimates"].shape),
        "expected_actor": [env.num_envs, interface.NUM_OBS],
        "expected_critic": [env.num_envs, observations.NUM_CRITIC_OBS],
        "expected_estimates": [env.num_envs, observations.NUM_ESTIMATES],
        "action_space": env.cfg.action_space,
        "episode_length_steps": int(env.max_episode_length),
        "joint_order_ok": list(env._robot.joint_names)[env._order[0]] == interface.DOF_NAMES[0],
        "force_start_step": cfg.force_start_step,
        "force_start_iteration": cfg.force_start_step // task_cfg.NUM_STEPS_PER_ENV,
        "robot_usd": robot_usd,
        "robot_usd_sha256": hashlib.sha256(Path(robot_usd).read_bytes()).hexdigest(),
        "stack": stack_versions(),
        "git_commit": git_output("rev-parse", "HEAD"),
        "git_status": git_output("status", "--short"),
    }
    report["shapes_ok"] = (report["actor_obs"] == report["expected_actor"]
                           and report["critic_obs"] == report["expected_critic"]
                           and report["estimate_obs"] == report["expected_estimates"])

    if args.mode == "train":
        status = train(args, env, report, run_dir)
    elif args.mode in ("manifest", "eval"):
        status = evaluate(args, env, report, run_dir)
        write_json(run_dir / "run.json", report)
        report["run_dir"] = str(run_dir)
    else:
        # Written before the loop as well as after, so a run that is interrupted -- a viewer
        # session closed, a crash -- still leaves a record. Writing it only at the end means an
        # aborted run is invisible, which is exactly the run most worth having a note of.
        # The checkpoint goes in before the loop, not just after it. `rollout()` records the
        # iteration and source once it has loaded the weights, but a viewer session closed
        # halfway would otherwise leave a record that does not say which policy was on screen --
        # which is most of what such a record is for.
        if args.mode == "play":
            report["checkpoint"] = os.path.abspath(args.checkpoint)
        report["status"] = "running"
        write_json(run_dir / "run.json", report)
        report["run_dir"] = str(run_dir)
        status = rollout(args, env, obs, report, simulation_app)
        report["status"] = "finished"
        write_json(run_dir / "run.json", report)

    print(json.dumps(report, indent=2), flush=True)
    env.close()
    # Isaac Sim 5.1 hangs in teardown here as it does for run_unifp_isaaclab.py; everything this
    # prints is already flushed, so leave rather than wait.
    sys.stdout.flush()
    os._exit(status)


def rollout(args, env, obs, report: dict, simulation_app) -> int:
    """Step the environment open-loop, or under an existing checkpoint."""
    import torch

    from unifp_isaaclab import interface

    policy = None
    if args.mode == "play":
        # `eval.EvaluablePolicy` rather than `UniFPPolicy` because it loads *both* checkpoint
        # layouts -- UniFP's single module and rsl-rl 5.x's split actor -- behind the same `act`.
        # Without it `play` can only show a policy trained in the other stack, which stopped being
        # the interesting case the moment this one trained a policy of its own. The two loaders
        # agree to 0.0 on the same weights (`tests/test_unifp_train.py`), so this changes nothing
        # about what an Isaac Gym checkpoint does here.
        from unifp_train.eval import EvaluablePolicy

        policy = EvaluablePolicy(args.checkpoint, device=str(env.device))
        report["checkpoint"] = os.path.abspath(args.checkpoint)
        report["checkpoint_iteration"] = policy.iteration
        report["checkpoint_source"] = policy.source

    rewards_seen, resets, pushed, errors = [], 0, 0, []
    for _ in range(args.steps):
        if not simulation_app.is_running():
            break
        if policy is None:
            actions = torch.zeros(env.num_envs, interface.NUM_ACTIONS, device=env.device)
        else:
            actions = policy.act(obs["policy"])
        obs, reward, terminated, truncated, _ = env.step(actions)
        rewards_seen.append(float(reward.mean()))
        resets += int((terminated | truncated).sum())
        pushed += int((env._ee_force_w.norm(dim=-1) > 1e-6).sum())
        # The tool-tip error is reported because the main objective, `tracking_ee_force_world`,
        # is `exp(-2 * error)`: anything past about 4 m is indistinguishable from zero in the
        # reward and from a broken frame convention in the log. A number here says which.
        errors.append((env._tip_pos() - env._goal_world()).norm(dim=-1))

    report["steps"] = len(rewards_seen)
    report["mean_reward_per_step"] = round(sum(rewards_seen) / max(1, len(rewards_seen)), 5)
    report["driven_by"] = "zero actions" if policy is None else os.path.basename(args.checkpoint)
    report["resets"] = resets
    #: Environment-steps in which a non-zero external force was acting. Zero unless the force
    #: curriculum was opened with --force_start_iteration.
    report["env_steps_with_external_force"] = pushed
    if errors:
        stacked = torch.cat(errors)
        report["tip_goal_error_m"] = {
            "median": round(float(stacked.median()), 4),
            "p90": round(float(stacked.quantile(0.9)), 4),
            "max": round(float(stacked.max()), 4),
        }
        # Metres of error means the tool and its goal are being measured in different frames,
        # not that the policy is bad: the goal sphere is about half a metre across.
        report["frames_consistent"] = bool(stacked.median() < 1.0)
    # Per-term means, so a comparison against the Isaac Gym recording is 27 numbers rather than
    # one. A single total can agree while two terms are wrong in opposite directions.
    report["reward_terms_per_step"] = {
        name: round(float(total.mean()) / max(1, len(rewards_seen)), 6)
        for name, total in env._episode_sums.items()}
    return 0 if report["shapes_ok"] and report.get("frames_consistent", True) else 1


def evaluate(args, env, report: dict, run_dir: Path) -> int:
    """Freeze an evaluation set, or score a policy against one."""
    from unifp_train import eval as evaluation

    run_dir.mkdir(parents=True, exist_ok=True)
    conditions = evaluation.conditions_of(env)
    report["conditions"] = conditions

    if args.mode == "manifest":
        # The schedule is policy-independent by construction -- every draw is triggered by the
        # episode clock, not by anything the robot does -- so the set is realised with zero
        # actions and the digests hold for any policy. `eval` re-checks that on every run.
        records = evaluation.run_episodes(env, evaluation.build(
            args.role, env.num_envs, args.seed, conditions), policy=None)
        manifest = evaluation.build(
            args.role, env.num_envs, args.seed, conditions,
            episodes=[{"index": r["index"], "schedule_sha256": r["schedule_sha256"]}
                      for r in records])
        out = Path(args.manifest_out or
                   ROOT / f"results/manifests/unifp_isaaclab_{args.role}.json")
        evaluation.save(manifest, str(out))
        report["manifest"] = str(out)
        report["manifest_sha256"] = manifest["content_sha256"]
        report["episodes"] = len(manifest["episodes"])
        report["status"] = "manifest written"
        print(f"[manifest] {out}  {manifest['content_sha256'][:16]}  "
              f"{len(manifest['episodes'])} episodes", flush=True)
        return 0

    manifest = evaluation.load(args.manifest)
    if args.hold_command:
        # Replacing the sampler covers both the five-second timer and the reset path, so every
        # environment holds this command for the whole episode.
        import torch as _torch

        fixed = _torch.tensor(args.hold_command, device=env.device)

        def hold(env_ids):
            env._commands[env_ids, :3] = fixed

        env._resample_velocity_commands = hold
        report["schedule_overridden"] = args.hold_command
        print(f"[eval] holding base velocity command {tuple(args.hold_command)} for every "
              f"episode; schedule digests will not match the manifest by design", flush=True)
    policy = None
    if not args.zero_actions:
        policy = evaluation.EvaluablePolicy(args.checkpoint, device=str(env.device))
        report["checkpoint"] = os.path.abspath(args.checkpoint)
        report["checkpoint_source"] = policy.source
        report["checkpoint_iteration"] = policy.iteration
    controller = "zero actions" if policy is None else (
        f"{os.path.basename(args.checkpoint)} ({policy.source})")

    records = evaluation.run_episodes(env, manifest, policy=policy)
    summary = evaluation.summarise(
        records, manifest, controller,
        conditions=evaluation.compare_conditions(manifest, conditions))
    report["summary"] = summary
    report["status"] = "evaluated"
    write_json(run_dir / "eval.json", summary)
    write_json(run_dir / "eval_episodes.json", {"records": records})

    quiet = summary["goal_tracking_quiet_m"]
    print(f"\n[eval] {controller}  on {manifest['role']} "
          f"({summary['episodes']} episodes)", flush=True)
    print(f"  goal tracking, unforced steps : "
          f"{quiet['median'] * 100:.1f} cm median, {quiet['p90'] * 100:.1f} cm p90"
          if quiet else "  goal tracking: no unforced steps", flush=True)
    for key, label, scale, unit in (
            ("unified_tracking_m", "unified tracking (the objective)", 100, "cm"),
            ("unified_tracking_pushed_m", "unified tracking, under force", 100, "cm"),
            ("force_realised_n", "force realised of commanded", 1, "N"),
            ("estimator_err_n", "estimator error", 1, "N"),
            ("base_vel_err_m_s", "base velocity error", 1, "m/s")):
        stat = summary[key]
        if stat:
            print(f"  {label:31s}: {stat['median'] * scale:.2f} {unit} median", flush=True)
    cmd = summary["force_cmd_n"]
    if cmd:
        print(f"  {'force commanded':31s}: {cmd['median']:.2f} N median", flush=True)
    print(f"  falls                          : {summary['falls']['count']} of "
          f"{summary['episodes']}", flush=True)
    if summary["schedule_mismatches"] and not args.hold_command:
        print(f"  !! {len(summary['schedule_mismatches'])} episodes did not reproduce their "
              f"frozen schedule; this run is not comparable", flush=True)
        return 1
    if summary["condition_mismatches"]:
        print(f"  !! conditions differ from the manifest: {summary['condition_mismatches']}",
              flush=True)
    return 0


def train(args, env, report: dict, run_dir: Path) -> int:
    """PPO with the adaptation module, through rsl-rl's `OnPolicyRunner`."""
    from copy import deepcopy

    from isaaclab.utils.io import dump_yaml
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
    from rsl_rl.runners import OnPolicyRunner

    from unifp_train.agent import make_agent_cfg

    if not report["shapes_ok"] or not report["joint_order_ok"]:
        report["status"] = "refused: observation shapes or joint order wrong"
        write_json(run_dir / "run.json", report)
        return 1

    agent_cfg = make_agent_cfg(seed=args.seed, device=str(env.device), iterations=args.iterations,
                               run_name=args.run_name)
    departures = {}
    if args.std_type is not None:
        departures["std_type"] = args.std_type
        agent_cfg["actor"]["distribution_cfg"]["std_type"] = args.std_type
    if args.init_std is not None:
        departures["init_std"] = args.init_std
        agent_cfg["actor"]["distribution_cfg"]["init_std"] = args.init_std
    if args.entropy_coef is not None:
        departures["entropy_coef"] = args.entropy_coef
        agent_cfg["algorithm"]["entropy_coef"] = args.entropy_coef
    if args.learning_rate_scale is not None:
        departures["learning_rate_scale"] = args.learning_rate_scale
        agent_cfg["algorithm"]["learning_rate_scale"] = args.learning_rate_scale
    if args.estimator_learning_rate is not None:
        departures["estimator_learning_rate"] = args.estimator_learning_rate
        agent_cfg["algorithm"]["estimator_learning_rate"] = args.estimator_learning_rate
    if args.fixed_learning_rate is not None:
        departures["fixed_learning_rate"] = args.fixed_learning_rate
        agent_cfg["algorithm"]["fixed_learning_rate"] = args.fixed_learning_rate
    report["departures_from_unifp"] = departures

    run_dir.mkdir(parents=True, exist_ok=True)
    # The watcher checks liveness by this file rather than by `pgrep -f run_unifp_train.py`,
    # which matches any shell that merely mentions the script -- including the one that ran the
    # grep, so it reports a finished run as still going.
    (run_dir / "train.pid").write_text(f"{os.getpid()}\n")
    report["run_dir"] = str(run_dir)
    report["pid"] = os.getpid()
    report["iterations_requested"] = args.iterations
    report["agent"] = deepcopy(agent_cfg)
    report["status"] = "running"
    write_json(run_dir / "run.json", report)
    write_json(run_dir / "agent.json", agent_cfg)
    dump_yaml(str(run_dir / "env.yaml"), env.cfg)

    # clip_actions matches UniFP, which clips actions at +/-100 before its own scaling.
    from unifp_isaaclab import interface

    wrapped = RslRlVecEnvWrapper(env, clip_actions=interface.CLIP_ACTIONS)
    runner = OnPolicyRunner(wrapped, deepcopy(agent_cfg), log_dir=str(run_dir),
                            device=str(env.device))
    if args.resume_from:
        runner.load(args.resume_from)
    if args.diagnose_storage:
        runner.alg.diagnose_storage_requested = True
        report["resumed_from"] = os.path.abspath(args.resume_from)

    # Isaac Sim installs its own signal handling, so a plain SIGINT kills the process without
    # Python ever raising KeyboardInterrupt and the shutdown below never runs -- which is how the
    # run of 2026-09-20 was left with a `run.json` still saying "running". Installing handlers
    # here puts the interpreter back in charge of both the interrupt and a polite terminate.
    import signal

    def stop(signum, _frame):
        raise KeyboardInterrupt(f"signal {signal.Signals(signum).name}")

    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, stop)

    started = datetime.now(timezone.utc)
    status = 0
    try:
        runner.learn(num_learning_iterations=args.iterations, init_at_random_ep_len=True)
        report["status"] = "complete"
    except KeyboardInterrupt:
        report["status"] = "interrupted"
    except Exception as error:  # noqa: BLE001 - recorded, then re-raised through the status code
        report["status"] = f"failed: {type(error).__name__}: {error}"
        status = 1
    finally:
        report["iterations_completed"] = runner.current_learning_iteration
        report["wall_clock_s"] = round(
            (datetime.now(timezone.utc) - started).total_seconds(), 1)
        checkpoint = run_dir / f"model_{runner.current_learning_iteration}.pt"
        if report["status"] != "complete" or not checkpoint.exists():
            runner.save(str(checkpoint))
        report["final_checkpoint"] = str(checkpoint)
        write_json(run_dir / "run.json", report)
        (run_dir / "train.pid").unlink(missing_ok=True)
    return status


if __name__ == "__main__":
    raise SystemExit(main())
