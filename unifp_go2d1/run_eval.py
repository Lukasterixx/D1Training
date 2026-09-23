"""Build a frozen manifest, or evaluate a checkpoint (or the zero-action baseline) against one.

    # freeze a set of episodes, recording the schedule each seed produces
    python unifp_go2d1/run_eval.py build --role development --episodes 50 --seed 20260920

    # the baseline, then a checkpoint, on the same frozen set
    python unifp_go2d1/run_eval.py eval --manifest results/manifests/unifp_development.json --zero
    python unifp_go2d1/run_eval.py eval --manifest results/manifests/unifp_development.json \
        --load_run Sep18_11-10-15_ --checkpoint 48800

Each `eval` writes a run directory (`eval.json`, `episodes.csv`, `run.json`) for
`./dashboard.py record`. Randomisation and observation noise are off, and external forces are on:
evaluating a force policy with the forces off would measure the wrong thing (F-082).
"""

import argparse
import json
import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
UNIFP = os.environ.get("UNIFP_ROOT", os.path.expanduser("~/thesis_b_legacy/UniFP"))
sys.path.insert(0, UNIFP)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

DEFAULT_MANIFEST_DIR = os.path.join(REPO, "results", "manifests")


def make_env(args, episodes, task="go2d1_pos_force"):
    """One environment per manifest episode, randomisation and noise off, external forces on."""
    from legged_gym.utils import task_registry

    env_cfg, train_cfg = task_registry.get_cfgs(name=task)
    env_cfg.env.num_envs = episodes
    env_cfg.terrain.num_rows, env_cfg.terrain.num_cols = 5, 5
    env_cfg.terrain.curriculum = False
    env_cfg.noise.add_noise = False
    for flag in ("randomize_friction", "push_robots", "randomize_base_mass", "randomize_leg_mass",
                 "randomize_gripper_mass", "randomize_motor", "randomize_base_com"):
        setattr(env_cfg.domain_rand, flag, False)
    env_cfg.env.test = True

    env, env_cfg = task_registry.make_env(name=task, args=args, env_cfg=env_cfg)
    env.play = True
    # Open the force curriculum gate: global_steps starts at 0 in a fresh process, so without
    # this the policy is evaluated with the forces it was trained against switched off (F-082).
    env.global_steps = (env_cfg.commands.force_start_step + 1) * train_cfg.runner.num_steps_per_env
    return env, env_cfg, train_cfg


def conditions_of(env_cfg, train_cfg, episodes):
    """What an evaluation run must match for its numbers to be comparable."""
    c = env_cfg.commands
    return {
        "task": "go2d1_pos_force",
        "unifp_commit": "68847a070f88d731058c3d8476929bc3b205f5bd",
        "episode_length_s": float(env_cfg.env.episode_length_s),
        "policy_hz": round(1.0 / (0.005 * env_cfg.control.decimation), 6),
        # Part of the frozen conditions: the environment draws its schedule in batches across
        # environments, so the same seed with a different batch size is a different episode set.
        "num_envs": episodes,
        "noise": False,
        "domain_randomisation": False,
        "forces_active": True,
        "ee_force_cmd_range_n": list(c.max_push_force_xyz_gripper_cmd),
        "ee_force_ext_range_n": list(c.max_push_force_xyz_gripper_ext),
        "gripper_force_kp": list(c.gripper_force_kp_range),
        "goal_sphere_centre_m": [env_cfg.goal_ee.sphere_center.x_offset,
                                 env_cfg.goal_ee.sphere_center.y_offset,
                                 env_cfg.goal_ee.sphere_center.z_invariant_offset],
        "goal_radius_m": list(env_cfg.goal_ee.ranges.pos_l),
        "goal_pitch_rad": [round(v, 6) for v in env_cfg.goal_ee.ranges.pos_p],
        "goal_yaw_rad": [round(v, 6) for v in env_cfg.goal_ee.ranges.pos_y],
        "lin_vel_x_range": list(c.ranges.lin_vel_x),
        "action_scale": env_cfg.control.action_scale,
        "tool_body": "ee_gripper_link",
        "terrain": f"{env_cfg.terrain.num_rows}x{env_cfg.terrain.num_cols} "
                   f"{env_cfg.terrain.mesh_type} height {list(env_cfg.terrain.height)}",
    }


def cmd_build(args):
    import isaacgym  # noqa: F401
    import torch  # noqa: F401
    import legged_gym.envs  # noqa: F401
    from legged_gym.utils import get_args
    import eval_manifest
    from evaluate import run_episodes

    gym_args = get_args()
    env, env_cfg, train_cfg = make_env(gym_args, args.episodes)
    conditions = conditions_of(env_cfg, train_cfg, args.episodes)

    draft = eval_manifest.build(args.role, args.episodes, args.seed, conditions)
    print(f"[build] realising {args.episodes} episodes to record their schedules "
          f"(zero-action controller; the schedule does not depend on the policy)")
    started = time.monotonic()
    records = run_episodes(env, draft, policy=None, device=gym_args.rl_device,
                           progress=lambda r: print(
                               f"  episode {r['index']:3d}  {r['steps']:4d} steps"
                               f"{'  FELL' if r['fell'] else ''}", flush=True))
    episodes = [{"index": r["index"], "schedule_sha256": r["schedule_sha256"]} for r in records]
    manifest = eval_manifest.build(args.role, args.episodes, args.seed, conditions, episodes)

    path = args.out or os.path.join(DEFAULT_MANIFEST_DIR, f"unifp_{args.role}.json")
    eval_manifest.save(manifest, path)
    print(f"[build] wrote {path}")
    print(f"[build] content_sha256 {manifest['content_sha256']}")
    print(f"[build] {time.monotonic() - started:.0f}s, "
          f"{sum(r['fell'] for r in records)} of {len(records)} episodes fell under zero actions")


def cmd_eval(args):
    import isaacgym  # noqa: F401
    import torch  # noqa: F401
    import legged_gym.envs  # noqa: F401
    from legged_gym.utils import get_args, task_registry
    import eval_manifest
    import run_metadata
    from evaluate import run_episodes, summarise

    manifest = eval_manifest.load(args.manifest)
    gym_args = get_args()
    env, env_cfg, train_cfg = make_env(gym_args, manifest["episode_count"])

    controller = "zero_actions"
    policy = None
    if not args.zero:
        train_cfg.runner.resume = True
        runner, train_cfg = task_registry.make_alg_runner(env=env, name=gym_args.task,
                                                          args=gym_args, train_cfg=train_cfg)
        policy = runner.get_inference_policy(device=gym_args.rl_device)
        controller = f"{gym_args.load_run}/model_{gym_args.checkpoint}.pt"

    mismatches = eval_manifest.compare_conditions(
        manifest, conditions_of(env_cfg, train_cfg, manifest["episode_count"]))
    if mismatches:
        print("[eval] CONDITION MISMATCHES against the manifest:")
        for key, expected, got in mismatches:
            print(f"  {key}: manifest {expected!r}, this run {got!r}")

    print(f"[eval] {controller} on {manifest['role']} ({manifest['episode_count']} episodes)")
    started = time.monotonic()
    records = run_episodes(env, manifest, policy=policy, device=gym_args.rl_device,
                           progress=lambda r: print(
                               f"  episode {r['index']:3d}  {r['steps']:4d} steps"
                               f"{'  FELL' if r['fell'] else ''}"
                               + (f"  goal {r['goal_tracking_quiet_m']['median'] * 100:5.1f} cm"
                                  if r['goal_tracking_quiet_m'] else ""), flush=True))
    result = summarise(records, manifest, controller,
                       [{"key": k, "manifest": e, "run": g} for k, e, g in mismatches])
    result["elapsed_s"] = round(time.monotonic() - started, 1)

    out = args.out or os.path.join(
        UNIFP, "logs", "eval",
        f"{time.strftime('%Y%m%dT%H%M%S')}_{manifest['role']}_"
        f"{'zero' if args.zero else str(gym_args.checkpoint)}")
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "eval.json"), "w") as fh:
        json.dump({"summary": result, "episodes": records}, fh, indent=2)
    with open(os.path.join(out, "episodes.csv"), "w") as fh:
        fh.write("index,steps,fell,goal_quiet_median_m,unified_median_m,"
                 "unified_pushed_median_m,force_cmd_mean_n,force_realised_mean_n,"
                 "estimator_err_median_n,base_vel_err_median_m_s\n")
        for r in records:
            def g(key, field="median"):
                return f"{r[key][field]:.5f}" if r.get(key) else ""
            fh.write(f"{r['index']},{r['steps']},{int(r['fell'])},"
                     f"{g('goal_tracking_quiet_m')},{g('unified_tracking_m')},"
                     f"{g('unified_tracking_pushed_m')},{g('force_cmd_n','mean')},"
                     f"{g('force_realised_n','mean')},{g('estimator_err_n')},"
                     f"{g('base_vel_err_m_s')}\n")
    run_metadata.write(out, mode="eval", seed=manifest["seed"], status="finished",
                       num_envs=manifest["episode_count"], iterations=None,
                       urdf=os.path.join(UNIFP, env_cfg.asset.file),
                       command=" ".join(sys.argv),
                       notes=f"{controller} on {manifest['role']} "
                             f"({manifest['episode_count']} episodes)",
                       extra={"controller": controller, "manifest": os.path.basename(args.manifest),
                              "manifest_sha256": manifest["content_sha256"],
                              "schedule_mismatches": result["schedule_mismatches"],
                              "condition_mismatches": result["condition_mismatches"]})

    print(json.dumps(result, indent=2))
    print(f"[eval] wrote {out}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="command", required=True)
    b = sub.add_parser("build")
    b.add_argument("--role", default="development")
    b.add_argument("--episodes", type=int, default=50)
    b.add_argument("--seed", type=int, default=20260920)
    b.add_argument("--out")
    e = sub.add_parser("eval")
    e.add_argument("--manifest", required=True)
    e.add_argument("--zero", action="store_true", help="zero-action baseline")
    e.add_argument("--out")
    known, rest = p.parse_known_args()
    sys.argv = [sys.argv[0]] + rest
    (cmd_build if known.command == "build" else cmd_eval)(known)


if __name__ == "__main__":
    main()
