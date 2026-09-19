"""Watch a trained Go2+D1 UniFP policy, with the camera on the robot and the forces switched on.

UniFP's own `play_*.py` works, but two things make it a poor look at a *force* policy:

  * **It applies no external forces.** The force curriculum is gated on `env.global_steps`, which
    starts at 0 in a fresh process, so every play session sits below `force_start_step` and the
    robot is never pushed. You would be watching a force-trained policy with the forces off and
    have no way to tell from the window. This is the same defect as F-071, which was fixed for
    resumed *training*; the play path has it too. `--forces` winds `global_steps` past the gate.
  * **The camera does not follow the robot**, which walks out of frame in a few seconds.

It also prints one readable line instead of two vectors per step.

    python unifp_go2d1/play_policy.py --task=go2d1_pos_force --load_run <run> \
        --checkpoint 48800 --forces
    python unifp_go2d1/play_policy.py --task=go2d1_pos_force --load_run <run> \
        --checkpoint 48800 --forces --headless --steps 600

Nothing here changes the policy: same environment and observations, and randomisation is turned off
exactly as upstream's play script turns it off. It changes what you are shown, not what runs.
"""

import argparse
import os
import sys

UNIFP = os.environ.get("UNIFP_ROOT", os.path.expanduser("~/thesis_b_legacy/UniFP"))
sys.path.insert(0, UNIFP)


def main():
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--forces", action="store_true",
                     help="apply the external forces the policy was trained against "
                          "(winds global_steps past commands.force_start_step)")
    pre.add_argument("--follow", dest="follow", action="store_true", default=True)
    pre.add_argument("--no-follow", dest="follow", action="store_false")
    pre.add_argument("--steps", type=int, default=0, help="0 = run until killed")
    pre.add_argument("--command", nargs=3, type=float, metavar=("VX", "VY", "WZ"),
                     help="hold a fixed base velocity command instead of the sampled one")
    pre.add_argument("--report-every", type=int, default=50)
    pre.add_argument("--out", metavar="DIR",
                     help="write a run directory (run.json + trace.csv) so the rollout can be "
                          "recorded with ./dashboard.py record")
    known, rest = pre.parse_known_args()
    sys.argv = [sys.argv[0]] + rest

    import isaacgym  # noqa: F401  (must precede torch)
    import torch
    import legged_gym.envs  # noqa: F401  (importing registers the tasks)
    from legged_gym.utils import get_args, task_registry

    args = get_args()
    env_cfg, train_cfg = task_registry.get_cfgs(name=args.task)

    # The same overrides upstream's play script uses: one robot, no noise, no randomisation.
    env_cfg.env.num_envs = 1
    env_cfg.terrain.num_rows, env_cfg.terrain.num_cols = 5, 5
    env_cfg.terrain.curriculum = False
    env_cfg.noise.add_noise = False
    for flag in ("randomize_friction", "push_robots", "randomize_base_mass", "randomize_leg_mass",
                 "randomize_gripper_mass", "randomize_motor", "randomize_base_com"):
        setattr(env_cfg.domain_rand, flag, False)
    env_cfg.env.test = True
    if args.flat_terrain:
        env_cfg.terrain.height = [0.0, 0.0]

    env, _ = task_registry.make_env(name=args.task, args=args, env_cfg=env_cfg)
    obs = env.get_observations()
    train_cfg.runner.resume = True
    runner, train_cfg = task_registry.make_alg_runner(env=env, name=args.task, args=args,
                                                      train_cfg=train_cfg)
    policy = runner.get_inference_policy(device=env.device)
    env.play = True

    steps_per_iter = train_cfg.runner.num_steps_per_env
    if known.forces:
        env.global_steps = (env_cfg.commands.force_start_step + 1) * steps_per_iter
        print(f"[play] forces ON: global_steps={env.global_steps} "
              f"(gate at >{env_cfg.commands.force_start_step * steps_per_iter}); "
              f"EE force command range {env_cfg.commands.max_push_force_xyz_gripper_cmd} N",
              flush=True)
    else:
        print("[play] forces OFF (upstream play behaviour) -- pass --forces to push the gripper",
              flush=True)

    trace, trace_path = None, None
    if known.out:
        os.makedirs(known.out, exist_ok=True)
        trace_path = os.path.join(known.out, "trace.csv")
        trace = open(trace_path, "w")
        trace.write("step,tip_err_l1_m,tip_err_x_m,tip_err_y_m,tip_err_z_m,base_z_m,"
                    "force_cmd_n,force_meas_n,force_est_n\n")

    total = known.steps or 10 ** 9
    info = {}
    with torch.no_grad():
        for step in range(total):
            actions = policy(obs, info)
            if known.command:
                env.commands[:, 0], env.commands[:, 1], env.commands[:, 2] = known.command
            obs, _rew, _done, _extras = env.step(actions.detach())

            if known.follow and not args.headless:
                base = env.root_states[0, :3].tolist()
                env.set_camera([base[0] + 1.6, base[1] - 1.6, base[2] + 0.9], base)

            if trace is not None or step % known.report_every == 0:
                tip = env.ee_pos[0]
                goal = env.curr_ee_goal_cart_world[0]
                err = torch.abs(tip - goal)
                f_meas = env.forces[0, env.gripper_idx, 0:3]
                f_cmd = env.current_Fxyz_gripper_cmd[0]
                latents = info.get("latents")

            if trace is not None:
                est_mag = 0.0
                if latents is not None:
                    e = latents[0, 6:9] / env.obs_scales.ee_force
                    est_mag = float(e.norm()) if torch.is_tensor(e) else float((e ** 2).sum() ** 0.5)
                trace.write(f"{step},{err.sum():.5f},{err[0]:.5f},{err[1]:.5f},{err[2]:.5f},"
                            f"{env.root_states[0, 2]:.5f},{f_cmd.norm():.4f},"
                            f"{f_meas.norm():.4f},{est_mag:.4f}\n")

            if step % known.report_every == 0:
                line = (f"[{step:6d}] tip err L1 {err.sum() * 100:5.1f} cm "
                        f"(xyz {err[0] * 100:4.1f} {err[1] * 100:4.1f} {err[2] * 100:4.1f})"
                        f"  base z {env.root_states[0, 2]:.3f} m"
                        f"  |F| cmd {f_cmd.norm():4.1f} meas {f_meas.norm():4.1f} N")
                if latents is not None:
                    # `latents` comes back as a numpy array, not a tensor.
                    est = latents[0, 6:9] / env.obs_scales.ee_force
                    mag = float(est.norm()) if torch.is_tensor(est) else float((est ** 2).sum() ** 0.5)
                    line += f" est {mag:4.1f} N"
                print(line, flush=True)

    if trace is not None:
        trace.close()
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import run_metadata
        run_metadata.write(
            known.out, mode="playback", seed=train_cfg.seed, status="finished",
            num_envs=env.num_envs, steps=known.steps,
            urdf=os.path.join(UNIFP, env_cfg.asset.file),
            command=" ".join(sys.argv),
            notes=f"policy rollout from {args.load_run}/model_{args.checkpoint}.pt, "
                  f"forces {'on' if known.forces else 'off'}, randomisation off",
            extra={"checkpoint": args.checkpoint, "load_run": args.load_run,
                   "forces": bool(known.forces), "trace": os.path.basename(trace_path)})
        print(f"[play] wrote {known.out}", flush=True)

    print("[play] done", flush=True)


if __name__ == "__main__":
    main()
