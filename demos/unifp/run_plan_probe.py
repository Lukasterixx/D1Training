#!/usr/bin/env python3
"""A whole-body run as the dog would play it: far goals and walking, scored against the dog's own stops.

    ./demos/unifp/run_plan_probe.py --headless --plan far --checkpoint <pt> --leg_friction 1
    ./demos/unifp/run_plan_probe.py --headless --plan walk --checkpoint <pt> --leg_friction 1

`folded_start_probe.py` holds one goal and zeroes the velocity command. This plays a schedule, one per environment: the
hand's goal slid in spherical coordinates as `EeGoalTrajectory` slides it, the velocity command held for a while and
released, the gait clock running as `gait_step` runs it. Handed over as the dog hands over: the arm at zero, an empty
history, and with `--real_stance` the dog's own stance from its four `66600` runs (the simulator's settled stance is
4-5 cm lower, with the calves 0.25-0.4 rad further in).

Scored per environment, against what would stop the dog (`reach_arm.ArmWatch`, `press_live`):
  * lead: the asked arm target against the 9 Hz reports: past 25 deg for 0.5 s, or 60 at once, from 1 s on; 20 deg in one 20 ms tick;
  * legs: the largest leg target off default, 1.2 rad in the first second, 0.9 after;
  * tilt: 15 deg;
  * falls, and contact on the dog's body (the arm's links report none, so a strike is read on what it strikes).
Plus the hand against the goal at the end of each hold, the arm's pose there, and for walking the speed it made good.

Simulation. Nothing here is a claim about the dog: it says which plans are worth a dry run, and F-258 is why a clean
result here is not a forecast.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent

#: The dog at handover, the mean of its four `66600` runs: leg q - default in UniFP's order, spread 0.008 rad.
REAL_LEG_OFFSETS = (-0.007, 0.062, -0.162, 0.011, 0.074, -0.184, 0.099, -0.061, -0.182, -0.111, -0.05, -0.206)
REAL_BASE_M, REAL_PITCH_DEG, REAL_ROLL_DEG = 0.288, -3.7, 0.4
#: The stops: `reach_arm.MAX_LEAD_DEG`, `LEAD_HOLD_S`, `MAX_LEAD_HARD_DEG`, `MAX_STEP_DEG`, `GRACE_S`; `press_live`'s bounds.
LEAD_DEG, LEAD_HOLD_S, LEAD_HARD_DEG, STEP_DEG, GRACE_S = 25.0, 0.5, 60.0, 20.0, 1.0
LEG_FIRST_RAD, LEG_AFTER_RAD, TILT_DEG = 1.2, 0.9, 15.0

START = (0.5, math.pi / 5, 0.0)
#: `interface.EE_GOAL_CENTER_OFFSET`: the goal sphere's centre over the ground under the base.
WIPE_CENTRE_M = 0.49
#: Driving up to a bench on the slow sticks, the last move still held when A locks them: back and forward, both turns, both ways sideways, ending about where it began
#: so the bench is where the probe put it.
WIPE_WALK = [(3.0, (-.2, 0, 0)), (3.0, (.2, 0, 0)), (2.0, (0, 0, .4)), (2.0, (0, 0, -.4)), (1.5, (0, -.15, 0)), (1.5, (0, .15, 0))]


def far_plans(args):
    """Every goal in the grid: out from the start goal, held, and back."""
    plans = []
    for r in args.radius:
        for p in args.pitch_deg:
            for y in args.yaw_deg:
                plans.append({"name": f"r{r:.2f} p{p:+03.0f} y{y:+03.0f}", "goals": [((r, math.radians(p), math.radians(y)), args.move_s, args.hold_goal_s),
                                                                                      (START, args.move_s, args.tail_s)], "walk": []})
    return plans


def tour_plans(args):
    """The named goals, one run each, and the whole tour: what the console offers, in the order a test would go."""
    def goal(r, p, y):
        return (r, math.radians(p), math.radians(y))
    named = {"low": goal(.45, 0, 0), "left": goal(.45, 36, 40), "right": goal(.45, 36, -40), "forward": goal(.55, 15, 0)}
    plans = [{"name": n, "goals": [(g, args.move_s, args.hold_goal_s), (START, args.move_s, args.tail_s)], "walk": []} for n, g in named.items()]
    plans.append({"name": "tour", "goals": [(named["low"], args.move_s, 3.0), (named["left"], args.move_s, 3.0), (named["right"], args.move_s, 3.0), (START, args.move_s, args.tail_s)], "walk": []})
    plans.append({"name": "fast low", "goals": [(named["low"], 2.0, args.hold_goal_s), (START, 2.0, args.tail_s)], "walk": []})
    return plans


def wipe_plans(args):
    """Across a flat bench: down onto the near end of a stroke, along it and back, up to the start goal. One height per environment.

    A stroke is a sweep of the goal's yaw at a fixed radius and pitch, so the hand holds its height over the bench and `GoalPath` plays it as it is.
    """
    plans = []
    for h in args.wipe_heights:
        pitch = math.asin((args.bench_top_m + h - WIPE_CENTRE_M) / args.wipe_radius)
        left, right = ((args.wipe_radius, pitch, math.radians(y)) for y in (args.wipe_yaw_deg, -args.wipe_yaw_deg))
        for force in args.wipe_force_n:
            up_pitch = math.asin((args.bench_top_m + h + 0.08 - WIPE_CENTRE_M) / args.wipe_radius)
            up_left, up_right = ((args.wipe_radius, up_pitch, math.radians(y)) for y in (args.wipe_yaw_deg, -args.wipe_yaw_deg))
            back = [(right, args.sweep_s, 1.0)] if not args.wipe_one_way else [(up_left, 2.0, 0.0), (up_right, args.sweep_s - 1.0, 0.0), (right, 2.0, 1.0), (left, args.sweep_s, 1.0)]
            plans.append({"name": f"h{h * 100:+05.1f}cm F{force:04.1f}", "height_m": h, "force_n": force,
                          "goals": [(right, args.move_s, 1.0), (left, args.sweep_s, 1.0), *back, (START, args.move_s, args.tail_s)], "walk": [],
                          "walk_first": WIPE_WALK if args.wipe_walk_first else []})
    return plans


def walk_plans(args):
    """Each velocity held for `--walk_s` and released, the hand's goal where it starts."""
    wanted = [("x+0.15", .15, 0, 0), ("x+0.20", .2, 0, 0), ("x+0.30", .3, 0, 0), ("x+0.40", .4, 0, 0), ("x+0.50", .5, 0, 0), ("x-0.20", -.2, 0, 0), ("x-0.30", -.3, 0, 0),
              ("y+0.20", 0, .2, 0), ("y-0.20", 0, -.2, 0), ("y+0.30", 0, .3, 0), ("w+0.30", 0, 0, .3), ("w-0.30", 0, 0, -.3), ("w+0.50", 0, 0, .5), ("w-0.50", 0, 0, -.5),
              ("x+0.30 w+0.30", .3, 0, .3), ("stand", 0, 0, 0)]
    plans = [{"name": n, "goals": [], "walk": [(args.walk_s, (vx, vy, wz)), (args.tail_s, (0.0, 0.0, 0.0))]} for n, vx, vy, wz in wanted]
    # A hand on the sticks: reversing, turning, stopping and going again, each change at once.
    still = (0.0, 0.0, 0.0)
    plans.append({"name": "sticks a", "goals": [], "walk": [(2.0, (.3, 0, 0)), (2.0, (-.3, 0, 0)), (2.0, (0, 0, .5)), (2.0, (0, 0, -.5)), (1.0, still), (2.0, (.3, 0, 0)), (args.tail_s, still)]})
    plans.append({"name": "sticks b", "goals": [], "walk": [(1.0, (.2, 0, 0)), (.5, still), (1.0, (.2, 0, 0)), (.5, still), (1.0, (0, .2, 0)), (.5, still), (1.0, (0, -.2, 0)), (args.tail_s, still)]})
    plans.append({"name": "sticks c", "goals": [], "walk": [(3.0, (.3, .2, .5)), (3.0, (.3, -.2, -.5)), (3.0, (-.2, 0, .3)), (args.tail_s, still)]})
    return plans


def schedule(plan, settle_s, dt, total_steps):
    """(goal (T, 3), velocity (T, 3), windows): the goal lerped in spherical coordinates, as `EeGoalTrajectory.step`."""
    goal, vel, windows = [], [], []
    now = START
    t = 0

    def hold(g, seconds, label=None):
        nonlocal t
        n = int(round(seconds / dt))
        goal.extend([g] * n)
        vel.extend([(0.0, 0.0, 0.0)] * n)
        t += n
        if label:
            windows.append((label, t - n, t, g))

    hold(START, settle_s, "settled")
    for seconds, velocity in plan.get("walk_first", []):
        n = int(round(seconds / dt))
        goal.extend([now] * n)
        vel.extend([velocity] * n)
        t += n
        windows.append(("walk", t - n, t, now))
    if plan.get("walk_first"):
        hold(START, settle_s, "after A")
    for target, move_s, hold_s in plan["goals"]:
        n = int(round(move_s / dt))
        for k in range(1, n + 1):
            u = k / n
            goal.append(tuple(a + (b - a) * u for a, b in zip(now, target)))
            vel.append((0.0, 0.0, 0.0))
        t += n
        now = target
        if "height_m" in plan:
            windows.append(("sweep", t - n, t, target))
        hold(target, hold_s, "held")
    for seconds, velocity in plan["walk"]:
        n = int(round(seconds / dt))
        goal.extend([now] * n)
        vel.extend([velocity] * n)
        t += n
        windows.append(("walk" if any(velocity) else "stopped", t - n, t, now))
    while len(goal) < total_steps:
        goal.append(now)
        vel.append((0.0, 0.0, 0.0))
    return goal[:total_steps], vel[:total_steps], windows


def leg_gains(env, torch, kp_of, kd_of):
    """Every leg joint's gains in the running actuators, by name: what `press_live` changes on the dog by sending new kp and kd."""
    for actuator in env._robot.actuators.values():
        for column, name in enumerate(actuator.joint_names):
            kind = next((k for k in ("hip", "thigh", "calf") if k in name), None)
            if kind:
                actuator.stiffness[:, column] = kp_of(kind)
                actuator.damping[:, column] = kd_of(kind)


def stop_phase(args, env, plans, device, dt, body_sensor, torch, interface, fell_at):
    """The stop, from wherever the plan got to. What `press_live._arm_stop` then `_stand_end` do, or `abort`: nothing here chooses."""
    sys.path.insert(0, str(ROOT))
    import d1_ik  # noqa: E402
    from demos.combiner_key.robot import reach_arm  # noqa: E402

    stand_kp, stand_kd = {"hip": 60.0, "thigh": 80.0, "calf": 80.0}, {"hip": 5.0, "thigh": 4.0, "calf": 4.0}
    blend_s, stand_s = 0.5, 1.0                                    # `press_live.END_BLEND_S`, `END_STAND_S`
    no = torch.zeros(env.num_envs, dtype=torch.bool, device=device)
    env._get_dones = lambda: (no, no)                              # a fall plays out: the env would reset it
    count = len(plans)
    legs_from = env._dof()[0][:, :12].clone()
    default = torch.tensor(interface.DEFAULT_DOF_POS[:12], device=device)
    held = env._arm_link.reported.clone()
    routes = []
    for i in range(count):
        points = reach_arm.route(d1_ik.to_servo_deg(held[i].cpu().numpy()), reach_arm.HOME_SERVO_DEG, reach_arm.TUCK_SERVO_DEG, reach_arm.FOLD_SERVO_DEG)
        rad = [torch.tensor(reach_arm.from_servo_deg(p), dtype=torch.float32, device=device).reshape(6) for p in points]
        routes.append(torch.stack(rad))
    longest = max(len(r) for r in routes)
    seconds = (stand_s + 0.5 + longest * 0.1 + 1.0) if args.stop_kind == "soft" else 4.0
    ticks = int(seconds / dt)
    action = torch.zeros(count, interface.NUM_ACTIONS, device=device)
    peak = torch.zeros(count, device=device)
    base_min = torch.full((count,), 9.0, device=device)
    hand_low = torch.full((count,), 9.0, device=device)
    contact = torch.zeros(count, device=device)
    press = torch.zeros(count, device=device)
    arm_torque = torch.zeros(count, 6, device=device)
    leg_gains_now = (25.0, 0.5)
    with torch.no_grad():
        for k in range(ticks):
            t = k * dt
            if args.stop_kind == "soft":
                s = min(t / blend_s, 1.0)
                leg_gains(env, torch, lambda kind: leg_gains_now[0] + (stand_kp[kind] - leg_gains_now[0]) * s, lambda kind: leg_gains_now[1] + (stand_kd[kind] - leg_gains_now[1]) * s)
                u = min(t / stand_s, 1.0)
                target = legs_from + (default - legs_from) * u
                action[:, :12] = (target - default) / interface.ACTION_SCALE
                at = int(max(0.0, t - stand_s - 0.5) / 0.1)
                arm = torch.stack([r[min(at, len(r) - 1)] for r in routes])
                action[:, 12:18] = arm / interface.ACTION_SCALE
            else:
                leg_gains(env, torch, lambda kind: 0.0, lambda kind: 3.0)
                action[:, :12] = 0.0
                action[:, 12:18] = held / interface.ACTION_SCALE
            env._commands[:, :3] = 0.0
            env.step(action)
            peak = torch.maximum(peak, torch.rad2deg(interface.body_roll_pitch(env._robot.data.root_quat_w)).abs().amax(dim=1))
            base_min = torch.minimum(base_min, env._robot.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2])
            hand_low = torch.minimum(hand_low, env._tip_pos()[:, 2])
            contact = torch.maximum(contact, env._contact_sensor.data.net_forces_w.norm(dim=-1)[:, body_sensor].amax(dim=1))
            arm_torque = torch.maximum(arm_torque, env._robot.data.applied_torque[:, env._order[12:18]].abs())
            if args.bench:
                press = torch.maximum(press, env._bench_sensor.data.net_forces_w[:, 0, 2].abs())
    tilt_end = torch.rad2deg(interface.body_roll_pitch(env._robot.data.root_quat_w)).abs().amax(dim=1)
    base_end = env._robot.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2]
    return [{"name": p["name"], "fell_before": int(fell_at[i]) >= 0, "tilt_peak_deg": float(peak[i]), "tilt_end_deg": float(tilt_end[i]), "base_min_m": float(base_min[i]),
             "base_end_m": float(base_end[i]), "hand_lowest_m": float(hand_low[i]), "contact_n": float(contact[i]), "seconds": seconds,
             "press_n": float(press[i]), "arm_torque_nm": [round(float(v), 1) for v in arm_torque[i]]} for i, p in enumerate(plans)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--checkpoint", default=str(ROOT / "checkpoints/unifp_go2d1_isaaclab_sticky_model_66600.pt"))
    parser.add_argument("--plan", choices=("far", "walk", "tour", "wipe"), required=True)
    parser.add_argument("--wipe_heights", type=float, nargs="+", default=[0.08, 0.05, 0.03, 0.01], help="the tip over the bench's top, m, one per environment")
    parser.add_argument("--wipe_force_n", type=float, nargs="+", default=[0.0], help="the policy's force command, down, through the stroke; each height runs each force")
    parser.add_argument("--wipe_one_way", action="store_true", help="press right to left, lift, come back in the air, press again, as the console's force wipes do")
    parser.add_argument("--wipe_radius", type=float, default=0.52)
    parser.add_argument("--wipe_yaw_deg", type=float, default=25.0, help="a stroke runs from minus this to plus this")
    parser.add_argument("--sweep_s", type=float, default=6.0)
    parser.add_argument("--wipe_walk_first", action="store_true", help="walk on the sticks first, as `--reach_walk_first` does: is the hand's height the same after it?")
    parser.add_argument("--start_back_m", type=float, default=0.0, help="start this far behind the origin, so a walk ends where an operator would have stopped it")
    parser.add_argument("--bench_top_m", type=float, default=0.40)
    parser.add_argument("--bench", action="store_true", help="put the bench in the scene: without it the goals are the same and nothing touches")
    parser.add_argument("--bench_edge_m", type=float, default=0.45, help="the bench's near edge ahead of the base's origin")
    parser.add_argument("--bench_depth_m", type=float, default=0.35)
    parser.add_argument("--pad_n_per_m", type=float, default=2000.0, help="the cloth, as the bench's compliant contact; 0 is rigid. Invented")
    parser.add_argument("--floor_grip", type=float, default=None, help="the floor's friction coefficient; the task's is 1")
    parser.add_argument("--radius", type=float, nargs="+", default=[0.35, 0.45, 0.55])
    parser.add_argument("--pitch_deg", type=float, nargs="+", default=[0.0, 15.0, 36.0, 50.0])
    parser.add_argument("--yaw_deg", type=float, nargs="+", default=[-40.0, -20.0, 0.0, 20.0, 40.0])
    parser.add_argument("--settle_s", type=float, default=3.0, help="on the start goal, as the dog's runs settled, before the plan moves")
    parser.add_argument("--move_s", type=float, default=3.0, help="a goal's slide; training's are 1-3 s")
    parser.add_argument("--hold_goal_s", type=float, default=4.0)
    parser.add_argument("--walk_s", type=float, default=6.0)
    parser.add_argument("--tail_s", type=float, default=3.0)
    parser.add_argument("--leg_friction", type=float, default=1.0)
    parser.add_argument("--leg_answer", type=float, default=None)
    parser.add_argument("--real_stance", action="store_true", help="hand over from the dog's own stance, not the simulator's")
    parser.add_argument("--stop_at_s", type=float, default=None, help="end the policy here, as a stop on the dog would, and watch what follows")
    parser.add_argument("--stop_kind", choices=("soft", "damping"), default="soft",
                        help="soft: `press_live._arm_stop`: the arm held, the legs blended to the stand-up gains and the default stance, then the fold by home, "
                             "tuck and fold. damping: `abort`: kp 0, kd 3 on the legs, the arm held out")
    parser.add_argument("--hold_s", type=float, default=1.0)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--run_name", default="plan")
    parser.add_argument("--output", default=str(ROOT / "logs/unifp_demos"))
    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    sys.path.insert(0, str(ROOT))
    import torch  # noqa: E402
    from datetime import datetime, timezone  # noqa: E402

    from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR  # noqa: E402
    from weld import build_welded_robot_usd  # noqa: E402
    import motor_model  # noqa: E402
    from unifp_isaaclab import interface, robot as robot_mod  # noqa: E402
    from unifp_train import arm_link, leg_actuators  # noqa: E402
    from unifp_train.env import Go2D1PosForceEnv  # noqa: E402
    from unifp_train.env_cfg import Go2D1PosForceEnvCfg, shrink_physx  # noqa: E402
    from unifp_train.eval import EvaluablePolicy  # noqa: E402

    plans = {"far": far_plans, "walk": walk_plans, "tour": tour_plans, "wipe": wipe_plans}[args.plan](args)
    count, dt = len(plans), interface.POLICY_DT
    longest = max(sum(m + h for _, m, h in p["goals"]) + sum(s for s, _ in p["walk"]) + sum(s for s, _ in p.get("walk_first", [])) + (args.settle_s if p.get("walk_first") else 0.0) for p in plans)
    T = int(round((args.settle_s + longest) / dt))
    tables = [schedule(p, args.settle_s, dt, T) for p in plans]
    if args.stop_at_s is not None:
        T = min(T, int(round(args.stop_at_s / dt)))

    robot_usd = build_welded_robot_usd(
        go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd", d1_urdf_path=str(ROOT / "d1_arm/d1.urdf"),
        out_usd_path=str(ROOT / "generated/go2_d1_payload.usd"), arm_mass_kg=3.152, payload_physics=True).usd_path
    cfg = Go2D1PosForceEnvCfg()
    cfg.scene.num_envs = count
    cfg.seed = args.seed
    cfg.robot = robot_mod.make_robot_cfg(robot_usd, prim_path="/World/envs/env_.*/Robot")
    cfg.robot_usd = robot_usd
    cfg.force_start_step = 10 ** 9
    cfg.tool_body, cfg.tool_offset_m = interface.TOOL_BODY, interface.TOOL_OFFSET_M
    cfg.episode_length_s = 10 ** 4
    shrink_physx(cfg.sim)
    leg_actuators.swap_legs(cfg.robot, "unitree_geared", 4, 0.0002, 0.0007, 3)
    if args.leg_friction:
        cfg.leg_friction_range = (args.leg_friction, args.leg_friction)
    if args.leg_answer is not None:
        cfg.leg_answer_range = (args.leg_answer, args.leg_answer)
    if args.floor_grip is not None:
        import isaaclab.sim as sim_utils
        from isaaclab.assets import AssetBaseCfg

        floor = sim_utils.RigidBodyMaterialCfg(static_friction=args.floor_grip, dynamic_friction=args.floor_grip, restitution=0.0)
        cfg.sim.physics_material = floor
        cfg.terrain = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg(physics_material=floor))
    arm_link.swap_arm(cfg.robot)
    cfg.arm_servo = True
    cfg.arm_link = {"locked": True, "feedback_hz": motor_model.D1_FEEDBACK_HZ, "quantum_deg": motor_model.D1_FEEDBACK_QUANTUM_DEG, "planner": True,
                    "plan": motor_model.arm_trajectory("locked"), "servo_lag_s": motor_model.D1_LOCKED_SERVO_LAG_S,
                    "report_age_s": motor_model.D1_LOCKED_REPORT_AGE_S, "follower": True}
    env_class = Go2D1PosForceEnv
    if args.bench:
        import isaaclab.sim as sim_utils
        from isaaclab.sensors import ContactSensor, ContactSensorCfg

        class BenchEnv(Go2D1PosForceEnv):
            """The scene with a bench across the dog's way: a fixed box, its contact compliant as a folded cloth is."""

            def _setup_scene(self):
                soft = {"compliant_contact_stiffness": args.pad_n_per_m, "compliant_contact_damping": args.pad_n_per_m / 20.0} if args.pad_n_per_m else {}
                bench = sim_utils.CuboidCfg(
                    size=(args.bench_depth_m, 1.6, args.bench_top_m), collision_props=sim_utils.CollisionPropertiesCfg(),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True), activate_contact_sensors=True,
                    physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.5, **soft))
                bench.func("/World/envs/env_0/Bench", bench, translation=(args.bench_edge_m + args.bench_depth_m / 2, 0.0, args.bench_top_m / 2))
                self._bench_sensor = ContactSensor(ContactSensorCfg(prim_path="/World/envs/env_.*/Bench", update_period=0.0, history_length=1))
                self.scene.sensors["bench"] = self._bench_sensor
                super()._setup_scene()

        env_class = BenchEnv
    env = env_class(cfg)
    env.reset()
    device = env.device
    policy = EvaluablePolicy(args.checkpoint, device=str(device))

    goal_table = torch.tensor([t[0] for t in tables], dtype=torch.float32, device=device).permute(1, 0, 2).contiguous()   # (T, N, 3)
    vel_table = torch.tensor([t[1] for t in tables], dtype=torch.float32, device=device).permute(1, 0, 2).contiguous()
    tick = {"k": 0}

    def goal_now():
        # `current` too: `_goal_world`, which the hand is scored against, reads it, not what `step` returned.
        env._goals.current = goal_table[min(tick["k"], T - 1)]
        return env._goals.current

    env._goals.step = goal_now          # type: ignore[assignment]
    env._resample_velocity_commands = lambda ids: None
    start = torch.tensor(START, dtype=torch.float32, device=device).expand(count, 3).clone()
    for buffer in ("current", "start", "goal"):
        setattr(env._goals, buffer, start.clone())

    hold = torch.zeros(count, interface.NUM_ACTIONS, device=device)
    zeros3 = torch.zeros(count, 3, device=device)
    with torch.no_grad():
        for _ in range(int(args.hold_s / dt)):
            env._commands[:, :3] = zeros3
            env.step(hold)
    if args.real_stance:
        order = env._order[:12]
        joint_pos, joint_vel = env._robot.data.joint_pos.clone(), torch.zeros_like(env._robot.data.joint_pos)
        joint_pos[:, order] = torch.tensor(interface.DEFAULT_DOF_POS[:12], device=device) + torch.tensor(REAL_LEG_OFFSETS, device=device)
        root = env._robot.data.root_state_w.clone()
        root[:, :2] = env.scene.env_origins[:, :2]
        root[:, 0] -= args.start_back_m
        root[:, 2] = env.scene.env_origins[:, 2] + REAL_BASE_M
        half_p, half_r = math.radians(REAL_PITCH_DEG) / 2, math.radians(REAL_ROLL_DEG) / 2
        root[:, 3:7] = torch.tensor([math.cos(half_r) * math.cos(half_p), math.sin(half_r) * math.cos(half_p),
                                     math.cos(half_r) * math.sin(half_p), -math.sin(half_r) * math.sin(half_p)], device=device)
        root[:, 7:] = 0.0
        env._robot.write_root_state_to_sim(root)
        env._robot.write_joint_state_to_sim(joint_pos, joint_vel)
        env._robot.set_joint_position_target(joint_pos)
        hold[:, :12] = torch.tensor(REAL_LEG_OFFSETS, device=device) / interface.ACTION_SCALE
        with torch.no_grad():
            for _ in range(5):
                env._commands[:, :3] = zeros3
                env.step(hold)
    env._actions.zero_()
    env._last_actions.zero_()
    env._last_dof_vel[:] = env._dof()[1]
    env._actor_history.reset(None)
    env._critic_history.reset(None)
    env._gait_phase.zero_()
    obs = env._get_observations()
    leg_now = env._dof()[0][0, :12] - torch.tensor(interface.DEFAULT_DOF_POS[:12], device=device)
    base_z = float(env._robot.data.root_pos_w[0, 2] - env.scene.env_origins[0, 2])
    print(f"[handover] leg q - default {[round(float(v), 3) for v in leg_now]} base {base_z:.3f}", flush=True)

    names = list(env._contact_sensor.body_names)
    body_sensor = [i for i, n in enumerate(names) if n in ("base", "Head_upper", "Head_lower") or n.endswith(("_hip", "_thigh"))]
    spawn_xy = env._robot.data.root_pos_w[:, :2].clone()
    z = lambda *shape: torch.zeros(T, count, *shape, device=device)  # noqa: E731
    err, q_arm, reported, asked, leg_off = z(), z(6), z(6), z(6), z()
    contact, tilt, base_h, drift = z(), z(2), z(), z()
    speed = z(3)
    tip, press, base_pose = z(3), z(), z(3)
    arm_order = env._order[12:18]
    fell_at = torch.full((count,), -1, dtype=torch.long, device=device)
    decoder = None
    if args.plan == "wipe":
        import numpy as np  # noqa: E402
        from demos.combiner_key.robot import leg_policy  # noqa: E402
        decoder = leg_policy.LegPolicy(str(ROOT / "checkpoints/reach_force_sticky_model_66600.npz"))
    est_force = z(3)
    walked_s = sum(s for s, _ in plans[0].get("walk_first", [])) + (args.settle_s if plans[0].get("walk_first") else 0.0)
    press_on = int(round((args.settle_s + walked_s + args.move_s) / dt))
    press_off = int(round((args.settle_s + walked_s + sum(m + h for _, m, h in plans[0]["goals"][:-1])) / dt)) if plans[0].get("goals") else 0
    told = torch.zeros(count, 3, device=device)
    told[:, 2] = -torch.tensor([p.get("force_n", 0.0) for p in plans], dtype=torch.float32, device=device)
    with torch.no_grad():
        for step in range(T):
            tick["k"] = step
            env._commands[:, :3] = vel_table[step]
            env._commands[:, interface.CMD_EE_FORCE] = told if press_on <= step < press_off else 0.0
            action = policy.act(obs["policy"])
            obs, _, terminated, truncated, _ = env.step(action)
            fresh = (terminated | truncated) & (fell_at < 0)
            fell_at[fresh] = step
            err[step] = torch.norm(env._tip_pos() - env._goal_world(), dim=1)
            q_arm[step] = env._dof()[0][:, 12:18]
            reported[step] = env._arm_link.reported
            asked[step] = env._joint_targets[:, 12:18]
            leg_off[step] = env._actions[:, :12].abs().amax(dim=1) * interface.ACTION_SCALE
            contact[step] = env._contact_sensor.data.net_forces_w.norm(dim=-1)[:, body_sensor].amax(dim=1)
            tilt[step] = torch.rad2deg(interface.body_roll_pitch(env._robot.data.root_quat_w))
            base_h[step] = env._robot.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2]
            drift[step] = torch.norm(env._robot.data.root_pos_w[:, :2] - spawn_xy, dim=1)
            speed[step, :, :2] = env._robot.data.root_lin_vel_b[:, :2]
            speed[step, :, 2] = env._robot.data.root_ang_vel_b[:, 2]
            tip[step] = env._tip_pos()
            if decoder is not None:
                # The adaptation decoder's external force at the tool, yaw frame, N: what the dog logs in `estimate[6:9]`.
                est_force[step] = torch.as_tensor(decoder.estimate(obs["policy"].cpu().numpy())[:, 6:9] / decoder.meta["obs_scale_ee_force"], dtype=torch.float32, device=device)
            q_w = env._robot.data.root_quat_w
            base_pose[step, :, :2] = env._robot.data.root_pos_w[:, :2] - env.scene.env_origins[:, :2]
            base_pose[step, :, 2] = torch.rad2deg(torch.atan2(2 * (q_w[:, 0] * q_w[:, 3] + q_w[:, 1] * q_w[:, 2]), 1 - 2 * (q_w[:, 2] ** 2 + q_w[:, 3] ** 2)))
            if args.bench:
                press[step] = env._bench_sensor.data.net_forces_w[:, 0, 2].abs()

    stop_records = None
    if args.stop_at_s is not None:
        stop_records = stop_phase(args, env, plans, device, dt, body_sensor, torch, interface, fell_at)

    grace = int(GRACE_S / dt)
    cycle = max(1, round(0.111 / dt))
    lead = torch.rad2deg((asked - reported).abs()).amax(dim=2)
    # `ArmWatch.check` runs every tick, so its step is one tick's. Over a command cycle it is shown for information: the arm sees that one.
    step_deg = torch.rad2deg((asked[1:] - asked[:-1]).abs()).amax(dim=2)
    cycle_deg = torch.rad2deg((asked[cycle:] - asked[:-cycle]).abs()).amax(dim=2)
    records = []
    for i, plan in enumerate(plans):
        end = int(fell_at[i]) if int(fell_at[i]) >= 0 else T
        over, longest_over = 0, 0
        for v in lead[grace:end, i].tolist():
            over = over + 1 if v > LEAD_DEG else 0
            longest_over = max(longest_over, over)
        stops = []
        if int(fell_at[i]) >= 0:
            stops.append(f"fell at {fell_at[i] * dt:.1f} s")
        if longest_over * dt >= LEAD_HOLD_S:
            stops.append("lead")
        if end > grace and float(lead[grace:end, i].max()) > LEAD_HARD_DEG:
            stops.append("hard lead")
        if end > grace and float(step_deg[grace:max(end - 1, grace + 1), i].max()) > STEP_DEG:
            stops.append("step")
        if float(leg_off[:grace, i].max()) > LEG_FIRST_RAD or (end > grace and float(leg_off[grace:end, i].max()) > LEG_AFTER_RAD):
            stops.append("legs")
        if float(tilt[:end, i].abs().max()) > TILT_DEG:
            stops.append("tilt")
        record = {"name": plan["name"], "stops": stops, "lead_deg": float(lead[grace:end, i].max()) if end > grace else None, "lead_over_25_s": longest_over * dt,
                  "step_deg": float(step_deg[grace:max(end - 1, grace + 1), i].max()) if end > grace + 1 else None,
                  "cycle_step_deg": float(cycle_deg[grace:max(end - cycle, grace + 1), i].max()) if end > grace + cycle else None,
                  "leg_first_rad": float(leg_off[:grace, i].max()), "leg_after_rad": float(leg_off[grace:end, i].max()) if end > grace else None,
                  "tilt_deg": float(tilt[:end, i].abs().max()), "contact_n": float(contact[:end, i].max()), "base_min_m": float(base_h[:end, i].min()), "windows": []}
        for label, a, b, g in tables[i][2]:
            a2 = a if label == "sweep" else a + (b - a) // 2
            if a2 >= end or b <= a:
                continue
            b2 = min(b, end)
            w = {"label": label, "goal": list(g), "error_cm": float(err[a2:b2, i].mean()) * 100, "arm_deg": [round(float(v), 1) for v in torch.rad2deg(q_arm[b2 - 1, i])],
                 "pitch_deg": float(tilt[a2:b2, i, 1].mean()), "roll_deg": float(tilt[a2:b2, i, 0].mean()), "speed": [float(v) for v in speed[a2:b2, i].mean(dim=0)], "drift_m": float(drift[b2 - 1, i])}
            if args.plan == "wipe":
                over = tip[a2:b2, i, 2] - args.bench_top_m
                w.update({"tip_over_bench_cm": [round(float(v) * 100, 1) for v in (over.min(), over.mean(), over.max())], "tip_x_m": round(float(tip[a2:b2, i, 0].mean()), 3),
                          "press_n": [round(float(v), 1) for v in (press[a2:b2, i].mean(), press[a2:b2, i].max())],
                          "touching": round(float((press[a2:b2, i] > 0.5).float().mean()), 2),
                          "est_force_z_n": [round(float(v), 1) for v in (est_force[a2:b2, i, 2].mean(), est_force[a2:b2, i, 2].std())],
                          "base_x_y_yaw": [round(float(v), 3) for v in base_pose[b2 - 1, i]]})
            record["windows"].append(w)
        records.append(record)

    if args.plan == "wipe":
        print(f"wipe: r {args.wipe_radius} yaw +-{args.wipe_yaw_deg} bench {'in' if args.bench else 'absent'} top {args.bench_top_m} edge {args.bench_edge_m} pad {args.pad_n_per_m} N/m")
        for r in records:
            for w in r["windows"]:
                print(f"  {r['name']} {w['label'][:10]:10s} err {w['error_cm']:.1f} tip over bench min/mean/max {w['tip_over_bench_cm']} cm x {w['tip_x_m']} press mean/max {w['press_n']} N touching {w['touching']} arm {w['arm_deg']} pitch {w['pitch_deg']:.1f} est z {w['est_force_z_n']} base {w['base_x_y_yaw']}")
    print(f"{args.plan}, friction {args.leg_friction}, {'the dog stance' if args.real_stance else 'the simulator stance'}, handover base {base_z:.3f}")
    print(f"{'plan':18s} lead  over25 step | legs 1st after | tilt  contact base | windows (cm off; speed for walking) | stops")
    for r in records:
        ws = " ".join((f"{w['label'][:4]} {w['error_cm']:.1f}" + (f" v {w['speed'][0]:+.2f} {w['speed'][1]:+.2f} {w['speed'][2]:+.2f}" if w["label"] == "walk" else "")) for w in r["windows"])
        print(f"{r['name']:18s} {r['lead_deg'] or 0:4.0f} {r['lead_over_25_s']:5.2f} {r['step_deg'] or 0:5.1f} | {r['leg_first_rad']:.2f} {r['leg_after_rad'] or 0:.2f} | {r['tilt_deg']:4.1f} {r['contact_n']:6.1f} {r['base_min_m']:.3f} | {ws} | {','.join(r['stops'])}", flush=True)
    if stop_records is not None:
        print(f"{args.stop_kind} stop at {args.stop_at_s} s")
        print(f"{'plan':18s} tilt peak  tilt rest | base min, end | hand lowest m | contact N | seconds to the end")
        for r in stop_records:
            print(f"  {r['name']} press peak {r['press_n']:.0f} N, arm torque peak {r['arm_torque_nm']} N·m")
            print(f"{r['name']:18s} {r['tilt_peak_deg']:6.1f} {r['tilt_end_deg']:8.1f}   | {r['base_min_m']:.3f} {r['base_end_m']:.3f}  | {r['hand_lowest_m']:.3f}   | {r['contact_n']:6.1f} | {r['seconds']:.1f}", flush=True)
    clean = [r for r in records if not r["stops"]]
    print(f"{len(clean)} of {len(records)} clear of every stop")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    run_dir = Path(args.output).resolve() / f"{stamp}_probe_seed{args.seed}_{args.run_name}"
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "run_plan.json").write_text(json.dumps({"records": records, "stop": stop_records}, indent=1))
    run_json = {"mode": "run_plan_probe", "status": "finished", "seed": args.seed, "checkpoint": os.path.abspath(args.checkpoint), "checkpoint_source": policy.source,
                "checkpoint_iteration": policy.iteration, "plan": args.plan, "steps": T, "plans": count, "clear": len(clean), "leg_friction": args.leg_friction, "floor_grip": args.floor_grip, "real_stance": args.real_stance,
                "handover_base_m": base_z, "stop_at_s": args.stop_at_s, "stop_kind": args.stop_kind, "tool_point": "fingertip", "forces": "off", "records_file": "run_plan.json", "command": " ".join(sys.argv)}
    (run_dir / "run.json").write_text(json.dumps(run_json, indent=2))
    print(f"written to {run_dir}", flush=True)
    env.close()
    simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
