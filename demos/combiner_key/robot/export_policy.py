#!/usr/bin/env python3
"""Export a leg-driving policy for the dog: its weights and its interface's constants in one `.npz` (on the PC). A
12-output leg policy, or an 18-output reach policy, whose legs `press_live.py` can drive while the arm stays folded,
or (`--task insert`) the key insertion's 18-output whole-body policy with its task layer's and IK's constants, or
(`--task mechanism`) a held-mechanism (T1, S4b) whole-body policy with its force law's, plan's and goal's constants.

    python demos/combiner_key/robot/export_policy.py --checkpoint <model.pt> --variant force --out checkpoints/<name>.npz
    python demos/combiner_key/robot/export_policy.py --task insert --checkpoint logs/unifp_train/keyinsert/ki6_force_8200.pt \
        --variant force --train_run logs/unifp_train/20261112T205503_train_seed41_ki6_force \
        --out checkpoints/insert_force_ki6_8200.npz
    python demos/combiner_key/robot/export_policy.py --task mechanism --checkpoint logs/unifp_train/s4/s4b_A_s91_68200.pt \
        --variant force --train_run logs/unifp_train/20261118T132002_train_seed91_s4b_A_s91 --out <name>.npz

The dog runs it with `leg_policy.LegPolicy` (numpy only), an insertion with `insert_policy.InsertRunner` around it, a
mechanism with `mech_layer.MechRunner`. The export reads every constant from the training code (`unifp_isaaclab.interface`,
`unifp_train.leg_cfg`, `unifp_train.observations`; for the insertion `unifp_train.keylock`, `arm_kin`, `keyhold` and
`keyinsert_env`'s source; for a mechanism `mech_cfg`, `hook_cfg`, `arm_kin` and `mech_env`'s source, the run's json and,
for S4c's grasp gate, its env.yaml),
so the dog's side has none of its own, and refuses to write unless the numpy network reproduces the checkpoint's own
actions and estimates (`unifp_train.eval.EvaluablePolicy`) on random stacks to `--tolerance`.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))


def linear_layers(module) -> list[tuple[np.ndarray, np.ndarray]]:
    """The (weight, bias) of an MLP's linear layers, checking it is Linear (ELU Linear)* as `leg_policy` runs it."""
    import torch.nn as nn

    # In order and with repeats: rsl-rl's MLP registers one activation module between every pair of layers, which
    # `module.modules()` would list once.
    def flat(m):
        kids = list(m._modules.values())
        return [m] if not kids else [leaf for kid in kids for leaf in flat(kid)]

    children = flat(module)
    kinds = ["L" if isinstance(m, nn.Linear) else "E" if isinstance(m, nn.ELU) else type(m).__name__ for m in children]
    pattern = "".join(kinds)
    if not pattern or pattern != "LE" * (pattern.count("L") - 1) + "L":
        raise ValueError(f"unexpected MLP layout {kinds}; leg_policy runs Linear (ELU Linear)*")
    return [(m.weight.detach().cpu().numpy(), m.bias.detach().cpu().numpy()) for m in children if isinstance(m, nn.Linear)]


def reach_goals() -> dict:
    """A reach policy's goals as (radius, pitch, yaw) about the goal sphere's centre: the one every training episode
    opens on, and the folded hand's own, with the base at `leg_cfg.SETTLED_BASE_HEIGHT_M` (the sphere's centre is
    measured from the ground under the base, `unifp_isaaclab.task.goal_sphere_center`)."""
    import d1_ik
    from demos.combiner_key.robot import leg_policy
    from mount import ARM_MOUNT_POS_M
    from unifp_isaaclab import interface
    from unifp_train import leg_cfg

    tip, _ = d1_ik.tool_pose(d1_ik.load_urdf()[0], np.asarray(leg_cfg.ARM_REST_POSE))
    hand = np.asarray(ARM_MOUNT_POS_M) + tip + np.array([0.0, 0.0, leg_cfg.SETTLED_BASE_HEIGHT_M])
    folded = leg_policy.cart_to_sphere(hand - np.asarray(interface.EE_GOAL_CENTER_OFFSET))
    return {"init_start": list(interface.EE_GOAL_INIT_START), "folded": [float(v) for v in folded]}


def meta_for(variant: str, checkpoint: Path, iteration, num_policy_actions: int, layers: dict,
             task: str | None = None, train_run: Path | None = None, mech_law: dict | None = None) -> dict:
    from demos.combiner_key import classic_stance
    from unifp_isaaclab import interface
    from unifp_train import leg_cfg, observations, task_cfg

    insert = task == "insert"
    mech = task == "mechanism"
    reach = num_policy_actions == interface.NUM_ACTIONS and not insert and not mech
    return {
        "format": "d1training-leg-policy-1",
        # "reach": 18 outputs, the legs' first; the robot programs here send the legs and never the arm's six.
        # "insert": 18 outputs, the arm's six a residual on the task layer's IK reference (`insert_policy`).
        # "mechanism": 18 outputs, UniFP's whole-body actions; a task layer writes the goal and force (`mech_layer`).
        "task": "insert" if insert else "mechanism" if mech else "reach" if reach else "leg",
        **({"reach_goals": reach_goals(), "arm_joint_limits": [list(l) for l in task_cfg.JOINT_POS_LIMITS[12:18]]}
           if reach else {}),
        **({"insert": insert_meta(checkpoint, variant, train_run),
            "arm_joint_limits": [list(l) for l in task_cfg.JOINT_POS_LIMITS[12:18]]} if insert else {}),
        **({"mechanism": mech_meta(checkpoint, variant, train_run, mech_law),
            "arm_joint_limits": [list(l) for l in task_cfg.JOINT_POS_LIMITS[12:18]]} if mech else {}),
        "variant": variant,
        "checkpoint": str(checkpoint),
        "checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
        "checkpoint_iteration": iteration,
        "layers": layers,
        "dof_names": list(interface.DOF_NAMES),
        "num_actions": interface.NUM_ACTIONS,
        "num_leg_actions": min(num_policy_actions, 12),
        "default_dof_pos": list(interface.DEFAULT_DOF_POS),
        "obs_scale_ang_vel": interface.OBS_SCALE_ANG_VEL,
        "obs_scale_dof_pos": interface.OBS_SCALE_DOF_POS,
        "obs_scale_dof_vel": interface.OBS_SCALE_DOF_VEL,
        "commands_scale": list(interface.COMMANDS_SCALE),
        "num_commands": interface.NUM_COMMANDS,
        "cmd_ee_radius": interface.CMD_EE_RADIUS,
        "frame_stack": interface.FRAME_STACK,
        "num_single_obs": interface.NUM_SINGLE_OBS,
        "clip_observations": interface.CLIP_OBSERVATIONS,
        "clip_actions": interface.CLIP_ACTIONS,
        "action_scale": interface.ACTION_SCALE,
        "policy_dt": interface.POLICY_DT,
        "leg_kp": interface.P_GAINS[0],
        "leg_kd": interface.D_GAINS[0],
        "leg_torque_limits": list(interface.TORQUE_LIMITS[:12]),
        "goal_centre": list(leg_cfg.GOAL_CENTRE),
        "key_down_goal": list(leg_cfg.KEY_DOWN_GOAL),
        "arm_rest_pose": list(leg_cfg.ARM_REST_POSE),
        "settled_leg_pos": list(leg_cfg.SETTLED_LEG_POS),
        "settled_base_height_m": leg_cfg.SETTLED_BASE_HEIGHT_M,
        "ee_sphere_scale": list(observations.EE_SPHERE_SCALE),
        "obs_scale_ee_force": observations.OBS_SCALE_EE_FORCE,
        "obs_scale_base_force": observations.OBS_SCALE_BASE_FORCE,
        "kinematics": {"hip_x": classic_stance.HIP_X, "hip_y": classic_stance.HIP_Y, "thigh_y": classic_stance.THIGH_Y,
                       "link_m": classic_stance.LINK_M, "foot_sphere_x": classic_stance.FOOT_SPHERE_X,
                       "foot_radius_m": classic_stance.FOOT_RADIUS_M},
        "trained_on": trained_on(checkpoint, train_run),
        "start_gate": start_gate(checkpoint, train_run),
    }


def run_record_path(checkpoint: Path, train_run: Path | None = None) -> Path:
    """The training run's run.json: `train_run`'s if given, else the one beside the checkpoint."""
    return (Path(train_run) if train_run is not None else checkpoint.parent) / "run.json"


def trained_on(checkpoint: Path, train_run: Path | None = None) -> dict:
    """What the checkpoint's own training run recorded about its legs, key and starts (its run.json), if it is there."""
    record = run_record_path(checkpoint, train_run)
    if not record.exists():
        return {"note": "no run.json beside the checkpoint"}
    leg = json.loads(record.read_text()).get("leg") or {}
    keys = ("variant", "leg_actuator", "leg_actuator_model", "leg_armature_range", "knee_armature_factor", "key_turn_deg",
            "key_overdrive_deg", "key_hold_s", "key_lock_range_nm", "key_detent_range_nm", "key_max_force_n",
            "press_limit_n", "press_excess_weight", "spawn_states", "spawn_states_fraction", "leg_friction_range",
            "knee_friction_factor", "press_up_cap_n", "obs_noise", "weights")
    out = {k: leg[k] for k in keys if k in leg}
    out["run"] = record.parent.name
    return out


#: `keyinsert_env`'s module constants the insertion runner needs. That module imports Isaac Lab, so they are read off its
#: source (`source_constants`), not imported.
INSERT_ENV_CONSTANTS = ("START_STEPS", "LOOP_GAIN", "LOOP_MAX_M", "LIMIT_SHARE", "POLISH_ITERATIONS", "POLISH_DAMPING",
                        "POLISH_TOL_M", "END_AFTER_DONE_S", "INSERT_OBS_SCALE")
#: What `keyinsert_env._ik` writes as literals: `track`'s iterations, the polish's finite difference and the rotation
#: tolerance of a solution (`tests/test_insert_policy.py` checks them against its source).
IK_LITERALS = {"track_iterations": 10, "polish_eps": 1e-4, "polish_rot_tol_rad": 0.01}


def source_constants(path: Path, names, namespace: dict) -> dict:
    """Module-level `NAME = <expression>` assignments of `path`, evaluated in `namespace` (the modules they refer to)."""
    tree = ast.parse(Path(path).read_text())
    out = {}
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id in names:
                    out[target.id] = eval(compile(ast.Expression(node.value), str(path), "eval"), dict(namespace))
    missing = [n for n in names if n not in out]
    if missing:
        raise ValueError(f"{path} does not assign {missing}")
    return out


def arm_constants(limit_share: float, wrist_payload: bool) -> dict:
    """The D1's chain (`arm_kin.ArmKinematics`, float64), its limits at `limit_share` of the hard range, its PD stiffness
    and the gravity model's links (`keyhold.ArmGravity`), as `insert_policy` builds them."""
    import torch

    from unifp_isaaclab import interface
    from unifp_train import arm_kin, keyhold

    kin = arm_kin.ArmKinematics("cpu", torch.float64)
    lst = lambda t: t.detach().cpu().numpy().tolist()
    mid, half = 0.5 * (kin.hard_lower + kin.hard_upper), 0.5 * (kin.hard_upper - kin.hard_lower)
    gravity = keyhold.ArmGravity(kin, wrist_payload=wrist_payload)
    return {
        "arm": {"hard_lower": lst(kin.hard_lower), "hard_upper": lst(kin.hard_upper), "limit_share": limit_share,
                "lower": lst(mid - limit_share * half), "upper": lst(mid + limit_share * half),
                "kp": list(interface.P_GAINS[12:18])},
        "kinematics": {"xyz": [lst(v) for v in kin.xyz], "rpy": [lst(v) for v in kin.rpy],
                       "axis": [lst(v) for v in kin.axis], "mount": lst(kin.mount),
                       "hard_lower": lst(kin.hard_lower), "hard_upper": lst(kin.hard_upper),
                       "wrist_local": lst(kin.wrist_local), "tool_from_wrist": lst(kin.tool_from_wrist),
                       "jaw_local": lst(kin.jaw_local)},
        "gravity": {"mass": lst(gravity.mass), "com": lst(gravity.com), "wrist_payload": wrist_payload},
    }


def insert_meta(checkpoint: Path, variant: str, train_run: Path | None) -> dict:
    """Everything `insert_policy.InsertRunner` needs beside the network, from the training code and the training run."""
    from demos.combiner_key.robot import insert_policy, leg_policy
    from unifp_train import keylock, leg_cfg

    env = source_constants(REPO / "unifp_train" / "keyinsert_env.py", INSERT_ENV_CONSTANTS,
                           {"keylock": keylock, "leg_cfg": leg_cfg, "math": math})
    env["IK_EVERY_STEPS"] = leg_cfg.KEYHOLD_IK_EVERY_STEPS
    record_path = run_record_path(checkpoint, train_run)
    record = json.loads(record_path.read_text()) if record_path.exists() else {}
    args = record.get("arguments") or {}
    if not args:
        raise ValueError(f"an insertion export needs its training run's run.json (--train_run): none at {record_path}")
    if not args.get("key_insert"):
        raise ValueError(f"{record_path} is not a key-insertion training run")
    if args.get("leg_variant") and args["leg_variant"] != variant:
        raise ValueError(f"--variant {variant}, but {record_path.parent.name} trained {args['leg_variant']}")
    checks = {}
    if train_run is not None:
        # The checkpoint is the run's own model file (by content), not a namesake.
        it = record.get("final_checkpoint")
        sha = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
        same = [p.name for p in sorted(Path(train_run).glob("model_*.pt")) if p.stat().st_size == checkpoint.stat().st_size
                and hashlib.sha256(p.read_bytes()).hexdigest() == sha]
        if not same:
            raise ValueError(f"{checkpoint} is none of {train_run}'s model files")
        checks = {"checkpoint_in_run": same, "run_final_checkpoint": it}
    # `cfg.wrist_payload` is `--payload physical`'s for the key hold (run_unifp_train.py); the sag is off with the stiff
    # servo (`keyhold_env._sag`: `--arm_locked` sets `arm_servo`), as for every insertion run through the D1's link.
    wrist_payload = args.get("payload") == "physical"
    seat_test = args.get("insert_seat_test") or "old"
    press_range = tuple(args["insert_press_n"]) if args.get("insert_press_n") else tuple(keylock.PRESS_N)
    return {
        "force_aware": variant == "force",
        "hold_arm": args.get("hold_arm"),
        "commands": {"foot_goal": [3, 6], "miss": [6, 9], "press": [9, 12], "miss_obs_scale": float(env["INSERT_OBS_SCALE"]),
                     "foot_goal_sphere": [float(v) for v in leg_policy.cart_to_sphere(
                         np.asarray(leg_cfg.KEY_DOWN_GOAL) - np.asarray(leg_cfg.GOAL_CENTRE))],
                     "miss_is": "the jaw centre's goal less its measured place, base frame, m",
                     "press_is": "the layer's force along the estimated axis, base yaw frame, N (force-aware only)"},
        "layer": {"constants": {name: getattr(keylock, name) for name in insert_policy.LAYER_CONSTANTS},
                  "seat_test": seat_test, "light_probe": bool(args.get("insert_light_probe", False)),
                  "press_n_training": list(press_range), "press_n_eval": keylock.EVAL_PRESS_N,
                  "start_off_m_training": list(keylock.START_OFF_M)},
        "env": env,
        "ik": {"nominal": list(leg_cfg.KEYHOLD_IK_NOMINAL), "seeds": [list(s) for s in leg_cfg.KEYHOLD_IK_SEEDS],
               "polish_iterations": int(env["POLISH_ITERATIONS"]), "polish_damping": float(env["POLISH_DAMPING"]),
               "polish_tol_m": float(env["POLISH_TOL_M"]), **IK_LITERALS},
        "key": {"name": "v4", "tip_from_jaw_m": keylock.TIP_FROM_JAW_M, "offset_m": [0.0, 0.0],
                "key_back_link6_z_m": keylock.KEY_BACK_LINK6_Z_M, "jaw_link6_z_m": keylock.JAW_LINK6_Z_M,
                "key_length_m": keylock.KEY_LENGTH_M, "shaft_r_m": keylock.SHAFT_R_M,
                "ward_half_len_m": keylock.WARD_HALF_LEN_M, "ward_half_width_m": keylock.WARD_HALF_WIDTH_M,
                "ward_angle_rad": keylock.WARD_ANGLE_RAD, "grip_roll": leg_cfg.KEYHOLD_GRIP_ROLL},
        "lock": {"keyway_depth_m": keylock.KEYWAY_DEPTH_M, "face_off_door_m": keylock.LOCK_FACE_OFF_DOOR_M,
                 "play_m": list(keylock.PLAY_M), "damage_n": keylock.DAMAGE_N, "press_limit_n": keylock.PRESS_LIMIT_N},
        **arm_constants(float(env["LIMIT_SHARE"]), wrist_payload),
        "sag_compensation": not bool(args.get("arm_servo", False)),
        "trained_with": {**{k: v for k, v in args.items() if k.startswith("insert_")},
                         **{k: args.get(k) for k in ("leg_variant", "hold_arm", "arm_locked", "arm_follower", "arm_servo",
                                                     "leg_friction_range", "leg_delay_steps", "leg_delay_min_steps",
                                                     "spawn_states", "seed", "resume_from")},
                         "arm_link": record.get("arm_link"), "run": record_path.parent.name, **checks},
    }


def function_default(path: Path, function: str, argument: str):
    """The default of `argument` in the (first) `def function(...)` of `path`, read off its source (a literal)."""
    for node in ast.walk(ast.parse(Path(path).read_text())):
        if isinstance(node, ast.FunctionDef) and node.name == function:
            a = node.args
            pairs = list(zip(a.args[len(a.args) - len(a.defaults):], a.defaults)) + list(zip(a.kwonlyargs, a.kw_defaults))
            for arg, default in pairs:
                if arg.arg == argument and default is not None:
                    return ast.literal_eval(default)
    raise ValueError(f"{path} has no default for {function}({argument}=...)")


#: The mechanism environment's evaluation delay before a grasp, the default of `Go2D1MechEnv.set_evaluation`'s
#: `engage_delay_steps` (`run_unifp_train.py mech_eval` passes none). `mech_env` imports Isaac Lab, so it is read off the
#: source.
MECH_ENV_SRC = REPO / "unifp_train" / "mech_env.py"
#: The bleed S4b scores every arm with (`--force_law_bleed 1.0`, the week 10 log, 2026-11-19), s.
MECH_EVAL_BLEED_S = 1.0
#: The approach's slide on the dog, s: a choice in the middle of training's 1-3 s draw (`interface.EE_GOAL_TRAJ_TIME_S`),
#: not a trained value. `MechRunner.reset(traj_steps=...)` takes another.
MECH_DEFAULT_SLIDE_S = 2.0


def run_env_cfg(record_path: Path) -> dict | None:
    """The training run's environment config as it trained (`env.yaml` beside its run.json, `run_unifp_train.py`'s
    `dump_yaml` of `env.cfg`), or None without one. Read with the safe loader, Isaac Lab's tuples as lists; any other
    Python tag is refused."""
    import yaml

    path = Path(record_path).parent / "env.yaml"
    if not path.exists():
        return None

    class Loader(yaml.SafeLoader):
        pass

    Loader.add_constructor("tag:yaml.org,2002:python/tuple", lambda loader, node: loader.construct_sequence(node))
    return yaml.load(path.read_text(), Loader=Loader)


def grasp_gate(record_path: Path, args: dict, hook_cfg, interface) -> dict:
    """The grasp's arrival gate and fallback as the run trained them (S4c, `mech_env.Go2D1MechEnvCfg.grasp_within_m` and
    `engage_max_wait_steps`): from its `env.yaml`, the config the environment ran, checked against run.json's
    `--mech_grasp_within_m` / `--mech_engage_max_wait_s`; run.json's alone for a run without one. None: no gate (every run
    before S4c), and the fallback `hook_cfg.ENGAGE_MAX_WAIT_STEPS`."""
    flag_within = args.get("mech_grasp_within_m")
    flag_wait_s = args.get("mech_engage_max_wait_s")
    flag_wait = None if flag_wait_s is None else int(math.ceil(float(flag_wait_s) / interface.POLICY_DT))
    cfg = run_env_cfg(record_path)
    if cfg is None:
        within, wait, source = flag_within, flag_wait, "run.json arguments (no env.yaml)"
    else:
        within, wait, source = cfg.get("grasp_within_m"), cfg.get("engage_max_wait_steps"), "env.yaml"
        if (within is None) != (flag_within is None) or (within is not None and float(within) != float(flag_within)):
            raise ValueError(f"{record_path.parent.name}: env.yaml's grasp_within_m {within} is not run.json's "
                             f"--mech_grasp_within_m {flag_within}")
        if (wait is None) != (flag_wait is None) or (wait is not None and int(wait) != flag_wait):
            raise ValueError(f"{record_path.parent.name}: env.yaml's engage_max_wait_steps {wait} is not run.json's "
                             f"--mech_engage_max_wait_s {flag_wait_s}")
    return {"grasp_within_m": None if within is None else float(within),
            "max_wait_steps": hook_cfg.ENGAGE_MAX_WAIT_STEPS if wait is None else int(wait), "source": source}


def mech_meta(checkpoint: Path, variant: str, train_run: Path | None, law: dict | None = None) -> dict:
    """Everything `mech_layer.MechRunner` needs beside the network, from the training code and the training run. `law`
    ({"kp", "ki", "max_n", "bleed_s"}, any subset) replaces the evaluation's law: `mech_cfg.FORCE_LAW_*` and S4b's bleed."""
    from unifp_isaaclab import interface
    from unifp_train import arm_kin, hook_cfg, mech_cfg, task_cfg

    record_path = run_record_path(checkpoint, train_run)
    record = json.loads(record_path.read_text()) if record_path.exists() else {}
    args = record.get("arguments") or {}
    if not args:
        raise ValueError(f"a mechanism export needs its training run's run.json (--train_run): none at {record_path}")
    if record.get("task") != "mechanism":
        raise ValueError(f"{record_path} is not a mechanism training run (task {record.get('task')!r})")
    trained = record.get("mechanism") or {}
    if not trained.get("force_law"):
        raise ValueError(f"{record_path.parent.name} trained without the force law; this layer is the law's")
    if variant != "force":
        raise ValueError("the layer's law writes the force command: export a mechanism policy with --variant force")
    tool_body, tool_offset = record.get("tool_body"), record.get("tool_offset_m")
    if tool_body != "Link6" or tool_offset is None:
        raise ValueError(f"the layer's forward kinematics end at Link6; the run's tool is {tool_body} {tool_offset}")
    checks = {}
    if train_run is not None:
        sha = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
        same = [p.name for p in sorted(Path(train_run).glob("model_*.pt")) if p.stat().st_size == checkpoint.stat().st_size
                and hashlib.sha256(p.read_bytes()).hexdigest() == sha]
        if not same:
            raise ValueError(f"{checkpoint} is none of {train_run}'s model files")
        checks = {"checkpoint_in_run": same, "run_final_checkpoint": record.get("final_checkpoint")}
    variant_trained = trained.get("law_variant") or args.get("law_variant") or "v1"
    v2 = variant_trained in ("v2", "v3")
    gains = trained.get("force_law_gains") or [mech_cfg.FORCE_LAW_KP_N_PER_M, mech_cfg.FORCE_LAW_KI_N_PER_M_S,
                                               mech_cfg.FORCE_LAW_MAX_N]
    bleed_trained = trained.get("force_law_bleed_s")
    if bleed_trained is None and v2:
        bleed_trained = mech_cfg.LAW_V2_BLEED_S
    evaluation = {"kp": mech_cfg.FORCE_LAW_KP_N_PER_M, "ki": mech_cfg.FORCE_LAW_KI_N_PER_M_S,
                  "max_n": mech_cfg.FORCE_LAW_MAX_N, "bleed_s": MECH_EVAL_BLEED_S}
    evaluation.update({k: v for k, v in (law or {}).items() if k in evaluation})
    evaluation["bleed_s"] = float(evaluation["bleed_s"]) if evaluation["bleed_s"] else None
    evaluation["arrived_m"] = mech_cfg.FORCE_LAW_ARRIVED_M
    roll = bool(record.get("roll_objective"))
    stance = (record.get("legs") or {}).get("dog_stance_fraction", args.get("dog_stance_fraction"))
    wrist_payload = args.get("payload") == "physical"
    kinds = {k: mech_cfg.EVAL_KINDS[k] for k in mech_cfg.EVAL_DEFAULT_KINDS}
    gate = grasp_gate(record_path, args, hook_cfg, interface)
    return {
        "law": {"evaluation": evaluation,
                "training": {"variant": variant_trained, "kp": gains[0], "ki": gains[1], "max_n": gains[2],
                             "bleed_s": bleed_trained, "arrived_m": mech_cfg.FORCE_LAW_ARRIVED_M,
                             "max_range_n": list(mech_cfg.LAW_V2_MAX_RANGE_N) if v2 else None},
                "is": "lag = ref - s while grasped; integral of the lag, zeroed when it pushes against the lag; with "
                      "bleed_s, the integral times exp(-dt / bleed_s) a step while |lag| < arrived_m; "
                      "F = clamp(kp lag + ki integral, -max_n, max_n) along the path's tangent at s",
                "source": "unifp_train/mech_env.py Go2D1MechEnv._step_forces, set_force_law"},
        "plan": {"fraction": 1.0, "speed_m_s": mech_cfg.DEV_CONDITIONS["speed_m_s"],
                 "test_speed_m_s": mech_cfg.TEST_CONDITIONS["speed_m_s"], "held": "to the end of the episode",
                 "source": "mech_env.set_evaluation: TravelGoal.use_plan(1.0, conditions['speed_m_s'])",
                 "training": {"speed_m_s": list(mech_cfg.GOAL_SPEED_M_S), "first_fraction": list(mech_cfg.GOAL_FIRST_FRACTION),
                              "hold_s": list(mech_cfg.GOAL_HOLD_S)}},
        "approach": {"init_start": list(interface.EE_GOAL_INIT_START), "slide_s_range": list(interface.EE_GOAL_TRAJ_TIME_S),
                     "hold_s_range": list(interface.EE_GOAL_HOLD_TIME_S), "default_slide_s": MECH_DEFAULT_SLIDE_S,
                     "engage_delay_steps": int(function_default(MECH_ENV_SRC, "set_evaluation", "engage_delay_steps")),
                     "engage_delay_steps_training": list(hook_cfg.ENGAGE_DELAY_STEPS),
                     "settle_speed_m_s": hook_cfg.ENGAGE_SETTLE_SPEED_M_S, "max_wait_steps": gate["max_wait_steps"],
                     "grasp_within_m": gate["grasp_within_m"],
                     "grasp_within_is": "the rule, its fallback included, closes only with the tool within this of the "
                                        "goal it was given (mech_env._engage_due, S4c); null: wherever the tool is",
                     "grasp_gate_source": gate["source"],
                     "roll_rad": 0.0,
                     "grasp_is": "in simulation the mechanism is built wherever the tool is when the rule fires"},
        "goal_sphere": {"centre_offset": list(interface.EE_GOAL_CENTER_OFFSET),
                        "centre_is": "the base's x, y at z = 0 (the ground), plus centre_offset in the base's yaw frame",
                        "coordinates": "(radius, pitch, yaw) of the goal less the centre, in the base's yaw frame",
                        "held_goal_is": "with the law: the handle at the measured travel s (not the reference)",
                        "radius_range": list(interface.EE_GOAL_RADIUS_RANGE),
                        "pitch_range": list(interface.EE_GOAL_PITCH_RANGE), "yaw_range": list(interface.EE_GOAL_YAW_RANGE)},
        "commands": {"velocity": [0, 3], "goal": [3, 6], "roll": [6, 8], "roll_value": [0.0, 1.0] if roll else [0.0, 0.0],
                     "force": [9, 12], "force_is": "the law's force along the path, N, in the base's yaw frame",
                     "reset_frame": "all zero", "roll_objective": roll},
        "tool": {"body": tool_body, "offset_m": list(tool_offset), "point": args.get("tool_point"),
                 "jaw_centre_link6_m": list(arm_kin.JAW_CENTRE_LINK6_M)},
        "gripper_force_kp": task_cfg.GRIPPER_FORCE_KP,
        "force_target_is": "UniFP's tool target: the goal plus the force command / gripper_force_kp (N/m)",
        "stance": {"dog_stance_fraction_training": stance, "dog_stance_fraction_evaluation": 1.0,
                   "leg_offsets": list(task_cfg.DOG_STANCE_LEG_OFFSETS), "base_m": task_cfg.DOG_STANCE_BASE_M,
                   "pitch_deg": task_cfg.DOG_STANCE_PITCH_DEG, "roll_deg": task_cfg.DOG_STANCE_ROLL_DEG},
        "tilt_stop_deg": args.get("mech_tilt_stop_deg"),
        "evaluation": {"conditions": {"validation": dict(mech_cfg.DEV_CONDITIONS), "test": dict(mech_cfg.TEST_CONDITIONS)},
                       "sets": mech_cfg.S4B_MANIFEST, "kinds": kinds, "budget_n": mech_cfg.BUDGET_N,
                       "budget_hold_steps": mech_cfg.BUDGET_HOLD_STEPS, "opened_fraction": mech_cfg.EVAL_OPENED_FRACTION,
                       "episode_s": mech_cfg.EVAL_EPISODE_S, "grip_n": mech_cfg.GRIP_N},
        **arm_constants(1.0, wrist_payload),
        "trained_with": {**{k: args.get(k) for k in ("task", "force_law", "law_variant", "force_law_bleed", "force_law_gains",
                                                     "dog_stance_fraction", "mech_tilt_stop_deg", "mech_grasp_within_m",
                                                     "mech_engage_max_wait_s", "mech_approach_weight", "leg_variant",
                                                     "leg_actuator", "leg_armature_range", "leg_knee_armature_factor",
                                                     "leg_knee_friction_factor", "leg_friction_range", "leg_delay_steps",
                                                     "leg_delay_min_steps", "arm_locked", "arm_locked_range",
                                                     "arm_follower", "arm_servo", "arm_step_weight", "arm_wrist_weight",
                                                     "payload", "tool_point", "seed", "resume_from", "iterations")},
                         "mechanism": trained, "arm_link": record.get("arm_link"), "legs": record.get("legs"),
                         "run": record_path.parent.name, **checks},
    }


#: The start gate's joint bound is this percentile of the library's stances' largest joint offset from the settled
#: stance, and its pitch bounds the library's own widened by `GATE_PITCH_MARGIN_DEG`.
GATE_PERCENTILE, GATE_PITCH_MARGIN_DEG = 95.0, 1.0


def start_gate(checkpoint: Path, train_run: Path | None = None) -> dict | None:
    """The stances the policy trained from, as bounds the dog's settle gate can check (`press_live._stand_up`): the
    largest joint offset from `leg_cfg.SETTLED_LEG_POS` (rad) and the pitch range (degrees, + nose down) of the training
    run's spawn library, the settled stance included. None (the gate's own defaults) without a library."""
    from unifp_train import leg_cfg

    record = run_record_path(checkpoint, train_run)
    leg = (json.loads(record.read_text()).get("leg") or {}) if record.exists() else {}
    library = Path(leg.get("spawn_states") or "")
    if not leg.get("spawn_states") or not library.exists():
        return None
    states = json.loads(library.read_text())
    settled = np.asarray(leg_cfg.SETTLED_LEG_POS)
    off = [float(np.abs(np.asarray(s["legs"]) - settled).max()) for s in states]
    pitch = [0.0] + [math.degrees(s["pitch"]) for s in states]
    return {"library": library.name, "stances": len(states),
            "max_off_settled_rad": round(float(np.percentile(off, GATE_PERCENTILE)), 3),
            "pitch_deg": [round(min(pitch) - GATE_PITCH_MARGIN_DEG, 1), round(max(pitch) + GATE_PITCH_MARGIN_DEG, 1)],
            "rule": f"p{GATE_PERCENTILE:g} of each stance's largest joint offset; pitch range +-{GATE_PITCH_MARGIN_DEG} deg"}


def export(checkpoint: str | Path, variant: str, out: str | Path, tolerance: float = 1e-4, task: str | None = None,
           train_run: str | Path | None = None, mech_law: dict | None = None) -> dict:
    """Write `out` from `checkpoint`; refuse (raise) unless numpy reproduces the checkpoint's actions and estimates.
    `task` "insert" adds the insertion's constants (`insert_meta`) from `train_run` (its run.json; default: beside the
    checkpoint), "mechanism" the held mechanism's (`mech_meta`, with `mech_law` replacing the evaluation's law); None
    names the task by the action count (leg or reach), as before."""
    import torch
    from tensordict import TensorDict

    from demos.combiner_key.robot import leg_policy
    from unifp_isaaclab import interface
    from unifp_train.eval import EvaluablePolicy

    checkpoint = Path(checkpoint).resolve()
    train_run = Path(train_run).resolve() if train_run is not None else None
    policy = EvaluablePolicy(str(checkpoint), device="cpu")
    if policy.num_tool_obs:
        # S1's tool-conditioned networks read a descriptor beside the stack; `leg_policy` runs the stack alone.
        raise ValueError(f"{checkpoint.name} reads a {policy.num_tool_obs}-wide tool input, which leg_policy does not run")
    if task in ("insert", "mechanism") and policy.num_actions != interface.NUM_ACTIONS:
        raise ValueError(f"an {task} policy has {interface.NUM_ACTIONS} outputs, {checkpoint.name} has {policy.num_actions}")
    actor = policy.actor
    parts = {"encoder": linear_layers(actor.encoder), "actor": linear_layers(actor.mlp),
             "decoder": linear_layers(actor.decoder)}
    arrays = {}
    for name, layers in parts.items():
        for k, (w, b) in enumerate(layers):
            arrays[f"{name}_w{k}"], arrays[f"{name}_b{k}"] = w.astype(np.float32), b.astype(np.float32)
    meta = meta_for(variant, checkpoint, policy.iteration, policy.num_actions,
                    {name: len(layers) for name, layers in parts.items()}, task=task, train_run=train_run,
                    mech_law=mech_law)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(".tmp.npz")
    np.savez_compressed(tmp, meta=np.array(json.dumps(meta)), **arrays)

    # The numpy network against the checkpoint's own, on stacks at the observation's scale and wider.
    exported = leg_policy.LegPolicy(tmp)
    rng = np.random.default_rng(20261021)
    stacks = np.concatenate((rng.normal(0.0, 0.5, (48, interface.NUM_OBS)), rng.normal(0.0, 2.0, (16, interface.NUM_OBS))))
    with torch.inference_mode():
        batch = torch.as_tensor(stacks, dtype=torch.float32)
        ref_act = policy.act(batch).numpy()
        ref_est = actor.estimate(TensorDict({"policy": batch}, batch_size=[len(stacks)])).numpy()
    result = {"checkpoint": str(checkpoint), "variant": variant, "iteration": policy.iteration, "stacks": len(stacks),
              "action_max_abs_err": float(np.abs(exported.act(stacks) - ref_act).max()),
              "estimate_max_abs_err": float(np.abs(exported.estimate(stacks) - ref_est).max()),
              "action_max_abs": float(np.abs(ref_act).max())}
    if result["action_max_abs_err"] > tolerance or result["estimate_max_abs_err"] > tolerance:
        tmp.unlink()
        raise ValueError(f"numpy does not reproduce the checkpoint to {tolerance}: {result}")
    if task == "insert":
        # The runner builds from the export alone (its layer, kinematics and limits).
        from demos.combiner_key.robot import insert_policy
        insert_policy.InsertRunner(exported.meta)
        result["task"] = "insert"
    if task == "mechanism":
        from demos.combiner_key.robot import mech_layer
        mech_layer.MechRunner(exported.meta)
        result["task"] = "mechanism"
    tmp.replace(out)
    result.update(out=str(out), bytes=out.stat().st_size, sha256=hashlib.sha256(out.read_bytes()).hexdigest())
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--variant", choices=("force", "position"), required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--tolerance", type=float, default=1e-4, help="largest |numpy - torch| allowed")
    parser.add_argument("--task", choices=("insert", "mechanism"), default=None,
                        help="insert: the key insertion's whole-body policy, with its layer's and IK's constants "
                             "(insert_policy.InsertRunner); mechanism: a held-mechanism (T1) policy with its force "
                             "law's, plan's and goal's constants (mech_layer.MechRunner); unset, a leg or reach policy by "
                             "its action count")
    parser.add_argument("--law_gains", type=float, nargs=3, default=None, metavar=("KP", "KI", "MAX"),
                        help="--task mechanism: the law the export records for the dog (N/m, N/(m*s), N); default "
                             "mech_cfg.FORCE_LAW_* (400 1000 80), as mech_eval scores")
    parser.add_argument("--law_bleed_s", type=float, default=None,
                        help=f"--task mechanism: the law's integral bleed, s (default {MECH_EVAL_BLEED_S}, S4b's scoring; "
                             "0 for none)")
    parser.add_argument("--train_run", default=None,
                        help="the checkpoint's training run directory (its run.json), when the checkpoint is a copy "
                             "outside it; the checkpoint must be one of its model files")
    args = parser.parse_args()
    law = None
    if args.task == "mechanism" and (args.law_gains is not None or args.law_bleed_s is not None):
        law = {}
        if args.law_gains is not None:
            law.update(zip(("kp", "ki", "max_n"), args.law_gains))
        if args.law_bleed_s is not None:
            law["bleed_s"] = args.law_bleed_s
    try:
        r = export(args.checkpoint, args.variant, args.out, args.tolerance, task=args.task, train_run=args.train_run,
                   mech_law=law)
    except ValueError as error:
        print(f"REFUSED: {error}", file=sys.stderr)
        return 1
    print(f"numpy against torch on {r['stacks']} random stacks: actions {r['action_max_abs_err']:.2e}, estimates "
          f"{r['estimate_max_abs_err']:.2e} (|a| up to {r['action_max_abs']:.2f})")
    print(f"wrote {r['out']} ({r['bytes'] / 1e6:.1f} MB, sha256 {r['sha256'][:16]}...) from "
          f"{Path(r['checkpoint']).name}, {r['variant']}, iteration {r['iteration']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
