"""The foot-press program's handover on the dog, simulated: where the legs come to rest before the policy starts.

`run_unifp_train.py leg_handover --task leg --leg_object press` drives every robot through what
`demos/combiner_key/robot/press_live.py` does to the real legs before it hands them to the policy, with nothing else
acting: lying (sport mode's pose, held), the stand-up to the policy's default targets on unitree_rl_lab's FixStand gains
(`lowlevel.STAND_KP/KD`, the gains ramped in over 0.5 s, the targets smoothstepped over `STAND_S`), the hold, the gains
blended to the policy's 25 / 0.5 with the targets kept, and the wait the settle gate gives it (`SETTLE_MAX_S`). The
physics is the training's (the leg actuator, armature, delay and, with `--leg_friction_range`, joint friction drawn per
robot), stepped directly: no task, no rewards, no resets.

It writes, per robot, the state at the end of the hold and at the end of the wait, against the dog's (`DOG`, the second
live run, F-191), and the rests in the spawn library's format (`leg_env.Go2D1LegEnvCfg.spawn_states`).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import torch

from demos.combiner_key.robot import lowlevel as ll, press_live
from unifp_isaaclab import interface

from . import leg_cfg

#: Unitree's leg order (FR, FL, RR, RL) from UniFP's (FL, FR, RL, RR), and back: the same permutation.
UNITREE_FROM_UNIFP = np.array([3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8])

#: The dog, 2026-10-23, the second live run (`d1dogpress --live --presses 0`, run 20261023T020839_leg_press_live, F-191).
#: Lying before the stand-up: the calves from its "lying" event; hips and thighs from the stand-in's lying pose (the
#: dog's LowState that day, `fake_go2.LYING_Q`), whose calves (-2.78 to -2.85) were past the URDF's -2.72.
DOG_LYING_UNITREE = np.array([-0.046, 1.253, -2.467, 0.081, 1.247, -2.481, -0.346, 1.271, -2.531, 0.302, 1.295, -2.503])
DOG = {
    #: The stand-up's hold at kp 80 (the notes' figures): base over the feet, pitch (+ nose down).
    "hold": {"base_m": 0.303, "pitch_deg": -4.6},
    #: At rest at kp 25 after the blend, the settle gate's event at 14.12 s.
    "rest": {"base_m": 0.261, "roll_deg": 1.4, "pitch_deg": -8.1,
             "q_minus_settled_unitree": [0.069, -0.053, 0.083, -0.062, -0.074, 0.127,
                                         -0.102, -0.162, -0.094, 0.103, -0.2, -0.013]},
}
#: Lying: the base spawned this high and the pose held on the stand-up's gains this long (sport mode holds it), then
#: the legs passive while the program releases sport mode and listens on /lowcmd (the dog's 6.18 -> 6.60 s).
LIE_Z_M, LIE_S, RELEASE_S = 0.20, 1.5, 0.42
#: Besides the end of the hold and of the wait, the states saved for the spawn library, with their velocities: seconds
#: after the hold ends (the blend is the first `GAIN_BLEND_S`), so the library spans the sink from the hold to the rest,
#: through the second after the blend in which the robot drops fastest.
LIBRARY_AFTER_HOLD_S = (0.0, 0.5, 0.75, 1.0, 1.15, 1.3, 1.45, 1.6, 2.0, 3.0, 4.0)


def dog_rest_unifp() -> np.ndarray:
    """The dog's kp-25 rest, UniFP's order (rad)."""
    settled_unitree = np.asarray(leg_cfg.SETTLED_LEG_POS)[UNITREE_FROM_UNIFP]
    rest_unitree = settled_unitree + np.asarray(DOG["rest"]["q_minus_settled_unitree"])
    return rest_unitree[UNITREE_FROM_UNIFP]


def _groups(robot):
    """The legs' explicit actuator groups and, per group, the hip / thigh / calf index of each of its joints."""
    from isaaclab.actuators import IdealPDActuator

    out = []
    for name, group in robot.actuators.items():
        if name == "unifp":
            continue
        if not isinstance(group, IdealPDActuator):
            raise ValueError(f"leg_handover changes the gains as it goes: group {name!r} ({type(group).__name__}) is "
                             "not an explicit PD actuator; use --leg_actuator unitree_geared (or another explicit model)")
        part = [0 if n.endswith("hip_joint") else 1 if n.endswith("thigh_joint") else 2 for n in group.joint_names]
        out.append((group, torch.tensor(part, device=robot.device)))
    return out


def _set_gains(groups, kp: np.ndarray, kd: np.ndarray) -> None:
    """Per-part gains (hip, thigh, calf), the same on every leg."""
    for group, part in groups:
        group.stiffness[:] = torch.as_tensor(kp, dtype=torch.float32, device=part.device)[part]
        group.damping[:] = torch.as_tensor(kd, dtype=torch.float32, device=part.device)[part]


def run(env, args, report: dict, run_dir: Path) -> int:
    robot, n, dev = env._robot, env.num_envs, env.device
    legs = env._order[:12]
    groups = _groups(robot)
    origins = env.scene.env_origins
    default = torch.as_tensor(interface.DEFAULT_DOF_POS[:12], dtype=torch.float32, device=dev)
    lying = torch.as_tensor(DOG_LYING_UNITREE[UNITREE_FROM_UNIFP], dtype=torch.float32, device=dev)
    kp_stand, kd_stand = ll.STAND_KP[:3], ll.STAND_KD[:3]
    kp_pol, kd_pol = np.full(3, interface.P_GAINS[0]), np.full(3, interface.D_GAINS[0])

    # Lying, every robot alike.
    joint_pos = robot.data.default_joint_pos.clone()
    joint_pos[:, legs] = lying
    joint_pos[:, env._order[12:18]] = env._arm_pose
    root = robot.data.default_root_state.clone()
    root[:, :3] = origins
    root[:, 2] += LIE_Z_M
    root[:, 3:7] = torch.tensor([1.0, 0.0, 0.0, 0.0], device=dev)
    root[:, 7:] = 0.0
    robot.write_root_pose_to_sim(root[:, :7])
    robot.write_root_velocity_to_sim(root[:, 7:])
    robot.write_joint_state_to_sim(joint_pos, torch.zeros_like(joint_pos))
    env.scene.reset()

    dt = env.physics_dt
    stand_s, hold_s, blend_s, wait_s = press_live.STAND_S, press_live.STAND_HOLD_S, press_live.GAIN_BLEND_S, \
        press_live.SETTLE_MAX_S
    phases = [("lie", LIE_S), ("release", RELEASE_S), ("stand", stand_s), ("hold", hold_s), ("blend", blend_s),
              ("wait", wait_s)]
    total = sum(s for _, s in phases)
    trace = {k: [] for k in ("t", "base_m", "roll", "pitch", "q", "dq", "root_vel")}
    marks: dict[str, dict] = {}
    from_q = lying.expand(n, 12).clone()

    def state() -> dict:
        rp = interface.body_roll_pitch(robot.data.root_quat_w)
        return {"base_m": (robot.data.root_pos_w[:, 2] - origins[:, 2]).clone(), "roll": rp[:, 0].clone(),
                "pitch": rp[:, 1].clone(), "q": robot.data.joint_pos[:, legs].clone(),
                "dq": robot.data.joint_vel[:, legs].clone(), "root_vel": robot.data.root_vel_w.clone()}

    steps = int(round(total / dt))
    t0 = {name: sum(s for _, s in phases[:k]) for k, (name, _) in enumerate(phases)}
    for i in range(steps):
        t = i * dt
        phase = next(name for name, _ in reversed(phases) if t >= t0[name] - 1e-9)
        s = t - t0[phase]
        if phase == "lie":
            target, kp, kd = lying.expand(n, 12), kp_stand, kd_stand
        elif phase == "release":
            target, kp, kd = lying.expand(n, 12), np.zeros(3), np.zeros(3)
        elif phase == "stand":
            if s < dt / 2:
                from_q = robot.data.joint_pos[:, legs].clone()
            u = min(max(s / stand_s, 0.0), 1.0)
            target = from_q + (default - from_q) * (u * u * (3.0 - 2.0 * u))
            kp, kd = kp_stand * min(s / 0.5, 1.0), kd_stand
        elif phase == "hold":
            target, kp, kd = default.expand(n, 12), kp_stand, kd_stand
        elif phase == "blend":
            target = default.expand(n, 12)
            kp, kd = ll.blend(kp_stand, kp_pol, s / blend_s), ll.blend(kd_stand, kd_pol, s / blend_s)
        else:
            target, kp, kd = default.expand(n, 12), kp_pol, kd_pol
        _set_gains(groups, kp, kd)
        robot.set_joint_position_target(target, joint_ids=legs)
        robot.set_joint_position_target(env._arm_pose, joint_ids=env._order[12:18])
        robot.set_joint_position_target(env._jaw_targets, joint_ids=env._jaw_joint_ids)
        env.scene.write_data_to_sim()
        env.sim.step(render=False)
        env.scene.update(dt=dt)
        if (i + 1) % interface.DECIMATION == 0:
            now = state()
            trace["t"].append(round((i + 1) * dt, 4))
            for k in ("base_m", "roll", "pitch", "q", "dq", "root_vel"):
                trace[k].append(now[k].cpu().numpy())
        end = {"hold": t0["blend"], "wait": total} | {f"after_hold_{a}": t0["blend"] + a for a in LIBRARY_AFTER_HOLD_S}
        for name, at in end.items():
            if abs((i + 1) * dt - at) < dt / 2:
                marks[name] = {k: v.cpu().numpy() for k, v in state().items()}

    tr = {k: np.asarray(v) for k, v in trace.items()}
    # The settle gate's rule (`press_live._stand_up`): still once no joint moved SETTLED_MOVE_RAD over SETTLED_TICKS
    # policy ticks, judged from the end of the blend.
    after = np.nonzero(tr["t"] >= t0["wait"] - 1e-6)[0]
    q = tr["q"]
    still_at = np.full(n, np.nan)
    k0 = press_live.SETTLED_TICKS
    for idx in after:
        if idx < k0 - 1:
            continue
        moved = np.abs(q[idx] - q[idx - k0 + 1]).max(axis=-1)
        fresh = np.isnan(still_at) & (moved < press_live.SETTLED_MOVE_RAD)
        still_at[fresh] = tr["t"][idx] - t0["wait"]

    friction = getattr(env, "_leg_friction_drawn", None)
    armature = getattr(env, "_leg_armature_drawn", None)
    settled = np.asarray(leg_cfg.SETTLED_LEG_POS)
    dog_rest = dog_rest_unifp()
    rest, hold = marks["wait"], marks["hold"]
    rows, library = [], []
    for e in range(n):
        off = rest["q"][e] - settled
        row = {
            "env": e,
            "friction_nm": None if friction is None else [round(float(friction[e, 0]), 4), round(float(friction[e, 2]), 4)],
            "armature": None if armature is None else round(float(armature[e, 0]), 5),
            "hold": {"base_m": round(float(hold["base_m"][e]), 4), "pitch_deg": round(math.degrees(hold["pitch"][e]), 2)},
            "rest": {"base_m": round(float(rest["base_m"][e]), 4), "roll_deg": round(math.degrees(rest["roll"][e]), 2),
                     "pitch_deg": round(math.degrees(rest["pitch"][e]), 2),
                     "q_minus_settled_unitree": np.round(off[UNITREE_FROM_UNIFP], 3).tolist(),
                     "max_off_settled_rad": round(float(np.abs(off).max()), 3),
                     "max_off_dog_rad": round(float(np.abs(rest["q"][e] - dog_rest).max()), 3),
                     "still_after_blend_s": None if np.isnan(still_at[e]) else round(float(still_at[e]), 2),
                     "max_dq": round(float(np.abs(rest["dq"][e]).max()), 4)},
        }
        rows.append(row)
        for a in LIBRARY_AFTER_HOLD_S:
            m = marks[f"after_hold_{a}"]
            library.append({"legs": [round(float(v), 5) for v in m["q"][e]], "base_z": round(float(m["base_m"][e]), 4),
                            "roll": round(float(m["roll"][e]), 5), "pitch": round(float(m["pitch"][e]), 5),
                            "joint_vel": [round(float(v), 4) for v in m["dq"][e]],
                            "root_vel": [round(float(v), 4) for v in m["root_vel"][e]],
                            "source": f"leg_handover {run_dir.name} env {e}, {a} s after the hold",
                            "friction_nm": row["friction_nm"]})

    def stats(values) -> dict:
        v = np.asarray(values, dtype=float)
        return {"median": round(float(np.median(v)), 4), "min": round(float(v.min()), 4), "max": round(float(v.max()), 4)}

    summary = {
        "phases_s": dict(phases),
        "stand_kp_kd": [kp_stand.tolist(), kd_stand.tolist()], "policy_kp_kd": [kp_pol.tolist(), kd_pol.tolist()],
        "lying_unitree": DOG_LYING_UNITREE.tolist(), "lie_z_m": LIE_Z_M, "release_s": RELEASE_S,
        "library_after_hold_s": list(LIBRARY_AFTER_HOLD_S),
        "dog": DOG,
        "hold_base_m": stats([r["hold"]["base_m"] for r in rows]),
        "hold_pitch_deg": stats([r["hold"]["pitch_deg"] for r in rows]),
        "rest_base_m": stats([r["rest"]["base_m"] for r in rows]),
        "rest_pitch_deg": stats([r["rest"]["pitch_deg"] for r in rows]),
        "rest_roll_deg": stats([r["rest"]["roll_deg"] for r in rows]),
        "rest_max_off_settled_rad": stats([r["rest"]["max_off_settled_rad"] for r in rows]),
        "rest_max_off_dog_rad": stats([r["rest"]["max_off_dog_rad"] for r in rows]),
        "still_fraction": round(float(np.mean(~np.isnan(still_at))), 3),
        "gate_would_pass_fraction": round(float(np.mean([
            r["rest"]["max_off_settled_rad"] <= press_live.SETTLED_TOL_RAD
            and max(abs(r["rest"]["roll_deg"]), abs(r["rest"]["pitch_deg"])) <= press_live.SETTLED_TILT_DEG
            and r["rest"]["still_after_blend_s"] is not None for r in rows])), 3),
    }
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "handover.json").write_text(json.dumps({"summary": summary, "robots": rows}, indent=1))
    (run_dir / "handover_rests.json").write_text(json.dumps(library, indent=1))
    np.savez_compressed(run_dir / "handover_trace.npz", **tr, phase_start_s=np.array([t0[p] for p, _ in phases]))
    report["handover"] = summary
    report["status"] = "finished"
    print(json.dumps(summary, indent=1), flush=True)
    return 0
