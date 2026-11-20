"""`mech_eval --mech_shadow_dump N`: every per-step input the dog's numpy mechanism runner needs, and every output to
compare it with, for N environments of a frozen mechanism evaluation (`mech_shadow_dump.npz`).

Read-only: it copies the environment's buffers to the CPU before and after each `env.step` and draws nothing, steps
nothing and calls nothing that changes state (not `ArmLink.observe`, which advances the link's report clock; `_tip_pos`,
`_tool_vel_w`, `_goal_centre` and `Mechanism.tangent` only read), so an evaluation with the dump scores what one without
it does. Checked against by `demos/combiner_key/robot/check_mech.py --sim`.

Which environments: N spread evenly over the plan's rows (`mech_eval.build_plan`: kind slowest, then level, then
placement), so each kind is in it at several levels; their indices are `env_ids`.

Index `t` of a step is the state frame `t` was built from (t = 0: the reset; then after each policy step): the robot,
the mechanism, the travel goal, the law's buffers, UniFP's goal generator and the grasp's counters as the environment
left them for that frame, the frame and its commands, and the action the policy took on it. The `*_after` fields are
what the step produced from that action: the joint targets (UniFP's order, before the arm's link), the law's integral
and force (`_step_forces`, run before the physics). `alive` is 1 until the environment's first episode has ended (its
last step included).
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import torch

from unifp_isaaclab import interface


def _np(value: torch.Tensor) -> np.ndarray:
    return value.detach().float().cpu().numpy()


class MechShadowDump:
    def __init__(self, env, count: int):
        n = min(int(count), env.num_envs)
        self.ids = torch.as_tensor(np.unique(np.round(np.linspace(0, env.num_envs - 1, n)).astype(np.int64)),
                                   device=env.device)
        self.steps: dict[str, list] = {}
        self.after: dict[str, list] = {}
        plan = env._evaluation or {}
        from . import hook_cfg, mech_cfg

        max_wait = getattr(env.cfg, "engage_max_wait_steps", None)

        ids = self.ids
        self.meta = {
            "joint_names": np.array(list(env._robot.joint_names)),
            "env_ids": ids.cpu().numpy(),
            "kind_names": np.array(list(mech_cfg.EVAL_KINDS)),
            "kinds": plan["kinds"][ids].cpu().numpy() if plan else np.zeros(len(ids)),
            "levels_n": plan["levels"][ids].cpu().numpy() if plan else np.zeros(len(ids)),
            "handles": plan["handles"][ids].cpu().numpy() if plan else np.zeros((len(ids), 3)),
            "engage_delay_steps": np.array(int(plan.get("engage_delay", -1))),
            # The grasp's arrival gate (S4c, `mech_env.Go2D1MechEnvCfg.grasp_within_m`; json, null: off) and the settle
            # rule's fallback as the environment ran them (`_engage_due`).
            "grasp_within_m": np.array(json.dumps(getattr(env.cfg, "grasp_within_m", None))),
            "engage_max_wait_steps": np.array(int(hook_cfg.ENGAGE_MAX_WAIT_STEPS if max_wait is None else max_wait)),
            "conditions": np.array(json.dumps(plan.get("conditions"))),
            "force_law": np.array(json.dumps(env._force_law)),
            "law_max": _np(env._law_max[ids]),
            "roll_objective": np.array(bool(env.cfg.roll_objective)),
            "tool_body": np.array(str(env.cfg.tool_body)),
            "tool_offset": np.asarray(env.cfg.tool_offset_m, dtype=np.float64),
            "arm_link": np.array(env._arm_link is not None),
            "leg_answer": np.array(getattr(env, "_leg_answer", None) is not None),
            "dog_stance_fraction": np.array(float(getattr(env.cfg, "dog_stance_fraction", 0.0))),
            "policy_dt": np.array(interface.POLICY_DT),
            "traj_steps": _np(env._goals.traj_steps[ids]),
            "total_steps": _np(env._goals.total_steps[ids]),
        }

    def _push(self, store: dict, key: str, value: torch.Tensor) -> None:
        store.setdefault(key, []).append(_np(value[self.ids]))

    def before_step(self, env, obs: dict, actions: torch.Tensor, done: torch.Tensor) -> None:
        """The state frame `t` was built from, the layer's buffers for it, and the action on it."""
        data, m, tg, g = env._robot.data, env._mech, env._travel_goal, env._goals
        rec = {"alive": (~done).float(), "joint_pos": data.joint_pos, "joint_vel": data.joint_vel,
               "root_quat_w": data.root_quat_w, "root_ang_vel_b": data.root_ang_vel_b,
               "root_pos": data.root_pos_w - env.scene.env_origins,
               "commands": env._commands, "frame": obs["policy"][:, -interface.NUM_SINGLE_OBS:], "action": actions,
               "tip_pos": env._tip_pos(), "tool_vel": env._tool_vel_w(), "goal_centre": env._goal_centre(),
               "feet_fz": env._contact_forces(env._feet_contacts)[:, :, 2]}
        if env._arm_link is not None:
            rec["arm_reported"], rec["arm_rate"] = env._arm_link.reported, env._arm_link.rate
        else:
            q, dq = env._dof()
            rec["arm_reported"], rec["arm_rate"] = q[:, 12:18], dq[:, 12:18]
        rec.update({
            "mech_grasped": m.grasped.float(), "mech_s": m.s, "mech_v": m.v, "mech_travel": m.travel,
            "mech_hinge": m.hinge.float(), "mech_start": m.start, "mech_direction": m.direction, "mech_axis": m.axis,
            "mech_radial": m.radial, "mech_centre": m.centre, "mech_radius": m.radius, "mech_tangent": m.tangent(),
            "mech_drive": m.drive_n, "mech_force_on_tool": m.force_on_tool, "mech_latched": m.latched.float(),
            "mech_released": m.released.float(), "mech_torn": m.torn.float(),
            "tg_ref": tg.ref, "tg_target": tg.target, "tg_speed": tg.speed, "tg_timer": tg.timer.float(),
            "law_integral": env._lag_integral, "law_command_w": env._law_command_w,
            "goal_current": g.current, "goal_current_roll": g.current_roll, "goal_timer": g.timer,
            "goal_start": g.start, "goal_goal": g.goal, "goal_hold_mask": g.hold_mask.float(), "goal_hold_goal": g.hold_goal,
            "since_reset": env._since_reset.float(), "engage_step": env._engage_step.float(),
            "ever_grasped": env._ever_grasped.float(), "grasp_step": env._grasp_step.float(),
            "grasp_error": env._grasp_error, "handle_sphere": env._handle_sphere, "mech_env": env._mech_env.float(),
        })
        for key, value in rec.items():
            self._push(self.steps, key, value)

    def after_step(self, env, ended: torch.Tensor) -> None:
        """The joint targets the action produced, the law's buffers after `_step_forces`, and whether and how the episode
        ended on it (`_get_dones`' torn, fallen and tilt flags; an ended environment's other buffers are its reset's)."""
        for key, value in (("joint_targets", env._joint_targets), ("ended", ended.float()),
                           ("law_integral_after", env._lag_integral), ("law_command_w_after", env._law_command_w),
                           ("force_cmd_after", env._commands[:, interface.CMD_EE_FORCE]),
                           ("term_torn", env._term_torn.float()), ("term_fallen", env._term_fallen.float()),
                           ("term_tilt", env._term_tilt.float())):
            self._push(self.after, key, value)

    def save(self, path: Path) -> dict:
        arrays = {k: np.stack(v, axis=1) for k, v in {**self.steps, **self.after}.items()}
        np.savez_compressed(path, **self.meta, **arrays)
        alive = arrays["alive"] > 0.5
        return {"file": str(path), "env_ids": [int(i) for i in self.meta["env_ids"]],
                "steps": [int(a.sum()) for a in alive]}
