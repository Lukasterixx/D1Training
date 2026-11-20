"""`leg_eval --key_insert --insert_shadow_dump N`: every per-step input the dog's numpy insertion runner needs, and every
output to compare it with, for the first N environments of a frozen insertion evaluation (`insert_shadow_dump.npz`).

Read-only: it copies the environment's buffers to the CPU before and after each `env.step` and draws nothing, steps
nothing and calls nothing that changes state (not even `ArmLink.observe`, which advances the link's report clock), so an
evaluation with the dump scores what one without it does. Checked against by
`demos/combiner_key/robot/check_insert.py --sim`.

Index `t` of a step is the state the frame `t` was built from (t = 0: the reset; then after each policy step), the
layer's and the environment's buffers as `_update_hold` left them for that frame, the frame and its commands, the action
the policy took on it and the joint targets that action produced (`env._joint_targets`, UniFP's order, before the
arm's link). `alive` is 1 until the environment's first episode has ended (its last step included).
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import torch

from unifp_isaaclab import interface

#: The layer's buffers (`keylock.InsertLayer`) recorded each step.
LAYER_FIELDS = ("phase", "phase_for", "ref_depth", "ref_lateral", "force_cmd", "roll", "touch_depth", "lag_for",
                "lag_base", "held_for", "still_for", "still_any", "jiggle_for", "spiral_s", "spiral_dir", "wiggle_t",
                "entered_by", "last_depth", "seat_end", "press_n", "locate_stage", "face_depth", "refused", "retries",
                "relocates", "touched", "settle_for", "roll_wait")
#: The environment's (`keyinsert_env.Go2D1KeyInsertEnv`): its estimate of the lock, the believed key, the start, the
#: measured tip, the goal, the outer loop, the IK's reference and seed, the actor's miss.
ENV_FIELDS = (("est_o", "_est_o"), ("est_a", "_est_a"), ("est_u", "_est_u"), ("est_t", "_est_t"),
              ("start_depth", "_start_depth"), ("press_draw", "_press_draw"), ("ins_active", "_ins_active"),
              ("layer_started", "_layer_started"), ("since_reset", "_since_reset"), ("meas_depth", "_meas_depth"),
              ("meas_lateral", "_meas_lateral"), ("goal_jaw", "_goal_jaw"), ("loop", "_loop"), ("ins_ref", "_ins_ref"),
              ("ins_geo", "_ins_geo"), ("miss_b", "_miss_b"), ("done_for", "_done_for"), ("ik_fail", "_ik_fail"),
              ("ik_calls", "_ik_calls"), ("arm_pose", "_arm_pose"))


def _np(value: torch.Tensor) -> np.ndarray:
    return value.detach().float().cpu().numpy()


class InsertShadowDump:
    def __init__(self, env, count: int):
        self.n = min(int(count), env.num_envs)
        self.steps: dict[str, list] = {}
        self.after: dict[str, list] = {}
        names = list(env._robot.joint_names)
        self.meta = {
            "joint_names": np.array(names),
            "force_aware": np.array(bool(env._force_aware)),
            "seat_test": np.array(env._layer_ins.seat_test),
            "light_probe": np.array(bool(env._layer_ins.light_probe)),
            "hold_arm": np.array(env.cfg.hold_arm),
            "leg_answer": np.array(getattr(env, "_leg_answer", None) is not None),
            "sag_on": np.array(bool(env.cfg.sag_compensation and not env.cfg.arm_servo)),
            "arm_link": np.array(env._arm_link is not None),
            "arm_lower": _np(env._lower), "arm_upper": _np(env._upper),
            "policy_dt": np.array(interface.POLICY_DT),
        }

    def _push(self, store: dict, key: str, value) -> None:
        store.setdefault(key, []).append(value[:self.n].copy() if isinstance(value, np.ndarray) else value)

    def before_step(self, env, obs: dict, actions: torch.Tensor, done: torch.Tensor) -> None:
        """The state frame `t` was built from, the layer's and the environment's buffers for it, and the action on it."""
        n, data = self.n, env._robot.data
        rec = {"alive": (~done).float(), "joint_pos": data.joint_pos, "joint_vel": data.joint_vel,
               "root_quat_w": data.root_quat_w, "root_ang_vel_b": data.root_ang_vel_b,
               "root_pos": data.root_pos_w - env.scene.env_origins,
               "commands": env._commands, "frame": obs["policy"][:, -interface.NUM_SINGLE_OBS:], "action": actions}
        if env._arm_link is not None:
            rec["arm_reported"], rec["arm_rate"] = env._arm_link.reported, env._arm_link.rate
        else:
            q, dq = env._dof()
            rec["arm_reported"], rec["arm_rate"] = q[:, 12:18], dq[:, 12:18]
        rec["tool_reach"] = env._tool_bel["reach"]
        rec["tool_offset"] = env._tool_bel["offset"]
        for key, attr in ENV_FIELDS:
            rec[key] = getattr(env, attr)
        ly = env._layer_ins
        for key in LAYER_FIELDS:
            if hasattr(ly, key):
                rec[f"layer_{key}"] = getattr(ly, key)
        for key, value in rec.items():
            self._push(self.steps, key, _np(value[:n]))

    def after_step(self, env, ended: torch.Tensor) -> None:
        """The joint targets the action produced, and whether the episode ended on it."""
        self._push(self.after, "joint_targets", _np(env._joint_targets[:self.n]))
        self._push(self.after, "ended", _np(ended[:self.n].float()))

    def save(self, path: Path) -> dict:
        arrays = {k: np.stack(v, axis=1) for k, v in {**self.steps, **self.after}.items()}
        np.savez_compressed(path, **self.meta, **arrays)
        alive = arrays["alive"] > 0.5
        return {"file": str(path), "episodes": self.n, "steps": [int(a.sum()) for a in alive]}
