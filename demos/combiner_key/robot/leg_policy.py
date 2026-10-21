"""The key-turning leg policy without torch: its observation, its history and its network, for the dog.

The dog's ROS image has numpy and no torch, so `export_policy.py` (on the PC) writes a checkpoint's weights and every
constant the policy's interface uses into one `.npz`, and this module runs it. Nothing here is copied from the training
code by hand: the defaults, scales, joint names and the waiting stage's command all come from the export, which reads
them from `unifp_isaaclab.interface` and `unifp_train.leg_cfg`. The only fixed fact is the order of Unitree's
`LowState.motor_state` (the firmware's).

What the policy sees each 50 Hz step (`interface.single_obs`, 76 wide): the base's roll and pitch (UniFP's
`euler_from_quat`), its angular velocity in the body frame x0.25, the 18 actioned joints minus their defaults (12 legs in
UniFP's FL, FR, RL, RR order, then the D1's six), their velocities x0.05, the previous actions (12 legs, then six zeros:
the leg task holds the arm), the gait phase (pinned to 0: sin 0, cos 1), and 15 commands x their scales. The encoder
reads the last 32 frames, oldest first, zero at the start (`unifp_isaaclab.policy.ObsHistory`); the actor reads the
newest frame and the encoder's 64-wide latent. Its 12 outputs are leg joint targets: default + 0.25 x action, held by
kp 25 / kd 0.5.

Checked against the training code by `tests/test_leg_shadow.py` (synthetic states) and `check_shadow.py --sim` (a
recorded Isaac rollout).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

#: `LowState.motor_state` order, fixed by the Go2's firmware: per leg FR, FL, RR, RL, each hip, thigh, calf.
UNITREE_LEGS = tuple(f"{leg}_{part}_joint" for leg in ("FR", "FL", "RR", "RL") for part in ("hip", "thigh", "calf"))


def _elu(x: np.ndarray) -> np.ndarray:
    return np.where(x > 0.0, x, np.expm1(np.minimum(x, 0.0)))


class _MLP:
    def __init__(self, layers: list[tuple[np.ndarray, np.ndarray]]):
        self.layers = layers

    def __call__(self, x: np.ndarray) -> np.ndarray:
        for k, (w, b) in enumerate(self.layers):
            x = x @ w.T + b
            if k < len(self.layers) - 1:
                x = _elu(x)
        return x


class LegPolicy:
    """An exported key-turning leg policy (`export_policy.py`): `act` and `estimate` on a (N, 2432) stack."""

    def __init__(self, path: str | Path):
        data = np.load(path, allow_pickle=False)
        self.meta = json.loads(str(data["meta"]))

        def mlp(name: str) -> _MLP:
            count = self.meta["layers"][name]
            return _MLP([(data[f"{name}_w{k}"].astype(np.float64), data[f"{name}_b{k}"].astype(np.float64))
                         for k in range(count)])

        self.encoder, self.actor, self.decoder = mlp("encoder"), mlp("actor"), mlp("decoder")
        self.num_single_obs = int(self.meta["num_single_obs"])

    def _latent_input(self, stacked: np.ndarray) -> np.ndarray:
        stacked = np.atleast_2d(stacked)
        return np.concatenate((stacked[:, -self.num_single_obs:], self.encoder(stacked)), axis=-1)

    def act(self, stacked: np.ndarray) -> np.ndarray:
        """The deterministic action (the distribution's mean), (N, 12)."""
        return self.actor(self._latent_input(stacked))

    def estimate(self, stacked: np.ndarray) -> np.ndarray:
        """The adaptation decoder's 12 estimates, in the units it was trained on (`decode_estimates` converts)."""
        return self.decoder(self.encoder(np.atleast_2d(stacked)))


def sphere_to_cart(radius: float, pitch: float, yaw: float) -> np.ndarray:
    """UniFP's (radius, pitch, yaw) -> (x, y, z): pitch is the elevation above the x-y plane."""
    return np.array([radius * math.cos(pitch) * math.cos(yaw), radius * math.cos(pitch) * math.sin(yaw),
                     radius * math.sin(pitch)])


def cart_to_sphere(p) -> np.ndarray:
    p = np.asarray(p, dtype=float)
    radius = float(np.linalg.norm(p))
    return np.array([radius, math.asin(p[2] / radius), math.atan2(p[1], p[0])])


def roll_pitch(quat_wxyz) -> tuple[float, float]:
    """UniFP's `euler_from_quat` (`interface.body_roll_pitch`): roll, pitch of a w-first quaternion."""
    w, x, y, z = (float(v) for v in quat_wxyz)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


class LegObservation:
    """Builds the policy's frames from the dog's state and keeps its 32-frame history.

    `meta` is the export's. Legs come in Unitree's order (`UNITREE_LEGS`), the arm as the D1's six URDF joint angles
    (radians, `d1_ik.from_servo_deg`). `step` takes the state the policy should act on and returns the stack to act
    on; `record_action` must follow with what the policy output, which the next frame carries as its previous action.
    """

    def __init__(self, meta: dict):
        self.meta = meta
        names = list(meta["dof_names"])
        self.num_actions, self.num_legs = int(meta["num_actions"]), int(meta["num_leg_actions"])
        #: UniFP index -> Unitree index, for the 12 legs.
        self.from_unitree = np.array([UNITREE_LEGS.index(n) for n in names[:self.num_legs]])
        #: Unitree index -> UniFP index.
        self.to_unitree = np.array([names.index(n) for n in UNITREE_LEGS])
        self.default = np.asarray(meta["default_dof_pos"], dtype=float)[:self.num_actions]
        self.commands_scale = np.asarray(meta["commands_scale"], dtype=float)
        self.frame_stack, self.num_single_obs = int(meta["frame_stack"]), int(meta["num_single_obs"])
        self.clip_obs, self.clip_actions = float(meta["clip_observations"]), float(meta["clip_actions"])
        self.history = np.zeros((self.frame_stack, self.num_single_obs))
        self.prev_actions = np.zeros(self.num_actions)

    def reset(self) -> None:
        """As at a training reset: an all-zero history and no previous action."""
        self.history[:] = 0.0
        self.prev_actions[:] = 0.0

    def frame(self, quat_wxyz, gyro_b, q_unitree, dq_unitree, arm_q, arm_dq, commands) -> np.ndarray:
        """One 76-wide frame (`interface.single_obs`), clipped as the environment clips it."""
        m = self.meta
        q = np.concatenate((np.asarray(q_unitree, dtype=float)[self.from_unitree], np.asarray(arm_q, dtype=float)))
        dq = np.concatenate((np.asarray(dq_unitree, dtype=float)[self.from_unitree], np.asarray(arm_dq, dtype=float)))
        out = np.concatenate((
            np.array(roll_pitch(quat_wxyz)),
            np.asarray(gyro_b, dtype=float) * m["obs_scale_ang_vel"],
            (q - self.default) * m["obs_scale_dof_pos"],
            dq * m["obs_scale_dof_vel"],
            self.prev_actions,
            np.array([0.0, 1.0]),                       # the gait phase, pinned to 0 (the leg task never walks)
            np.asarray(commands, dtype=float) * self.commands_scale,
        ))
        return np.clip(out, -self.clip_obs, self.clip_obs)

    def step(self, frame: np.ndarray) -> np.ndarray:
        """Push a frame; return the (2432,) stack, oldest first."""
        self.history = np.roll(self.history, -1, axis=0)
        self.history[-1] = frame
        return self.history.reshape(-1).copy()

    def record_action(self, action_legs: np.ndarray) -> np.ndarray:
        """Clip the policy's 12 outputs as the environment does and keep them (plus the held arm's zeros) for the next
        frame; returns the clipped 12."""
        legs = np.clip(np.asarray(action_legs, dtype=float).reshape(-1), -self.clip_actions, self.clip_actions)
        self.prev_actions = np.concatenate((legs, np.zeros(self.num_actions - self.num_legs)))
        return legs

    def leg_targets_unitree(self, action_legs: np.ndarray) -> np.ndarray:
        """The 12 joint targets the policy asks for (default + scale x action), in Unitree's order."""
        targets = np.asarray(action_legs, dtype=float) * self.meta["action_scale"] + self.default[:self.num_legs]
        return targets[self.to_unitree]


def waiting_command(meta: dict, foot_goal_local=None) -> np.ndarray:
    """The 15 commands the policy sees while it waits to lift (`leg_env` WAITING): no walking, the foot's goal at its
    standing footfall (`leg_cfg.KEY_DOWN_GOAL`, in the yaw frame from the ground under the base) as UniFP's
    (radius, pitch, yaw) about the goal centre (`leg_cfg.GOAL_CENTRE`), and no force. `foot_goal_local` replaces the
    footfall with another point in that frame (a diagnostic: the policy is open loop in shadow mode).

    The frame a reset returns has all-zero commands instead (the environment builds it before any step writes them;
    `check_shadow.py --sim` shows it), so a run that starts as the frozen evaluation does sends zeros first."""
    goal = np.asarray(meta["key_down_goal"] if foot_goal_local is None else foot_goal_local, dtype=float)
    commands = np.zeros(int(meta["num_commands"]))
    i = int(meta["cmd_ee_radius"])
    commands[i:i + 3] = cart_to_sphere(goal - np.asarray(meta["goal_centre"], dtype=float))
    return commands


def decode_estimates(meta: dict, estimate: np.ndarray) -> dict:
    """The decoder's 12 estimates in SI (`observations.PRIVILEGED_LAYOUT`'s first four blocks): the base's velocity in
    its body frame (m/s), the FR foot in the yaw frame from the ground under the base (m), and the external forces on
    the foot and the base in the yaw frame (N)."""
    e = np.asarray(estimate, dtype=float).reshape(-1)
    sphere = e[3:6] / np.asarray(meta["ee_sphere_scale"], dtype=float)
    foot = np.asarray(meta["goal_centre"], dtype=float) + sphere_to_cart(*sphere)
    return {"base_lin_vel_b": e[0:3] / meta["commands_scale"][0], "foot_local": foot,
            "foot_force": e[6:9] / meta["obs_scale_ee_force"], "base_force": e[9:12] / meta["obs_scale_base_force"]}


# --- the Go2's leg, for where the targets would put the foot ---------------------------------------------------------

def foot_local(leg: str, q, meta: dict) -> np.ndarray:
    """A foot sphere's centre in the base frame for one leg's (hip, thigh, calf), from the export's URDF numbers."""
    k = meta["kinematics"]
    sx = 1.0 if leg[0] == "F" else -1.0
    sy = 1.0 if leg[1] == "L" else -1.0
    hip, thigh, calf = (float(v) for v in q)
    link = k["link_m"]
    x = -link * math.sin(thigh) - link * math.sin(thigh + calf) + k["foot_sphere_x"] * math.cos(thigh + calf)
    z = -link * math.cos(thigh) - link * math.cos(thigh + calf) + k["foot_sphere_x"] * math.sin(thigh + calf)
    y = sy * k["thigh_y"]
    c, s = math.cos(hip), math.sin(hip)
    return np.array([sx * k["hip_x"] + x, sy * k["hip_y"] + c * y - s * z, s * y + c * z])
