"""The dog's numpy mechanism layer (`demos/combiner_key/robot/mech_layer.py`) against the training code, no simulator.

`unifp_train/mech_env.py` imports Isaac Lab, so its `Go2D1MechEnv` class is compiled from its source onto a CPU stand-in
for the base environment (`_FakeBase`: DirectRLEnv's step order, `env.py`'s goal and tool helpers compiled from its
source, and a scripted robot), with the real `Mechanism`, `TravelGoal`, `EeGoalTrajectory` and `HeldGoals` (compiled
from `hook_env.py`). On that stand-in: the numpy pieces one by one against the torch ones (the path, the plan, UniFP's
approach, the law, the grasp's rule, the goal on the handle, the evaluation's paths); `run_unifp_train.mech_evaluate`
with and without `--mech_shadow_dump` (the dump is read-only); and `check_mech.check_sim` on the dump (the whole runner
against the environment, step for step). Then the export's meta and refusals, and `mech_gate.py`'s replay on a still
stand. Needs the Isaac environment's torch (CPU); the export of a random actor needs rsl-rl.

The scripted robot is not a robot: it says nothing about what a policy does, only that the layer computes what the
environment computes from the same state.
"""
from __future__ import annotations

import __future__
import ast
import importlib.util
import json
import math
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path

import numpy as np
import torch

from demos.combiner_key.robot import export_policy, leg_policy, mech_layer
from unifp_isaaclab import interface, task as goal_task
from unifp_isaaclab.policy import ObsHistory
from unifp_train import fixture, hook_cfg, hook_rewards, mech_cfg, mech_eval, mech_rewards, mechanism, rewards, task_cfg

REPO = Path(__file__).resolve().parents[1]
MECH_SRC = REPO / "unifp_train" / "mech_env.py"
HOOK_SRC = REPO / "unifp_train" / "hook_env.py"
ENV_SRC = REPO / "unifp_train" / "env.py"
F32 = np.float32


def _compile(path: Path, node, namespace: dict) -> dict:
    code = compile(ast.Module(body=[node], type_ignores=[]), str(path), "exec",
                   flags=__future__.annotations.compiler_flag, dont_inherit=True)
    scope = dict(namespace)
    exec(code, scope)
    return scope


def _source_class(path: Path, name: str, namespace: dict):
    node = next(n for n in ast.walk(ast.parse(path.read_text())) if isinstance(n, ast.ClassDef) and n.name == name)
    return _compile(path, node, namespace)[name]


def _source_functions(path: Path, names, namespace: dict) -> dict:
    found = {}
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.FunctionDef) and node.name in names and node.name not in found:
            found[node.name] = _compile(path, node, namespace)[node.name]
    missing = [n for n in names if n not in found]
    if missing:
        raise AssertionError(f"{path.name} has no {missing}")
    return found


HeldGoals = _source_class(HOOK_SRC, "HeldGoals", {"goal_task": goal_task, "torch": torch})
_ENV_HELPERS = _source_functions(ENV_SRC, ("_tip_pos", "_goal_centre", "_goal_world", "_base_yaw_quat", "_dof",
                                           "_contact_forces"),
                                 {"torch": torch, "interface": interface, "goal_task": goal_task})


class _FakeBase:
    """`Go2D1PosForceEnv` as far as `Go2D1MechEnv` reaches into it, on the CPU, in DirectRLEnv's step order, with a
    scripted robot: the tool point pulled toward UniFP's target (the goal plus the force command / 200 N/m, base yaw
    frame) by a 400 N/m spring against 60 N*s/m and pushed by the grasp's force, capped at 0.6 m/s; the base drifting
    and turning a little; the joints wandering (unrelated to the tool)."""

    def __init__(self, cfg, render_mode=None, *, num_envs: int, episode_steps: int, seed: int):
        n = num_envs
        self.cfg, self.num_envs, self.device = cfg, n, torch.device("cpu")
        self.max_episode_length = episode_steps
        self.physics_dt = interface.SIM_DT
        self.episode_length_buf = torch.zeros(n, dtype=torch.long)
        self.common_step_counter = 0
        self.extras = {}
        self._gen = torch.Generator().manual_seed(seed)
        names = list(interface.ISAACLAB_NAMES)
        self._order = torch.arange(len(names))
        self._default_dof_pos = torch.as_tensor(interface.DEFAULT_DOF_POS, dtype=torch.float32)
        identity = torch.tensor([1.0, 0.0, 0.0, 0.0])
        data = types.SimpleNamespace(
            joint_pos=self._default_dof_pos.expand(n, -1).clone(), joint_vel=torch.zeros(n, len(names)),
            root_pos_w=torch.zeros(n, 3), root_quat_w=identity.repeat(n, 1), root_ang_vel_b=torch.zeros(n, 3),
            applied_torque=torch.zeros(n, len(names)), body_pos_w=torch.zeros(n, 2, 3),
            body_quat_w=identity.repeat(n, 2, 1), body_lin_vel_w=torch.zeros(n, 2, 3), body_ang_vel_w=torch.zeros(n, 2, 3))
        self._robot = types.SimpleNamespace(data=data, joint_names=names, body_names=["base", "Link6"],
                                            _ALL_INDICES=torch.arange(n))
        self._tip_body = 1
        self.scene = types.SimpleNamespace(env_origins=torch.stack(
            (torch.arange(n) * 2.5, torch.arange(n) * -1.5, torch.zeros(n)), -1).float())
        self._commands = torch.zeros(n, interface.NUM_COMMANDS)
        self._actions = torch.zeros(n, interface.NUM_ACTIONS)
        self._joint_targets = self._default_dof_pos[:18].expand(n, -1).clone()
        self._gait_phase = torch.zeros(n)
        self._ee_force_w = torch.zeros(n, 3)
        self._gripper_force_kp = torch.full((n, 3), task_cfg.GRIPPER_FORCE_KP)
        self._weights, self._terms, self._episode_sums = {}, {}, {}
        self._goals = goal_task.EeGoalTrajectory(n, device="cpu",
                                                 roll_range=task_cfg.EE_ROLL_RANGE_RAD if cfg.roll_objective else None)
        self._actor_history = ObsHistory(n)
        self._arm_link = None
        self._contact_sensor = types.SimpleNamespace(body_names=["FL_foot", "FR_foot", "RL_foot", "RR_foot", "base"],
                                                     data=types.SimpleNamespace(net_forces_w=torch.zeros(n, 5, 3)))
        self._feet_contacts = [0, 1, 2, 3]
        self._arm_contact = types.SimpleNamespace(body_names=["Link1", "Link6"],
                                                  data=types.SimpleNamespace(net_forces_w=torch.zeros(n, 2, 3)))
        self._tool_w = torch.zeros(n, 3)
        self._tool_v = torch.zeros(n, 3)
        self._rpy = torch.zeros(n, 3)
        self._base_v = torch.zeros(n, 3)
        self._t = 0.0

    def _rand(self, *shape) -> torch.Tensor:
        return torch.randn(*shape, generator=self._gen)

    # --- the robot ---------------------------------------------------------------------------------------------------

    def _set_root(self) -> None:
        r, p, y = self._rpy.unbind(-1)
        cr, sr, cp, sp, cy, sy = (torch.cos(r / 2), torch.sin(r / 2), torch.cos(p / 2), torch.sin(p / 2),
                                  torch.cos(y / 2), torch.sin(y / 2))
        self._robot.data.root_quat_w = torch.stack((cy * cp * cr + sy * sp * sr, cy * cp * sr - sy * sp * cr,
                                                    cy * sp * cr + sy * cp * sr, sy * cp * cr - cy * sp * sr), -1)

    def _set_tool(self) -> None:
        offset = torch.as_tensor(self.cfg.tool_offset_m).expand(self.num_envs, 3)
        self._robot.data.body_pos_w[:, 1] = self._tool_w - offset
        self._robot.data.body_lin_vel_w[:, 1] = self._tool_v

    def _physics(self) -> None:
        dt, n, data = interface.SIM_DT, self.num_envs, self._robot.data
        self._t += dt
        target = (self._goal_world() + self.scene.env_origins
                  + interface.quat_apply(self._base_yaw_quat(), self._commands[:, interface.CMD_EE_FORCE]) / 200.0)
        v = (400.0 * (target - self._tool_w) + self._ee_force_w) / 60.0
        speed = v.norm(dim=-1, keepdim=True)
        v = torch.where(speed > 0.6, v * 0.6 / speed.clamp(min=1e-9), v)
        self._tool_w = self._tool_w + v * dt
        self._tool_v = v
        self._base_v = 0.98 * self._base_v + 0.02 * 0.03 * self._rand(n, 3) * torch.tensor([1.0, 1.0, 0.2])
        data.root_pos_w = data.root_pos_w + self._base_v * dt
        rate = torch.stack((0.02 * torch.cos(torch.full((n,), 3.0 * self._t)), -0.03 * torch.sin(torch.full((n,), 2.0 * self._t)),
                            0.04 * torch.sin(torch.full((n,), 0.7 * self._t) + torch.arange(n))), -1)
        self._rpy = self._rpy + rate * dt
        self._set_root()
        data.root_ang_vel_b = rate + 0.01 * self._rand(n, 3)
        data.joint_pos = data.joint_pos + 0.002 * self._rand(n, 20)
        data.joint_vel = 0.1 * self._rand(n, 20)
        fz = torch.full((n, 4), 40.0)
        fz[:, 1] = torch.where(torch.sin(torch.full((n,), 5.0 * self._t)) > 0.8, 0.0, 40.0)
        self._contact_sensor.data.net_forces_w[:, :4, 2] = fz
        self._set_tool()

    # --- the environment's surface -------------------------------------------------------------------------------------

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self._last_actions = self._actions
        self._actions = torch.clip(actions, -interface.CLIP_ACTIONS, interface.CLIP_ACTIONS)
        self._joint_targets = interface.joint_targets(self._actions)
        self._step_forces()

    def _apply_action(self) -> None:
        pass

    def _write_wrench(self) -> None:
        pass

    def _get_dones(self):
        fallen = torch.zeros(self.num_envs, dtype=torch.bool)
        return fallen, self.episode_length_buf >= self.max_episode_length - 1

    def _get_rewards(self) -> torch.Tensor:
        self._gait_phase = interface.gait_step(self._gait_phase, self._commands)
        self._commands[:, interface.CMD_EE_RADIUS:interface.CMD_EE_YAW + 1] = self._goals.step()
        if self.cfg.roll_objective:
            self._commands[:, interface.CMD_EE_ORN_R:interface.CMD_EE_ORN_P + 1] = rewards.encode_roll(
                self._goals.current_roll)
        return torch.zeros(self.num_envs)

    def _resample_velocity_commands(self, env_ids: torch.Tensor) -> None:
        self._commands[env_ids, :3] = 0.0

    def _reset_idx(self, env_ids: torch.Tensor) -> None:
        data, n = self._robot.data, len(env_ids)
        joint = self._default_dof_pos.expand(n, -1).clone()
        joint[:, :12] += torch.tensor(task_cfg.DOG_STANCE_LEG_OFFSETS)
        data.joint_pos[env_ids] = joint
        data.joint_vel[env_ids] = 0.0
        data.root_pos_w[env_ids] = self.scene.env_origins[env_ids] + torch.tensor([0.0, 0.0, task_cfg.DOG_STANCE_BASE_M])
        self._rpy[env_ids] = torch.tensor([math.radians(task_cfg.DOG_STANCE_ROLL_DEG),
                                           math.radians(task_cfg.DOG_STANCE_PITCH_DEG), 0.0])
        self._set_root()
        start = goal_task.sphere_goal_to_world(torch.tensor([interface.EE_GOAL_INIT_START]).expand(n, 3),
                                               data.root_pos_w[env_ids], self._base_yaw_quat()[env_ids])
        self._tool_w[env_ids] = start
        self._tool_v[env_ids] = 0.0
        self._set_tool()
        self._actions[env_ids] = 0.0
        self._commands[env_ids] = 0.0
        self._ee_force_w[env_ids] = 0.0
        self._goals.reset(env_ids)
        self._resample_velocity_commands(env_ids)
        self._actor_history.reset(env_ids)
        self.episode_length_buf[env_ids] = 0

    def _get_observations(self) -> dict:
        dof_pos, dof_vel = self._dof()
        actor = interface.single_obs(self._robot.data.root_quat_w, self._robot.data.root_ang_vel_b, dof_pos, dof_vel,
                                     self._actions, self._gait_phase, self._commands)
        return {"policy": self._actor_history.append(torch.clip(actor, -interface.CLIP_OBSERVATIONS,
                                                                interface.CLIP_OBSERVATIONS))}

    def reset(self):
        self._reset_idx(None)
        return self._get_observations(), {}

    def step(self, actions: torch.Tensor):
        self._pre_physics_step(actions)
        for _ in range(interface.DECIMATION):
            self._apply_action()
            self._physics()
        self.episode_length_buf += 1
        self.common_step_counter += 1
        terminated, truncated = self._get_dones()
        reward = self._get_rewards()
        ids = (terminated | truncated).nonzero(as_tuple=False).flatten()
        if len(ids) > 0:
            self._reset_idx(ids)
        return self._get_observations(), reward, terminated, truncated, {}


for _name, _fn in _ENV_HELPERS.items():
    setattr(_FakeBase, _name, _fn)

MechEnv = _source_class(MECH_SRC, "Go2D1MechEnv", {
    "torch": torch, "math": math, "interface": interface, "fixture": fixture, "hook_cfg": hook_cfg,
    "hook_rewards": hook_rewards, "mech_cfg": mech_cfg, "mech_rewards": mech_rewards, "mechanism": mechanism,
    "task_cfg": task_cfg, "Go2D1PosForceEnv": _FakeBase, "HeldGoals": HeldGoals,
    "ContactSensor": None, "Go2D1MechEnvCfg": None})


def _cfg(**kw):
    cfg = dict(roll_objective=True, tool_body="Link6", tool_offset_m=task_cfg.TOOL_OFFSET_M, force_law=False,
               force_law_bleed_s=None, law_variant="v1", force_law_gains=(400.0, 1000.0, 80.0), offpath_weight=0.0,
               peak_ceiling_n=15.0, peak_ceiling_max_n=80.0, curriculum=True, mechanism_fraction=0.8,
               opening_probs=mech_cfg.OPENING_PROBS, lever_fraction=0.0, lever_handle_pitch_range=hook_cfg.HANDLE_PITCH_RANGE,
               tilt_stop_rad=None, dog_stance_fraction=1.0, action_space=18)
    cfg.update(kw)
    return types.SimpleNamespace(**cfg)


#: 10 N opens with the handle close behind the reference (the bleed's region); 200 N never does, so the law winds to
#: its cap (the stand-in's robot pulls about 1.7 x the command, under the 150 N grip).
KINDS, LEVELS = ["drawer", "latch", "door", "button"], [10.0, 200.0]


def _env(seed: int = 3, steps: int = 330, law=(400.0, 1000.0, 80.0), bleed=1.0, **cfg):
    """The evaluation's environment on the stand-in: S4b validation's first two placements, the four kinds at 10 and
    200 N, the law with the bleed, as `run_unifp_train.py mech_eval --force_law --force_law_bleed 1.0` sets it; `cfg`
    sets config fields (S4c's `grasp_within_m`)."""
    torch.manual_seed(seed)
    placements = mech_cfg.s4_set("validation", mech_cfg.S4B_MANIFEST)[0][:2]
    n = len(KINDS) * len(LEVELS) * len(placements)
    env = MechEnv(_cfg(**cfg), None, num_envs=n, episode_steps=steps, seed=seed)
    env.set_evaluation(*mech_eval.build_plan(KINDS, LEVELS, "cpu", placements), conditions=dict(mech_cfg.DEV_CONDITIONS))
    env.set_force_law(*law, bleed_s=bleed)
    return env, placements


def _meta(tmp: Path, env_yaml: str | None = None, arguments: dict | None = None, **record_kw) -> dict:
    """A mechanism export's meta without a network: a fake training run beside a stand-in checkpoint file. `env_yaml` is
    the run's env.yaml (none without it), `arguments` added to its run.json's."""
    run = tmp / "run"
    run.mkdir(exist_ok=True)
    (run / "model_1.pt").write_bytes(b"not a network")
    record = {"task": "mechanism", "tool_body": "Link6", "tool_offset_m": list(task_cfg.TOOL_OFFSET_M),
              "roll_objective": True, "legs": {"dog_stance_fraction": 0.75}, "leg": None,
              "mechanism": {"force_law": True, "law_variant": "v1", "force_law_bleed_s": 1.0,
                            "force_law_gains": [400.0, 1000.0, 80.0]},
              "arguments": {"task": "mechanism", "force_law": True, "law_variant": "v1", "force_law_bleed": 1.0,
                            "payload": "physical", "tool_point": "jaw_centre", "mech_tilt_stop_deg": 15.0,
                            **(arguments or {})}}
    record.update(record_kw)
    (run / "run.json").write_text(json.dumps(record))
    if env_yaml is not None:
        (run / "env.yaml").write_text(env_yaml)
    elif (run / "env.yaml").exists():
        (run / "env.yaml").unlink()
    return json.loads(json.dumps(export_policy.meta_for("force", run / "model_1.pt", 1, interface.NUM_ACTIONS,
                                                        {"encoder": 4, "actor": 4, "decoder": 3}, task="mechanism",
                                                        train_run=run)))


def _gated_meta(tmp: Path, radius: float) -> dict:
    """`_meta` of an S4c run: env.yaml with the gate (and Isaac Lab's tuple tags), run.json's flag beside it."""
    tmp.mkdir(parents=True, exist_ok=True)
    yaml_text = (f"viewer:\n  eye: !!python/tuple\n  - 7.5\n  - 7.5\n  - 7.5\ntilt_stop_rad: 0.26\n"
                 f"grasp_within_m: {radius!r}\nengage_max_wait_steps: null\napproach_weight: 1.0\n")
    return _meta(tmp, env_yaml=yaml_text, arguments={"mech_grasp_within_m": radius, "mech_engage_max_wait_s": None})


def _install_shadow() -> None:
    """`unifp_train.mech_shadow` as the copy at $D1_MECH_SHADOW_SRC, if set (to test an edit before it replaces the
    live file while a queue imports it); `run_unifp_train.mech_evaluate` imports it from `sys.modules`."""
    path = os.environ.get("D1_MECH_SHADOW_SRC")
    if path:
        spec = importlib.util.spec_from_file_location("unifp_train.mech_shadow", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        sys.modules["unifp_train.mech_shadow"] = module


def _dump_run(R, tmp: Path, name: str, seed: int = 3, **cfg) -> Path:
    """`mech_evaluate` with a 16-environment dump on the stand-in (`_env(**cfg)`); the dump's path."""
    env, placements = _env(seed=seed, **cfg)
    obs, _ = env.reset()
    run_dir = tmp / name
    args = types.SimpleNamespace(zero_actions=True, checkpoint=None, mech_set="s4b_validation", mech_test=False,
                                 lever_high=False, mech_shadow_dump=16)
    R.mech_evaluate(args, env, obs, {}, run_dir, KINDS, LEVELS, placements)
    return run_dir / "mech_shadow_dump.npz"


def _runner_module():
    """`run_unifp_train` (or the copy at $D1_RUN_UNIFP_TRAIN, to test an edit before it goes in place)."""
    path = os.environ.get("D1_RUN_UNIFP_TRAIN")
    if not path:
        import run_unifp_train
        return run_unifp_train
    spec = importlib.util.spec_from_file_location("run_unifp_train_staged", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# --- the pieces --------------------------------------------------------------------------------------------------------

class Path_(unittest.TestCase):
    def test_path_is_the_mechanisms(self):
        rng = np.random.default_rng(1)
        for hinge in (False, True):
            for _ in range(20):
                point, opening = rng.normal(size=3), rng.normal(size=3)
                axis = np.cross(opening, rng.normal(size=3))
                radius, travel = float(rng.uniform(0.1, 0.4)), float(rng.uniform(0.02, 0.3))
                m = mechanism.Mechanism(1, "cpu", grasp_damping=10.0)
                t = lambda x: torch.as_tensor(np.asarray(x), dtype=torch.float32)[None]
                one = lambda v: torch.full((1,), float(v))
                m.grasp(torch.arange(1), t(point), hinge=torch.tensor([hinge]), opening=t(opening), axis=t(axis),
                        radius=one(radius), travel=one(travel), mass=one(1), spring_n=one(0), spring_k=one(0),
                        damping=one(0), friction_static=one(0), friction_kinetic=one(0), latch_n=one(0),
                        grasp_k=one(2000), grip_n=one(150))
                path = mech_layer.MechPath.from_grasp(hinge, point, opening, axis, radius, travel)
                for s in rng.uniform(0.0, travel, 5):
                    np.testing.assert_allclose(path.position(s), m.position(one(s))[0].numpy(), atol=3e-6)
                    np.testing.assert_allclose(path.tangent(s), m.tangent(one(s))[0].numpy(), atol=3e-6)
                    # A point on the path reads back its own s; one moved straight off it reads the same s.
                    got, off, _ = path.project(path.position(s))
                    self.assertAlmostEqual(got, s, places=9)
                    self.assertLess(off, 1e-9)
                    normal = np.cross(path.tangent(s), path.axis if hinge else rng.normal(size=3))
                    normal /= np.linalg.norm(normal)
                    if hinge:
                        normal = path.axis
                    got, off, _ = path.project(path.position(s) + 0.01 * normal)
                    self.assertAlmostEqual(got, s, places=9)
                    self.assertAlmostEqual(off, 0.01, places=9)
                # Past the stops it clamps.
                self.assertEqual(path.project(path.position(0.0) - 0.05 * path.tangent(0.0))[0], 0.0)

    def test_world_and_sphere(self):
        rng = np.random.default_rng(2)
        for _ in range(20):
            base = rng.normal(size=3) * [1, 1, 0.05] + [0, 0, 0.29]
            q = rng.normal(size=4)
            q /= np.linalg.norm(q)
            point = base + rng.normal(size=3) * 0.3
            sphere = mech_layer.world_to_sphere(point, base, q, interface.EE_GOAL_CENTER_OFFSET)
            tq = torch.as_tensor(q)[None]
            yaw = interface.yaw_quat(interface.yaw_from_quat(tq))
            centre = goal_task.goal_sphere_center(torch.as_tensor(base)[None], yaw)
            ref = interface.cart2sphere(interface.quat_rotate_inverse(yaw, torch.as_tensor(point)[None] - centre))[0]
            np.testing.assert_allclose(sphere, ref.numpy(), atol=1e-12)
            np.testing.assert_allclose(mech_layer.sphere_to_world(sphere, base, q, interface.EE_GOAL_CENTER_OFFSET),
                                       point, atol=1e-12)


class Plan(unittest.TestCase):
    def test_travel_plan_is_travel_goal_bit_for_bit(self):
        rng = np.random.default_rng(3)
        for fraction, speed, travel in ((1.0, 0.10, 0.12), (1.0, 0.15, 0.015), (0.6, 0.07, 0.1745), (0.0, 0.1, 0.03)):
            tg = mechanism.TravelGoal(1, "cpu", speed_m_s=(0.05, 0.2), hold_s=(1.0, 3.0), first_fraction=(0.6, 1.0),
                                      dt=interface.POLICY_DT)
            tg.use_plan(fraction, speed)
            tg.start(torch.arange(1), torch.full((1,), travel))
            plan = mech_layer.TravelPlan(fraction, speed, interface.POLICY_DT)
            plan.start(float(F32(travel)))
            self.assertEqual(F32(plan.target), tg.target[0].numpy())
            for _ in range(200):
                active = bool(rng.random() < 0.9)
                tg.step(torch.tensor([active]), torch.full((1,), travel))
                plan.step(active)
                self.assertEqual(F32(plan.ref), tg.ref[0].numpy())

    def test_approach_is_ee_goal_trajectory(self):
        torch.manual_seed(4)
        n = 6
        goals = HeldGoals(n, device="cpu", roll_range=task_cfg.EE_ROLL_RANGE_RAD)
        handles = torch.tensor(mech_cfg.s4_set("validation", mech_cfg.S4B_MANIFEST)[0][:n])
        goals.reset()
        goals.aim(torch.arange(n), handles, torch.zeros(n))
        layers = []
        for e in range(n):
            a = mech_layer.ApproachGoal(interface.EE_GOAL_INIT_START)
            a.reset(handles[e].numpy(), float(goals.traj_steps[e]))
            layers.append(a)
        for _ in range(int(goals.traj_steps.min()) + 20):       # inside every robot's hold: no resample yet
            current = goals.step()
            for e, a in enumerate(layers):
                np.testing.assert_allclose(a.step(), current[e].numpy(), atol=1e-7)
            np.testing.assert_allclose(goals.current_roll.numpy(), 0.0, atol=0.0)


class Law(unittest.TestCase):
    """`ForceLaw` and `MechLayer.law_step` against `Go2D1MechEnv._step_forces` on random sequences."""

    def _torch_law(self, n, kp, ki, mx, bleed):
        fns = _source_functions(MECH_SRC, ("_step_forces",), {"torch": torch, "math": math, "interface": interface,
                                                              "mech_cfg": mech_cfg})
        env = types.SimpleNamespace(_force_law={"kp": kp, "ki": ki, "max": mx, "bleed_s": bleed},
                                    _lag_integral=torch.zeros(n), _law_max=torch.full((n,), mx),
                                    _law_command_w=torch.zeros(n, 3), _commands=torch.zeros(n, 15))
        env._mech = types.SimpleNamespace(grasped=torch.zeros(n, dtype=torch.bool), s=torch.zeros(n),
                                          tangent=lambda: env._tangent)
        env._travel_goal = types.SimpleNamespace(ref=torch.zeros(n))
        env._tangent = torch.zeros(n, 3)
        env._yaw = torch.tensor([[1.0, 0, 0, 0]]).repeat(n, 1)
        env._base_yaw_quat = lambda: env._yaw
        return env, lambda: fns["_step_forces"](env)

    def test_law_is_step_forces(self):
        rng = np.random.default_rng(5)
        for kp, ki, mx, bleed in ((400.0, 1000.0, 80.0, 1.0), (400.0, 1000.0, 40.0, 1.0), (400.0, 1000.0, 80.0, None),
                                  (250.0, 600.0, 30.0, 0.5)):
            n = 16
            env, step = self._torch_law(n, kp, ki, mx, bleed)
            laws = [mech_layer.ForceLaw(kp, ki, mx, bleed, mech_cfg.FORCE_LAW_ARRIVED_M, interface.POLICY_DT)
                    for _ in range(n)]
            ref, s = np.zeros(n, F32), np.zeros(n, F32)
            held = np.zeros(n, bool)
            clamped = bled = 0
            for k in range(400):
                held = held | (rng.random(n) < 0.05)
                # A reference that runs, stops and backs off; a handle that lags, sticks and jumps; exact float32 lags at
                # the bleed's threshold now and then.
                ref = (ref + F32(0.002) * (rng.random(n) < 0.7) - F32(0.001) * (rng.random(n) < 0.1)).astype(F32)
                stuck = (np.arange(n) < 4) & (100 <= k < 250)       # four handles stuck for 3 s: the integral winds up
                s = np.where(stuck | (rng.random(n) < 0.3), s, ref - rng.normal(0.01, 0.02, n)).astype(F32)
                edge = rng.random(n) < 0.05
                s = np.where(edge, (ref - F32(mech_cfg.FORCE_LAW_ARRIVED_M)).astype(F32), s)
                env._travel_goal.ref = torch.as_tensor(ref)
                env._mech.s = torch.as_tensor(s)
                env._mech.grasped = torch.as_tensor(held)
                tang = rng.normal(size=(n, 3))
                env._tangent = torch.as_tensor(tang / np.linalg.norm(tang, axis=1, keepdims=True), dtype=torch.float32)
                yaw = rng.uniform(-0.5, 0.5, n)
                env._yaw = torch.as_tensor(np.stack((np.cos(yaw / 2), 0 * yaw, 0 * yaw, np.sin(yaw / 2)), -1), dtype=torch.float32)
                step()
                for e, law in enumerate(laws):
                    f = law.step(ref[e], s[e], bool(held[e]))
                    clamped += int(abs(float(f)) >= mx - 1e-4)
                    bled += int(bool(held[e]) and bleed is not None and abs(law.lag) < F32(mech_cfg.FORCE_LAW_ARRIVED_M))
                    self.assertEqual(F32(law.integral), env._lag_integral[e].numpy(), msg=f"step {k} env {e}")
                    vec = float(f) * env._tangent[e].double().numpy() * float(held[e])
                    np.testing.assert_allclose(vec, env._law_command_w[e].numpy(), atol=2e-5)
                    cmd = interface.quat_rotate_inverse(env._yaw[e:e + 1].double(), torch.as_tensor(vec)[None])[0]
                    np.testing.assert_allclose(cmd.numpy(), env._commands[e, 9:12].numpy(), atol=5e-5)
            self.assertGreater(clamped, 0, "the clamp was never reached")
            if bleed is not None:
                self.assertGreater(bled, 0, "the bleed never ran")


class EvaluationPaths(unittest.TestCase):
    def test_gate_paths_are_draw_evaluation(self):
        """`mech_gate.eval_path` against `_directions` + `_draw_evaluation` + `Mechanism.grasp`, compiled from source."""
        from demos.combiner_key.robot import mech_gate

        env, _ = _env(steps=5)
        rng = np.random.default_rng(6)
        kinds = list(mech_cfg.EVAL_KINDS)
        for kind in ("drawer", "latch", "door", "button", "lid", "bolt"):
            n = env.num_envs
            point = torch.as_tensor(rng.normal(size=(n, 3)) * [0.1, 0.25, 0.1] + [0.6, 0.0, 0.45], dtype=torch.float32)
            env._robot.data.root_pos_w = env.scene.env_origins + torch.as_tensor(rng.normal(size=(n, 3)) * 0.02,
                                                                                 dtype=torch.float32)
            handles = torch.as_tensor(np.stack((np.full(n, 0.45), np.zeros(n), rng.uniform(-0.5, 0.5, n)), -1),
                                      dtype=torch.float32)
            env._evaluation = {**env._evaluation, "kinds": torch.full((n,), kinds.index(kind)), "handles": handles,
                               "levels": torch.full((n,), 30.0)}
            ids = torch.arange(n)
            draw = env._draw_evaluation(ids, point)
            m = mechanism.Mechanism(n, "cpu", grasp_damping=10.0)
            m.grasp(ids, point, **draw)
            for e in range(n):
                path = mech_gate.eval_path(mech_cfg.EVAL_KINDS[kind], point[e].numpy(),
                                           (env._robot.data.root_pos_w[e] - env.scene.env_origins[e]).numpy(),
                                           float(handles[e, 2]))
                self.assertEqual(path.hinge, bool(m.hinge[e]))
                self.assertAlmostEqual(path.travel, float(m.travel[e]), places=6)
                for s in np.linspace(0.0, path.travel, 5):
                    sv = torch.full((n,), float(s))
                    np.testing.assert_allclose(path.position(s), m.position(sv)[e].numpy(), atol=2e-6)
                    np.testing.assert_allclose(path.tangent(s), m.tangent(sv)[e].numpy(), atol=2e-6)


# --- the environment's step, the dump and the replay ---------------------------------------------------------------

def _zero_policy(meta):
    return types.SimpleNamespace(meta=meta, act=lambda stack: np.zeros((1, 18)))


class Rollout(unittest.TestCase):
    """`mech_evaluate` on the stand-in with and without the dump, then `check_mech` on the dump."""

    @classmethod
    def setUpClass(cls):
        _install_shadow()
        cls.R = _runner_module()
        cls.tmp = tempfile.TemporaryDirectory()
        cls.meta = _meta(Path(cls.tmp.name))
        cls.runs = {}
        for name, dump in (("plain", 0), ("dump", 16)):
            env, placements = _env()
            obs, _ = env.reset()
            run_dir = Path(cls.tmp.name) / name
            args = types.SimpleNamespace(zero_actions=True, checkpoint=None, mech_set="s4b_validation", mech_test=False,
                                         lever_high=False, mech_shadow_dump=dump)
            report = {}
            status = cls.R.mech_evaluate(args, env, obs, report, run_dir, KINDS, LEVELS, placements)
            cls.runs[name] = (status, report, run_dir)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_the_dump_is_read_only(self):
        (s0, r0, d0), (s1, r1, d1) = self.runs["plain"], self.runs["dump"]
        self.assertEqual(s0, s1)
        a, b = np.load(d0 / "mech_trace.npz"), np.load(d1 / "mech_trace.npz")
        self.assertEqual(sorted(a.files), sorted(b.files))
        for key in a.files:
            np.testing.assert_array_equal(a[key], b[key], err_msg=key)
        self.assertEqual(json.loads((d0 / "mech_eval.json").read_text()), json.loads((d1 / "mech_eval.json").read_text()))
        self.assertNotIn("mech_shadow_dump", r0)
        self.assertEqual(len(r1["mech_shadow_dump"]["env_ids"]), 16)
        self.assertFalse((d0 / "mech_shadow_dump.npz").exists())

    def test_the_rollout_exercises_the_layer(self):
        d = dict(np.load(self.runs["dump"][2] / "mech_shadow_dump.npz"))
        alive = d["alive"] > 0.5
        grasped = (d["mech_grasped"] > 0.5) & alive
        self.assertGreaterEqual(int(grasped.any(axis=1).sum()), 12, "most episodes grasp")
        self.assertTrue(((np.abs(d["force_cmd_after"]).max(-1) > 1.0) & alive).any())
        force = np.linalg.norm(d["law_command_w_after"], axis=-1)
        self.assertTrue((force[alive] >= 80.0 - 1e-3).any(), "the law reaches its cap somewhere")
        self.assertTrue((d["mech_s"][grasped] > 0.01).any(), "handles move")

    def test_check_mech_reproduces_the_rollout(self):
        from demos.combiner_key.robot import check_mech

        out = check_mech.check_sim(_zero_policy(self.meta), self.runs["dump"][2] / "mech_shadow_dump.npz")
        e = out["max_abs_err"]
        self.assertEqual(out["rule_mismatched_steps"], 0)
        for key in ("since_reset", "grasped", "engage_step", "ref_m", "target_m"):
            self.assertEqual(e[key], 0.0, key)
        self.assertLess(e["law_integral_m_s"], 1e-9)
        self.assertLess(e["law_force_w_n"], 1e-4)
        self.assertLess(e["law_force_b_n"], 1e-4)
        self.assertLess(e["commands_force_n"], 1e-4)
        self.assertLess(e["commands_goal_held"], 1e-5)
        self.assertLess(e["commands_goal_approach"], 1e-6)
        self.assertEqual(e["commands_velocity"], 0.0)
        self.assertLess(e["commands_roll"], 1e-7)
        self.assertLess(e["targets_raw_rad"], 1e-6)
        self.assertLess(out["frame_max_abs_err"], 1e-5)
        self.assertEqual(e["action"], 0.0)
        # The dog's reading ran; the stand-in's joints are unrelated to its tool, so only the tip's is meaningful: the
        # grasp spring's stretch along the path, signed, about the law's force / 2,000 N/m.
        reading = [ep["dog_reading"] for ep in out["by_episode"] if "dog_reading" in ep]
        self.assertTrue(reading)
        for r in reading:
            self.assertLess(r["law_true_vs_isaac_n"], 1e-4)
            self.assertIsNotNone(r["tip"]["s_minus_true_m"])
        self.assertIn("drawer", out["dog_reading_summary"])
        self.assertEqual(out["episodes_resampled_before_grasp"], 0)
        np.testing.assert_array_less(e["goal_centre_m"], 1e-6)
        self.assertEqual({ep["ended_by"] for ep in out["by_episode"]}, {"time"})

    def test_a_goal_that_moves_on_before_the_grasp_is_compared_apart(self):
        """With the grasp's delay past UniFP's hold, the environment's goal moves on before the tool is held (a new
        goal and roll); the layer keeps the handle. Those episodes go apart, and the held phase still matches."""
        from demos.combiner_key.robot import check_mech

        env, placements = _env(seed=8)
        env._evaluation["engage_delay"] = 110
        obs, _ = env.reset()
        run_dir = Path(self.tmp.name) / "late"
        args = types.SimpleNamespace(zero_actions=True, checkpoint=None, mech_set="s4b_validation", mech_test=False,
                                     lever_high=False, mech_shadow_dump=16)
        self.R.mech_evaluate(args, env, obs, {}, run_dir, KINDS, LEVELS, placements)
        out = check_mech.check_sim(_zero_policy(self.meta), run_dir / "mech_shadow_dump.npz", dog=False)
        e = out["max_abs_err"]
        self.assertGreater(out["episodes_resampled_before_grasp"], 0)
        self.assertGreater(e["commands_goal_after_resample"], 1e-3)
        self.assertLess(e["law_integral_m_s"], 1e-9)
        self.assertEqual(e["ref_m"], 0.0)
        self.assertLess(e.get("commands_goal_held", 0.0), 1e-5)


class ArrivalGate(unittest.TestCase):
    """S4c's arrival gate (`grasp_within_m`): the layer's rule against `mech_env._engage_due`, compiled from its source,
    on every step of a gated rollout on the stand-in (through the dump and `check_mech`), and off unchanged."""

    @classmethod
    def setUpClass(cls):
        from demos.combiner_key.robot import check_mech
        _install_shadow()
        cls.CM = check_mech
        cls.R = _runner_module()
        cls.tmp = tempfile.TemporaryDirectory()
        tmp = Path(cls.tmp.name)
        cls.plain_dump = _dump_run(cls.R, tmp, "plain")
        # The radius: the ungated grasps' median error on this stand-in, so some grasps close inside it and the settle
        # rule fires outside it in the others.
        d = check_mech.load(cls.plain_dump)
        first = [int(np.flatnonzero(row > 0.5)[0]) for row in d["mech_grasped"] if (row > 0.5).any()]
        errors = [float(d["grasp_error"][e, t]) for e, t in zip(np.flatnonzero((d["mech_grasped"] > 0.5).any(1)), first)]
        cls.radius = float(np.median(errors))
        cls.gated_dump = _dump_run(cls.R, tmp, "gated", grasp_within_m=cls.radius)
        (tmp / "m_plain").mkdir()
        cls.meta_plain = _meta(tmp / "m_plain")
        cls.meta_gated = _gated_meta(tmp / "m_gated", cls.radius)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_the_export_carries_the_radius(self):
        self.assertEqual(self.meta_gated["mechanism"]["approach"]["grasp_within_m"], self.radius)
        self.assertIsNone(self.meta_plain["mechanism"]["approach"]["grasp_within_m"])
        layer = mech_layer.MechLayer(self.meta_gated["mechanism"])
        self.assertEqual(layer.grasp_within, F32(self.radius))
        self.assertIsNone(mech_layer.MechLayer(self.meta_gated["mechanism"], grasp_within_m=None).grasp_within)
        self.assertEqual(mech_layer.MechLayer(self.meta_plain["mechanism"], grasp_within_m=0.03).grasp_within, F32(0.03))

    def test_the_dump_records_the_gate(self):
        gated, plain = self.CM.load(self.gated_dump), self.CM.load(self.plain_dump)
        self.assertEqual(json.loads(str(gated["grasp_within_m"])), self.radius)
        self.assertIsNone(json.loads(str(plain["grasp_within_m"])))
        for d in (gated, plain):
            self.assertEqual(int(d["engage_max_wait_steps"]), hook_cfg.ENGAGE_MAX_WAIT_STEPS)

    def test_the_envs_distance_is_read_off_the_dump(self):
        """`check_mech.env_goal_world` is `_goal_world()` as `_engage_due` read it: it gives each grasp's recorded error."""
        d = self.CM.load(self.gated_dump)
        grasps = 0
        for e in range(d["frame"].shape[0]):
            g = np.flatnonzero(d["mech_grasped"][e] > 0.5)
            if len(g):
                dist = np.linalg.norm(d["tip_pos"][e, g[0]] - self.CM.env_goal_world(d, e, int(g[0])))
                self.assertAlmostEqual(dist, float(d["grasp_error"][e, g[0]]), delta=1e-6)
                self.assertLessEqual(F32(dist), F32(self.radius))
                grasps += 1
        self.assertGreater(grasps, 0)

    def test_the_layers_rule_is_the_envs_on_every_step(self):
        out = self.CM.check_sim(_zero_policy(self.meta_gated), self.gated_dump, dog=False)
        gate, e = out["grasp_gate"], out["max_abs_err"]
        self.assertEqual(out["rule_mismatched_steps"], 0)
        self.assertEqual(gate["grasp_within_m"], self.radius)
        self.assertEqual(gate["rollout"]["source"], "dump")
        # The gate did something: steps where the settle rule (or its fallback) would have grasped outside the radius,
        # and grasps it let through.
        self.assertGreater(gate["settled_steps_held_back"], 0)
        self.assertGreater(gate["grasps"], 0)
        # The same episodes ungated (the same seed, so the same until a grasp): no gated grasp comes earlier, some later.
        plain = self.CM.load(self.plain_dump)
        first = lambda d, e: (int(np.flatnonzero(d["mech_grasped"][e] > 0.5)[0]) if (d["mech_grasped"][e] > 0.5).any()
                              else 10 ** 6)
        steps = [(first(plain, ep["episode"]), ep["grasp_step_isaac"] if ep["grasp_step_isaac"] is not None else 10 ** 6)
                 for ep in out["by_episode"]]
        self.assertTrue(all(g >= p for p, g in steps))
        self.assertTrue(any(g > p for p, g in steps))
        # The layer's own goal (the dog's) gives the environment's distance while UniFP's goal has not moved on.
        self.assertEqual(gate["rule_mismatched_steps_own_goal"]["goal_as_env"], 0)
        self.assertLess(e["tool_to_goal_m"], 1e-5)
        for key in ("since_reset", "grasped", "engage_step", "ref_m", "target_m"):
            self.assertEqual(e[key], 0.0, key)
        self.assertLess(e["law_force_b_n"], 1e-4)
        for ep in out["by_episode"]:
            if ep["grasp_step_isaac"] is not None and not ep["compared_apart"]:
                self.assertLessEqual(ep["tool_to_placement_at_grasp_m"], self.radius + 1e-4)

    def test_an_ungated_layer_on_a_gated_rollout_is_refused_or_mismatched(self):
        with self.assertRaises(ValueError):
            self.CM.check_sim(_zero_policy(self.meta_plain), self.gated_dump, dog=False)
        # Without the dump's record of the gate (a dump from before it), the old rule grasps where the gate held back.
        d = self.CM.load(self.gated_dump)
        d.pop("grasp_within_m"), d.pop("engage_max_wait_steps")
        old = Path(self.tmp.name) / "old_dump" / "mech_shadow_dump.npz"
        old.parent.mkdir(exist_ok=True)
        np.savez_compressed(old, **d)
        out = self.CM.check_sim(_zero_policy(self.meta_plain), old, dog=False)
        self.assertGreater(out["rule_mismatched_steps"], 0)
        self.assertIsNone(out["grasp_gate"]["rollout"]["source"])
        # --grasp_within_m on that export brings it back.
        out = self.CM.check_sim(_zero_policy(self.meta_plain), old, dog=False, grasp_within_m=self.radius)
        self.assertEqual(out["rule_mismatched_steps"], 0)

    def test_off_is_unchanged(self):
        out = self.CM.check_sim(_zero_policy(self.meta_plain), self.plain_dump, dog=False)
        self.assertEqual(out["rule_mismatched_steps"], 0)
        self.assertIsNone(out["grasp_gate"]["grasp_within_m"])
        self.assertIsNone(out["grasp_gate"]["settled_steps_held_back"])
        with self.assertRaises(ValueError):                  # a gated layer on an ungated rollout
            self.CM.check_sim(_zero_policy(self.meta_gated), self.plain_dump, dog=False)
        # The layer with the gate off decides as before whatever position or distance it is given: the old rule
        # (`settle_rule`), grasp for grasp and goal for goal.
        rng = np.random.default_rng(9)
        mech = self.meta_plain["mechanism"]
        layers = [mech_layer.MechLayer(mech, dt=interface.POLICY_DT) for _ in range(3)]
        path = mech_layer.MechPath.slide([0.5, 0.0, 0.3], [-1.0, 0.0, 0.0], 0.1)
        for layer in layers:
            layer.reset([0.45, -0.1, 0.2], 60.0, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
        for k in range(200):
            speed = float(rng.uniform(0.0, 0.3))
            base = np.array([0.01 * k / 200, 0.0, 0.29])
            q = np.array([math.cos(0.001 * k), 0.0, 0.0, math.sin(0.001 * k)])
            # The rule as it was before the gate (`engage_due` of the S4b layer), on this step's count.
            ly = layers[0]
            waited = ly.since_reset + 1 - ly.engage_step
            old = (not ly.ever_grasped and waited >= 0 and (F32(speed) < ly.settle_speed or waited >= ly.max_wait))
            layers[0].update(base, q, speed, s=0.0, path=path)
            layers[1].update(base, q, speed, s=0.0, path=path, tool_pos=rng.normal(size=3))
            layers[2].update(base, q, speed, s=0.0, path=path, tool_to_goal_m=float(rng.uniform(0.0, 1.0)))
            self.assertEqual(layers[0].rule_due, old)
            for other in layers[1:]:
                self.assertEqual(other.rule_due, layers[0].rule_due)
                self.assertEqual(other.grasp_step, layers[0].grasp_step)
                np.testing.assert_array_equal(other.goal, layers[0].goal)
        self.assertIsNotNone(layers[0].grasp_step)

    def test_on_it_needs_the_tool(self):
        layer = mech_layer.MechLayer(self.meta_gated["mechanism"], dt=interface.POLICY_DT)
        layer.reset([0.45, -0.1, 0.2], 10.0, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
        with self.assertRaises(ValueError):
            layer.update(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), 0.0)
        with self.assertRaises(ValueError):
            layer.engage_due(0.0)


class Runner(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.meta = _meta(Path(cls.tmp.name))

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_meta_carries_the_training_constants(self):
        m = self.meta["mechanism"]
        self.assertEqual(self.meta["task"], "mechanism")
        self.assertEqual(m["law"]["evaluation"], {"kp": mech_cfg.FORCE_LAW_KP_N_PER_M, "ki": mech_cfg.FORCE_LAW_KI_N_PER_M_S,
                                                  "max_n": mech_cfg.FORCE_LAW_MAX_N, "bleed_s": 1.0,
                                                  "arrived_m": mech_cfg.FORCE_LAW_ARRIVED_M})
        self.assertEqual(m["approach"]["engage_delay_steps"], 15)
        self.assertEqual(m["approach"]["settle_speed_m_s"], hook_cfg.ENGAGE_SETTLE_SPEED_M_S)
        self.assertEqual(m["approach"]["max_wait_steps"], hook_cfg.ENGAGE_MAX_WAIT_STEPS)
        self.assertEqual(m["plan"]["speed_m_s"], mech_cfg.DEV_CONDITIONS["speed_m_s"])
        self.assertEqual(m["commands"]["roll_value"], [0.0, 1.0])
        self.assertEqual(m["tool"]["offset_m"], list(task_cfg.TOOL_OFFSET_M))
        self.assertEqual(m["stance"]["dog_stance_fraction_training"], 0.75)
        self.assertEqual(m["kinematics"]["jaw_local"], list(m["tool"]["offset_m"]))
        self.assertEqual(sorted(m["evaluation"]["kinds"]), sorted(mech_cfg.EVAL_DEFAULT_KINDS))

    def test_refusals(self):
        for bad in ({"mechanism": {"force_law": False}}, {"tool_body": "Link7_1"}, {"task": "unifp"}):
            folder = Path(self.tmp.name) / f"bad{len(list(Path(self.tmp.name).iterdir()))}"
            folder.mkdir()
            with self.assertRaises(ValueError):
                _meta(folder, **bad)
        with self.assertRaises(ValueError):
            export_policy.mech_meta(Path(self.tmp.name) / "run" / "model_1.pt", "position", Path(self.tmp.name) / "run")

    def test_a_still_robot_reaches_grasps_and_pulls(self):
        """A robot that does not move and a policy that asks nothing: the goal slides from UniFP's opening goal to the
        handle, the rule grasps at the approach's end plus 15 steps (the tool is still), the reference runs at
        0.10 m/s, the law's force follows its formula, and the goal stays on the (unmoving) handle."""
        runner = mech_layer.MechRunner(self.meta)
        base_pos, base_quat = np.array([0.0, 0.0, 0.29]), np.array([1.0, 0.0, 0.0, 0.0])
        handle = np.array([0.45, -0.2, 0.3])
        runner.reset(handle, base_pos, base_quat, traj_steps=100.0)
        handle_w = mech_layer.sphere_to_world(handle, base_pos, base_quat, interface.EE_GOAL_CENTER_OFFSET)
        path = mech_layer.MechPath.slide(handle_w, [-1.0, 0.0, 0.0], 0.12)
        legs = np.asarray(task_cfg.DOG_STANCE_LEG_OFFSETS) + np.asarray(interface.DEFAULT_DOF_POS[:12])
        legs = legs[runner.obs.to_unitree]
        policy = _zero_policy(self.meta)
        out = []
        for k in range(200):
            out.append(runner.step(policy, base_quat, np.zeros(3), legs, np.zeros(12), np.zeros(6), np.zeros(6),
                                   base_pos, base_quat, tool_speed=0.0, s=0.0, path=path))
            self.assertTrue(np.isfinite(runner.last_frame).all())
            if k == 0:
                np.testing.assert_array_equal(runner.commands, 0.0)
        grasp = next(ev for ev in runner.events if ev["event"] == "grasp")
        self.assertEqual(grasp["step"], 100 + 15)
        self.assertFalse(out[114]["grasped"])
        self.assertTrue(out[115]["grasped"])
        np.testing.assert_allclose(out[114]["goal"], handle, atol=1e-6)
        np.testing.assert_allclose(out[150]["goal"], handle, atol=1e-6)
        self.assertAlmostEqual(out[115 + 9]["ref"], 10 * 0.002, places=6)
        # F = kp lag + ki integral with lag = ref (the handle stuck at 0): the integral sums the lag every step, and
        # bleeds while the lag is under 5 mm (the first two steps).
        expected, integral = [], 0.0
        for o in out[115:150]:
            integral = (integral + o["ref"] * 0.02) * (math.exp(-0.02) if o["ref"] < 0.005 else 1.0)
            expected.append(min(400.0 * o["ref"] + 1000.0 * integral, 80.0))
        np.testing.assert_allclose([o["force_n"] for o in out[115:150]], expected, rtol=1e-5)
        np.testing.assert_allclose(runner.commands[6:8], [0.0, 1.0])
        np.testing.assert_allclose(runner.commands[9:12] / np.linalg.norm(runner.commands[9:12]), [-1.0, 0.0, 0.0],
                                   atol=1e-9)

    def test_the_gate_waits_for_the_hand(self):
        """With the arrival gate a still robot grasps on the ungated step when the handle is at its hand (the forward
        kinematics of the reported arm, `step`'s default tool), and never when the handle is 5 cm off, the fallback
        included."""
        meta = _gated_meta(Path(self.tmp.name) / "gated", 0.02)
        base_pos, base_quat = np.array([0.0, 0.0, 0.29]), np.array([1.0, 0.0, 0.0, 0.0])
        legs = np.asarray(task_cfg.DOG_STANCE_LEG_OFFSETS) + np.asarray(interface.DEFAULT_DOF_POS[:12])
        policy = _zero_policy(meta)
        for shift, grasp_at in ((0.0, 115), (0.05, None)):
            runner = mech_layer.MechRunner(meta)
            hand = runner.hand(np.zeros(6), base_pos, base_quat)
            handle = mech_layer.world_to_sphere(hand + [0.0, shift, 0.0], base_pos, base_quat,
                                                interface.EE_GOAL_CENTER_OFFSET)
            runner.reset(handle, base_pos, base_quat, traj_steps=100.0)
            path = mech_layer.MechPath.slide(hand, [-1.0, 0.0, 0.0], 0.12)
            held_back = 0
            for k in range(200):
                runner.step(policy, base_quat, np.zeros(3), legs[runner.obs.to_unitree], np.zeros(12), np.zeros(6),
                            np.zeros(6), base_pos, base_quat, tool_speed=0.0, s=0.0, path=path)
                held_back += int(runner.layer.settle_due and not runner.layer.rule_due)
            grasps = [ev for ev in runner.events if ev["event"] == "grasp"]
            if grasp_at is None:
                self.assertEqual(grasps, [])
                self.assertEqual(held_back, 200 - 115)
                self.assertAlmostEqual(runner.layer.tool_to_goal_m, 0.05, places=6)
            else:
                self.assertEqual(grasps[0]["step"], grasp_at)
                self.assertLess(grasps[0]["tool_to_goal_m"], 1e-6)

    def test_the_export_reads_the_gate_from_the_run(self):
        tmp = Path(self.tmp.name)
        approach = _gated_meta(tmp / "g1", 0.02)["mechanism"]["approach"]
        self.assertEqual((approach["grasp_within_m"], approach["max_wait_steps"], approach["grasp_gate_source"]),
                         (0.02, hook_cfg.ENGAGE_MAX_WAIT_STEPS, "env.yaml"))
        (tmp / "g2").mkdir()
        approach = _meta(tmp / "g2", env_yaml="grasp_within_m: 0.03\nengage_max_wait_steps: 75\n",
                         arguments={"mech_grasp_within_m": 0.03, "mech_engage_max_wait_s": 1.5})["mechanism"]["approach"]
        self.assertEqual((approach["grasp_within_m"], approach["max_wait_steps"]), (0.03, 75))
        self.assertEqual(mech_layer.MechLayer({**self.meta["mechanism"], "approach": approach}).max_wait, 75)
        # No env.yaml: run.json's flags; neither (every run before S4c): off, the default fallback.
        (tmp / "g3").mkdir()
        approach = _meta(tmp / "g3", arguments={"mech_grasp_within_m": 0.025})["mechanism"]["approach"]
        self.assertEqual((approach["grasp_within_m"], approach["grasp_gate_source"]), (0.025, "run.json arguments (no env.yaml)"))
        self.assertIsNone(self.meta["mechanism"]["approach"]["grasp_within_m"])
        self.assertEqual(self.meta["mechanism"]["approach"]["max_wait_steps"], hook_cfg.ENGAGE_MAX_WAIT_STEPS)
        # env.yaml and run.json disagreeing is refused.
        for yaml_text, args in (("grasp_within_m: 0.02\n", {"mech_grasp_within_m": 0.03}),
                                ("grasp_within_m: 0.02\n", {}),
                                ("grasp_within_m: null\nengage_max_wait_steps: 60\n", {})):
            folder = tmp / f"bad{len(list(tmp.iterdir()))}"
            folder.mkdir()
            with self.assertRaises(ValueError):
                _meta(folder, env_yaml=yaml_text, arguments=args)

    def test_the_dogs_reading(self):
        """`measure_travel` on the forward kinematics: a hand put on the path at s reads s; `tool_speed_estimate` is the
        point's speed along the joints' rates."""
        runner = mech_layer.MechRunner(self.meta)
        q = np.array([0.1, -0.4, 0.6, 0.05, 0.3, -0.2])
        base_pos, base_quat = np.array([0.1, -0.05, 0.29]), insert_quat(0.02, -0.04, 0.3)
        hand = runner.hand(q, base_pos, base_quat)
        for hinge in (False, True):
            s = 0.04
            path = (mech_layer.MechPath.swing(hand, [-0.8, 0.6, 0.0], [0.0, 0.0, 1.0], 0.25, 0.17) if hinge
                    else mech_layer.MechPath.slide(hand, [-0.8, 0.6, 0.0], 0.12))
            # Put the path's start so the hand is at s along it.
            shift = path.position(s) - path.position(0.0)
            moved = mech_layer.MechPath.from_grasp(hinge, hand - shift, path.direction, path.axis, path.radius, path.travel)
            got, off = runner.measure(moved, q, base_pos, base_quat)
            self.assertAlmostEqual(got, s, places=9)
            self.assertLess(off, 1e-9)
        dq = np.array([0.3, -0.2, 0.1, 0.0, 0.4, 0.2])
        h = 1e-6
        numeric = np.linalg.norm(runner.hand(q + h * dq, base_pos, base_quat) - runner.hand(q, base_pos, base_quat)) / h
        self.assertAlmostEqual(mech_layer.tool_speed_estimate(runner.kin, q, dq, base_quat, runner.tool_offset) / numeric,
                               1.0, delta=1e-2)

    def test_no_torch_on_the_dog(self):
        tree = ast.parse((REPO / "demos" / "combiner_key" / "robot" / "mech_layer.py").read_text())
        imported = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
        imported |= {n.module.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.module}
        self.assertLessEqual(imported, {"__future__", "math", "dataclasses", "numpy", "demos"})


def insert_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    from demos.combiner_key.robot import insert_gate
    return insert_gate.quat_rpy(roll, pitch, yaw)


class Gate(unittest.TestCase):
    """`mech_gate.py`'s replay on synthetic still frames with a policy that asks nothing."""

    @classmethod
    def setUpClass(cls):
        from demos.combiner_key.robot import insert_gate, mech_gate
        cls.G, cls.IG = mech_gate, insert_gate
        cls.tmp = tempfile.TemporaryDirectory()
        cls.meta = _meta(Path(cls.tmp.name))
        n = 400
        frame = np.zeros((n, 76))
        frame[:, 5:17] = np.asarray(task_cfg.DOG_STANCE_LEG_OFFSETS)
        cls.state = insert_gate.frames_state(cls.meta, {"frame": frame, "rpy": np.zeros((n, 3))})
        cls.policy = _zero_policy(cls.meta)
        cls.placement = mech_gate.placements()[0]

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_placements_are_s4b_validation(self):
        self.assertEqual([tuple(p) for p in self.G.placements()], list(mech_cfg.s4_set("validation", mech_cfg.S4B_MANIFEST)[0]))

    def test_hold_on_a_still_stand(self):
        out = self.G.replay(self.policy, self.state, self.placement, "drawer", "hold")
        self.assertLess(out["replay_frame_err"], 1e-12)
        self.assertIsNotNone(out["grasp_step"])
        self.assertEqual(out["legs_rad_max_first_s"], 0.0)
        self.assertEqual(out["law_peak_n"], 0.0)
        self.assertIn("marks_under_proposed_thresholds", out)
        self.assertEqual(out["marks_under_proposed_thresholds"]["legs"], True)

    def test_stuck_winds_up_in_about_a_second(self):
        """The reference at 0.10 m/s on a handle read at 0: 400 (0.1 t) + 1000 (0.05 t^2) = 80 N at t = 0.93 s."""
        out = self.G.replay(self.policy, self.state, self.placement, "drawer", "stuck")
        self.assertAlmostEqual(out["law_cap_after_grasp_s"], (-40.0 + math.sqrt(1600.0 + 16000.0)) / 100.0, delta=0.05)
        self.assertNotIn("marks_under_proposed_thresholds", out)

    def test_free_reads_the_hand(self):
        out = self.G.replay(self.policy, self.state, self.placement, "door", "free")
        self.assertIsNotNone(out["reading_behind_ref_m"])
        self.assertIn("final_fraction", out)

    def test_a_gated_export_is_gated_in_the_replay(self):
        """The replay's grasp waits for the reports' hand within the export's radius of the layer's goal: a radius the
        still arm never reaches holds it back for good; one it is always inside grasps on the ungated step."""
        tmp = Path(self.tmp.name)
        plain = self.G.replay(self.policy, self.state, self.placement, "drawer", "hold")
        self.assertIsNone(plain["grasp_within_m"])
        reach = plain["tool_to_goal_min_from_engage_m"]
        self.assertGreater(reach, 1e-3)
        for name, radius, grasps in (("tight", reach / 2.0, False), ("wide", 10.0, True)):
            meta = _gated_meta(tmp / name, radius)
            out = self.G.replay(_zero_policy(meta), self.state, self.placement, "drawer", "hold")
            self.assertAlmostEqual(out["grasp_within_m"], radius, places=6)
            self.assertAlmostEqual(out["tool_to_goal_min_from_engage_m"], reach, places=4)
            if grasps:
                self.assertEqual(out["grasp_step"], plain["grasp_step"])
                self.assertEqual(out["grasp_by_settle_rule"], plain["grasp_by_settle_rule"])
            else:
                self.assertIsNone(out["grasp_step"])
                self.assertFalse(out["marks_under_proposed_thresholds"]["grasp_by_settle_rule"])
                self.assertFalse(out["all_marks"])


@unittest.skipUnless(importlib.util.find_spec("rsl_rl") is not None, "the export loads checkpoints through rsl-rl")
class Export(unittest.TestCase):
    def test_export_of_a_random_mechanism_actor(self):
        from tensordict import TensorDict

        from unifp_train import observations
        from unifp_train.models import UniFPActor

        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            meta = _meta(tmp)
            run = tmp / "run"
            torch.manual_seed(7)
            probe = TensorDict({"policy": torch.zeros(1, interface.NUM_OBS),
                                "critic": torch.zeros(1, observations.NUM_CRITIC_OBS)}, batch_size=[1])
            actor = UniFPActor(probe, {"actor": ["policy"], "critic": ["critic"]}, "actor", 18,
                               distribution_cfg={"class_name": "GaussianDistribution", "init_std": 1.0, "std_type": "scalar"})
            ckpt = run / "model_1.pt"
            torch.save({"actor_state_dict": actor.state_dict(), "iter": 1}, ckpt)
            result = export_policy.export(ckpt, "force", tmp / "p.npz", task="mechanism", train_run=run,
                                          mech_law={"max_n": 40.0})
            self.assertLess(result["action_max_abs_err"], 1e-4)
            self.assertEqual(result["task"], "mechanism")
            policy = leg_policy.LegPolicy(result["out"])
            self.assertEqual(policy.meta["mechanism"]["law"]["evaluation"]["max_n"], 40.0)
            self.assertEqual(policy.meta["mechanism"]["plan"], meta["mechanism"]["plan"])
            with self.assertRaises(ValueError):
                export_policy.export(ckpt, "position", tmp / "q.npz", task="mechanism", train_run=run)


if __name__ == "__main__":
    unittest.main()
