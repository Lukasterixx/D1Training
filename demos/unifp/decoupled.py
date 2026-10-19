"""The door pull without UniFP: a walking policy on the legs and inverse kinematics on the arm.

The decoupled baseline for `run_demo.py --task door` (`--controller decoupled`). Only the controller changes:
- the box, its placements, the door script (`script.door_phases`), the walk round the hinge
  (`script.base_velocity_command`), the latch (unlocked), the jaws' drive and commands, the trace and the success
  criterion are the UniFP door pull's own, through `UniFPDemoEnv`;
- the legs run Rescue's velocity-tracking policy (`logs/rsl_rl/unitree_go2_rough`, the walking checkpoint
  `sim.py` plays back). It was trained with 12 leg actions and the arm as a passive payload (F-008). It gets its
  own 235-wide observation, rebuilt here from the same state `flat_env_cfg.ObservationsCfg` reads, height scan
  included, and drives the legs through the actuator it was trained against (Isaac Lab's Go2 `DCMotor`);
- the arm is put where the script asks by inverse kinematics in the base's current frame (`decoupled_arm.py`),
  at the D1's 10 Hz command rate. The setpoints go through the D1 firmware's fitted motion planner
  (`position_only.core.TrapezoidTracker`, F-045) into the stiff servo drive with the published torque and
  measured speed limits, as the scripted demos and the position-only task model the arm.

Two things therefore differ from the UniFP run besides the controller, and both belong to the decoupled stack:
- the leg actuator: DCMotor here, UniFP's ideal PD there, the same 25/0.5 gains;
- the arm model: the measured D1 here, UniFP's 60/40 PD there, which is faster than the real arm (F-097).

The jaws keep UniFP's drive (800 N/m, 15 N), so the grip itself is the same. No perception in either: the box's
placement is given, and the base's pose comes from the simulator, as it does for the UniFP run's goal.

Needs the Isaac app: import after `AppLauncher`.
"""
from __future__ import annotations

import copy
import hashlib
from pathlib import Path

import numpy as np
import torch

from isaaclab.actuators import IdealPDActuatorCfg, ImplicitActuatorCfg
from isaaclab.sensors import RayCaster, RayCasterCfg, patterns
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG

from flat_env_cfg import ARM_BASE_DAMPING, ARM_BASE_STIFFNESS, LEG_JOINTS
from motor_model import (D1_COMMAND_DEAD_TIME_S, D1_COMMAND_HZ, D1_EFFORT_LIMIT_NM, D1_PLAN_ACCEL_RAD_S2,
                         D1_PLAN_DECEL_RAD_S2, D1_REPLAN_VELOCITY_RETENTION, D1_VELOCITY_LIMIT_RAD_S)
from position_only.core import TrapezoidTracker
from position_only.workspace import load_urdf
from demos.cup.pick_demo.grasp import arm_clear
from unifp_isaaclab import interface, robot as robot_mod

from . import decoupled_arm, script as demo_script
from .env import UniFPDemoEnv

ROOT = Path(__file__).resolve().parent.parent.parent
WALK_LOG_DIR = ROOT / "logs" / "rsl_rl" / "unitree_go2_rough"
ARM_NAMES = tuple(f"Joint{i}" for i in range(1, 7))
#: The feet's friction. The walking policy trained and plays back on 0.8 static, 0.6 dynamic (`flat_env_cfg`'s
#: material event, multiplied with a 1.0 ground). UniFP's ground averages, so these give the same 0.8 and 0.6 against
#: its 1.0. Only the feet: the fingers keep UniFP's, so the grip on the handle is unchanged. At UniFP's 1.0 the policy
#: tracked a third of a constant (-0.2, -0.33, 0.2) command and tilted 19-32 deg, against 7.6 deg in its own playback.
FOOT_STATIC_FRICTION = 0.6
FOOT_DYNAMIC_FRICTION = 0.2
#: The walking policy's observation: base velocity (3), angular velocity (3), projected gravity (3), command (3),
#: leg positions and velocities (12 + 12), its last action (12), and a 17 x 11 height scan.
WALK_OBS_DIM = 235


def walking_checkpoint() -> Path:
    """The walking checkpoint `sim.py` loads: the newest `model_*.pt` of the newest run."""
    runs = sorted(p for p in WALK_LOG_DIR.iterdir() if p.is_dir())
    models = sorted(runs[-1].glob("model_*.pt"), key=lambda p: int(p.stem.split("_")[1]))
    return models[-1]


class WalkingPolicy:
    """Rescue's Go2 velocity tracker (`agent_cfg.py`: 512-256-128, ELU, no observation normaliser), deterministic."""

    def __init__(self, path: Path, device):
        state = torch.load(path, map_location=device, weights_only=False)["actor_state_dict"]
        layers = [torch.nn.Linear(WALK_OBS_DIM, 512), torch.nn.ELU(), torch.nn.Linear(512, 256), torch.nn.ELU(),
                  torch.nn.Linear(256, 128), torch.nn.ELU(), torch.nn.Linear(128, 12)]
        self.net = torch.nn.Sequential(*layers).to(device).eval()
        self.net.load_state_dict({k.removeprefix("mlp."): v for k, v in state.items() if k.startswith("mlp.")})
        self.path = path
        self.sha256 = hashlib.sha256(path.read_bytes()).hexdigest()

    @torch.no_grad()
    def act(self, obs: torch.Tensor) -> torch.Tensor:
        return self.net(obs)


def make_robot_cfg(usd_path: str):
    """UniFP's welded robot (weld, force drives, self-collision, spawn, solver) with the decoupled stack's actuators."""
    cfg = robot_mod.make_robot_cfg(usd_path, prim_path="/World/envs/env_.*/Robot")
    # Off, as in the stock Go2 config the walking policy trained and plays back with. On (UniFP's setting), its legs
    # meet each other stepping sideways: walking the door's path in free space with the arm parked, 1 of 2 fell with
    # it on and 0 of 2 with it off (Week 6 log, 2026-10-19). The arm is then kept off the trunk by its IK's clearance
    # check alone, and every command that breaks it is counted (`ik_counts["trunk_proxy"]`).
    cfg.spawn.articulation_props.enabled_self_collisions = False
    jaws = list(interface.ISAACLAB_NAMES[18:])
    cfg.actuators = {
        # What the walking checkpoint was trained against; `sim.py` keeps it for the same reason.
        "base_legs": copy.deepcopy(UNITREE_GO2_CFG.actuators["base_legs"]),
        # The D1's servo as the position-only task and the scripted demos model it (`flat_env_cfg`, d1_servo).
        "d1_arm": ImplicitActuatorCfg(joint_names_expr=list(ARM_NAMES), stiffness=ARM_BASE_STIFFNESS,
                                      damping=ARM_BASE_DAMPING, effort_limit_sim=dict(D1_EFFORT_LIMIT_NM),
                                      velocity_limit_sim=dict(D1_VELOCITY_LIMIT_RAD_S)),
        # UniFP's jaw drive, unchanged, so the grip is the same in both runs.
        "d1_jaws": IdealPDActuatorCfg(joint_names_expr=jaws, stiffness=interface.GRIPPER_STIFFNESS,
                                      damping=interface.GRIPPER_DAMPING,
                                      effort_limit=interface.TORQUE_LIMITS[18], effort_limit_sim=interface.TORQUE_LIMITS[18],
                                      armature=robot_mod.ARM_ARMATURE),
    }
    return cfg


def height_scanner_cfg() -> RayCasterCfg:
    """`flat_env_cfg.FlatSceneCfg.height_scanner`, pointed at this scene's base and ground."""
    return RayCasterCfg(
        prim_path="/World/envs/env_.*/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
        debug_vis=False,
        drift_range=(0.0, 0.0),
        mesh_prim_paths=["/World/ground"],
        max_distance=100.0,
        update_period=interface.POLICY_DT,
    )


class DecoupledDoorEnv(UniFPDemoEnv):
    """`UniFPDemoEnv` driven by a walking policy and arm IK instead of UniFP's 18 actions."""

    def __init__(self, cfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        robot = self._robot
        # The walking policy's leg order: `SceneEntityCfg(joint_names=LEG_JOINTS)` resolves in articulation order.
        self._leg_ids, leg_names = robot.find_joints(LEG_JOINTS)
        self._leg_ids = torch.as_tensor(self._leg_ids, device=self.device)
        # Where each of its 12 actions goes in UniFP's 18-wide action: the same target, default + 0.25 * action.
        self._leg_slots = torch.as_tensor([interface.ISAACLAB_NAMES.index(n) for n in leg_names], device=self.device)
        self._arm_ids = torch.as_tensor(robot.find_joints(list(ARM_NAMES), preserve_order=True)[0], device=self.device)
        self.walk_last_action = torch.zeros(self.num_envs, 12, device=self.device)

        self._planner = TrapezoidTracker(
            self.num_envs, 6, D1_PLAN_ACCEL_RAD_S2, D1_PLAN_DECEL_RAD_S2,
            [D1_VELOCITY_LIMIT_RAD_S[n] for n in ARM_NAMES], dead_time=D1_COMMAND_DEAD_TIME_S,
            retention=D1_REPLAN_VELOCITY_RETENTION, device=self.device)
        self._command_period = round(1.0 / (D1_COMMAND_HZ * interface.POLICY_DT))
        # Spread the environments' 10 Hz command cycles over the policy steps, deterministically.
        self._command_phase = torch.arange(self.num_envs, device=self.device) % self._command_period
        joints, links = load_urdf()
        self._joints = joints
        self.arms = [decoupled_arm.ArmIK(joints, links, seeded_loop=getattr(cfg, "seeded_ik_loop", False))
                     for _ in range(self.num_envs)]
        self._set_foot_friction()
        #: The IK's outcome at each environment's latest command: `decoupled_arm.IKStep.status`.
        self.ik_status = ["hold"] * self.num_envs
        self.ik_counts = [dict() for _ in range(self.num_envs)]
        #: Diagnostic: never command the arm, so it stays at its reset pose while the legs walk the script's path.
        #: Separates the walking policy carrying an outstretched arm from the walking policy itself.
        self.park_arm = False

    def _set_foot_friction(self) -> None:
        """`FOOT_STATIC_FRICTION` and `FOOT_DYNAMIC_FRICTION` on the feet's shapes, as `mdp.randomize_rigid_body_material`
        addresses one body's shapes."""
        robot = self._robot
        view = robot.root_physx_view
        shapes = [robot._physics_sim_view.create_rigid_body_view(path).max_shapes for path in view.link_paths[0]]
        if sum(shapes) != view.max_shapes:
            raise RuntimeError("could not map the robot's collision shapes to its bodies")
        materials = view.get_material_properties()
        for body in robot.find_bodies(".*_foot")[0]:
            start = sum(shapes[:body])
            materials[:, start:start + shapes[body], 0] = FOOT_STATIC_FRICTION
            materials[:, start:start + shapes[body], 1] = FOOT_DYNAMIC_FRICTION
        view.set_material_properties(materials, torch.arange(view.count))

    # --- scene ------------------------------------------------------------------------------------

    def _setup_scene(self):
        super()._setup_scene()
        self._height_scanner = RayCaster(height_scanner_cfg())
        self.scene.sensors["height_scanner"] = self._height_scanner

    # --- the legs ---------------------------------------------------------------------------------

    def walking_observation(self) -> torch.Tensor:
        """The walking policy's observation, as `flat_env_cfg.ObservationsCfg.PolicyCfg` builds it (no noise)."""
        data = self._robot.data
        ids = self._leg_ids
        scanner = self._height_scanner.data
        scan = (scanner.pos_w[:, 2].unsqueeze(1) - scanner.ray_hits_w[..., 2] - 0.5).clamp(-1.0, 1.0)
        return torch.cat((
            data.root_lin_vel_b, data.root_ang_vel_b, data.projected_gravity_b, self._base_cmd,
            data.joint_pos[:, ids] - data.default_joint_pos[:, ids],
            data.joint_vel[:, ids] - data.default_joint_vel[:, ids],
            self.walk_last_action, scan), dim=-1)

    def action_from_legs(self, leg_actions: torch.Tensor) -> torch.Tensor:
        """UniFP's 18-wide action with the walking policy's 12 in the leg slots; the arm's are replaced below."""
        self.walk_last_action = leg_actions.clone()
        action = torch.zeros(self.num_envs, interface.NUM_ACTIONS, device=self.device)
        action[:, self._leg_slots] = leg_actions
        return action

    # --- the arm ----------------------------------------------------------------------------------

    def _scripted_goal(self) -> torch.Tensor:
        """The parent's script step (commands, jaws, walk), then this step's arm commands where they are due."""
        goal = super()._scripted_goal()
        due = ((self.episode_length_buf + self._command_phase) % self._command_period) == 0
        if due.any() and not self.park_arm:
            self._command_arm(due)
        return goal

    def _command_arm(self, due: torch.Tensor) -> None:
        data = self._robot.data
        base_pos = (data.root_pos_w - self.scene.env_origins).cpu().numpy()
        base_rot = _quat_to_matrix(data.root_quat_w.cpu().numpy())
        spawn_pos = self._spawn_pos.cpu().numpy()
        spawn_rot = _quat_to_matrix(self._spawn_yaw_quat.cpu().numpy())
        targets = self._planner.goal.clone()
        for index in torch.nonzero(due).flatten().tolist():
            command = self.commands[index]
            if command is None:
                continue
            to_base = base_rot[index].T
            point = to_base @ (spawn_pos[index] + spawn_rot[index] @ np.asarray(command.point) - base_pos[index])
            arm = self.arms[index]
            if command.phase == "spawn":
                # Settling after the drop: the jaw centre to the script's home point, its orientation free.
                step = arm.solve(point, None)
            else:
                approach = to_base @ spawn_rot[index] @ np.asarray(command.approach)
                bar = to_base @ spawn_rot[index] @ np.asarray(command.object_axis)
                if arm.grasp is None:
                    site = self.sites[index]
                    grasp_point = to_base @ (spawn_pos[index] + spawn_rot[index] @ np.asarray(site.grasp_point_m())
                                             - base_pos[index])
                    arm.choose_grasp(grasp_point, approach, bar, demo_script.DoorTiming().standoff_m)
                step = arm.solve(point, decoupled_arm.hand_rotation(approach, bar, *arm.grasp))
            self.ik_status[index] = step.status
            counts = self.ik_counts[index]
            counts[step.status] = counts.get(step.status, 0) + 1
            if not bool(arm_clear(self._joints, step.q)[0]):
                counts["trunk_proxy"] = counts.get("trunk_proxy", 0) + 1
            targets[index] = torch.as_tensor(step.q, dtype=torch.float32, device=self.device)
        self._planner.command(due, targets)

    def _apply_action(self) -> None:
        super()._apply_action()
        # The arm's slots of the parent's targets are UniFP's default pose; the planner's trajectory replaces them,
        # with its velocity fed forward as the position-only task's arm action does (`HeldJointPositionAction`).
        self._robot.set_joint_position_target(self._planner.step(self.cfg.sim.dt), joint_ids=self._arm_ids)
        self._robot.set_joint_velocity_target(self._planner.velocity, joint_ids=self._arm_ids)

    def _reset_idx(self, env_ids) -> None:
        super()._reset_idx(env_ids)
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        q = self._robot.data.joint_pos[env_ids][:, self._arm_ids]
        self._planner.reset(env_ids, q)
        self.walk_last_action[env_ids] = 0.0
        for row, index in enumerate(env_ids.tolist()):
            self.arms[index].reset(q[row].cpu().numpy())
            self.ik_status[index] = "hold"
            self.ik_counts[index] = {}


def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """(N, 4) wxyz to (N, 3, 3)."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], axis=-1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], axis=-1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], axis=-1)], axis=1)
