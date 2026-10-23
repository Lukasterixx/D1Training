"""The key turned by the arm, in simulation: the combiner key demo's environment with the D1's fingers, not the
front-right foot, turning Lukas's lever key from 45 degrees up to 45 down (`arm_turn`, the push planned in numpy).

On top of `env.CombinerKeyEnv`, with the key already in the lock (`key_inserted`):
- **The look is the door demo's**: the head RealSense's AprilTag and the MID-360's face fit put the box; nothing else
  is told to the arm. The arm unfolds, the walking policy walks the robot to the push's stance (`arm_turn.stance_base`,
  from the estimate) and keeps the legs throughout: there is no leg policy here.
- **The push is scripted**: out in front of the platform, over it, down onto it, round the key's arc to its stop (the
  hand pose rotated rigidly about the shaft, its edge `arm_turn.PRESS_DEPTH_M` inside the platform), a hold, lift off,
  back out, fold. Every point and orientation is from the look's estimate; the arm IK and the D1's planner at 10 Hz
  run them as in the door demo.
- **The lock is the leg turn's plant** (`unifp_train.handle.LeverHandle(pad=True)`: the key's sticking friction, its
  stops, its gravity), at the key's true pose, against a sphere `arm_turn.CONTACT_RADIUS_M` inside the fingertips' lower
  edge instead of the foot, with the key insertion's contact stiffness. Its force acts on Link6 at that point. PhysX
  draws the key where the plant has it. The door unlocks past 30 degrees, as in the leg demo.
- **An overview camera** beside the box, for pictures of the push (`overview_view`).

Not modelled: the rest of the hand against the key (only the sphere at the fingertips touches it), the real cam's
feel (the plant's friction is a constant from Lukas's 5 N), the wrist RealSense's mass (the door demo's robot draws the
payload; `arm_turn.capacity_scan` checks the camera's weight on the CPU).

Needs the Isaac app: import after `AppLauncher`.
"""
from __future__ import annotations

import math

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.sensors import Camera, CameraCfg

from demos.combiner_door import env as door_env
from demos.unifp import script as demo_script
from unifp_isaaclab import interface
from unifp_train import handle, leg_cfg

from . import arm_turn as at
from . import env as key_env
from . import geometry as kg

#: The fingers on the printed platform, in the lock's plant, and a friction for aluminium fingers on PLA. The contact
#: force ripples 3-6 N about the lock's 5 N through the turn: not the contact -- at the key insertion's 16,000 N/m and
#: 40 N·s/m and at these (the foot's stiffness, damped harder) its step-to-step change was the same (median 0.38 against
#: 0.39 N, 2026-10-21) -- but the D1's 10 Hz streaming: its firmware's plan restarts from rest at each setpoint
#: (`motor_model.D1_REPLAN_VELOCITY_RETENTION`), and the shoulder's torque ripples at 10 Hz with the hand held still.
CONTACT_K_N_PER_M = 8000.0
CONTACT_C_N_S_PER_M = 60.0
CONTACT_MU = 0.4
#: Lukas, 2026-10-21: "the real lever takes about 5N to rotate in the lock". Taken as felt where the fingers push.
LOCK_PUSH_N = 5.0

#: The jaws through the push: shut (`JAW_HOLD_M` is the travel that holds the key's bar).
JAW_SHUT_M = 0.0
#: The phases' lengths, s. The walk is the key demo's.
OUT_S, ABOVE_S, TOUCH_S, HOLD_S, LIFT_S, BACK_S, STAND_S = 3.0, 1.5, 1.5, 1.5, 1.0, 1.5, 1.5
WALK_S, SETTLE_S = 12.0, 1.5
PUSH_PHASES = ("arm_out", "arm_above", "arm_touch", "turn", "arm_hold", "arm_lift", "arm_back")
#: Pictures every this often through the push.
FRAME_EVERY_S = 0.25
#: The robot's own check that the key turned (`arm_turn.TRACK_LIMIT_MM`).
TRACK_LIMIT_MM = at.TRACK_LIMIT_MM


def _pose(st) -> np.ndarray:
    return key_env.floor_site(st["box_site"])


def _hand(st, theta_deg: float) -> np.ndarray:
    """Link6's axes in the script frame with the key at `theta_deg`, from the estimate."""
    return _pose(st)[:3, :3] @ at.hand_rotation(theta_deg)


def _jaw(st, where: str, theta_deg: float) -> tuple[float, float, float]:
    """The jaw centre (the IK's point) for the tip edge `where` (`arm_turn.tip_target`), script frame."""
    pose = _pose(st)
    tip = pose[:3, :3] @ at.tip_target(where, theta_deg) + pose[:3, 3]
    return tuple(float(v) for v in at.jaw_centre(tip, _hand(st, theta_deg)))


def arm_phases() -> list[demo_script.Phase]:
    """After the spawn's settle: look (folded), unfold, walk to the push's stance with the hand at home, settle, then
    the push, fold, stand. Orientations are Link6's z (approach) and x (`object_axis`, with the grasp (0, +1))."""
    jaws = JAW_SHUT_M
    home = lambda st, u: door_env.home_at(st["base"])
    start, end = at.START_DEG, at.END_DEG
    z = lambda deg: (lambda st, *u: tuple(_hand(st, deg)[:, 2]))
    x = lambda deg: (lambda st, *u: tuple(_hand(st, deg)[:, 0]))
    at_angle = lambda where, deg: (lambda st: _jaw(st, where, deg))
    push = lambda name, seconds, where, deg: demo_script.Phase(name, seconds, target=at_angle(where, deg), jaw_m=jaws,
                                                               approach=z(deg), object_axis=x(deg))
    turning = lambda st, u: _jaw(st, "pressed", at.turn_angle(u))
    return [demo_script.Phase("look", door_env.LOOK_S, path=home, jaw_m=jaws),
            demo_script.Phase("unfold", door_env.UNFOLD_S, path=home, jaw_m=jaws),
            demo_script.Phase("walk_to_key", WALK_S, path=home, jaw_m=jaws),
            demo_script.Phase("settle_arm", SETTLE_S, path=home, jaw_m=jaws),
            push("arm_out", OUT_S, "out", start),
            push("arm_above", ABOVE_S, "above", start),
            push("arm_touch", TOUCH_S, "pressed", start),
            demo_script.Phase("turn", at.turn_seconds(), path=turning, jaw_m=jaws,
                              approach_path=lambda st, u: tuple(_hand(st, at.turn_angle(u))[:, 2]),
                              object_axis_path=lambda st, u: tuple(_hand(st, at.turn_angle(u))[:, 0])),
            push("arm_hold", HOLD_S, "resting", end),
            push("arm_lift", LIFT_S, "above", end),
            push("arm_back", BACK_S, "out", end),
            demo_script.Phase("fold", door_env.UNFOLD_S, path=home, jaw_m=jaws),
            demo_script.Phase("stand", STAND_S, path=home, jaw_m=jaws)]


class ArmTurn:
    """The key's lock against the fingertips, in `CombinerKeyEnv`'s turn slot (`self.leg`, whose interface it keeps):
    the legs stay with the walking policy, and the plant's force goes to Link6 at the contact sphere."""

    def __init__(self, env):
        n, device = env.num_envs, env.device
        self.env, self.num_envs, self.device = env, n, device
        self._handle = handle.LeverHandle(n, device, k=CONTACT_K_N_PER_M, c=CONTACT_C_N_S_PER_M)
        zeros = lambda *shape: torch.zeros(*shape, device=device)
        self.active = torch.zeros(n, dtype=torch.bool, device=device)
        self._contact_local = torch.as_tensor(at.contact_link6(), dtype=torch.float32, device=device).expand(n, 3).contiguous()
        self._link6_ids = torch.tensor([env._link6], dtype=torch.int32, device=device)
        self.force_w = zeros(n, 3)
        self.peak_n, self.max_theta = zeros(n), torch.full((n,), -10.0, device=device)
        #: Per environment, the estimate the robot measures the turn against: the lock's point, the robot's right.
        self.est_pivot, self.est_u0 = zeros(n, 3), zeros(n, 3)
        self.est_u0[:, 1] = 1.0

    def start(self, index: int, door_w, normal_w, right_w, est_door_w, est_right_w, friction: float, detent: float) -> None:
        c = leg_cfg
        ids = torch.tensor([index], device=self.device)
        t = lambda v: torch.as_tensor(np.asarray(v, dtype=np.float32), device=self.device).reshape(1, -1)
        one = lambda v: torch.full((1,), float(v), device=self.device)
        self._handle.reset(ids)
        self._handle.place(ids, door=t(door_w), normal=t(normal_w), u0=t(right_w), standoff=one(c.KEY_AIM_OFF_DOOR_M),
                           length=one(c.KEY_BAR_BOX[0][1]), inner=one(0.0), radius=one(c.KEY_TOP_M),
                           inertia=one(np.mean(c.KEY_INERTIA_KG_M2)), rest_nm=one(0.0), full_nm=one(0.0),
                           damping=one(np.mean(c.KEY_DAMPING)), friction=one(friction),
                           theta_max=one(math.radians(at.END_DEG)), theta_latch=one(c.KEY_THETA_UNLOCK_RAD),
                           mu=one(CONTACT_MU), theta_min=one(math.radians(at.START_DEG)), pad=True,
                           detent_nm=one(detent), detent_from=one(c.KEY_THETA_UNLOCK_RAD - c.KEY_DETENT_SPAN_RAD),
                           gravity_nm=one(c.KEY_GRAVITY_NM))
        self.est_pivot[ids], self.est_u0[ids] = t(est_door_w), t(est_right_w)
        self.peak_n[ids], self.max_theta[ids] = 0.0, math.radians(at.START_DEG)
        self.active[ids] = True

    def finished(self) -> torch.Tensor:
        return torch.zeros_like(self.active)        # the script ends the push

    def stop(self, index: int) -> None:
        self.active[index] = False
        self._handle.reset(torch.tensor([index], device=self.device))

    def step(self, walk=None) -> torch.Tensor:
        return torch.zeros(self.num_envs, interface.NUM_ACTIONS, device=self.device)

    def contact_state(self) -> tuple[torch.Tensor, torch.Tensor]:
        """The contact sphere's centre (env frame) and velocity, from Link6's pose and twist."""
        data = self.env._robot.data
        link = self.env._link6
        arm = interface.quat_apply(data.body_quat_w[:, link], self._contact_local)
        centre = data.body_pos_w[:, link] + arm - self.env.scene.env_origins
        return centre, data.body_lin_vel_w[:, link] + torch.cross(data.body_ang_vel_w[:, link], arm, dim=-1)

    def tip_edge(self) -> torch.Tensor:
        """The fingertips' lower edge (env frame), as forward kinematics of the arm's encoders would put it."""
        data = self.env._robot.data
        link = self.env._link6
        edge = torch.as_tensor(at.TIP_EDGE_LINK6, dtype=torch.float32, device=self.device).expand(self.num_envs, 3)
        return data.body_pos_w[:, link] + interface.quat_apply(data.body_quat_w[:, link], edge) - self.env.scene.env_origins

    def measured_deg(self) -> torch.Tensor:
        """The turn as the robot measures it: the key angle its fingertips' edge implies on the estimated key, resting
        on the platform `PUSH_REACH_M` out (`handle.measured_theta`). Meaningful only while it presses."""
        reach = torch.full((self.num_envs,), at.PUSH_REACH_M, device=self.device)
        clearance = torch.full((self.num_envs,), leg_cfg.KEY_TOP_M, device=self.device)
        return torch.rad2deg(handle.measured_theta(self.tip_edge(), self.est_pivot, self.est_u0, reach, clearance))

    def physics_step(self, dt: float) -> None:
        if not bool(self.active.any()):
            self.force_w.zero_()
            return
        centre, velocity = self.contact_state()
        on = self.active.unsqueeze(-1).float()
        self.force_w = self._handle.step(centre, velocity, dt, foot_r=at.CONTACT_RADIUS_M) * on
        self.peak_n = torch.maximum(self.peak_n, self._handle.take_peak() * self.active)
        self.max_theta = torch.where(self.active, torch.maximum(self.max_theta, self._handle.theta), self.max_theta)
        robot = self.env._robot
        local = interface.quat_rotate_inverse(robot.data.body_quat_w[:, self.env._link6], self.force_w)
        robot.permanent_wrench_composer.set_forces_and_torques(
            forces=local.unsqueeze(1).contiguous(), positions=self._contact_local.unsqueeze(1).contiguous(),
            body_ids=self._link6_ids)


class ArmTurnEnv(key_env.CombinerKeyEnv):
    def __init__(self, cfg, render_mode=None, **kwargs):
        cfg.key_inserted = True
        super().__init__(cfg, render_mode, **kwargs)
        self.parked_phases = {"spawn", "look", "no_tag", "stand"}
        self.position_only_phases = {"walk_to_key", "settle_arm"}
        self.joint_phases = {"unfold", "fold"}
        self.lock_friction = float(getattr(cfg, "lock_friction_nm", at.lock_torque_nm(LOCK_PUSH_N)))
        self.lock_detent = float(getattr(cfg, "lock_detent_nm", 0.0))
        from motor_model import D1_EFFORT_LIMIT_NM
        from demos.unifp.decoupled import ARM_NAMES

        self.arm_effort = torch.as_tensor([D1_EFFORT_LIMIT_NM[name] for name in ARM_NAMES], device=self.device)

    def _make_turn(self, cfg):
        return ArmTurn(self)

    def _setup_scene(self):
        super()._setup_scene()
        self._overview = None
        if not getattr(self.cfg, "overview", True):
            return
        self._overview = Camera(CameraCfg(
            prim_path="/World/envs/env_.*/OverviewCam", update_period=0.0, width=960, height=720, data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, clipping_range=(0.05, 20.0))))
        self.scene.sensors["overview_cam"] = self._overview

    def overview_view(self, index: int) -> tuple[np.ndarray, np.ndarray]:
        """(eye, target) in the world for environment `index`: from the robot's front left, above, onto the key's arc
        and the hand (box frame offsets from the lock, true box)."""
        m = self._truth_env(index)
        lock = np.asarray(kg.LOCK)
        eye = m[:3, :3] @ (lock + np.array([0.55, -0.55, 0.30])) + m[:3, 3]
        target = m[:3, :3] @ (lock + np.array([0.06, 0.08, -0.02])) + m[:3, 3]
        origin = self.scene.env_origins[index].cpu().numpy()
        return eye + origin, target + origin

    def aim_overview(self) -> None:
        eyes, targets = zip(*(self.overview_view(i) for i in range(self.num_envs)))
        t = lambda v: torch.as_tensor(np.stack(v), dtype=torch.float32, device=self.device)
        self._overview.set_world_poses_from_view(t(eyes), t(targets))

    def overview_rgb(self, index: int = 0) -> np.ndarray:
        return self._overview.data.output["rgb"][index, ..., :3].cpu().numpy().astype(np.uint8)

    # --- the legs stay with the walking policy ------------------------------------------------------

    def legs_from_policy(self) -> torch.Tensor:
        return torch.zeros_like(self.leg.active)

    # --- the walk -----------------------------------------------------------------------------------

    def _stance_target(self, index: int, phase: str) -> tuple[float, float]:
        pose = key_env.floor_site(self.estimates[index])
        lock = pose[:3, :3] @ np.asarray(kg.LOCK) + pose[:3, 3]
        normal = pose[:3, :3] @ np.asarray(kg.OUT_OF_DOOR_BOX)
        right = pose[:3, :3] @ np.asarray(kg.LEVER_LEVEL_BOX)
        base = at.stance_base(lock, normal, right)
        return float(base[0]), float(base[1])

    def _estimated_stance(self, index: int) -> tuple[float, float, float]:
        """Where the look's estimate puts the lock from the base as it stands: (ahead m, left m, yaw from facing the
        door deg)."""
        pose = key_env.floor_site(self.estimates[index])
        lock = pose[:3, :3] @ np.asarray(kg.LOCK) + pose[:3, 3]
        normal = pose[:3, :3] @ np.asarray(kg.OUT_OF_DOOR_BOX)
        x, y, yaw = self._base_now[index]
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = lock[0] - x, lock[1] - y
        error = (math.atan2(-normal[1], -normal[0]) - yaw + math.pi) % (2 * math.pi) - math.pi
        return c * dx + s * dy, -s * dx + c * dy, math.degrees(error)

    def _walk_base(self) -> None:
        """The key demo's walk to the stance, stopped -- and the walk phase ended -- the first time the estimated
        stance is inside `arm_turn`'s window: close enough, as the real door pull takes it, not a point to hunt for."""
        super()._walk_base()
        for index, command in enumerate(self.commands):
            if command is None or command.phase != "walk_to_key" or self.estimates[index] is None:
                continue
            if not self._accepted[index]:
                ahead, left, yaw = self._estimated_stance(index)
                if not at.in_window(ahead, left, yaw):
                    continue
                self._accepted[index] = True
                demo = self.scripts[index]
                self.records[index]["walk_accepted"] = {"after_s": round(demo.elapsed_s, 2), "door_ahead_m": round(ahead, 4),
                                                        "lock_left_m": round(left, 4), "yaw_error_deg": round(yaw, 2)}
            self._base_cmd[index] = 0.0
            demo = self.scripts[index]
            demo.elapsed_s = max(demo.elapsed_s, demo.phases[demo.index].duration_s)     # the walk is over
        self._commands[:, :3] = self._base_cmd

    # --- the push -----------------------------------------------------------------------------------

    def _enter(self, index: int, phase: str, previous: str | None) -> None:
        super()._enter(index, phase, previous)
        record = self.records[index]
        if phase == "arm_out":
            record["at_stance"] = self._stance_record(index)
            record["out_seed"] = self._seed_push(index)
            self._start_push(index)
        if phase == "turn":
            record["touched"] = self._push_record(index)
        if phase == "arm_lift":
            record["held"] = self._push_record(index)
        if phase == "fold":
            record["after"] = self._push_record(index)
            record.pop("after_release", None)        # the parent's seat check, for an inserted key
            self._frozen_deg[index] = float(record["after"]["key_deg"])
            self.leg.stop(index)
            self.key_mode[index] = "left_turned"

    def _start_turn(self, index: int) -> None:
        """The key demo starts the leg policy here; the push started at `arm_out`."""

    def _start_push(self, index: int) -> None:
        """The lock's plant at the key's true pose; the robot's measure of the turn on the estimate."""
        truth, est = self._truth_env(index), self._box_to_env(index, self.estimates[index])
        at_ = lambda m: (m[:3, :3] @ np.asarray(kg.LOCK) + m[:3, 3], m[:3, :3] @ np.asarray(kg.OUT_OF_DOOR_BOX),
                         m[:3, :3] @ np.asarray(kg.LEVER_LEVEL_BOX))
        door, normal, right = at_(truth)
        est_door, _, est_right = at_(est)
        self.leg.start(index, door, normal, right, est_door, est_right, self.lock_friction, self.lock_detent)
        self.key_mode[index] = "turned"

    def _seed_push(self, index: int) -> str:
        """Warm-start the arm's IK at the first push pose from the seeded search, as `_seed_arm` does the insertion's."""
        from demos.cup.pick_demo.grasp import _seeds, solve_link6
        from demos.unifp import decoupled_arm

        data = self._robot.data
        base_pos = (data.root_pos_w[index] - self.scene.env_origins[index]).cpu().numpy()
        base_rot = door_env.head_camera.quat_to_matrix(data.root_quat_w[index].cpu().numpy())
        spawn_pos = self._spawn_pos[index].cpu().numpy()
        spawn_rot = door_env.head_camera.quat_to_matrix(self._spawn_yaw_quat[index].cpu().numpy())
        state = self._state_of(index)
        to_base = base_rot.T
        point_b = to_base @ (spawn_pos + spawn_rot @ np.asarray(_jaw(state, "out", at.START_DEG)) - base_pos)
        hand = _hand(state, at.START_DEG)
        rot = decoupled_arm.hand_rotation(to_base @ spawn_rot @ hand[:, 2], to_base @ spawn_rot @ hand[:, 0], 0.0, 1)
        arm = self.arms[index]
        result = solve_link6(arm.joints, arm.links, point_b, rot, _seeds(point_b, [arm.q]), arm.offset)
        if result is None:
            return "no seeded solution"
        arm.q = np.asarray(result.q, dtype=float)
        return "seeded"

    def arm_torque_use(self) -> torch.Tensor:
        """Each arm joint's torque as PhysX applied it, as a share of its limit (N, 6)."""
        forces = self._robot.root_physx_view.get_dof_projected_joint_forces()[:, self._arm_ids]
        return forces.abs() / self.arm_effort

    def _stance_record(self, index: int) -> dict:
        """Where the base stopped against the stance the estimate asked for, and against the true box's."""
        x, y, yaw = self._base_now[index]
        want = self._stance_target(index, "walk_to_key")
        truth = key_env.floor_site(self.truth_sites[index])
        lock = truth[:3, :3] @ np.asarray(kg.LOCK) + truth[:3, 3]
        rel = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]]) @ (lock[:2] - np.array([x, y]))
        data = self._robot.data
        return {"stance_error_m": round(math.hypot(want[0] - x, want[1] - y), 4),
                "door_ahead_m": round(float(rel[0]), 4), "lock_left_m": round(float(rel[1]), 4),
                "base_z_m": round(float(data.root_pos_w[index, 2] - self.scene.env_origins[index, 2]), 4)}

    def _push_record(self, index: int) -> dict:
        use = self.arm_torque_use()[index].cpu().numpy()
        return {"key_deg": round(math.degrees(float(self.leg._handle.theta[index])), 2),
                "measured_deg": round(float(self.leg.measured_deg()[index]), 2),
                "contact_n": round(float(self.leg.force_w[index].norm()), 2),
                "torque_use": [round(float(v), 3) for v in use], "ik_last": self.ik_status[index],
                "ik": dict(self.ik_counts[index])}

    # --- the key after the push --------------------------------------------------------------------

    def _place_key(self) -> None:
        """The parent's, and a key left where the push left it ("left_turned") once the plant is off."""
        super()._place_key()
        writes = [(i, self.key_pose_env(i, self._truth_env(i), self._frozen_deg[i]))
                  for i, m in enumerate(self.key_mode) if m == "left_turned"]
        if writes:
            ids = torch.tensor([w[0] for w in writes], device=self.device)
            state = self._key.data.default_root_state[ids].clone()
            for row, (i, (pos, q)) in enumerate(writes):
                state[row, :3] = torch.as_tensor(pos, dtype=torch.float32, device=self.device) + self.scene.env_origins[i]
                state[row, 3:7] = torch.as_tensor(q, dtype=torch.float32, device=self.device)
            state[:, 7:] = 0.0
            self._key.write_root_pose_to_sim(state[:, :7], env_ids=ids)
            self._key.write_root_velocity_to_sim(state[:, 7:], env_ids=ids)

    def _reset_idx(self, env_ids) -> None:
        super()._reset_idx(env_ids)
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        if not hasattr(self, "_frozen_deg"):
            self._frozen_deg = [at.START_DEG] * self.num_envs
            self._accepted = [False] * self.num_envs
        for index in env_ids.tolist():
            # The push's hand: `object_axis` is Link6's x, so the grasp is (pitch 0, roll +1) of `hand_rotation`.
            self.arms[index].grasp, self.arms[index].grasp_note = (0.0, 1), "the key push's hand (pitch 0, roll +1)"
            self._frozen_deg[index] = at.START_DEG
            self._accepted[index] = False
