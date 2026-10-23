"""The key drawn out of Lukas's holster beside the D1's base, in simulation: the combiner key demo's robot standing on
P2Dingo's walking policy, the arm scripted through `holster.py`'s draw at 10 Hz (the arm IK and the D1's planner and
servo model, as every demo arm here).

On top of `env.CombinerKeyEnv` (its robot, its key's spring grip):
- **The holster** (`asset.build_holster_usd`) is a kinematic body carried with the robot's base every physics step, at
  `holster.key_in_d1_base()` on the arm's mount: part of the robot, but its colliders are its own (boxes,
  `holster.HOLSTER_BOXES`). Its contacts with the robot's own links are filtered out; with the key they are not.
- **The key** is the simulation's (v3 mirrored, `asset.build_holster_key_usd`), a free rigid body with its bar, platform and shaft as
  colliders. Until the hand comes for it it is held in the holster's pose (`holstered`); from `pre` on PhysX alone
  keeps it there, resting in the holster's boxes. Its contacts with the robot are not filtered: a finger, the D1's base
  or a link it touches pushes it.
- **The draw** (`holster_phases`): Joint1 turned with the arm still folded, the arm unfolded in joint space to the
  hand 45 mm out along the lever's axis past its root, jaws open (`holster.unfold_stages`, solved from the robot's base
  as it stands); then the IK slides the hand onto the bar's root, the jaws close, and the hand draws the key out along
  the axis 100 mm and holds it. The points and the hand's rotation are `holster.draw_points` in the base's current
  frame, so the base's own sway is followed.
- **A misplaced holster** (`holster_shift`): the holster and the key in it put off from where the draw is planned, to
  see what the jaws' 10 mm a side round the bar take up.
- **The grip** is the key demo's spring (`env.GRIP_*`), switched on as the draw starts, at the key's pose in the hand
  where the jaws closed on it (not where they were meant to: how far that was off is the record's `grasp_miss`). The
  fingers close round the bar as well, and they do collide with it.

Not modelled: the holster's cup round the bar's rounded end (the bar's collider ends square, `holster.KEY_BOXES`),
friction of PLA on PETG beyond a constant, the holster's own flex, the wrist RealSense's mass (the demo's robot draws
the payload only).

Needs the Isaac app: import after `AppLauncher`.
"""
from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObject, RigidObjectCfg
from isaaclab.sensors import Camera, CameraCfg

import d1_ik
from demos.combiner_door import env as door_env
from demos.cup.pick_demo.grasp import _seeds, arm_clear
from demos.unifp import script as demo_script
from mount import ARM_MOUNT_POS_M
from unifp_isaaclab import interface

from . import asset, env as key_env, holster

#: The phases' lengths at `speed` 1, s: turning Joint1 at the fold, the unfold, the wait at `pre`, onto the bar, the
#: jaws closing, the draw, the hold after it. `holster_phases(speed)` divides the moves' (turn, reach, on, draw) by it;
#: the waits and the jaws keep theirs.
TURN_S, REACH_S, PRE_S, ON_S, CLOSE_S, DRAW_S, HOLD_S = 2.5, 3.5, 1.5, 3.0, 1.0, 5.0, 2.0
#: The joint-space moves' step per 10 Hz command at `speed` 1 (`DecoupledDoorEnv.joint_step_rad`'s 4 degrees), and the
#: most it is raised to: 6.5 degrees a command is 1.13 rad/s, under the D1's measured 1.21-1.29
#: (`motor_model.D1_VELOCITY_LIMIT_RAD_S`).
JOINT_STEP_DEG, JOINT_STEP_MAX_DEG = 4.0, 6.5
#: The defaults (`run_holster.py`): the unfold twice as fast, the slide onto the bar and the draw paced at 0.6 rad/s on
#: the fastest joint. Against the even 1x (8 attempts each, the holster up to 3 mm off, 2026-10-23): the sequence 12.3 s
#: not 18.5, the draw's fastest joint near-stopped in 8% of 20 ms samples not 28%, the hand's worst lag 7.9 mm not 6.2,
#: the grip's worst error 3.2 mm not 3.6. Twice as fast evenly lagged 37 mm (the wrist roll and Joint1 at ~1.25 rad/s);
#: paced at 1.0 rad/s, 14 mm.
SPEED, JOINT_SPEED_RAD_S = 2.0, 0.6
DRAW_PHASES = ("turn", "reach", "pre", "on", "close", "draw", "hold")
#: Pictures every this often from `turn` on.
FRAME_EVERY_S = 0.25
#: The key out: its far end past the holster's open end by this much more than it started inside it, along the axis.
OUT_MM = 75.0


def holster_phases(speed: float = SPEED, joint_speed: float | None = JOINT_SPEED_RAD_S) -> list[demo_script.Phase]:
    """After the spawn's settle: the unfold in two joint-space moves, then the draw by the IK; the moves `speed` times
    as fast as the defaults. With `joint_speed` (rad/s), the two straight moves (`on`, `draw`) are paced instead so the
    fastest joint turns at that speed throughout (`holster.joint_paced`), not eased in and out evenly along the line.

    Faster is smoother on the D1, to a point: its firmware plans each 10 Hz setpoint from rest (`motor_model`, F-045),
    so a joint asked for less than ~2.5 degrees a command gets there inside the 100 ms and stops until the next one."""
    point = lambda name: (lambda st, u: st["draw"][name])
    along = lambda a, b: (lambda st, u: tuple(np.asarray(st["draw"][a]) + _smooth(u) * (np.asarray(st["draw"][b])
                                                                                        - np.asarray(st["draw"][a]))))
    hand = dict(approach=lambda st: st["draw"]["approach"], object_axis=lambda st: st["draw"]["bar"])
    opened, held = holster.JAW_OPEN_M, holster.JAW_HOLD_M
    on_s, draw_s = ON_S / speed, DRAW_S / speed
    if joint_speed is not None:
        paced = holster.joint_paced(joint_speed)
        (on_s, on_u, on_f), (draw_s, draw_u, draw_f) = paced["on"], paced["draw"]
        line = lambda a, b, us, fs: (lambda st, u: tuple(np.asarray(st["draw"][a]) + float(np.interp(u, us, fs))
                                                         * (np.asarray(st["draw"][b]) - np.asarray(st["draw"][a]))))
        on_path, draw_path = line("pre", "grip", on_u, on_f), line("grip", "out", draw_u, draw_f)
    else:
        on_path, draw_path = along("pre", "grip"), along("grip", "out")
    return [demo_script.Phase("turn", TURN_S / speed, path=point("pre"), jaw_m=opened, **hand),
            demo_script.Phase("reach", REACH_S / speed, path=point("pre"), jaw_m=opened, **hand),
            demo_script.Phase("pre", PRE_S, path=point("pre"), jaw_m=opened, **hand),
            demo_script.Phase("on", on_s, path=on_path, jaw_m=opened, **hand),
            demo_script.Phase("close", CLOSE_S, path=point("grip"), jaw_m=held, **hand),
            demo_script.Phase("draw", draw_s, path=draw_path, jaw_m=held, **hand),
            demo_script.Phase("hold", HOLD_S, path=point("out"), jaw_m=held, **hand)]


def _smooth(u: float) -> float:
    return u * u * (3.0 - 2.0 * u)


class _NoTurn:
    """`CombinerKeyEnv`'s turn slot, empty: nothing turns a key here."""

    def __init__(self, n: int, device):
        self.active = torch.zeros(n, dtype=torch.bool, device=device)
        self._handle = SimpleNamespace(theta=torch.zeros(n, device=device))

    def finished(self) -> torch.Tensor:
        return torch.zeros_like(self.active)

    def stop(self, index: int) -> None:
        pass

    def physics_step(self, dt: float) -> None:
        pass


class HolsterEnv(key_env.CombinerKeyEnv):
    def __init__(self, cfg, render_mode=None, **kwargs):
        cfg.given_box = True
        super().__init__(cfg, render_mode, **kwargs)
        speed = float(getattr(cfg, "holster_speed", SPEED))
        self.joint_step_rad = math.radians(min(JOINT_STEP_DEG * speed, JOINT_STEP_MAX_DEG))
        self.parked_phases = {"spawn"}
        self.position_only_phases = set()
        self.joint_phases = {"turn", "reach"}
        from motor_model import D1_EFFORT_LIMIT_NM
        from demos.unifp.decoupled import ARM_NAMES

        self.arm_effort = torch.as_tensor([D1_EFFORT_LIMIT_NM[name] for name in ARM_NAMES], device=self.device)
        mount = np.eye(4)
        mount[:3, 3] = ARM_MOUNT_POS_M
        #: The holstered key's frame (and the holster's prim frame) in the Go2's base.
        self.key_in_base = mount @ holster.key_in_d1_base()
        self._key_in_base_t = torch.as_tensor(self.key_in_base[:3, 3], dtype=torch.float32, device=self.device)
        self._key_quat_base_t = torch.as_tensor(key_env._matrix_quat(self.key_in_base[:3, :3]), dtype=torch.float32,
                                                device=self.device)
        #: Where each environment's holster really is against where the draw is planned for: a shift in the base's
        #: frame, metres (`run_holster.py --holster_error_mm`). The plan never sees it.
        self.holster_shift = torch.zeros(self.num_envs, 3, device=self.device)

    def _make_turn(self, cfg):
        return _NoTurn(self.num_envs, self.device)

    # --- scene ------------------------------------------------------------------------------------

    def _setup_scene(self):
        door_env.DecoupledDoorEnv._setup_scene(self)
        out = door_env.head_camera.ROOT / "generated/combiner_key"
        key_usd = asset.build_holster_key_usd(out / "holster_key.usda")
        holster_usd = asset.build_holster_usd(out / "holster.usda")
        self._key = RigidObject(RigidObjectCfg(
            prim_path="/World/envs/env_.*/Key",
            spawn=sim_utils.UsdFileCfg(
                usd_path=str(key_usd),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(solver_position_iteration_count=16,
                                                             solver_velocity_iteration_count=4,
                                                             max_depenetration_velocity=0.5, disable_gravity=False),
                mass_props=sim_utils.MassPropertiesCfg(mass=key_env.KEY_SIM_MASS_KG)),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, key_env.PARK_Z_M))))
        self._holster = RigidObject(RigidObjectCfg(
            prim_path="/World/envs/env_.*/Holster",
            spawn=sim_utils.UsdFileCfg(usd_path=str(holster_usd),
                                       rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True)),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, key_env.PARK_Z_M + 1.0))))
        self.scene.rigid_objects["key"] = self._key
        self.scene.rigid_objects["holster"] = self._holster
        self._filter_holster()
        self._overview = None
        if getattr(self.cfg, "overview", True):
            self._overview = Camera(CameraCfg(
                prim_path="/World/envs/env_.*/OverviewCam", update_period=0.0, width=960, height=720, data_types=["rgb"],
                spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, clipping_range=(0.02, 20.0))))
            self.scene.sensors["overview_cam"] = self._overview

    def _filter_holster(self) -> None:
        """The holster is part of the robot: no contacts between it and the robot's links."""
        from pxr import UsdPhysics

        stage = sim_utils.get_current_stage()
        for index in range(self.cfg.scene.num_envs):
            prim = stage.GetPrimAtPath(f"/World/envs/env_{index}/Holster")
            if prim.IsValid():
                UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().AddTarget(
                    f"/World/envs/env_{index}/Robot")

    def _set_key_inertia(self) -> None:
        """The key demo's: its key frame is this key's (`holster.py`), so are its centre of mass and raised inertia."""
        view = self._key.root_physx_view
        coms = view.get_coms().clone()
        coms[:, :3] = torch.tensor(key_env.KEY_COM_M)
        view.set_coms(coms, torch.arange(view.count))
        inertias = view.get_inertias().clone()
        inertias[:] = 0.0
        inertias[:, 0], inertias[:, 4], inertias[:, 8] = key_env.KEY_INERTIA_KG_M2
        view.set_inertias(inertias, torch.arange(view.count))

    # --- the camera ---------------------------------------------------------------------------------

    def overview_view(self, index: int) -> tuple[np.ndarray, np.ndarray]:
        """(eye, target) in the world: from the robot's front right and above, onto the holster and the hand."""
        data = self._robot.data
        pos = data.root_pos_w[index].cpu().numpy()
        rot = door_env.head_camera.quat_to_matrix(data.root_quat_w[index].cpu().numpy())
        target = self.key_in_base[:3, 3] + np.array([0.02, 0.0, 0.03])
        eye = target + np.array([0.42, -0.42, 0.30])
        return pos + rot @ eye, pos + rot @ target

    def aim_overview(self) -> None:
        eyes, targets = zip(*(self.overview_view(i) for i in range(self.num_envs)))
        t = lambda v: torch.as_tensor(np.stack(v), dtype=torch.float32, device=self.device)
        self._overview.set_world_poses_from_view(t(eyes), t(targets))

    def overview_rgb(self, index: int = 0) -> np.ndarray:
        return self._overview.data.output["rgb"][index, ..., :3].cpu().numpy().astype(np.uint8)

    # --- the script's state: the draw in the base's current frame -----------------------------------

    def _base_pose(self, index: int) -> tuple[np.ndarray, np.ndarray]:
        data = self._robot.data
        return ((data.root_pos_w[index] - self.scene.env_origins[index]).cpu().numpy(),
                door_env.head_camera.quat_to_matrix(data.root_quat_w[index].cpu().numpy()))

    def _state_of(self, index: int) -> dict:
        state = super()._state_of(index)
        base_pos, base_rot = self._base_pose(index)
        spawn_pos = self._spawn_pos[index].cpu().numpy()
        spawn_rot = door_env.head_camera.quat_to_matrix(self._spawn_yaw_quat[index].cpu().numpy())
        to_script = lambda p: tuple(spawn_rot.T @ (base_pos + base_rot @ np.asarray(p) - spawn_pos))
        turn = lambda v: tuple(spawn_rot.T @ base_rot @ np.asarray(v))
        pts = holster.draw_points(self.key_in_base)
        axes = holster.key_axes(self.key_in_base)
        state["draw"] = {name: to_script(pts[name]) for name in ("pre", "grip", "out")}
        # `hand_rotation(approach, bar, 0, GRIP_ROLL)` is `holster.grip_rotation`: approach along the shaft, bar along u.
        state["draw"]["approach"], state["draw"]["bar"] = turn(-axes["n"]), turn(axes["u"])
        return state

    # --- the phases --------------------------------------------------------------------------------

    def _enter(self, index: int, phase: str, previous: str | None) -> None:
        record = self.records[index]
        arm = self.arms[index]
        if phase == "turn":
            q_pre, note = self._solve_pre(index)
            record["pre_solve"] = note
            self._q_pre = getattr(self, "_q_pre", {})
            self._q_pre[index] = q_pre
            stages = holster.unfold_stages(self._rest_arm(), q_pre)
            self._joint_goals[index] = ("turn", stages[0])
        elif phase == "reach":
            self._joint_goals[index] = ("reach", self._q_pre[index])
        elif phase == "pre":
            # The IK loop warm-starts from its last solution: here, the unfold's goal, on the draw's branch.
            arm.q = np.asarray(self._q_pre[index], dtype=float).copy()
            self.key_mode[index] = "free"
            record["key_released_at"] = self._key_record(index)
        elif phase == "draw":
            record["grasp"] = self._take_hold(index)
        elif phase == "hold":
            record["drawn"] = self._key_record(index)

    def _solve_pre(self, index: int) -> tuple[np.ndarray, str]:
        """The arm's joints with the hand at `pre`, seeded from the folded rest as `holster.check` solves it."""
        pts = holster.draw_points(self.key_in_base)       # the IK's frame is the base's: the holster is fixed in it
        arm = self.arms[index]
        rest = self._rest_arm()
        offset = (0.0, 0.0, holster.JAW_CENTRE_Z_M)
        for seed in [rest] + _seeds(pts["pre"]):
            r = d1_ik.solve(arm.joints, pts["pre"], pts["rot"], q0=seed, body="Link6", offset=offset,
                            max_iterations=400, soft=holster.IK_SOFT)
            if r.converged and bool(arm_clear(arm.joints, r.q)[0]):
                return np.asarray(r.q, dtype=float), "solved"
        return np.asarray(r.q, dtype=float), f"not converged: {1000 * r.position_error_m:.1f} mm"

    def _take_hold(self, index: int) -> dict:
        """The grip's spring on, at the key's pose in the hand now; how far that is from the grip meant
        (`holster.KEY_ORIGIN_LINK6` with `holster.grip_rotation`)."""
        data = self._robot.data
        link_p, link_q = data.body_pos_w[index, self._link6], data.body_quat_w[index, self._link6]
        key = self._key.data
        rel_p = interface.quat_rotate_inverse(link_q[None], (key.root_pos_w[index] - link_p)[None])[0]
        rel_q = key_env._quat_mul(key_env._quat_conj(link_q[None]), key.root_quat_w[index][None])[0]
        self._key_in_link6[index] = rel_p
        self._key_quat_link6[index] = rel_q
        self._grip[index] = True
        self.key_mode[index] = "held"
        # Where the grip meant the key: the key frame in Link6 is the grip's rotation, inverted, and `KEY_ORIGIN_LINK6`.
        want = holster.grip_rotation(holster.key_axes(np.eye(4))).T
        got = door_env.head_camera.quat_to_matrix(rel_q.cpu().numpy())
        miss = rel_p.cpu().numpy() - np.asarray(holster.KEY_ORIGIN_LINK6)
        angle = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(want.T @ got) - 1.0) / 2.0))))
        return {"grasp_miss_mm": [round(1000 * float(v), 2) for v in miss], "grasp_miss_deg": round(angle, 2),
                "jaw_axis_miss_mm": round(1000 * float(miss[1]), 2), **self._key_record(index)}

    # --- where the key is ---------------------------------------------------------------------------

    def key_pose_base(self, index: int) -> np.ndarray:
        """The key frame (`holster.py`) in the robot's base frame, 4x4."""
        base_pos, base_rot = self._base_pose(index)
        key = self._key.data
        pos = (key.root_pos_w[index] - self.scene.env_origins[index]).cpu().numpy()
        rot = door_env.head_camera.quat_to_matrix(key.root_quat_w[index].cpu().numpy())
        out = np.eye(4)
        out[:3, :3], out[:3, 3] = base_rot.T @ rot, base_rot.T @ (pos - base_pos)
        return out

    def key_out_mm(self, index: int) -> tuple[float, float, float]:
        """(out along the holster's axis, across it, tilt deg): how far the key is from holstered, in the holster's frame
        (where it really is, `holster_shift` included)."""
        real = self.key_in_base.copy()
        real[:3, 3] += self.holster_shift[index].cpu().numpy()
        rel = np.linalg.inv(real) @ self.key_pose_base(index)
        along = 1000 * float(np.asarray(holster.SLIDE_OUT_KEY) @ rel[:3, 3])
        across = 1000 * float(math.hypot(rel[0, 3], rel[2, 3]))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rel[1, 1])))))
        return along, across, tilt

    def _key_record(self, index: int) -> dict:
        along, across, tilt = self.key_out_mm(index)
        return {"key_out_mm": round(along, 2), "key_across_mm": round(across, 2), "key_tilt_deg": round(tilt, 2),
                "grip_err_mm": round(float(self.grip_err_mm[index]), 2),
                "grip_err_deg": round(float(self.grip_err_deg[index]), 2),
                "ik_last": self.ik_status[index], "ik": dict(self.ik_counts[index])}

    def arm_torque_use(self) -> torch.Tensor:
        """Each arm joint's torque as PhysX applied it, as a share of its limit (N, 6)."""
        forces = self._robot.root_physx_view.get_dof_projected_joint_forces()[:, self._arm_ids]
        return forces.abs() / self.arm_effort

    # --- physics ------------------------------------------------------------------------------------

    def _place_key(self) -> None:
        """The holster carried with the base; the key in it while `holstered`; then the parent's grip."""
        data = self._robot.data
        n = self.num_envs
        pos = data.root_pos_w + interface.quat_apply(data.root_quat_w, self._key_in_base_t.expand(n, 3) + self.holster_shift)
        quat = key_env._quat_mul(data.root_quat_w, self._key_quat_base_t.expand(n, 4))
        pose = torch.cat([pos, quat], dim=-1)
        self._holster.write_root_pose_to_sim(pose)
        self._holster.write_root_velocity_to_sim(torch.zeros(n, 6, device=self.device))
        ids = [i for i, mode in enumerate(self.key_mode) if mode == "holstered"]
        if ids:
            idx = torch.tensor(ids, device=self.device)
            self._key.write_root_pose_to_sim(pose[idx], env_ids=idx)
            self._key.write_root_velocity_to_sim(torch.zeros(len(ids), 6, device=self.device), env_ids=idx)
        super()._place_key()

    def _reset_idx(self, env_ids) -> None:
        super()._reset_idx(env_ids)
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        for index in env_ids.tolist():
            self.key_mode[index] = "holstered"
            self.arms[index].grasp, self.arms[index].grasp_note = (0.0, holster.GRIP_ROLL), "the holster's grip"
