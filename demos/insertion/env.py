"""The insertion demo's environment: the training's task with a PhysX door and key in place of its analytic slot.

`unifp_train.insert_env.Go2D1InsertEnv` is reused whole -- the task layer, the approach and roll commands, the force
law, the policy's observation -- with three differences:

  * the slot is geometry: a kinematic door (`asset.py`) placed per attempt at the *true* slot pose, the estimate moved
    by the plan's errors exactly as the training moves its analytic slot; the key is a collider on the hand
    (`key.py`); PhysX makes every contact force and no analytic force is applied;
  * the estimate is fixed in the world from the moment the attempt starts, as a tag seen before the approach would
    fix it, where the training holds it in the robot's frame until the task layer starts (the outer loops,
    `insertion.PoseCorrection`, are the training's, and aim at this estimate);
  * the contact force on the key is read off the arm's contact sensor (`Link6`, which carries a rigid blade, or the
    `key` link a compliant one hangs on: `key.py`), and so is the key's pose.

The training's analytic `Slot` is still stepped, on the true geometry, but only for its bookkeeping -- the tip's depth
past the true face, and whether it is seated -- never for a force. Depth is a fact of the geometry, and the key can
only be that deep through the channel.

Drawn, and taking no part in the physics: the key in the hand (`asset.build_key_usd`, on the blade's link); for
watching, the door the lock is in (`asset.build_scenery_usd`, placed upright at the true slot each attempt) and a floor
(`asset.build_floor_usd`); and the wrist RealSense and its bracket on Link6 (`demos/cup/pick_demo`'s assets, at the
saved mount). They add no mass: the policy was trained, and is run here, without the camera's weight.
"""
from __future__ import annotations

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObject, RigidObjectCfg
from isaaclab.utils import configclass

from unifp_isaaclab import interface
from unifp_train import insert_cfg, insertion, rewards
from unifp_train.env import Go2D1PosForceEnv
from unifp_train.insert_env import Go2D1InsertEnv, Go2D1InsertEnvCfg

from .geometry import frame_quat
from .key import key_link


@configclass
class InsertionDemoEnvCfg(Go2D1InsertEnvCfg):
    insertion_fraction: float = 1.0
    curriculum: bool = False
    door: RigidObjectCfg | None = None
    #: The blade on its own sprung link (`key.write_key_urdf(compliant=True)`) rather than rigid on Link6.
    compliant_key: bool = False
    #: Drawn only, each None to leave it out: the key (`asset.build_key_usd`, for the same `compliant_key`); the door
    #: around the lock (`asset.build_scenery_usd`) and the floor (`asset.build_floor_usd`); the RealSense's case
    #: (`camera_asset.build_camera_usd`) at `camera_body_pose`, ((x, y, z), (w, x, y, z)) in Link6; its bracket
    #: (`camera_asset.build_bracket_usd`), authored in Link6.
    key_usd: str | None = None
    scenery_usd: str | None = None
    floor_usd: str | None = None
    camera_body_usd: str | None = None
    camera_body_pose: tuple | None = None
    bracket_usd: str | None = None


class InsertionDemoEnv(Go2D1InsertEnv):
    cfg: InsertionDemoEnvCfg

    def __init__(self, cfg: InsertionDemoEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, device = self.num_envs, self.device
        blade = "key" if cfg.compliant_key else "Link6"
        self._link6_contact = self._arm_contact.body_names.index(blade)
        self._key_body = self._robot.body_names.index("key") if cfg.compliant_key else None
        self._est_mouth = torch.zeros(n, 3, device=device)
        self._est_axis = torch.zeros(n, 3, device=device)
        self._key_force_w = torch.zeros(n, 3, device=device)
        self._scenery = None
        if cfg.scenery_usd:
            from isaaclab.sim.views import XformPrimView

            self._scenery = XformPrimView("/World/envs/env_.*/Scenery", device=str(device), sync_usd_on_fabric_write=True)

    def _setup_scene(self):
        super()._setup_scene()
        self._door = RigidObject(self.cfg.door)
        self.scene.rigid_objects["door"] = self._door
        # The visual-only pieces, spawned into every environment (the spawner clones over the regex).
        for name, usd in (("Scenery", self.cfg.scenery_usd), ("Floor", self.cfg.floor_usd)):
            if usd:
                spawn = sim_utils.UsdFileCfg(usd_path=usd)
                spawn.func(f"/World/envs/env_.*/{name}", spawn)
        if self.cfg.key_usd:
            spawn = sim_utils.UsdFileCfg(usd_path=self.cfg.key_usd)
            spawn.func(f"{self.cfg.robot.prim_path}/D1/{key_link(self.cfg.compliant_key)[0]}/KeyVisual", spawn)
        link6 = f"{self.cfg.robot.prim_path}/D1/Link6"
        if self.cfg.camera_body_usd:
            pos, rot = self.cfg.camera_body_pose
            spawn = sim_utils.UsdFileCfg(usd_path=self.cfg.camera_body_usd)
            spawn.func(f"{link6}/rs_body", spawn, translation=tuple(pos), orientation=tuple(rot))
        if self.cfg.bracket_usd:
            spawn = sim_utils.UsdFileCfg(usd_path=self.cfg.bracket_usd)
            spawn.func(f"{link6}/rs_bracket", spawn)

    # --- the key's contact is PhysX's --------------------------------------------------------------------------

    def _apply_action(self) -> None:
        Go2D1PosForceEnv._apply_action(self)
        key = self.key_state()
        self._slot.step(key["tip"], key["vel"], key["axis"], key["long"], interface.SIM_DT)   # bookkeeping only
        self._key_force_w = self._arm_contact.data.net_forces_w[:, self._link6_contact]
        self._ee_force_w = self._key_force_w
        force = self._key_force_w.norm(dim=-1)
        self._peak_contact = torch.maximum(self._peak_contact, force * self._engaged)
        self._damage_now |= force > insert_cfg.DAMAGE_N

    def _write_wrench(self) -> None:
        """Nothing: the door pushes back through PhysX."""

    def key_state(self) -> dict[str, torch.Tensor]:
        """A compliant key's state from its own link, whose frame sits at the jaw centre (the pivot); else Link6's."""
        if self._key_body is None:
            return super().key_state()
        data, b = self._robot.data, self._key_body
        quat = data.body_quat_w[:, b]
        pos = data.body_pos_w[:, b] - self.scene.env_origins
        unit = lambda v: torch.tensor(v, device=self.device).expand(self.num_envs, 3).contiguous()
        arm = interface.quat_apply(quat, unit([0.0, 0.0, insertion.KEY_LENGTH_M]))
        vel = data.body_lin_vel_w[:, b] + torch.cross(data.body_ang_vel_w[:, b], arm, dim=-1)
        return {"tip": pos + arm, "vel": vel, "axis": interface.quat_apply(quat, unit([0.0, 0.0, 1.0])),
                "long": interface.quat_apply(quat, unit([0.0, 1.0, 0.0])), "offset_local": None}

    # --- the estimate is fixed in the world -------------------------------------------------------------------

    def _planned_world(self, ids: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        return self._est_mouth[ids], self._est_axis[ids]

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        super()._reset_idx(env_ids)
        # The estimate, from the robot's spawn pose; then the door at the true pose it misses by.
        # The pose the reset just wrote (the data buffers carry it before the next physics step), reduced to its yaw.
        ground = self._robot.data.root_pos_w[env_ids] - self.scene.env_origins[env_ids]
        ground[:, 2] = 0.0
        yaw = interface.yaw_quat(interface.yaw_from_quat(self._robot.data.root_quat_w[env_ids]))
        mouth_local, elevation = self._mouth_local[env_ids], self._elevation[env_ids]
        bearing = torch.atan2(mouth_local[:, 1], mouth_local[:, 0])
        self._est_mouth[env_ids] = ground + interface.quat_apply(yaw, mouth_local)
        self._est_axis[env_ids] = rewards.approach_direction(elevation, bearing, yaw)
        plan = self._evaluation
        axis, roll = self._est_axis[env_ids], self._roll[env_ids]
        u, w = insertion.roll_frame(axis, roll)
        lateral, tilt = plan["lateral"][env_ids], plan["tilt"][env_ids]
        true_mouth = self._est_mouth[env_ids] + lateral[:, :1] * u + lateral[:, 1:] * w
        true_axis = insertion.unit(axis + tilt[:, :1] * u + tilt[:, 1:] * w)
        true_u, true_w = insertion.roll_frame(true_axis, roll + plan["roll_error"][env_ids])
        pose = torch.cat((true_mouth + self.scene.env_origins[env_ids], frame_quat(true_axis, true_u, true_w)), dim=-1)
        self._door.write_root_pose_to_sim(pose, env_ids=env_ids)
        self._door.write_root_velocity_to_sim(torch.zeros(len(env_ids), 6, device=self.device), env_ids=env_ids)
        if self._scenery is not None:
            # The door at the same mouth and axis, but upright: its z is the world's up, square to the axis.
            up = torch.zeros_like(true_axis)
            up[:, 2] = 1.0
            z = insertion.unit(up - (up * true_axis).sum(-1, keepdim=True) * true_axis)
            y = torch.cross(z, true_axis, dim=-1)
            self._scenery.set_world_poses(true_mouth + self.scene.env_origins[env_ids], frame_quat(true_axis, y, z),
                                          indices=env_ids.tolist())
