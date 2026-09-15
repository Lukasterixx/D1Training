"""World-anchored reaching commands for the initial free-space experiment."""
from __future__ import annotations

import torch

from isaaclab.envs.mdp.actions import JointPositionAction
from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg
from isaaclab.managers import CommandTerm, CommandTermCfg, ManagerTermBase
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.sim import CuboidCfg, PreviewSurfaceCfg, SphereCfg
from isaaclab.utils import configclass

from .core import SampleAndHold, box_edges, point_in_world, tracking_reward, world_to_body
from .tool_point import TOOL_BODY, TOOL_OFFSET_M


class LimitedJointPositionAction(JointPositionAction):
    """Clamp PD targets to the articulation's soft position limits."""

    def process_actions(self, actions):
        super().process_actions(actions.clamp(-1.0, 1.0))
        limits = self._asset.data.soft_joint_pos_limits[:, self._joint_ids]
        self._processed_actions[:] = torch.clamp(
            self._processed_actions, min=limits[..., 0], max=limits[..., 1]
        )


@configclass
class LimitedJointPositionActionCfg(JointPositionActionCfg):
    class_type: type = LimitedJointPositionAction


class HeldJointPositionAction(LimitedJointPositionAction):
    """Pass new targets through only at the actuator interface's command rate.

    The D1 takes streamed joint-angle setpoints at about 10 Hz, so under a 50 Hz policy the arm
    sees every `hold_steps`-th output. Between updates the last target is held. The phase is random
    per environment, so the policy cannot rely on updates landing on particular steps. After a
    reset the arm holds its default pose until its first update.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._hold = SampleAndHold(self.num_envs, self.action_dim, cfg.hold_steps, env.step_dt, self.device)
        self._hold.reset(None, self._offset)

    def process_actions(self, actions):
        super().process_actions(actions)
        if self.cfg.hold_steps > 1:
            self._hold.update(self._env.episode_length_buf, self._processed_actions)
            self._processed_actions[:] = self._hold.value

    def reset(self, env_ids=None):
        super().reset(env_ids)
        ids = slice(None) if env_ids is None else env_ids
        self._hold.reset(env_ids, self._offset[ids])


@configclass
class HeldJointPositionActionCfg(LimitedJointPositionActionCfg):
    class_type: type = HeldJointPositionAction
    hold_steps: int = 1
    """Policy steps per command update (1 = every step)."""


class SampledJointFeedback(ManagerTermBase):
    """Joint angles (relative to default) or their velocity, as an angle-only feed reports them.

    The D1 publishes joint angles, not velocities, at 10 Hz. Angles are sampled every
    `period_steps` policy steps at a random phase; velocity is the difference of consecutive
    samples over the sample interval. Terms naming the same joints share one sampler, so angle and
    velocity come from the same feedback message. With `period_steps=1` both are exact.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        asset_cfg, period = cfg.params["asset_cfg"], cfg.params["period_steps"]
        registry = env.__dict__.setdefault("_sampled_joint_feedback", {})
        key = (asset_cfg.name, tuple(asset_cfg.joint_names), period)
        if key not in registry:
            robot = env.scene[asset_cfg.name]
            sampler = SampleAndHold(env.num_envs, len(asset_cfg.joint_ids), period, env.step_dt, env.device)
            sampler.reset(None, torch.zeros(env.num_envs, len(asset_cfg.joint_ids), device=env.device))
            registry[key] = (robot, asset_cfg.joint_ids, sampler)
        self._robot, self._joint_ids, self._sampler = registry[key]
        self._period = period

    def reset(self, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        count = self._sampler.value[ids].shape[0]
        # Idempotent: a second term on the same sampler redraws the phase but holds the same zeros.
        self._sampler.reset(env_ids, torch.zeros(count, self._sampler.value.shape[1], device=self.device))

    def __call__(self, env, asset_cfg, period_steps, quantity):
        data = self._robot.data
        pos = data.joint_pos[:, self._joint_ids] - data.default_joint_pos[:, self._joint_ids]
        if period_steps <= 1:
            return pos if quantity == "pos" else data.joint_vel[:, self._joint_ids] - data.default_joint_vel[:, self._joint_ids]
        self._sampler.update(env.episode_length_buf, pos)
        return self._sampler.value if quantity == "pos" else self._sampler.rate


SUCCESS_RADIUS_M = 0.05  # G1a: within 5 cm


class WorldPositionCommand(CommandTerm):
    """A static world goal per episode, exposed in the current robot base frame.

    The sampled coordinates are relative to each environment origin, not the
    live base. Moving or tilting the base must not move the target with it.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.robot = env.scene[cfg.asset_name]
        body_ids, _ = self.robot.find_bodies(cfg.body_name)
        if len(body_ids) != 1:
            raise ValueError(f"Expected one end-effector body, got {body_ids}.")
        self.body_id = body_ids[0]
        self.target_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.tip_offset = torch.tensor(cfg.tip_offset, device=self.device)
        self.metrics["position_error_m"] = torch.zeros(self.num_envs, device=self.device)

    @property
    def command(self):
        return world_to_body(self.target_w, self.robot.data.root_pos_w, self.robot.data.root_quat_w)

    @property
    def tip_w(self):
        # Link origin, not center of mass: the offset is in the controlled body's link frame.
        return point_in_world(
            self.robot.data.body_link_pos_w[:, self.body_id],
            self.robot.data.body_link_quat_w[:, self.body_id],
            self.tip_offset,
        )

    @property
    def error_m(self):
        return torch.linalg.vector_norm(self.tip_w - self.target_w, dim=-1)

    def _resample_command(self, env_ids):
        origins = self._env.scene.env_origins[env_ids]
        target = torch.empty_like(origins)
        for axis, bounds in enumerate(self.cfg.ranges):
            target[:, axis].uniform_(*bounds)
        self.target_w[env_ids] = origins + target

    def _update_metrics(self):
        # Diagnostic snapshot; formal episode metrics need the evaluation runner.
        self.metrics["position_error_m"][:] = self.error_m

    def _update_command(self):
        pass

    # Marker prototypes, in the order their indices are used below.
    _TARGET_FAR, _TARGET_MET, _TIP, _BOX_EDGE, _SPAWN = range(5)

    def _set_debug_vis_impl(self, debug_vis):
        if debug_vis and not hasattr(self, "_markers"):
            def surface(rgb):
                return PreviewSurfaceCfg(diffuse_color=rgb, emissive_color=tuple(0.35 * c for c in rgb))

            self._markers = VisualizationMarkers(VisualizationMarkersCfg(
                prim_path="/Visuals/PositionOnly/targets",
                markers={
                    "target_far": SphereCfg(radius=0.02, visual_material=surface((0.9, 0.15, 0.1))),
                    "target_met": SphereCfg(radius=0.02, visual_material=surface((0.1, 0.9, 0.2))),
                    "tip": SphereCfg(radius=0.012, visual_material=surface((0.15, 0.45, 1.0))),
                    "box_edge": CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=surface((1.0, 0.55, 0.1))),
                    "spawn": CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=surface((0.85, 0.85, 0.85))),
                },
            ))
            centres, sizes = box_edges(self.cfg.ranges)
            self._edge_centres, self._edge_sizes = centres.to(self.device), sizes.to(self.device)
        if hasattr(self, "_markers"):
            self._markers.set_visibility(debug_vis)

    def _debug_vis_callback(self, event):
        """Target (green once the controlled point is within G1a's 5 cm, red otherwise), the controlled
        point, the target box as a wireframe, and a ground plate under each robot's spawn position."""
        if not self.robot.is_initialized:
            return
        n, device = self.num_envs, self.device
        origins = self._env.scene.env_origins
        spawn = origins + self.robot.data.default_root_state[:, :3] * torch.tensor([1.0, 1.0, 0.0], device=device)
        met = (self.error_m <= SUCCESS_RADIUS_M).long()
        translations = torch.cat([self.target_w, self.tip_w, (origins[:, None] + self._edge_centres).reshape(-1, 3),
                                  spawn + torch.tensor([0.0, 0.0, 0.002], device=device)])
        ones = torch.ones(n, 3, device=device)
        scales = torch.cat([ones, ones, self._edge_sizes.repeat(n, 1),
                            torch.tensor([0.10, 0.10, 0.004], device=device).expand(n, 3)])
        indices = torch.cat([self._TARGET_FAR + met, torch.full((n,), self._TIP, device=device),
                             torch.full((12 * n,), self._BOX_EDGE, device=device),
                             torch.full((n,), self._SPAWN, device=device)])
        self._markers.visualize(translations, scales=scales, marker_indices=indices)


@configclass
class WorldPositionCommandCfg(CommandTermCfg):
    class_type: type = WorldPositionCommand
    asset_name: str = "robot"
    # Controlled point: the Link7_1 pincer tip (tool_point.py, CAD-derived). Until 2026-09-15 it was
    # the Link6 origin; --tip_body Link6 --tip_offset 0 0 0 restores that.
    body_name: str = TOOL_BODY
    tip_offset: tuple[float, float, float] = TOOL_OFFSET_M
    ranges: tuple = ((0.24, 0.36), (-0.08, 0.08), (0.66, 0.78))
    # Longer than the 10 s episode: one fixed target, resampled on reset only.
    resampling_time_range: tuple[float, float] = (1000.0, 1000.0)
    debug_vis: bool = False


def tip_position_b(env):
    term = env.command_manager.get_term("ee_position")
    robot = term.robot
    return world_to_body(term.tip_w, robot.data.root_pos_w, robot.data.root_quat_w)


def position_tracking(env, std: float):
    return tracking_reward(env.command_manager.get_term("ee_position").error_m, std)


def base_motion_l2(env):
    return torch.sum(torch.square(env.scene["robot"].data.root_lin_vel_b), dim=-1)


def outside_workspace(env, distance: float):
    delta = env.scene["robot"].data.root_pos_w - env.scene.env_origins
    return torch.linalg.vector_norm(delta[:, :2], dim=-1) > distance


def base_too_low(env, minimum_height: float):
    height = env.scene["robot"].data.root_pos_w[:, 2] - env.scene.env_origins[:, 2]
    return height < minimum_height
