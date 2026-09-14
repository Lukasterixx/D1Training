"""World-anchored reaching commands for the initial free-space experiment."""
from __future__ import annotations

import torch

from isaaclab.envs.mdp.actions import JointPositionAction
from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg
from isaaclab.managers import CommandTerm, CommandTermCfg
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.sim import SphereCfg, PreviewSurfaceCfg
from isaaclab.utils import configclass

from .core import point_in_world, tracking_reward, world_to_body


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
        # Link origin, not center of mass: the registered offset belongs to Link6.
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

    def _set_debug_vis_impl(self, debug_vis):
        if debug_vis and not hasattr(self, "_markers"):
            self._markers = VisualizationMarkers(VisualizationMarkersCfg(
                prim_path="/Visuals/PositionOnly/targets",
                markers={"target": SphereCfg(
                    radius=0.015,
                    visual_material=PreviewSurfaceCfg(diffuse_color=(0.1, 0.9, 0.2)),
                )},
            ))
        if hasattr(self, "_markers"):
            self._markers.set_visibility(debug_vis)

    def _debug_vis_callback(self, event):
        if self.robot.is_initialized:
            self._markers.visualize(self.target_w)


@configclass
class WorldPositionCommandCfg(CommandTermCfg):
    class_type: type = WorldPositionCommand
    asset_name: str = "robot"
    body_name: str = "Link6"
    # Provisional Link6 origin. Register the physical grasp/tool point before
    # reporting tool-tip metrics; --tip_offset exposes this explicitly.
    tip_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
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
