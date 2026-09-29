"""The wiping task: the robot wipes a pad along a path on a panel it knows only from an estimate, at a set force.

What an episode is, for the `wipe_fraction` of environments that get one:

  1. A panel is drawn in the robot's yaw frame (`wipe_cfg.CENTRE_*`), facing it, and a path on it: a straight stroke or
     a zigzag (`wiping.line_path`, `zigzag_path`). The goal slides to the pre-contact pose -- the pad's centre `standoff`
     off the surface over the path's start, the hand's approach into the panel (`rewards.tracking_ee_orn_approach`),
     the jaws level -- and the approach is commanded from the start.
  2. Once the hand has settled there with the pad converged, the task layer (`wiping.WipeTaskLayer`) starts on that
     *estimate*, and the true surface (`wiping.Surface`) is placed off it by the curriculum's errors: along its normal
     and in tilt. The policy observes neither. Before contact the insertion's outer loops (`insertion.PoseCorrection`)
     integrate the pad's measured miss and the approach's elevation miss into the commands (F-118, F-120).
  3. The layer wipes: a PI force command along the estimated normal capped at the wiping force; once the pad has
     stalled against the surface, the goal along the path at the wiping speed and a PI force across the normal on the
     pad's miss of it; a lift at the end. The contact acts at the pad's centre of pressure. The velocity command is zero.
  4. Success is the plan's: at least 90% of the path wiped (`wiping.WIPE_RADIUS_M`) and contact during at least 80% of
     the motion; a normal force past `wipe_cfg.DAMAGE_N`, the pad through the panel, or a fall ends the episode.

The rest reach through free space with no pushes and no approach command. The policy's widths do not change.
"""
from __future__ import annotations

import math

import torch

from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.utils import configclass

from unifp_isaaclab import interface

from . import insertion, rewards, task_cfg, wipe_cfg, wipe_rewards, wiping
from .env import Go2D1PosForceEnv
from .env_cfg import Go2D1PosForceEnvCfg
from .hook_env import HeldGoals


@configclass
class Go2D1WipeEnvCfg(Go2D1PosForceEnvCfg):
    """UniFP's robot and observation, holding a wiping pad; the roll and approach objectives on."""

    roll_objective: bool = True
    force_start_step: int = 0
    wipe_fraction: float = wipe_cfg.WIPE_FRACTION
    curriculum: bool = True
    curriculum_level: float = wipe_cfg.CURRICULUM_START
    curriculum_min_iterations: int = wipe_cfg.CURRICULUM_MIN_ITERATIONS
    tip_correction_gain: float = wipe_cfg.TIP_CORRECTION_GAIN
    tip_correction_max_m: float = wipe_cfg.TIP_CORRECTION_MAX_M
    elevation_correction_gain: float = wipe_cfg.ELEVATION_CORRECTION_GAIN
    elevation_correction_max_rad: float = wipe_cfg.ELEVATION_CORRECTION_MAX_RAD
    arm_contact_sensor: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/D1/.*", history_length=0, update_period=0.0, track_air_time=False)


class Go2D1WipeEnv(Go2D1PosForceEnv):
    cfg: Go2D1WipeEnvCfg

    def __init__(self, cfg: Go2D1WipeEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, device = self.num_envs, self.device
        self._goals = HeldGoals.adopt(self._goals)
        self._surface = wiping.Surface(n, device)
        self._layer = wiping.WipeTaskLayer(
            n, device, kp=wipe_cfg.FORCE_LAW_KP_N_PER_M, ki=wipe_cfg.FORCE_LAW_KI_N_PER_M_S,
            kp_t=wipe_cfg.PATH_LAW_KP_N_PER_M, ki_t=wipe_cfg.PATH_LAW_KI_N_PER_M_S,
            tangent_ratio=wipe_cfg.PATH_LAW_CAP_RATIO)
        self._correction = insertion.PoseCorrection(
            n, device, tip_gain=cfg.tip_correction_gain, tip_max_m=cfg.tip_correction_max_m,
            elevation_gain=cfg.elevation_correction_gain, elevation_max_rad=cfg.elevation_correction_max_rad,
            roll_gain=0.0, roll_max_rad=0.0)
        zeros = lambda *shape: torch.zeros(*shape, device=device)
        flags = lambda: torch.zeros(n, dtype=torch.bool, device=device)
        self._wipe_env, self._engaged, self._missed = flags(), flags(), flags()
        self._damage_now, self._term_damage, self._term_fallen = flags(), flags(), flags()
        self._since_reset = torch.zeros(n, dtype=torch.long, device=device)
        self._engage_step = torch.zeros(n, dtype=torch.long, device=device)
        #: The planned surface in the robot's yaw frame at reset (the path's centre, from the ground under the base),
        #: its inward normal's elevation, the standoff, and the path in the surface's plane.
        self._centre_local, self._elevation, self._standoff = zeros(n, 3), zeros(n), zeros(n)
        self._points = zeros(n, wiping.PATH_SAMPLES, 2)
        self._arc = zeros(n, wiping.PATH_SAMPLES)
        self._kind = torch.zeros(n, dtype=torch.long, device=device)
        self._pre_goal = zeros(n, 3)
        self._approach_des_w = zeros(n, 3)
        self._tip_goal_w = zeros(n, 3)
        #: Which path samples the pad has wiped, and the steps spent wiping and in contact while wiping.
        self._wiped = torch.zeros(n, wiping.PATH_SAMPLES, dtype=torch.bool, device=device)
        self._motion_steps, self._contact_steps = zeros(n), zeros(n)
        self._finished_step = torch.full((n,), -1, dtype=torch.long, device=device)
        self._peak_contact, self._step_peak = zeros(n), zeros(n)
        #: At the layer's start: the pad's miss of its pre-contact pose (m), the approach's elevation and azimuth misses.
        self._engage_miss = torch.full((n, 3), float("nan"), device=device)
        self._force_offset = torch.zeros(n, 1, 3, device=device)
        self._level = float(cfg.curriculum_level)
        self._success_ema: float | None = None
        self._last_promotion_step = 0
        self._evaluation: dict | None = None

        self._gripper_force_kp.fill_(task_cfg.GRIPPER_FORCE_KP)
        self._weights.update(wipe_cfg.scaled_weights())
        self._terms.update(wipe_rewards.TERMS)
        self._weights.update({name: w * interface.POLICY_DT for name, w in task_cfg.APPROACH_WEIGHTS.items()})
        self._terms.update(rewards.APPROACH_TERMS)
        for name in self._weights:
            self._episode_sums.setdefault(name, zeros(n))

    def _setup_scene(self):
        super()._setup_scene()
        self._arm_contact = ContactSensor(self.cfg.arm_contact_sensor)
        self.scene.sensors["arm_contact"] = self._arm_contact

    # --- the pad ----------------------------------------------------------------------------------------------------

    def _tool_vel_w(self) -> torch.Tensor:
        """World velocity of the controlled point (the jaw centre)."""
        quat = self._robot.data.body_quat_w[:, self._tip_body]
        arm = interface.quat_apply(quat, self._tool_offset[:, 0])
        return (self._robot.data.body_lin_vel_w[:, self._tip_body]
                + torch.cross(self._robot.data.body_ang_vel_w[:, self._tip_body], arm, dim=-1))

    def pad_state(self) -> dict[str, torch.Tensor]:
        """The pad's centre (env frame), its velocity, the hand's turning rate and approach (the pad's face)."""
        quat = self._robot.data.body_quat_w[:, self._tip_body]
        link = self._robot.data.body_pos_w[:, self._tip_body] - self.scene.env_origins
        offset = self._tool_offset[:, 0].clone()
        offset[:, 2] += wiping.PAD_LENGTH_M
        arm = interface.quat_apply(quat, offset)
        omega = self._robot.data.body_ang_vel_w[:, self._tip_body]
        vel = self._robot.data.body_lin_vel_w[:, self._tip_body] + torch.cross(omega, arm, dim=-1)
        approach = interface.quat_apply(quat, torch.tensor([0.0, 0.0, 1.0], device=self.device).expand(self.num_envs, 3))
        return {"centre": link + arm, "vel": vel, "omega": omega, "approach": approach, "link": link, "quat": quat}

    def _apply_action(self) -> None:
        super()._apply_action()
        pad = self.pad_state()
        self._ee_force_w = self._surface.step(pad["centre"], pad["approach"], pad["vel"], pad["omega"])
        # The wrench goes where the pressure is: the pad's centre when it is flat, its rim when it is not.
        self._force_offset = interface.quat_rotate_inverse(
            pad["quat"], self._surface.pressure_centre - pad["link"]).unsqueeze(1).contiguous()
        self._damage_now |= (self._surface.normal_n > wipe_cfg.DAMAGE_N) | self._surface.pierced
        self._write_wrench()

    def _write_wrench(self) -> None:
        """As the base class, but at the pad's centre of pressure rather than the controlled point."""
        quat = self._robot.data.body_quat_w[:, self._tip_body]
        local = interface.quat_rotate_inverse(quat, self._ee_force_w)
        self._robot.permanent_wrench_composer.set_forces_and_torques(
            forces=local.unsqueeze(1).contiguous(), positions=self._force_offset, body_ids=self._tip_body_ids)
        self._wrench_active = True

    def _step_forces(self) -> None:
        """No UniFP pushes. The force command is the task layer's, in the base's yaw frame."""
        self._commands[:, interface.CMD_EE_FORCE] = interface.quat_rotate_inverse(
            self._base_yaw_quat(), self._layer.force_cmd)

    # --- the task layer, at the policy rate ----------------------------------------------------------------------

    def _planned_world(self, ids: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """The planned surface's origin (the path's centre, env frame) and outward normal (world) for `ids`."""
        base = self._robot.data.root_pos_w[ids] - self.scene.env_origins[ids]
        yaw = self._base_yaw_quat()[ids]
        ground = base.clone()
        ground[:, 2] = 0.0
        origin = ground + interface.quat_apply(yaw, self._centre_local[ids])
        bearing = torch.atan2(self._centre_local[ids, 1], self._centre_local[ids, 0])
        return origin, -rewards.approach_direction(self._elevation[ids], bearing, yaw)

    def _path_start(self, origin: torch.Tensor, normal: torch.Tensor, ids: torch.Tensor) -> torch.Tensor:
        a, b = wiping.plane_axes(normal)
        start = self._points[ids, 0]
        return origin + start[:, :1] * a + start[:, 1:] * b

    def _jaw_for_pad(self, pad_goal: torch.Tensor, into: torch.Tensor, measured: bool) -> torch.Tensor:
        """Where the jaw centre must be for the pad's centre to be at `pad_goal`: behind it along `into`, or -- with the
        tip loop on and `measured` -- along the hand's measured approach, plus the loop's integrated offset."""
        if self._correction.tip_gain <= 0.0 or not measured:
            return pad_goal - wiping.PAD_LENGTH_M * into
        approach = self._hand_frame_terms()["ee_approach_w"]
        return self._correction.jaw_for_tip(pad_goal, approach, wiping.PAD_LENGTH_M)

    def _engage_due(self) -> None:
        """Start the task layer once the hand has settled with the pad near its pre-contact pose; a hand that has not
        got there by the deadline gets no surface this episode (a free reach), as in the insertion."""
        waited = self._since_reset - self._engage_step
        waiting = self._wipe_env & ~self._engaged & (waited >= 0)
        if not bool(waiting.any()):
            return
        settled = self._tool_vel_w().norm(dim=-1) < wipe_cfg.ENGAGE_SETTLE_SPEED_M_S
        all_ids = torch.arange(self.num_envs, device=self.device)
        origin_all, normal_all = self._planned_world(all_ids)
        planned = self._path_start(origin_all, normal_all, all_ids) + self._standoff.unsqueeze(-1) * normal_all
        miss = (self.pad_state()["centre"] - planned).norm(dim=-1)
        converged = wipe_cfg.ENGAGE_CONVERGED_M if self._correction.tip_gain > 0.0 else wipe_cfg.ENGAGE_NEAR_M
        deadline = waited >= wipe_cfg.ENGAGE_MAX_WAIT_STEPS
        due = waiting & ((settled & (miss < converged)) | (deadline & (miss < wipe_cfg.ENGAGE_NEAR_M)))
        late = waiting & ~due & deadline
        if bool(late.any()):
            self._wipe_env &= ~late
            self._goals.hold_mask &= ~late
            self._missed |= late
        if not bool(due.any()):
            return
        ids = due.nonzero(as_tuple=False).flatten()
        origin, normal = origin_all[ids], normal_all[ids]
        draw = self._draw_evaluation(ids) if self._evaluation is not None else self._draw(ids)
        # The true surface: the estimate moved along its normal (positive: further from the robot) and tilted.
        a, b = wiping.plane_axes(normal)
        into = -normal
        true_into = insertion.unit(into + draw["tilt"][:, :1] * a + draw["tilt"][:, 1:] * b)
        self._surface.place(ids, origin=origin + draw["offset"].unsqueeze(-1) * into, normal=-true_into,
                            half_a=torch.full((len(ids),), wipe_cfg.PANEL_HALF_M, device=self.device),
                            half_b=torch.full((len(ids),), wipe_cfg.PANEL_HALF_M, device=self.device), mu=draw["mu"])
        count = len(ids)
        full_of = lambda v: torch.full((count,), float(v), device=self.device)
        self._layer.start(ids, origin=origin, normal=normal, points=self._points[ids], arc=self._arc[ids],
                          standoff=self._standoff[ids], overdrive=full_of(wipe_cfg.OVERDRIVE_M), force_n=draw["force"],
                          speed=draw["speed"], approach_speed=full_of(wipe_cfg.APPROACH_SPEED_M_S))
        self._approach_des_w[ids] = into
        self._engaged[ids] = True
        self._goals.hold_mask[ids] = True
        self._goals.hold_roll[ids] = 0.0
        self._engage_miss[ids] = torch.cat((miss[ids].unsqueeze(-1), self._attitude_miss()[ids]), dim=-1)

    def _draw(self, ids: torch.Tensor) -> dict[str, torch.Tensor]:
        count, device, level = len(ids), self.device, self._level
        rand = lambda *shape: torch.rand(*shape, device=device)
        span = lambda r: r[0] + rand(count) * (r[1] - r[0])
        angle = 2 * math.pi * rand(count, 1)
        tilt = level * wipe_cfg.TILT_ERROR_RAD * torch.sqrt(rand(count, 1)) * torch.cat(
            [torch.cos(angle), torch.sin(angle)], dim=-1)
        return {"offset": (2 * rand(count) - 1) * level * wipe_cfg.OFFSET_ERROR_M, "tilt": tilt,
                "mu": span(wipe_cfg.FRICTION_MU), "force": span(wipe_cfg.FORCE_N), "speed": span(wipe_cfg.SPEED_M_S)}

    def _draw_evaluation(self, ids: torch.Tensor) -> dict[str, torch.Tensor]:
        plan, c = self._evaluation, self._evaluation["conditions"]
        full = lambda v: torch.full((len(ids),), float(v), device=self.device)
        return {"offset": plan["offset"][ids], "tilt": plan["tilt"][ids], "force": plan["force"][ids],
                "mu": full(c["mu"]), "speed": full(c["speed_m_s"])}

    def _run_layer(self) -> None:
        """Advance the task layer and hold each engaged environment's goal where it says, in the base's sphere now."""
        engaged = self._engaged
        if not bool(engaged.any()):
            return
        centre = self.pad_state()["centre"]
        goal_pad, _ = self._layer.step(centre, interface.POLICY_DT)
        # Across the normal the pad is held on the layer's goal until it touches; while wiping the in-plane law
        # carries its miss, and two integrators on one error would fight.
        free = engaged & (self._layer.phase == wiping.APPROACHING)
        self._correction.update_tip(free, goal_pad, centre, interface.POLICY_DT, across=self._layer.n_in)
        self._tip_goal_w = torch.where(engaged.unsqueeze(-1), goal_pad, self._tip_goal_w)
        goal_jaw = self._jaw_for_pad(goal_pad, self._layer.n_in, measured=True)
        local = interface.quat_rotate_inverse(self._base_yaw_quat(), goal_jaw - self._goal_centre())
        self._goals.hold_goal = torch.where(engaged.unsqueeze(-1), interface.cart2sphere(local), self._goals.hold_goal)

    def _hold_pre_contact(self) -> None:
        """Before the layer starts: hold the jaw goal where the pad lands on its pre-contact pose, the tip loop
        integrating its miss once the goal has arrived there."""
        waiting = self._wipe_env & ~self._engaged
        if not bool(waiting.any()):
            return
        all_ids = torch.arange(self.num_envs, device=self.device)
        origin, normal = self._planned_world(all_ids)
        pad_goal = self._path_start(origin, normal, all_ids) + self._standoff.unsqueeze(-1) * normal
        holding = waiting & self._goals.hold_mask
        self._correction.update_tip(holding, pad_goal, self.pad_state()["centre"], interface.POLICY_DT)
        self._tip_goal_w = torch.where(waiting.unsqueeze(-1), pad_goal, self._tip_goal_w)
        jaw = torch.where(holding.unsqueeze(-1), self._jaw_for_pad(pad_goal, -normal, measured=True),
                          self._jaw_for_pad(pad_goal, -normal, measured=False))
        sphere = interface.cart2sphere(interface.quat_rotate_inverse(self._base_yaw_quat(), jaw - self._goal_centre()))
        self._pre_goal = torch.where(waiting.unsqueeze(-1), sphere, self._pre_goal)
        self._goals.hold_goal = torch.where(holding.unsqueeze(-1), sphere, self._goals.hold_goal)

    def _attitude_miss(self) -> torch.Tensor:
        """(elevation, azimuth) of the hand's approach minus the surface's inward normal, rad: (N, 2)."""
        approach, want = self._hand_frame_terms()["ee_approach_w"], self._approach_des_w
        elevation = torch.asin(approach[:, 2].clamp(-1.0, 1.0)) - torch.asin(want[:, 2].clamp(-1.0, 1.0))
        azimuth = insertion.wrap_half_turn(torch.atan2(approach[:, 1], approach[:, 0])
                                           - torch.atan2(want[:, 1], want[:, 0]))
        return torch.stack((elevation, azimuth), dim=-1)

    def _correct_attitude(self) -> None:
        """The elevation loop: from the moment the goal holds the pre-contact pose until the pad touches."""
        mask = self._wipe_env & self._goals.hold_mask & ~self._layer.touched
        if not bool(mask.any()):
            return
        miss = self._attitude_miss()
        zero = torch.zeros_like(miss[:, 0])
        self._correction.update_attitude(mask, zero, miss[:, 0], zero, zero, interface.POLICY_DT)

    def _get_rewards(self) -> torch.Tensor:
        self._since_reset += 1
        arrived = self._wipe_env & ~self._goals.hold_mask & (self._since_reset >= self._goals.traj_steps.long())
        if bool(arrived.any()):
            self._goals.hold_goal[arrived] = self._pre_goal[arrived]
            self._goals.hold_roll[arrived] = 0.0
            self._goals.hold_mask[arrived] = True
        self._hold_pre_contact()
        self._engage_due()
        self._run_layer()
        waiting = self._wipe_env & ~self._engaged
        if bool(waiting.any()):
            ids = waiting.nonzero(as_tuple=False).flatten()
            _, normal = self._planned_world(ids)
            self._approach_des_w[ids] = -normal
        self._correct_attitude()
        self._commands[:, interface.CMD_EE_ORN_Y] = rewards.encode_approach(
            self._elevation + self._correction.elevation, self._wipe_env)
        wiping_now = self._engaged & (self._layer.phase == wiping.WIPING)
        contact = self._surface.in_contact & wiping_now
        self._wiped |= wiping.wiped_samples(self._layer.points, self._layer.in_plane(self._surface.pressure_centre),
                                            contact)
        self._motion_steps += wiping_now.float()
        self._contact_steps += contact.float()
        finished = self._engaged & (self._layer.phase >= wiping.LIFTING) & (self._finished_step < 0)
        self._finished_step = torch.where(finished, self._since_reset, self._finished_step)
        self._step_peak = self._surface.take_peak() * self._engaged
        self._peak_contact = torch.maximum(self._peak_contact, self._step_peak)
        return super()._get_rewards()

    def coverage(self) -> torch.Tensor:
        """The fraction of each environment's path wiped so far."""
        return self._wiped.float().mean(dim=-1)

    def _task_state(self):
        state = super()._task_state()
        layer, surface = self._layer, self._surface
        engaged = (self._engaged & surface.active & (layer.phase <= wiping.WIPING)).float()
        pad = self.pad_state()
        state.fixture_engaged = engaged
        # Along the normal only: the friction the pad drags is the in-plane law's business, not the force it presses.
        state.fixture_applied_w = surface.normal_n.unsqueeze(-1) * layer.n_in
        state.fixture_command_w = layer.push.unsqueeze(-1) * layer.n_in
        state.fixture_axis_w = layer.n_in
        state.tool_vel_w = pad["vel"]
        state.approach_des_w = self._approach_des_w
        state.approach_active = self._wipe_env.float()
        wiping_now = self._engaged & (layer.phase == wiping.WIPING)
        state.wipe_wiping = wiping_now.float()
        state.wipe_contact = (surface.in_contact & wiping_now).float()
        state.wipe_normal_n = surface.normal_n
        miss = self._tip_pos() - self._goal_world()
        across = miss - (miss * layer.n_in).sum(-1, keepdim=True) * layer.n_in
        state.wipe_track_error = torch.where(self._engaged, across.norm(dim=-1), miss.norm(dim=-1))
        v = pad["vel"]
        v_across = v - (v * layer.n_in).sum(-1, keepdim=True) * layer.n_in
        approaching = (self._engaged & (layer.phase == wiping.APPROACHING)).unsqueeze(-1)
        state.wipe_vel_error_w = torch.where(wiping_now.unsqueeze(-1), v - layer.ref_vel,
                                             torch.where(approaching, v_across, v))
        state.wipe_active = self._wipe_env.float()
        state.wipe_holding = (self._wipe_env & self._goals.hold_mask & (layer.phase <= wiping.WIPING)).float()
        state.mech_torn = self._damage_now.float()
        return state

    def _privileged_extra(self) -> torch.Tensor:
        """The surface as the critic sees it, in the `mass_params` block. Zero where there is none."""
        s, layer = self._surface, self._layer
        held = (self._engaged & s.active).float().unsqueeze(-1)
        yaw = self._base_yaw_quat()
        total = layer.total.clamp(min=1e-3)
        out = torch.zeros(self.num_envs, wipe_cfg.PRIVILEGED_WIDTH, device=self.device)
        out[:, 0:1] = held
        out[:, 1:2] = layer.touched.float().unsqueeze(-1) * held
        out[:, 2:3] = s.in_contact.float().unsqueeze(-1) * held
        out[:, 3:4] = s.depth.clamp(-0.05, 0.05).unsqueeze(-1) * 20.0 * held
        out[:, 4:5] = (layer.ref - layer.s).unsqueeze(-1) * 50.0 * held
        out[:, 5:6] = ((s.origin - layer.origin) * layer.n_in).sum(-1, keepdim=True) * 50.0 * held
        out[:, 6:9] = interface.quat_rotate_inverse(yaw, -s.normal - layer.n_in) * 10.0 * held
        out[:, 9:12] = interface.quat_rotate_inverse(yaw, s.force_on_pad) * 0.02 * held
        out[:, 12:13] = s.mu.unsqueeze(-1) * held
        out[:, 13:14] = (layer.progress / total).unsqueeze(-1) * held
        out[:, 14:15] = (layer.path_ref - layer.progress).unsqueeze(-1) * 20.0 * held
        out[:, 15:16] = layer.force_n.unsqueeze(-1) * 0.04 * held
        out[:, 16:17] = layer.phase.float().unsqueeze(-1) / 3.0 * held
        out[:, 17:18] = self.coverage().unsqueeze(-1) * held
        out[:, 18:19] = layer.speed.unsqueeze(-1) * 20.0 * held
        out[:, 19:20] = self._wipe_env.float().unsqueeze(-1)
        out[:, 20] = self._level
        out[:, 21:22] = self._elevation.unsqueeze(-1) * self._wipe_env.float().unsqueeze(-1)
        return out

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        fallen, timed_out = super()._get_dones()
        damaged = self._damage_now.clone()
        self._term_damage, self._term_fallen = damaged, fallen.clone()
        self._damage_now[:] = False
        return fallen | damaged, timed_out

    def _resample_velocity_commands(self, env_ids: torch.Tensor) -> None:
        """UniFP's draw, except that an episode with a surface stands."""
        super()._resample_velocity_commands(env_ids)
        standing = env_ids[self._wipe_env[env_ids]]
        self._commands[standing, :3] = 0.0

    # --- evaluation -----------------------------------------------------------------------------------------------

    def set_evaluation(self, plan: dict) -> None:
        """Fix every episode's surface and path (`wipe_eval.build_plan`). Call before `reset()`. The curriculum is off."""
        self._evaluation = {k: (v.to(self.device) if torch.is_tensor(v) else v) for k, v in plan.items()}
        self.cfg.curriculum = False

    # --- resetting --------------------------------------------------------------------------------------------------

    def _update_curriculum(self, env_ids: torch.Tensor) -> None:
        log = self.extras.setdefault("log", {})
        planned = self._engaged[env_ids] | self._missed[env_ids]
        if bool(planned.any()):
            log["Wipe/missed_fraction"] = float(self._missed[env_ids][planned].float().mean())
        done = env_ids[self._engaged[env_ids]]
        if len(done) > 0:
            coverage = self.coverage()[done]
            contact = self._contact_steps[done] / self._motion_steps[done].clamp(min=1.0)
            success = ((coverage >= wipe_cfg.SUCCESS_COMPLETION) & (contact >= wipe_cfg.SUCCESS_CONTACT)
                       & (self._motion_steps[done] > 0) & ~self._term_damage[done]).float()
            log["Wipe/success_fraction"] = float(success.mean())
            log["Wipe/completion"] = float(coverage.mean())
            log["Wipe/contact_during_motion"] = float(contact.mean())
            log["Wipe/touched_fraction"] = float(self._layer.touched[done].float().mean())
            log["Wipe/finished_fraction"] = float((self._finished_step[done] >= 0).float().mean())
            log["Wipe/peak_contact_n"] = float(self._peak_contact[done].mean())
            log["Wipe/damaged_fraction"] = float(self._term_damage[done].float().mean())
            log["Wipe/engage_tip_miss_mm"] = 1000.0 * float(self._engage_miss[done, 0].median())
            log["Wipe/engage_elevation_miss_deg"] = math.degrees(float(self._engage_miss[done, 1].abs().median()))
            rate = float(success.mean())
            self._success_ema = rate if self._success_ema is None else (
                (1 - wipe_cfg.CURRICULUM_EMA) * self._success_ema + wipe_cfg.CURRICULUM_EMA * rate)
        wait = self.cfg.curriculum_min_iterations * task_cfg.NUM_STEPS_PER_ENV
        if (self.cfg.curriculum and self._success_ema is not None and self._success_ema > wipe_cfg.PROMOTE_SUCCESS
                and self._level < 1.0 and self.common_step_counter - self._last_promotion_step >= wait):
            self._level = min(1.0, self._level + wipe_cfg.CURRICULUM_STEP)
            self._last_promotion_step = self.common_step_counter
            self._success_ema = None
        log["Wipe/curriculum_level"] = self._level
        if self._success_ema is not None:
            log["Wipe/success_ema"] = self._success_ema

    def _sample_paths(self, count: int) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """(points, arc, kind) for `count` new paths in their surface's plane."""
        device = self.device
        rand = lambda: torch.rand(count, device=device)
        span = lambda r: r[0] + rand() * (r[1] - r[0])
        sign = torch.where(rand() < 0.5, -1.0, 1.0)
        line = wiping.resample(wiping.line_path(sign * span(wipe_cfg.LINE_LENGTH_M), span(wipe_cfg.LINE_ANGLE_RAD)))
        zigzag = wiping.resample(wiping.zigzag_path(sign * span(wipe_cfg.ZIGZAG_LENGTH_M),
                                                    span(wipe_cfg.ZIGZAG_SPACING_M), wipe_cfg.ZIGZAG_STROKES))
        is_line = rand() < wipe_cfg.LINE_PROB
        points = torch.where(is_line[:, None, None], line[0], zigzag[0])
        arc = torch.where(is_line[:, None], line[1], zigzag[1])
        return points, arc, (~is_line).long()

    def _sample_surfaces(self, count: int) -> dict[str, torch.Tensor]:
        """New surfaces and paths, every point of the path reachable with the pad on it (the jaw inside the trained
        goal sphere at the path's extremes)."""
        device = self.device
        span = lambda r, n: r[0] + torch.rand(n, device=device) * (r[1] - r[0])
        out = {"centre": torch.zeros(count, 3, device=device), "elevation": torch.zeros(count, device=device),
               "standoff": torch.zeros(count, device=device),
               "points": torch.zeros(count, wiping.PATH_SAMPLES, 2, device=device),
               "arc": torch.zeros(count, wiping.PATH_SAMPLES, device=device),
               "kind": torch.zeros(count, dtype=torch.long, device=device)}
        pending = torch.arange(count, device=device)
        for _ in range(20):
            n = len(pending)
            centre = torch.stack((span(wipe_cfg.CENTRE_AHEAD_M, n), span(wipe_cfg.CENTRE_SIDE_M, n),
                                  span(wipe_cfg.CENTRE_HEIGHT_M, n)), dim=-1)
            elevation, standoff = span(wipe_cfg.NORMAL_ELEVATION_RAD, n), span(wipe_cfg.STANDOFF_M, n)
            points, arc, kind = self._sample_paths(n)
            for key, value in (("centre", centre), ("elevation", elevation), ("standoff", standoff),
                               ("points", points), ("arc", arc), ("kind", kind)):
                out[key][pending] = value
            ok = self._reachable(centre, elevation, standoff, points)
            pending = pending[~ok]
            if len(pending) == 0:
                break
        if len(pending) > 0:
            out["centre"][pending] = torch.tensor((0.52, 0.0, 0.78), device=device)
            out["elevation"][pending], out["standoff"][pending] = 0.0, 0.015
            points, arc = wiping.resample(wiping.line_path(torch.full((len(pending),), 0.10, device=device),
                                                           torch.zeros(len(pending), device=device)))
            out["points"][pending], out["arc"][pending], out["kind"][pending] = points, arc, 0
        return out

    @staticmethod
    def _surface_frame(centre: torch.Tensor, elevation: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """(inward normal, a, b) of a surface through `centre` (yaw frame) facing the robot, leaning by `elevation`."""
        bearing = torch.atan2(centre[:, 1], centre[:, 0])
        into = torch.stack((torch.cos(elevation) * torch.cos(bearing), torch.cos(elevation) * torch.sin(bearing),
                            torch.sin(elevation)), dim=-1)
        a, b = wiping.plane_axes(-into)
        return into, a, b

    def _pre_contact_sphere(self, centre: torch.Tensor, elevation: torch.Tensor, standoff: torch.Tensor,
                            start: torch.Tensor) -> torch.Tensor:
        """The jaw centre's goal, as goal-sphere coordinates, for the pad `standoff` off the surface over `start`."""
        into, a, b = self._surface_frame(centre, elevation)
        pad = centre + start[:, :1] * a + start[:, 1:] * b - standoff.unsqueeze(-1) * into
        jaw = pad - wiping.PAD_LENGTH_M * into
        return interface.cart2sphere(jaw - torch.tensor(interface.EE_GOAL_CENTER_OFFSET, device=centre.device))

    def _reachable(self, centre: torch.Tensor, elevation: torch.Tensor, standoff: torch.Tensor,
                   points: torch.Tensor) -> torch.Tensor:
        """Every extreme of the path, with the pad on the surface there, puts the jaw inside the goal sphere."""
        ok = torch.ones(len(centre), dtype=torch.bool, device=centre.device)
        lo, hi = points.min(dim=1).values, points.max(dim=1).values
        for corner in (lo, hi, torch.stack((lo[:, 0], hi[:, 1]), dim=-1), torch.stack((hi[:, 0], lo[:, 1]), dim=-1)):
            sphere = self._pre_contact_sphere(centre, elevation, standoff, corner)
            ok &= ((sphere[:, 0] >= interface.EE_GOAL_RADIUS_RANGE[0] + 0.01)
                   & (sphere[:, 0] <= interface.EE_GOAL_RADIUS_RANGE[1] - 0.01)
                   & (sphere[:, 1] <= interface.EE_GOAL_PITCH_RANGE[1]))
        return ok

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        self._update_curriculum(env_ids)
        count = len(env_ids)
        if self._evaluation is not None:
            self._wipe_env[env_ids] = True
        else:
            self._wipe_env[env_ids] = torch.rand(count, device=self.device) < self.cfg.wipe_fraction
        super()._reset_idx(env_ids)
        self._surface.reset(env_ids)
        self._layer.reset(env_ids)
        self._correction.reset(env_ids)
        self._engage_miss[env_ids] = float("nan")
        self._tip_goal_w[env_ids] = 0.0
        for buf in (self._engaged, self._damage_now, self._missed):
            buf[env_ids] = False
        self._wiped[env_ids] = False
        self._since_reset[env_ids] = 0
        self._finished_step[env_ids] = -1
        for buf in (self._motion_steps, self._contact_steps, self._peak_contact, self._step_peak, self._elevation):
            buf[env_ids] = 0.0
        self._ee_force_w[env_ids] = 0.0
        self._goals.hold_mask[env_ids] = False
        ids = env_ids[self._wipe_env[env_ids]]
        if len(ids) > 0:
            if self._evaluation is not None:
                plan = self._evaluation
                draw = {"centre": plan["centre"][ids], "elevation": plan["elevation"][ids],
                        "standoff": torch.full((len(ids),), plan["conditions"]["standoff_m"], device=self.device),
                        "points": plan["points"][ids], "arc": plan["arc"][ids], "kind": plan["kind"][ids]}
                delay = torch.full((len(ids),), 15, device=self.device)
            else:
                draw = self._sample_surfaces(len(ids))
                low, high = wipe_cfg.ENGAGE_DELAY_STEPS
                delay = torch.randint(low, high + 1, (len(ids),), device=self.device)
            self._centre_local[ids], self._elevation[ids], self._standoff[ids] = (
                draw["centre"], draw["elevation"], draw["standoff"])
            self._points[ids], self._arc[ids], self._kind[ids] = draw["points"], draw["arc"], draw["kind"]
            self._pre_goal[ids] = self._pre_contact_sphere(draw["centre"], draw["elevation"], draw["standoff"],
                                                           draw["points"][:, 0])
            self._goals.aim(ids, self._pre_goal[ids], torch.zeros(len(ids), device=self.device))
            self._engage_step[ids] = self._goals.traj_steps[ids].long() + delay.long()
