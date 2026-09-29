"""The pressing task: the robot presses a spring-loaded button it knows only from an estimate and a datasheet.

What an episode is, for the `button_fraction` of environments that get one:

  1. A button is drawn in the robot's yaw frame (`press_cfg.FACE_*`), on a panel facing it, and a tool: the bare jaws or
     a probe (`pressing.TOOLS`). The goal slides to the pre-press pose -- the tool's tip `standoff` short of the cap, the
     hand's approach along the button's axis (`rewards.tracking_ee_orn_approach`), the jaws level -- and the approach is
     commanded from the start.
  2. Once the hand has settled there with the tip converged, the task layer (`pressing.PressTaskLayer`) starts on that
     *estimate*, and the true button (`pressing.Button`) is placed off it by the curriculum's errors: across the axis,
     along it, in tilt; its datasheet force wrong by a drawn factor. The policy observes none of it. Throughout, the
     insertion's outer loops (`insertion.PoseCorrection`) integrate the tip's measured miss and the approach's elevation
     miss into the commands (F-118, F-120).
  3. The layer presses: the goal across the axis on the estimate, a PI force command along it on how far the tip lags a
     reference depth, light until something is touched; a steady hold at full travel for `press_cfg.HOLD_S`; a release
     back to the standoff. The contact force acts at the tool's tip. The velocity command is zero.
  4. Success is the cap held at full travel for `press_cfg.SUCCESS_HOLD_S` (the plan's criterion); a contact force past
     `press_cfg.DAMAGE_N`, the tool through the panel, or a fall ends the episode.

The rest reach through free space with no pushes and no approach command. The policy's widths do not change.
"""
from __future__ import annotations

import math

import torch

from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.utils import configclass

from unifp_isaaclab import interface

from . import insertion, press_cfg, press_rewards, pressing, rewards, task_cfg
from .env import Go2D1PosForceEnv
from .env_cfg import Go2D1PosForceEnvCfg
from .hook_env import HeldGoals


@configclass
class Go2D1PressEnvCfg(Go2D1PosForceEnvCfg):
    """UniFP's robot and observation, holding a pressing tool; the roll and approach objectives on."""

    roll_objective: bool = True
    force_start_step: int = 0
    button_fraction: float = press_cfg.BUTTON_FRACTION
    #: The tools drawn, equally often (`pressing.TOOLS`).
    tools: tuple[str, ...] = press_cfg.TOOLS
    curriculum: bool = True
    curriculum_level: float = press_cfg.CURRICULUM_START
    curriculum_min_iterations: int = press_cfg.CURRICULUM_MIN_ITERATIONS
    tip_correction_gain: float = press_cfg.TIP_CORRECTION_GAIN
    tip_correction_max_m: float = press_cfg.TIP_CORRECTION_MAX_M
    elevation_correction_gain: float = press_cfg.ELEVATION_CORRECTION_GAIN
    elevation_correction_max_rad: float = press_cfg.ELEVATION_CORRECTION_MAX_RAD
    arm_contact_sensor: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/D1/.*", history_length=0, update_period=0.0, track_air_time=False)


class Go2D1PressEnv(Go2D1PosForceEnv):
    cfg: Go2D1PressEnvCfg

    def __init__(self, cfg: Go2D1PressEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, device = self.num_envs, self.device
        self._goals = HeldGoals.adopt(self._goals)
        self._button = pressing.Button(n, device)
        self._layer = pressing.PressTaskLayer(n, device, kp=press_cfg.FORCE_LAW_KP_N_PER_M,
                                              ki=press_cfg.FORCE_LAW_KI_N_PER_M_S)
        self._correction = insertion.PoseCorrection(
            n, device, tip_gain=cfg.tip_correction_gain, tip_max_m=cfg.tip_correction_max_m,
            elevation_gain=cfg.elevation_correction_gain, elevation_max_rad=cfg.elevation_correction_max_rad,
            roll_gain=0.0, roll_max_rad=0.0)
        zeros = lambda *shape: torch.zeros(*shape, device=device)
        flags = lambda: torch.zeros(n, dtype=torch.bool, device=device)
        self._press_env, self._engaged, self._missed = flags(), flags(), flags()
        self._damage_now, self._term_damage, self._term_fallen = flags(), flags(), flags()
        self._since_reset = torch.zeros(n, dtype=torch.long, device=device)
        self._engage_step = torch.zeros(n, dtype=torch.long, device=device)
        #: The tool: its index into `pressing.TOOL_NAMES`, its length beyond the jaw centre and its tip's radius.
        self._tool_index = torch.zeros(n, dtype=torch.long, device=device)
        self._tool_length, self._tool_radius = zeros(n), zeros(n)
        #: The planned button in the robot's yaw frame at reset (from the ground under the base), its axis's elevation.
        self._face_local, self._elevation, self._standoff = zeros(n, 3), zeros(n), zeros(n)
        self._pre_goal = zeros(n, 3)
        self._approach_des_w = zeros(n, 3)
        self._tip_goal_w = zeros(n, 3)
        #: The cap pressed without a break (s), the longest such spell, the step it was first at full travel.
        self._pressed_for, self._longest_pressed = zeros(n), zeros(n)
        self._pressed_step = torch.full((n,), -1, dtype=torch.long, device=device)
        self._peak_contact, self._peak_lateral = zeros(n), zeros(n)
        #: The largest contact force within the last policy step, sampled at the physics rate (the plan's peak force).
        self._step_peak = zeros(n)
        #: At the layer's start: the tip's miss of its pre-press pose (m), the approach's elevation and azimuth misses.
        self._engage_miss = torch.full((n, 3), float("nan"), device=device)
        #: The wrench's point on Link6, per environment: the jaw centre plus the tool's length along the approach.
        self._force_offset = torch.zeros(n, 1, 3, device=device)
        self._level = float(cfg.curriculum_level)
        self._success_ema: float | None = None
        self._last_promotion_step = 0
        self._evaluation: dict | None = None

        self._gripper_force_kp.fill_(task_cfg.GRIPPER_FORCE_KP)
        self._weights.update(press_cfg.scaled_weights())
        self._terms.update(press_rewards.TERMS)
        self._weights.update({name: w * interface.POLICY_DT for name, w in task_cfg.APPROACH_WEIGHTS.items()})
        self._terms.update(rewards.APPROACH_TERMS)
        for name in self._weights:
            self._episode_sums.setdefault(name, zeros(n))

    def _setup_scene(self):
        super()._setup_scene()
        self._arm_contact = ContactSensor(self.cfg.arm_contact_sensor)
        self.scene.sensors["arm_contact"] = self._arm_contact

    # --- the tool ---------------------------------------------------------------------------------------------------

    def _tool_vel_w(self) -> torch.Tensor:
        """World velocity of the controlled point (the jaw centre)."""
        quat = self._robot.data.body_quat_w[:, self._tip_body]
        arm = interface.quat_apply(quat, self._tool_offset[:, 0])
        return (self._robot.data.body_lin_vel_w[:, self._tip_body]
                + torch.cross(self._robot.data.body_ang_vel_w[:, self._tip_body], arm, dim=-1))

    def tool_state(self) -> dict[str, torch.Tensor]:
        """The tool's tip (env frame), its velocity, the hand's approach, and the tip sphere's centre."""
        quat = self._robot.data.body_quat_w[:, self._tip_body]
        pos = self._robot.data.body_pos_w[:, self._tip_body] - self.scene.env_origins
        offset = self._tool_offset[:, 0].clone()
        offset[:, 2] += self._tool_length
        arm = interface.quat_apply(quat, offset)
        vel = (self._robot.data.body_lin_vel_w[:, self._tip_body]
               + torch.cross(self._robot.data.body_ang_vel_w[:, self._tip_body], arm, dim=-1))
        approach = interface.quat_apply(quat, torch.tensor([0.0, 0.0, 1.0], device=self.device).expand(self.num_envs, 3))
        tip = pos + arm
        return {"tip": tip, "vel": vel, "approach": approach,
                "centre": tip - self._tool_radius.unsqueeze(-1) * approach, "offset_local": offset}

    def _apply_action(self) -> None:
        super()._apply_action()
        tool = self.tool_state()
        self._force_offset = tool["offset_local"].unsqueeze(1).contiguous()
        self._ee_force_w = self._button.step(tool["centre"], tool["vel"], self._tool_radius, interface.SIM_DT)
        self._damage_now |= (self._button.normal_n > press_cfg.DAMAGE_N) | self._button.pierced
        self._write_wrench()

    def _write_wrench(self) -> None:
        """As the base class, but at the tool's tip rather than the controlled point."""
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
        """The planned cap face (env frame) and axis (world) for `ids`, from the base's pose now."""
        base = self._robot.data.root_pos_w[ids] - self.scene.env_origins[ids]
        yaw = self._base_yaw_quat()[ids]
        ground = base.clone()
        ground[:, 2] = 0.0
        face = ground + interface.quat_apply(yaw, self._face_local[ids])
        bearing = torch.atan2(self._face_local[ids, 1], self._face_local[ids, 0])
        return face, rewards.approach_direction(self._elevation[ids], bearing, yaw)

    def _jaw_for_tip(self, tip_goal: torch.Tensor, axis: torch.Tensor, measured: bool) -> torch.Tensor:
        """Where the jaw centre must be for the tool's tip to be at `tip_goal`: behind it along `axis`, or -- with the
        tip loop on and `measured` -- along the hand's measured approach, plus the loop's integrated offset."""
        length = self._tool_length.unsqueeze(-1)
        if self._correction.tip_gain <= 0.0 or not measured:
            return tip_goal - length * axis
        approach = self._hand_frame_terms()["ee_approach_w"]
        return tip_goal - length * approach + self._correction.offset

    def _engage_due(self) -> None:
        """Start the task layer once the hand has settled with the tip near its pre-press pose; a hand that has not got
        there by the deadline gets no button this episode (a free reach), as in the insertion."""
        waited = self._since_reset - self._engage_step
        waiting = self._press_env & ~self._engaged & (waited >= 0)
        if not bool(waiting.any()):
            return
        settled = self._tool_vel_w().norm(dim=-1) < press_cfg.ENGAGE_SETTLE_SPEED_M_S
        all_ids = torch.arange(self.num_envs, device=self.device)
        face_all, axis_all = self._planned_world(all_ids)
        planned_tip = face_all - self._standoff.unsqueeze(-1) * axis_all
        miss = (self.tool_state()["tip"] - planned_tip).norm(dim=-1)
        converged = press_cfg.ENGAGE_CONVERGED_M if self._correction.tip_gain > 0.0 else press_cfg.ENGAGE_NEAR_M
        deadline = waited >= press_cfg.ENGAGE_MAX_WAIT_STEPS
        due = waiting & ((settled & (miss < converged)) | (deadline & (miss < press_cfg.ENGAGE_NEAR_M)))
        late = waiting & ~due & deadline
        if bool(late.any()):
            self._press_env &= ~late
            self._goals.hold_mask &= ~late
            self._missed |= late
        if not bool(due.any()):
            return
        ids = due.nonzero(as_tuple=False).flatten()
        face, axis = face_all[ids], axis_all[ids]
        draw = self._draw_evaluation(ids) if self._evaluation is not None else self._draw(ids)
        # The true button: the estimate moved across its axis and along it, and tilted.
        u, w = insertion.roll_frame(axis, torch.zeros(len(ids), device=self.device))
        lateral = draw["lateral"][:, :1] * u + draw["lateral"][:, 1:] * w
        tilt = draw["tilt"]
        true_axis = insertion.unit(axis + tilt[:, :1] * u + tilt[:, 1:] * w)
        true_face = face + lateral + draw["depth_error"].unsqueeze(-1) * axis
        self._button.place(ids, face=true_face, axis=true_axis, cap_radius=draw["cap_radius"], proud=draw["proud"],
                           travel=draw["travel"], preload_n=draw["preload"], stiffness=draw["stiffness"],
                           damping=draw["damping"], snap_n=draw["snap_n"], snap_at=draw["snap_at"], mu=draw["mu"])
        # The datasheet, off by the rating error: what the layer sizes its forces from.
        rating = draw["rating"]
        operating = self._button.operating_n()[ids] * rating
        full = self._button.full_travel_n()[ids] * rating
        max_n = (draw["max_margin"] * operating).clamp(max=press_cfg.MAX_N_LIMIT)
        hold_n = torch.minimum(draw["hold_margin"] * full, max_n)
        count = len(ids)
        full_of = lambda v: torch.full((count,), float(v), device=self.device)
        self._layer.start(ids, face=face, axis=axis, standoff=self._standoff[ids], travel=draw["travel"],
                          overdrive=full_of(press_cfg.OVERDRIVE_M), speed=draw["speed"],
                          touch_n=full_of(press_cfg.TOUCH_N), max_n=max_n, hold_n=hold_n,
                          hold_s=full_of(press_cfg.HOLD_S))
        self._approach_des_w[ids] = axis
        self._engaged[ids] = True
        self._goals.hold_mask[ids] = True
        self._goals.hold_roll[ids] = 0.0
        self._engage_miss[ids] = torch.cat((miss[ids].unsqueeze(-1), self._attitude_miss()[ids]), dim=-1)

    def _draw(self, ids: torch.Tensor) -> dict[str, torch.Tensor]:
        count, device, level = len(ids), self.device, self._level
        rand = lambda *shape: torch.rand(*shape, device=device)
        span = lambda r: r[0] + rand(count) * (r[1] - r[0])

        def disc(radius: float) -> torch.Tensor:
            angle = 2 * math.pi * rand(count, 1)
            return radius * torch.sqrt(rand(count, 1)) * torch.cat([torch.cos(angle), torch.sin(angle)], dim=-1)

        travel, cap_radius, proud = span(press_cfg.TRAVEL_M), span(press_cfg.CAP_RADIUS_M), span(press_cfg.PROUD_M)
        # Pressable with this episode's tool: raise the cap until the tip can reach its full travel past the rim.
        reach = pressing.reachable_front(self._tool_radius[ids], cap_radius, proud)
        short = (travel + 0.001 - reach).clamp(min=0.0)
        proud = proud + torch.where(torch.isfinite(short), short, torch.zeros_like(short))
        return {"lateral": disc(level * press_cfg.LATERAL_ERROR_M),
                "depth_error": (2 * rand(count) - 1) * level * press_cfg.DEPTH_ERROR_M,
                "tilt": disc(level * press_cfg.TILT_ERROR_RAD),
                "rating": 1.0 + (2 * rand(count) - 1) * level * press_cfg.RATING_ERROR,
                "travel": travel, "cap_radius": cap_radius, "proud": proud,
                "preload": span(press_cfg.PRELOAD_N), "stiffness": span(press_cfg.STIFFNESS_N_PER_M),
                "damping": span(press_cfg.DAMPING_N_S_PER_M), "snap_n": rand(count) * level * press_cfg.SNAP_N,
                "snap_at": span(press_cfg.SNAP_AT), "mu": span(press_cfg.FRICTION_MU),
                "speed": span(press_cfg.REFERENCE_SPEED_M_S), "max_margin": span(press_cfg.MAX_MARGIN),
                "hold_margin": span(press_cfg.HOLD_MARGIN)}

    def _draw_evaluation(self, ids: torch.Tensor) -> dict[str, torch.Tensor]:
        plan, c = self._evaluation, self._evaluation["conditions"]
        full = lambda v: torch.full((len(ids),), float(v), device=self.device)
        out = {key: plan[key][ids] for key in ("lateral", "depth_error", "tilt", "rating", "travel", "cap_radius",
                                                  "proud", "preload", "stiffness", "damping", "snap_n", "snap_at", "mu")}
        out.update({"speed": full(c["speed_m_s"]), "max_margin": full(c["max_margin"]),
                    "hold_margin": full(c["hold_margin"])})
        return out

    def _run_layer(self) -> None:
        """Advance the task layer and hold each engaged environment's goal where it says, in the base's sphere now."""
        engaged = self._engaged
        if not bool(engaged.any()):
            return
        tip = self.tool_state()["tip"]
        goal_tip, _ = self._layer.step(tip, interface.POLICY_DT)
        # Across the axis the tip is held on the layer's goal until it touches; after, the miss is the contact's.
        free = engaged & ~self._layer.touched & (self._layer.phase == pressing.PRESSING)
        self._correction.update_tip(free, goal_tip, tip, interface.POLICY_DT, across=self._layer.n)
        self._tip_goal_w = torch.where(engaged.unsqueeze(-1), goal_tip, self._tip_goal_w)
        goal_jaw = self._jaw_for_tip(goal_tip, self._layer.n, measured=True)
        local = interface.quat_rotate_inverse(self._base_yaw_quat(), goal_jaw - self._goal_centre())
        self._goals.hold_goal = torch.where(engaged.unsqueeze(-1), interface.cart2sphere(local), self._goals.hold_goal)

    def _hold_pre_press(self) -> None:
        """Before the layer starts: hold the jaw goal where the tip lands on its pre-press pose, the tip loop
        integrating its miss once the goal has arrived there."""
        waiting = self._press_env & ~self._engaged
        if not bool(waiting.any()):
            return
        face, axis = self._planned_world(torch.arange(self.num_envs, device=self.device))
        tip_goal = face - self._standoff.unsqueeze(-1) * axis
        holding = waiting & self._goals.hold_mask
        self._correction.update_tip(holding, tip_goal, self.tool_state()["tip"], interface.POLICY_DT)
        self._tip_goal_w = torch.where(waiting.unsqueeze(-1), tip_goal, self._tip_goal_w)
        jaw = torch.where(holding.unsqueeze(-1), self._jaw_for_tip(tip_goal, axis, measured=True),
                          self._jaw_for_tip(tip_goal, axis, measured=False))
        sphere = interface.cart2sphere(interface.quat_rotate_inverse(self._base_yaw_quat(), jaw - self._goal_centre()))
        self._pre_goal = torch.where(waiting.unsqueeze(-1), sphere, self._pre_goal)
        self._goals.hold_goal = torch.where(holding.unsqueeze(-1), sphere, self._goals.hold_goal)

    def _attitude_miss(self) -> torch.Tensor:
        """(elevation, azimuth) of the hand's approach minus what is wanted, rad: (N, 2)."""
        approach, want = self._hand_frame_terms()["ee_approach_w"], self._approach_des_w
        elevation = torch.asin(approach[:, 2].clamp(-1.0, 1.0)) - torch.asin(want[:, 2].clamp(-1.0, 1.0))
        azimuth = insertion.wrap_half_turn(torch.atan2(approach[:, 1], approach[:, 0])
                                           - torch.atan2(want[:, 1], want[:, 0]))
        return torch.stack((elevation, azimuth), dim=-1)

    def _correct_attitude(self) -> None:
        """The elevation loop: from the moment the goal holds the pre-press pose until the tip touches."""
        mask = self._press_env & self._goals.hold_mask & ~self._layer.touched
        if not bool(mask.any()):
            return
        miss = self._attitude_miss()
        zero = torch.zeros_like(miss[:, 0])
        self._correction.update_attitude(mask, zero, miss[:, 0], zero, zero, interface.POLICY_DT)

    def _get_rewards(self) -> torch.Tensor:
        self._since_reset += 1
        arrived = self._press_env & ~self._goals.hold_mask & (self._since_reset >= self._goals.traj_steps.long())
        if bool(arrived.any()):
            self._goals.hold_goal[arrived] = self._pre_goal[arrived]
            self._goals.hold_roll[arrived] = 0.0
            self._goals.hold_mask[arrived] = True
        self._hold_pre_press()
        self._engage_due()
        self._run_layer()
        waiting = self._press_env & ~self._engaged
        if bool(waiting.any()):
            ids = waiting.nonzero(as_tuple=False).flatten()
            _, axis = self._planned_world(ids)
            self._approach_des_w[ids] = axis
        self._correct_attitude()
        self._commands[:, interface.CMD_EE_ORN_Y] = rewards.encode_approach(
            self._elevation + self._correction.elevation, self._press_env)
        pressed = self._button.pressed() & self._engaged
        dt = interface.POLICY_DT
        self._pressed_for = torch.where(pressed, self._pressed_for + dt, torch.zeros_like(self._pressed_for))
        self._longest_pressed = torch.maximum(self._longest_pressed, self._pressed_for)
        self._pressed_step = torch.where(pressed & (self._pressed_step < 0), self._since_reset, self._pressed_step)
        self._step_peak = self._button.take_peak() * self._engaged
        self._peak_contact = torch.maximum(self._peak_contact, self._step_peak)
        self._peak_lateral = torch.maximum(self._peak_lateral, self._button.lateral_n * self._engaged)
        return super()._get_rewards()

    def _task_state(self):
        state = super()._task_state()
        layer = self._layer
        pushing = (self._engaged & self._button.active & (layer.phase <= pressing.HOLDING)).float()
        force = self._button.force_on_tool
        lateral = force - (force * layer.n).sum(-1, keepdim=True) * layer.n
        state.fixture_engaged = pushing
        state.fixture_applied_w = -force
        state.fixture_command_w = layer.force_cmd
        state.fixture_axis_w = layer.n
        state.tool_vel_w = self.tool_state()["vel"]
        state.approach_des_w = self._approach_des_w
        state.approach_active = self._press_env.float()
        pressing_now = (self._engaged & (layer.phase == pressing.PRESSING)).float()
        state.press_error = (layer.s - layer.ref) * pressing_now
        state.press_pressing = pressing_now
        state.press_pressed = (self._button.pressed() & self._engaged).float()
        state.press_contact_n = self._button.normal_n
        state.press_lateral_n = lateral.norm(dim=-1)
        state.mech_torn = self._damage_now.float()
        miss = self._tip_pos() - self._goal_world()
        across = miss - (miss * layer.n).sum(-1, keepdim=True) * layer.n
        state.press_reach_error = torch.where(self._engaged, across.norm(dim=-1), miss.norm(dim=-1))
        state.press_active = self._press_env.float()
        state.press_holding = (self._press_env & self._goals.hold_mask).float()
        return state

    def _privileged_extra(self) -> torch.Tensor:
        """The button as the critic sees it, in the `mass_params` block. Zero where there is none."""
        b, layer = self._button, self._layer
        held = (self._engaged & b.active).float().unsqueeze(-1)
        yaw = self._base_yaw_quat()
        travel = b.travel.clamp(min=1e-3)
        out = torch.zeros(self.num_envs, press_cfg.PRIVILEGED_WIDTH, device=self.device)
        out[:, 0:1] = held
        out[:, 1:2] = b.on_cap.float().unsqueeze(-1) * held
        out[:, 2:3] = b.pressed().float().unsqueeze(-1) * held
        out[:, 3:4] = (b.s / travel).unsqueeze(-1) * held
        out[:, 4:5] = (layer.ref - layer.s).unsqueeze(-1) * 50.0 * held
        out[:, 5:8] = interface.quat_rotate_inverse(yaw, b.face - layer.face) * 100.0 * held
        out[:, 8:11] = interface.quat_rotate_inverse(yaw, b.n - layer.n) * 10.0 * held
        out[:, 11:14] = interface.quat_rotate_inverse(yaw, b.force_on_tool) * 0.02 * held
        out[:, 14:15] = b.full_travel_n().unsqueeze(-1) * 0.04 * held
        out[:, 15:16] = b.mu.unsqueeze(-1) * held
        out[:, 16:17] = b.cap_radius.unsqueeze(-1) * 50.0 * held
        out[:, 17:18] = layer.touched.float().unsqueeze(-1) * held
        out[:, 18:19] = layer.phase.float().unsqueeze(-1) / 3.0 * held
        out[:, 19:20] = self._press_env.float().unsqueeze(-1)
        out[:, 20] = self._level
        out[:, 21:22] = self._tool_index.float().unsqueeze(-1) * self._press_env.float().unsqueeze(-1)
        return out

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        fallen, timed_out = super()._get_dones()
        damaged = self._damage_now.clone()
        self._term_damage, self._term_fallen = damaged, fallen.clone()
        self._damage_now[:] = False
        return fallen | damaged, timed_out

    def _resample_velocity_commands(self, env_ids: torch.Tensor) -> None:
        """UniFP's draw, except that an episode with a button stands."""
        super()._resample_velocity_commands(env_ids)
        standing = env_ids[self._press_env[env_ids]]
        self._commands[standing, :3] = 0.0

    # --- evaluation -----------------------------------------------------------------------------------------------

    def set_evaluation(self, plan: dict) -> None:
        """Fix every episode's button and tool (`press_eval.build_plan`). Call before `reset()`. The curriculum is off."""
        self._evaluation = {k: (v.to(self.device) if torch.is_tensor(v) else v) for k, v in plan.items()}
        self.cfg.curriculum = False

    # --- resetting --------------------------------------------------------------------------------------------------

    def _update_curriculum(self, env_ids: torch.Tensor) -> None:
        log = self.extras.setdefault("log", {})
        planned = self._engaged[env_ids] | self._missed[env_ids]
        if bool(planned.any()):
            log["Press/missed_fraction"] = float(self._missed[env_ids][planned].float().mean())
        done = env_ids[self._engaged[env_ids]]
        if len(done) > 0:
            success = (self._longest_pressed[done] >= press_cfg.SUCCESS_HOLD_S).float()
            log["Press/success_fraction"] = float(success.mean())
            log["Press/ever_pressed_fraction"] = float((self._pressed_step[done] >= 0).float().mean())
            log["Press/touched_cap_fraction"] = float(self._button.touched_cap[done].float().mean())
            log["Press/peak_contact_n"] = float(self._peak_contact[done].mean())
            log["Press/peak_lateral_n"] = float(self._peak_lateral[done].mean())
            log["Press/damaged_fraction"] = float(self._term_damage[done].float().mean())
            log["Press/engage_tip_miss_mm"] = 1000.0 * float(self._engage_miss[done, 0].median())
            log["Press/engage_elevation_miss_deg"] = math.degrees(float(self._engage_miss[done, 1].abs().median()))
            reached = self._pressed_step[done] >= 0
            if bool(reached.any()):
                steps = (self._pressed_step[done][reached] - self._engage_step[done][reached]).float()
                log["Press/time_to_press_s"] = float(steps.mean()) * interface.POLICY_DT
            rate = float(success.mean())
            self._success_ema = rate if self._success_ema is None else (
                (1 - press_cfg.CURRICULUM_EMA) * self._success_ema + press_cfg.CURRICULUM_EMA * rate)
        wait = self.cfg.curriculum_min_iterations * task_cfg.NUM_STEPS_PER_ENV
        if (self.cfg.curriculum and self._success_ema is not None and self._success_ema > press_cfg.PROMOTE_PRESSED
                and self._level < 1.0 and self.common_step_counter - self._last_promotion_step >= wait):
            self._level = min(1.0, self._level + press_cfg.CURRICULUM_STEP)
            self._last_promotion_step = self.common_step_counter
            self._success_ema = None
        log["Press/curriculum_level"] = self._level
        if self._success_ema is not None:
            log["Press/success_ema"] = self._success_ema

    def _sample_buttons(self, count: int, length: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """(face in the yaw frame, elevation, standoff) for `count` new buttons, the pre-press pose inside the trained
        goal sphere for a tool of `length`."""
        device = self.device
        span = lambda r, n: r[0] + torch.rand(n, device=device) * (r[1] - r[0])
        face = torch.zeros(count, 3, device=device)
        elevation, standoff = torch.zeros(count, device=device), torch.zeros(count, device=device)
        pending = torch.arange(count, device=device)
        for _ in range(20):
            n = len(pending)
            face[pending] = torch.stack((span(press_cfg.FACE_AHEAD_M, n), span(press_cfg.FACE_SIDE_M, n),
                                         span(press_cfg.FACE_HEIGHT_M, n)), dim=-1)
            elevation[pending] = span(press_cfg.AXIS_ELEVATION_RAD, n)
            standoff[pending] = span(press_cfg.STANDOFF_M, n)
            sphere = self._pre_press_sphere(face[pending], elevation[pending], standoff[pending], length[pending])
            ok = ((sphere[:, 0] >= interface.EE_GOAL_RADIUS_RANGE[0] + 0.01)
                  & (sphere[:, 0] <= interface.EE_GOAL_RADIUS_RANGE[1] - 0.01)
                  & (sphere[:, 1] <= interface.EE_GOAL_PITCH_RANGE[1]))
            pending = pending[~ok]
            if len(pending) == 0:
                break
        if len(pending) > 0:
            face[pending] = torch.tensor((0.50, 0.0, 0.76), device=device)
            elevation[pending], standoff[pending] = 0.0, 0.012
        return face, elevation, standoff

    @staticmethod
    def _pre_press_sphere(face: torch.Tensor, elevation: torch.Tensor, standoff: torch.Tensor,
                          length: torch.Tensor) -> torch.Tensor:
        """The jaw centre's goal, as goal-sphere coordinates, for a tool tip `standoff` short of `face` (yaw frame)."""
        bearing = torch.atan2(face[:, 1], face[:, 0])
        axis = torch.stack((torch.cos(elevation) * torch.cos(bearing), torch.cos(elevation) * torch.sin(bearing),
                            torch.sin(elevation)), dim=-1)
        jaw = face - (standoff + length).unsqueeze(-1) * axis
        centre = torch.tensor(interface.EE_GOAL_CENTER_OFFSET, device=face.device)
        return interface.cart2sphere(jaw - centre)

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        self._update_curriculum(env_ids)
        count = len(env_ids)
        if self._evaluation is not None:
            self._press_env[env_ids] = True
        else:
            self._press_env[env_ids] = torch.rand(count, device=self.device) < self.cfg.button_fraction
        super()._reset_idx(env_ids)
        self._button.reset(env_ids)
        self._layer.reset(env_ids)
        self._correction.reset(env_ids)
        self._engage_miss[env_ids] = float("nan")
        self._tip_goal_w[env_ids] = 0.0
        for buf in (self._engaged, self._damage_now, self._missed):
            buf[env_ids] = False
        self._since_reset[env_ids] = 0
        self._pressed_step[env_ids] = -1
        for buf in (self._pressed_for, self._longest_pressed, self._peak_contact, self._peak_lateral, self._elevation):
            buf[env_ids] = 0.0
        self._ee_force_w[env_ids] = 0.0
        self._goals.hold_mask[env_ids] = False
        # The tool, for every environment (a free reach carries one too).
        if self._evaluation is not None:
            index = self._evaluation["tool"][env_ids]
        else:
            allowed = torch.tensor([pressing.TOOL_NAMES.index(name) for name in self.cfg.tools], device=self.device)
            index = allowed[torch.randint(0, len(allowed), (count,), device=self.device)]
        self._tool_index[env_ids] = index
        length, radius = pressing.tool_by_index(index)
        self._tool_length[env_ids], self._tool_radius[env_ids] = length, radius
        ids = env_ids[self._press_env[env_ids]]
        if len(ids) > 0:
            if self._evaluation is not None:
                plan = self._evaluation
                face, elevation = plan["face"][ids], plan["elevation"][ids]
                standoff = torch.full((len(ids),), plan["conditions"]["standoff_m"], device=self.device)
                delay = torch.full((len(ids),), 15, device=self.device)
            else:
                face, elevation, standoff = self._sample_buttons(len(ids), self._tool_length[ids])
                low, high = press_cfg.ENGAGE_DELAY_STEPS
                delay = torch.randint(low, high + 1, (len(ids),), device=self.device)
            self._face_local[ids], self._elevation[ids], self._standoff[ids] = face, elevation, standoff
            self._pre_goal[ids] = self._pre_press_sphere(face, elevation, standoff, self._tool_length[ids])
            self._goals.aim(ids, self._pre_goal[ids], torch.zeros(len(ids), device=self.device))
            self._engage_step[ids] = self._goals.traj_steps[ids].long() + delay.long()
