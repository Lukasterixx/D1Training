"""The insertion task: the robot holds a key and pushes it into a keyed slot it knows only from an estimate.

What an episode is, for the `insertion_fraction` of environments that get one:

  1. A slot is drawn in the robot's yaw frame (`insert_cfg.MOUTH_*`): where its mouth is, its axis's elevation, its
     roll. The goal slides to the pre-insertion pose -- the key's tip `standoff` short of the mouth, the hand's
     approach along the slot's axis (the approach objective, `rewards.tracking_ee_orn_approach`), its roll the
     blade's -- and the approach is commanded from the start, so the hand arrives lined up.
  2. Once the hand has settled there, the task layer (`insertion.InsertionTaskLayer`) starts on that *estimate* of the
     slot, and the true slot (`insertion.Slot`) is placed off it by the curriculum's errors: across the axis, in tilt,
     in roll. The policy observes neither. Throughout, the layer's outer loops (`insertion.PoseCorrection`) integrate
     the measured miss of the key's tip, the approach's elevation and the blade's roll into the commands, because the
     policy never observes its own hand (F-118); with them on, the layer waits for the tip to converge before it starts.
  3. The layer runs the insertion: the goal across the axis on the estimate (or its spiral search while the key is
     stalled on the door's face), a PI force command along the axis on how far the tip lags a reference depth.
     The contact force acts on the key's tip, `insertion.KEY_LENGTH_M` beyond the jaw centre, so the hand feels its
     moment. The velocity command is zero.
  4. Seated is the goal; a contact force past `insert_cfg.DAMAGE_N`, or a fall, ends the episode.

The rest reach through free space with no pushes and no approach command. The policy's widths do not change: the
approach command rides in `CMD_EE_ORN_Y` (`rewards.encode_approach`), a channel UniFP declares and never writes.
"""
from __future__ import annotations

import math
from types import SimpleNamespace

import torch

from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.utils import configclass

from unifp_isaaclab import interface

from . import insert_cfg, insert_rewards, insertion, rewards, task_cfg
from .env import Go2D1PosForceEnv
from .env_cfg import Go2D1PosForceEnvCfg
from .hook_env import HeldGoals


@configclass
class Go2D1InsertEnvCfg(Go2D1PosForceEnvCfg):
    """UniFP's robot and observation, holding a key; the roll and approach objectives on."""

    roll_objective: bool = True
    force_start_step: int = 0
    insertion_fraction: float = insert_cfg.INSERTION_FRACTION
    curriculum: bool = True
    curriculum_level: float = insert_cfg.CURRICULUM_START
    #: The fewest iterations between two promotions of the curriculum.
    curriculum_min_iterations: int = insert_cfg.CURRICULUM_MIN_ITERATIONS
    #: The outer loops' integral gains, 1/s (`insert_cfg`, `insertion.PoseCorrection`); 0 turns one off.
    tip_correction_gain: float = insert_cfg.TIP_CORRECTION_GAIN
    elevation_correction_gain: float = insert_cfg.ELEVATION_CORRECTION_GAIN
    roll_correction_gain: float = insert_cfg.ROLL_CORRECTION_GAIN
    #: The loops' clamps: how far the tip loop may move the jaw goal, m, and the elevation loop the approach, rad.
    tip_correction_max_m: float = insert_cfg.TIP_CORRECTION_MAX_M
    elevation_correction_max_rad: float = insert_cfg.ELEVATION_CORRECTION_MAX_RAD
    roll_correction_max_rad: float = insert_cfg.ROLL_CORRECTION_MAX_RAD
    #: Hold the key in a compliant grip (`insertion.CompliantGrip`) rather than rigidly (from 2026-09-27).
    compliant_grip: bool = False
    #: The task layer holds a seated key home (`InsertionTaskLayer.home`, from 2026-09-27); False drives it as before.
    hold_home: bool = True
    #: Score the policy's reach and roll against what the task wants -- the key's tip on the layer's tip goal, the
    #: slot's roll -- rather than against the commands the outer loops have shifted (from v11, 2026-09-27). Scored
    #: against the commands, the policy is paid for tracking the loops' corrections, and the two can chase each other.
    reward_true_targets: bool = False
    #: With the loops on, also wait for the approach's elevation and the blade's roll to be within this of their
    #: targets, rad, before the layer starts (None: the tip alone; the deadline applies either way).
    engage_attitude_rad: float | None = None
    #: The approach term's weight, per second (`task_cfg.APPROACH_WEIGHTS`, 2.5, unless set).
    approach_weight: float | None = None
    arm_contact_sensor: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/D1/.*", history_length=0, update_period=0.0, track_air_time=False)


class Go2D1InsertEnv(Go2D1PosForceEnv):
    cfg: Go2D1InsertEnvCfg

    def __init__(self, cfg: Go2D1InsertEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, device = self.num_envs, self.device
        self._goals = HeldGoals.adopt(self._goals)
        self._slot = insertion.Slot(n, device)
        self._layer = insertion.InsertionTaskLayer(
            n, device, kp=insert_cfg.FORCE_LAW_KP_N_PER_M, ki=insert_cfg.FORCE_LAW_KI_N_PER_M_S,
            bleed_s=insert_cfg.FORCE_LAW_BLEED_S, home_m=0.001 if cfg.hold_home else -1.0)
        self._correction = insertion.PoseCorrection(
            n, device, tip_gain=cfg.tip_correction_gain, tip_max_m=cfg.tip_correction_max_m,
            elevation_gain=cfg.elevation_correction_gain, elevation_max_rad=cfg.elevation_correction_max_rad,
            roll_gain=cfg.roll_correction_gain, roll_max_rad=cfg.roll_correction_max_rad)
        zeros = lambda *shape: torch.zeros(*shape, device=device)
        flags = lambda: torch.zeros(n, dtype=torch.bool, device=device)
        self._ins_env, self._engaged, self._ever_seated = flags(), flags(), flags()
        #: Episodes whose hand never got near its pre-insertion pose in time, and so got no slot.
        self._missed = flags()
        self._damage_now, self._term_damage, self._term_fallen = flags(), flags(), flags()
        self._since_reset = torch.zeros(n, dtype=torch.long, device=device)
        self._engage_step = torch.zeros(n, dtype=torch.long, device=device)
        self._seated_step = torch.full((n,), -1, dtype=torch.long, device=device)
        #: The planned slot in the robot's yaw frame at reset (from the ground under the base), and its axis and roll.
        self._mouth_local, self._elevation, self._roll, self._standoff = zeros(n, 3), zeros(n), zeros(n), zeros(n)
        #: The pre-insertion goal (goal-sphere coordinates), held once the goal's trajectory has arrived there.
        self._pre_goal = zeros(n, 3)
        #: The approach the hand is asked for, world (the planned axis until the task layer starts, its axis after).
        self._approach_des_w = zeros(n, 3)
        self._peak_contact, self._peak_lateral = zeros(n), zeros(n)
        #: At the moment the layer started: the tip's miss of its pre-insertion pose (m), and the approach's elevation
        #: and azimuth misses and the blade's roll miss (rad, signed). NaN before.
        self._engage_miss = torch.full((n, 4), float("nan"), device=device)
        #: The key tip's goal, world (env frame): the planned pre-insertion tip while waiting, the layer's after.
        self._tip_goal_w = zeros(n, 3)
        #: The wrench's point on Link6: the jaw centre plus the key's length along the approach.
        offset = torch.as_tensor(self.cfg.tool_offset_m, device=device) + torch.tensor(
            [0.0, 0.0, insertion.KEY_LENGTH_M], device=device)
        self._force_offset = offset.expand(n, 1, 3).contiguous()
        #: The key's grip: rigid (None), or compliant about the jaw centre, in which case the tip and the wrench's
        #: point move with its tilt.
        self._grip = insertion.CompliantGrip(n, device) if cfg.compliant_grip else None
        self._level = float(cfg.curriculum_level)
        self._seated_ema: float | None = None
        self._last_promotion_step = 0
        self._evaluation: dict | None = None

        # The task layer's force command goes through UniFP's force channel at the stiffness it was trained through,
        # as the mechanism task's law does (`mech_env.set_force_law`).
        self._gripper_force_kp.fill_(task_cfg.GRIPPER_FORCE_KP)
        self._weights.update(insert_cfg.scaled_weights())
        self._terms.update(insert_rewards.TERMS)
        approach = dict(task_cfg.APPROACH_WEIGHTS)
        if cfg.approach_weight is not None:
            approach = {name: cfg.approach_weight for name in approach}
        self._weights.update({name: w * interface.POLICY_DT for name, w in approach.items()})
        self._terms.update(rewards.APPROACH_TERMS)
        for name in self._weights:
            self._episode_sums.setdefault(name, zeros(n))

    def _setup_scene(self):
        super()._setup_scene()
        self._arm_contact = ContactSensor(self.cfg.arm_contact_sensor)
        self.scene.sensors["arm_contact"] = self._arm_contact

    # --- the key ---------------------------------------------------------------------------------------------------

    def _tool_vel_w(self) -> torch.Tensor:
        """World velocity of the controlled point: the body's plus rotation about its offset."""
        quat = self._robot.data.body_quat_w[:, self._tip_body]
        arm = interface.quat_apply(quat, self._tool_offset[:, 0])
        return (self._robot.data.body_lin_vel_w[:, self._tip_body]
                + torch.cross(self._robot.data.body_ang_vel_w[:, self._tip_body], arm, dim=-1))

    def key_state(self) -> dict[str, torch.Tensor]:
        """The key's tip (env frame), its velocity, its axis and its blade's long direction (world)."""
        if self._grip is None:
            n = self.num_envs
            axis_local = torch.tensor([0.0, 0.0, 1.0], device=self.device).expand(n, 3)
            long_local = torch.tensor([0.0, 1.0, 0.0], device=self.device).expand(n, 3)
        else:
            axis_local, long_local = self._grip.axes_local()
        return self._key_at(axis_local, long_local)

    def _key_at(self, axis_local: torch.Tensor, long_local: torch.Tensor) -> dict[str, torch.Tensor]:
        """`key_state` with the key pointing `axis_local`, its blade along `long_local`, in Link6's frame."""
        quat = self._robot.data.body_quat_w[:, self._tip_body]
        pos = self._robot.data.body_pos_w[:, self._tip_body] - self.scene.env_origins
        offset = self._tool_offset[:, 0] + insertion.KEY_LENGTH_M * axis_local
        arm = interface.quat_apply(quat, offset)
        vel = (self._robot.data.body_lin_vel_w[:, self._tip_body]
               + torch.cross(self._robot.data.body_ang_vel_w[:, self._tip_body], arm, dim=-1))
        axis = interface.quat_apply(quat, axis_local.contiguous())
        long = interface.quat_apply(quat, long_local.contiguous())
        return {"tip": pos + arm, "vel": vel, "axis": axis, "long": long, "offset_local": offset}

    def _apply_action(self) -> None:
        super()._apply_action()
        if self._grip is not None:
            quat = self._robot.data.body_quat_w[:, self._tip_body]

            def probe(axis_local, long_local):
                trial = self._key_at(axis_local, long_local)
                force, normal = self._slot.probe(trial["tip"], trial["vel"], trial["axis"], trial["long"])
                return interface.quat_rotate_inverse(quat, force), normal > 0.0

            self._grip.settle(probe, insertion.KEY_LENGTH_M, interface.SIM_DT)
        key = self.key_state()
        if self._grip is not None:
            self._force_offset = key["offset_local"].unsqueeze(1).contiguous()
        self._ee_force_w = self._slot.step(key["tip"], key["vel"], key["axis"], key["long"], interface.SIM_DT)
        self._peak_contact = torch.maximum(self._peak_contact, self._slot.normal_sum_n)
        self._damage_now |= (self._slot.normal_sum_n > insert_cfg.DAMAGE_N) | self._slot.pierced
        self._write_wrench()

    def _write_wrench(self) -> None:
        """As the base class, but at the key's tip rather than the controlled point."""
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
        """The planned mouth (env frame) and axis (world) for `ids`, from the base's pose now."""
        base = self._robot.data.root_pos_w[ids] - self.scene.env_origins[ids]
        yaw = self._base_yaw_quat()[ids]
        ground = base.clone()
        ground[:, 2] = 0.0
        mouth = ground + interface.quat_apply(yaw, self._mouth_local[ids])
        bearing = torch.atan2(self._mouth_local[ids, 1], self._mouth_local[ids, 0])
        axis = rewards.approach_direction(self._elevation[ids], bearing, yaw)
        return mouth, axis

    def _engage_due(self) -> None:
        """Start the task layer once the hand has settled with the key's tip near its pre-insertion pose. A hand that has
        not got there by the deadline gets no slot this episode -- it becomes a free reach -- rather than a door placed
        around wherever it happens to be."""
        waited = self._since_reset - self._engage_step
        waiting = self._ins_env & ~self._engaged & (waited >= 0)
        if not bool(waiting.any()):
            return
        settled = self._tool_vel_w().norm(dim=-1) < insert_cfg.ENGAGE_SETTLE_SPEED_M_S
        all_ids = torch.arange(self.num_envs, device=self.device)
        mouth_all, axis_all = self._planned_world(all_ids)
        planned_tip = mouth_all - self._standoff.unsqueeze(-1) * axis_all
        miss = (self.key_state()["tip"] - planned_tip).norm(dim=-1)
        converged = insert_cfg.ENGAGE_CONVERGED_M if self._correction.tip_gain > 0.0 else insert_cfg.ENGAGE_NEAR_M
        deadline = waited >= insert_cfg.ENGAGE_MAX_WAIT_STEPS
        ready = settled & (miss < converged)
        if self.cfg.engage_attitude_rad is not None:
            attitude = self._attitude_miss().abs()
            ready &= (attitude[:, 0] < self.cfg.engage_attitude_rad) & (attitude[:, 2] < self.cfg.engage_attitude_rad)
        due = waiting & (ready | (deadline & (miss < insert_cfg.ENGAGE_NEAR_M)))
        late = waiting & ~due & deadline
        if bool(late.any()):
            self._ins_env &= ~late
            self._goals.hold_mask &= ~late
            self._missed |= late
        if not bool(due.any()):
            return
        ids = due.nonzero(as_tuple=False).flatten()
        mouth, axis = mouth_all[ids], axis_all[ids]
        draw = self._draw_evaluation(ids) if self._evaluation is not None else self._draw(ids)
        # The true slot: the estimate moved across its axis, tilted, and rolled.
        u, w = insertion.roll_frame(axis, self._roll[ids])
        lateral = draw["lateral"][:, :1] * u + draw["lateral"][:, 1:] * w
        tilt = draw["tilt"]
        true_axis = insertion.unit(axis + tilt[:, :1] * u + tilt[:, 1:] * w)
        half_u = insertion.KEY_HALF_U_M + draw["clearance"]
        half_w = insertion.KEY_HALF_W_M + draw["clearance"]
        self._slot.place(ids, mouth=mouth + lateral, axis=true_axis, roll=self._roll[ids] + draw["roll_error"],
                         half_u=half_u, half_w=half_w, chamfer=draw["chamfer"], depth=draw["depth"],
                         detent_depth=draw["depth"] * draw["detent_fraction"], detent_n=draw["detent_n"],
                         mu=draw["mu"])
        self._layer.start(ids, mouth=mouth, axis=axis, roll=self._roll[ids], standoff=self._standoff[ids],
                          depth=draw["depth"], speed=draw["speed"], search_n=draw["search_n"], max_n=draw["max_n"])
        self._approach_des_w[ids] = axis
        self._engaged[ids] = True
        self._goals.hold_mask[ids] = True
        self._goals.hold_roll[ids] = self._roll[ids] + self._correction.roll[ids]
        attitude = self._attitude_miss()
        self._engage_miss[ids] = torch.cat((miss[ids].unsqueeze(-1), attitude[ids]), dim=-1)

    def _draw(self, ids: torch.Tensor) -> dict[str, torch.Tensor]:
        count, device, level = len(ids), self.device, self._level
        rand = lambda *shape: torch.rand(*shape, device=device)
        span = lambda r: r[0] + rand(count) * (r[1] - r[0])

        def disc(radius: float) -> torch.Tensor:
            """Uniform over a disc of `radius`, (count, 2)."""
            angle = 2 * math.pi * rand(count, 1)
            return radius * torch.sqrt(rand(count, 1)) * torch.cat([torch.cos(angle), torch.sin(angle)], dim=-1)

        return {"lateral": disc(level * insert_cfg.LATERAL_ERROR_M), "tilt": disc(level * insert_cfg.TILT_ERROR_RAD),
                "roll_error": (2 * rand(count) - 1) * level * insert_cfg.ROLL_ERROR_RAD,
                "clearance": span(insert_cfg.CLEARANCE_M) + (1.0 - level) * insert_cfg.CLEARANCE_EASY_EXTRA_M,
                "chamfer": span(insert_cfg.CHAMFER_M) + (1.0 - level) * insert_cfg.CHAMFER_EASY_EXTRA_M,
                "depth": span(insert_cfg.DEPTH_M), "detent_fraction": span(insert_cfg.DETENT_FRACTION),
                "detent_n": rand(count) * level * insert_cfg.DETENT_N, "mu": span(insert_cfg.FRICTION_MU),
                "speed": span(insert_cfg.REFERENCE_SPEED_M_S), "search_n": span(insert_cfg.SEARCH_N),
                "max_n": span(insert_cfg.MAX_N)}

    def _draw_evaluation(self, ids: torch.Tensor) -> dict[str, torch.Tensor]:
        plan, c = self._evaluation, self._evaluation["conditions"]
        full = lambda v: torch.full((len(ids),), float(v), device=self.device)
        return {"lateral": plan["lateral"][ids], "tilt": plan["tilt"][ids], "roll_error": plan["roll_error"][ids],
                "clearance": full(c["clearance_m"]), "chamfer": full(c["chamfer_m"]), "depth": full(c["depth_m"]),
                "detent_fraction": full(c["detent_fraction"]), "detent_n": plan["detent_n"][ids], "mu": full(c["mu"]),
                "speed": full(c["speed_m_s"]), "search_n": full(c["search_n"]), "max_n": full(c["max_n"])}

    def _run_layer(self) -> None:
        """Advance the task layer and hold each engaged environment's goal where it says, in the base's sphere now."""
        engaged = self._engaged
        if not bool(engaged.any()):
            return
        key = self.key_state()
        goal_tip, _ = self._layer.step(key["tip"], interface.POLICY_DT)
        # Across the axis, the tip is held on the layer's goal until the key touches; after, the miss is the slot's.
        free = engaged & ~self._layer.touched & ~self._layer.inserted
        self._correction.update_tip(free, goal_tip, key["tip"], interface.POLICY_DT, across=self._layer.n)
        self._tip_goal_w = torch.where(engaged.unsqueeze(-1), goal_tip, self._tip_goal_w)
        goal_jaw = self._jaw_for_tip(goal_tip, self._layer.n)
        local = interface.quat_rotate_inverse(self._base_yaw_quat(), goal_jaw - self._goal_centre())
        self._goals.hold_goal = torch.where(engaged.unsqueeze(-1), interface.cart2sphere(local), self._goals.hold_goal)

    # --- the outer loops ------------------------------------------------------------------------------------------

    def _jaw_for_tip(self, tip_goal: torch.Tensor, axis: torch.Tensor) -> torch.Tensor:
        """Where the jaw centre must be for the key's tip to be at `tip_goal`: behind it along `axis` (the estimate's),
        or -- with the tip loop on -- along the hand's measured approach, plus the loop's integrated offset."""
        if self._correction.tip_gain <= 0.0:
            return tip_goal - insertion.KEY_LENGTH_M * axis
        approach = self._hand_frame_terms()["ee_approach_w"]
        return self._correction.jaw_for_tip(tip_goal, approach, insertion.KEY_LENGTH_M)

    def _hold_pre_insertion(self) -> None:
        """Before the layer starts: hold the jaw goal where the key's tip lands on its planned pre-insertion pose, the
        tip loop integrating its miss once the goal has arrived there."""
        waiting = self._ins_env & ~self._engaged
        if not bool(waiting.any()):
            return
        mouth, axis = self._planned_world(torch.arange(self.num_envs, device=self.device))
        tip_goal = mouth - self._standoff.unsqueeze(-1) * axis
        holding = waiting & self._goals.hold_mask
        self._correction.update_tip(holding, tip_goal, self.key_state()["tip"], interface.POLICY_DT)
        self._tip_goal_w = torch.where(waiting.unsqueeze(-1), tip_goal, self._tip_goal_w)
        jaw = torch.where(holding.unsqueeze(-1), self._jaw_for_tip(tip_goal, axis),
                          tip_goal - insertion.KEY_LENGTH_M * axis)
        sphere = interface.cart2sphere(interface.quat_rotate_inverse(self._base_yaw_quat(), jaw - self._goal_centre()))
        self._pre_goal = torch.where(waiting.unsqueeze(-1), sphere, self._pre_goal)
        self._goals.hold_goal = torch.where(holding.unsqueeze(-1), sphere, self._goals.hold_goal)

    def _attitude_miss(self) -> torch.Tensor:
        """(elevation, azimuth, roll) of the hand minus what is wanted, rad: (N, 3)."""
        frame = self._hand_frame_terms()
        approach, want = frame["ee_approach_w"], self._approach_des_w
        elevation = torch.asin(approach[:, 2].clamp(-1.0, 1.0)) - torch.asin(want[:, 2].clamp(-1.0, 1.0))
        azimuth = insertion.wrap_half_turn(torch.atan2(approach[:, 1], approach[:, 0])
                                           - torch.atan2(want[:, 1], want[:, 0]))
        roll = insertion.wrap_half_turn(rewards.tool_roll(SimpleNamespace(**frame)) - self._roll)
        return torch.stack((elevation, azimuth, roll), dim=-1)

    def _correct_attitude(self) -> None:
        """The elevation and roll loops: from the moment the goal holds the pre-insertion pose until the key is in."""
        mask = self._ins_env & self._goals.hold_mask & ~self._layer.inserted
        if not bool(mask.any()):
            return
        miss = self._attitude_miss()
        zero = torch.zeros_like(miss[:, 0])
        self._correction.update_attitude(mask, zero, miss[:, 0], zero, miss[:, 2], interface.POLICY_DT)
        held = self._ins_env & self._goals.hold_mask
        self._goals.hold_roll = torch.where(held, self._roll + self._correction.roll, self._goals.hold_roll)

    def _get_rewards(self) -> torch.Tensor:
        self._since_reset += 1
        # Hold the pre-insertion goal once the trajectory has brought it there, so UniFP's generator does not move on.
        arrived = self._ins_env & ~self._goals.hold_mask & (self._since_reset >= self._goals.traj_steps.long())
        if bool(arrived.any()):
            self._goals.hold_goal[arrived] = self._pre_goal[arrived]
            self._goals.hold_roll[arrived] = self._roll[arrived]
            self._goals.hold_mask[arrived] = True
        self._hold_pre_insertion()
        self._engage_due()
        self._run_layer()
        # The approach: commanded for every environment with a slot, along its planned axis until the layer starts.
        waiting = self._ins_env & ~self._engaged
        if bool(waiting.any()):
            ids = waiting.nonzero(as_tuple=False).flatten()
            _, axis = self._planned_world(ids)
            self._approach_des_w[ids] = axis
        self._correct_attitude()
        self._commands[:, interface.CMD_EE_ORN_Y] = rewards.encode_approach(
            self._elevation + self._correction.elevation, self._ins_env)
        seated = self._slot.seated()
        self._seated_step = torch.where(seated & (self._seated_step < 0), self._since_reset, self._seated_step)
        self._ever_seated |= seated
        lateral = self._slot.force_on_tip - (self._slot.force_on_tip * self._layer.n).sum(-1, keepdim=True) * self._layer.n
        self._peak_lateral = torch.maximum(self._peak_lateral, lateral.norm(dim=-1) * self._engaged)
        return super()._get_rewards()

    def _task_state(self):
        state = super()._task_state()
        engaged = (self._engaged & self._slot.active).float()
        force = self._slot.force_on_tip
        lateral = force - (force * self._layer.n).sum(-1, keepdim=True) * self._layer.n
        state.fixture_engaged = engaged
        state.fixture_applied_w = -force
        state.fixture_command_w = self._layer.force_cmd
        state.fixture_axis_w = self._layer.n
        state.tool_vel_w = self._tool_vel_w()
        state.approach_des_w = self._approach_des_w
        state.approach_active = self._ins_env.float()
        state.ins_error = (self._layer.s - self._layer.ref) * engaged
        state.ins_travel = self._layer.travel
        state.ins_seated = self._slot.seated().float()
        state.ins_contact_n = self._slot.normal_sum_n
        state.ins_lateral_n = lateral.norm(dim=-1)
        state.mech_torn = self._damage_now.float()
        miss = self._tip_pos() - self._goal_world()
        across = miss - (miss * self._layer.n).sum(-1, keepdim=True) * self._layer.n
        state.ins_reach_error = torch.where(self._engaged, across.norm(dim=-1), miss.norm(dim=-1))
        if self.cfg.reward_true_targets:
            # Once the goal is held: the key's tip against the tip goal (across the axis once the layer has started).
            held = self._ins_env & self._goals.hold_mask
            tip_miss = self.key_state()["tip"] - self._tip_goal_w
            tip_across = tip_miss - (tip_miss * self._layer.n).sum(-1, keepdim=True) * self._layer.n
            true_error = torch.where(self._engaged, tip_across.norm(dim=-1), tip_miss.norm(dim=-1))
            state.ins_reach_error = torch.where(held, true_error, state.ins_reach_error)
            state.roll_true = self._roll
            state.roll_true_mask = held
        state.ins_active = self._ins_env.float()
        state.ins_holding = (self._ins_env & self._goals.hold_mask).float()
        return state

    def _privileged_extra(self) -> torch.Tensor:
        """The slot as the critic sees it, in the `mass_params` block. Zero where there is none."""
        s, layer = self._slot, self._layer
        held = (self._engaged & s.active).float().unsqueeze(-1)
        yaw = self._base_yaw_quat()
        out = torch.zeros(self.num_envs, insert_cfg.PRIVILEGED_WIDTH, device=self.device)
        out[:, 0:1] = held
        out[:, 1:2] = s.entered.float().unsqueeze(-1) * held
        out[:, 2:3] = s.clicked.float().unsqueeze(-1) * held
        out[:, 3:4] = s.tip_depth.clamp(-0.05, 0.05).unsqueeze(-1) * 20.0 * held
        out[:, 4:5] = (layer.ref - layer.s).unsqueeze(-1) * 50.0 * held
        out[:, 5:8] = interface.quat_rotate_inverse(yaw, s.mouth - layer.mouth) * 100.0 * held
        out[:, 8:11] = interface.quat_rotate_inverse(yaw, s.n - layer.n) * 10.0 * held
        out[:, 11:14] = interface.quat_rotate_inverse(yaw, s.force_on_tip) * 0.02 * held
        out[:, 14:15] = s.detent_n.unsqueeze(-1) * 0.04 * held
        out[:, 15:16] = s.mu.unsqueeze(-1) * held
        out[:, 16:17] = (s.half_w - insertion.KEY_HALF_W_M).unsqueeze(-1) * 500.0 * held
        out[:, 17:18] = layer.inserted.float().unsqueeze(-1) * held
        out[:, 18:19] = layer.stall.clamp(max=2.0).unsqueeze(-1) * held
        out[:, 19:20] = self._ins_env.float().unsqueeze(-1)
        out[:, 20] = self._level
        out[:, 21:22] = self._elevation.unsqueeze(-1) * self._ins_env.float().unsqueeze(-1)
        return out

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        fallen, timed_out = super()._get_dones()
        damaged = self._damage_now.clone()
        self._term_damage, self._term_fallen = damaged, fallen.clone()
        self._damage_now[:] = False
        return fallen | damaged, timed_out

    def _resample_velocity_commands(self, env_ids: torch.Tensor) -> None:
        """UniFP's draw, except that an episode with a slot stands."""
        super()._resample_velocity_commands(env_ids)
        standing = env_ids[self._ins_env[env_ids]]
        self._commands[standing, :3] = 0.0

    # --- evaluation -----------------------------------------------------------------------------------------------

    def set_evaluation(self, plan: dict) -> None:
        """Fix every episode's slot (`insert_eval.build_plan`). Call before `reset()`. The curriculum is off."""
        self._evaluation = {k: (v.to(self.device) if torch.is_tensor(v) else v) for k, v in plan.items()}
        self.cfg.curriculum = False

    # --- resetting --------------------------------------------------------------------------------------------------

    def _update_curriculum(self, env_ids: torch.Tensor) -> None:
        log = self.extras.setdefault("log", {})
        planned = self._engaged[env_ids] | self._missed[env_ids]
        if bool(planned.any()):
            log["Insertion/missed_fraction"] = float(self._missed[env_ids][planned].float().mean())
        done = env_ids[self._engaged[env_ids]]
        if len(done) > 0:
            seated = self._ever_seated[done].float()
            log["Insertion/seated_fraction"] = float(seated.mean())
            log["Insertion/inserted_fraction"] = float(self._layer.inserted[done].float().mean())
            log["Insertion/searched_fraction"] = float((self._layer.theta[done] > 0.0).float().mean())
            log["Insertion/peak_contact_n"] = float(self._peak_contact[done].mean())
            log["Insertion/peak_lateral_n"] = float(self._peak_lateral[done].mean())
            log["Insertion/damaged_fraction"] = float(self._term_damage[done].float().mean())
            at_engage = self._engage_miss[done]
            log["Insertion/engage_tip_miss_mm"] = 1000.0 * float(at_engage[:, 0].median())
            for k, name in ((1, "elevation"), (2, "azimuth"), (3, "roll")):
                log[f"Insertion/engage_{name}_miss_deg"] = math.degrees(float(at_engage[:, k].abs().median()))
            steps = (self._seated_step[done] - self._engage_step[done]).float() * interface.POLICY_DT
            if bool(seated.any()):
                log["Insertion/time_to_seat_s"] = float(steps[seated > 0].mean())
            rate = float(seated.mean())
            self._seated_ema = rate if self._seated_ema is None else (
                (1 - insert_cfg.CURRICULUM_EMA) * self._seated_ema + insert_cfg.CURRICULUM_EMA * rate)
        # The approach at the moment the layer starts is logged by the reward term; the curriculum's level here.
        wait = self.cfg.curriculum_min_iterations * task_cfg.NUM_STEPS_PER_ENV
        if (self.cfg.curriculum and self._seated_ema is not None and self._seated_ema > insert_cfg.PROMOTE_SEATED
                and self._level < 1.0 and self.common_step_counter - self._last_promotion_step >= wait):
            self._level = min(1.0, self._level + insert_cfg.CURRICULUM_STEP)
            self._last_promotion_step = self.common_step_counter
            self._seated_ema = None
        log["Insertion/curriculum_level"] = self._level
        if self._seated_ema is not None:
            log["Insertion/seated_ema"] = self._seated_ema

    def _sample_slots(self, count: int) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """(mouth in the yaw frame, elevation, roll, standoff) for `count` new slots, the pre-insertion pose inside
        the trained goal sphere."""
        device = self.device
        span = lambda r, n: r[0] + torch.rand(n, device=device) * (r[1] - r[0])
        mouth = torch.zeros(count, 3, device=device)
        elevation, roll, standoff = torch.zeros(count, device=device), torch.zeros(count, device=device), torch.zeros(count, device=device)
        pending = torch.arange(count, device=device)
        for _ in range(20):
            n = len(pending)
            mouth[pending] = torch.stack((span(insert_cfg.MOUTH_AHEAD_M, n), span(insert_cfg.MOUTH_SIDE_M, n),
                                          span(insert_cfg.MOUTH_HEIGHT_M, n)), dim=-1)
            elevation[pending] = span(insert_cfg.AXIS_ELEVATION_RAD, n)
            roll[pending] = span(insert_cfg.ROLL_RAD, n)
            standoff[pending] = span(insert_cfg.STANDOFF_M, n)
            sphere = self._pre_insertion_sphere(mouth[pending], elevation[pending], standoff[pending])
            ok = ((sphere[:, 0] >= interface.EE_GOAL_RADIUS_RANGE[0] + 0.01)
                  & (sphere[:, 0] <= interface.EE_GOAL_RADIUS_RANGE[1] - 0.01)
                  & (sphere[:, 1] <= interface.EE_GOAL_PITCH_RANGE[1]))
            pending = pending[~ok]
            if len(pending) == 0:
                break
        # Anything still outside goes to the middle of the range.
        if len(pending) > 0:
            mouth[pending] = torch.tensor((0.50, 0.0, 0.76), device=device)
            elevation[pending], roll[pending], standoff[pending] = 0.0, 0.0, 0.012
        return mouth, elevation, roll, standoff

    @staticmethod
    def _pre_insertion_sphere(mouth: torch.Tensor, elevation: torch.Tensor, standoff: torch.Tensor) -> torch.Tensor:
        """The jaw centre's goal, as goal-sphere coordinates, for a key tip `standoff` short of `mouth` (yaw frame)."""
        bearing = torch.atan2(mouth[:, 1], mouth[:, 0])
        axis = torch.stack((torch.cos(elevation) * torch.cos(bearing), torch.cos(elevation) * torch.sin(bearing),
                            torch.sin(elevation)), dim=-1)
        jaw = mouth - (standoff + insertion.KEY_LENGTH_M).unsqueeze(-1) * axis
        centre = torch.tensor(interface.EE_GOAL_CENTER_OFFSET, device=mouth.device)
        return interface.cart2sphere(jaw - centre)

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        self._update_curriculum(env_ids)
        count = len(env_ids)
        if self._evaluation is not None:
            self._ins_env[env_ids] = True
        else:
            self._ins_env[env_ids] = torch.rand(count, device=self.device) < self.cfg.insertion_fraction
        super()._reset_idx(env_ids)
        self._slot.reset(env_ids)
        self._layer.reset(env_ids)
        self._correction.reset(env_ids)
        if self._grip is not None:
            self._grip.reset(env_ids)
        self._engage_miss[env_ids] = float("nan")
        self._tip_goal_w[env_ids] = 0.0
        for buf in (self._engaged, self._ever_seated, self._damage_now, self._missed):
            buf[env_ids] = False
        self._since_reset[env_ids] = 0
        self._seated_step[env_ids] = -1
        self._peak_contact[env_ids] = 0.0
        self._peak_lateral[env_ids] = 0.0
        self._ee_force_w[env_ids] = 0.0
        self._goals.hold_mask[env_ids] = False
        self._elevation[env_ids] = 0.0
        ids = env_ids[self._ins_env[env_ids]]
        if len(ids) > 0:
            if self._evaluation is not None:
                plan = self._evaluation
                mouth, elevation, roll = plan["mouth"][ids], plan["elevation"][ids], plan["roll"][ids]
                standoff = torch.full((len(ids),), plan["conditions"]["standoff_m"], device=self.device)
                delay = torch.full((len(ids),), 15, device=self.device)
            else:
                mouth, elevation, roll, standoff = self._sample_slots(len(ids))
                low, high = insert_cfg.ENGAGE_DELAY_STEPS
                delay = torch.randint(low, high + 1, (len(ids),), device=self.device)
            self._mouth_local[ids], self._elevation[ids], self._roll[ids], self._standoff[ids] = mouth, elevation, roll, standoff
            self._pre_goal[ids] = self._pre_insertion_sphere(mouth, elevation, standoff)
            self._goals.aim(ids, self._pre_goal[ids], roll)
            self._engage_step[ids] = self._goals.traj_steps[ids].long() + delay.long()
