"""The front-right-leg task: UniFP's position/force formulation with the Go2's front-right foot as the end effector.

Lukas's request of 2026-10-19: can the Go2's front-right leg, far stronger than the D1, turn a lever door handle at the
Go2's own body height, trained position-only and force-aware as UniFP is, while the arm holds the wrist camera on what
the foot is doing? This environment is UniFP's (`env.py`) with four changes:

  1. **The end effector is the foot.** The controlled point is the centre of FR's collision sphere; its goal walks
     about the FR thigh joint (`foot_goals.FootGoals`) in UniFP's three goal channels, so no observation width
     changes. The robot stands: velocity commands are zero and the gait clock is pinned, as UniFP pins it standing.
  2. **The arm holds the camera.** The policy has 12 actions, the legs; the D1 is driven to a fixed look pose
     (`leg_cfg.ARM_LOOK_POSE`, jittered per episode) that points the wrist RealSense at the handle region. The
     observation is UniFP's 76 wide, the arm's six action slots zero.
  3. **UniFP's pushes act on the foot**, +-35 N a side rather than +-8 N (`leg_cfg.FOOT_FORCE_RANGE_N`), through a
     400 N/m virtual spring rather than 200 (`leg_cfg.FOOT_FORCE_KP`).
  4. **Two variants**, the plan's P0 and P1 (`cfg.leg_variant`). *position*: the external push is a disturbance the
     foot has to reject; there is no force command, the target is the goal, and neither the critic nor the
     estimator's target carries a force. *force*: UniFP's -- a commanded force, the external one, the target
     `goal + (external + command) / k`, the critic and the estimator given the external force.

With `cfg.handle_fraction > 0`, that share of episodes has a door with a lever handle ahead of the robot
(`handle.LeverHandle`, drawn from `leg_cfg`). The foot is scripted up and over to a point above the estimated bar,
and once it settles there the task layer (`handle.HandleTaskLayer`) runs the turn: a reference down onto the bar and
round its arc past the estimated stop, held, then lifted off -- and, for the force variant, a PI force command along
the arc. The handle's contact force is the foot's external force in those episodes (no pushes), so for the force
variant it is what displaces the target, exactly as UniFP's pushes do. Success is the latch held back for 1 s.
"""
from __future__ import annotations

import math

import torch

from isaaclab.utils import configclass

from unifp_isaaclab import interface

from . import forces, foot_goals, handle, insertion, leg_cfg, leg_rewards, observations, rewards, task_cfg
from .env import Go2D1PosForceEnv
from .env_cfg import Go2D1PosForceEnvCfg

#: Approach stages of a handle episode; a key episode then puts the foot back on its footfall (RETURNING, FOOT_DOWN).
WAITING, LIFTING, OVER, AT_PRE, ENGAGED, MISSED, RETURNING, FOOT_DOWN = 0, 1, 2, 3, 4, 5, 6, 7


@configclass
class Go2D1LegEnvCfg(Go2D1PosForceEnvCfg):
    """UniFP's robot and observation; the legs act, the arm holds the camera, the front-right foot is controlled."""

    action_space = 12
    episode_length_s = leg_cfg.EPISODE_LENGTH_S
    #: "position" (P0) or "force" (P1); see the module docstring.
    leg_variant: str = "force"
    #: Share of episodes with a door handle ahead; the rest are free-space goal walks with pushes.
    handle_fraction: float = 0.0
    #: Iterations over which the push magnitude ramps from zero once `force_start_step` has passed.
    force_ramp_iterations: int = 1000
    force_start_step: int = 500 * task_cfg.NUM_STEPS_PER_ENV
    #: Peak per-axis force, N, of UniFP's external push on the foot (the physical one) and of its command channel.
    #: Both `leg_cfg.FOOT_FORCE_RANGE_N` (+-35) in stage 1. At +-35 a push can reach ~60 N on the raised foot, ~18 N·m
    #: about the edge of the three-foot support against ~14 N·m of gravity: v3's episodes fell from 776 to 622 steps
    #: as the pushes reached full strength. The handle stage runs the physical push at +-20 (`--foot_push_n`).
    foot_push_n: float = leg_cfg.FOOT_FORCE_RANGE_N[1]
    foot_command_n: float = leg_cfg.FOOT_FORCE_RANGE_N[1]
    #: Jitter the arm's held pose per episode (`leg_cfg.ARM_POSE_JITTER`).
    arm_jitter: bool = True
    #: Render the wrist RealSense (the cup pick's camera and saved mount) and a fixed side view, for frames of an
    #: evaluation (`leg_eval --leg_frames`). Off in training. Needs `--enable_cameras`.
    wrist_camera: bool = False
    overview_camera: bool = False
    #: Draw UniFP's goal, target and force markers beside the handle's (viewer and frames).
    unifp_markers: bool = True
    #: "handle" (a lever door handle at body height, 2026-10-19) or "key" (the combiner box's lever key in its cam lock,
    #: 2026-10-21; `leg_cfg`, "the combiner box's key"). `run_unifp_train.py --leg_object key` also sets the four below.
    leg_object: str = "handle"
    #: The arm's held pose: "look" (the wrist camera on the handle region) or "rest" (folded on its stop).
    arm_pose: str = "look"
    #: Start every episode standing settled (`leg_cfg.SETTLED_*`) instead of dropping from UniFP's spawn height.
    settled_spawn: bool = False
    #: Weight of `support_stance` (0: off, as in the handle runs).
    stance_weight: float = 0.0
    #: Draw the head RealSense on its mount (P2Dingo's bracket and Intel's case, `demos/combiner_door/head_camera.py`),
    #: visual only: for the viewer and an evaluation's frames, where the key task has its camera. Off in training.
    head_camera_visual: bool = False
    #: Render the head RealSense (the combiner door demo's model at its mount), for an evaluation's frames of the key.
    head_camera: bool = False
    #: Stances to start from as well as the settled one (a json list of {legs (UniFP order), base_z, roll, pitch}: the
    #: walking policy's stops, `demos/combiner_key/assets/walker_stances.json`), in this share of episodes, the joints
    #: jittered by `spawn_state_jitter` rad. Empty: the settled stance only.
    spawn_states: str = ""
    spawn_states_fraction: float = 0.5
    spawn_state_jitter: float = 0.03
    #: Use `spawn_states` in a frozen evaluation too (a diagnostic: the handover's stances, at rest).
    spawn_states_in_eval: bool = False


class Go2D1LegEnv(Go2D1PosForceEnv):
    cfg: Go2D1LegEnvCfg

    def __init__(self, cfg: Go2D1LegEnvCfg, render_mode: str | None = None, **kwargs):
        if cfg.leg_variant not in ("position", "force"):
            raise ValueError(f"leg_variant must be 'position' or 'force', not {cfg.leg_variant!r}")
        if cfg.leg_object not in ("handle", "key") or cfg.arm_pose not in ("look", "rest"):
            raise ValueError(f"leg_object must be 'handle' or 'key' and arm_pose 'look' or 'rest'")
        super().__init__(cfg, render_mode, **kwargs)
        n, device = self.num_envs, self.device
        self._force_aware = cfg.leg_variant == "force"
        body = self._robot.body_names.index
        self._foot_body, self._knee_body = body(leg_cfg.FOOT_BODY), body(leg_cfg.KNEE_BODY)
        self._base_body = body("base")
        self._wrench_bodies = torch.tensor([self._foot_body, self._knee_body, self._base_body], dtype=torch.int32,
                                           device=device)
        self._foot_offset = torch.as_tensor(handle.FOOT_CENTRE_LOCAL_M, device=device).expand(n, 3).contiguous()
        self._head_offset = torch.as_tensor(leg_cfg.HEAD_POINT_B, device=device).expand(n, 3).contiguous()
        self._wrench_positions = torch.zeros(n, 3, 3, device=device)
        self._wrench_positions[:, 0] = self._foot_offset
        self._wrench_positions[:, 2] = self._head_offset
        self._knee_force_w = torch.zeros(n, 3, device=device)
        self._head_force_w = torch.zeros(n, 3, device=device)

        self._key = cfg.leg_object == "key"
        self._down_goal = leg_cfg.KEY_DOWN_GOAL if self._key else leg_cfg.DOWN_GOAL
        self._goals = foot_goals.FootGoals(n, device, down_goal=self._down_goal,
                                           pitch_range=leg_cfg.KEY_GOAL_PITCH_RANGE if self._key else leg_cfg.GOAL_PITCH_RANGE)
        self._push_cmd = forces.PushSchedule(n, device, force_range=(-cfg.foot_command_n, cfg.foot_command_n),
                                             clear_on_finish=True)
        self._push_ext = forces.PushSchedule(n, device, force_range=(-cfg.foot_push_n, cfg.foot_push_n),
                                             clear_on_finish=False)
        self._push_w = torch.zeros(n, 3, device=device)
        self._gripper_force_kp.fill_(leg_cfg.FOOT_FORCE_KP)

        self._arm_base, self._arm_jitter = ((leg_cfg.ARM_REST_POSE, leg_cfg.ARM_REST_JITTER) if cfg.arm_pose == "rest"
                                            else (leg_cfg.ARM_LOOK_POSE, leg_cfg.ARM_POSE_JITTER))
        self._arm_pose = torch.as_tensor(self._arm_base, device=device).expand(n, 6).clone()
        default = list(interface.DEFAULT_DOF_POS)
        soft = list(task_cfg.soft_joint_pos_limits())
        for j in leg_cfg.FR_JOINTS:
            lower, upper = task_cfg.JOINT_POS_LIMITS[j]
            middle, half = 0.5 * (lower + upper), 0.5 * (upper - lower) * leg_cfg.SOFT_DOF_POS_LIMIT_FR
            soft[j] = (middle - half, middle + half)
        self._soft_limits = torch.as_tensor(soft, device=device)
        del default

        self._handle = handle.LeverHandle(n, device)
        self._layer = handle.HandleTaskLayer(n, device, kp=leg_cfg.FORCE_LAW_KP_N_PER_M, ki=leg_cfg.FORCE_LAW_KI_N_PER_M_S)
        #: The task layer's outer loop on the foot's measured position (the insertion's, F-120): the policy observes
        #: its joints, not its foot, and holds a steady goal centimetres off (5 cm at the pre-contact point in the
        #: stage-2 pilot of 2026-10-20), so the layer integrates the miss into the goal it commands.
        self._correction = insertion.PoseCorrection(
            n, device, tip_gain=leg_cfg.FOOT_CORRECTION_GAIN, tip_max_m=leg_cfg.FOOT_CORRECTION_MAX_M,
            elevation_gain=0.0, elevation_max_rad=0.0, roll_gain=0.0, roll_max_rad=0.0)
        zeros = lambda *shape: torch.zeros(*shape, device=device)
        flags = lambda: torch.zeros(n, dtype=torch.bool, device=device)
        self._handle_env, self._damage_now, self._term_damage, self._term_fallen = flags(), flags(), flags(), flags()
        self._damage_seen = flags()
        self._stage = torch.zeros(n, dtype=torch.long, device=device)
        self._stage_step = torch.zeros(n, dtype=torch.long, device=device)
        self._since_reset = torch.zeros(n, dtype=torch.long, device=device)
        self._start_delay = torch.zeros(n, dtype=torch.long, device=device)
        #: The handle's estimate as the task layer is told it, and its scripted approach (env frame).
        self._est = {name: zeros(n, 3) for name in ("pivot", "u0", "door_n")}
        self._est.update({name: zeros(n) for name in ("reach", "clearance", "theta_end", "max_n", "descent",
                                                      "descend_speed", "turn_speed")})
        self._approach_from, self._waypoint, self._pre = zeros(n, 3), zeros(n, 3), zeros(n, 3)
        self._return_from = zeros(n, 3)
        self._down_for = zeros(n)
        self._est["theta_start"] = zeros(n)
        self._goal_w_held = zeros(n, 3)
        #: Episode bookkeeping for the handle: the latch's longest open spell, when it first opened, peaks, slips.
        self._open_for, self._longest_open = zeros(n), zeros(n)
        self._open_step = torch.full((n,), -1, dtype=torch.long, device=device)
        self._engage_step = torch.full((n,), -1, dtype=torch.long, device=device)
        self._peak_n, self._step_peak, self._max_theta = zeros(n), zeros(n), zeros(n)
        self._off_bar_for = zeros(n)
        self._slipped, self._slip_now = flags(), zeros(n)
        self._door_peak = zeros(n)
        self._done_for = zeros(n)
        #: Force while the latch is held open (sum, count) and the force command's error while pushing (sum sq, count).
        self._open_force_sum, self._open_count = zeros(n), zeros(n)
        self._cmd_err_sq, self._cmd_count = zeros(n), zeros(n)
        self._engage_miss = torch.full((n,), float("nan"), device=device)
        self._evaluation: dict | None = None
        self._trace: list | None = None

        self._weights = leg_cfg.scaled_weights(self._weight_table(handles=cfg.handle_fraction > 0.0))
        self._terms = dict(leg_rewards.TERMS)
        self._episode_sums = {name: zeros(n) for name in self._weights}

    def _weight_table(self, handles: bool) -> dict[str, float]:
        weights = dict(leg_cfg.REWARD_WEIGHTS)
        if handles:
            weights.update(leg_cfg.HANDLE_WEIGHTS)
        if self.cfg.stance_weight > 0.0:
            weights["support_stance"] = self.cfg.stance_weight
        return weights

    # --- acting -----------------------------------------------------------------------------------------------------

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self._last_actions = self._actions
        legs = torch.clip(actions, -interface.CLIP_ACTIONS, interface.CLIP_ACTIONS)
        self._actions = torch.cat((legs, torch.zeros(self.num_envs, 6, device=self.device)), dim=-1)
        self._joint_targets = interface.joint_targets(self._actions)
        self._joint_targets[:, 12:18] = self._arm_pose
        self._step_forces()

    def _push_scale(self) -> float:
        start = self.cfg.force_start_step
        if self.common_step_counter <= start:
            return 0.0
        ramp = max(1, self.cfg.force_ramp_iterations * task_cfg.NUM_STEPS_PER_ENV)
        return min(1.0, (self.common_step_counter - start) / ramp)

    def _step_forces(self) -> None:
        """UniFP's two push channels on the foot in free-space episodes; the task layer's force in handle ones."""
        scale = self._push_scale()
        free = ~self._handle_env
        if scale > 0.0:
            commanded = self._push_cmd.step(self.episode_length_buf) * scale
            external = self._push_ext.step(self.episode_length_buf) * scale
        else:
            commanded = external = torch.zeros(self.num_envs, 3, device=self.device)
        self._push_w = external * free.unsqueeze(-1)
        layer_cmd = interface.quat_rotate_inverse(self._base_yaw_quat(), self._layer.force_cmd)
        command = torch.where(free.unsqueeze(-1), commanded, layer_cmd)
        self._commands[:, interface.CMD_EE_FORCE] = command if self._force_aware else 0.0

    def _foot_state(self) -> tuple[torch.Tensor, torch.Tensor]:
        """The foot sphere's centre (env frame) and velocity (world)."""
        quat = self._robot.data.body_quat_w[:, self._foot_body]
        arm = interface.quat_apply(quat, self._foot_offset)
        pos = self._robot.data.body_pos_w[:, self._foot_body] + arm - self.scene.env_origins
        vel = (self._robot.data.body_lin_vel_w[:, self._foot_body]
               + torch.cross(self._robot.data.body_ang_vel_w[:, self._foot_body], arm, dim=-1))
        return pos, vel

    def _apply_action(self) -> None:
        super()._apply_action()
        force = self._push_w
        if bool(self._handle.active.any()):
            foot, foot_v = self._foot_state()
            knee = self._robot.data.body_pos_w[:, self._knee_body] - self.scene.env_origins
            knee_v = self._robot.data.body_lin_vel_w[:, self._knee_body]
            head = head_v = None
            if self._key:
                quat = self._robot.data.root_quat_w
                arm = interface.quat_apply(quat, self._head_offset)
                head = self._robot.data.root_pos_w + arm - self.scene.env_origins
                head_v = self._robot.data.root_lin_vel_w + torch.cross(self._robot.data.root_ang_vel_w, arm, dim=-1)
            contact = self._handle.step(foot, foot_v, interface.SIM_DT, knee=knee, knee_v=knee_v, head=head,
                                        head_v=head_v, head_r=leg_cfg.HEAD_RADIUS_M)
            force = force + contact
            self._knee_force_w = self._handle.knee_force
            self._head_force_w = self._handle.head_force
        else:
            self._knee_force_w = torch.zeros_like(force)
            self._head_force_w = torch.zeros_like(force)
        self._ee_force_w = force
        self._write_wrench()

    def _write_wrench(self) -> None:
        """The foot's force at the foot sphere's centre, the knee's at the calf's origin, the head's at its sphere's
        centre on the base, each in its link's frame."""
        foot_quat = self._robot.data.body_quat_w[:, self._foot_body]
        knee_quat = self._robot.data.body_quat_w[:, self._knee_body]
        local = torch.stack((interface.quat_rotate_inverse(foot_quat, self._ee_force_w),
                             interface.quat_rotate_inverse(knee_quat, self._knee_force_w),
                             interface.quat_rotate_inverse(self._robot.data.root_quat_w, self._head_force_w)), dim=1)
        self._robot.permanent_wrench_composer.set_forces_and_torques(
            forces=local.contiguous(), positions=self._wrench_positions, body_ids=self._wrench_bodies)
        self._wrench_active = True

    # --- frames -----------------------------------------------------------------------------------------------------

    def _tip_pos(self) -> torch.Tensor:
        return self._foot_state()[0]

    def _ground_under_base(self) -> torch.Tensor:
        base = self._robot.data.root_pos_w - self.scene.env_origins
        return torch.cat((base[:, :2], torch.zeros_like(base[:, 2:3])), dim=1)

    def _goal_centre(self) -> torch.Tensor:
        centre = torch.as_tensor(leg_cfg.GOAL_CENTRE, device=self.device).expand(self.num_envs, 3)
        return self._ground_under_base() + interface.quat_apply(self._base_yaw_quat(), centre)

    def _goal_world(self) -> torch.Tensor:
        return self._goal_centre() + interface.quat_apply(self._base_yaw_quat(), interface.sphere2cart(self._goals.current))

    def _to_sphere(self, point_env: torch.Tensor) -> torch.Tensor:
        """An env-frame point as goal-sphere coordinates about the base's current goal centre."""
        return interface.cart2sphere(interface.quat_rotate_inverse(self._base_yaw_quat(), point_env - self._goal_centre()))

    # --- the handle episodes ----------------------------------------------------------------------------------------

    def _advance_approach(self) -> None:
        """The scripted approach: wait, lift to a waypoint short of and above the pre-contact point, slide over onto it,
        hold there until the foot settles (then the layer starts) or the deadline passes (then it is missed)."""
        h = self._handle_env
        if not bool(h.any()):
            return
        foot, foot_v = self._foot_state()
        since = self._since_reset - self._stage_step
        smooth = lambda t: t * t * (3 - 2 * t)
        # waiting -> lifting
        go = h & (self._stage == WAITING) & (self._since_reset >= self._start_delay)
        if bool(go.any()):
            self._approach_from[go] = foot[go]
            self._stage[go], self._stage_step[go] = LIFTING, self._since_reset[go]
            since = self._since_reset - self._stage_step
        lift_steps, over_steps = round(leg_cfg.LIFT_S / interface.POLICY_DT), round(leg_cfg.OVER_S / interface.POLICY_DT)
        t_lift = (since.float() / lift_steps).clamp(0.0, 1.0)
        t_over = (since.float() / over_steps).clamp(0.0, 1.0)
        lifting = h & (self._stage == LIFTING)
        over = h & (self._stage == OVER)
        goal = self._goal_w_held.clone()
        stance = self._ground_under_base() + interface.quat_apply(
            self._base_yaw_quat(), torch.as_tensor(self._down_goal, device=self.device).expand(self.num_envs, 3))
        goal = torch.where((h & (self._stage == WAITING)).unsqueeze(-1), stance, goal)
        # The lift in two legs: straight up off the floor to `LIFT_FIRST_HEIGHT_M` first, then to the waypoint behind and
        # above the bar (since 2026-10-20 10:45: on a straight line the foot, which has to swing forward to rise, cut
        # the corner under the bar and jammed there or kicked it).
        first = self._approach_from.clone()
        first[:, 2] = leg_cfg.LIFT_FIRST_HEIGHT_M
        split = leg_cfg.LIFT_FIRST_SHARE
        leg1 = torch.lerp(self._approach_from, first, smooth((t_lift / split).clamp(max=1.0)).unsqueeze(-1))
        leg2 = torch.lerp(first, self._waypoint, smooth(((t_lift - split) / (1.0 - split)).clamp(min=0.0)).unsqueeze(-1))
        lift_goal = torch.where((t_lift < split).unsqueeze(-1), leg1, leg2)
        goal = torch.where(lifting.unsqueeze(-1), lift_goal, goal)
        goal = torch.where(over.unsqueeze(-1), torch.lerp(self._waypoint, self._pre, smooth(t_over).unsqueeze(-1)), goal)
        at_pre = h & (self._stage == AT_PRE)
        goal = torch.where(at_pre.unsqueeze(-1), self._pre, goal)
        self._goal_w_held = torch.where((h & (self._stage <= AT_PRE)).unsqueeze(-1), goal, self._goal_w_held)
        if self._key:
            self._advance_return(foot, stance, smooth)
        to_over = lifting & (t_lift >= 1.0)
        self._stage = torch.where(to_over, torch.full_like(self._stage, OVER), self._stage)
        self._stage_step = torch.where(to_over, self._since_reset, self._stage_step)
        to_pre = over & (t_over >= 1.0)
        self._stage = torch.where(to_pre, torch.full_like(self._stage, AT_PRE), self._stage)
        self._stage_step = torch.where(to_pre, self._since_reset, self._stage_step)
        # at the pre-contact point: engage once settled
        waiting = h & (self._stage == AT_PRE)
        if bool(waiting.any()):
            waited = self._since_reset - self._stage_step
            miss = (foot - self._pre).norm(dim=-1)
            settled = foot_v.norm(dim=-1) < leg_cfg.ENGAGE_SETTLE_SPEED_M_S
            deadline = waited >= leg_cfg.ENGAGE_MAX_WAIT_STEPS
            # In training the layer starts at the deadline wherever the foot is, so a poor approach still teaches the press
            # (since 2026-10-20 06:20: with the deadline's 4 cm limit ~99% of stage-2 approaches ended unengaged and
            # taught nothing); a frozen evaluation keeps the limit and scores the rest as missed.
            near = leg_cfg.ENGAGE_NEAR_M if self._evaluation is not None else float("inf")
            due = waiting & (waited >= 5) & ((settled & (miss < leg_cfg.ENGAGE_CONVERGED_M)) | (deadline & (miss < near)))
            late = waiting & ~due & deadline
            self._stage = torch.where(late, torch.full_like(self._stage, MISSED), self._stage)
            if bool(due.any()):
                ids = due.nonzero(as_tuple=False).flatten()
                e = self._est
                count = len(ids)
                self._layer.start(ids, pivot=e["pivot"][ids], u0=e["u0"][ids], reach=e["reach"][ids],
                                  clearance=e["clearance"][ids], theta_end=e["theta_end"][ids],
                                  descent=e["descent"][ids], descend_speed=e["descend_speed"][ids],
                                  turn_speed=e["turn_speed"][ids], max_n=e["max_n"][ids],
                                  touch_n=torch.full((count,), leg_cfg.TOUCH_N, device=self.device),
                                  hold_s=torch.full((count,), leg_cfg.HOLD_S, device=self.device),
                                  release_s=torch.full((count,), leg_cfg.RELEASE_S, device=self.device),
                                  force_law=torch.full((count,), self._force_aware, dtype=torch.bool, device=self.device),
                                  touch_s=torch.full((count,), leg_cfg.TOUCH_HOLD_S, device=self.device),
                                  theta_start=e["theta_start"][ids])
                self._stage[ids] = ENGAGED
                self._engage_step[ids] = self._since_reset[ids]
                self._engage_miss[ids] = miss[ids]

    def _advance_return(self, foot: torch.Tensor, stance: torch.Tensor, smooth) -> None:
        """A key episode's end: once the layer has lifted off, the foot goes back to the waypoint and then down onto its
        footfall, where the robot stands on four legs again."""
        h = self._handle_env
        done = h & (self._stage == ENGAGED) & (self._layer.phase == handle.DONE)
        if bool(done.any()):
            self._return_from[done] = self._goal_w_held[done]
            self._stage[done], self._stage_step[done] = RETURNING, self._since_reset[done]
        since = (self._since_reset - self._stage_step).float()
        up_steps, down_steps = leg_cfg.RETURN_UP_S / interface.POLICY_DT, leg_cfg.RETURN_DOWN_S / interface.POLICY_DT
        t_up = (since / up_steps).clamp(0.0, 1.0)
        t_down = ((since - up_steps) / down_steps).clamp(0.0, 1.0)
        back = torch.lerp(self._return_from, self._waypoint, smooth(t_up).unsqueeze(-1))
        goal = torch.where((t_up < 1.0).unsqueeze(-1), back, torch.lerp(self._waypoint, stance, smooth(t_down).unsqueeze(-1)))
        returning = h & (self._stage == RETURNING)
        down = h & (self._stage == FOOT_DOWN)
        self._goal_w_held = torch.where(returning.unsqueeze(-1), goal, self._goal_w_held)
        self._goal_w_held = torch.where(down.unsqueeze(-1), stance, self._goal_w_held)
        landed = returning & (t_down >= 1.0)
        self._stage = torch.where(landed, torch.full_like(self._stage, FOOT_DOWN), self._stage)
        self._down_for = torch.where(down, self._down_for + interface.POLICY_DT, self._down_for)

    def _run_layer(self) -> None:
        engaged = self._handle_env & (self._stage == ENGAGED)
        if not bool(engaged.any()):
            return
        foot, _ = self._foot_state()
        goal, _ = self._layer.step(foot, interface.POLICY_DT)
        self._goal_w_held = torch.where(engaged.unsqueeze(-1), goal, self._goal_w_held)

    def _correct_foot(self) -> None:
        """The outer loop: the whole miss while the foot comes over and holds the pre-contact point; while pressing
        or holding, only its part across the way the foot pushes (along it, the miss is the handle's resistance)."""
        h = self._handle_env
        if not bool(h.any()):
            return
        foot, _ = self._foot_state()
        dt = interface.POLICY_DT
        # Only once the goal stands still at the pre-contact point (while it slides, the miss is the policy's lag, and
        # integrating it wound the offset up 3-4 cm before the foot arrived), and never while the door or the bar
        # holds the foot back (the miss is then the obstacle's, and the loop would push the foot into it).
        touching = (self._handle.door_n + self._handle.knee_door_n > 0.0) | self._handle.on_bar
        free = h & (self._stage == AT_PRE) & ~touching
        self._correction.update_tip(free, self._goal_w_held, foot, dt)
        layer = self._layer
        pushing = h & (self._stage == ENGAGED) & ((layer.phase == handle.PRESSING) | (layer.phase == handle.HOLDING))
        pushing &= (self._handle.door_n + self._handle.knee_door_n) <= 0.0
        self._correction.update_tip(pushing, self._goal_w_held, foot, dt, across=layer.tangent)

    def _hold_goals(self) -> None:
        held = self._handle_env.clone()
        self._goals.hold_mask = held
        if bool(held.any()):
            corrected = (self._stage >= AT_PRE) & (self._stage <= ENGAGED)
            commanded = self._goal_w_held + self._correction.offset * corrected.unsqueeze(-1)
            self._goals.hold_goal = torch.where(held.unsqueeze(-1), self._to_sphere(commanded), self._goals.hold_goal)

    def _bookkeep_handle(self) -> None:
        dt = interface.POLICY_DT
        engaged = self._handle_env & (self._stage == ENGAGED)
        layer = self._layer
        pushing = engaged & ((layer.phase == handle.PRESSING) | (layer.phase == handle.HOLDING))
        opened = self._handle.latch_open() & engaged
        self._open_for = torch.where(opened, self._open_for + dt, torch.zeros_like(self._open_for))
        self._longest_open = torch.maximum(self._longest_open, self._open_for)
        self._open_step = torch.where(opened & (self._open_step < 0), self._since_reset, self._open_step)
        self._step_peak = self._handle.take_peak() * self._handle.active
        self._damage_now |= self._step_peak > leg_cfg.DAMAGE_N
        self._peak_n = torch.maximum(self._peak_n, self._step_peak)
        self._max_theta = torch.maximum(self._max_theta, self._handle.theta * self._handle.active)
        self._door_peak = torch.maximum(self._door_peak, self._handle.door_n)
        contact_n = self._handle.force_on_foot.norm(dim=-1)
        self._open_force_sum += contact_n * opened
        self._open_count += opened.float()
        # A slip-off: the bar touched while pressing, then left for longer than `SLIP_S` before the release.
        off = pushing & self._handle.touched & ~self._handle.on_bar
        self._off_bar_for = torch.where(off, self._off_bar_for + dt, torch.zeros_like(self._off_bar_for))
        slip = (self._off_bar_for >= leg_cfg.SLIP_S) & ~self._slipped
        self._slip_now = slip.float()
        self._slipped |= slip
        if self._force_aware:
            # The force the foot puts on the handle along the command's direction, against the command, while on the arc.
            cmd = layer.force_cmd
            magnitude = cmd.norm(dim=-1)
            on_arc = pushing & (magnitude > 0.0) & self._handle.on_bar
            applied = (-self._handle.force_on_foot * cmd).sum(-1) / magnitude.clamp(min=1e-6)
            self._cmd_err_sq += torch.square(applied - magnitude) * on_arc
            self._cmd_count += on_arc.float()
        finished = engaged & (layer.phase == handle.DONE)
        self._done_for = torch.where(finished, self._done_for + dt, self._done_for)

    # --- the viewer -----------------------------------------------------------------------------------------------------

    def _setup_scene(self):
        super()._setup_scene()
        if self.cfg.head_camera_visual:
            import isaaclab.sim as sim_utils

            from demos.combiner_door import head_camera

            base = "/World/envs/env_.*/Robot/base"
            bracket = sim_utils.UsdFileCfg(usd_path=head_camera.build_bracket_usd(head_camera.ROOT / "generated/combiner_door"))
            bracket.func(f"{base}/head_rs_bracket", bracket)
            case = sim_utils.UsdFileCfg(usd_path=head_camera.build_case_usd(head_camera.ROOT / "generated/pick_demo"))
            case.func(f"{base}/head_rs_body", case, translation=head_camera.COLOUR_SENSOR_BASE_M,
                      orientation=head_camera.OPTICAL_QUAT_BASE_WXYZ)
        if self.cfg.wrist_camera or self.cfg.overview_camera or self.cfg.head_camera:
            from isaaclab.sensors import Camera

            from . import leg_vis
        if self.cfg.wrist_camera:
            self._wrist_cam = Camera(leg_vis.wrist_camera_cfg())
            self.scene.sensors["wrist_cam"] = self._wrist_cam
        if self.cfg.overview_camera:
            target = leg_vis.KEY_OVERVIEW_TARGET if self.cfg.leg_object == "key" else leg_vis.OVERVIEW_TARGET
            eye = leg_vis.KEY_OVERVIEW_EYE if self.cfg.leg_object == "key" else leg_vis.OVERVIEW_EYE
            self._overview_cam = Camera(leg_vis.overview_camera_cfg(eye, target))
            self.scene.sensors["overview_cam"] = self._overview_cam
        if self.cfg.head_camera:
            from isaaclab.sensors import TiledCamera

            from demos.combiner_door import head_camera

            self._head_cam = TiledCamera(head_camera.camera_cfg(head_camera.camera_model()))
            self.scene.sensors["head_cam"] = self._head_cam

    def _set_debug_vis_impl(self, enable: bool) -> None:
        if self.cfg.unifp_markers:
            super()._set_debug_vis_impl(enable)
        if enable and not hasattr(self, "_handle_markers"):
            from .leg_vis import HandleMarkers

            self._handle_markers = HandleMarkers("/Visuals/leg_handle", self.num_envs, str(self.device))
        if hasattr(self, "_handle_markers"):
            self._handle_markers.set_visibility(enable)

    def _debug_vis_callback(self, event) -> None:
        if hasattr(self, "_markers"):
            super()._debug_vis_callback(event)
        if hasattr(self, "_handle_markers"):
            self._handle_markers.update(self._handle, self.scene.env_origins)

    # --- the step -----------------------------------------------------------------------------------------------------

    def _get_rewards(self) -> torch.Tensor:
        self._since_reset += 1
        self._advance_approach()
        self._run_layer()
        self._correct_foot()
        self._hold_goals()
        self._commands[:, :3] = 0.0
        self._gait_phase.zero_()
        self._commands[:, interface.CMD_EE_RADIUS:interface.CMD_EE_YAW + 1] = self._goals.step()
        self._bookkeep_handle()
        if self._trace is not None:
            self._record_trace()

        state = self._task_state()
        total = torch.zeros(self.num_envs, device=self.device)
        for name, weight in self._weights.items():
            value = self._terms[name](state) * weight
            total += value
            self._episode_sums[name] += value
        self._last_dof_vel = state.dof_vel.clone()
        return total

    def _task_state(self) -> rewards.TaskState:
        dof_pos, dof_vel = self._dof()
        yaw = self._base_yaw_quat()
        feet_forces = self._contact_forces(self._feet_contacts)
        contact = (feet_forces[:, :, 2] > 5.0).float()
        zero3 = torch.zeros(self.num_envs, 3, device=self.device)
        measured = self._ee_force_w if self._force_aware else zero3
        command = self._commands[:, interface.CMD_EE_FORCE] if self._force_aware else zero3
        feet_vel = self._robot.data.body_lin_vel_w[:, self._feet_bodies]
        state = rewards.TaskState(
            base_lin_vel_b=self._robot.data.root_lin_vel_b, base_ang_vel_b=self._robot.data.root_ang_vel_b,
            root_pos_w=self._robot.data.root_pos_w - self.scene.env_origins,
            dof_pos=dof_pos, dof_vel=dof_vel, last_dof_vel=self._last_dof_vel,
            torques=self._robot.data.applied_torque[:, self._order],
            actions=self._actions, last_actions=self._last_actions,
            commands=self._commands, gait_phase=self._gait_phase,
            ref_leg_pos=torch.as_tensor(interface.DEFAULT_DOF_POS[:12], device=self.device).expand(self.num_envs, 12),
            stance_mask=self._stance_mask(), contact_mask=contact,
            feet_pos_w=self._robot.data.body_pos_w[:, self._feet_bodies] - self.scene.env_origins.unsqueeze(1),
            feet_contact_forces=feet_forces, feet_air_time=self._feet_air_time,
            penalised_contact_forces=self._contact_forces(self._penalised_contacts),
            ee_pos_w=self._tip_pos(), ee_goal_w=self._goal_world(),
            thigh_pos_w=self._robot.data.body_pos_w[:, self._thigh_bodies] - self.scene.env_origins.unsqueeze(1),
            feet_vel_w=feet_vel, last_contacts=self._last_contacts, base_yaw_quat=yaw,
            ee_force_measured=measured, ee_force_cmd=command,
            base_force_measured=self._base_force_w, base_force_cmd=self._commands[:, interface.CMD_BASE_FORCE],
            gripper_force_kp=self._gripper_force_kp, base_force_kd=self._base_force_kd)
        state.projected_gravity_b = self._robot.data.projected_gravity_b
        state.goal_down = self._goals.goal_local()[:, 2] < 0.05
        state.soft_limits = self._soft_limits
        state.support_feet_vel_w = feet_vel[:, list(leg_cfg.SUPPORT_FEET)]
        engaged = self._handle_env & (self._stage == ENGAGED)
        layer = self._layer
        pushing = engaged & ((layer.phase == handle.PRESSING) | (layer.phase == handle.HOLDING))
        state.handle_pushing = pushing.float()
        # Measured from the lever's start stop (0 for a handle, -45 degrees for the key), so progress is 0..1 either way.
        state.handle_theta = self._handle.theta - self._handle.theta_min
        state.handle_latch = self._handle.theta_latch - self._handle.theta_min
        state.handle_open = self._handle.latch_open().float()
        state.handle_on_bar = self._handle.on_bar.float()
        state.handle_slip = self._slip_now
        # The door's overlap with the foot and the knee, cm: geometry, not force, so the position variant is not given a
        # force term (the plan's P0 rule); the penalty contact makes it proportional to the force anyway.
        state.handle_door_cm = 100.0 * (self._handle.door_n + self._handle.knee_door_n) / handle.CONTACT_K_N_PER_M \
            * self._handle_env
        # Overlap with the bar or the door before the layer has started, cm: the approach should touch nothing.
        approaching = (self._handle_env & (self._stage < ENGAGED)).float()
        state.handle_approach_cm = 100.0 * (self._handle.bar_n + self._handle.door_n + self._handle.knee_door_n) \
            / handle.CONTACT_K_N_PER_M * approaching
        return state

    def _stance_mask(self) -> torch.Tensor:
        mask = torch.ones(self.num_envs, 4, device=self.device)
        mask[:, leg_cfg.FR_FOOT_INDEX] = (self._goals.goal_local()[:, 2] < 0.05).float()
        return mask

    def _get_observations(self) -> dict:
        dof_pos, dof_vel = self._dof()
        actor = interface.single_obs(
            self._robot.data.root_quat_w, self._robot.data.root_ang_vel_b,
            dof_pos, dof_vel, self._actions, self._gait_phase, self._commands)
        actor = torch.clip(actor, -interface.CLIP_OBSERVATIONS, interface.CLIP_OBSERVATIONS)
        yaw = self._base_yaw_quat()
        feet_forces = self._contact_forces(self._feet_contacts)
        measured = self._ee_force_w if self._force_aware else torch.zeros_like(self._ee_force_w)
        critic = observations.privileged_obs(
            base_lin_vel_b=self._robot.data.root_lin_vel_b,
            ee_pos_w=self._tip_pos(), ee_goal_w=self._goal_world(),
            goal_centre_w=self._goal_centre(), base_yaw_quat=yaw,
            ee_force_w=measured, base_force_w=self._base_force_w,
            gripper_force_kp=self._gripper_force_kp,
            ee_force_cmd=self._commands[:, interface.CMD_EE_FORCE],
            leg_ref_diff=dof_pos[:, :12] - torch.as_tensor(interface.DEFAULT_DOF_POS[:12], device=self.device),
            mass_params=self._privileged_extra(),
            friction=torch.ones(self.num_envs, 1, device=self.device),
            motor_strength=torch.ones(self.num_envs, interface.NUM_ACTIONS, device=self.device),
            stance_mask=self._stance_mask(),
            contact_mask=(feet_forces[:, :, 2] > 5.0).float(),
            projected_gravity=self._robot.data.projected_gravity_b,
            base_ang_vel_b=self._robot.data.root_ang_vel_b,
            dof_pos=dof_pos, dof_vel=dof_vel, actions=self._actions,
            gait_phase=self._gait_phase, commands=self._commands)
        critic = torch.clip(critic, -interface.CLIP_OBSERVATIONS, interface.CLIP_OBSERVATIONS)
        return {"policy": self._actor_history.append(actor), "critic": self._critic_history.append(critic),
                "estimates": critic[:, :observations.NUM_ESTIMATES]}

    def _privileged_extra(self) -> torch.Tensor:
        """The handle as the critic sees it, in the 22-wide `mass_params` block; its contact force only for the
        force variant (the plan: a force term in a critic only where the matching actor has one)."""
        hd, layer = self._handle, self._layer
        on = (self._handle_env & hd.active).float().unsqueeze(-1)
        yaw = self._base_yaw_quat()
        out = torch.zeros(self.num_envs, 22, device=self.device)
        out[:, 0:1] = on
        out[:, 1:2] = (self._stage == ENGAGED).float().unsqueeze(-1) * on
        out[:, 2:3] = (hd.theta / hd.theta_max.clamp(min=1e-3)).unsqueeze(-1) * on
        out[:, 3:4] = (hd.omega * 0.1).unsqueeze(-1) * on
        out[:, 4:5] = hd.latch_open().float().unsqueeze(-1) * on
        out[:, 5:6] = hd.on_bar.float().unsqueeze(-1) * on
        out[:, 6:7] = (layer.phase.float() / 4.0).unsqueeze(-1) * on
        out[:, 7:8] = (layer.theta_ref - hd.theta).clamp(-1.0, 1.0).unsqueeze(-1) * on
        out[:, 8:11] = interface.quat_rotate_inverse(yaw, hd.pivot - self._est["pivot"]) * 50.0 * on
        # What it takes to turn it: a handle's spring at its stop, a key's lock friction and cam.
        rating = torch.where(hd.pad, hd.friction + hd.detent, hd.spring_nm(hd.theta_max))
        out[:, 11:12] = (rating * 0.25).unsqueeze(-1) * on
        out[:, 12:13] = (hd.preload * 0.5).unsqueeze(-1) * on
        out[:, 13:14] = hd.theta_max.unsqueeze(-1) * on
        out[:, 14:15] = hd.theta_latch.unsqueeze(-1) * on
        out[:, 15:16] = hd.mu.unsqueeze(-1) * on
        out[:, 16:17] = (hd.length * 5.0).unsqueeze(-1) * on
        foot, _ = self._foot_state()
        out[:, 17:18] = (((foot - hd.door) * hd.n).sum(-1) * 10.0).clamp(-3, 3).unsqueeze(-1) * on
        out[:, 18:19] = self._handle_env.float().unsqueeze(-1)
        if self._force_aware:
            out[:, 19:22] = interface.quat_rotate_inverse(yaw, hd.force_on_foot) * 0.02 * on
        return out

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        roll_pitch = interface.body_roll_pitch(self._robot.data.root_quat_w)
        fallen = (roll_pitch[:, 1].abs() > 1.0) | (roll_pitch[:, 0].abs() > 0.8)
        fallen |= (self._robot.data.root_pos_w[:, 2] - self.scene.env_origins[:, 2]) < leg_cfg.MIN_BASE_HEIGHT_M
        damaged = self._damage_now.clone()
        # A frozen evaluation ends an episode at the damage force, as the plan's common failure limit. Training only
        # records it (since 2026-10-20 10:10): terminating there taught the position-only policy to keep its foot on the
        # floor in handle episodes (engaged 28-40 cm off) and the force-aware one little but avoidance.
        self._damage_seen |= damaged
        self._term_damage = self._damage_seen.clone()
        if self._evaluation is None:
            damaged = torch.zeros_like(damaged)
        self._damage_now[:] = False
        timed_out = self.episode_length_buf >= self.max_episode_length - 1
        if self._evaluation is None:
            # A handle episode is over once the foot is off the handle again, or the approach missed.
            ended = (self._down_for > 1.0) if self._key else (self._done_for > 0.5)
            timed_out |= self._handle_env & (ended | (self._stage == MISSED))
        self._term_fallen = fallen.clone()
        return fallen | damaged, timed_out

    def _resample_velocity_commands(self, env_ids: torch.Tensor) -> None:
        self._commands[env_ids, :3] = 0.0

    # --- resetting ----------------------------------------------------------------------------------------------------

    def set_evaluation(self, plan: dict) -> None:
        """Fix every episode's handle and estimate (`leg_eval.build_plan`). Call before `reset()`."""
        move = lambda v: (v.to(self.device) if torch.is_tensor(v)
                          else {k: move(x) for k, x in v.items()} if isinstance(v, dict) else v)
        self._evaluation = move(plan)
        self._weights = leg_cfg.scaled_weights(self._weight_table(handles=True))
        self._episode_sums = {name: torch.zeros(self.num_envs, device=self.device) for name in self._weights}

    def start_trace(self) -> None:
        self._trace = []

    def _record_trace(self) -> None:
        foot, _ = self._foot_state()
        self._trace.append({
            "theta": self._handle.theta.clone(), "theta_ref": self._layer.theta_ref.clone(),
            "theta_meas": self._layer.theta_meas.clone(), "phase": self._layer.phase.clone(),
            "stage": self._stage.clone(), "force": self._handle.force_on_foot.clone(),
            "force_cmd": self._layer.force_cmd.clone(), "foot": foot.clone(), "goal": self._goal_world().clone(),
            "on_bar": self._handle.on_bar.clone(), "peak": self._step_peak.clone(),
            "base": (self._robot.data.root_pos_w - self.scene.env_origins).clone(),
            "rp": interface.body_roll_pitch(self._robot.data.root_quat_w).clone(),
        })

    def _draw_handles(self, ids: torch.Tensor) -> dict[str, torch.Tensor]:
        """Handles for `ids` in the yaw frame of a robot standing at the origin facing +x."""
        count, device = len(ids), self.device
        rand = lambda *shape: torch.rand(*shape, device=device)
        span = lambda r, n=count: r[0] + rand(n) * (r[1] - r[0])
        sym = lambda a, n=count: (2 * rand(n) - 1) * a
        out = {"door_ahead": span(leg_cfg.DOOR_AHEAD_M), "door_yaw": span(leg_cfg.DOOR_YAW_RAD),
               "height": span(leg_cfg.SPINDLE_HEIGHT_M), "contact_side": span(leg_cfg.CONTACT_SIDE_M),
               "left": rand(count) < leg_cfg.LEVER_LEFT_PROB, "length": span(leg_cfg.LEVER_LENGTH_M),
               "radius": span(leg_cfg.LEVER_RADIUS_M), "standoff": span(leg_cfg.LEVER_STANDOFF_M),
               "inertia": span(leg_cfg.LEVER_INERTIA_KG_M2), "rest_nm": span(leg_cfg.REST_NM),
               "full_nm": span(leg_cfg.FULL_NM), "damping": span(leg_cfg.SPINDLE_DAMPING),
               "friction": span(leg_cfg.SPINDLE_FRICTION_NM), "theta_max": span(leg_cfg.THETA_MAX_RAD),
               "latch_fraction": span(leg_cfg.LATCH_FRACTION), "mu": span(leg_cfg.FOOT_MU),
               "err_side": sym(leg_cfg.EST_ERROR_SIDE_M), "err_up": sym(leg_cfg.EST_ERROR_UP_M),
               "err_depth": sym(leg_cfg.EST_ERROR_DEPTH_M), "err_length": sym(leg_cfg.EST_ERROR_LENGTH),
               "err_theta": sym(leg_cfg.EST_ERROR_THETA), "err_rating": sym(leg_cfg.EST_ERROR_RATING),
               "descent": span(leg_cfg.DESCENT_M), "descend_speed": span(leg_cfg.DESCEND_SPEED_M_S),
               "turn_speed": span(leg_cfg.TURN_SPEED_RAD_S)}
        out["full_nm"] = torch.maximum(out["full_nm"], out["rest_nm"] + 0.2)
        return out

    def _place_handles(self, ids: torch.Tensor, d: dict[str, torch.Tensor]) -> None:
        """Mount the drawn handles in the env frame (the robot has just been reset to the origin, facing +x), and give
        the task layer its estimate of them and the scripted approach."""
        count, device = len(ids), self.device
        z = torch.zeros(count, 3, device=device)
        z[:, 2] = 1.0
        psi = d["door_yaw"]
        normal = torch.stack((-torch.cos(psi), -torch.sin(psi), torch.zeros_like(psi)), dim=-1)
        along = torch.stack((-torch.sin(psi), torch.cos(psi), torch.zeros_like(psi)), dim=-1)    # the robot's left
        u0 = torch.where(d["left"].unsqueeze(-1), along, -along)
        reach_true = leg_cfg.REACH_FRACTION * d["length"]
        # The pivot's position along the door, chosen so the contact point lands `contact_side` to the side.
        s = (d["contact_side"] - d["standoff"] * normal[:, 1] - reach_true * u0[:, 1]) / along[:, 1]
        door = torch.stack((d["door_ahead"], torch.zeros_like(psi), d["height"]), dim=-1) + s.unsqueeze(-1) * along
        theta_max = d["theta_max"]
        self._handle.place(ids, door=door, normal=normal, u0=u0, standoff=d["standoff"], length=d["length"],
                           inner=torch.full((count,), leg_cfg.LEVER_INNER_M, device=device), radius=d["radius"],
                           inertia=d["inertia"], rest_nm=d["rest_nm"], full_nm=d["full_nm"], damping=d["damping"],
                           friction=d["friction"], theta_max=theta_max, theta_latch=d["latch_fraction"] * theta_max,
                           mu=d["mu"])
        pivot = self._handle.pivot[ids]
        e = self._est
        # The layer aims `AIM_TOWARD_ROBOT_M` off the bar's centreline toward the robot, away from the door's face.
        e["pivot"][ids] = pivot + d["err_side"].unsqueeze(-1) * along + d["err_up"].unsqueeze(-1) * z \
            + (d["err_depth"] + leg_cfg.AIM_TOWARD_ROBOT_M).unsqueeze(-1) * normal
        e["u0"][ids], e["door_n"][ids] = u0, normal
        length_est = d["length"] * (1.0 + d["err_length"])
        e["reach"][ids] = leg_cfg.REACH_FRACTION * length_est
        e["clearance"][ids] = handle.FOOT_RADIUS_M + d["radius"]
        e["theta_end"][ids] = theta_max * (1.0 + d["err_theta"]) + leg_cfg.OVERDRIVE_RAD
        rated = (self._handle.preload[ids] + self._handle.stiffness[ids] * theta_max) * (1.0 + d["err_rating"])
        e["max_n"][ids] = (leg_cfg.MAX_MARGIN * rated / e["reach"][ids]).clamp(*leg_cfg.MAX_N_LIMITS)
        for name in ("descent", "descend_speed", "turn_speed"):
            e[name][ids] = d[name]
        rest = handle.foot_on_bar(e["pivot"][ids], u0, torch.zeros(count, device=device), e["reach"][ids],
                                  e["clearance"][ids])
        self._pre[ids] = rest + d["descent"].unsqueeze(-1) * z
        # Above the pre-contact point and a little toward the robot, so the foot comes down onto it rather than in
        # toward the door (until 2026-10-20 the waypoint was 0.10 m toward the robot and 0.03 m up, and the zero-shot
        # foot met the door's face or the bar on the way in: v3 `model_1000`, 42 of 144 past the damage force).
        self._waypoint[ids] = self._pre[ids] + leg_cfg.WAYPOINT_BACK_M * normal + leg_cfg.WAYPOINT_UP_M * z

    def _draw_keys(self, ids: torch.Tensor) -> dict[str, torch.Tensor]:
        """Keys for `ids` in the yaw frame of a robot standing at the origin facing +x (`leg_cfg`, the key's)."""
        count, device = len(ids), self.device
        rand = lambda *shape: torch.rand(*shape, device=device)
        span = lambda r: r[0] + rand(count) * (r[1] - r[0])
        sym = lambda a: (2 * rand(count) - 1) * a
        c = leg_cfg
        return {"door_ahead": span(c.KEY_DOOR_AHEAD_M), "door_yaw": span(c.KEY_DOOR_YAW_RAD),
                "height": c.KEY_AXIS_HEIGHT_M + sym(c.KEY_AXIS_HEIGHT_JITTER_M), "axis_side": span(c.KEY_AXIS_SIDE_M),
                "inertia": span(c.KEY_INERTIA_KG_M2), "friction": span(c.KEY_FRICTION_NM),
                "detent": span(c.KEY_DETENT_NM), "damping": span(c.KEY_DAMPING), "mu": span(c.KEY_MU),
                "reach": span(c.KEY_REACH_M), "err_side": sym(c.KEY_EST_ERROR_SIDE_M), "err_up": sym(c.KEY_EST_ERROR_UP_M),
                "err_depth": sym(c.KEY_EST_ERROR_DEPTH_M), "err_rating": sym(c.KEY_EST_ERROR_RATING),
                "descent": span(c.KEY_DESCENT_M), "descend_speed": span(c.DESCEND_SPEED_M_S),
                "turn_speed": span(c.TURN_SPEED_RAD_S)}

    def _place_keys(self, ids: torch.Tensor, d: dict[str, torch.Tensor]) -> None:
        """Mount the drawn keys in the env frame (the robot just reset to the origin facing +x), and give the task layer
        its estimate and the approach: the key's shaft on the door's face `axis_side` to the side, pointing to the
        robot's right when level, 45 degrees up to start."""
        count, device, c = len(ids), self.device, leg_cfg
        z = torch.zeros(count, 3, device=device)
        z[:, 2] = 1.0
        psi = d["door_yaw"]
        normal = torch.stack((-torch.cos(psi), -torch.sin(psi), torch.zeros_like(psi)), dim=-1)
        along = torch.stack((-torch.sin(psi), torch.cos(psi), torch.zeros_like(psi)), dim=-1)    # the robot's left
        s = d["axis_side"] / along[:, 1]
        door = torch.stack((d["door_ahead"], torch.zeros_like(psi), d["height"]), dim=-1) + s.unsqueeze(-1) * along
        full = lambda v: torch.full((count,), float(v), device=device)
        self._handle.place(ids, door=door, normal=normal, u0=-along, standoff=full(c.KEY_AIM_OFF_DOOR_M),
                           length=full(c.KEY_BAR_BOX[0][1]), inner=full(0.0), radius=full(c.KEY_TOP_M),
                           inertia=d["inertia"], rest_nm=full(0.0), full_nm=full(0.0), damping=d["damping"],
                           friction=d["friction"], theta_max=full(c.KEY_THETA_END_RAD),
                           theta_latch=full(c.KEY_THETA_UNLOCK_RAD), mu=d["mu"], theta_min=full(c.KEY_THETA_START_RAD),
                           pad=True, detent_nm=d["detent"],
                           detent_from=full(c.KEY_THETA_UNLOCK_RAD - c.KEY_DETENT_SPAN_RAD),
                           gravity_nm=full(c.KEY_GRAVITY_NM))
        pivot = self._handle.pivot[ids]
        e = self._est
        e["pivot"][ids] = pivot + d["err_side"].unsqueeze(-1) * along + d["err_up"].unsqueeze(-1) * z \
            + d["err_depth"].unsqueeze(-1) * normal
        e["u0"][ids], e["door_n"][ids] = -along, normal
        e["reach"][ids] = d["reach"]
        e["clearance"][ids] = handle.FOOT_RADIUS_M + c.KEY_TOP_M
        e["theta_start"][ids] = c.KEY_THETA_START_RAD
        e["theta_end"][ids] = c.KEY_THETA_END_RAD + c.OVERDRIVE_RAD
        rated = (d["friction"] + d["detent"]) * (1.0 + d["err_rating"])
        e["max_n"][ids] = (c.MAX_MARGIN * rated / d["reach"]).clamp(*c.KEY_MAX_N_LIMITS)
        for name in ("descent", "descend_speed", "turn_speed"):
            e[name][ids] = d[name]
        start = full(c.KEY_THETA_START_RAD)
        rest = handle.foot_on_bar(e["pivot"][ids], -along, start, d["reach"], e["clearance"][ids])
        self._pre[ids] = rest + d["descent"].unsqueeze(-1) * handle.lever_top(-along, start)
        self._waypoint[ids] = self._pre[ids] + c.KEY_WAYPOINT_BACK_M * normal + c.KEY_WAYPOINT_UP_M * z

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        self._log_handle_episodes(env_ids)
        count = len(env_ids)
        if self._evaluation is not None:
            self._handle_env[env_ids] = True
        else:
            self._handle_env[env_ids] = torch.rand(count, device=self.device) < self.cfg.handle_fraction
        super()._reset_idx(env_ids)
        # The arm starts at, and is held to, its pose (look or rest).
        pose = torch.as_tensor(self._arm_base, device=self.device).expand(count, 6).clone()
        if self.cfg.arm_jitter and self._evaluation is None:
            jitter = torch.as_tensor(self._arm_jitter, device=self.device)
            pose += (2 * torch.rand(count, 6, device=self.device) - 1) * jitter
        self._arm_pose[env_ids] = pose
        joint_pos = self._robot.data.default_joint_pos[env_ids].clone()
        joint_pos[:, self._order[12:18]] = pose
        if self.cfg.settled_spawn:
            # Standing, settled: the legs bent as gravity holds them at the default targets, the base at its height.
            joint_pos[:, self._order[:12]] = torch.as_tensor(leg_cfg.SETTLED_LEG_POS, device=self.device)
            root = self._robot.data.default_root_state[env_ids].clone()
            root[:, :3] += self.scene.env_origins[env_ids]
            root[:, 2] = self.scene.env_origins[env_ids, 2] + leg_cfg.SETTLED_BASE_HEIGHT_M
            if self.cfg.spawn_states and (self._evaluation is None or self.cfg.spawn_states_in_eval):
                # Some episodes start from where the walking policy leaves the robot instead (the staged demo's handover).
                if not hasattr(self, "_spawn_library"):
                    import json

                    states = json.loads(open(self.cfg.spawn_states).read())
                    self._spawn_library = {
                        "legs": torch.tensor([s["legs"] for s in states], device=self.device),
                        "z": torch.tensor([s["base_z"] for s in states], device=self.device),
                        "rp": torch.tensor([[s["roll"], s["pitch"]] for s in states], device=self.device)}
                lib = self._spawn_library
                pick = torch.rand(count, device=self.device) < self.cfg.spawn_states_fraction
                if bool(pick.any()):
                    k = int(pick.sum())
                    which = torch.randint(len(lib["z"]), (k,), device=self.device)
                    legs = lib["legs"][which] + (2 * torch.rand(k, 12, device=self.device) - 1) * self.cfg.spawn_state_jitter
                    rows = pick.nonzero().flatten()
                    joint_pos[rows.unsqueeze(-1), self._order[:12].unsqueeze(0)] = legs
                    root[rows, 2] = self.scene.env_origins[env_ids[rows], 2] + lib["z"][which]
                    roll, pitch = lib["rp"][which, 0], lib["rp"][which, 1]
                    cr, sr, cp, sp = torch.cos(roll / 2), torch.sin(roll / 2), torch.cos(pitch / 2), torch.sin(pitch / 2)
                    root[rows, 3:7] = torch.stack((cr * cp, sr * cp, cr * sp, -sr * sp), dim=-1)
            self._robot.write_root_pose_to_sim(root[:, :7], env_ids)
            self._robot.write_root_velocity_to_sim(torch.zeros_like(root[:, 7:]), env_ids)
        self._robot.write_joint_state_to_sim(joint_pos, self._robot.data.default_joint_vel[env_ids].clone(), env_ids=env_ids)
        self._goals.reset(env_ids)
        self._push_cmd.reset(env_ids)
        self._push_ext.reset(env_ids)
        self._push_w[env_ids] = 0.0
        self._ee_force_w[env_ids] = 0.0
        self._knee_force_w[env_ids] = 0.0
        self._handle.reset(env_ids)
        self._layer.reset(env_ids)
        self._correction.reset(env_ids)
        self._stage[env_ids] = WAITING
        for buf in (self._stage_step, self._since_reset):
            buf[env_ids] = 0
        for buf in (self._open_for, self._longest_open, self._peak_n, self._step_peak, self._max_theta,
                    self._off_bar_for, self._slip_now, self._door_peak, self._done_for, self._open_force_sum,
                    self._open_count, self._cmd_err_sq, self._cmd_count):
            buf[env_ids] = 0.0
        self._open_step[env_ids] = -1
        self._engage_step[env_ids] = -1
        self._slipped[env_ids] = False
        self._damage_now[env_ids] = False
        self._damage_seen[env_ids] = False
        self._engage_miss[env_ids] = float("nan")
        self._goal_w_held[env_ids] = 0.0
        self._down_for[env_ids] = 0.0
        self._head_force_w[env_ids] = 0.0
        ids = env_ids[self._handle_env[env_ids]]
        if len(ids) > 0:
            if self._evaluation is not None:
                draw = {k: v[ids] for k, v in self._evaluation["handles"].items()}
                self._start_delay[ids] = 25
            else:
                draw = (self._draw_keys if self._key else self._draw_handles)(ids)
                self._start_delay[ids] = torch.randint(15, 50, (len(ids),), device=self.device)
            (self._place_keys if self._key else self._place_handles)(ids, draw)
            # The episode's largest angle starts from the lever's start stop (a key's is -45 degrees), not from 0.
            self._max_theta[ids] = self._handle.theta_min[ids]

    def _log_handle_episodes(self, env_ids: torch.Tensor) -> None:
        """Training-time handle diagnostics over the episodes ending now (sampled at reset, not a frozen evaluation)."""
        log = self.extras.setdefault("log", {})
        foot_err = (self._tip_pos()[env_ids] - self._goal_world()[env_ids]).norm(dim=-1)
        free = ~self._handle_env[env_ids]
        if bool(free.any()):
            log["Leg/free_foot_error_m"] = float(foot_err[free].median())
        stance = leg_rewards.stance_error(
            self._robot.data.body_pos_w[env_ids][:, self._feet_bodies] - self.scene.env_origins[env_ids].unsqueeze(1),
            self._robot.data.root_pos_w[env_ids] - self.scene.env_origins[env_ids], self._base_yaw_quat()[env_ids])
        log["Leg/stance_error_cm"] = 100.0 * float(stance[:, list(leg_cfg.SUPPORT_FEET)].median())
        log["Leg/stance_error_max_cm"] = 100.0 * float(stance[:, list(leg_cfg.SUPPORT_FEET)].max(dim=1).values.median())
        h = env_ids[self._handle_env[env_ids]]
        if len(h) == 0:
            return
        engaged = self._engage_step[h] >= 0
        log["Handle/missed_fraction"] = float((~engaged).float().mean())
        if bool(engaged.any()):
            e = h[engaged]
            log["Handle/success_fraction"] = float(((self._longest_open[e] >= leg_cfg.SUCCESS_OPEN_S)
                                                    & ~self._term_damage[e] & ~self._term_fallen[e]).float().mean())
            log["Handle/opened_fraction"] = float((self._open_step[e] >= 0).float().mean())
            log["Handle/peak_n"] = float(self._peak_n[e].mean())
            log["Handle/max_theta_deg"] = math.degrees(float(self._max_theta[e].mean()))
            log["Handle/slip_fraction"] = float(self._slipped[e].float().mean())
            log["Handle/damaged_fraction"] = float(self._term_damage[e].float().mean())
            log["Handle/engage_miss_mm"] = 1000.0 * float(self._engage_miss[e].median())
