"""The force-transmission task: UniFP's whole-body controller, asked to load a handle or a button.

UniFP's task (`env.py`) asks for a few newtons at wherever the goal happens to be. This one puts
the tool *on* something -- a claw over a handle's bar, a pad on a button -- and asks for tens of
newtons into it, far more than the D1's motors hold in a bent reach (9.4 N median, F-101). Meeting
it means doing what the static study says is possible and the arm cannot do alone: turning the arm
so the force runs through its joints, and making the legs and body supply the force.

What an episode is, for the `fixture_fraction` of environments that get a fixture:

  1. The goal slides to a handle placement drawn in front of the robot (`hook_cfg.HANDLE_*`),
     exactly as UniFP's goal slides anywhere -- same generator, a chosen destination.
  2. Shortly after it arrives, the tool engages the fixture wherever the tool actually is. From then
     on the goal is held on the fixture's anchor and the velocity command is zero.
  3. The force command ramps between levels up to the curriculum's ceiling, along the fixture's
     axis: back toward the robot for a pull, into the panel for a press. It is written into UniFP's
     own force-command channel, so the observation is unchanged.
  4. The fixture's reaction is the "measured" external force UniFP's reward and estimator read.
     Backing off the handle, or sliding off it, loses the contact and ends the episode.

The other environments run UniFP's task unchanged, pushes and all, so the free-space behaviour the
warm-start checkpoint already has stays in the training distribution.

**No width changes.** Actor 32 x 76, critic 3 x 153, 18 actions: a UniFP checkpoint loads and
continues. The fixture state the critic needs goes into its `mass_params` block, which is zero in
UniFP's task (`hook_cfg.PRIVILEGED_WIDTH`).
"""
from __future__ import annotations

import math

import torch

from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.utils import configclass

from unifp_isaaclab import interface, task as goal_task

from . import fixture, hook_cfg, hook_rewards, task_cfg
from .env import Go2D1PosForceEnv
from .env_cfg import Go2D1PosForceEnvCfg


@configclass
class Go2D1HookEnvCfg(Go2D1PosForceEnvCfg):
    """UniFP's task plus a fixture to pull or press, with the roll objective on by default."""

    roll_objective: bool = True
    #: Forces from the first step: this task is fine-tuned from a checkpoint that already has them.
    force_start_step: int = 0
    fixture_fraction: float = hook_cfg.FIXTURE_FRACTION
    press_fraction: float = hook_cfg.PRESS_FRACTION
    force_ceiling_n: float = hook_cfg.CURRICULUM_START_N
    force_ceiling_max_n: float = hook_cfg.CURRICULUM_MAX_N
    curriculum: bool = True
    #: Contact on the D1's own links. UniFP's sensor matches `Robot/.*`, one level deep, and the weld
    #: puts the arm a level further down (`Robot/D1/...`), so the arm had no contact sensing at all.
    #: This task needs it: an arm braced against the robot's own body passes load around its motors,
    #: and nothing else would show it. Read by the evaluator; no reward term uses it.
    arm_contact_sensor: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/D1/.*", history_length=0, update_period=0.0,
        track_air_time=False)


class HeldGoals(goal_task.EeGoalTrajectory):
    """UniFP's goal generator, able to hold chosen environments on a fixed point.

    Held environments report `hold_goal` and `hold_roll` instead of their trajectory. The
    trajectory keeps running underneath, so releasing a hold resumes it; nothing is lost but the
    goal's position while it was held.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._allocate_holds()

    @classmethod
    def adopt(cls, goals: goal_task.EeGoalTrajectory) -> "HeldGoals":
        """Turn an existing generator into this one, keeping its state and drawing nothing.

        Constructing a fresh one would draw every robot's cadence again from the global RNG, and
        then a run with no fixtures would not reproduce UniFP's episodes -- which is what lets the
        frozen free-space manifest score a policy trained on this task.
        """
        goals.__class__ = cls
        goals._allocate_holds()
        return goals

    def _allocate_holds(self) -> None:
        self.hold_mask = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.hold_goal = torch.zeros(self.num_envs, 3, device=self.device)
        self.hold_roll = torch.zeros(self.num_envs, device=self.device)

    def step(self) -> torch.Tensor:
        current = super().step()
        self.current = torch.where(self.hold_mask.unsqueeze(-1), self.hold_goal, current)
        if self.roll_range is not None:
            self.current_roll = torch.where(self.hold_mask, self.hold_roll, self.current_roll)
        return self.current

    def aim(self, env_ids: torch.Tensor, goal: torch.Tensor, roll: torch.Tensor) -> None:
        """Send the goal from wherever it starts to `goal`, arriving with `roll`."""
        self.goal[env_ids] = goal
        self.goal_roll[env_ids] = roll
        self.timer[env_ids] = 0.0


class Go2D1HookEnv(Go2D1PosForceEnv):
    cfg: Go2D1HookEnvCfg

    def __init__(self, cfg: Go2D1HookEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, device = self.num_envs, self.device
        # The parent's generator, taught to hold a goal on the fixture; no new draws (see `adopt`).
        self._goals = HeldGoals.adopt(self._goals)
        self._fixture = fixture.ContactFixture(n, device, damping=hook_cfg.FIXTURE_DAMPING)
        self._levels = fixture.ForceLevelSchedule(
            n, device, hold_s=hook_cfg.LEVEL_HOLD_S, ramp_n_per_s=hook_cfg.LEVEL_RAMP_N_PER_S,
            zero_prob=hook_cfg.LEVEL_ZERO_PROB, dt=interface.POLICY_DT,
            frontier_prob=hook_cfg.LEVEL_FRONTIER_PROB, frontier_fraction=hook_cfg.LEVEL_FRONTIER_FRACTION)
        zeros = lambda *shape: torch.zeros(*shape, device=device)
        self._fixture_env = torch.zeros(n, dtype=torch.bool, device=device)
        self._press_env = torch.zeros(n, dtype=torch.bool, device=device)
        #: Of the pulls, the ones on a ring rather than a bar; drawn at engagement.
        self._ring_env = torch.zeros(n, dtype=torch.bool, device=device)
        self._handle_sphere = zeros(n, 3)
        self._engage_step = torch.zeros(n, dtype=torch.long, device=device)
        self._since_reset = torch.zeros(n, dtype=torch.long, device=device)
        #: UniFP's external push, kept separately: the wrench on the tool is this plus the fixture's.
        self._push_force_w = zeros(n, 3)
        self._lost_now = torch.zeros(n, dtype=torch.bool, device=device)
        self._term_lost = torch.zeros(n, dtype=torch.bool, device=device)
        self._term_fallen = torch.zeros(n, dtype=torch.bool, device=device)
        self._term_lost_how = torch.zeros(n, dtype=torch.long, device=device)
        self._offset = torch.as_tensor(self.cfg.tool_offset_m, device=device).expand(n, 3)

        self._ceiling = float(cfg.force_ceiling_n)
        #: The press curriculum (hook_cfg.PRESS_SLIP_START_M): how far a pad may slide in training.
        self._press_slip = hook_cfg.PRESS_SLIP_START_M
        self._press_ended, self._press_lost = 0, 0
        self._rel_err_ema: float | None = None
        self._last_promotion_step = 0
        self._err_sum, self._cmd_sum, self._engaged_steps = zeros(n), zeros(n), zeros(n)

        #: Evaluation plan, set by `set_evaluation`: every episode a fixture, placed and loaded as
        #: the plan says rather than drawn.
        self._evaluation: dict | None = None

        self._weights.update(hook_cfg.scaled_weights())
        self._terms.update(hook_rewards.TERMS)
        for name in hook_cfg.WEIGHTS:
            self._episode_sums[name] = zeros(n)

    def _setup_scene(self):
        super()._setup_scene()
        self._arm_contact = ContactSensor(self.cfg.arm_contact_sensor)
        self.scene.sensors["arm_contact"] = self._arm_contact

    # --- evaluation ---------------------------------------------------------------------------

    def set_evaluation(self, handles: torch.Tensor, press: bool, levels, hold_steps: int,
                       engage_delay_steps: int = 15, kind: str = "ring") -> None:
        """Fix every episode's placement, axis and force staircase. Call before `reset()`.

        `handles` is (num_envs, 3) in goal-sphere coordinates. Axes are exact: straight back toward
        the robot for a pull, straight away from it for a press, no noise. A pull is on a `kind` of
        fixture, "ring" or a horizontal "bar", at `hook_cfg.FIXTURE_STIFFNESS`. The curriculum is off.
        """
        if kind not in hook_cfg.EVAL_FIXTURE_KINDS:
            raise ValueError(f"fixture kind {kind!r} is not one of {hook_cfg.EVAL_FIXTURE_KINDS}")
        self._evaluation = {"handles": handles.to(self.device), "press": press,
                            "engage_delay": int(engage_delay_steps), "kind": kind}
        self._levels.use_staircase(levels, hold_steps)
        self.cfg.curriculum = False

    # --- the fixture, stepped at the physics rate --------------------------------------------

    def _tool_vel_w(self) -> torch.Tensor:
        """World velocity of the controlled point: the body's plus rotation about its offset."""
        quat = self._robot.data.body_quat_w[:, self._tip_body]
        arm = interface.quat_apply(quat, self._offset)
        return (self._robot.data.body_lin_vel_w[:, self._tip_body]
                + torch.cross(self._robot.data.body_ang_vel_w[:, self._tip_body], arm, dim=-1))

    def _apply_action(self) -> None:
        super()._apply_action()
        reaction = self._fixture.step(self._tip_pos(), self._tool_vel_w())
        self._ee_force_w = self._push_force_w + reaction
        self._write_wrench()

    def _step_forces(self) -> None:
        """UniFP's push schedule for the free environments; the level schedule for the engaged ones."""
        free = ~self._fixture_env
        if self.common_step_counter > self.cfg.force_start_step:
            commanded, external = self._forces.step(self.episode_length_buf)
            self._commands[:, interface.CMD_EE_FORCE] = torch.where(
                free.unsqueeze(-1), commanded, torch.zeros_like(commanded))
            self._push_force_w = torch.where(free.unsqueeze(-1), external, torch.zeros_like(external))
        # A detached pad is still being asked to press: the command stays, the reward does not.
        engaged = self._fixture.engaged | self._fixture.detached
        level = self._levels.step(engaged, self._ceiling)
        command_yaw = interface.quat_rotate_inverse(
            self._base_yaw_quat(), level.unsqueeze(-1) * self._fixture.axis)
        self._commands[:, interface.CMD_EE_FORCE] = torch.where(
            engaged.unsqueeze(-1), command_yaw,
            torch.where(self._fixture_env.unsqueeze(-1), torch.zeros_like(command_yaw),
                        self._commands[:, interface.CMD_EE_FORCE]))

    def _fixture_command_w(self) -> torch.Tensor:
        return (self._levels.current * self._fixture.engaged).unsqueeze(-1) * self._fixture.axis

    # --- engagement and the held goal -------------------------------------------------------

    def _engage_due(self) -> None:
        waited = self._since_reset - self._engage_step
        settled = self._tool_vel_w().norm(dim=-1) < hook_cfg.ENGAGE_SETTLE_SPEED_M_S
        due = (self._fixture_env & ~self._fixture.engaged & ~self._fixture.lost & ~self._fixture.detached
               & ~self._fixture.ever_detached
               & (waited >= 0) & (settled | (waited >= hook_cfg.ENGAGE_MAX_WAIT_STEPS)))
        if not bool(due.any()):
            return
        ids = due.nonzero(as_tuple=False).flatten()
        point = self._tip_pos()[ids]
        base = self._robot.data.root_pos_w[ids] - self.scene.env_origins[ids]
        press = self._press_env[ids]
        noise = 0.0 if self._evaluation is not None else hook_cfg.PULL_YAW_NOISE_RAD
        elevation = (0.0, 0.0) if self._evaluation is not None else hook_cfg.PULL_ELEVATION_RAD
        down = 0.0 if self._evaluation is not None else hook_cfg.PRESS_DOWN_PROB
        pull_axis = fixture.pull_axes(point, base, yaw_noise_rad=noise, elevation_rad=elevation)
        press_axis = fixture.press_axes(point, base, yaw_noise_rad=noise, elevation_rad=elevation,
                                        down_prob=down)
        axis = torch.where(press.unsqueeze(-1), press_axis, pull_axis)
        count = len(ids)
        if self._evaluation is not None:
            ring = torch.full((count,), self._evaluation["kind"] == "ring", dtype=torch.bool, device=self.device)
        else:
            ring = torch.rand(count, device=self.device) < hook_cfg.RING_PROB
        ring &= ~press
        self._ring_env[ids] = ring
        # Per-kind constants: (claw on a bar, ring, pad on a button).
        pick = lambda bar_value, press_value, ring_value=None: torch.where(
            press, torch.full((count,), float(press_value), device=self.device),
            torch.where(ring, torch.full((count,), float(bar_value if ring_value is None else ring_value),
                                         device=self.device),
                        torch.full((count,), float(bar_value), device=self.device)))
        never = fixture.ContactFixture.NEVER_M
        if self._evaluation is not None:
            stiffness = torch.full((count,), hook_cfg.FIXTURE_STIFFNESS, device=self.device)
            vertical = torch.zeros(count, dtype=torch.bool, device=self.device)
        else:
            low, high = hook_cfg.FIXTURE_STIFFNESS_RANGE
            stiffness = torch.exp(math.log(low) + torch.rand(count, device=self.device) * math.log(high / low))
            vertical = torch.rand(count, device=self.device) < hook_cfg.HOOK_VERTICAL_BAR_PROB
        # A ring and a pad have no bar: they take the isotropic contact.
        bar = fixture.bar_directions(axis, vertical) * (~press & ~ring).unsqueeze(-1)
        self._fixture.engage(ids, point, axis, stiffness=stiffness,
                             mu=pick(hook_cfg.HOOK_MU, hook_cfg.PRESS_MU, hook_cfg.RING_MU),
                             allowance_n=pick(hook_cfg.HOOK_ALLOWANCE_N, self._press_allowance(),
                                              hook_cfg.RING_ALLOWANCE_N),
                             slip_radius_m=pick(hook_cfg.HOOK_SLIP_RADIUS_M,
                                                (hook_cfg.PRESS_SLIP_RADIUS_M if self._evaluation is not None
                                                 else self._press_slip), never),
                             release_m=pick(never, hook_cfg.PRESS_RELEASE_M),
                             backstop_m=pick(hook_cfg.HOOK_BACKSTOP_M, never),
                             bar=bar, lift_release_m=pick(hook_cfg.HOOK_LIFT_RELEASE_M, never, never),
                             reattach=press & (self._evaluation is None) & hook_cfg.PRESS_REATTACH)
        self._levels.reset(ids)
        self._goals.hold_mask[ids] = True
        self._goals.hold_roll[ids] = self._goals.current_roll[ids]

    def _press_allowance(self) -> float:
        """The pad's resting friction, on the press curriculum (hook_cfg.PRESS_ALLOWANCE_START_N)."""
        if self._evaluation is not None:
            return hook_cfg.PRESS_ALLOWANCE_N
        span = hook_cfg.PRESS_SLIP_START_M - hook_cfg.PRESS_SLIP_RADIUS_M
        progress = (self._press_slip - hook_cfg.PRESS_SLIP_RADIUS_M) / span if span > 0 else 0.0
        return hook_cfg.PRESS_ALLOWANCE_N + progress * (hook_cfg.PRESS_ALLOWANCE_START_N - hook_cfg.PRESS_ALLOWANCE_N)

    def _hold_goals_on_anchor(self) -> None:
        """Express each held anchor in the goal sphere of the base as it stands now."""
        held = self._goals.hold_mask
        if not bool(held.any()):
            return
        local = interface.quat_rotate_inverse(self._base_yaw_quat(),
                                              self._fixture.anchor - self._goal_centre())
        self._goals.hold_goal = torch.where(held.unsqueeze(-1), interface.cart2sphere(local),
                                            self._goals.hold_goal)

    def _get_rewards(self) -> torch.Tensor:
        self._since_reset += 1
        self._engage_due()
        self._hold_goals_on_anchor()
        total = super()._get_rewards()
        engaged = self._fixture.engaged
        command = self._fixture_command_w()
        # The curriculum reads the along-axis error at the frontier only; see
        # hook_cfg.CURRICULUM_PROMOTE_REL_ERR.
        error, _ = hook_rewards.force_error(self._fixture.applied_by_robot(), command, self._fixture.axis)
        frontier = engaged & (self._levels.current >= hook_cfg.LEVEL_FRONTIER_FRACTION * self._ceiling)
        self._err_sum += error * frontier
        self._cmd_sum += command.norm(dim=-1) * frontier
        self._engaged_steps += engaged.float()
        return total

    def _task_state(self):
        state = super()._task_state()
        state.fixture_engaged = self._fixture.engaged.float()
        state.fixture_applied_w = self._fixture.applied_by_robot()
        state.fixture_command_w = self._fixture_command_w()
        state.fixture_lost = self._lost_now.float()
        state.fixture_axis_w = self._fixture.axis
        state.fixture_risk = self._fixture.risk
        state.fixture_active = (self._fixture.engaged | self._fixture.detached).float()
        state.tool_vel_w = self._tool_vel_w()
        return state

    def _privileged_extra(self) -> torch.Tensor:
        """The fixture as the critic sees it, in the `mass_params` block (see the module docstring)."""
        engaged = self._fixture.engaged.float().unsqueeze(-1)
        yaw = self._base_yaw_quat()
        out = torch.zeros(self.num_envs, hook_cfg.PRIVILEGED_WIDTH, device=self.device)
        out[:, 0:3] = interface.quat_rotate_inverse(yaw, self._fixture.axis) * engaged
        out[:, 3:4] = engaged
        out[:, 4:5] = self._press_env.float().unsqueeze(-1) * engaged
        out[:, 5:8] = interface.quat_rotate_inverse(yaw, self._fixture.anchor - self._tip_pos()) * 10.0 * engaged
        out[:, 8:9] = self._fixture.penetration_m.unsqueeze(-1) * 10.0
        out[:, 9:10] = self._levels.current.unsqueeze(-1) * 0.01 * engaged
        out[:, 10] = self._ceiling * 0.01
        out[:, 11:12] = self._fixture.stiffness.unsqueeze(-1) / 3000.0 * engaged
        out[:, 12:15] = interface.quat_rotate_inverse(yaw, self._fixture.bar) * engaged
        out[:, 15:18] = interface.quat_rotate_inverse(yaw, self._fixture.support) * engaged
        out[:, 18:19] = self._ring_env.float().unsqueeze(-1) * engaged
        out[:, 19:20] = self._fixture.risk.unsqueeze(-1)
        out[:, 20:21] = self._fixture.slip_radius_m.clamp(max=1.0).unsqueeze(-1) * engaged
        return out

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        fallen, timed_out = super()._get_dones()
        self._lost_now = self._fixture.lost.clone()
        # Why each environment last ended, kept through the reset that follows so an evaluator
        # stepping from outside can still read it (`hook_eval.record_step`).
        self._term_lost, self._term_fallen = self._lost_now.clone(), fallen.clone()
        self._term_lost_how = self._fixture.lost_how.clone()
        return fallen | self._lost_now, timed_out

    def _resample_velocity_commands(self, env_ids: torch.Tensor) -> None:
        """UniFP's draw, except that an episode with a fixture stands."""
        super()._resample_velocity_commands(env_ids)
        standing = env_ids[self._fixture_env[env_ids]]
        self._commands[standing, :3] = 0.0

    # --- resetting ------------------------------------------------------------------------

    def _update_curriculum(self, env_ids: torch.Tensor) -> None:
        """Fold finished episodes into the force-error average; raise the ceiling when it is low."""
        counted = env_ids[(self._engaged_steps[env_ids] >= hook_cfg.CURRICULUM_MIN_ENGAGED_STEPS)]
        log = self.extras.setdefault("log", {})
        finished_fixture = env_ids[self._fixture_env[env_ids]]
        if len(finished_fixture) > 0:
            log["Fixture/lost_fraction"] = float(self._fixture.lost[finished_fixture].float().mean())
            how = self._fixture.lost_how[finished_fixture]
            for code, name in ((fixture.ContactFixture.LOST_BACKED_OFF, "backed_off"),
                               (fixture.ContactFixture.LOST_SLID_OFF, "slid_off"),
                               (fixture.ContactFixture.LOST_LIFTED_OFF, "lifted_off")):
                log[f"Fixture/lost_{name}_fraction"] = float((how == code).float().mean())
        if len(counted) > 0:
            commanded = float(self._cmd_sum[counted].sum())
            if commanded > 1.0:
                rel = float(self._err_sum[counted].sum()) / commanded
                self._rel_err_ema = rel if self._rel_err_ema is None else (
                    (1 - hook_cfg.CURRICULUM_EMA) * self._rel_err_ema + hook_cfg.CURRICULUM_EMA * rel)
                log["Fixture/relative_force_error"] = rel
                log["Fixture/mean_error_n"] = float(
                    self._err_sum[counted].sum() / self._engaged_steps[counted].sum())
        # The press curriculum: close the pad's slide allowance while presses keep their pad.
        # Presses that engaged at all; an episode reset before its pad landed says nothing.
        fx = self._fixture
        landed = fx.engaged | fx.lost | fx.detached | fx.ever_detached
        presses = env_ids[self._fixture_env[env_ids] & self._press_env[env_ids] & landed[env_ids]]
        if len(presses) > 0:
            self._press_ended += len(presses)
            self._press_lost += int((fx.lost[presses] | fx.ever_detached[presses]).sum())
        if self._press_ended >= hook_cfg.PRESS_CURRICULUM_MIN_EPISODES:
            lost = self._press_lost / self._press_ended
            log["Fixture/press_lost_fraction"] = lost
            if (self.cfg.curriculum and lost < hook_cfg.PRESS_LOST_TARGET
                    and self._press_slip > hook_cfg.PRESS_SLIP_RADIUS_M):
                self._press_slip = max(self._press_slip - hook_cfg.PRESS_SLIP_STEP_M,
                                       hook_cfg.PRESS_SLIP_RADIUS_M)
            self._press_ended, self._press_lost = 0, 0
        log["Fixture/press_slip_radius_m"] = self._press_slip
        wait = hook_cfg.CURRICULUM_MIN_ITERATIONS * task_cfg.NUM_STEPS_PER_ENV
        if (self.cfg.curriculum and self._rel_err_ema is not None
                and self._rel_err_ema < hook_cfg.CURRICULUM_PROMOTE_REL_ERR
                and self._ceiling < self.cfg.force_ceiling_max_n
                and self.common_step_counter - self._last_promotion_step >= wait):
            self._ceiling = min(self._ceiling + hook_cfg.CURRICULUM_STEP_N, self.cfg.force_ceiling_max_n)
            self._last_promotion_step = self.common_step_counter
            # The average was earned at the old ceiling; start the new one from scratch.
            self._rel_err_ema = None
        log["Fixture/force_ceiling_n"] = self._ceiling
        if self._rel_err_ema is not None:
            log["Fixture/relative_force_error_ema"] = self._rel_err_ema

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        self._update_curriculum(env_ids)
        count = len(env_ids)
        if self._evaluation is not None:
            self._fixture_env[env_ids] = True
            self._press_env[env_ids] = bool(self._evaluation["press"])
        elif self.cfg.fixture_fraction <= 0.0:
            # Drawing nothing keeps the global RNG where UniFP's task has it, so its frozen manifests
            # replay here episode for episode.
            self._fixture_env[env_ids] = False
            self._press_env[env_ids] = False
        else:
            self._fixture_env[env_ids] = torch.rand(count, device=self.device) < self.cfg.fixture_fraction
            self._press_env[env_ids] = torch.rand(count, device=self.device) < self.cfg.press_fraction
        # Drawn before the parent reset, which resamples the velocity command and must see it stand.
        super()._reset_idx(env_ids)

        self._fixture.reset(env_ids)
        self._ring_env[env_ids] = False
        self._levels.reset(env_ids)
        self._push_force_w[env_ids] = 0.0
        self._ee_force_w[env_ids] = 0.0
        self._lost_now[env_ids] = False
        self._since_reset[env_ids] = 0
        self._err_sum[env_ids] = 0.0
        self._cmd_sum[env_ids] = 0.0
        self._engaged_steps[env_ids] = 0.0
        self._goals.hold_mask[env_ids] = False

        ids = env_ids[self._fixture_env[env_ids]]
        if len(ids) > 0:
            if self._evaluation is not None:
                handles = self._evaluation["handles"][ids]
                delay = torch.full((len(ids),), self._evaluation["engage_delay"], device=self.device)
                roll = torch.zeros(len(ids), device=self.device)
            else:
                handles = fixture.sample_handles(len(ids), self.device)
                low, high = hook_cfg.ENGAGE_DELAY_STEPS
                delay = torch.randint(low, high + 1, (len(ids),), device=self.device)
                roll_low, roll_high = task_cfg.EE_ROLL_RANGE_RAD
                roll = roll_low + torch.rand(len(ids), device=self.device) * (roll_high - roll_low)
            self._handle_sphere[ids] = handles
            self._goals.aim(ids, handles, roll)
            self._engage_step[ids] = self._goals.traj_steps[ids].long() + delay.long()
