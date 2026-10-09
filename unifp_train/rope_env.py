"""The rope-pull task: UniFP's whole-body controller holding a steady pull on a rope, and recovering when it gives.

The hook task (`hook_env.py`, F-103) showed the body can put 60 N through the D1 when the arm is drawn straight along
the pull and the legs make the force. This task asks the two things that showed nothing about (Lukas, 2026-10-07):

  * **a consistent force, no repeated tugs.** The claw holds a rope (`rope.RopeTether`) that pulls only along its own
    line, and the tension is read at the physics rate: its ripple and the rope going slack are priced
    (`rope_rewards`), on top of the hook task's force objective.
  * **recovery when the resistance goes.** In `release_prob` of rope episodes the anchor lets go mid-pull (snap, slip
    or fade, `rope_cfg.RELEASE_*`). The robot was leaning on that force. A task layer notices after a delay -- or
    never -- ramps its command to zero and lets the goal return to the grip's place relative to the body.

What an episode is, for the `rope_fraction` that get a rope:

  1. The goal slides to a grip placement in front of the robot (the hook task's sampler) and the robot stands.
  2. Once the tool has settled there, the rope is tied to it, running `length` straight out from it against the pull
     direction (back toward the robot, turned and tilted by the hook task's noise). It starts exactly taut.
  3. The goal is held on that grip point in the world, and the force command ramps between levels up to the
     curriculum's ceiling along the pull direction, in UniFP's own force channel.
  4. Maybe, the anchor lets go. Until the task layer notices, nothing changes for the policy but what it feels.
     After, the command ramps to zero and the goal slides back to the grip placement in the body frame, from where
     UniFP's free-space goals resume.

The rest run UniFP's task, pushes and all, as the hook task's do. **No width changes**: actor 32 x 76, critic
3 x 153, 18 actions; the rope's state goes into the critic's `mass_params` block.
"""
from __future__ import annotations

import math

import torch

from isaaclab.sensors import ContactSensor
from isaaclab.utils import configclass

from unifp_isaaclab import interface

from . import fixture, hook_cfg, rope, rope_cfg, rope_rewards, task_cfg
from .env import Go2D1PosForceEnv
from .hook_env import Go2D1HookEnv, Go2D1HookEnvCfg, HeldGoals


@configclass
class Go2D1RopeEnvCfg(Go2D1HookEnvCfg):
    """The hook task's configuration (roll objective, forces from the first step, arm contact) with a rope."""

    rope_fraction: float = rope_cfg.ROPE_FRACTION
    force_ceiling_n: float = rope_cfg.CURRICULUM_START_N
    force_ceiling_max_n: float = rope_cfg.CURRICULUM_MAX_N
    release_prob: float = rope_cfg.RELEASE_PROB
    detect_never_prob: float = rope_cfg.DETECT_NEVER_PROB
    stiffness_range: tuple[float, float] = rope_cfg.ROPE_STIFFNESS_RANGE
    length_range: tuple[float, float] = rope_cfg.ROPE_LENGTH_RANGE_M
    #: Replacements for `rope_cfg.WEIGHTS`, as ((name, weight), ...): the CLI's `--rope_weight`.
    weight_overrides: tuple = ()
    #: How the curriculum judges the frontier: "window" (the error summed since the last promotion; the default since
    #: 2026-10-07 evening) or "ema" (v1-v3's average, restarted from one batch after each promotion; kept so their
    #: recipe reproduces). See `rope_cfg.CURRICULUM_PROMOTE_REL_ERR`.
    curriculum_rule: str = "window"


class Go2D1RopeEnv(Go2D1PosForceEnv):
    cfg: Go2D1RopeEnvCfg

    # The hook task's, unchanged: the controlled point's world velocity, with rotation about its offset.
    _tool_vel_w = Go2D1HookEnv._tool_vel_w

    def __init__(self, cfg: Go2D1RopeEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, device = self.num_envs, self.device
        # The parent's goal generator, taught to hold a goal (no new draws: `HeldGoals.adopt`).
        self._goals = HeldGoals.adopt(self._goals)
        self._rope = rope.RopeTether(n, device)
        self._levels = fixture.ForceLevelSchedule(
            n, device, hold_s=rope_cfg.LEVEL_HOLD_S, ramp_n_per_s=rope_cfg.LEVEL_RAMP_N_PER_S,
            zero_prob=rope_cfg.LEVEL_ZERO_PROB, dt=interface.POLICY_DT,
            frontier_prob=rope_cfg.LEVEL_FRONTIER_PROB, frontier_fraction=rope_cfg.LEVEL_FRONTIER_FRACTION)
        zeros = lambda *shape: torch.zeros(*shape, device=device)
        longs = lambda: torch.zeros(n, dtype=torch.long, device=device)
        flags = lambda: torch.zeros(n, dtype=torch.bool, device=device)
        self._offset = torch.as_tensor(self.cfg.tool_offset_m, device=device).expand(n, 3)
        self._rope_env = flags()
        self._handle_sphere = zeros(n, 3)
        self._engage_step = longs()
        self._since_reset = longs()
        self._engaged_at = longs()
        #: The commanded pull's direction (unit, toward the robot) and the grip point, env frame, fixed at engagement.
        self._pull_dir = zeros(n, 3)
        self._grip_point = zeros(n, 3)
        self._push_force_w = zeros(n, 3)
        #: The commanded tension, N: the level schedule while the anchor holds, frozen once it lets go, ramped to
        #: zero once the task layer has noticed.
        self._cmd_level = zeros(n)
        # The release plan, drawn at reset: whether, when (policy steps after engagement), how, and when it is noticed.
        self._release_planned = flags()
        self._release_after = longs()
        self._release_kind = longs()
        self._release_residual = zeros(n)
        self._release_ramp = longs()
        self._detect_delay = longs()           # policy steps after the release; -1 = never noticed
        self._release_at = torch.full((n,), -1, dtype=torch.long, device=device)
        self._detected = flags()
        self._min_release_command = rope_cfg.RELEASE_MIN_COMMAND_N
        # Physics-rate bookkeeping for the current policy step.
        decimation = int(self.cfg.decimation)
        self._substep = 0
        self._tension_sub = zeros(n, decimation)
        self._last_err = zeros(n)
        #: The force error's running mean (`rope_cfg.WOBBLE_TAU_S`), and this step's squared deviations from it.
        self._err_mean = zeros(n)
        self._wobble = zeros(n)
        self._tv = zeros(n)
        self._slack_count = zeros(n)
        self._fallen_now = flags()
        self._term_fallen = flags()
        self._term_released = flags()

        self._ceiling = float(cfg.force_ceiling_n)
        self._rel_err_ema: float | None = None
        #: The frontier's force error and command, summed over every finished episode since the last promotion: the
        #: curriculum's evidence (v3 on). An average restarted from the first batch after each promotion and then moved
        #: at 0.005 per batch was that first batch for hundreds of batches, and promoted v3 from 30 to 55 N in 239
        #: iterations on a lucky one.
        self._window_err = 0.0
        self._window_cmd = 0.0
        self._last_promotion_step = 0
        self._err_sum, self._cmd_sum, self._engaged_steps = zeros(n), zeros(n), zeros(n)
        self._ep_tv_sum, self._ep_slack_sum, self._ep_intact_steps = zeros(n), zeros(n), zeros(n)
        self._recovery_steps = round(rope_cfg.RECOVERY_WINDOW_S / interface.POLICY_DT)

        #: Evaluation plan, set by `set_evaluation`: every episode a rope, placed, loaded and released as it says.
        self._evaluation: dict | None = None

        weights = dict(rope_cfg.WEIGHTS)
        for name, value in self.cfg.weight_overrides:
            if name not in weights:
                raise ValueError(f"no rope reward term {name!r}; terms are {sorted(weights)}")
            weights[name] = float(value)
        self._rope_weights = weights
        self._weights.update({name: weight * interface.POLICY_DT for name, weight in weights.items()})
        self._terms.update(rope_rewards.TERMS)
        for name in weights:
            self._episode_sums[name] = zeros(n)
        self._trunk_contacts, _ = self._contact_sensor.find_bodies(["base"])
        #: Link2 sits on Joint2 (the shoulder pitch): the arm's line for `rope_eval` runs from there to the tool.
        self._shoulder_body = self._robot.body_names.index("Link2")

    def _setup_scene(self):
        super()._setup_scene()
        self._arm_contact = ContactSensor(self.cfg.arm_contact_sensor)
        self.scene.sensors["arm_contact"] = self._arm_contact

    # --- the viewer -------------------------------------------------------------------------------

    def _set_debug_vis_impl(self, enable: bool) -> None:
        """UniFP's overlay, plus the rope, its post and its load (`rope_vis`)."""
        super()._set_debug_vis_impl(enable)
        if enable:
            if not hasattr(self, "_rope_markers"):
                from . import rope_vis

                self._rope_markers = rope_vis.RopeMarkers("/Visuals/rope", self.num_envs, str(self.device))
            self._rope_markers.set_visibility(True)
        elif hasattr(self, "_rope_markers"):
            self._rope_markers.set_visibility(False)

    def _debug_vis_callback(self, event) -> None:
        super()._debug_vis_callback(event)
        origins = self.scene.env_origins
        rp = self._rope
        tied = self._grip_point - rp.length.unsqueeze(-1) * self._pull_dir
        capacity = torch.where(torch.isfinite(rp.capacity), rp.capacity, torch.full_like(rp.capacity, 1.0e6))
        self._rope_markers.update(tip_w=self._tip_pos() + origins, load_w=rp.anchor + origins, tied_w=tied + origins,
                                  ground_z=origins[:, 2], held=rp.engaged, released=rp.released, tension=rp.tension,
                                  capacity=capacity)

    # --- evaluation ---------------------------------------------------------------------------

    def set_evaluation(self, rows: list[dict], levels, hold_steps: int) -> None:
        """Fix every episode's placement, rope, staircase and release (`rope_cfg.eval_plan()` rows). Before `reset()`."""
        if len(rows) != self.num_envs:
            raise ValueError(f"{len(rows)} evaluation rows for {self.num_envs} environments")
        kinds = {"snap": rope.SNAP, "slip": rope.SLIP, "fade": rope.FADE}
        tensor = lambda values, dtype=torch.float32: torch.as_tensor(values, dtype=dtype, device=self.device)
        self._evaluation = {
            "spheres": tensor([r["sphere"] for r in rows]),
            "stiffness": tensor([r["stiffness"] for r in rows]),
            "length": tensor([r.get("length", rope_cfg.EVAL_ROPE_LENGTH_M) for r in rows]),
            "elevation": tensor([r.get("elevation", 0.0) for r in rows]),
            "kind": tensor([kinds[r["kind"]] if r["kind"] else 0 for r in rows], torch.long),
            "residual": tensor([r["residual"] for r in rows]),
            "ramp": tensor([round(r["ramp_s"] / interface.SIM_DT) for r in rows], torch.long),
            "detect": tensor([-1 if r["detect_s"] is None else round(r["detect_s"] / interface.POLICY_DT)
                              for r in rows], torch.long),
            "planned": tensor([bool(r["kind"]) for r in rows], torch.bool),
            # The release comes at the end of the last level's hold.
            "after": len(levels) * int(hold_steps) + 1,
        }
        self._levels.use_staircase(levels, hold_steps)
        self._min_release_command = 0.0
        self.cfg.curriculum = False

    # --- acting, and the rope at the physics rate ---------------------------------------------

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        super()._pre_physics_step(actions)
        self._substep = 0
        self._tv.zero_()
        self._wobble.zero_()
        self._slack_count.zero_()

    def _step_forces(self) -> None:
        """UniFP's pushes for the free environments; the tension command for the roped ones."""
        free = ~self._rope_env
        if self.common_step_counter > self.cfg.force_start_step:
            commanded, external = self._forces.step(self.episode_length_buf)
            self._commands[:, interface.CMD_EE_FORCE] = torch.where(
                free.unsqueeze(-1), commanded, torch.zeros_like(commanded))
            self._push_force_w = torch.where(free.unsqueeze(-1), external, torch.zeros_like(external))
        held = self._rope.engaged
        intact = held & ~self._rope.released
        level = self._levels.step(intact, self._ceiling)
        self._cmd_level = torch.where(intact, level, self._cmd_level)
        noticed = (self._cmd_level - rope_cfg.DETECT_RAMP_N_PER_S * interface.POLICY_DT).clamp(min=0.0)
        self._cmd_level = torch.where(self._detected, noticed, self._cmd_level)
        self._cmd_level = torch.where(held, self._cmd_level, torch.zeros_like(self._cmd_level))
        command_yaw = interface.quat_rotate_inverse(
            self._base_yaw_quat(), self._cmd_level.unsqueeze(-1) * self._pull_dir)
        self._commands[:, interface.CMD_EE_FORCE] = torch.where(
            held.unsqueeze(-1), command_yaw,
            torch.where(self._rope_env.unsqueeze(-1), torch.zeros_like(command_yaw),
                        self._commands[:, interface.CMD_EE_FORCE]))

    def _apply_action(self) -> None:
        super()._apply_action()
        reaction = self._rope.step(self._tip_pos(), self._tool_vel_w())
        self._ee_force_w = self._push_force_w + reaction
        self._write_wrench()
        tension = self._rope.tension
        error = tension - self._cmd_level
        self._tv += (error - self._last_err).abs() * self._rope.engaged
        self._last_err = torch.where(self._rope.engaged, error, torch.zeros_like(error))
        self._err_mean = torch.where(self._rope.engaged, self._err_mean + (error - self._err_mean)
                                     * (interface.SIM_DT / rope_cfg.WOBBLE_TAU_S), torch.zeros_like(error))
        self._wobble += torch.square(error - self._err_mean) * self._rope.engaged
        self._slack_count += (self._rope.slack & (self._cmd_level >= rope_cfg.SLACK_COMMAND_N)).float()
        if self._substep < self._tension_sub.shape[1]:
            self._tension_sub[:, self._substep] = tension
        self._substep += 1

    def _command_w(self) -> torch.Tensor:
        return (self._cmd_level * self._rope.engaged).unsqueeze(-1) * self._pull_dir

    # --- engagement, release, and the held goal ------------------------------------------------

    def _engage_due(self) -> None:
        waited = self._since_reset - self._engage_step
        settled = self._tool_vel_w().norm(dim=-1) < hook_cfg.ENGAGE_SETTLE_SPEED_M_S
        due = (self._rope_env & ~self._rope.engaged & (waited >= 0)
               & (settled | (waited >= hook_cfg.ENGAGE_MAX_WAIT_STEPS)))
        if not bool(due.any()):
            return
        ids = due.nonzero(as_tuple=False).flatten()
        count = len(ids)
        point = self._tip_pos()[ids]
        base = self._robot.data.root_pos_w[ids] - self.scene.env_origins[ids]
        evaluating = self._evaluation is not None
        direction = fixture.pull_axes(point, base, yaw_noise_rad=0.0 if evaluating else rope_cfg.PULL_YAW_NOISE_RAD,
                                      elevation_rad=(0.0, 0.0) if evaluating else rope_cfg.PULL_ELEVATION_RAD)
        log_uniform = lambda span: torch.exp(math.log(span[0]) + torch.rand(count, device=self.device)
                                             * math.log(span[1] / span[0]))
        if evaluating:
            length = self._evaluation["length"][ids]
            stiffness = self._evaluation["stiffness"][ids]
            # Tilt the horizontal pull by the row's elevation (positive pulls upward).
            elevation = self._evaluation["elevation"][ids].unsqueeze(-1)
            up = torch.zeros_like(direction)
            up[:, 2] = 1.0
            direction = torch.cos(elevation) * direction + torch.sin(elevation) * up
        else:
            length = log_uniform(self.cfg.length_range)
            stiffness = log_uniform(self.cfg.stiffness_range)
        self._rope.engage(ids, point, direction, length=length, stiffness=stiffness,
                          damping=torch.full((count,), rope_cfg.ROPE_DAMPING, device=self.device))
        self._pull_dir[ids] = direction
        self._grip_point[ids] = point
        self._engaged_at[ids] = self._since_reset[ids]
        self._levels.reset(ids)
        self._cmd_level[ids] = 0.0
        self._last_err[ids] = 0.0
        self._err_mean[ids] = 0.0
        self._goals.hold_mask[ids] = True
        self._goals.hold_roll[ids] = self._goals.current_roll[ids]

    def _release_due(self) -> None:
        """Let planned anchors go; and let the task layer notice the ones whose delay has run."""
        since_engage = self._since_reset - self._engaged_at
        due = (self._rope.engaged & ~self._rope.released & self._release_planned
               & (since_engage >= self._release_after) & (self._cmd_level >= self._min_release_command))
        if bool(due.any()):
            ids = due.nonzero(as_tuple=False).flatten()
            self._rope.release(ids, self._release_kind[ids], self._release_residual[ids], self._release_ramp[ids])
            self._release_at[ids] = self._since_reset[ids]
        since_release = self._since_reset - self._release_at
        notice = (self._rope.released & ~self._detected & (self._detect_delay >= 0)
                  & (since_release >= self._detect_delay))
        if bool(notice.any()):
            ids = notice.nonzero(as_tuple=False).flatten()
            self._detected[ids] = True
            # The goal goes back to the grip placement relative to the body, from where it is held now, along
            # UniFP's own trajectory; once there, UniFP's free-space goals take over.
            goals = self._goals
            goals.start[ids] = goals.hold_goal[ids]
            goals.start_roll[ids] = goals.hold_roll[ids]
            goals.aim(ids, self._handle_sphere[ids], goals.hold_roll[ids])
            goals.hold_mask[ids] = False

    def _hold_goals_on_grip(self) -> None:
        """Express each held grip point in the goal sphere of the base as it stands now."""
        held = self._goals.hold_mask
        if not bool(held.any()):
            return
        local = interface.quat_rotate_inverse(self._base_yaw_quat(), self._grip_point - self._goal_centre())
        self._goals.hold_goal = torch.where(held.unsqueeze(-1), interface.cart2sphere(local), self._goals.hold_goal)

    def _get_rewards(self) -> torch.Tensor:
        self._since_reset += 1
        self._engage_due()
        self._release_due()
        self._hold_goals_on_grip()
        total = super()._get_rewards()
        intact = self._rope.engaged & ~self._rope.released
        command = self._command_w()
        along = (self._rope.applied_by_robot() * self._pull_dir).sum(-1)
        error = (along - self._cmd_level).abs()
        frontier = intact & (self._cmd_level >= rope_cfg.LEVEL_FRONTIER_FRACTION * self._ceiling)
        self._err_sum += error * frontier
        self._cmd_sum += command.norm(dim=-1) * frontier
        self._engaged_steps += intact.float()
        self._ep_tv_sum += self._tv * intact
        self._ep_slack_sum += (self._slack_count / self._tension_sub.shape[1]) * intact
        self._ep_intact_steps += intact.float()
        return total

    def _task_state(self):
        state = super()._task_state()
        held = self._rope.engaged
        intact = held & ~self._rope.released
        since_release = self._since_reset - self._release_at
        state.fixture_engaged = intact.float()
        state.rope_held = held.float()
        state.fixture_applied_w = self._rope.applied_by_robot()
        state.fixture_command_w = self._command_w()
        state.fixture_axis_w = self._pull_dir
        state.fixture_active = held.float()
        state.tool_vel_w = self._tool_vel_w()
        state.rope_tv_n = self._tv
        state.rope_slack_fraction = self._slack_count / self._tension_sub.shape[1]
        state.rope_wobble_n2 = self._wobble / self._tension_sub.shape[1]
        state.rope_recovering = (self._rope.released & (since_release < self._recovery_steps)).float()
        state.base_lin_vel_w = self._robot.data.root_lin_vel_w
        state.rope_fallen = (self._fallen_now & self._rope_env).float()
        state.rope_arm_line_w = self._tip_pos() - (self._robot.data.body_pos_w[:, self._shoulder_body]
                                                   - self.scene.env_origins)
        state.rope_line_w = -self._rope.direction
        return state

    def _privileged_extra(self) -> torch.Tensor:
        """The rope as the critic sees it, in the 22-wide `mass_params` block."""
        held = self._rope.engaged
        h = held.float().unsqueeze(-1)
        yaw = self._base_yaw_quat()
        since_release = (self._since_reset - self._release_at).float() * interface.POLICY_DT
        out = torch.zeros(self.num_envs, rope_cfg.PRIVILEGED_WIDTH, device=self.device)
        out[:, 0:3] = interface.quat_rotate_inverse(yaw, self._pull_dir) * h
        out[:, 3:4] = h
        out[:, 4] = (held & ~self._rope.released).float()
        out[:, 5] = self._rope.released.float()
        out[:, 6] = self._detected.float()
        out[:, 7] = torch.where(self._rope.released, (since_release / 3.0).clamp(max=1.0), torch.zeros_like(since_release))
        out[:, 8] = self._rope.tension * 0.01
        out[:, 9] = self._cmd_level * 0.01
        out[:, 10] = self._ceiling * 0.01
        out[:, 11] = self._rope.stiffness * held / 6000.0
        out[:, 12] = self._rope.stretch * 10.0
        out[:, 13] = self._rope.slack.float()
        capacity = torch.where(torch.isfinite(self._rope.capacity), self._rope.capacity * 0.01,
                               torch.ones_like(self._rope.capacity))
        out[:, 14] = capacity * held
        out[:, 15:18] = interface.quat_rotate_inverse(yaw, self._rope.direction) * h
        out[:, 18] = self._rope.length * held / 2.0
        out[:, 19:22] = interface.quat_rotate_inverse(yaw, self._grip_point - self._tip_pos()) * 10.0 * h
        return out

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        fallen, timed_out = super()._get_dones()
        trunk = self._contact_forces(self._trunk_contacts).norm(dim=-1).max(dim=1).values > rope_cfg.TRUNK_CONTACT_N
        fallen = fallen | (self._rope_env & trunk)
        self._fallen_now = fallen.clone()
        # Kept through the reset that follows, so an evaluator stepping from outside can still read why it ended.
        self._term_fallen = fallen.clone()
        self._term_released = self._rope.released.clone()
        return fallen, timed_out

    def _resample_velocity_commands(self, env_ids: torch.Tensor) -> None:
        """UniFP's draw, except that a roped episode stands."""
        super()._resample_velocity_commands(env_ids)
        standing = env_ids[self._rope_env[env_ids]]
        self._commands[standing, :3] = 0.0

    # --- resetting ------------------------------------------------------------------------

    def _update_curriculum(self, env_ids: torch.Tensor) -> None:
        """Fold finished episodes into the force-error average; raise the ceiling when it is low. And log."""
        log = self.extras.setdefault("log", {})
        roped = env_ids[self._rope_env[env_ids] & (self._ep_intact_steps[env_ids] > 0)]
        if len(roped) > 0:
            steps = self._ep_intact_steps[roped].sum().clamp(min=1.0)
            log["Rope/tension_tv_n_per_step"] = float(self._ep_tv_sum[roped].sum() / steps)
            log["Rope/slack_fraction"] = float(self._ep_slack_sum[roped].sum() / steps)
        released = env_ids[self._rope_env[env_ids] & self._rope.released[env_ids]]
        if len(released) > 0:
            fell = self._fallen_now[released] | self._term_fallen[released]
            log["Rope/released_fell_fraction"] = float(fell.float().mean())
        finished = env_ids[self._rope_env[env_ids]]
        if len(finished) > 0:
            log["Rope/fell_fraction"] = float(self._term_fallen[finished].float().mean())
            log["Rope/released_fraction"] = float(self._rope.released[finished].float().mean())
        counted = env_ids[self._engaged_steps[env_ids] >= rope_cfg.CURRICULUM_MIN_ENGAGED_STEPS]
        if self.cfg.curriculum_rule == "ema":
            self._update_curriculum_ema(counted, log)
            return
        if len(counted) > 0:
            commanded = float(self._cmd_sum[counted].sum())
            if commanded > 1.0:
                rel = float(self._err_sum[counted].sum()) / commanded
                self._window_err += float(self._err_sum[counted].sum())
                self._window_cmd += commanded
                log["Rope/relative_force_error"] = rel
        if self._last_promotion_step == 0 and self.common_step_counter > 0:
            # The first window opens when stepping starts (on a resume the counter starts at the resumed iteration's).
            self._last_promotion_step = self.common_step_counter
        window = self._window_err / self._window_cmd if self._window_cmd > 1.0 else None
        wait = rope_cfg.CURRICULUM_MIN_ITERATIONS * task_cfg.NUM_STEPS_PER_ENV
        if (self.cfg.curriculum and window is not None
                and self.common_step_counter - self._last_promotion_step >= wait):
            if window < rope_cfg.CURRICULUM_PROMOTE_REL_ERR and self._ceiling < self.cfg.force_ceiling_max_n:
                self._ceiling = min(self._ceiling + rope_cfg.CURRICULUM_STEP_N, self.cfg.force_ceiling_max_n)
                self._last_promotion_step = self.common_step_counter
                self._window_err, self._window_cmd = 0.0, 0.0
            elif self.common_step_counter - self._last_promotion_step >= 2 * wait:
                # Not promoted: keep judging on the most recent evidence, half a window at a time.
                self._window_err, self._window_cmd = 0.5 * self._window_err, 0.5 * self._window_cmd
                self._last_promotion_step += wait // 2
        log["Rope/force_ceiling_n"] = self._ceiling
        if window is not None:
            log["Rope/relative_force_error_window"] = window

    def _update_curriculum_ema(self, counted: torch.Tensor, log: dict) -> None:
        """v1-v3's rule, unchanged: an average over reset batches, restarted from one batch after each promotion."""
        if len(counted) > 0:
            commanded = float(self._cmd_sum[counted].sum())
            if commanded > 1.0:
                rel = float(self._err_sum[counted].sum()) / commanded
                self._rel_err_ema = rel if self._rel_err_ema is None else (
                    (1 - rope_cfg.CURRICULUM_EMA) * self._rel_err_ema + rope_cfg.CURRICULUM_EMA * rel)
                log["Rope/relative_force_error"] = rel
        wait = rope_cfg.CURRICULUM_MIN_ITERATIONS * task_cfg.NUM_STEPS_PER_ENV
        if (self.cfg.curriculum and self._rel_err_ema is not None
                and self._rel_err_ema < rope_cfg.CURRICULUM_PROMOTE_REL_ERR
                and self._ceiling < self.cfg.force_ceiling_max_n
                and self.common_step_counter - self._last_promotion_step >= wait):
            self._ceiling = min(self._ceiling + rope_cfg.CURRICULUM_STEP_N, self.cfg.force_ceiling_max_n)
            self._last_promotion_step = self.common_step_counter
            self._rel_err_ema = None
        log["Rope/force_ceiling_n"] = self._ceiling
        if self._rel_err_ema is not None:
            log["Rope/relative_force_error_ema"] = self._rel_err_ema

    def _draw_releases(self, ids: torch.Tensor) -> None:
        count = len(ids)
        rand = lambda: torch.rand(count, device=self.device)
        span = lambda r: r[0] + rand() * (r[1] - r[0])
        if self._evaluation is not None:
            plan = self._evaluation
            self._release_planned[ids] = plan["planned"][ids]
            self._release_after[ids] = plan["after"]
            self._release_kind[ids] = plan["kind"][ids]
            self._release_residual[ids] = plan["residual"][ids]
            self._release_ramp[ids] = plan["ramp"][ids]
            self._detect_delay[ids] = plan["detect"][ids]
            return
        self._release_planned[ids] = rand() < self.cfg.release_prob
        self._release_after[ids] = (span(rope_cfg.RELEASE_AFTER_S) / interface.POLICY_DT).long()
        snap_p, slip_p, _ = rope_cfg.RELEASE_KIND_PROBS
        pick = rand()
        code = lambda value: torch.full((count,), value, dtype=torch.long, device=self.device)
        kind = torch.where(pick < snap_p, code(rope.SNAP),
                           torch.where(pick < snap_p + slip_p, code(rope.SLIP), code(rope.FADE)))
        self._release_kind[ids] = kind
        self._release_residual[ids] = torch.where(kind == rope.SLIP, span(rope_cfg.SLIP_RESIDUAL_FRACTION),
                                                  torch.zeros(count, device=self.device))
        ramp_s = torch.where(kind == rope.SLIP, span(rope_cfg.SLIP_RAMP_S),
                             torch.where(kind == rope.FADE, span(rope_cfg.FADE_RAMP_S), torch.zeros(count, device=self.device)))
        self._release_ramp[ids] = (ramp_s / interface.SIM_DT).round().long()
        delay = (span(rope_cfg.DETECT_DELAY_S) / interface.POLICY_DT).round().long()
        self._detect_delay[ids] = torch.where(rand() < self.cfg.detect_never_prob, torch.full_like(delay, -1), delay)

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        self._update_curriculum(env_ids)
        count = len(env_ids)
        if self._evaluation is not None:
            self._rope_env[env_ids] = True
        elif self.cfg.rope_fraction <= 0.0:
            # Drawing nothing keeps the global RNG where UniFP's task has it, so its frozen manifests replay.
            self._rope_env[env_ids] = False
        else:
            self._rope_env[env_ids] = torch.rand(count, device=self.device) < self.cfg.rope_fraction
        # Drawn before the parent reset, which resamples the velocity command and must see it stand.
        super()._reset_idx(env_ids)

        self._rope.reset(env_ids)
        self._levels.reset(env_ids)
        self._push_force_w[env_ids] = 0.0
        self._ee_force_w[env_ids] = 0.0
        self._cmd_level[env_ids] = 0.0
        self._since_reset[env_ids] = 0
        self._engaged_at[env_ids] = 0
        self._release_at[env_ids] = -1
        self._detected[env_ids] = False
        self._last_err[env_ids] = 0.0
        self._err_mean[env_ids] = 0.0
        self._wobble[env_ids] = 0.0
        self._tv[env_ids] = 0.0
        self._slack_count[env_ids] = 0.0
        self._fallen_now[env_ids] = False
        for buffer in (self._err_sum, self._cmd_sum, self._engaged_steps, self._ep_tv_sum, self._ep_slack_sum,
                       self._ep_intact_steps):
            buffer[env_ids] = 0.0
        self._goals.hold_mask[env_ids] = False

        ids = env_ids[self._rope_env[env_ids]]
        if len(ids) > 0:
            if self._evaluation is not None:
                handles = self._evaluation["spheres"][ids]
                delay = torch.full((len(ids),), rope_cfg.EVAL_ENGAGE_DELAY_STEPS, device=self.device)
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
            self._draw_releases(ids)
        free = env_ids[~self._rope_env[env_ids]]
        self._release_planned[free] = False
