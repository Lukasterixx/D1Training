"""The goal-commanded task: the robot holds a handle, and is told only where it should go.

The pull task (`hook_env.py`) commands a force. This one commands the *outcome* -- a reference point
sliding along a mechanism's path, the way a task layer that knows the box's geometry from its tags
would plan it -- and leaves the force to the policy. What the mechanism resists with (a spring, a
closer's preload, stiction, a latch that snaps) is drawn per episode and never observed. The policy
has UniFP's proprioceptive history and force estimator to work it out, and the arm-margin term
(`hook_rewards.arm_torque_margin`) to make the body carry what the arm cannot.

What an episode is, for the `mechanism_fraction` of environments that get one:

  1. The goal slides to a handle placement, as in the pull task (`hook_cfg.HANDLE_*`).
  2. Once the tool has settled there, it grasps the handle wherever it actually is, and a mechanism
     is built around that point: a slide or a hinge, opening in a drawn direction, with a drawn
     travel shortened until the whole path is somewhere the robot can work (`mech_cfg.PATH_*`).
  3. The goal becomes the reference point on the path (`mechanism.TravelGoal`): open most of the
     way, hold, then somewhere else along it, closing included. The velocity command is zero.
  4. The mechanism's reaction through the grasp is the external force UniFP's estimator is trained
     on. Tearing the handle out of the claw, or falling, ends the episode.

The rest reach through free space with no pushes. **UniFP's virtual spring is made rigid** for every
environment (`mech_cfg.STIFF_KP`): in UniFP's task a push displaces the target and the policy is paid
to give way; here the target is the goal and a resisting force is something to overcome.

**No width changes.** Actor 32 x 76, critic 3 x 153, 18 actions: a UniFP or pull-task checkpoint loads
and continues. The mechanism as the critic sees it goes in the `mass_params` block.
"""
from __future__ import annotations

import math

import torch

from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.utils import configclass

from unifp_isaaclab import interface

from . import fixture, hook_cfg, hook_rewards, mech_cfg, mech_rewards, mechanism, task_cfg
from .env import Go2D1PosForceEnv
from .env_cfg import Go2D1PosForceEnvCfg
from .hook_env import HeldGoals


@configclass
class Go2D1MechEnvCfg(Go2D1PosForceEnvCfg):
    """UniFP's robot and observation, holding a mechanism; the roll objective on by default."""

    roll_objective: bool = True
    #: Unused: this task applies no UniFP pushes (`Go2D1MechEnv._step_forces`).
    force_start_step: int = 0
    mechanism_fraction: float = mech_cfg.MECHANISM_FRACTION
    peak_ceiling_n: float = mech_cfg.CURRICULUM_START_N
    peak_ceiling_max_n: float = mech_cfg.CURRICULUM_MAX_N
    curriculum: bool = True
    #: The hierarchical variant (v3): the task layer's force law writes the force command in training
    #: too (`set_force_law`), and the pull task's force-tracking term joins the reward
    #: (`mech_cfg.LAW_WEIGHT_CHANGES`). Off, the policy gets the goal alone.
    force_law: bool = False
    #: The law's integral bleed time constant, s (`set_force_law`); None for none.
    force_law_bleed_s: float | None = None
    #: "v1" as F-108 trained; "v2" adds the over-force limit, a heavier lunge price and a cap drawn per episode
    #: (`mech_cfg.LAW_V2_*`); "v3" is v2 without the outcome reward, a pure force-follower (`LAW_V3_WEIGHT_CHANGES`).
    law_variant: str = "v1"
    #: Which way training mechanisms open: pull, push, side, up (`mech_cfg.OPENING_NAMES`), summing to one.
    opening_probs: tuple[float, float, float, float] = mech_cfg.OPENING_PROBS
    force_law_gains: tuple[float, float, float] = (mech_cfg.FORCE_LAW_KP_N_PER_M, mech_cfg.FORCE_LAW_KI_N_PER_M_S,
                                                   mech_cfg.FORCE_LAW_MAX_N)
    #: The D1's own links, for the evaluator (see `hook_env.Go2D1HookEnvCfg.arm_contact_sensor`).
    arm_contact_sensor: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/D1/.*", history_length=0, update_period=0.0,
        track_air_time=False)


class Go2D1MechEnv(Go2D1PosForceEnv):
    cfg: Go2D1MechEnvCfg

    def __init__(self, cfg: Go2D1MechEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, device = self.num_envs, self.device
        self._goals = HeldGoals.adopt(self._goals)
        self._mech = mechanism.Mechanism(n, device, grasp_damping=mech_cfg.GRASP_DAMPING)
        self._travel_goal = mechanism.TravelGoal(
            n, device, speed_m_s=mech_cfg.GOAL_SPEED_M_S, hold_s=mech_cfg.GOAL_HOLD_S,
            first_fraction=mech_cfg.GOAL_FIRST_FRACTION, dt=interface.POLICY_DT)
        zeros = lambda *shape: torch.zeros(*shape, device=device)
        flags = lambda: torch.zeros(n, dtype=torch.bool, device=device)
        self._mech_env = flags()
        self._ever_grasped = flags()
        self._handle_sphere = zeros(n, 3)
        self._engage_step = torch.zeros(n, dtype=torch.long, device=device)
        self._since_reset = torch.zeros(n, dtype=torch.long, device=device)
        self._grasp_step = torch.zeros(n, dtype=torch.long, device=device)
        self._torn_now = flags()
        self._term_torn = flags()
        self._term_fallen = flags()
        self._offset = torch.as_tensor(self.cfg.tool_offset_m, device=device).expand(n, 3)
        #: Per episode: the peak force the mechanism needs, the ceiling it was drawn under, which way
        #: it opens (index into `mech_cfg.OPENING_NAMES`), and the furthest the handle has got.
        self._peak_n = zeros(n)
        self._draw_ceiling = zeros(n)
        self._opening_kind = torch.zeros(n, dtype=torch.long, device=device)
        self._max_s = zeros(n)

        self._ceiling = float(cfg.peak_ceiling_n)
        self._opened_ema: float | None = None
        self._last_promotion_step = 0
        #: Evaluation plan (`set_evaluation`): every episode a mechanism, as the plan says.
        self._evaluation: dict | None = None
        #: The force-commanded baseline (`set_force_law`): a task-layer law turning the handle's lag
        #: behind its reference into a force command, for a policy trained to follow one.
        self._force_law: dict | None = None
        self._lag_integral = zeros(n)
        #: The law's command as of this policy step, world frame (zero without the law).
        self._law_command_w = zeros(n, 3)

        self._gripper_force_kp.fill_(mech_cfg.STIFF_KP)
        self._weights.update(mech_cfg.scaled_weights())
        self._terms.update(mech_rewards.TERMS)
        #: The law's cap per environment, N: the law's `max` everywhere, or drawn per grasp in law v2 training.
        self._law_max = zeros(n)
        self._law_max_range: tuple[float, float] | None = None
        if cfg.force_law:
            v2 = cfg.law_variant in ("v2", "v3")
            self.set_force_law(*cfg.force_law_gains,
                               bleed_s=cfg.force_law_bleed_s if cfg.force_law_bleed_s is not None
                               else (mech_cfg.LAW_V2_BLEED_S if v2 else None))
            self._law_max_range = mech_cfg.LAW_V2_MAX_RANGE_N if v2 else None
            changes = {"v1": mech_cfg.LAW_WEIGHT_CHANGES, "v2": mech_cfg.LAW_V2_WEIGHT_CHANGES,
                       "v3": mech_cfg.LAW_V3_WEIGHT_CHANGES}[cfg.law_variant]
            self._weights.update(mech_cfg.scaled_weights(changes))
            self._terms["fixture_force_tracking"] = hook_rewards.fixture_force_tracking
        for name in self._weights:
            self._episode_sums.setdefault(name, zeros(n))

    def _setup_scene(self):
        super()._setup_scene()
        self._arm_contact = ContactSensor(self.cfg.arm_contact_sensor)
        self.scene.sensors["arm_contact"] = self._arm_contact

    # --- evaluation ---------------------------------------------------------------------------

    def set_evaluation(self, handles: torch.Tensor, kinds: torch.Tensor, levels: torch.Tensor,
                       engage_delay_steps: int = 15, conditions: dict | None = None) -> None:
        """Fix every episode's placement, mechanism and resistance. Call before `reset()`.

        `handles` (num_envs, 3) in goal-sphere coordinates; `kinds` (num_envs,) indexes
        `mech_cfg.EVAL_KINDS` in order; `levels` (num_envs,) is each mechanism's peak force, N.
        The target is fully open, held to the end. `conditions` (`mech_cfg.DEV_CONDITIONS` by default,
        `TEST_CONDITIONS` for the held-out set) fix the moving mass, grasp stiffness, reference speed and
        damping. The curriculum is off.
        """
        conditions = dict(mech_cfg.DEV_CONDITIONS if conditions is None else conditions)
        self._evaluation = {"handles": handles.to(self.device), "kinds": kinds.to(self.device),
                            "levels": levels.to(self.device).float(), "engage_delay": int(engage_delay_steps),
                            "conditions": conditions}
        self._travel_goal.use_plan(1.0, conditions["speed_m_s"])
        self.cfg.curriculum = False

    def set_force_law(self, kp_n_per_m: float, ki_n_per_m_s: float, max_n: float, bleed_s: float | None = None) -> None:
        """Evaluate a force-commanded policy on the same mechanisms (the baseline for this task).

        The pull task's policy follows a force command with the goal held on the handle. Here a
        task layer that knows only the geometry and the handle's position (from tags) makes one:
        ``F = kp lag + ki integral(lag)`` along the path, `lag` the reference minus the handle, the
        integral kept only while it pushes the same way, clamped to `max_n`. The goal is put on the
        handle itself, as the pull task trained, and the rigid target is set back to UniFP's 200 N/m,
        the stiffness that policy's force channel was trained through.

        `bleed_s`, if given, lets the integral decay with that time constant while the handle is within
        `mech_cfg.FORCE_LAW_ARRIVED_M` of its reference: without it the integral holds whatever force it
        built up against a stop for as long as the goal stays there (the 12 torn buttons of F-108).
        """
        self._force_law = {"kp": float(kp_n_per_m), "ki": float(ki_n_per_m_s), "max": float(max_n),
                           "bleed_s": None if bleed_s is None else float(bleed_s)}
        self._law_max.fill_(float(max_n))
        self._gripper_force_kp.fill_(task_cfg.GRIPPER_FORCE_KP)

    # --- the mechanism, stepped at the physics rate --------------------------------------------

    def _tool_vel_w(self) -> torch.Tensor:
        """World velocity of the controlled point: the body's plus rotation about its offset."""
        quat = self._robot.data.body_quat_w[:, self._tip_body]
        arm = interface.quat_apply(quat, self._offset)
        return (self._robot.data.body_lin_vel_w[:, self._tip_body]
                + torch.cross(self._robot.data.body_ang_vel_w[:, self._tip_body], arm, dim=-1))

    def _apply_action(self) -> None:
        super()._apply_action()
        self._ee_force_w = self._mech.step(self._tip_pos(), self._tool_vel_w(), interface.SIM_DT)
        self._write_wrench()

    def _step_forces(self) -> None:
        """No UniFP pushes, and no force command unless the baseline's law is on (`set_force_law`)."""
        if self._force_law is None:
            return
        law = self._force_law
        held = self._mech.grasped
        lag = (self._travel_goal.ref - self._mech.s) * held
        integral = self._lag_integral + lag * interface.POLICY_DT
        # Anti-windup: the integral never holds a push the other way from the one the lag asks for.
        integral = torch.where(integral * lag < 0.0, torch.zeros_like(integral), integral)
        if law.get("bleed_s"):
            arrived = lag.abs() < mech_cfg.FORCE_LAW_ARRIVED_M
            integral = torch.where(arrived, integral * math.exp(-interface.POLICY_DT / law["bleed_s"]), integral)
        self._lag_integral = integral * held
        force = torch.maximum(torch.minimum(law["kp"] * lag + law["ki"] * self._lag_integral, self._law_max), -self._law_max)
        self._law_command_w = force.unsqueeze(-1) * self._mech.tangent() * held.unsqueeze(-1)
        self._commands[:, interface.CMD_EE_FORCE] = interface.quat_rotate_inverse(
            self._base_yaw_quat(), self._law_command_w)

    # --- grasping, and the goal on the path ---------------------------------------------------

    def _engage_due(self) -> None:
        waited = self._since_reset - self._engage_step
        settled = self._tool_vel_w().norm(dim=-1) < hook_cfg.ENGAGE_SETTLE_SPEED_M_S
        due = (self._mech_env & ~self._ever_grasped & (waited >= 0)
               & (settled | (waited >= hook_cfg.ENGAGE_MAX_WAIT_STEPS)))
        if not bool(due.any()):
            return
        ids = due.nonzero(as_tuple=False).flatten()
        point = self._tip_pos()[ids]
        draw = self._draw_evaluation(ids, point) if self._evaluation is not None else self._draw(ids, point)
        self._mech.grasp(ids, point, **draw)
        if self._law_max_range is not None and self._evaluation is None:
            low, high = self._law_max_range
            self._law_max[ids] = low + torch.rand(len(ids), device=self.device) * (high - low)
        self._travel_goal.start(ids, self._mech.travel[ids])
        self._ever_grasped[ids] = True
        self._grasp_step[ids] = self._since_reset[ids]
        self._goals.hold_mask[ids] = True
        self._goals.hold_roll[ids] = self._goals.current_roll[ids]

    def _directions(self, ids: torch.Tensor, point: torch.Tensor, noise: bool,
                    outward: torch.Tensor | None = None) -> dict[str, torch.Tensor]:
        """Unit opening directions of each kind at `point`: pull, push, side, up.

        "side" is left or right at random, unless `outward` (N,) bool gives it: True for the robot's
        left (+y of a robot facing the handle), which is outward for a handle on the left."""
        base = self._robot.data.root_pos_w[ids] - self.scene.env_origins[ids]
        yaw = mech_cfg.YAW_NOISE_RAD if noise else 0.0
        elevation = mech_cfg.ELEVATION_RAD if noise else (0.0, 0.0)
        pull = fixture.pull_axes(point, base, yaw_noise_rad=yaw, elevation_rad=elevation)
        push = fixture.press_axes(point, base, yaw_noise_rad=yaw, elevation_rad=elevation,
                                  down_prob=mech_cfg.PUSH_DOWN_PROB if noise else 0.0)
        back = fixture.pull_axes(point, base, yaw_noise_rad=0.0, elevation_rad=(0.0, 0.0))
        up = torch.zeros_like(back)
        up[:, 2] = 1.0
        side = torch.cross(back, up, dim=-1)
        flip = torch.rand(len(ids), device=self.device) < 0.5 if outward is None else ~outward
        side = torch.where(flip.unsqueeze(-1), -side, side)
        return {"pull": pull, "push": push, "side": side, "up": up}

    def _fit_travel(self, ids: torch.Tensor, point: torch.Tensor, geometry: dict,
                    travel: torch.Tensor) -> torch.Tensor:
        """Shorten each path until every checked point on it is somewhere the robot can work."""
        centre = self._goal_centre()[ids]
        yaw = self._base_yaw_quat()[ids]
        radial, hinge_centre = mechanism.hinge_geometry(point, geometry["opening"], geometry["axis"],
                                                        geometry["radius"])
        for _ in range(mech_cfg.PATH_SHRINK_TRIES):
            ok = torch.ones(len(ids), dtype=torch.bool, device=self.device)
            for fraction in mech_cfg.PATH_CHECK_FRACTIONS:
                p = mechanism.path_position(geometry["hinge"], point, geometry["opening"], hinge_centre,
                                            geometry["axis"], radial, geometry["radius"], travel * fraction)
                local = interface.quat_rotate_inverse(yaw, p - centre)
                reach = local.norm(dim=-1)
                clear = (local[:, 0] >= mech_cfg.PATH_CLEAR_AHEAD_M) | (local[:, 1].abs() >= mech_cfg.PATH_CLEAR_SIDE_M)
                ok &= ((reach >= mech_cfg.PATH_RADIUS_M[0]) & (reach <= mech_cfg.PATH_RADIUS_M[1]) & clear
                       & (p[:, 2] >= mech_cfg.PATH_MIN_HEIGHT_M))
            if bool(ok.all()):
                break
            travel = torch.where(ok, travel, (travel * mech_cfg.PATH_SHRINK).clamp(min=mech_cfg.PATH_MIN_TRAVEL_M))
        return travel

    def _draw(self, ids: torch.Tensor, point: torch.Tensor) -> dict:
        """A training mechanism for each environment in `ids`, grasped at `point`."""
        count, device = len(ids), self.device
        rand = lambda: torch.rand(count, device=device)
        span = lambda r: r[0] + rand() * (r[1] - r[0])
        log_span = lambda r: torch.exp(math.log(r[0]) + rand() * math.log(r[1] / r[0]))

        kind = torch.multinomial(torch.tensor(self.cfg.opening_probs, device=device), count, replacement=True)
        directions = self._directions(ids, point, noise=True)
        opening = torch.zeros(count, 3, device=device)
        slide_travel = torch.zeros(count, device=device)
        for index, name in enumerate(mech_cfg.OPENING_NAMES):
            mask = (kind == index).unsqueeze(-1)
            opening = torch.where(mask, directions[name], opening)
            slide_travel = torch.where(mask[:, 0], span(mech_cfg.SLIDE_TRAVEL_M[name]), slide_travel)
        hinge = rand() < mech_cfg.HINGE_PROB
        axis = mechanism.hinge_axes(opening, rand() < mech_cfg.HINGE_VERTICAL_PROB, rand() < 0.5)
        radius = span(mech_cfg.HINGE_RADIUS_M)
        travel = torch.where(hinge, radius * span(mech_cfg.HINGE_ANGLE_RAD), slide_travel)
        geometry = {"hinge": hinge, "opening": opening, "axis": axis, "radius": radius}
        travel = self._fit_travel(ids, point, geometry, travel)

        frontier = mech_cfg.FRONTIER_FRACTION + (1.0 - mech_cfg.FRONTIER_FRACTION) * rand()
        peak = (torch.where(rand() < mech_cfg.FRONTIER_PROB, frontier, rand()) * self._ceiling).clamp(
            min=mech_cfg.PEAK_MIN_N)
        profile = mechanism.resistance_profile(
            peak, latch=rand() < mech_cfg.LATCH_PROB, preload_fraction=span(mech_cfg.PRELOAD_FRACTION),
            kinetic_fraction=span(mech_cfg.KINETIC_FRACTION), weights=torch.rand(count, 3, device=device))
        self._peak_n[ids] = peak
        self._draw_ceiling[ids] = self._ceiling
        self._opening_kind[ids] = kind
        return {**geometry, "travel": travel, "mass": log_span(mech_cfg.MASS_KG),
                "spring_n": profile["spring_closed_n"],
                "spring_k": (profile["spring_open_n"] - profile["spring_closed_n"]) / travel,
                "damping": span(mech_cfg.DAMPING_N_S_M), "friction_static": profile["friction_static"],
                "friction_kinetic": profile["friction_kinetic"], "latch_n": profile["latch_n"],
                "grasp_k": log_span(mech_cfg.GRASP_STIFFNESS_RANGE),
                "grip_n": torch.full((count,), mech_cfg.GRIP_N, device=device)}

    def _draw_evaluation(self, ids: torch.Tensor, point: torch.Tensor) -> dict:
        """The planned mechanism for each environment in `ids`: exact directions, fixed travel."""
        count, device = len(ids), self.device
        plan = self._evaluation
        kinds, levels = plan["kinds"][ids], plan["levels"][ids]
        bearing = plan["handles"][ids, 2]
        directions = self._directions(ids, point, noise=False, outward=bearing >= 0.0)
        full = lambda value: torch.full((count,), float(value), device=device)
        out = {"hinge": torch.zeros(count, dtype=torch.bool, device=device), "opening": torch.zeros(count, 3, device=device),
               "radius": full(1.0), "travel": full(0.0), "weights": torch.zeros(count, 3, device=device),
               "preload": full(0.0), "kinetic": full(0.0),
               "vertical": torch.ones(count, dtype=torch.bool, device=device),
               "outward": torch.ones(count, dtype=torch.bool, device=device)}
        for index, spec in enumerate(mech_cfg.EVAL_KINDS.values()):
            mask = kinds == index
            out["hinge"] = torch.where(mask, torch.full_like(out["hinge"], spec["hinge"]), out["hinge"])
            out["opening"] = torch.where(mask.unsqueeze(-1), directions[spec["opening"]], out["opening"])
            if spec["hinge"]:
                out["vertical"] &= ~mask | (spec["axis"] == "vertical")
                out["outward"] &= ~mask | (spec["hinge_side"] == "outward")
                out["radius"] = torch.where(mask, full(spec["radius_m"]), out["radius"])
                out["travel"] = torch.where(mask, full(spec["radius_m"] * spec["angle_rad"]), out["travel"])
            else:
                out["travel"] = torch.where(mask, full(spec["travel_m"]), out["travel"])
            out["weights"] = torch.where(mask.unsqueeze(-1), torch.tensor(spec["weights"], device=device), out["weights"])
            out["preload"] = torch.where(mask, full(spec["preload"]), out["preload"])
            out["kinetic"] = torch.where(mask, full(spec["kinetic"]), out["kinetic"])
        # A door swings outward, away from the robot's centreline: the hinge goes on the handle's side
        # (flipping a vertical axis moves it to the robot's left). A lid's level hinge stays beyond it.
        axis = mechanism.hinge_axes(out["opening"], out["vertical"], out["outward"] & (bearing >= 0.0))
        profile = mechanism.resistance_profile(levels, latch=out["weights"][:, 2] > 0.0,
                                               preload_fraction=out["preload"], kinetic_fraction=out["kinetic"],
                                               weights=out["weights"])
        self._peak_n[ids] = levels
        self._draw_ceiling[ids] = levels
        travel = out["travel"]
        return {"hinge": out["hinge"], "opening": out["opening"], "axis": axis, "radius": out["radius"],
                "travel": travel, "mass": full(plan["conditions"]["mass_kg"]),
                "spring_n": profile["spring_closed_n"],
                "spring_k": (profile["spring_open_n"] - profile["spring_closed_n"]) / travel,
                "damping": full(plan["conditions"]["damping_n_s_m"]), "friction_static": profile["friction_static"],
                "friction_kinetic": profile["friction_kinetic"], "latch_n": profile["latch_n"],
                "grasp_k": full(plan["conditions"]["grasp_k"]), "grip_n": full(mech_cfg.GRIP_N)}

    def _hold_goals_on_path(self) -> None:
        """Put each held goal on the path's reference point, in the goal sphere of the base now."""
        held = self._goals.hold_mask
        if not bool(held.any()):
            return
        # The baseline holds the goal on the handle and asks for force; this task asks for the reference.
        point = self._mech.position(None if self._force_law is not None else self._travel_goal.ref)
        local = interface.quat_rotate_inverse(self._base_yaw_quat(), point - self._goal_centre())
        self._goals.hold_goal = torch.where(held.unsqueeze(-1), interface.cart2sphere(local),
                                            self._goals.hold_goal)

    def _get_rewards(self) -> torch.Tensor:
        self._since_reset += 1
        self._engage_due()
        grasped = self._mech.grasped
        self._travel_goal.step(grasped, self._mech.travel)
        self._hold_goals_on_path()
        total = super()._get_rewards()
        self._max_s = torch.where(grasped, torch.maximum(self._max_s, self._mech.s), self._max_s)
        return total

    def _task_state(self):
        state = super()._task_state()
        grasped = self._mech.grasped.float()
        state.fixture_engaged = grasped
        state.mech_error = (self._mech.s - self._travel_goal.ref) * grasped
        state.mech_speed = self._mech.v * grasped
        state.mech_speed_limit = self._travel_goal.speed + mech_cfg.OVERSPEED_MARGIN_M_S
        state.mech_torn = self._torn_now.float()
        state.mech_force = self._mech.force_on_tool.norm(dim=-1)
        state.mech_drive = self._mech.drive_n * grasped
        state.mech_travel = self._mech.travel
        state.mech_command = (self._law_command_w * self._mech.tangent()).sum(-1)
        # Read only by the hierarchical variant's force-tracking term (`hook_rewards.fixture_force_tracking`).
        state.fixture_applied_w = self._mech.applied_by_robot()
        state.fixture_command_w = self._law_command_w
        state.fixture_axis_w = self._mech.tangent()
        state.tool_vel_w = self._tool_vel_w()
        return state

    def _privileged_extra(self) -> torch.Tensor:
        """The mechanism as the critic sees it, in the `mass_params` block. Zero where none is held."""
        m = self._mech
        held = m.grasped.float().unsqueeze(-1)
        yaw = self._base_yaw_quat()
        out = torch.zeros(self.num_envs, mech_cfg.PRIVILEGED_WIDTH, device=self.device)
        out[:, 0:1] = held
        out[:, 1:2] = m.hinge.float().unsqueeze(-1) * held
        out[:, 2:3] = m.s.unsqueeze(-1) * 10.0 * held
        out[:, 3:4] = m.v.unsqueeze(-1) * held
        out[:, 4:5] = self._travel_goal.ref.unsqueeze(-1) * 10.0 * held
        out[:, 5:6] = m.travel.unsqueeze(-1) * 10.0 * held
        out[:, 6:9] = interface.quat_rotate_inverse(yaw, m.tangent()) * held
        out[:, 9:12] = interface.quat_rotate_inverse(yaw, self._tip_pos() - m.position()) * 10.0 * held
        out[:, 12:13] = self._peak_n.unsqueeze(-1) * 0.01 * held
        out[:, 13:14] = m.latch_n.unsqueeze(-1) * 0.01 * held
        out[:, 14:15] = m.latched.float().unsqueeze(-1) * held
        out[:, 15:16] = m.friction_static.unsqueeze(-1) * 0.01 * held
        out[:, 16:17] = m.friction_kinetic.unsqueeze(-1) * 0.01 * held
        out[:, 17:18] = m.resistance_at(m.s).unsqueeze(-1) * 0.01 * held
        out[:, 18:19] = m.mass.unsqueeze(-1) / 3.0 * held
        out[:, 19:20] = m.grasp_k.unsqueeze(-1) / 3000.0 * held
        out[:, 20] = self._ceiling * 0.01
        out[:, 21:22] = self._travel_goal.speed.unsqueeze(-1) * 5.0 * held
        return out

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        fallen, timed_out = super()._get_dones()
        self._torn_now = self._mech.torn.clone()
        # Kept through the reset that follows, for an evaluator stepping from outside.
        self._term_torn, self._term_fallen = self._torn_now.clone(), fallen.clone()
        return fallen | self._torn_now, timed_out

    def _resample_velocity_commands(self, env_ids: torch.Tensor) -> None:
        """UniFP's draw, except that an episode with a mechanism stands."""
        super()._resample_velocity_commands(env_ids)
        standing = env_ids[self._mech_env[env_ids]]
        self._commands[standing, :3] = 0.0

    # --- resetting ------------------------------------------------------------------------------

    def _update_curriculum(self, env_ids: torch.Tensor) -> None:
        """Score finished episodes that grasped; raise the ceiling when the frontier opens often enough.

        An episode counts once it has had time to open -- the reference's run to its first target
        plus a second -- or if it ended early by falling or tearing, which are failures to open.
        Episodes cut short by the random start lengths of a fresh run say nothing and are skipped.
        """
        log = self.extras.setdefault("log", {})
        grasped = env_ids[self._ever_grasped[env_ids]]
        if len(grasped) > 0:
            held_steps = self._since_reset[grasped] - self._grasp_step[grasped]
            needed = (self._travel_goal.first_target[grasped]
                      / self._travel_goal.speed[grasped].clamp(min=1e-3) / interface.POLICY_DT + 50.0)
            failed = self._mech.torn[grasped] | self._term_fallen[grasped]
            counted = (held_steps >= needed) | failed
            opened = (self._max_s[grasped] >= mech_cfg.OPENED_FRACTION * self._travel_goal.first_target[grasped]) & ~failed
            frontier = self._peak_n[grasped] >= mech_cfg.FRONTIER_FRACTION * self._draw_ceiling[grasped]
            if bool(counted.any()):
                log["Mechanism/opened_fraction"] = float(opened[counted].float().mean())
                log["Mechanism/torn_fraction"] = float(self._mech.torn[grasped][counted].float().mean())
                log["Mechanism/latch_released_fraction"] = float(
                    self._mech.released[grasped][counted & (self._mech.latch_n[grasped] > 0)].float().mean()
                    if bool((counted & (self._mech.latch_n[grasped] > 0)).any()) else float("nan"))
                for index, name in enumerate(mech_cfg.OPENING_NAMES):
                    kind = counted & (self._opening_kind[grasped] == index)
                    if bool(kind.any()):
                        log[f"Mechanism/opened_{name}"] = float(opened[kind].float().mean())
                hinge = counted & self._mech.hinge[grasped]
                if bool(hinge.any()):
                    log["Mechanism/opened_hinge"] = float(opened[hinge].float().mean())
            front = counted & frontier
            if bool(front.any()):
                rate = float(opened[front].float().mean())
                log["Mechanism/frontier_opened_fraction"] = rate
                self._opened_ema = rate if self._opened_ema is None else (
                    (1 - mech_cfg.CURRICULUM_EMA) * self._opened_ema + mech_cfg.CURRICULUM_EMA * rate)
        wait = mech_cfg.CURRICULUM_MIN_ITERATIONS * task_cfg.NUM_STEPS_PER_ENV
        if (self.cfg.curriculum and self._opened_ema is not None
                and self._opened_ema > mech_cfg.CURRICULUM_PROMOTE_OPENED
                and self._ceiling < self.cfg.peak_ceiling_max_n
                and self.common_step_counter - self._last_promotion_step >= wait):
            self._ceiling = min(self._ceiling + mech_cfg.CURRICULUM_STEP_N, self.cfg.peak_ceiling_max_n)
            self._last_promotion_step = self.common_step_counter
            self._opened_ema = None
        log["Mechanism/peak_ceiling_n"] = self._ceiling
        if self._opened_ema is not None:
            log["Mechanism/frontier_opened_ema"] = self._opened_ema

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        self._update_curriculum(env_ids)
        count = len(env_ids)
        if self._evaluation is not None:
            self._mech_env[env_ids] = True
        elif self.cfg.mechanism_fraction <= 0.0:
            self._mech_env[env_ids] = False
        else:
            self._mech_env[env_ids] = torch.rand(count, device=self.device) < self.cfg.mechanism_fraction
        # Drawn before the parent reset, which resamples the velocity command and must see it stand.
        super()._reset_idx(env_ids)

        self._mech.reset(env_ids)
        self._travel_goal.reset(env_ids)
        self._lag_integral[env_ids] = 0.0
        self._law_command_w[env_ids] = 0.0
        self._ee_force_w[env_ids] = 0.0
        self._torn_now[env_ids] = False
        self._ever_grasped[env_ids] = False
        self._since_reset[env_ids] = 0
        self._grasp_step[env_ids] = 0
        self._max_s[env_ids] = 0.0
        self._peak_n[env_ids] = 0.0
        self._goals.hold_mask[env_ids] = False

        ids = env_ids[self._mech_env[env_ids]]
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
