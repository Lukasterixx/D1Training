"""The foot-press task: the leg environment with plates instead of a key (`--task leg --leg_object press`).

Lukas, 2026-10-21 (`foot_press_cfg`): a general force-aware foot policy, UniFP's goal walk and pushes at the front-right
foot plus plates anywhere it reaches, facing any way it can push, rigid or sprung, pressed with a commanded profile.
Everything about the robot is the key task's (`leg_env.Go2D1LegEnv` with `leg_object="key"`): the settled spawn and
the sport stands, the support feet held to their places, the arm folded at rest, the head checked against what is
ahead, the foot back on its footfall at the end. What changes is the object and its layer:

  * `foot_press.PlateSet` in place of the key's plant (the same attribute names, so the leg environment's contact,
    outer loop and return path run unchanged), drawn from `foot_press.PlateTable`;
  * `foot_press.PressLayer` in place of the key's: in along the estimated normal until the foot stalls, then the
    profile, then back out;
  * the scripted approach's pre-contact point is `standoff` off the estimated face along its normal, the waypoint
    `WAYPOINT_OUT_M` further out, the lift's first leg no higher than the waypoint;
  * the reward's object terms are the key's contact, slip, knee-and-head and approach terms on the plate, the press
    limit at `PRESS_LIMIT_N`, and -- the force-aware variant only -- `press_force`, the applied force against the
    commanded one while it presses;
  * the critic's 22-wide object block describes the plate.
"""
from __future__ import annotations

import math

import torch

from unifp_isaaclab import interface

from . import foot_press, foot_press_cfg as pc, handle, leg_cfg, leg_env, leg_rewards, rewards
from .leg_env import ENGAGED, FOOT_DOWN, RETURNING, Go2D1LegEnv, Go2D1LegEnvCfg

#: The table is the same in every environment of a process; built once.
_TABLE: foot_press.PlateTable | None = None


def plate_table() -> foot_press.PlateTable:
    global _TABLE
    if _TABLE is None:
        _TABLE = foot_press.PlateTable()
    return _TABLE


def press_force(state: rewards.TaskState) -> torch.Tensor:
    """The force the foot puts on the plate against the commanded one, `exp(-|applied - command| / sigma)`, while the
    layer presses (after contact is called): the force-aware variant's only."""
    return torch.exp(-state.press_force_error_n / pc.PRESS_FORCE_SIGMA_N) * state.press_pushing


class Go2D1FootPressEnv(Go2D1LegEnv):
    cfg: Go2D1LegEnvCfg

    def __init__(self, cfg: Go2D1LegEnvCfg, render_mode: str | None = None, **kwargs):
        if cfg.leg_object != "press":
            raise ValueError("Go2D1FootPressEnv needs leg_object='press'")
        super().__init__(cfg, render_mode, **kwargs)
        n, device = self.num_envs, self.device
        self._handle = foot_press.PlateSet(n, device, edges=cfg.press_plate_edges)
        self._layer = foot_press.PressLayer(n, device, release_from_foot=cfg.press_release_from_foot)
        self._table = plate_table().to(device)
        # Free-space goals as far round to the right as the plates go (the key's stop at -0.65 rad of yaw).
        self._goals.yaw_range = pc.GOAL_YAW_RANGE
        zeros = lambda *shape: torch.zeros(*shape, device=device)
        self._plate_est = {k: zeros(n, 3) for k in ("contact", "n")}
        self._plate_true = zeros(n, 3)
        self._profile: dict[str, torch.Tensor] = {}
        self._profile_keys = ("standoff", "speed", "levels", "ramps", "holds", "segments", "tan_ratio", "tan_dir",
                              "slide", "slide_dir", "slide_speed", "slide_dist")
        #: Bookkeeping: the force error's sum and count while pressing, the largest contact force, contact called by a
        #: stall, the foot ever on the plate.
        self._err_sum, self._err_count = zeros(n), zeros(n)
        self._terms["press_force"] = press_force
        # The tilt terms while standing only (`cfg.tilt_stages == "standing"`).
        standing = lambda: (self._handle_env & ((self._stage == leg_env.WAITING) | (self._stage == FOOT_DOWN))).float()
        self._terms["orientation_standing"] = lambda state: leg_rewards.orientation(state) * standing()
        self._terms["tilt_excess_standing"] = lambda state: leg_rewards.tilt_excess(state) * standing()

    def _weight_table(self, handles: bool) -> dict[str, float]:
        weights = dict(leg_cfg.REWARD_WEIGHTS)
        if handles:
            weights.update(pc.PLATE_WEIGHTS)
            weights["press_excess"] = pc.PRESS_EXCESS_WEIGHT
            if self.cfg.leg_variant == "force":
                weights["press_force"] = pc.PRESS_FORCE_WEIGHT
        if self.cfg.stance_weight > 0.0:
            weights["support_stance"] = self.cfg.stance_weight
        if self.cfg.tilt_stages == "standing":
            weights["orientation"] = pc.ORIENTATION_WEIGHT
            if self.cfg.press_orientation_weight is not None:
                weights["orientation_standing"] = self.cfg.press_orientation_weight
            if self.cfg.tilt_excess_weight != 0.0:
                weights["tilt_excess_standing"] = self.cfg.tilt_excess_weight
            return weights
        weights["orientation"] = (pc.ORIENTATION_WEIGHT if self.cfg.press_orientation_weight is None
                                  else self.cfg.press_orientation_weight)
        if self.cfg.tilt_excess_weight != 0.0:
            weights["tilt_excess"] = self.cfg.tilt_excess_weight
        return weights

    # --- placing plates -----------------------------------------------------------------------------------------------

    def _draw_objects(self, ids: torch.Tensor) -> dict[str, torch.Tensor]:
        """Plates and profiles for `ids` in the yaw frame of a robot standing at the origin facing +x."""
        count, device = len(ids), self.device
        rand = lambda *shape: torch.rand(*shape, device=device)
        span = lambda r, *shape: r[0] + rand(*(shape or (count,))) * (r[1] - r[0])
        sym = lambda a, *shape: (2 * rand(*(shape or (count,))) - 1) * a
        rows = torch.randint(self._table["kind"].shape[0], (count,), device=device)
        hand = rand(count) < self.cfg.press_hand_fraction
        if bool(hand.any()):
            hands = torch.tensor(pc.EVAL_DOG_ROWS, device=device)
            rows = torch.where(hand, hands[torch.randint(len(hands), (count,), device=device)], rows)
        d = {k: v[rows] for k, v in self._table.items()}
        rigid = rand(count) < pc.RIGID_PROB
        lo, hi = (math.log(v) for v in pc.SOFT_K_N_PER_M)
        d["k_soft"] = torch.where(rigid, torch.full((count,), handle.CONTACT_K_N_PER_M, device=device),
                                  torch.exp(span((lo, hi))))
        d["travel"] = torch.where(rigid, torch.zeros(count, device=device), span(pc.SOFT_TRAVEL_M))
        d["mu"] = span(pc.PLATE_MU)
        d["err_plane"] = sym(pc.EST_ERROR_PLANE_M, count, 2)
        d["err_normal"] = sym(pc.EST_ERROR_NORMAL_M)
        d["err_tilt"] = sym(pc.EST_ERROR_TILT_RAD, count, 2)
        d["standoff"], d["speed"] = span(pc.STANDOFF_M), span(pc.APPROACH_SPEED_M_S)
        k = foot_press.MAX_SEGMENTS
        d["segments"] = torch.randint(pc.SEGMENTS[0], pc.SEGMENTS[1] + 1, (count,), device=device)
        d["levels"], d["ramps"], d["holds"] = span(pc.FORCE_LEVELS_N, count, k), span(pc.RAMP_S, count, k), \
            span(pc.SEG_HOLD_S, count, k)
        if self.cfg.press_up_cap_n is not None:
            # The foot pushes along -n: its upward part on a plate facing down is capped (F-182).
            up = (-d["n"][:, 2]).clamp(min=1e-3)
            d["levels"] = torch.minimum(d["levels"], (self.cfg.press_up_cap_n / up).unsqueeze(-1))
        tangential = (rand(count) < pc.TANGENT_PROB).float().unsqueeze(-1)
        d["tan_ratio"] = rand(count, k) * pc.TANGENT_RATIO * tangential
        angle = rand(count) * 2 * math.pi
        d["tan_dir"] = torch.cos(angle).unsqueeze(-1) * d["t1"] + torch.sin(angle).unsqueeze(-1) * d["t2"]
        d["slide"] = rand(count) < pc.SLIDE_PROB
        angle = rand(count) * 2 * math.pi
        d["slide_dir"] = torch.cos(angle).unsqueeze(-1) * d["t1"] + torch.sin(angle).unsqueeze(-1) * d["t2"]
        d["slide_speed"], d["slide_dist"] = span(pc.SLIDE_SPEED_M_S), span(pc.SLIDE_DIST_M)
        if bool(hand.any()):
            # The dog's hands: its levels, no tangential part, no slide.
            d["levels"] = torch.where(hand.unsqueeze(-1), span(self.cfg.press_hand_levels_n, count, k), d["levels"])
            d["tan_ratio"] = torch.where(hand.unsqueeze(-1), torch.zeros_like(d["tan_ratio"]), d["tan_ratio"])
            d["slide"] = d["slide"] & ~hand
        return d

    def _place_objects(self, ids: torch.Tensor, d: dict[str, torch.Tensor]) -> None:
        """Mount the plates (the robot just reset to the origin facing +x, so the yaw frame is the environment's), give
        the layer its estimate and the approach: the pre-contact point `standoff` off the estimated face, the waypoint
        further out."""
        self._handle.place(ids, face=d["face"], n=d["n"], t1=d["t1"], half=d["half"], k_soft=d["k_soft"],
                           travel=d["travel"], mu=d["mu"], kind=d["kind"])
        n, t1, t2 = self._handle.n[ids], self._handle.t1[ids], self._handle.t2[ids]
        # The estimate: the contact point off in the plane and along the normal, the normal tilted.
        contact = d["contact"] + d["err_plane"][:, :1] * t1 + d["err_plane"][:, 1:] * t2 + d["err_normal"].unsqueeze(-1) * n
        tilt = d["err_tilt"]
        n_est = foot_press.unit(n + torch.tan(tilt[:, :1]) * t1 + torch.tan(tilt[:, 1:]) * t2)
        self._plate_est["contact"][ids], self._plate_est["n"][ids] = contact, n_est
        self._plate_true[ids] = d["contact"]
        self._pre[ids] = contact + d["standoff"].unsqueeze(-1) * n_est
        self._waypoint[ids] = self._pre[ids] + pc.WAYPOINT_OUT_M * n_est
        # Straight up to the waypoint's height, then across to it (a lift to the key's 14 cm and then on a slant met
        # plates low ahead, round 2's model_3000).
        self._lift_first[ids] = self._waypoint[ids, 2].clamp(min=pc.LIFT_MIN_M, max=pc.LIFT_MAX_M)
        for key in self._profile_keys:
            value = d[key]
            if key not in self._profile:
                self._profile[key] = torch.zeros((self.num_envs,) + tuple(value.shape[1:]), dtype=value.dtype,
                                                 device=self.device)
            self._profile[key][ids] = value
        self._err_sum[ids], self._err_count[ids] = 0.0, 0.0

    def _start_layer(self, ids: torch.Tensor, foot: torch.Tensor) -> None:
        p = {k: v[ids] for k, v in self._profile.items()}
        self._layer.start(ids, pre=self._pre[ids], n=self._plate_est["n"][ids], standoff=p["standoff"], speed=p["speed"],
                          levels=p["levels"], ramps=p["ramps"], holds=p["holds"], segments=p["segments"],
                          tan_ratio=p["tan_ratio"], tan_dir=p["tan_dir"], slide=p["slide"], slide_dir=p["slide_dir"],
                          slide_speed=p["slide_speed"], slide_dist=p["slide_dist"],
                          force_law=torch.full((len(ids),), self._force_aware, dtype=torch.bool, device=self.device),
                          foot=foot[ids])

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        self._lift_first[env_ids] = leg_cfg.LIFT_FIRST_HEIGHT_M
        super()._reset_idx(env_ids)

    # --- the step -----------------------------------------------------------------------------------------------------

    def _correct_foot(self) -> None:
        """The key's outer loop on the foot's miss: the whole miss while it settles at the pre-contact point, the part
        across the normal while the layer moves it in, and while the plate is pressed as `cfg.press_hold_loop` says.

        Rounds 1-4 ran it on the whole in-plane miss while pressing ("full"): it made slides happen (round 4's final
        slid the full 4 cm) but integrated away the displacement a tangential command asks for (0.43 N held of 4).
        Off once contact is called ("off", 04:05-05:25) the same policy held 3.0 N of the 4 but slid 3 mm of 4 cm:
        friction holds the foot and UniFP's target only asks for the slide less friction over k. "hybrid" is the
        layer's force/position split along the plate: the miss is integrated across the normal and across a commanded
        tangential force, so the plate's own directions without a force command are held in position."""
        h = self._handle_env
        if not bool(h.any()):
            return
        foot, _ = self._foot_state()
        dt = interface.POLICY_DT
        touching = (self._handle.door_n + self._handle.knee_door_n > 0.0) | self._handle.on_bar
        self._correction.update_tip(h & (self._stage == leg_env.AT_PRE) & ~touching, self._goal_w_held, foot, dt)
        layer = self._layer
        clear = h & (self._stage == ENGAGED) & (self._handle.knee_door_n <= 0.0)
        self._correction.update_tip(clear & (layer.phase == handle.PRESSING), self._goal_w_held, foot, dt,
                                    across=layer.tangent)
        mode = self.cfg.press_hold_loop
        if mode == "off":
            return
        holding = clear & (layer.phase == handle.HOLDING)
        if mode == "full":
            self._correction.update_tip(holding, self._goal_w_held, foot, dt, across=layer.tangent)
            return
        # hybrid: drop the miss along the normal and along a commanded tangential force, then integrate as update_tip.
        c = self._correction
        n = layer.tangent
        miss = self._goal_w_held - foot
        miss = miss - (miss * n).sum(-1, keepdim=True) * n
        forced = (layer._segment(layer.tan_ratio) > 0.0).unsqueeze(-1)
        t = layer.tan_dir - (layer.tan_dir * n).sum(-1, keepdim=True) * n
        t = t / t.norm(dim=-1, keepdim=True).clamp(min=1e-9)
        miss = torch.where(forced, miss - (miss * t).sum(-1, keepdim=True) * t, miss)
        offset = c.offset + c.tip_gain * dt * miss
        offset = offset * (c.tip_max / offset.norm(dim=-1, keepdim=True).clamp(min=1e-9)).clamp(max=1.0)
        c.offset = torch.where(holding.unsqueeze(-1), offset, c.offset)

    def _advance_return(self, foot: torch.Tensor, stance: torch.Tensor, smooth) -> None:
        """After the press: out to the waypoint, across at its height to above the footfall, straight down onto it.
        (The key's return, waypoint then straight to the footfall, cut through plates between the two: round 2's
        model_3000 put 22 of its 83 peaks past 60 N there.)"""
        h = self._handle_env
        done = h & (self._stage == ENGAGED) & (self._layer.phase == handle.DONE)
        if bool(done.any()):
            self._return_from[done] = self._goal_w_held[done]
            self._stage[done], self._stage_step[done] = RETURNING, self._since_reset[done]
        since = (self._since_reset - self._stage_step).float()
        s1, s2, s3 = (v / interface.POLICY_DT for v in (pc.RETURN_OUT_S, pc.RETURN_OVER_S, pc.RETURN_DOWN_S))
        t1 = (since / s1).clamp(0.0, 1.0)
        t2 = ((since - s1) / s2).clamp(0.0, 1.0)
        t3 = ((since - s1 - s2) / s3).clamp(0.0, 1.0)
        above = stance.clone()
        above[:, 2] = self._waypoint[:, 2].clamp(min=pc.LIFT_MIN_M, max=pc.LIFT_MAX_M)
        leg1 = torch.lerp(self._return_from, self._waypoint, smooth(t1).unsqueeze(-1))
        leg2 = torch.lerp(self._waypoint, above, smooth(t2).unsqueeze(-1))
        leg3 = torch.lerp(above, stance, smooth(t3).unsqueeze(-1))
        goal = torch.where((t1 < 1.0).unsqueeze(-1), leg1, torch.where((t2 < 1.0).unsqueeze(-1), leg2, leg3))
        returning = h & (self._stage == RETURNING)
        down = h & (self._stage == FOOT_DOWN)
        self._goal_w_held = torch.where(returning.unsqueeze(-1), goal, self._goal_w_held)
        self._goal_w_held = torch.where(down.unsqueeze(-1), stance, self._goal_w_held)
        landed = returning & (t3 >= 1.0)
        self._stage = torch.where(landed, torch.full_like(self._stage, FOOT_DOWN), self._stage)
        self._down_for = torch.where(down, self._down_for + interface.POLICY_DT, self._down_for)

    def _pushing(self) -> torch.Tensor:
        """The layer pressing, contact called (its profile running), the foot not yet backing out."""
        return self._handle_env & (self._stage == ENGAGED) & (self._layer.phase == handle.HOLDING)

    def _applied(self) -> torch.Tensor:
        """The force the foot puts on the plate (world)."""
        return -self._handle.force_on_foot

    def _bookkeep_handle(self) -> None:
        dt = interface.POLICY_DT
        pushing = self._pushing()
        self._step_peak = self._handle.take_peak() * self._handle.active
        self._damage_now |= self._step_peak > leg_cfg.DAMAGE_N
        self._peak_n = torch.maximum(self._peak_n, self._step_peak)
        self._door_peak = torch.maximum(self._door_peak, self._handle.knee_door_n)
        off = pushing & self._handle.touched & ~self._handle.on_bar
        self._off_bar_for = torch.where(off, self._off_bar_for + dt, torch.zeros_like(self._off_bar_for))
        slip = (self._off_bar_for >= leg_cfg.SLIP_S) & ~self._slipped
        self._slip_now = slip.float()
        self._slipped |= slip
        # The normal force against what the profile asks, while pressing (both variants: the position-only layer's
        # target stands for the same profile).
        normal = (self._applied() * -self._layer.n).sum(-1)
        error = (normal - self._layer.normal_level).abs()
        self._err_sum += error * pushing
        self._err_count += pushing.float()
        finished = self._handle_env & (self._stage == ENGAGED) & (self._layer.phase == handle.DONE)
        self._done_for = torch.where(finished, self._done_for + dt, self._done_for)

    def _object_state(self, state: rewards.TaskState) -> None:
        pushing = self._pushing()
        state.handle_pushing = pushing.float()
        state.press_pushing = pushing.float()
        state.handle_on_bar = self._handle.on_bar.float()
        state.handle_slip = self._slip_now
        k = handle.CONTACT_K_N_PER_M
        state.handle_door_cm = 100.0 * self._handle.knee_door_n / k * self._handle_env
        approaching = (self._handle_env & (self._stage < ENGAGED)).float()
        state.handle_approach_cm = 100.0 * (self._handle.normal_n + self._handle.knee_door_n) / k * approaching
        state.handle_press_excess_cm = 100.0 * (self._step_peak - pc.PRESS_LIMIT_N).clamp(min=0.0) / k * self._handle_env
        state.press_force_error_n = (self._applied() - self._layer.force_cmd).norm(dim=-1)

    def _privileged_extra(self) -> torch.Tensor:
        """The plate as the critic sees it, in the 22-wide `mass_params` block; its contact force only for the force
        variant (the plan: a force term in a critic only where the matching actor has one)."""
        hd, layer = self._handle, self._layer
        on = (self._handle_env & hd.active).float().unsqueeze(-1)
        yaw = self._base_yaw_quat()
        out = torch.zeros(self.num_envs, 22, device=self.device)
        out[:, 0:1] = on
        out[:, 1:2] = (self._stage == ENGAGED).float().unsqueeze(-1) * on
        out[:, 2:3] = (layer.phase.float() / 4.0).unsqueeze(-1) * on
        out[:, 3:4] = layer.stalled.float().unsqueeze(-1) * on
        out[:, 4:5] = hd.on_bar.float().unsqueeze(-1) * on
        out[:, 5:8] = interface.quat_rotate_inverse(yaw, hd.n) * on
        out[:, 8:11] = interface.quat_rotate_inverse(yaw, self._plate_true - self._plate_est["contact"]) * 50.0 * on
        out[:, 11:12] = (torch.log(hd.k_soft / handle.CONTACT_K_N_PER_M) / 3.0).unsqueeze(-1) * on
        out[:, 12:13] = (hd.travel * 30.0).unsqueeze(-1) * on
        out[:, 13:14] = hd.mu.unsqueeze(-1) * on
        out[:, 14:15] = (hd.height * 10.0).clamp(-3, 3).unsqueeze(-1) * on
        out[:, 15:16] = (layer.normal_level / 35.0).unsqueeze(-1) * on
        out[:, 16:17] = (layer.slide & (layer.slide_s > 0)).float().unsqueeze(-1) * on
        out[:, 17:18] = (hd.depth * 50.0).unsqueeze(-1) * on
        out[:, 18:19] = self._handle_env.float().unsqueeze(-1)
        if self._force_aware:
            out[:, 19:22] = interface.quat_rotate_inverse(yaw, hd.force_on_foot) * 0.02 * on
        return out

    def _record_trace(self) -> None:
        foot, _ = self._foot_state()
        applied = self._applied()
        self._trace.append({
            "stage": self._stage.clone(), "phase": self._layer.phase.clone(), "foot": foot.clone(),
            "goal": self._goal_world().clone(), "on_plate": self._handle.on_bar.clone(),
            "applied": applied.clone(), "force_cmd": self._layer.force_cmd.clone(),
            "level": self._layer.normal_level.clone(), "peak": self._step_peak.clone(),
            "normal_n": (applied * -self._layer.n).sum(-1), "stalled": self._layer.stalled.clone(),
            "base": (self._robot.data.root_pos_w - self.scene.env_origins).clone(),
            "rp": interface.body_roll_pitch(self._robot.data.root_quat_w).clone(),
        })

    def _log_handle_episodes(self, env_ids: torch.Tensor) -> None:
        """Training-time plate diagnostics over the episodes ending now (sampled at reset, not a frozen evaluation)."""
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
        log["Press/missed_fraction"] = float((~engaged).float().mean())
        if bool(engaged.any()):
            e = h[engaged]
            pressed = self._err_count[e] > 0
            log["Press/pressed_fraction"] = float(pressed.float().mean())
            log["Press/stalled_fraction"] = float(self._layer.stalled[e].float().mean())
            log["Press/touched_fraction"] = float(self._handle.touched[e].float().mean())
            if bool(pressed.any()):
                p = e[pressed]
                log["Press/force_error_n"] = float((self._err_sum[p] / self._err_count[p]).median())
            log["Press/peak_n"] = float(self._peak_n[e].mean())
            log["Press/slip_fraction"] = float(self._slipped[e].float().mean())
            log["Press/damaged_fraction"] = float(self._term_damage[e].float().mean())
            log["Press/foot_down_fraction"] = float((self._stage[e] == FOOT_DOWN).float().mean())
            log["Press/engage_miss_mm"] = 1000.0 * float(self._engage_miss[e].median())

    # --- the viewer -----------------------------------------------------------------------------------------------------

    def _set_debug_vis_impl(self, enable: bool) -> None:
        if self.cfg.unifp_markers:
            super(Go2D1LegEnv, self)._set_debug_vis_impl(enable)
        if enable and not hasattr(self, "_plate_markers"):
            from .leg_vis import PlateMarkers

            self._plate_markers = PlateMarkers("/Visuals/foot_plate", self.num_envs, str(self.device))
        if hasattr(self, "_plate_markers"):
            self._plate_markers.set_visibility(enable)

    def _debug_vis_callback(self, event) -> None:
        if hasattr(self, "_markers"):
            super(Go2D1LegEnv, self)._debug_vis_callback(event)
        if hasattr(self, "_plate_markers"):
            self._plate_markers.update(self._handle, self._layer, self.scene.env_origins)
