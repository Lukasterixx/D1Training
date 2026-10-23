"""The foot-press policy's task layer for the dog, without torch: the scripted approach, the press and the return.

The general foot-press policy (`checkpoints/leg_press_force_r4_model_5999.npz`, F-180) takes a goal for the front-right
foot and a force command; in training `unifp_train.foot_press_env` makes both from a plate's estimate. This module is
that environment's task for one robot, in numpy, so the dog can run it (`press_live.py`):

  * `PressLayer` is `unifp_train.foot_press.PressLayer` for one robot: in along the estimated normal until the foot
    stalls, the profile, the force off, back out;
  * `PressTask` is the leg environment's stage machine around it (`leg_env.Go2D1LegEnv._advance_approach`,
    `_run_layer`, `_correct_foot`, `_hold_goals`, with `foot_press_env`'s return and hybrid outer loop): wait, lift
    straight up and across to a waypoint, over onto the pre-contact point, settle, press, out to the waypoint, across
    above the footfall, down onto it. Its `step` is the environment's `_get_rewards` for one policy step and returns
    the 15 commands the next frame carries, the force one step behind the layer as `_step_forces` writes it.

The frame is the training's yaw frame from the ground under the base. On the dog it moves with the base: there is no
odometry here, so the plate, the waypoints and the footfall are all held relative to the base, where the training held
the plate and the waypoints fixed in the world (the base moves a few centimetres over a press there).

Three rules are the dog's, not the training's (each says so where it is applied):
  * the press waits at the pre-contact point for the operator (`go`), up to `hand_wait_s`, where training engaged as
    soon as the foot settled -- the hand goes there once the foot is out;
  * an approach that runs out of travel without the foot stalling backs out instead of pressing (`press_without_stall`
    False): no stall means no hand;
  * the profile's levels and the first ramp's start are capped at `max_force_n`, and a press whose force would point up
    is refused (F-182: pushing up on a plate above the foot at 28 N tipped the robot).

Constants from `unifp_train.foot_press_cfg` are imported (it is plain Python); those of `leg_cfg`, `handle` and
`leg_env` are repeated below because those modules import torch, and `tests/test_press_live.py` checks each against its
source. `tests/test_press_live.py` also runs this layer and torch's `PressLayer` on the same foot paths.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from unifp_train import foot_press_cfg as pc

# --- repeated from modules that import torch (checked by tests/test_press_live.py) ------------------------------------

POLICY_DT = 0.02                              # interface.POLICY_DT
FOOT_FORCE_KP = 400.0                         # leg_cfg.FOOT_FORCE_KP, the force-aware target's virtual stiffness
LIFT_S, OVER_S, LIFT_FIRST_SHARE = 2.0, 0.8, 0.3
FOOT_CORRECTION_GAIN, FOOT_CORRECTION_MAX_M = 2.0, 0.07
ENGAGE_CONVERGED_M, ENGAGE_SETTLE_SPEED_M_S, ENGAGE_NEAR_M, ENGAGE_MAX_WAIT_STEPS = 0.02, 0.10, 0.04, 150
#: The frozen evaluation's wait before the lift (`leg_env._reset_idx`), steps.
EVAL_START_DELAY_STEPS = 25
FOOT_RADIUS_M = 0.022                         # handle.FOOT_RADIUS_M
IDLE, PRESSING, HOLDING, RELEASING, DONE = 0, 1, 2, 3, 4                                   # handle
WAITING, LIFTING, OVER, AT_PRE, ENGAGED, MISSED, RETURNING, FOOT_DOWN = 0, 1, 2, 3, 4, 5, 6, 7   # leg_env
STAGE_NAMES = ("waiting", "lifting", "over", "at_pre", "engaged", "missed", "returning", "foot_down")
PHASE_NAMES = ("idle", "pressing", "holding", "releasing", "done")
#: The commands' layout (`unifp_isaaclab.interface`).
CMD_EE_RADIUS, CMD_EE_FORCE = 3, slice(9, 12)

#: The dog's own cap on the force commanded (N): the profile's levels and the first ramp's start never exceed it.
DEFAULT_MAX_FORCE_N = 5.0


def unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    return v / max(float(np.linalg.norm(v)), 1e-9)


def smooth(t: float) -> float:
    return t * t * (3.0 - 2.0 * t)


def clamp(x: float, lo: float, hi: float) -> float:
    return min(max(x, lo), hi)


def cart_to_sphere(p) -> np.ndarray:
    p = np.asarray(p, dtype=float)
    radius = float(np.linalg.norm(p))
    return np.array([radius, math.asin(p[2] / radius), math.atan2(p[1], p[0])])


# --- the plate and the profile -------------------------------------------------------------------------------------

@dataclass
class Plate:
    """Where the layer is told the plate is: the foot sphere's centre touching it (`contact`) and its outward normal,
    in the yaw frame from the ground under the base (`foot_press.PlateTable`'s columns)."""
    contact: np.ndarray
    n: np.ndarray

    def __post_init__(self):
        self.contact, self.n = np.asarray(self.contact, dtype=float), unit(self.n)


#: Plates for the operator's hand, each a row of the trained table (`foot_press.PlateTable()`, rounded; the test
#: checks that each has its kind's rows within 2 cm): the palm facing up under the foot, which presses down onto it,
#: and the palm facing the robot ahead of the foot, which presses forward.
PLATES = {
    "palm_up": Plate(contact=(0.331, -0.202, 0.112), n=(0.0, 0.0, 1.0)),          # table row 14027, kind level_up
    "palm_facing": Plate(contact=(0.422, -0.198, 0.221), n=(-0.998, -0.064, 0.0)),  # row 10851, kind wall
}


@dataclass
class Profile:
    """One press: the standoff and approach speed, then up to three levels, each ramped to and held. No tangential
    part and no slide (the frozen evaluation's profile)."""
    levels: list = field(default_factory=lambda: [DEFAULT_MAX_FORCE_N])
    ramps: list = field(default_factory=lambda: [pc.EVAL_RAMP_S])
    holds: list = field(default_factory=lambda: [pc.EVAL_HOLD_S])
    standoff: float = 0.04
    speed: float = pc.APPROACH_SPEED_M_S[0]


def check_plate(plate: Plate) -> list[str]:
    """What is wrong with a plate against the training's table rules (`foot_press.PlateTable`), and the dog's rule on
    pushing up; empty if nothing. Reach and clearance from the body are not checked here (the table's voxels need
    torch's module); `PLATES` are table rows."""
    problems = []
    shoulder = np.asarray(pc.SHOULDER_M)
    dist = float(np.linalg.norm(plate.contact - shoulder))
    if not pc.CONTACT_DIST_M[0] <= dist <= pc.CONTACT_DIST_M[1]:
        problems.append(f"the contact is {dist:.3f} m from the FR thigh joint, outside the trained {pc.CONTACT_DIST_M}")
    to_shoulder = unit(shoulder - plate.contact)
    tilt = math.degrees(math.acos(clamp(float(plate.n @ to_shoulder), -1.0, 1.0)))
    if tilt > math.degrees(pc.NORMAL_TILT_MAX_RAD) + 1e-6:
        problems.append(f"the normal is {tilt:.0f} deg off the direction to the FR thigh joint (trained to 60)")
    if plate.contact[2] < pc.MIN_CONTACT_HEIGHT_M:
        problems.append(f"the contact is {plate.contact[2]:.3f} m up, below the trained {pc.MIN_CONTACT_HEIGHT_M}")
    if plate.contact[0] < 0.12 or plate.contact[1] > 0.02:        # leg_cfg.GOAL_MIN_AHEAD_M, GOAL_MAX_LEFT_M
        problems.append("the contact is behind 0.12 m or left of 0.02 m (the trained goals' limits)")
    # The dog's rule (F-182): the press pushes along -n; never upward.
    if plate.n[2] < -0.2:
        problems.append("the press would push up (the normal faces down): F-182, pushing up tips the robot")
    return problems


# --- the layer -------------------------------------------------------------------------------------------------------

class PressLayer:
    """`foot_press.PressLayer` for one robot, force-aware, `release_from_foot` True (as round 4's final trained).

    Line for line with the torch layer's `step`; the dog's two changes are `press_without_stall` and `max_force_n`."""

    def __init__(self, *, k_virtual: float = FOOT_FORCE_KP, max_force_n: float = DEFAULT_MAX_FORCE_N,
                 press_without_stall: bool = False, force_cap_start: bool = True) -> None:
        self.k_virtual, self.max_force_n = k_virtual, max_force_n
        self.press_without_stall = press_without_stall
        #: Cap the first ramp's start at `max_force_n` (the dog's); False: at the trained `FORCE_LEVELS_N[1]` only.
        self.force_cap_start = force_cap_start
        self.phase = IDLE
        self.goal, self.force_cmd = np.zeros(3), np.zeros(3)
        self.normal_level = 0.0
        self.stalled = self.ran_out = False

    def start(self, *, pre, n, profile: Profile, foot) -> None:
        if len(profile.levels) != len(profile.ramps) or len(profile.levels) != len(profile.holds):
            raise ValueError("levels, ramps and holds need one value per segment")
        if not 1 <= len(profile.levels) <= 3:
            raise ValueError("a profile has 1-3 segments")
        self.pre, self.n = np.asarray(pre, dtype=float), unit(n)
        self.standoff, self.speed = float(profile.standoff), float(profile.speed)
        cap = lambda v: min(float(v), self.max_force_n)
        self.levels = np.array([cap(v) for v in profile.levels])
        self.ramps, self.holds = np.asarray(profile.ramps, dtype=float), np.asarray(profile.holds, dtype=float)
        self.segments = len(self.levels)
        self.tan_ratio, self.tan_dir = np.zeros(self.segments), np.zeros(3)
        self.slide, self.slide_dir, self.slide_speed, self.slide_dist = False, np.zeros(3), 0.0, 0.0
        self.phase = PRESSING
        self.ref = self.timer = self.seg_t = self.wait = self.level = self.slide_s = self.normal_level = 0.0
        self.release_from = self.depth = self.start_level = 0.0
        self.seg = 0
        self.stalled = self.ran_out = False
        self.origin = self.pre.copy()
        self.last_foot = np.asarray(foot, dtype=float).copy()
        self.goal = self.pre.copy()
        self.force_cmd = np.zeros(3)
        self.tangent = -self.n

    def _segment(self, values) -> float:
        return float(values[min(self.seg, self.segments - 1)])

    def step(self, foot, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """Advance one policy step; return (the goal for the foot's centre, the force command)."""
        foot = np.asarray(foot, dtype=float)
        n = self.n
        speed_in = float((foot - self.last_foot) @ -n) / dt
        self.last_foot = foot.copy()
        approaching = self.phase == PRESSING
        end = self.standoff + pc.OVER_TRAVEL_M
        if approaching:
            self.ref = min(self.ref + self.speed * dt, end)
        reference = self.pre - self.ref * n
        miss = reference - foot
        lag = float(miss @ -n)
        across_miss = float(np.linalg.norm(miss + lag * n))
        near = self.ref >= self.standoff - pc.DETECT_FROM_M
        stalled = approaching and near and across_miss < pc.DETECT_ACROSS_M and lag > pc.DETECT_LAG_M \
            and speed_in < pc.DETECT_SPEED_M_S
        if approaching and self.ref >= end:
            self.wait += dt
        ran_out = approaching and self.wait >= pc.DETECT_WAIT_S
        found = stalled or ran_out
        self.stalled |= stalled
        abandon = False
        if ran_out and not stalled and not self.press_without_stall:
            # The dog's rule: the reference ran out with nothing found -- no hand there -- so it backs out unpressed.
            self.ran_out, found, abandon = True, False, True
        if found:
            self.origin = reference + float((foot - reference) @ n) * n
            implied = min(pc.TOUCH_N + self.k_virtual * max(lag, 0.0), pc.FORCE_LEVELS_N[1])
            self.start_level = min(implied, self.max_force_n) if self.force_cap_start else implied
            self.phase, self.seg, self.seg_t = HOLDING, 0, 0.0
        finished = False
        if abandon:
            # Backed out from where the foot is, as release_from_foot does after a press.
            self.origin = foot.copy()
            self.phase, self.level, self.release_from, finished = RELEASING, 0.0, 0.0, True
            self.timer = 0.0
        holding = self.phase == HOLDING
        if holding and not found:
            self.seg_t += dt
        ramp, hold = max(self._segment(self.ramps), dt), self._segment(self.holds)
        prev = float(self.levels[self.seg - 1]) if self.seg > 0 else self.start_level
        target = self._segment(self.levels)
        level = prev + (target - prev) * min(self.seg_t / ramp, 1.0)
        if holding:
            self.level = level
        next_seg = holding and self.seg_t >= ramp + hold
        if next_seg:
            self.seg += 1
            self.seg_t = 0.0
        if next_seg and self.seg >= self.segments:
            finished = True
            self.release_from = self.level
            # release_from_foot: back straight out from where the foot is.
            self.origin = foot - self.slide_s * self.slide_dir + self.depth * n
            self.phase, self.timer = RELEASING, 0.0
        releasing = self.phase == RELEASING
        if releasing and not finished:
            self.timer += dt
        off = max(1.0 - self.timer / pc.RELEASE_FORCE_S, 0.0)
        if releasing:
            self.level = self.release_from * off
        back = clamp((self.timer - pc.RELEASE_FORCE_S) / pc.RELEASE_S, 0.0, 1.0)
        if releasing and self.timer >= pc.RELEASE_FORCE_S + pc.RELEASE_S:
            self.phase = DONE

        approaching, holding, releasing = self.phase == PRESSING, self.phase == HOLDING, self.phase == RELEASING
        sunk = float((self.origin - foot) @ n)
        if holding and not found:
            depth = self.depth + (sunk - self.depth) * min(1.0, dt / pc.FOLLOW_TAU_S)
            self.depth = clamp(depth, -pc.FOLLOW_OUT_M, pc.FOLLOW_MAX_M)
        pressed = self.origin + self.slide_s * self.slide_dir - self.depth * n
        out = pressed + (self.standoff * back) * n
        level = self.level if (holding or releasing) else 0.0
        self.normal_level = level
        force = -level * n
        if approaching:
            force = -pc.TOUCH_N * n
        if self.phase in (IDLE, DONE):
            force = np.zeros(3)
        self.force_cmd = force
        goal = reference if approaching else out
        if self.phase == DONE:
            goal = pressed + self.standoff * n
        if self.phase != IDLE:
            self.goal = goal
        return self.goal.copy(), self.force_cmd.copy()


# --- the task around it ----------------------------------------------------------------------------------------------

class PressTask:
    """The leg environment's press episode for one robot (see the module docstring). `step` once per policy step with
    the FR foot sphere's centre and velocity in the yaw frame from the ground; `go` is the operator's say-so, needed to
    lift (once `EVAL_START_DELAY_STEPS` have passed) and to press (at the pre-contact point)."""

    def __init__(self, plate: Plate, profile: Profile, *, down_goal, goal_centre,
                 max_force_n: float = DEFAULT_MAX_FORCE_N, hand_wait_s: float = 15.0,
                 press_without_stall: bool = False) -> None:
        problems = check_plate(plate)
        if problems:
            raise ValueError("; ".join(problems))
        self.plate, self.profile = plate, profile
        self.down_goal, self.goal_centre = np.asarray(down_goal, dtype=float), np.asarray(goal_centre, dtype=float)
        self.hand_wait_steps = round(hand_wait_s / POLICY_DT)
        self.layer = PressLayer(max_force_n=max_force_n, press_without_stall=press_without_stall)
        self.pre = plate.contact + profile.standoff * plate.n
        self.waypoint = self.pre + pc.WAYPOINT_OUT_M * plate.n
        self.lift_first = clamp(float(self.waypoint[2]), pc.LIFT_MIN_M, pc.LIFT_MAX_M)
        self.stage, self.stage_step, self.since_reset = WAITING, 0, 0
        self.approach_from, self.return_from = np.zeros(3), np.zeros(3)
        self.goal_w_held = np.zeros(3)
        self.offset = np.zeros(3)
        self.force_prev = np.zeros(3)
        #: Why the press ended without pressing, if it did ("no go", "missed", "no stall").
        self.ended_unpressed: str | None = None
        self.engage_miss = math.nan

    # The outer loop (`insertion.PoseCorrection.update_tip`).
    def _update_tip(self, foot, across=None) -> None:
        miss = self.goal_w_held - foot
        if across is not None:
            miss = miss - float(miss @ across) * across
        offset = self.offset + FOOT_CORRECTION_GAIN * POLICY_DT * miss
        norm = float(np.linalg.norm(offset))
        self.offset = offset * min(FOOT_CORRECTION_MAX_M / max(norm, 1e-9), 1.0)

    def _begin_return(self, why: str | None) -> None:
        self.return_from = self.goal_w_held.copy()
        self.stage, self.stage_step = RETURNING, self.since_reset
        if why is not None:
            self.ended_unpressed = why

    def _advance_approach(self, foot, foot_v, go: bool) -> None:
        since = self.since_reset - self.stage_step
        if self.stage == WAITING and self.since_reset >= EVAL_START_DELAY_STEPS and go:
            self.approach_from = foot.copy()
            self.stage, self.stage_step = LIFTING, self.since_reset
            since = 0
        lift_steps, over_steps = round(LIFT_S / POLICY_DT), round(OVER_S / POLICY_DT)
        t_lift, t_over = clamp(since / lift_steps, 0.0, 1.0), clamp(since / over_steps, 0.0, 1.0)
        goal = self.goal_w_held.copy()
        stance = self.down_goal
        if self.stage == WAITING:
            goal = stance.copy()
        first = self.approach_from.copy()
        first[2] = self.lift_first
        split = LIFT_FIRST_SHARE
        if self.stage == LIFTING:
            if t_lift < split:
                goal = self.approach_from + (first - self.approach_from) * smooth(min(t_lift / split, 1.0))
            else:
                goal = first + (self.waypoint - first) * smooth(max((t_lift - split) / (1.0 - split), 0.0))
        if self.stage == OVER:
            goal = self.waypoint + (self.pre - self.waypoint) * smooth(t_over)
        if self.stage == AT_PRE:
            goal = self.pre.copy()
        if self.stage <= AT_PRE:
            self.goal_w_held = goal
        self._advance_return(stance)
        if self.stage == LIFTING and t_lift >= 1.0:
            self.stage, self.stage_step = OVER, self.since_reset
        elif self.stage == OVER and t_over >= 1.0:
            self.stage, self.stage_step = AT_PRE, self.since_reset
        elif self.stage == AT_PRE:
            waited = self.since_reset - self.stage_step
            miss = float(np.linalg.norm(foot - self.pre))
            settled = float(np.linalg.norm(foot_v)) < ENGAGE_SETTLE_SPEED_M_S
            deadline = waited >= ENGAGE_MAX_WAIT_STEPS
            # The dog's rule: the press also waits for the operator (the hand under the foot), up to `hand_wait_s`.
            due = waited >= 5 and go and ((settled and miss < ENGAGE_CONVERGED_M) or (deadline and miss < ENGAGE_NEAR_M))
            if due:
                self.layer.start(pre=self.pre, n=self.plate.n, profile=self.profile, foot=foot)
                self.stage, self.engage_miss = ENGAGED, miss
            elif deadline and go:
                self.stage = MISSED
                self._begin_return("missed")        # the evaluation's MISSED holds the foot there; the dog brings it home
            elif waited >= max(self.hand_wait_steps, 5):
                self._begin_return("no go")

    def _advance_return(self, stance) -> None:
        """`foot_press_env._advance_return`: out to the waypoint, across at its height above the footfall, down."""
        if self.stage == ENGAGED and self.layer.phase == DONE:
            self._begin_return("no stall" if self.layer.ran_out else None)
        since = self.since_reset - self.stage_step
        s1, s2, s3 = (v / POLICY_DT for v in (pc.RETURN_OUT_S, pc.RETURN_OVER_S, pc.RETURN_DOWN_S))
        t1, t2, t3 = clamp(since / s1, 0.0, 1.0), clamp((since - s1) / s2, 0.0, 1.0), clamp((since - s1 - s2) / s3, 0.0, 1.0)
        above = stance.copy()
        above[2] = clamp(float(self.waypoint[2]), pc.LIFT_MIN_M, pc.LIFT_MAX_M)
        if t1 < 1.0:
            goal = self.return_from + (self.waypoint - self.return_from) * smooth(t1)
        elif t2 < 1.0:
            goal = self.waypoint + (above - self.waypoint) * smooth(t2)
        else:
            goal = above + (stance - above) * smooth(t3)
        if self.stage == RETURNING:
            self.goal_w_held = goal
            if t3 >= 1.0:
                self.stage = FOOT_DOWN
        elif self.stage == FOOT_DOWN:
            self.goal_w_held = stance.copy()

    def _run_layer(self, foot) -> None:
        if self.stage == ENGAGED:
            goal, _ = self.layer.step(foot, POLICY_DT)
            self.goal_w_held = goal

    def _correct_foot(self, foot) -> None:
        """`foot_press_env._correct_foot` with `press_hold_loop="hybrid"` and no tangential command: the whole miss at
        the pre-contact point, its part across the normal while the layer moves in and while it presses. Contact with
        the knee or the head, which training excluded, is not sensed here."""
        if self.stage == AT_PRE:
            self._update_tip(foot)
        elif self.stage == ENGAGED and self.layer.phase in (PRESSING, HOLDING):
            self._update_tip(foot, across=self.layer.tangent)

    def commanded_goal(self) -> np.ndarray:
        """The goal the policy is given (`_hold_goals`): the held goal plus the outer loop's offset from the
        pre-contact point to the press."""
        corrected = AT_PRE <= self.stage <= ENGAGED
        return self.goal_w_held + (self.offset if corrected else 0.0)

    def step(self, foot, foot_v, go: bool = False) -> np.ndarray:
        """One policy step (`Go2D1LegEnv._get_rewards`'s task part); returns the 15 commands for the next frame."""
        foot, foot_v = np.asarray(foot, dtype=float), np.asarray(foot_v, dtype=float)
        # The force the frame carries is the layer's from the step before (`_step_forces` runs before physics).
        force = self.force_prev.copy()
        self.since_reset += 1
        self._advance_approach(foot, foot_v, go)
        self._run_layer(foot)
        self._correct_foot(foot)
        commands = np.zeros(15)
        commands[CMD_EE_RADIUS:CMD_EE_RADIUS + 3] = cart_to_sphere(self.commanded_goal() - self.goal_centre)
        commands[CMD_EE_FORCE] = force
        self.force_prev = self.layer.force_cmd.copy()
        return commands

    # --- the dog's exits (not in training) ---------------------------------------------------------------------------

    def release_now(self, foot) -> None:
        """End a press early: the force ramped off from where it is and the foot backed out from where it is, as the
        layer's own release does (the operator's stop, or the force estimate past its limit)."""
        layer = self.layer
        if self.stage != ENGAGED or layer.phase not in (PRESSING, HOLDING):
            return
        layer.release_from = layer.level if layer.phase == HOLDING else 0.0
        layer.origin = np.asarray(foot, dtype=float) + layer.depth * layer.n
        layer.phase, layer.timer = RELEASING, 0.0

    def go_home(self, why: str, foot) -> None:
        """Bring the foot back to its footfall from wherever the episode is."""
        if self.stage == ENGAGED:
            self.release_now(foot)
        elif self.stage in (LIFTING, OVER, AT_PRE, MISSED):
            self._begin_return(why)

    def status(self) -> dict:
        return {"stage": STAGE_NAMES[self.stage], "phase": PHASE_NAMES[self.layer.phase],
                "level_n": round(self.layer.normal_level, 2), "stalled": self.layer.stalled}
