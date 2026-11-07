"""The arm half of the reach on the dog: the arm as the policy trained on it, and what stops it. numpy only.

The legs half is `press_live.py`'s (`--presses 0`), which computes the arm's six targets and sends nothing. This module is
what is between those targets and the D1, and between the D1's reports and the policy. It holds no connection and sends
nothing: the runner gives it reports and actions, and sends what it returns. `tests/test_reach_arm.py` holds it to
`unifp_train/arm_link.py`, the model the policy trained on, and to the commands and reports of two runs on the real arm.

The contract, each part because the policy trained on it (`checkpoints/README.md`):

- **`ArmReports`.** The actor reads the last `current_servo_angle` report, held between reports, and joint speed as the
  difference of the last two over their real interval: the arm reports no speed. Not `arm_Feedback/1`, which echoes stale
  copies after each command, and not `D1Client.feedback_age_s`, which those echoes refresh.
- **`ArmTargets`.** The policy's action to a target: default + 0.25 x action, clipped as the environment clips it, clamped to
  the hard limits as `d1_hardware` clamps it. In URDF radians until `to_servo_deg`: Joint1 and Joint4 are flipped on the wire
  (`d1_ik.SERVO_SIGN`), and a sign wrong there moves the arm the other way.
- **One command a report**, 15-85 ms after it. Nearer the next report the arm drops commands (98-106 ms: 36 of 42 cycles
  skip). `d1_hardware.ReportClock` is what keeps that; `in_window` is a check on it, not a replacement.

`ArmWatch` is the stops. It says to stop sending, which in mode 0 leaves the arm holding where it is (F-022): it does not move
the arm anywhere. Its thresholds are set from the policy's own commands in simulation and two runs on the arm, and a run with
a new policy should check them against that policy first.

**How it runs** (`press_live.py --presses 0 --reach_arm_obs reports --reach_arm_send`). Reports come over the arm link's port with their
`report_s`, polled by the link every ~3 ms. Targets go as `follow` goals, so the link's `ReportClock` sends one per report and its
`Follower` caps them at 45 deg/s and holds on lead: measured in simulation (`--arm_follower`) that costs the policy little and removes
most of what is left of its jitter (Week 8 log), so no second sender and no new path to the arm. The link holds if this goes quiet.

**Legs and arm together, never the arm alone.** With the legs dry the policy runs away: in simulation, through the `Follower`, with the legs
holding their stance and ignoring it, 10 of 27 goals are reached, the final error is 27 cm median and 77 at worst, the hand touches the dog in 8
of 27, and the policy's targets go to their clip (F-252). So `--reach_arm_send` needs `--live`.

**Not validated.** It has run only in tests, against fakes. Lukas chose this over an arm-only stage first (2026-11-06). The first run is the whole
thing at once: stay at the E-stop, keep the dog clear, and read `Week 8 log`'s list before it.

**The arm's part of a run.** It stays folded through the dog's lie-down and stand-up. With the legs on the stand-up gains it unfolds to the policy's
default pose by `UNFOLD_ROUTE`: the door demo's proven unfold to `HOME_SERVO_DEG`, then a short path on to zero. Only then do the gains blend and the
policy start, and only within `START_TOL_DEG` of that pose: from the fold the hand strikes the dog in simulation (4 of 27, to 485 N). At the end of a run
it holds, then folds by `FOLD_ROUTE` before the dog lies down: back to home, then the door demo's `tuck` and `fold`. Not straight: on 24 Oct that swung
the forearm onto the lidar cage and stalled it, which simulation cannot see, since only the dog's body reports contact. A stop holds the arm where
it is. Never release: it is a torque-off with no brake (F-028). The goal is fixed for a run unless it is given a path (`GoalPath`).

**Two kinds of stop.** The arm answering and only the policy's asking out of bounds (`ArmWatch.soft`) stops the arm, not the dog: it holds, the legs go to the stand-up gains, the arm folds, the dog lies down. So does a tilt past `--reach_soft_tilt_deg` or a leg bound, where the legs can still hold (Week 8 log). A dead feed, a bad number, a tilt past `--tilt_limit_deg` or the E-stop stays damping, the arm held out.

**First-ever on the real arm, all at once in the first run:** the arm at zero (the nearest logged is 25 deg off); `follow` on a locked stream; the policy on
the legs with the arm out; a stop from there, on a dog going to damping with the arm held out, whose effect on its balance is not known; a fold-back from a
policy pose (proven only from home).

**Two sets of limits apply.** `ArmTargets` takes the training limits (`meta["arm_joint_limits"]`, URDF radians, +-1.57 on J2 and J3);
`D1Client.set_all_joint_angles` then clamps to the arm's own, in servo degrees. Keep both. The fold is at or past the first: J2 -1.5865,
J3 +1.6005 at rest.
"""
from __future__ import annotations

import json
import math

import numpy as np

import d1_ik

#: The fold, as the arm link's programs command it (`plan.FOLD_SERVO_DEG`, checked by the tests): the rest pose inside the +-90 the driver takes.
FOLD_SERVO_DEG = (2.1, -90.0, 90.0, -3.1, 4.1, 3.1)
#: Where the door demo's unfold ends, as logged on the arm (run `20261020T084728_unfold_live`).
HOME_SERVO_DEG = (2.8, 3.9, 24.5, -2.4, -25.2, 2.8)
#: The door demo's `tuck`: the fold with the elbow 30 deg open (`plan.tuck_servo_deg`, checked by the tests).
TUCK_SERVO_DEG = (2.1, -90.0, 60.0, -3.1, 4.1, 3.1)
#: The joint moves' pace, deg/s, below the door program's 40 and the arm link's 45: the part past home has never run.
JOINT_MOVE_DEG_S = 30.0
#: Arrived, for a joint move, deg.
ARRIVED_DEG = 3.0
#: A joint move that has not arrived by this time has gone wrong, s. The longest route is ~8 s at 30 deg/s.
JOINT_MOVE_TIMEOUT_S = 20.0
#: A command 15-85 ms after a report (tested at 20, 50 and 80 ms held, 20 ms moving); 98-106 ms stalls the arm.
WINDOW_S = (0.015, 0.085)
#: Past this with no new report, the arm's state is not known: 0.21 s is the longest gap measured on a locked stream, and it is
#: one skipped report. Three cycles leaves a margin over that without chasing an arm that has stopped answering.
MAX_REPORT_GAP_S = 0.34
#: How far the policy's target may stay ahead of where the arm reports itself, deg, and for how long, s. Simulation, through the Follower, past the
#: first second, at most 13; a blocked arm or a policy going wrong stays past this. The Follower, not this, limits the arm: a tick past it is no harm,
#: and ending a run puts the dog in damping with the arm out, so it takes `LEAD_HOLD_S` of it.
MAX_LEAD_DEG = 25.0
LEAD_HOLD_S = 0.5
#: Past this at once, deg: a policy running away goes to its 25 rad clip within a second (F-237, F-252).
MAX_LEAD_HARD_DEG = 60.0
#: The most one joint's target may change in one 20 ms tick, deg. Simulation, past the first second: at most 7 even over a whole 111 ms cycle.
MAX_STEP_DEG = 20.0
#: How long after the first goal the lead and step checks wait, s. The policy starts on an empty history, as every episode does, and in
#: its first second asks for up to 64 deg in one step (simulation, `armft4` `65799`). The checks read the policy's target; the link's
#: `Follower` is what limits the arm then, to 5 deg a command and 7 deg of lead on the same start. Past it the target stays within 17
#: deg of the arm and 10 deg a step, standing with no pushes. So this grace is only safe through `follow`: never send these directly.
GRACE_S = 1.0
#: The legs' bound in that same first second, rad: on an empty history the policy asks up to 0.95 rad of a leg (simulation, `armft5` `66200`) and 0.68
#: after it, against `press_live`'s `--reach_max_leg_off_rad` of 0.9, which would end the run the tick the legs took over. A runaway goes to the clip,
#: 25 rad, so this still stops one.
LEG_OFF_GRACE_RAD = 1.2


#: The shortest interval a speed is differenced over, s: what `ArmLink.observe` floors it at, one policy step. Stamps are arrival
#: times, so reports held up and delivered together would otherwise read as a huge speed.
MIN_REPORT_INTERVAL_S = 0.02


def to_servo_deg(q_rad) -> np.ndarray:
    """URDF radians to what goes on the wire. `d1_ik` is the one place the sign convention is written down."""
    return np.array(d1_ik.to_servo_deg(np.asarray(q_rad, dtype=float)))


def from_servo_deg(angles_deg) -> np.ndarray:
    """What the arm reports to URDF radians."""
    return np.ravel(d1_ik.from_servo_deg(np.asarray(angles_deg, dtype=float)))


#: How near the policy's default pose every joint has to be to start sending, deg. Every training episode starts there.
START_TOL_DEG = 10.0


def at_start(q, default, tol_deg: float = START_TOL_DEG) -> str | None:
    """None, or why the arm is too far from where the policy starts. Not the fold: in simulation it strikes the dog from there."""
    off = np.degrees(np.abs(np.asarray(q, dtype=float) - np.asarray(default, dtype=float)))
    if float(off.max()) > tol_deg:
        return (f"joint {int(off.argmax()) + 1} is {off.max():.0f} deg from the policy's start (at most {tol_deg:.0f}): "
                "unfold the arm first")
    return None


def joint_path(start_servo_deg, end_servo_deg, speed_deg_s: float = JOINT_MOVE_DEG_S, dt: float = 0.1) -> list[list[float]]:
    """Points `dt` apart on the straight joint line, its longest joint at `speed_deg_s`: a timed path for the arm link. Servo degrees."""
    start, end = np.asarray(start_servo_deg, dtype=float), np.asarray(end_servo_deg, dtype=float)
    steps = max(2, int(math.ceil(float(np.abs(end - start).max()) / (speed_deg_s * dt))) + 1)
    return [[round(float(v), 3) for v in start + (end - start) * k / (steps - 1)] for k in range(steps)]


def route(start_servo_deg, *waypoints, speed_deg_s: float = JOINT_MOVE_DEG_S, dt: float = 0.1) -> list[list[float]]:
    """One timed path through `waypoints` on joint lines, from `start_servo_deg`. Servo degrees."""
    points = [[round(float(v), 3) for v in start_servo_deg]]
    for waypoint in waypoints:
        points += joint_path(points[-1], waypoint, speed_deg_s, dt)[1:]
    return points


def in_window(since_report_s: float) -> bool:
    """Whether a command now, this long after the last report, is where the arm takes it."""
    return WINDOW_S[0] <= since_report_s <= WINDOW_S[1]


class ArmReports:
    """The arm as the actor reads it: `arm_link.ArmLink.observe`, from real reports. URDF radians, seconds on one clock: stamps
    from `D1Client.servo_report_s` are `time.monotonic()` on arrival, so `now_s` in `age` and `ArmWatch.check` must be too."""

    def __init__(self, start_q):
        self.q = np.asarray(start_q, dtype=float).copy()
        self.dq = np.zeros(6)
        self.stamp: float | None = None
        self.count = 0

    def report(self, stamp_s: float, q) -> bool:
        """A `current_servo_angle` report at the arm's own `stamp_s`. A repeat of the last stamp changes nothing."""
        if self.stamp is not None and stamp_s <= self.stamp:
            return False
        q = np.asarray(q, dtype=float)
        if self.stamp is not None:
            self.dq = (q - self.q) / max(stamp_s - self.stamp, MIN_REPORT_INTERVAL_S)
        self.q, self.stamp = q.copy(), float(stamp_s)
        self.count += 1
        return True

    def age(self, now_s: float) -> float:
        return math.inf if self.stamp is None else now_s - self.stamp


class ArmTargets:
    """An export's arm actions to joint targets, as the environment makes them, clamped as the wire is."""

    def __init__(self, default, action_scale: float, clip_actions: float, lower, upper):
        self.default = np.asarray(default, dtype=float)
        self.scale, self.clip = float(action_scale), float(clip_actions)
        self.lower, self.upper = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
        if self.default.shape != (6,) or self.lower.shape != (6,) or self.upper.shape != (6,):
            raise ValueError("six joints: default, lower and upper")
        if np.any(self.lower >= self.upper):
            raise ValueError("a joint's lower limit is not below its upper")

    def target(self, arm_action) -> np.ndarray:
        """The six arm actions, UniFP's order, to URDF radians."""
        action = np.clip(np.asarray(arm_action, dtype=float), -self.clip, self.clip)
        return np.clip(self.default + self.scale * action, self.lower, self.upper)


#: What training draws goals from (`interface.EE_GOAL_*`), radius m, pitch and yaw rad, and how fast a slide may be: training's take 1-3 s, and the
#: Follower caps the arm at 45 deg/s, so a far goal goes no faster than this.
GOAL_RADIUS_M, GOAL_PITCH_RAD, GOAL_YAW_RAD = (0.30, 0.58), (-math.pi / 4, math.pi / 3), (-2 * math.pi / 5, 2 * math.pi / 5)
MIN_SLIDE_S = 2.0
#: Training's keep-out box about the goal sphere's centre, and how far under it a goal may go (`interface.EE_GOAL_COLLISION_*`, `EE_GOAL_UNDERGROUND_LIMIT`): it never
#: drew a slide through these, and the box is the dog.
GOAL_KEEP_OUT_LOWER, GOAL_KEEP_OUT_UPPER, GOAL_UNDERGROUND_M = (-0.48, -0.14, -0.55), (0.17, 0.14, -0.10), -0.40
GOAL_CHECK_SAMPLES = 10
#: Back on the start goal for this long before a run ends, so the fold starts where it has started on the dog.
BACK_HOLD_S = 2.0
#: The jaws on a cloth (`--reach_cloth`), servo units (`d1_hardware.GRIPPER_UNITS_OPEN`, `_CLOSED`, kept here off the dog's import path). They close
#: over `CLOTH_CLOSE_S`, slowly enough to see and with a hand nearby, and stop where the cloth stops them, still commanded shut so they go on squeezing.
#: `CLOTH_LET_GO_S` then gives the operator time to let go and step back. With no A in `CLOTH_WAIT_S` the run ends: the dog is not left standing on it.
JAWS_OPEN_UNITS, JAWS_SHUT_UNITS = 50.2, -19.8
CLOTH_CLOSE_S, CLOTH_LET_GO_S, CLOTH_WAIT_S = 2.0, 2.0, 90.0


def jaws_closing(elapsed_s: float) -> float:
    """The jaw command `elapsed_s` into closing."""
    return JAWS_OPEN_UNITS + (JAWS_SHUT_UNITS - JAWS_OPEN_UNITS) * min(max(elapsed_s / CLOTH_CLOSE_S, 0.0), 1.0)


def goal_is_clear(start, goal) -> bool:
    """`EeGoalTrajectory._collides`, negated: whether a slide stays out of the keep-out box and above the floor's limit, by the same samples."""
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    for u in np.linspace(0.0, 1.0, GOAL_CHECK_SAMPLES):
        r, pitch, yaw = start + (goal - start) * u
        point = r * np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch)])
        if np.all(point < GOAL_KEEP_OUT_UPPER) and np.all(point > GOAL_KEEP_OUT_LOWER) or point[2] < GOAL_UNDERGROUND_M:
            return False
    return True


class GoalPath:
    """A run's hand goal against time: the start goal for `settle_s`, then each `(goal, slide_s, hold_s)` in turn, and back to the start goal. A goal
    slides in spherical coordinates, as `EeGoalTrajectory.step` slides it: the policy has never seen one jump. `at(t)` gives (radius, pitch, yaw)."""

    def __init__(self, start, via, settle_s: float):
        now = tuple(float(v) for v in start)
        self.points: list[tuple[float, float, tuple, tuple]] = []
        t = float(settle_s)
        last_slide = MIN_SLIDE_S
        for goal, slide_s, hold_s in via:
            goal = tuple(float(v) for v in goal)
            for value, span, name in zip(goal, (GOAL_RADIUS_M, GOAL_PITCH_RAD, GOAL_YAW_RAD), ("radius", "pitch", "yaw")):
                if not span[0] <= value <= span[1]:
                    raise ValueError(f"goal {name} {value:.3f} is outside what the policy trained on, {span[0]:.3f} to {span[1]:.3f}")
            if slide_s < MIN_SLIDE_S:
                raise ValueError(f"a {slide_s} s slide: at least {MIN_SLIDE_S} s")
            if not goal_is_clear(now, goal):
                raise ValueError(f"the slide to {goal} passes through what training kept goals out of")
            self.points.append((t, t + slide_s, now, goal))
            t, now, last_slide = t + slide_s + hold_s, goal, float(slide_s)
        self.start = tuple(float(v) for v in start)
        if not goal_is_clear(now, self.start):
            raise ValueError("the slide back passes through what training kept goals out of")
        self.points.append((t, t + last_slide, now, self.start))
        self.duration = t + last_slide + BACK_HOLD_S

    def press_window(self) -> tuple[float, float]:
        """On the path's clock: the hand has reached the first goal, and is about to leave the last one for the start goal."""
        return self.points[0][1], self.points[-1][0]

    def at(self, t: float) -> np.ndarray:
        goal = self.start
        for begin, end, a, b in self.points:
            if t < begin:
                break
            u = min((t - begin) / (end - begin), 1.0)
            goal = tuple(x + (y - x) * u for x, y in zip(a, b))
        return np.array(goal)


#: The most the policy's force input trained on (`task_cfg.GRIPPER_FORCE_RANGE_N`), and how long a change of it takes.
MAX_FORCE_N, FORCE_RAMP_S = 8.0, 1.0


class PressForce:
    """The downward force to tell the policy against the path's clock: nothing until the hand has arrived, the given levels in equal parts, nothing again
    by the time it leaves. Each change is a ramp: the force is a move of the hand's target, and the policy has never seen one jump."""

    def __init__(self, levels, begin_s: float, end_s: float):
        self.levels = [float(v) for v in levels]
        if not self.levels or not all(0.0 <= v <= MAX_FORCE_N for v in self.levels):
            raise ValueError(f"forces are 0 to {MAX_FORCE_N} N, what the policy trained on: {self.levels}")
        self.begin, self.end = float(begin_s), float(end_s)
        self.part = (self.end - self.begin) / len(self.levels)
        if self.part < 2 * FORCE_RAMP_S:
            raise ValueError(f"{self.part:.1f} s a level: at least {2 * FORCE_RAMP_S} s")

    def at(self, t: float) -> float:
        if not self.begin < t < self.end:
            return 0.0
        k = min(int((t - self.begin) / self.part), len(self.levels) - 1)
        before = self.levels[k - 1] if k else 0.0
        level = before + (self.levels[k] - before) * min((t - self.begin - k * self.part) / FORCE_RAMP_S, 1.0)
        return level * min((self.end - t) / FORCE_RAMP_S, 1.0)


#: Just outside the dead zones of `leg_policy.WALK_DEAD_ZONE`, so a stick past its own dead zone walks and does not creep.
WALK_LEAST = (0.12, 0.12, 0.24)
#: What training asked at most (`task_cfg.LIN_VEL_*_RANGE`, `ANG_VEL_YAW_RANGE`).
WALK_MOST = (0.6, 0.4, 0.6)
#: A stick counts from `STICK_DEAD` and stops counting under `STICK_RELEASE`: with one edge, a thumb resting on it flips the command and resets the gait clock every few ticks.
STICK_DEAD, STICK_RELEASE = 0.2, 0.12


class WalkSticks:
    """The remote's sticks to a velocity: left stick forward and sideways, right stick turns. Inside `STICK_DEAD` nothing, then from just past the
    gait's dead zone to the most asked. Up on the left stick is forward; right on either is right, which is negative in the base frame."""

    def __init__(self, max_vx: float, max_vy: float, max_wz: float):
        self.most = (float(max_vx), float(max_vy), float(max_wz))
        self.pushed = [False, False, False]
        for most, least, ceiling in zip(self.most, WALK_LEAST, WALK_MOST):
            if not least <= most <= ceiling:
                raise ValueError(f"the most asked, {most}, is outside the gait's least, {least}, and what training asked, {ceiling}")

    def _axis(self, stick: float, k: int) -> float:
        size = abs(float(stick))
        self.pushed[k] = size >= (STICK_RELEASE if self.pushed[k] else STICK_DEAD)
        if not self.pushed[k]:
            return 0.0
        return math.copysign(WALK_LEAST[k] + max(size - STICK_DEAD, 0.0) / (1.0 - STICK_DEAD) * (self.most[k] - WALK_LEAST[k]), stick)

    def velocity(self, sticks: dict) -> tuple[float, float, float]:
        return self._axis(sticks["ly"], 0), self._axis(-sticks["lx"], 1), self._axis(-sticks["rx"], 2)


class ArmWatch:
    """Whether to go on sending. `check` gives None, or why to stop; once stopped it stays stopped until `reset`."""

    def __init__(self, max_report_gap_s: float = MAX_REPORT_GAP_S, max_lead_deg: float = MAX_LEAD_DEG,
                 max_step_deg: float = MAX_STEP_DEG, grace_s: float = GRACE_S, lead_hold_s: float = LEAD_HOLD_S,
                 max_lead_hard_deg: float = MAX_LEAD_HARD_DEG):
        self.max_gap, self.max_lead, self.max_step = float(max_report_gap_s), float(max_lead_deg), float(max_step_deg)
        self.grace, self.lead_hold, self.max_lead_hard = float(grace_s), float(lead_hold_s), float(max_lead_hard_deg)
        self.stopped: str | None = None
        #: The arm is answering and only the policy's asking is out of bounds: a stop for the arm, not for the legs. The first live run's lead stop
        #: dropped a dog whose legs were good to the floor (F-255). A dead feed or a bad number is not soft: nothing can be trusted to fold on.
        self.soft = False
        self.last_target: np.ndarray | None = None
        self.began: float | None = None
        self.lead_since: float | None = None

    def reset(self) -> None:
        self.stopped, self.last_target, self.began, self.lead_since, self.soft = None, None, None, None, False

    def begin(self, now_s: float) -> None:
        """The first goal is about to go: the grace runs from here. With no `begin` there is none."""
        self.began = float(now_s)

    def _stop(self, why: str, soft: bool = False) -> str:
        self.stopped, self.soft = why, soft
        return why

    def check(self, now_s: float, reports: ArmReports, target) -> str | None:
        """Before sending `target` (URDF radians) at `now_s`. A dead feed and a bad number stop it from the first tick."""
        if self.stopped is not None:
            return self.stopped
        target = np.asarray(target, dtype=float)
        if not np.all(np.isfinite(target)):
            return self._stop("the target is not finite")
        if reports.stamp is None:
            return self._stop("no report from the arm yet")
        age = reports.age(now_s)
        if age > self.max_gap:
            return self._stop(f"no report for {age:.2f} s (at most {self.max_gap})")
        if self.began is not None and now_s - self.began < self.grace:
            self.last_target = target.copy()
            return None
        lead = np.degrees(np.abs(target - reports.q))
        worst, joint = float(lead.max()), int(lead.argmax()) + 1
        if worst > self.max_lead_hard:
            return self._stop(f"joint {joint} {worst:.1f} deg ahead of its report (at most {self.max_lead_hard})", soft=True)
        if worst > self.max_lead:
            self.lead_since = now_s if self.lead_since is None else self.lead_since
            if now_s - self.lead_since >= self.lead_hold:
                return self._stop(f"joint {joint} {worst:.1f} deg ahead of its report for {now_s - self.lead_since:.1f} s (at most {self.max_lead})",
                                  soft=True)
        else:
            self.lead_since = None
        if self.last_target is not None:
            step = np.degrees(np.abs(target - self.last_target))
            if float(step.max()) > self.max_step:
                return self._stop(f"joint {int(step.argmax()) + 1} stepped {step.max():.1f} deg in a tick (at most {self.max_step})", soft=True)
        self.last_target = target.copy()
        return None


class ArmSender:
    """`follow` goals and a `hold` on the arm link's socket: the one writer to it. A failed write is a failed send, never an exception
    in the control loop: the link holds by itself when it hears nothing (`WATCHDOG_S`)."""

    def __init__(self, sock):
        self.sock = sock
        self.sent = 0
        self.held = False
        #: The jaw command, once there is one, on every message. The link takes a message with none as "wherever the jaws measure": on a cloth, that is letting go of it.
        self.gripper: float | None = None

    def _write(self, msg: dict) -> bool:
        if self.gripper is not None and msg["type"] != "hold":
            msg["gripper"] = round(float(self.gripper), 2)
        try:
            self.sock.sendall((json.dumps(msg) + "\n").encode())
            return True
        except OSError:
            return False

    def follow(self, q_rad) -> bool:
        """A goal in URDF radians, to servo degrees for the wire. Nothing goes after a hold."""
        if self.held:
            return False
        ok = self._write({"type": "follow", "q": [round(float(v), 3) for v in to_servo_deg(q_rad)]})
        self.sent += int(ok)
        return ok

    def hold(self) -> None:
        """Stop: the link commands the arm to where it is and streams no more. Once."""
        if not self.held:
            self.held = True
            self._write({"type": "hold"})

    def path(self, points, dt: float = 0.1) -> bool:
        """A timed joint move, servo degrees: the unfold and the fold. Clears a hold: it is the program's own next step, never a goal's."""
        self.held = False
        return self._write({"type": "path", "q": points, "dt": dt})

    def keepalive(self) -> None:
        """Still here, while a path plays: the link holds when it hears nothing."""
        self._write({"type": "keepalive"})
