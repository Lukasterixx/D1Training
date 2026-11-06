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
it is. Never release: it is a torque-off with no brake (F-028). The goal is fixed for the run.

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


def read_report(get_stamp, get_angles, tries: int = 5):
    """A report's stamp and its angles as a pair, or None. They are separate reads, so one can land between them: the stamp, the
    angles, the stamp again, retried if it moved. For `D1Client.servo_report_s` and `get_joint_angles`."""
    for _ in range(tries):
        before = get_stamp()
        angles = get_angles()
        if get_stamp() == before:
            return before, angles
    return None


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


class ArmWatch:
    """Whether to go on sending. `check` gives None, or why to stop; once stopped it stays stopped until `reset`."""

    def __init__(self, max_report_gap_s: float = MAX_REPORT_GAP_S, max_lead_deg: float = MAX_LEAD_DEG,
                 max_step_deg: float = MAX_STEP_DEG, grace_s: float = GRACE_S, lead_hold_s: float = LEAD_HOLD_S,
                 max_lead_hard_deg: float = MAX_LEAD_HARD_DEG):
        self.max_gap, self.max_lead, self.max_step = float(max_report_gap_s), float(max_lead_deg), float(max_step_deg)
        self.grace, self.lead_hold, self.max_lead_hard = float(grace_s), float(lead_hold_s), float(max_lead_hard_deg)
        self.stopped: str | None = None
        self.last_target: np.ndarray | None = None
        self.began: float | None = None
        self.lead_since: float | None = None

    def reset(self) -> None:
        self.stopped, self.last_target, self.began, self.lead_since = None, None, None, None

    def begin(self, now_s: float) -> None:
        """The first goal is about to go: the grace runs from here. With no `begin` there is none."""
        self.began = float(now_s)

    def _stop(self, why: str) -> str:
        self.stopped = why
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
            return self._stop(f"joint {joint} {worst:.1f} deg ahead of its report (at most {self.max_lead_hard})")
        if worst > self.max_lead:
            self.lead_since = now_s if self.lead_since is None else self.lead_since
            if now_s - self.lead_since >= self.lead_hold:
                return self._stop(f"joint {joint} {worst:.1f} deg ahead of its report for {now_s - self.lead_since:.1f} s (at most {self.max_lead})")
        else:
            self.lead_since = None
        if self.last_target is not None:
            step = np.degrees(np.abs(target - self.last_target))
            if float(step.max()) > self.max_step:
                return self._stop(f"joint {int(step.argmax()) + 1} stepped {step.max():.1f} deg in a cycle (at most {self.max_step})")
        self.last_target = target.copy()
        return None


class ArmSender:
    """`follow` goals and a `hold` on the arm link's socket: the one writer to it. A failed write is a failed send, never an exception
    in the control loop: the link holds by itself when it hears nothing (`WATCHDOG_S`)."""

    def __init__(self, sock):
        self.sock = sock
        self.sent = 0
        self.held = False

    def _write(self, msg: dict) -> bool:
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
