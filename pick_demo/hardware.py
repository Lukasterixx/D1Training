"""The scripted pick, driven against the physical D1 instead of the simulator.

`PickSequence` never held a simulator handle: each `update` takes the time, the arm's feedback angles and
world up, pulls a camera frame through a callable when it needs one, and returns a joint target. This
module is the other end of that seam -- `D1Client` and a real RealSense in place of PhysX and a rendered
camera -- so the state machine, the grasp planner and the perception are the same objects the simulated
pick runs, not a reimplementation of them.

What differs from `run_pick_demo.py`, and why:

* **No ground truth.** The simulator scores each estimate against the cup's actual pose. Here there is
  nothing to score against, so a run reports what it did and what it saw, never how accurate it was.
* **`up_b` is told, not measured.** In simulation it comes from the Go2's IMU. On a bench there is no
  dog: the arm's mounting decides which way is up and it is an argument, recorded with every run. There
  is no longer a base height beside it. Nothing in the planner asks where the surface is -- the poses are
  placed from the arm's own mount and the cup is measured by the camera -- so the number an operator used
  to type, and get wrong, is gone rather than merely defaulted.
* **The gripper is commanded on an unverified scale.** Servo 6's units have never been measured on this
  arm: the protocol advertises a 65 mm jaw and the arm reports ~41 at rest, but nobody has put a ruler
  across the fingers at a known command. `grip_gripper=True` sends the sequence's chosen width through
  `finger_travel_to_gripper_units`, the same convention the simulator uses, carried in the same message
  as the arm pose. What the jaw physically does at that number is the open question, so a run records
  the units it asked for and the units the arm reported, and claims nothing about millimetres.
  `grip_gripper=False` restores the earlier behaviour of logging the intent and sending nothing.
* **A wide cup is grasped by its wall.** The jaws open to 77.2 mm on the CAD and the bench mug measured
  70-83 mm depending on how much rim a look saw (F-058), so a cup that wide is taken by its wall instead
  (`GraspParams.wall_grasp`): one finger inside it and one outside, closing on the wall, which this arm
  can do because zero travel now commands the pads together (-19.8 units, F-063) rather than the CAD's
  17.2 mm. Where the cup is too wide even to pinch, both fingers go inside and open against the wall --
  and then the gripper is holding the cup *open*, so closing it is what lets go. Nothing here moves the
  jaws except the sequence, and a stop holds the arm without touching them, so a stopped pick keeps
  whatever grip it had. What no run can claim either way is millimetres: the travel comes from the CAD
  finger geometry and reaches the arm through `finger_travel_to_gripper_units`, whose ends are measured
  (F-063) but whose middle is a straight line nobody has put a ruler against.
* **The camera is borrowed.** One process at a time can hold a RealSense and the console's camera window
  already has it, so frames come from that pipeline (`d1_ui.camera_feed.CameraPipeline.wait_frame`)
  rather than a second device handle.

Safety, and it is the whole reason this file is careful:

* Nothing moves unless `execute=True`. A dry run does everything else -- perception, planning, the whole
  state machine -- and simply does not write to the arm, which is how a pick should be watched first.
  Because the sequence waits for the arm to *arrive* before it moves on, a dry run against a stationary
  arm would stall at the first waypoint and prove nothing, so a dry run follows a **virtual** pose that
  slews toward each target at the arm's measured 70 deg/s (F-033). Every frame it looks at is a real
  frame from the real camera; only the arm's position is imagined, and both the log and the result say
  so. A dry run therefore tests perception, planning and the sequence -- not the arm's tracking.
* Every joint target goes through `D1Client.set_all_joint_angles`, so the 10 Hz ceiling (F-032), mode 0
  (F-031) and the hard-limit clamp apply exactly as they do to the console's own moves.
* `should_stop` is checked every cycle. Stopping commands the arm to its own measured pose
  (`hold_here`), which decelerates it under torque. It does **not** release: release drops the arm
  (F-028).
* A target further than `max_step_deg` from the current pose aborts rather than being sent. This is a
  sanity bound on a wild target, **not** a smoothness limit: a large step is normal here. The sequence
  commands one distant waypoint and lets the firmware plan its own trapezoid to it (F-035), so the first
  move out of the rest pose is legitimately tens of degrees. The cap exists to catch a planning bug that
  produces nonsense, and it sits above anything the sequence should ever ask for. Smoothness is the
  arm's own planner's job; clearance is the grasp planner's, which refuses plans that cross the proxy.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import math
import time

import numpy as np

import d1_hardware
import d1_ik
from .grasp import GraspParams
from .perception import Frame
from .sequence import PickSequence, Timing

# The pick's own loop rate. The arm accepts motion commands at 10 Hz (F-032) and reports at 9 Hz (F-020),
# so there is nothing to gain from spinning faster than the commands can leave.
COMMAND_PERIOD_S = 0.10

# A sanity bound on a single target, in degrees from the current pose. Deliberately generous: the joints
# span about 180 deg and a legitimate first move out of the folded rest pose crosses most of one, so
# anything tighter would refuse the sequence's own waypoints (F-035). This catches nonsense, not motion.
MAX_STEP_DEG = 120.0

# What a dry run's virtual arm does per cycle: the measured peak slew (F-033), so the pretended motion
# takes about as long as the real one and the sequence's timeouts mean the same thing in both modes.
VIRTUAL_RATE_DEG_S = 70.0


class _VirtualArm:
    """A stand-in pose for dry runs: it starts where the arm is and slews toward whatever is commanded.

    Only the *position* is imagined. The camera, the perception, the planner and the state machine are
    the real ones, so a dry run exercises everything except the arm. It exists because the sequence is
    feedback-driven: without something arriving at the waypoints, a dry run would stall on the first
    move and report a timeout instead of testing anything.
    """

    def __init__(self, servo_deg, rate_deg_s: float = VIRTUAL_RATE_DEG_S):
        self.servo = np.asarray(servo_deg, dtype=float).copy()
        self.rate = rate_deg_s
        self.target = self.servo.copy()

    def command(self, servo_deg) -> None:
        self.target = np.asarray(servo_deg, dtype=float).copy()

    def advance(self, dt_s: float) -> list[float]:
        step = self.rate * max(dt_s, 0.0)
        self.servo = self.servo + np.clip(self.target - self.servo, -step, step)
        return [float(v) for v in self.servo]


@dataclass
class PickResult:
    ok: bool
    state: str
    failure: str | None
    reason: str
    executed: bool
    elapsed_s: float
    events: list = field(default_factory=list)
    frames_looked_at: int = 0
    frames_with_cup: int = 0
    cup: dict | None = None
    commands_sent: int = 0
    gripper_intent: dict | None = None
    grasp: dict | None = None
    feedback: str = "arm"

    def as_dict(self) -> dict:
        return {"ok": self.ok, "state": self.state, "failure": self.failure, "reason": self.reason,
                "executed": self.executed, "elapsed_s": round(self.elapsed_s, 2),
                "frames_looked_at": self.frames_looked_at, "frames_with_cup": self.frames_with_cup,
                "cup": self.cup, "commands_sent": self.commands_sent,
                "gripper_intent": self.gripper_intent, "grasp": self.grasp, "feedback": self.feedback,
                "interpretation": (
                    ("Real camera and real perception, but the arm's pose was virtual: a dry run shows "
                     "that the pipeline closes, not that the arm can follow it."
                     if self.feedback != "arm" else
                     "Physical arm. No ground truth, so nothing here says how accurate the estimate was.")
                    + (" The gripper was commanded, on a scale nobody has measured: the units asked for "
                       "are recorded, what the jaw did in millimetres is not known."
                       if (self.gripper_intent or {}).get("commanded") else
                       " The gripper was not commanded, so this is a reach, a descent and a lift -- "
                       "not a grasp.")
                    + ("" if (self.grasp or {}).get("mode", "outside") == "outside" else
                       f" The cup was too wide for the jaws, so this was a {self.grasp['mode']} grasp of its "
                       "wall: the fingers went inside the cup and the gripper's travel decided whether they "
                       "ever touched it, on that same unmeasured scale."))}


class CameraPipelineFrames:
    """Frames for the pick, taken from the console's camera pipeline and stamped with the arm's pose.

    The pipeline captures on its own thread, so the joint angles that belong to a frame are not handed
    over with it. They are read here at the moment the frame is taken, which is close but not exact. The
    sequence only asks for frames after it has arrived at a viewpoint and waited `camera_delay_s`, so in
    practice the arm is stationary and the skew does not matter; while it is moving it would, which is
    why nothing here is used mid-motion.
    """

    def __init__(self, pipeline, client, up_b, *, timeout_s: float = 2.0, log=None):
        self.pipeline, self.client = pipeline, client
        self.up_b = np.asarray(up_b, dtype=float)
        self.timeout_s = timeout_s
        self.log = log
        self.last_seq = -1
        self.taken = 0
        self.missing_depth = 0

    def __call__(self) -> Frame | None:
        got = self.pipeline.wait_frame(self.last_seq, timeout_s=self.timeout_s)
        if got is None:
            if self.log:
                self.log("pick: no camera frame within the timeout")
            return None
        seq, rgb, depth, _stamp = got
        self.last_seq = seq
        if depth is None:
            self.missing_depth += 1
            if self.log and self.missing_depth == 1:
                self.log("pick: the camera is streaming without depth; the pick cannot place a cup without it")
            return None
        try:
            q = d1_ik.from_servo_deg(self.client.get_joint_angles())[0]
        except RuntimeError:
            return None
        self.taken += 1
        return Frame(rgb=rgb, depth=depth, q=np.asarray(q, dtype=float), up_b=self.up_b.copy(),
                     stamp_s=time.monotonic())


def run_pick(*, client, joints, links, camera_model, mount, perception, pipeline,
             up_b=(0.0, 0.0, 1.0), execute: bool = False,
             timing: Timing | None = None, should_stop=None, on_event=None,
             max_time_s: float = 180.0, max_step_deg: float = MAX_STEP_DEG,
             grasp_params=None, grip_gripper: bool = True, cycle_s: float = COMMAND_PERIOD_S,
             clock=time.monotonic, sleep=time.sleep) -> PickResult:
    """Drive `PickSequence` against the real arm until it finishes, fails, times out or is stopped.

    `execute=False` runs everything and sends nothing. `should_stop` is a callable checked every cycle.
    `on_event` receives one-line strings for the console's log.

    `clock` and `sleep` are injected so the tests can run the whole sequence without waiting out its
    real-time settles and timeouts. Against hardware they are the real ones and must stay that way: the
    10 Hz command ceiling (F-032) is a property of the arm, not of this loop.
    """
    up = np.asarray(up_b, dtype=float)
    up = up / np.linalg.norm(up)
    should_stop = should_stop or (lambda: False)
    log = on_event or (lambda line: None)

    sequence = PickSequence(joints, links, camera_model, mount, perception, timing=timing,
                            grasp_params=grasp_params)
    frames = CameraPipelineFrames(pipeline, client, up, log=log)

    log(f"pick: starting {'LIVE' if execute else 'dry run'}, up {np.round(up, 3).tolist()}")
    if not execute:
        log("pick: dry run -- the camera and the planner are real, the arm's pose is simulated")
    started = clock()
    virtual = None
    last_cycle = started
    commands_sent = 0
    gripper_intent = None
    last_gripper = None
    stop_reason = None

    while True:
        now = clock()
        t = now - started

        if should_stop():
            stop_reason = "stopped"
            break
        if t > max_time_s:
            stop_reason = f"gave up after {max_time_s:.0f} s"
            break

        try:
            servo = client.get_joint_angles()
        except RuntimeError:
            if t > 5.0:
                stop_reason = "no joint feedback from the arm"
                break
            sleep(cycle_s)
            continue

        if not execute:
            # Dry run: follow a virtual pose starting from where the arm actually is, so the sequence
            # sees arrivals and runs to the end instead of timing out on its first waypoint.
            if virtual is None:
                virtual = _VirtualArm(servo)
            servo = virtual.advance(now - last_cycle)
        last_cycle = now
        q_fb = d1_ik.from_servo_deg(servo)[0]

        command = sequence.update(t, q_fb, up, frames)

        gripper_units = (d1_hardware.finger_travel_to_gripper_units(command.gripper_m)
                         if grip_gripper else None)
        if command.gripper_m != last_gripper:
            last_gripper = command.gripper_m
            reported = None
            try:
                reported = round(float(client.get_gripper_units()), 1)
            except (RuntimeError, AttributeError):
                pass
            gripper_intent = {
                "finger_travel_m": round(float(command.gripper_m), 4),
                "commanded": bool(grip_gripper and execute),
                "units_asked": None if gripper_units is None else round(gripper_units, 1),
                "units_reported_before": reported,
                "why": ("servo 6's scale is unverified; the units asked for are recorded, the "
                        "millimetres they produce are not known"
                        if grip_gripper else
                        "gripper commanding is off (grip_gripper=False); nothing was sent"),
            }
            if grip_gripper:
                log(f"pick: gripper -> {gripper_units:.1f} units "
                    f"({1000 * command.gripper_m:.1f} mm per finger, unverified scale)"
                    + ("" if execute else ", dry run so not sent"))
            else:
                log(f"pick: gripper would go to {1000 * command.gripper_m:.1f} mm per finger (not sent)")

        if command.q is not None:
            target_servo = d1_ik.to_servo_deg(command.q)
            step = float(np.max(np.abs(np.asarray(target_servo) - np.asarray(servo))))
            if step > max_step_deg:
                stop_reason = (f"refused a {step:.1f} deg step (cap {max_step_deg:.0f} deg) in state "
                               f"'{sequence.state}'")
                break
            if execute:
                # One message carries the arm pose and the jaw, so they never arrive a cycle apart and
                # the gripper costs none of the ten command slots a second the arm allows (F-032).
                client.set_all_joint_angles(target_servo, gripper=gripper_units)
                commands_sent += 1
            else:
                virtual.command(target_servo)

        if sequence.done:
            break
        sleep(cycle_s)

    elapsed = clock() - started
    if stop_reason is not None and not sequence.done:
        sequence.log(elapsed, "aborted", reason=stop_reason)
        if execute:
            try:
                client.hold_here()     # decelerate under torque; release would drop the arm (F-028)
                log("pick: holding where it is (not released)")
            except Exception as exc:
                log(f"pick: could not command a hold ({exc})")

    ok = sequence.state == "done" and stop_reason is None
    reason = stop_reason or sequence.failure or ("finished" if ok else sequence.state)
    log(f"pick: {'done' if ok else 'ended'} in {elapsed:.1f} s -- {reason}")
    return PickResult(
        ok=ok, state=sequence.state, failure=sequence.failure, reason=reason, executed=execute,
        elapsed_s=elapsed, events=sequence.events, frames_looked_at=frames.taken,
        frames_with_cup=len(sequence.observations),
        cup=sequence.cup.as_dict() if sequence.cup is not None else None,
        commands_sent=commands_sent, gripper_intent=gripper_intent,
        grasp=sequence.plan.as_dict() if sequence.plan is not None else None,
        feedback="arm" if execute else "virtual (dry run)")


def run_metadata(*, camera_model, mount, up_b, execute, grip_gripper: bool = True,
                 grasp_params=None, extra=None) -> dict:
    """The record a hardware pick writes beside its events: what it assumed, and which of it was measured."""
    meta = {
        "started_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "status": "initializing", "mode": "pick_hardware", "learning": None,
        "task": "scripted_top_down_cup_pick_on_hardware",
        "executed": bool(execute),
        "camera": {**{k: v for k, v in camera_model.__dict__.items()}},
        "mount": {"pos_link6_m": [float(v) for v in mount.pos_link6],
                  "pitch_deg": getattr(mount, "pitch_deg", None),
                  "source": getattr(mount, "source", "assumed; bracket not measured")},
        "frame": {"up_b": [float(v) for v in np.asarray(up_b, dtype=float)],
                  "source": "stated by the operator; there is no IMU on a bench-mounted arm",
                  "surface": "not stated and not used: every pose is placed from the arm's own mount"},
        "gripper": {"commanded": bool(grip_gripper),
                    "units_range": list(d1_hardware.GRIPPER_UNITS_RANGE),
                    "conversion": "finger travel (m) * 2000 -> servo 6 units, as the simulator's client does",
                    "why": ("servo 6's scale is unverified on this arm: no finding measures what one unit "
                            "is in millimetres of jaw gap, so the units are recorded and no length is claimed")},
        "grasp": {"wall_grasp": (grasp_params or GraspParams()).wall_grasp,
                  "wall_thickness_m": (grasp_params or GraspParams()).wall_thickness_m,
                  "why": ("a cup too wide for the 77.2 mm jaws is grasped by its wall instead: the fingers "
                          "go inside the mouth shut and open against the wall. The wall thickness is "
                          "assumed, not seen, and the pinch variant stays refused until the real closed "
                          "jaw gap is measured (F-059)")},
        "scope": ("A success here is a reach, a descent, a close and a lift against a real cup, with "
                  "perception through an uncalibrated wrist mount and a jaw commanded on an unmeasured "
                  "scale. It is not a calibration of either."
                  if grip_gripper else
                  "A success here is a reach, a descent and a lift against a real cup, with perception "
                  "through an uncalibrated wrist mount. It is not a grasp and not a calibration."),
    }
    if extra:
        meta.update(extra)
    return meta


__all__ = ["run_pick", "run_metadata", "PickResult", "CameraPipelineFrames",
           "COMMAND_PERIOD_S", "MAX_STEP_DEG"]
