"""Cartesian control of the *physical* D1 arm over its CycloneDDS protocol.

The simulator's Cartesian controller (`d1_ik_controller.py`) drives a `DirectD1`
client that writes to PhysX. This is the same seam against the real arm, so the
solver and the kinematics are shared and only the transport differs. `D1Client`
mirrors `DirectD1`'s methods (`poll`, `get_joint_angles`, `set_all_joint_angles`,
...) for that reason.

Protocol, as measured on the arm on 2026-09-16 (F-020, F-022) rather than taken
from the SDK headers:

  rt/arm_Command   ArmString_      JSON commands, angles in DEGREES
  rt/arm_Feedback  ArmString_      funcode 1 = angles (111 ms), funcode 3 = status (100 ms)
  current_servo_angle  PubServoInfo_   7 float32 servo angles (111 ms)

Three behaviours here are not what the vendor driver documents, and the code
depends on all three:

* **Feedback is 111 ms (9.0 Hz)**, not 10 Hz. Commanding faster does not make the
  arm observable faster, so `CartesianMover` paces itself off arrivals.
* **Enable is implicit.** `funcode 5 {"mode": 1}` does nothing; the arm energises
  itself when a motion command arrives. There is no arming step to rely on.
* **Power-off is ignored.** `funcode 6 {"power": 0}` does nothing, so the only
  software abort is release (`funcode 5 {"mode": 0}`).

**Release drops the arm.** There is no holding brake. `release()` removes torque,
and from an extended pose the arm falls to its folded mechanical stop -- measured
2026-09-16 at 60.8 deg of J1 and 35.1 deg of J2, uncommanded (F-028). It is an
emergency stop in the literal sense: it ends control, it does not hold position.
Nothing here releases the arm for you. Leave it energised and holding, and release
only deliberately, with the arm low or supported.

Nothing here moves the arm unless you pass `--execute`.
"""
# NOTE: do **not** add `from __future__ import annotations` to this module.
# CycloneDDS resolves an IdlStruct's field types by looking their names up in the
# defining module at runtime; PEP 563 turns them into strings and the lookup fails
# with "Type str as used in __main__ cannot be resolved". The wire types in
# `_message_types()` depend on real annotation objects.
import argparse
import json
import math
import os
import threading
import time
from dataclasses import dataclass

import numpy as np

import d1_ik

FEEDBACK_PERIOD_S = 0.111   # measured, F-020
COMMAND_PERIOD_S = 0.10     # the driver's streaming rate
MAX_COMMAND_HZ = 10.0       # hard ceiling on motion commands; see D1Client._pace
PEAK_RATE_DEG_S = 70.0      # measured on every joint, 68.6-74.1 deg/s (F-033)
ARM_IFACE = "enP8p1s0"      # the Go2 payload's arm-facing NIC
ARM_SERVOS = 6              # servos 0-5 are the IK chain; 6 is the gripper


def _message_types():
    """The two IDL types the arm uses, built at call time so importing this module
    does not require CycloneDDS.

    `PubServoInfo_` carries **float32** on the wire. cyclonedds-python maps a bare
    `float` to float64, which does not decode the arm's samples -- the vendor
    driver's dataclass has that bug and silently falls back to JSON parsing.
    """
    from cyclonedds.idl import IdlStruct
    from cyclonedds.idl.types import float32

    @dataclass
    class ArmString(IdlStruct, typename="unitree_arm::msg::dds_::ArmString_"):
        data_: str = ""

    @dataclass
    class PubServoInfo(IdlStruct, typename="unitree_arm::msg::dds_::PubServoInfo_"):
        servo0_data_: float32 = 0.0
        servo1_data_: float32 = 0.0
        servo2_data_: float32 = 0.0
        servo3_data_: float32 = 0.0
        servo4_data_: float32 = 0.0
        servo5_data_: float32 = 0.0
        servo6_data_: float32 = 0.0

    return ArmString, PubServoInfo


class D1Client:
    """Talks to the real arm. Mirrors `DirectD1`'s interface where it can."""

    def __init__(self, iface: str = ARM_IFACE, domain: int = 0):
        # Bind to the arm-facing NIC and drop whatever the ROS/Go2 stack left in
        # the environment, so this participant never inherits their DDS config.
        os.environ["CYCLONEDDS_URI"] = (
            "<CycloneDDS><Domain><General><Interfaces>"
            f"<NetworkInterface name='{iface}'/>"
            "</Interfaces></General></Domain></CycloneDDS>"
        )
        # Imported here, not at module scope: the solver and its tests must run
        # without CycloneDDS installed.
        from cyclonedds.core import InstanceState, ReadCondition, SampleState, ViewState, WaitSet
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.pub import DataWriter
        from cyclonedds.sub import DataReader
        from cyclonedds.topic import Topic
        from cyclonedds.util import duration

        ArmString, PubServoInfo = _message_types()
        self._ArmString = ArmString
        self._duration = duration

        dp = DomainParticipant(domain)
        self._dp = dp
        self._writer = DataWriter(dp, Topic(dp, "rt/arm_Command", ArmString))
        self._servo = DataReader(dp, Topic(dp, "current_servo_angle", PubServoInfo))
        self._fb = DataReader(dp, Topic(dp, "rt/arm_Feedback", ArmString))
        any_state = SampleState.Any | ViewState.Any | InstanceState.Any
        self._ws = WaitSet(dp)
        self._ws.attach(ReadCondition(self._servo, any_state))
        self._ws.attach(ReadCondition(self._fb, any_state))

        self._seq = 0
        # Motion commands are rate-limited. The arm's control cycle is 10 Hz and the
        # only public account of exceeding it (Zeng 2025) ends with an arm that stops
        # responding -- which this session then reproduced (F-032). Nothing in this
        # module may out-run the hardware, however tight the caller's loop is.
        self._min_command_interval_s = 1.0 / MAX_COMMAND_HZ
        self._last_motion_tx = 0.0
        self._angles_deg: list[float] | None = None
        self._status: dict | None = None
        self._last_rx = 0.0
        self._last_tx = 0.0
        self._lock = threading.Lock()

    @property
    def participant(self):
        """The DDS participant, for readers on other topics of the same domain
        (the Go2's `rt/lowstate` lives on the same NIC)."""
        return self._dp

    # ---------------------------------------------------------------- reading

    def poll(self, timeout_s: float = 0.0) -> bool:
        """Drain pending feedback into the cache. True if anything arrived."""
        got = False
        if timeout_s > 0:
            try:
                self._ws.wait(self._duration(milliseconds=int(timeout_s * 1000)))
            except Exception:
                pass
        for s in self._servo.take(N=50):
            with self._lock:
                self._angles_deg = [s.servo0_data_, s.servo1_data_, s.servo2_data_,
                                    s.servo3_data_, s.servo4_data_, s.servo5_data_,
                                    s.servo6_data_]
                self._last_rx = time.monotonic()
            got = True
        for s in self._fb.take(N=50):
            try:
                msg = json.loads(s.data_)
            except Exception:
                continue
            data = msg.get("data") or {}
            if msg.get("funcode") == 3:
                with self._lock:
                    self._status = data
            elif msg.get("funcode") == 1 and "angle0" in data:
                with self._lock:
                    self._angles_deg = [float(data[f"angle{i}"]) for i in range(7)]
                    self._last_rx = time.monotonic()
                got = True
        return got

    def wait_for_feedback(self, timeout_s: float = 3.0) -> list[float]:
        """Block until the arm has reported, then return all seven servo angles."""
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            self.poll(timeout_s=0.2)
            with self._lock:
                # Angles (111 ms) and status (100 ms) are separate streams, so wait
                # for both before reporting: status-None reads as "no arm", not
                # "the status message has not come round yet".
                ready = self._angles_deg is not None and self._status is not None
                angles = list(self._angles_deg) if self._angles_deg is not None else None
            if ready:
                return angles
        if angles is not None:
            return angles
        raise TimeoutError("no D1 feedback; check the arm's power and the DDS interface")

    def get_joint_angles(self) -> list[float]:
        """The six IK-chain servo angles in degrees. Matches `DirectD1`."""
        with self._lock:
            if self._angles_deg is None:
                raise RuntimeError("no feedback yet; call wait_for_feedback() first")
            return list(self._angles_deg[:ARM_SERVOS])

    def get_gripper_units(self) -> float:
        """Servo 6's raw value. **Not** millimetres: the stroke mapping is unverified,
        so this deliberately does not mimic `DirectD1.get_gripper_mm`."""
        with self._lock:
            if self._angles_deg is None:
                raise RuntimeError("no feedback yet")
            return float(self._angles_deg[6])

    @property
    def feedback_age_s(self) -> float:
        with self._lock:
            return time.monotonic() - self._last_rx if self._last_rx else float("inf")

    def is_powered(self) -> bool | None:
        with self._lock:
            return None if self._status is None else bool(self._status.get("power_status"))

    def is_enabled(self) -> bool | None:
        with self._lock:
            return None if self._status is None else bool(self._status.get("enable_status"))

    def error_status(self):
        with self._lock:
            return None if self._status is None else self._status.get("error_status")

    # ---------------------------------------------------------------- writing

    def _pace(self) -> None:
        """Block until at least one command interval has passed since the last motion
        command. Applied to funcode 1 and 2 only -- status and release must never be
        delayed, because release is the abort."""
        wait = self._min_command_interval_s - (time.monotonic() - self._last_motion_tx)
        if wait > 0:
            time.sleep(wait)
        self._last_motion_tx = time.monotonic()

    def _send(self, funcode: int, data: dict | None = None) -> str:
        self._seq = (self._seq % 9999) + 1
        msg = {"seq": self._seq, "address": 1, "funcode": funcode}
        if data is not None:
            msg["data"] = data
        payload = json.dumps(msg)
        self._writer.write(self._ArmString(data_=payload))
        self._last_tx = time.monotonic()
        return payload

    def set_all_joint_angles(self, angles_deg, mode: int = 0) -> str:
        """funcode 2. Six arm angles in degrees; the gripper is held where it is.

        **mode 0, not 1.** Unitree documents 0 as "small smoothing of 10 Hz data"
        and 1 as "large smoothing of trajectory-use". Measured on J0, 30 deg, from
        the same pose (F-031):

            mode 1: peak 13.5 deg/s, settle never (3 s), steady-state error 5.09 deg
            mode 0: peak 69.3 deg/s, settle 538 ms, steady-state error 0.20 deg

        Mode 1 is a slow interpolator, and commanding it at the feedback rate means
        re-commanding a waypoint the arm is still slewing toward. Everything the
        Cartesian loop blamed on the arm -- the ~6 mm residual, the 0.5-0.7 deg
        tracking offset, the stalls, the 10x step counts -- was this mode, not the
        hardware. Mode 0 is the streaming mode a control loop wants.

        Angles are clamped to the URDF's **hard** limits -- an out-of-range value is
        rejected by the arm, and a silently dropped command in a control loop is
        worse than a clamped one.

        Hard, not soft, deliberately. The soft limits (0.9 of range) are for the
        *solver*, to keep solutions off the stops. Clamping the wire to them would
        be a second, invisible controller: the arm rests at J1 = -90.9 deg, outside
        both (F-023), so a soft clamp turns the first command of any approach into a
        ~10 deg jump and silently defeats the caller's step cap.
        """
        angles = np.asarray(angles_deg, dtype=float).reshape(-1)[:ARM_SERVOS]
        # Clamp in servo space: a sign flip swaps a joint's low and high (F-030).
        lows, highs = d1_ik.servo_limits_deg(_JOINTS, soft=1.0)
        clamped = [float(v) for v in np.clip(angles, lows, highs)]
        try:
            gripper = self.get_gripper_units()
        except RuntimeError:
            gripper = 0.0
        data = {"mode": mode}
        data.update({f"angle{i}": round(a, 3) for i, a in enumerate(clamped)})
        data["angle6"] = round(gripper, 3)
        self._pace()
        return self._send(2, data)

    def set_joint_angle(self, servo_id: int, angle_deg: float, delay_ms: int = 0) -> str:
        """funcode 1, one servo. `delay_ms` is the arm's own move duration."""
        if not 0 <= servo_id <= 6:
            raise ValueError(f"servo_id must be 0-6, got {servo_id}")
        self._pace()
        return self._send(1, {"id": int(servo_id), "angle": round(float(angle_deg), 3),
                              "delay_ms": int(delay_ms)})

    # Folded-enough for a release to be a short settle rather than a fall. J1 and
    # J2 carry the arm's weight; near their stops the links are already down.
    SAFE_RELEASE_J1_DEG = -80.0
    SAFE_RELEASE_J2_DEG = 80.0

    def is_safe_to_release(self) -> bool:
        """True when the arm is folded enough that losing torque is not a fall."""
        try:
            angles = self.get_joint_angles()
        except RuntimeError:
            return False
        return angles[1] <= self.SAFE_RELEASE_J1_DEG and angles[2] >= self.SAFE_RELEASE_J2_DEG

    def hold_here(self) -> str:
        """Command the arm to its own measured pose: a controlled stop.

        The only way to interrupt a waypoint the arm is still executing. It keeps
        torque on and decelerates in place, unlike `release`, which drops it
        (F-028). Used by STOP during a single-shot move.
        """
        return self.set_all_joint_angles(self.get_joint_angles())

    def release(self) -> str:
        """funcode 5 mode 0: the only software stop that works (F-022).

        **This drops the arm.** There is no holding brake, so from an extended pose
        it falls to its folded stop (F-028). Check `is_safe_to_release()` first
        unless you are deliberately aborting.
        """
        return self._send(5, {"mode": 0})

    def set_motor_power(self, on: bool) -> str:
        """funcode 6. Power-on works; **power-off is ignored by the arm** (F-022)."""
        return self._send(6, {"power": 1 if on else 0})

    def return_to_zero(self) -> str:
        """funcode 7. Drives every joint to zero -- from a folded rest pose that is
        a large, fast motion. Never call it without clearance around the arm."""
        return self._send(7)

    def close(self) -> None:
        try:
            self.release()
        except Exception:
            pass


_JOINTS, _LINKS = d1_ik.load_urdf()


@dataclass
class MoveStep:
    """One command in an approach, and what the arm reported after it."""

    index: int
    commanded_deg: list[float]
    measured_deg: list[float]
    tool_pos_m: list[float]
    target_error_m: float


class CartesianMover:
    """Moves the tool point to a Cartesian target, one bounded step per cycle.

    The arm is commanded in joint space; IK re-solves from the *measured*
    configuration each cycle, so a joint that lags or refuses is seen rather than
    integrated over. Each command is capped at `max_joint_step_deg`, which is what
    stops a solution on the far side of the workspace becoming one violent swing.
    """

    def __init__(self, client: D1Client, joints=None, *,
                 max_joint_step_deg: float = 5.0,
                 base_height_m: float | None = None,
                 body: str = d1_ik.TOOL_BODY,
                 feedback_period_s: float = FEEDBACK_PERIOD_S,
                 level: bool = False):
        self._client = client
        self._joints = joints if joints is not None else _JOINTS
        self._max_step = float(max_joint_step_deg)
        self._base_height = base_height_m
        self._body = body
        self._level = bool(level)
        self._feedback_period = float(feedback_period_s)
        self.last_stop_reason: str | None = None
        self.last_settle_reason: str | None = None
        self.last_cycles: list[tuple[float, list[float]]] = []

    # `current_deg` is threaded through explicitly so a dry run can rehearse from
    # a hypothetical configuration without the client having to lie about one.
    def _current_deg(self, current_deg=None) -> list[float]:
        return list(current_deg) if current_deg is not None else self._client.get_joint_angles()

    def _check_clearance(self, q_here, plan, where: str = "") -> None:
        """Refuse a move whose ENDPOINT or PATH enters the trunk/ground proxy.

        The endpoint test alone is not enough: a move between two clear poses can
        sweep the arm through the dog (F-036). Skipped entirely when no base height
        is configured, because the proxy needs one.
        """
        if self._base_height is None:
            return
        suffix = f" {where}" if where else ""
        if not plan.clear_of_body:
            raise RuntimeError(f"IK solution fails the trunk/ground clearance proxy{suffix}; refusing to move")
        clear, frac = d1_ik.path_clearance(self._joints, q_here, plan.q, self._base_height)
        if not clear:
            raise RuntimeError(
                f"the path to that solution enters the trunk/ground proxy {100*frac:.0f}% along{suffix}, "
                "though both endpoints are clear; refusing to move")

    def measured_q(self) -> np.ndarray:
        return d1_ik.from_servo_deg(self._client.get_joint_angles())[0]

    def tool_now(self, current_deg=None):
        return d1_ik.tool_pose(self._joints, d1_ik.from_servo_deg(self._current_deg(current_deg))[0])

    def plan(self, target_pos, target_rot=None, current_deg=None) -> d1_ik.IKResult:
        """Solve seeded at the given configuration. Commands nothing."""
        q0 = d1_ik.from_servo_deg(self._current_deg(current_deg))[0]
        return d1_ik.solve(self._joints, target_pos, target_rot, q0=q0,
                           body=self._body, base_height=self._base_height,
                           level=self._level and target_rot is None,
                           max_iterations=400 if self._level else 200)

    def step_towards(self, goal_servo_deg, current_deg=None) -> list[float]:
        """The next commandable joint vector: `goal_servo_deg` capped to one step.

        Everything here is **servo degrees**, the arm's own convention, because the
        measured pose it is compared against is. Callers holding a solver result
        (URDF radians) must convert with `d1_ik.to_servo_deg` first -- the two
        spaces differ by `SERVO_SIGN` (F-030) and silently mixing them mirrors a
        joint.
        """
        current = np.asarray(self._current_deg(current_deg), dtype=float)
        goal = np.asarray(goal_servo_deg, dtype=float).reshape(-1)[:ARM_SERVOS]
        delta = goal - current
        longest = float(np.max(np.abs(delta))) if delta.size else 0.0
        if longest > self._max_step:
            delta = delta * (self._max_step / longest)
        return list(current + delta)

    def settle(self, target_pos, seconds: float = 2.0, hold_command=None):
        """Let the arm arrive and measure where it actually got to.

        `hold_command` re-sends one fixed joint vector every feedback cycle instead
        of going silent. That is not a detail: funcode 2 mode 0 is a **streaming**
        mode, so when the stream stops the arm stops wherever it had reached,
        typically a few tenths of a degree short. Holding the stream on the final
        target is what actually closes the gap (F-031).

        The approach loop re-solves every feedback cycle, which re-commands before
        the arm has finished moving; F-026 measured the arm still closing 3.0 -> 0.4
        deg over four cycles under a constant command. So the error at the end of
        the loop overstates the arm's real accuracy. This publishes nothing and
        simply watches, which is the honest way to quote a final number.

        Returns (settled_error_m, measured_deg, history) where `history` is the
        error at each feedback sample, so the settling can be seen rather than
        assumed.
        """
        end = time.monotonic() + seconds
        target = np.asarray(target_pos, dtype=float)
        history = []
        last = None
        while time.monotonic() < end:
            if hold_command is not None:
                self._client.set_all_joint_angles(hold_command)
            self._client.poll(timeout_s=0.2)
            measured = self._client.get_joint_angles()
            if measured == last:
                continue
            last = measured
            pos, _ = d1_ik.tool_pose(self._joints, d1_ik.from_servo_deg(measured)[0])
            history.append((round(time.monotonic() - (end - seconds), 3),
                            float(np.linalg.norm(target - pos))))
        measured = self._client.get_joint_angles()
        pos, _ = d1_ik.tool_pose(self._joints, d1_ik.from_servo_deg(measured)[0])
        return float(np.linalg.norm(target - pos)), measured, history

    def approach_joints(self, target_deg, *, execute: bool = False,
                        max_cycles: int = 200, tolerance_deg: float = 0.5,
                        on_step=None, should_stop=None) -> list[MoveStep]:
        """Move to a joint-space target under the same step cap as `run`.

        Needed wherever a pose matters more than a tool point -- parking the arm
        back in its folded rest pose is the usual case, and IK cannot express it:
        the rest pose sits outside the joint limits (F-023), so solving for its
        tool point returns a different, less folded configuration.

        `tolerance_deg` is compared against the worst joint. It defaults well
        above the arm's ~0.7 deg tracking offset (F-026), which no amount of
        re-commanding will close.
        """
        target = np.asarray(target_deg, dtype=float).reshape(-1)[:ARM_SERVOS]
        history: list[MoveStep] = []
        rehearsal = None if execute else self._client.get_joint_angles()
        self.last_stop_reason = "max_cycles"
        for index in range(max_cycles):
            if should_stop is not None and should_stop():
                self.last_stop_reason = "cancelled"
                break
            if execute:
                self._client.poll(timeout_s=0.3)
            commanded = self.step_towards(target, current_deg=rehearsal)
            if execute:
                self._client.set_all_joint_angles(commanded)
                deadline = time.monotonic() + 3.0
                time.sleep(self._feedback_period)
                while (self._client.feedback_age_s > 2 * self._feedback_period
                       and time.monotonic() < deadline):
                    self._client.poll(timeout_s=0.2)
                measured = self._client.get_joint_angles()
            else:
                rehearsal = commanded
                measured = commanded

            worst = float(np.max(np.abs(np.asarray(measured) - target)))
            pos, _ = d1_ik.tool_pose(self._joints, d1_ik.from_servo_deg(measured)[0])
            step = MoveStep(index, [round(c, 2) for c in commanded],
                            [round(m, 2) for m in measured],
                            [round(float(v), 4) for v in pos], worst)
            history.append(step)
            if on_step is not None:
                on_step(step)
            if worst <= tolerance_deg:
                self.last_stop_reason = "reached"
                break
        return history

    def run_oneshot(self, target_pos, target_rot=None, *, execute: bool = False,
                    passes: int = 2, settle_timeout_s: float = 6.0,
                    quiet_cycles: int = 3, max_excursion_deg: float = 70.0,
                    min_effective_deg: float = 0.6, should_stop=None, on_event=None):
        """Send the whole solution as one waypoint and let the arm drive itself there.

        The per-cycle loop in `run()` cannot move smoothly: the arm needs about two
        feedback cycles to reach cruise (F-035), so a waypoint issued every cycle
        restarts the acceleration phase forever and the arm never exceeds ~60% of
        its speed. Worse, every restart is a decelerate/accelerate pair, which is
        the stepping and the wobble.

        One waypoint instead gets one clean trapezoid at ~70 deg/s. The cost is that
        nothing corrects mid-flight, so accuracy comes from `passes`: move, let it
        settle, re-solve from where it actually arrived, move again.

        `max_excursion_deg` is the safety guard that the step cap used to provide.
        A single waypoint is one uninterrupted motion, so a far-away solution is
        refused rather than flown. STOP works through `should_stop`, which commands
        the arm to its measured pose and lets it decelerate in place.
        """
        history: list[MoveStep] = []
        self.last_cycles = []
        self.last_stop_reason = "reached"
        for attempt in range(max(1, passes)):
            if should_stop is not None and should_stop():
                self.last_stop_reason = "cancelled"
                break
            plan = self.plan(target_pos, target_rot)
            if not plan.converged:
                self.last_stop_reason = "unreachable"
                break
            here = self._client.get_joint_angles()
            self._check_clearance(d1_ik.from_servo_deg(here)[0], plan)
            goal = d1_ik.to_servo_deg(plan.q)
            excursion = float(np.max(np.abs(np.asarray(goal) - np.asarray(here))))
            if excursion > max_excursion_deg:
                raise RuntimeError(
                    f"single-shot excursion {excursion:.1f} deg exceeds the {max_excursion_deg:.0f} deg "
                    "limit; approach in stages or raise --max-excursion-deg deliberately")
            if attempt and excursion < min_effective_deg:
                # Below the arm's small-increment floor: commanding it moves nothing
                # and costs a settle window. The residual here is the floor itself.
                self.last_stop_reason = "at the arm's resolution"
                if on_event:
                    on_event(f"  pass {attempt}: correction {excursion:.2f} deg is below the "
                             f"{min_effective_deg} deg floor; stopping")
                break
            if on_event:
                on_event(f"  pass {attempt}: one waypoint, largest joint move {excursion:.1f} deg")
            if not execute:
                measured = goal
            else:
                self._client.set_all_joint_angles(goal)
                measured = self._await_still(settle_timeout_s, quiet_cycles, should_stop)
            pos, _ = d1_ik.tool_pose(self._joints, d1_ik.from_servo_deg(measured)[0])
            error = float(np.linalg.norm(np.asarray(target_pos, dtype=float) - pos))
            history.append(MoveStep(attempt, [round(v, 2) for v in goal],
                                    [round(v, 2) for v in measured],
                                    [round(float(v), 4) for v in pos], error))
            if on_event:
                on_event(f"    arrived at {error*1000:.2f} mm  [{self.last_settle_reason}]")
            if self.last_stop_reason == "cancelled":
                break
        return history

    def _await_still(self, timeout_s: float, quiet_cycles: int, should_stop=None,
                     start_grace_s: float = 1.0):
        """Watch until the arm has moved and then stopped. Returns its resting pose.

        Two phases, and the first one matters: the arm takes 60-130 ms to react
        (F-021), so the samples right after a command still show it stationary.
        Counting those as "settled" returns before the motion even begins, which
        reads as the arm stopping far short of its target. Quiet cycles are only
        counted once movement has actually been seen.
        """
        end = time.monotonic() + timeout_s
        grace_until = time.monotonic() + start_grace_s
        last = self._client.get_joint_angles()
        quiet = 0
        started = False
        samples = 0
        self.last_settle_reason = "timeout"
        while time.monotonic() < end:
            if should_stop is not None and should_stop():
                self._client.hold_here()
                self.last_stop_reason = "cancelled"
                self.last_settle_reason = "cancelled"
                break
            # Only judge stillness on a FRESH sample. The loop can spin faster than
            # the 111 ms feedback (F-020), and re-reading the same cached angles
            # looks exactly like a stationary arm -- three of those in a row ended
            # the wait mid-slew and reported the arm stopping short.
            if not self._client.poll(timeout_s=0.2):
                continue
            samples += 1
            now = self._client.get_joint_angles()
            self.last_cycles.append((time.monotonic(), list(now)))
            moved = max(abs(a - b) for a, b in zip(now, last))
            last = now
            if moved > 0.15:
                started = True
                quiet = 0
                continue
            if not started:
                # Still waiting for it to react; give up only after the grace period.
                if time.monotonic() > grace_until:
                    self.last_settle_reason = f"never started ({samples} samples)"
                    break
                continue
            quiet += 1
            if quiet >= quiet_cycles:
                self.last_settle_reason = f"still after {samples} samples"
                break
        return self._client.get_joint_angles()

    def run(self, target_pos, target_rot=None, *, execute: bool = False,
            max_cycles: int = 200, tolerance_m: float = 0.005,
            stall_patience: int = 12, stall_epsilon_m: float = 0.0005,
            on_step=None, should_stop=None) -> list[MoveStep]:
        """Approach `target_pos`, one bounded command per feedback cycle.

        Stops on one of four conditions, recorded in `last_stop_reason`:
        `"reached"`, `"stalled"`, `"max_cycles"`, or `"cancelled"` when
        `should_stop()` returns true. Cancelling simply stops commanding: the arm
        keeps holding wherever it is, which on this hardware is the only abort
        that does not drop it (F-028).

        Stalling is the normal outcome of a tight tolerance on this arm. The D1
        lands up to ~0.7 deg from a commanded angle (F-026), which is several
        millimetres at the tool, and re-solving cannot remove a plant offset --
        the controller just commands the same correction forever. So progress is
        watched directly: `stall_patience` cycles without improving on the best
        error by `stall_epsilon_m` ends the approach.

        With `execute=False` nothing is published. The first plan still comes from
        the arm's measured pose, but subsequent steps follow the commanded
        trajectory, since the real arm is not moving to supply the next one. So a
        dry run shows the path the arm would be asked to take -- not the path it
        would actually achieve, and in particular it cannot show this stall.
        """
        history: list[MoveStep] = []
        self.last_cycles: list[tuple[float, list[float]]] = []
        rehearsal = None if execute else self._client.get_joint_angles()
        best_error = float("inf")
        since_improvement = 0
        self.last_stop_reason = "max_cycles"
        for index in range(max_cycles):
            if should_stop is not None and should_stop():
                self.last_stop_reason = "cancelled"
                break
            if execute:
                self._client.poll(timeout_s=0.3)
            plan = self.plan(target_pos, target_rot, current_deg=rehearsal)
            self._check_clearance(d1_ik.from_servo_deg(self._current_deg(rehearsal))[0], plan,
                                  where=f"at step {index}")
            commanded = self.step_towards(d1_ik.to_servo_deg(plan.q), current_deg=rehearsal)

            if execute:
                self._client.set_all_joint_angles(commanded)
                # Wait for a fresh sample so the next solve sees the new pose.
                deadline = time.monotonic() + 3.0
                time.sleep(self._feedback_period)
                while (self._client.feedback_age_s > 2 * self._feedback_period
                       and time.monotonic() < deadline):
                    self._client.poll(timeout_s=0.2)
                measured = self._client.get_joint_angles()
            else:
                rehearsal = commanded
                measured = commanded

            self.last_cycles.append((time.monotonic(), list(measured)))
            pos, _ = d1_ik.tool_pose(self._joints, d1_ik.from_servo_deg(measured)[0])
            error = float(np.linalg.norm(np.asarray(target_pos, dtype=float) - pos))
            step = MoveStep(index, [round(c, 2) for c in commanded],
                            [round(m, 2) for m in measured],
                            [round(float(v), 4) for v in pos], error)
            history.append(step)
            if on_step is not None:
                on_step(step)

            if error < tolerance_m and plan.converged:
                self.last_stop_reason = "reached"
                break
            if error < best_error - stall_epsilon_m:
                best_error = error
                since_improvement = 0
            else:
                since_improvement += 1
                if since_improvement >= stall_patience:
                    self.last_stop_reason = "stalled"
                    break
        return history


def analyse_sweep(samples, t_cmd, joint, start_deg, target_deg,
                  move_eps_deg: float = 0.3, settle_tol_deg: float = 0.3,
                  t_end: float | None = None):
    """Timing of one commanded joint step, from a list of (t, [servo deg]) samples.

    Pure, so it is unit-tested against synthetic traces rather than the arm.

    Every figure is bounded by the 111 ms feedback cycle (F-020): latency is an
    **upper** bound (true value is this minus up to one sample period) and peak
    rate a **lower** bound, since a fast move spans only a handful of samples.
    `move_eps_deg` sits above the 0.1 deg quantisation and the <=0.05 deg resting
    noise, so a detection is real motion.
    """
    # `t_end` bounds the window at the next command. Without it the outbound leg's
    # statistics swallow the return leg, which makes the steady-state error read as
    # the full amplitude and the hold band as the whole sweep.
    after = [(t, a[joint]) for t, a in samples
             if t >= t_cmd and (t_end is None or t < t_end)]
    if not after:
        return {"samples": 0}

    t_move = next((t for t, v in after if abs(v - start_deg) > move_eps_deg), None)
    rates = [abs(b[1] - a[1]) / (b[0] - a[0])
             for a, b in zip(after, after[1:]) if b[0] > a[0]]
    peak = max(rates) if rates else 0.0

    settle = None
    for i, (t, _) in enumerate(after):
        window = [v for _, v in after[i:i + 5]]
        if window and all(abs(v - target_deg) <= settle_tol_deg for v in window):
            settle = t - t_cmd
            break

    tail = [v for t, v in after if t - t_cmd > 1.5]
    if not tail and after:           # a short window: use the last quarter
        tail = [v for _, v in after[-max(1, len(after) // 4):]]
    return {
        "samples": len(after),
        "commanded_deg": float(target_deg - start_deg),
        "latency_s": None if t_move is None else float(t_move - t_cmd),
        "peak_rate_deg_s": float(peak),
        "peak_rate_rad_s": float(peak * math.pi / 180.0),
        "settle_s": settle,
        "steady_state_error_deg": None if not tail else float(np.mean(tail) - target_deg),
        "hold_band_deg": None if not tail else float(max(tail) - min(tail)),
    }


def sweep_joint(client: "D1Client", joint: int, amplitude_deg: float, hold_s: float = 3.0,
                on_event=None, funcode: int = 2, mode: int = 0):
    """Step one joint by +amplitude, hold, return, hold. Records every sample.

    Only the named joint is commanded; with `funcode=2` the rest are held at their
    measured values, so whatever the joint does is attributable to it.

    `funcode` selects the command path, and they are **not** equivalent:
    2 is the all-joint waypoint command the Cartesian controller uses ("large
    smoothing of trajectory-use"), 1 is the single-joint command F-021 measured.
    """
    client.wait_for_feedback()
    base = client.get_joint_angles()
    start = base[joint]
    target = start + amplitude_deg

    samples: list[tuple[float, list[float]]] = []
    t0 = time.monotonic()

    def collect(until):
        while time.monotonic() < until:
            if client.poll(timeout_s=0.05):
                samples.append((time.monotonic() - t0, client.get_joint_angles()))

    collect(time.monotonic() + 1.0)          # baseline

    t_out = time.monotonic() - t0
    if funcode == 1:
        client.set_joint_angle(joint, target)
    else:
        out_cmd = list(base); out_cmd[joint] = target
        client.set_all_joint_angles(out_cmd, mode=mode)
    if on_event:
        on_event(f"  J{joint}: {start:+.1f} -> {target:+.1f} deg")
    collect(time.monotonic() + hold_s)

    t_back = time.monotonic() - t0
    if funcode == 1:
        client.set_joint_angle(joint, start)
    else:
        back_cmd = list(client.get_joint_angles()); back_cmd[joint] = start
        client.set_all_joint_angles(back_cmd, mode=mode)
    if on_event:
        on_event(f"  J{joint}: back to {start:+.1f} deg")
    collect(time.monotonic() + hold_s)

    return {
        "joint": joint,
        "amplitude_deg": amplitude_deg,
        "start_deg": start,
        "out": analyse_sweep(samples, t_out, joint, start, target, t_end=t_back),
        "back": analyse_sweep(samples, t_back, joint, target, start),
        "samples": samples,
    }


def analyse_motion(cycles, step_cap_deg: float, peak_rate_deg_s: float = PEAK_RATE_DEG_S):
    """Is the arm moving continuously, or arriving early and waiting?

    `cycles` is the per-cycle (t, measured servo deg) trace of an approach. The
    telling number is how far the arm actually travels per command cycle:

    * displacement ~= `step_cap_deg`  -> the arm reaches each waypoint with time
      to spare, decelerates to a stop and idles until the next command. Every
      cycle is a full accel/decel profile, which is what reads as stepping and
      what shakes the arm.
    * displacement ~= rate * period   -> the arm is still slewing when the next
      waypoint lands, so it never decelerates. Continuous motion.

    Feedback is 111 ms (F-020), so motion *within* a cycle cannot be resolved;
    this infers the duty cycle from displacement rather than observing it.
    """
    if len(cycles) < 3:
        return {"cycles": len(cycles)}
    t = np.asarray([c[0] for c in cycles], dtype=float)
    a = np.asarray([c[1] for c in cycles], dtype=float)
    dt = np.diff(t)
    disp = np.max(np.abs(np.diff(a, axis=0)), axis=1)      # worst joint per cycle
    moving = disp > 0.3                                     # above quantisation
    speed = disp[moving] / dt[moving] if moving.any() else np.array([0.0])
    period = float(np.median(dt))
    # The reference must come from OUTSIDE the trace: the observed speed is itself
    # capped by the step size, so comparing the trace against itself would call
    # every approach continuous.
    saturated = peak_rate_deg_s * period
    travelled = float(np.median(disp[moving])) if moving.any() else 0.0
    return {
        "cycles": len(cycles),
        "period_s": period,
        "step_cap_deg": step_cap_deg,
        "displacement_per_cycle_deg": travelled,
        "speed_deg_s": float(np.median(speed)),
        "saturated_displacement_deg": saturated,
        # 1.0 means the arm is speed-limited (always moving); well below means the
        # step cap binds first and the arm idles out the rest of each cycle.
        "duty": float(min(1.0, travelled / saturated)) if saturated else 0.0,
        "continuous": bool(saturated and travelled >= 0.85 * saturated),
    }


def measure_hold(client: "D1Client", joints, seconds: float = 15.0, stream: bool = False,
                 body: str = d1_ik.TOOL_BODY, resolve_target=None):
    """Quantify how still the arm holds, in joint and tool space.

    Three regimes, because they behave differently:
    `stream=False` sends nothing; `stream=True` re-sends one **fixed** pose; and
    `resolve_target` re-solves IK from the measured pose every cycle and commands
    the result, which is what a Cartesian loop does when it sits on its target.
    The last one feeds the encoder's 0.1 deg quantisation back into the command. The difference
    is what decides whether anything mounted on the end effector -- a camera --
    sees a steady scene.

    Tool figures come from FK of the reported angles, so they inherit the 0.1 deg
    encoder quantisation: a still arm cannot read better than about +/-0.1 deg,
    and motion below that is invisible here rather than absent.
    """
    client.wait_for_feedback()
    held = client.get_joint_angles()
    samples: list[tuple[float, list[float]]] = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        if resolve_target is not None:
            q0 = d1_ik.from_servo_deg(client.get_joint_angles())[0]
            plan = d1_ik.solve(joints, resolve_target, q0=q0, body=body)
            client.set_all_joint_angles(d1_ik.to_servo_deg(plan.q))
        elif stream:
            client.set_all_joint_angles(held)
        if client.poll(timeout_s=0.05):
            samples.append((time.monotonic() - t0, client.get_joint_angles()))

    angles = np.asarray([a for _, a in samples], dtype=float)
    tools = np.asarray([d1_ik.tool_pose(joints, d1_ik.from_servo_deg(a)[0])[0] for a in angles])
    return {
        "stream": stream,
        "regime": ("resolve" if resolve_target is not None else "stream" if stream else "silent"),
        "seconds": seconds,
        "samples": len(samples),
        "held_command_deg": held,
        "joint_stdev_deg": angles.std(axis=0).tolist() if len(angles) else [],
        "joint_ptp_deg": (angles.max(axis=0) - angles.min(axis=0)).tolist() if len(angles) else [],
        "tool_stdev_mm": (tools.std(axis=0) * 1000).tolist() if len(tools) else [],
        "tool_ptp_mm": ((tools.max(axis=0) - tools.min(axis=0)) * 1000).tolist() if len(tools) else [],
        "tool_rms_mm": float(np.linalg.norm(tools - tools.mean(axis=0), axis=1).std() * 1000) if len(tools) else 0.0,
        "tool_max_excursion_mm": float(np.linalg.norm(tools - tools.mean(axis=0), axis=1).max() * 1000) if len(tools) else 0.0,
    }


# --------------------------------------------------------------------- CLI


def _fmt(vec, places=3):
    return "[" + ", ".join(f"{float(v):+.{places}f}" for v in vec) + "]"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default=ARM_IFACE)
    sub = ap.add_subparsers(dest="command", required=True)

    w = sub.add_parser("watch", help="Read-only: stream measured joints and the tool point.")
    w.add_argument("--seconds", type=float, default=10.0)

    sub.add_parser("fk", help="Read-only: forward kinematics of the arm's current pose.")

    s = sub.add_parser("ik", help="Solve for a Cartesian target and print the plan. Never moves.")
    s.add_argument("--target", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"))
    s.add_argument("--from-zero", action="store_true",
                   help="Seed from the zero pose instead of the arm's measured pose (works offline).")
    s.add_argument("--base-height", type=float, default=None,
                   help="Enable the trunk/ground clearance proxy at this base height (m).")

    m = sub.add_parser("move", help="Approach a Cartesian target. Dry run unless --execute.")
    m.add_argument("--target", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"))
    m.add_argument("--execute", action="store_true", help="Actually command the arm.")
    m.add_argument("--max-joint-step-deg", type=float, default=5.0)
    m.add_argument("--max-cycles", type=int, default=200)
    m.add_argument("--tolerance-m", type=float, default=0.005)
    m.add_argument("--base-height", type=float, default=0.15,
                   help="Base height for the trunk/ground clearance proxy; 0 disables it. "
                        "The proxy assumes the STANDING trunk box, so it is a coarse guard.")
    m.add_argument("--level", action="store_true",
                   help="Finish with the gripper level: approach axis horizontal, no roll, heading free.")
    m.add_argument("--oneshot", action="store_true",
                   help="Send the solution as ONE waypoint and let the arm drive itself: smooth, fast.")
    m.add_argument("--passes", type=int, default=2, help="--oneshot: move, settle, re-solve, move again.")
    m.add_argument("--max-excursion-deg", type=float, default=70.0,
                   help="--oneshot safety limit on the largest single joint move.")
    m.add_argument("--hold-stream", action="store_true",
                   help="During settling keep streaming the final command instead of going silent.")
    m.add_argument("--settle-s", type=float, default=2.0,
                   help="After the approach, stop commanding and watch the arm arrive (F-026).")

    p = sub.add_parser("park", help="Bounded joint-space move, e.g. back to the folded rest pose.")
    p.add_argument("--joints", type=float, nargs=6, required=True, metavar="DEG",
                   help="Target servo angles 0-5 in degrees.")
    p.add_argument("--execute", action="store_true")
    p.add_argument("--max-joint-step-deg", type=float, default=3.0)
    p.add_argument("--tolerance-deg", type=float, default=0.5)

    sw = sub.add_parser("sweep", help="Step one joint and time it: latency, peak rate, settling, offset.")
    sw.add_argument("--joint", type=int, required=True, choices=range(6))
    sw.add_argument("--amplitude", type=float, default=15.0, help="Degrees, signed.")
    sw.add_argument("--hold-s", type=float, default=3.0)
    sw.add_argument("--execute", action="store_true")
    sw.add_argument("--out", help="Write the full result as JSON.")
    sw.add_argument("--mode", type=int, default=0, choices=(0, 1),
                    help="funcode 2 only: 1 = large smoothing (trajectory), 0 = small smoothing (10 Hz stream).")
    sw.add_argument("--funcode", type=int, default=2, choices=(1, 2),
                    help="2 = all-joint waypoint (what the Cartesian loop uses); 1 = single joint.")

    h = sub.add_parser("hold", help="Measure how still the arm holds, streaming vs silent.")
    h.add_argument("--seconds", type=float, default=15.0)
    h.add_argument("--stream", action="store_true",
                   help="Keep re-sending the current pose instead of going silent.")
    h.add_argument("--execute", action="store_true")
    h.add_argument("--resolve", type=float, nargs=3, metavar=("X", "Y", "Z"),
                   help="Re-solve IK to this target every cycle, as a Cartesian loop does.")
    h.add_argument("--out")

    r = sub.add_parser("release", help="Release the motors (funcode 5 mode 0). DROPS AN EXTENDED ARM.")
    r.add_argument("--execute", action="store_true")
    r.add_argument("--force", action="store_true",
                   help="Release even when the arm is not folded. It will fall.")

    args = ap.parse_args()

    if args.command == "ik" and args.from_zero:
        plan = d1_ik.solve(_JOINTS, args.target, q0=np.zeros(6), base_height=args.base_height)
        reached, _ = d1_ik.tool_pose(_JOINTS, plan.q)
        print(f"target       {_fmt(args.target, 4)} m")
        print(f"converged    {plan.converged} in {plan.iterations} iterations")
        print(f"position err {plan.position_error_m*1000:.2f} mm")
        print(f"reached      {_fmt(reached, 4)} m")
        print(f"joints       {_fmt(plan.servo_deg, 2)} deg (servo 0-5)")
        if args.base_height is not None:
            print(f"clear of body {plan.clear_of_body}")
        return 0 if plan.converged else 1

    client = D1Client(iface=args.iface)
    angles = client.wait_for_feedback()
    print(f"connected. servos {_fmt(angles, 2)} deg  "
          f"power={client.is_powered()} enable={client.is_enabled()} error={client.error_status()}")

    if args.command == "watch":
        end = time.monotonic() + args.seconds
        last = None
        while time.monotonic() < end:
            client.poll(timeout_s=0.3)
            angles = client.get_joint_angles()
            if angles != last:
                pos, _ = d1_ik.tool_pose(_JOINTS, d1_ik.from_servo_deg(angles)[0])
                print(f"  joints {_fmt(angles, 1)} deg   tool {_fmt(pos, 4)} m   "
                      f"grip {client.get_gripper_units():.1f}")
                last = angles
        return 0

    if args.command == "fk":
        q = d1_ik.from_servo_deg(client.get_joint_angles())[0]
        pos, rot = d1_ik.tool_pose(_JOINTS, q)
        print(f"joints (deg) {_fmt(client.get_joint_angles(), 2)}")
        print(f"joints (rad) {_fmt(q, 4)}")
        print(f"tool point   {_fmt(pos, 4)} m  (Go2 base frame, {d1_ik.TOOL_BODY} tip)")
        print("tool rotation")
        for row in rot:
            print(f"   {_fmt(row, 4)}")
        lows, highs = d1_ik.joint_limits(_JOINTS)
        out = [i for i, a in enumerate(q) if a < lows[i] or a > highs[i]]
        if out:
            print(f"NOTE joints {out} are outside the URDF soft limits (see F-023)")
        return 0

    if args.command == "park":
        mover = CartesianMover(client, _JOINTS, max_joint_step_deg=args.max_joint_step_deg)
        print(f"joints now   {_fmt(client.get_joint_angles(), 2)} deg")
        print(f"park target  {_fmt(args.joints, 2)} deg")
        if not args.execute:
            print("dry run (no DDS writes):")
        steps = mover.approach_joints(args.joints, execute=args.execute,
                                      tolerance_deg=args.tolerance_deg,
                                      on_step=lambda s: print(
                                          f"  {s.index:3d} cmd {_fmt(s.commanded_deg, 1)} "
                                          f"worst {s.target_error_m:5.2f} deg"))
        if steps:
            print(f"\n{len(steps)} steps, worst joint error {steps[-1].target_error_m:.2f} deg "
                  f"({mover.last_stop_reason})")
        if args.execute:
            print("arm left holding position (not released). "
                  "`release` drops torque and an extended arm will fall.")
        return 0

    if args.command == "sweep":
        lows, highs = d1_ik.servo_limits_deg(_JOINTS, soft=1.0)
        here = client.get_joint_angles()
        target = here[args.joint] + args.amplitude
        print(f"joints now  {_fmt(here, 2)} deg")
        print(f"sweep J{args.joint}: {here[args.joint]:+.1f} -> {target:+.1f} deg "
              f"(limits {lows[args.joint]:+.1f} .. {highs[args.joint]:+.1f})")
        if not (lows[args.joint] <= target <= highs[args.joint]):
            print("target outside the joint's hard limits; refusing.")
            return 1
        # What the model predicts this does to the tool, for a watcher to check.
        q = d1_ik.from_servo_deg(here)[0]
        frames, axes, origins = __import__("position_only.workspace", fromlist=["forward"]).forward(
            _JOINTS, q[None, :])
        from position_only.workspace import tool_position
        jac = d1_ik.jacobian(axes, origins, tool_position(frames))
        col = jac[0, :, args.joint] * (args.amplitude * d1_ik.DEG2RAD) * d1_ik.SERVO_SIGN[args.joint]
        print(f"model predicts tool moves {_fmt(col[:3]*1000, 1)} mm "
              f"and rotates about {_fmt(col[3:], 3)} (base frame: +x fwd, +y left, +z up)")
        if not args.execute:
            print("dry run. Pass --execute.")
            return 0
        result = sweep_joint(client, args.joint, args.amplitude, args.hold_s,
                             on_event=lambda m: print(m), funcode=args.funcode, mode=args.mode)
        print(f"  command path: funcode {args.funcode}"
              + (f" mode {args.mode}" if args.funcode == 2 else ""))
        for leg in ("out", "back"):
            r = result[leg]
            lat = "n/a" if r.get("latency_s") is None else f"{1e3*r['latency_s']:.0f} ms"
            st = "not reached" if r.get("settle_s") is None else f"{1e3*r['settle_s']:.0f} ms"
            print(f"  {leg:<4} latency {lat:>8}  peak {r['peak_rate_deg_s']:6.1f} deg/s "
                  f"({r['peak_rate_rad_s']:.3f} rad/s)  settle {st:>12}  "
                  f"steady-state err {r['steady_state_error_deg']:+.2f} deg  "
                  f"hold band {r['hold_band_deg']:.2f} deg")
        if args.out:
            with open(args.out, "w") as fh:
                json.dump({k: v for k, v in result.items() if k != "samples"} |
                          {"samples": result["samples"]}, fh)
            print(f"  wrote {args.out}")
        print("arm left holding position (not released).")
        return 0

    if args.command == "hold":
        if args.stream and not args.execute:
            print("dry run: --stream publishes commands. Pass --execute.")
            return 0
        if args.resolve and not args.execute:
            print("dry run: --resolve publishes commands. Pass --execute.")
            return 0
        r = measure_hold(client, _JOINTS, args.seconds, stream=args.stream,
                         resolve_target=args.resolve)
        how = {"resolve": "re-solving IK every cycle", "stream": "streaming a fixed pose",
               "silent": "silent (no commands)"}[r["regime"]]
        print(f"hold for {args.seconds:.0f}s, {how}: {r['samples']} samples")
        print(f"  held pose      {_fmt(r['held_command_deg'], 2)} deg")
        print(f"  joint stdev    {_fmt(r['joint_stdev_deg'], 3)} deg")
        print(f"  joint p-p      {_fmt(r['joint_ptp_deg'], 2)} deg")
        print(f"  tool p-p       {_fmt(r['tool_ptp_mm'], 3)} mm (x, y, z)")
        print(f"  tool max excursion from mean {r['tool_max_excursion_mm']:.3f} mm")
        if args.out:
            json.dump(r, open(args.out, "w"), indent=2)
            print(f"  wrote {args.out}")
        return 0

    if args.command == "release":
        safe = client.is_safe_to_release()
        if not safe:
            print(f"WARNING: arm is not folded (J1={angles[1]:.1f} J2={angles[2]:.1f}). "
                  "Releasing removes torque and it will fall to its stop.")
        if not args.execute:
            print("dry run: would publish funcode 5 {\"mode\": 0}. Pass --execute.")
            return 0
        if not safe and not args.force:
            print("refusing: pass --force to release an extended arm deliberately.")
            return 1
        print("sent:", client.release())
        time.sleep(0.5)
        client.poll(timeout_s=0.5)
        print(f"enable={client.is_enabled()} power={client.is_powered()}")
        return 0

    if args.command == "move":
        mover = CartesianMover(client, _JOINTS,
                               max_joint_step_deg=args.max_joint_step_deg,
                               base_height_m=(args.base_height or None),
                               level=args.level)
        start_pos, _ = mover.tool_now()
        plan = mover.plan(args.target)
        print(f"tool now     {_fmt(start_pos, 4)} m")
        print(f"target       {_fmt(args.target, 4)} m")
        print(f"plan         converged={plan.converged} err={plan.position_error_m*1000:.2f} mm "
              f"iters={plan.iterations} clear={plan.clear_of_body}")
        print(f"attitude     elevation {plan.elevation_deg:+.2f} deg, roll {plan.roll_deg:+.2f} deg"
              + ("  (level requested)" if args.level else ""))
        if args.level and max(abs(plan.elevation_deg), abs(plan.roll_deg)) > 1.0:
            print("             NOT level: that point is reachable but not with the gripper level.")
        print(f"goal joints  {_fmt(plan.servo_deg, 2)} deg")
        print(f"joint delta  {_fmt(np.asarray(plan.servo_deg) - np.asarray(client.get_joint_angles()), 2)} deg")
        if not plan.converged:
            print("target is not reachable from here; refusing to move.")
            return 1
        if not args.execute:
            print("\ndry run (no DDS writes). Rehearsing the approach:")
        if args.oneshot:
            steps = mover.run_oneshot(args.target, execute=args.execute, passes=args.passes,
                                      max_excursion_deg=args.max_excursion_deg,
                                      on_event=lambda m: print(m))
            final = steps[-1] if steps else None
            if final:
                print(f"\n{len(steps)} pass(es), final error {final.target_error_m*1000:.2f} mm "
                      f"({mover.last_stop_reason})")
                prof = analyse_motion(mover.last_cycles, args.max_excursion_deg)
                if prof.get("cycles", 0) >= 3:
                    print(f"  motion: {prof['displacement_per_cycle_deg']:.1f} deg/cycle at "
                          f"{prof['speed_deg_s']:.0f} deg/s, duty {prof['duty']:.2f} -> "
                          + ("CONTINUOUS" if prof["continuous"] else "not saturated"))
            if args.execute:
                print("arm left holding position (not released).")
            return 0
        steps = mover.run(args.target, execute=args.execute, max_cycles=args.max_cycles,
                          tolerance_m=args.tolerance_m,
                          on_step=lambda s: print(
                              f"  {s.index:3d} cmd {_fmt(s.commanded_deg, 1)} "
                              f"tool {_fmt(s.tool_pos_m, 4)} err {s.target_error_m*1000:7.2f} mm"))
        final = steps[-1] if steps else None
        if final:
            print(f"\n{len(steps)} steps, final error {final.target_error_m*1000:.2f} mm "
                  f"({mover.last_stop_reason})")
            prof = analyse_motion(mover.last_cycles, args.max_joint_step_deg)
            if prof.get("cycles", 0) >= 3:
                print(f"  motion: {prof['displacement_per_cycle_deg']:.1f} deg/cycle at "
                      f"{prof['speed_deg_s']:.0f} deg/s, period {1e3*prof['period_s']:.0f} ms; "
                      f"speed-limited displacement would be {prof['saturated_displacement_deg']:.1f} deg")
                print(f"  duty {prof['duty']:.2f} -> "
                      + ("CONTINUOUS (never decelerates between waypoints)" if prof["continuous"]
                         else "STEPPING (arrives early and idles; raise --max-joint-step-deg)"))
            if mover.last_stop_reason == "stalled":
                print("  stalled: the arm stopped closing the gap. Check the command "
                      "path first (F-031): funcode 2 mode 1 slews at ~14 deg/s and will "
                      "stall any loop running at the feedback rate. On mode 0 a stall is "
                      "the arm's small-increment floor, which holding does not close.")
        if args.execute and args.settle_s > 0:
            hold = steps[-1].commanded_deg if (args.hold_stream and steps) else None
            settled, measured, history = mover.settle(args.target, args.settle_s,
                                                      hold_command=hold)
            how = "streaming the final command" if hold else "with no further commands"
            print(f"\nsettling for {args.settle_s:.1f}s {how}:")
            for t, err in history:
                print(f"   t={t:5.2f}s  err {err*1000:7.2f} mm")
            loop_err = final.target_error_m * 1000 if final else float("nan")
            print(f"settled error {settled*1000:.2f} mm "
                  f"(loop ended at {loop_err:.2f} mm)")
            print(f"settled joints {_fmt(measured, 2)} deg")
        if args.execute:
            print("arm left holding position (not released). "
                  "`release` drops torque and an extended arm will fall.")
        return 0

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
