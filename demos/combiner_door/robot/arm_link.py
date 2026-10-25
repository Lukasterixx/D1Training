#!/usr/bin/env python3
"""The D1 arm for the combiner door's robot program (`node.py`): a socket on 127.0.0.1 in front of
`d1_hardware.D1Client`, this repo's measured client for the arm's own DDS protocol (F-020 to F-063).

It runs in a container from P2Dingo's arm image (`run_on_dog.sh`), which has CycloneDDS's Python binding; the ROS
image the node runs in does not, and a second DDS stack inside a ROS process is asking for trouble. One client at a
time, newline-delimited JSON:

    -> {"type": "target", "q": [6 servo deg], "gripper": units | null, "speed_deg_s": 30.0}
    -> {"type": "path", "q": [[6 servo deg], ...], "dt": 0.1, "gripper": units | null, "lead_max_deg": 15.0}
                                   a timed path (`PathPlayer`); `lead_max_deg` optional, at most 20
    -> {"type": "follow", "q": [6 servo deg], "gripper": units | null}
                                   a goal that moves every command, followed as a path plays (`Follower`)
    -> {"type": "keepalive", "gripper": units | null}
                                   still here, keep going (a gripper given: the jaws there from now on)
    -> {"type": "hold"}            stop: command the arm to where it is, and stream nothing more
    <- {"type": "hello", "observe": bool, "bridge": {...} | null, "q": [...]}
    <- {"type": "feedback", "q": [6], "gripper": units, "age_s": s, "streaming": bool}   at the arm's ~9 Hz
    <- {"type": "error", "detail": "..."}

What it will and will not do:
- **A smooth command** (`Profile`): streamed at a steady 10 Hz (the arm's own command rate, `D1Client._pace`), moving
  along the line in joint space to the target with its longest joint accelerating at `ACCEL_DEG_S2` to the asked
  `speed_deg_s` (at most `MAX_SPEED_DEG_S`) and slowing to arrive, **never more than `LEAD_DEG` from where the arm last
  reported itself**. Angles are clamped to the URDF's hard limits.
- **A timed path** (`PathPlayer`, since 2026-10-23, the holster draw's): joint points `dt` apart in time, played out
  by the clock, one command a point (10 Hz at `dt` 0.1, 5 Hz at 0.2: the arm pauses less at lower rates, F-202), each command the path's own point -- no slowing to stop on each one, as a stream
  of near targets gets from `Profile` (on the dog, 58% of its steps came out under 2.5 deg, which the firmware stops
  after, F-045). Its lead on the arm's report may grow by how far the path moves in the report's age (to
  `PATH_LEAD_MAX_DEG`): the reports come in bursts, with gaps of 0.4-0.6 s, and a 10 deg lead on a stale report
  stopped the arm for up to 0.3 s at a time. Past that lead the path's clock waits for the arm. It must start within
  `LEAD_DEG` of the arm and move no joint faster than `MAX_SPEED_DEG_S`; at its end the arm holds its last point.
- **A followed goal** (`Follower`, since 2026-10-24): the robot program's goal, sent each step (its IK at 10 Hz along
  the script's line), commanded as a timed path's points are -- each command the goal itself, at most
  `MAX_SPEED_DEG_S` from the last, with a path's lead on the report -- not a `Profile` that slows to stop on each one.
  The program sends its joint moves as timed paths (`plan.ArmSender`), the holster draw's way.
- **Holds** -- commands the arm to its own measured pose, the gripper to its last command, once, and stops streaming --
  when the node says so, goes quiet for `WATCHDOG_S`, or disconnects, and when this process is stopped. The arm then
  stays where it is, powered, and a grip on the lever stays shut.
- **Never releases the arm.** Release is the arm's only software stop and it drops an extended arm (F-028);
  `D1Client.close` releases, so it is never called here.
- **Refuses to start** while P2Dingo's arm bridge (`p2dingo-arm`, 127.0.0.1:8084) is streaming to the arm (connected,
  mode MANUAL or BUSY): two writers on one arm fight. E-STOP in the website's arm panel stops the bridge's stream.
- `--observe` (a dry run's): feedback only; it takes no target.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import socket
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

import d1_hardware  # noqa: E402

PORT = 8095
WATCHDOG_S = 0.5
#: The command's speed (its longest joint) and how fast it gets there and stops (`Profile`). The arm slews at ~70 deg/s
#: (F-033). History (2026-10-20): stepped 4 deg from each report it crept at ~8 deg/s; stepped 4 deg a report from the
#: last command it moved in jerks, each step at the arm's top speed and then a wait, with stalls whenever a report was
#: late (they come every 111 ms and sometimes not for 0.3-0.4 s).
DEFAULT_SPEED_DEG_S = 30.0
MAX_SPEED_DEG_S = 45.0
ACCEL_DEG_S2 = 60.0
COMMAND_PERIOD_S = 0.1
#: How far the command may run ahead of where the arm last reported itself, along any joint: an arm held back
#: (blocked, or loaded on the lever) is never driven hard.
LEAD_DEG = 10.0
#: A timed path's lead on the arm's report: `LEAD_DEG`, plus how far the path moves in the report's age, at most this
#: unless the path asks for more (`lead_max_deg`), never more than `PATH_LEAD_LIMIT_DEG`.
PATH_LEAD_MAX_DEG = 15.0
PATH_LEAD_LIMIT_DEG = 20.0
#: The arm's ~9 Hz report gone this long: hold.
STALE_FEEDBACK_S = 1.0
BRIDGE = ("127.0.0.1", 8084)


def log(msg: str) -> None:
    print(f"[arm_link {time.strftime('%H:%M:%S')}] {msg}", flush=True)


def bridge_status(timeout_s: float = 1.5) -> dict | None:
    """P2Dingo's arm bridge's status line, or None if it is not listening."""
    try:
        sock = socket.create_connection(BRIDGE, timeout=timeout_s)
    except OSError:
        return None
    sock.settimeout(timeout_s)
    buffer, deadline = b"", time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            chunk = sock.recv(4096)
            if not chunk:
                break
            buffer += chunk
            for line in buffer.split(b"\n")[:-1]:
                try:
                    msg = json.loads(line)
                except ValueError:
                    continue
                if msg.get("type") == "status":
                    return msg
    except OSError:
        pass
    finally:
        sock.close()
    return {"type": "status", "unknown": True}


class Profile:
    """A command that moves smoothly towards a target that may itself move: along the line in joint space to it, its
    longest joint accelerating at `accel` up to `speed` and slowing so as to stop on it, at whatever interval it is
    stepped; never more than `lead` from the arm's report (where it is held back, it waits for the arm, from rest).
    Servo degrees. numpy only."""

    def __init__(self, start, accel: float = ACCEL_DEG_S2, lead: float = LEAD_DEG):
        self.q = np.asarray(start, dtype=float).copy()
        self.v = 0.0
        self.accel, self.lead = float(accel), float(lead)

    def step(self, target, measured, dt: float, speed: float) -> list[float]:
        delta = np.asarray(target, dtype=float) - self.q
        dist = float(np.max(np.abs(delta)))
        if dist < 1e-6:
            self.v = 0.0
        else:
            # The fastest from which steps of this length, each `accel * dt` slower, still stop on the target.
            step_v = self.accel * dt
            stop_v = step_v * (math.sqrt(0.25 + 2.0 * dist / (step_v * dt)) - 0.5) if step_v > 0 else float(speed)
            want = min(float(speed), stop_v)
            self.v = min(want, self.v + self.accel * dt) if self.v < want else max(want, self.v - self.accel * dt)
            self.q = self.q + delta / dist * min(dist, self.v * dt)
        m = np.asarray(measured, dtype=float)
        held = np.clip(self.q, m - self.lead, m + self.lead)
        if np.any(np.abs(held - self.q) > 1e-9):
            self.q, self.v = held, 0.0
        return [float(v) for v in self.q]


class PathPlayer:
    """A timed joint path (`points`, `dt` s apart, servo degrees) played out by the clock: each step, the point the
    path has reached, unless that is further from the arm's report than `LEAD_DEG` plus the path's speed times the
    report's age and one step (at most `lead_max`, `PATH_LEAD_MAX_DEG` by default) -- then the path's clock waits, the
    command where it was. numpy only."""

    def __init__(self, points, dt: float, lead: float = LEAD_DEG, lead_max: float = PATH_LEAD_MAX_DEG):
        self.points = np.asarray(points, dtype=float)
        self.dt = float(dt)
        self.times = np.arange(len(self.points)) * self.dt
        steps = np.abs(np.diff(self.points, axis=0)).max(axis=1) if len(self.points) > 1 else np.zeros(1)
        self.speed = float(steps.max()) / self.dt
        self.lead, self.lead_max = float(lead), float(lead_max)
        self.t = 0.0
        self.command = self.points[0].copy()
        self.waited = 0

    @property
    def duration(self) -> float:
        return float(self.times[-1])

    @property
    def done(self) -> bool:
        return self.t >= self.duration - 1e-9

    def at(self, t: float) -> np.ndarray:
        return np.array([np.interp(t, self.times, self.points[:, j]) for j in range(self.points.shape[1])])

    def step(self, measured, age_s: float, dt: float) -> list[float]:
        # The next point is one step ahead of the arm by design, and the report is `age_s` old: both allowed for, on
        # top of `lead`. (Without the step, at 5 Hz an 8.6 deg step plus the arm's lag passed 10 deg and the path waited
        # every other command.)
        allow = min(self.lead + self.speed * (max(0.0, float(age_s)) + float(dt)), self.lead_max)
        t_next = min(self.t + float(dt), self.duration)
        want = self.at(t_next)
        if float(np.max(np.abs(want - np.asarray(measured, dtype=float)))) <= allow:
            self.t, self.command = t_next, want
        else:
            self.waited += 1
        return [float(v) for v in self.command]


class Follower:
    """A goal that moves each command (the program's IK along its script, 10 Hz), played as a timed path's points
    are (`PathPlayer`): each command the goal itself, or at most `MAX_SPEED_DEG_S` along the line towards it, never
    slowing to stop on it as `Profile` does (on the dog that left 58% of the steps under 2.5 deg, which the firmware
    stops after, F-045); held where it was while it would be more than `lead` plus its speed times the report's age
    and one step (at most `lead_max`) from the arm's report. The goals' own pacing is the program's (smooth scripts,
    joint moves sent as paths). numpy only."""

    def __init__(self, start, lead: float = LEAD_DEG, lead_max: float = PATH_LEAD_MAX_DEG):
        self.command = np.asarray(start, dtype=float).copy()
        self.goal = self.command.copy()
        self.lead, self.lead_max = float(lead), float(lead_max)
        self.waited = 0

    def step(self, measured, age_s: float, dt: float) -> list[float]:
        delta = self.goal - self.command
        dist = float(np.max(np.abs(delta)))
        reach = MAX_SPEED_DEG_S * float(dt)
        want = self.command + (delta if dist <= reach else delta * (reach / dist))
        speed = min(dist, reach) / max(float(dt), 1e-6)
        allow = min(self.lead + speed * (max(0.0, float(age_s)) + float(dt)), self.lead_max)
        if float(np.max(np.abs(want - np.asarray(measured, dtype=float)))) <= allow:
            self.command = want
        else:
            self.waited += 1
        return [float(v) for v in self.command]


def check_path(points, dt, measured) -> str | None:
    """Why a timed path may not be played (`Link.handle`), or None."""
    try:
        q = np.asarray(points, dtype=float)
        dt = float(dt)
    except (TypeError, ValueError):
        return "a path is a list of six-angle points and a dt"
    if q.ndim != 2 or q.shape[1] != 6 or len(q) < 2 or not np.all(np.isfinite(q)):
        return "a path is two or more points of six finite servo angles"
    if not 0.05 <= dt <= 0.5:
        return "a path's dt is 0.05-0.5 s"
    if float(np.abs(np.diff(q, axis=0)).max()) / dt > MAX_SPEED_DEG_S + 1e-6:
        return f"a path moves a joint faster than {MAX_SPEED_DEG_S} deg/s"
    if float(np.max(np.abs(q[0] - np.asarray(measured, dtype=float)))) > LEAD_DEG:
        return f"a path must start within {LEAD_DEG} deg of the arm"
    return None


class SimulatedArm:
    """A dry run's arm (the node's, and the tests'): this link's `Profile` driving a servo that follows its command as
    mode 0 does (F-033: ~70 deg/s at most, settling in about half a second), and a gripper that moves at
    `gripper_rate` units/s and stops closing at `stall_units` (the lever between the pads), when given. The gripper's
    rate is assumed, not measured. Stepped once per 10 Hz command."""

    SERVO_TAU_S = 0.12
    SERVO_RATE_DEG_S = 70.0

    def __init__(self, start, gripper: float = 40.8, gripper_rate: float = 70.0):
        self.q = np.asarray(start, dtype=float).copy()
        self.profile = Profile(self.q)
        self.gripper, self.gripper_rate = float(gripper), float(gripper_rate)
        self.stall_units: float | None = None

    def step(self, target, gripper=None, speed: float = DEFAULT_SPEED_DEG_S, dt: float = COMMAND_PERIOD_S) -> None:
        self._servo(np.asarray(self.profile.step(target, self.q, dt, speed)), gripper, dt)

    def step_command(self, command, gripper=None, dt: float = COMMAND_PERIOD_S) -> None:
        """The servo following a command as it is (a timed path's, `PathPlayer`), and the gripper; a later `step`
        starts its profile there, at rest, as the link's does at a path's end."""
        command = np.asarray(command, dtype=float)
        self.profile = Profile(command)
        self._servo(command, gripper, dt)

    def _servo(self, command, gripper, dt: float) -> None:
        move = (command - self.q) * (1.0 - math.exp(-dt / self.SERVO_TAU_S))
        self.q = self.q + np.clip(move, -self.SERVO_RATE_DEG_S * dt, self.SERVO_RATE_DEG_S * dt)
        if gripper is not None:
            goal = float(gripper)
            if self.stall_units is not None and self.gripper >= self.stall_units - 1e-9:
                goal = max(goal, self.stall_units)
            self.gripper += float(np.clip(goal - self.gripper, -self.gripper_rate * dt, self.gripper_rate * dt))


class SimulatedLink:
    """A dry run's link (the node's `VirtualArm`, the tests'): the messages `Link.handle` takes -- a target, a timed
    path, a followed goal, a keepalive -- played one 10 Hz command a `handle`, as `Link.control` plays them, onto a
    `SimulatedArm`'s servo."""

    def __init__(self, start, gripper: float = 40.8, gripper_rate: float = 70.0):
        self.arm = SimulatedArm(start, gripper, gripper_rate)
        self.gripper = None
        self.player: PathPlayer | None = None
        self.follower: Follower | None = None
        self.target = None
        self.speed = DEFAULT_SPEED_DEG_S
        self.command = None
        self.paths = 0

    @property
    def q(self):
        return self.arm.q

    def handle(self, msg: dict, dt: float = COMMAND_PERIOD_S) -> None:
        kind = msg["type"]
        if msg.get("gripper") is not None or kind in ("target", "path", "follow"):
            self.gripper = msg.get("gripper", self.gripper)
        if kind == "path":
            why = check_path(msg["q"], msg["dt"], self.arm.q)
            if why:
                raise ValueError(why)
            self.player, self.follower, self.target = PathPlayer(msg["q"], msg["dt"],
                                                                 lead_max=msg.get("lead_max_deg", PATH_LEAD_MAX_DEG)), None, None
            self.paths += 1
        elif kind == "follow":
            if self.follower is None:
                self.follower = Follower(self.command if self.command is not None else self.arm.q)
            self.follower.goal = np.asarray(msg["q"], dtype=float)
            self.player, self.target = None, None
        elif kind == "target":
            if self.target is None:
                self.arm.profile = Profile(self.command if self.command is not None else self.arm.q)
            self.player, self.follower = None, None
            self.target, self.speed = list(msg["q"]), float(msg.get("speed_deg_s", DEFAULT_SPEED_DEG_S))
        self.tick(dt)

    def tick(self, dt: float = COMMAND_PERIOD_S) -> None:
        if self.player is not None:
            self.command = self.player.step(self.arm.q, 0.0, dt)
            if self.player.done:
                self.player, self.follower = None, Follower(self.command)
            self.arm.step_command(self.command, self.gripper, dt)
        elif self.follower is not None:
            self.command = self.follower.step(self.arm.q, 0.0, dt)
            self.arm.step_command(self.command, self.gripper, dt)
        elif self.target is not None:
            self.arm.step(self.target, self.gripper, self.speed, dt)
            self.command = [float(v) for v in self.arm.profile.q]
        elif self.gripper is not None:
            self.arm._servo(self.arm.q if self.command is None else np.asarray(self.command), self.gripper, dt)


class Link:
    def __init__(self, client, observe: bool, log_path: Path | None):
        self.client = client
        self.observe = observe
        self.lock = threading.Lock()
        self.target = None
        self.gripper = None
        self.speed = DEFAULT_SPEED_DEG_S
        self.profile: Profile | None = None
        self.player: PathPlayer | None = None
        self.follower: Follower | None = None
        self.streaming = False
        self.last_rx = 0.0
        self.conn = None
        self.last_gripper = None
        self.last_command = None
        self.log_file = open(log_path, "a") if log_path else None
        self.stopping = False

    def record(self, **row) -> None:
        if self.log_file is not None:
            self.log_file.write(json.dumps({"t": round(time.time(), 3), **row}) + "\n")
            self.log_file.flush()

    def hold(self, why: str) -> None:
        with self.lock:
            was = self.streaming
            self.streaming, self.target = False, None
            self.last_command = None
            self.profile = None
            self.player = None
            self.follower = None
        if not was or self.observe:
            return
        try:
            # The joints where they are; the gripper where it was last sent, not where it stalled: on the lever a
            # squeeze stalls short of its command, and re-commanding the stall would let go of it.
            sent = self.client.set_all_joint_angles(self.client.get_joint_angles(), mode=0,
                                                    gripper=self.last_gripper)
            log(f"hold ({why})")
            self.record(kind="hold", why=why, sent=sent)
        except Exception as exc:  # noqa: BLE001 -- nothing else to do but say so
            log(f"hold failed ({why}): {exc!r}")

    # --- the arm ------------------------------------------------------------------------------------------------

    def control(self) -> None:
        """Poll the arm; while streaming, send the next command every `COMMAND_PERIOD_S`, a steady 10 Hz -- or, playing a
        timed path, every point of it (its `dt`, never less than `COMMAND_PERIOD_S`)."""
        last_sent = None
        while not self.stopping:
            self.client.poll(timeout_s=0.02)
            with self.lock:
                target, gripper, speed, streaming, last_rx = (self.target, self.gripper, self.speed,
                                                              self.streaming, self.last_rx)
            if not streaming or target is None:
                last_sent = None
                time.sleep(0.01)
                continue
            if time.monotonic() - last_rx > WATCHDOG_S:
                self.hold("the node went quiet")
                continue
            age = self.client.feedback_age_s
            if age > STALE_FEEDBACK_S:
                self.hold(f"arm feedback {age:.2f} s old")
                continue
            now = time.monotonic()
            with self.lock:
                period = max(COMMAND_PERIOD_S, self.player.dt) if self.player is not None else COMMAND_PERIOD_S
            if last_sent is not None and now - last_sent < period:
                continue
            dt = period if last_sent is None else min(now - last_sent, 3 * period)
            last_sent = now
            measured = self.client.get_joint_angles()
            with self.lock:
                player, follower = self.player, self.follower
                if self.profile is None:
                    self.profile = Profile(measured)
                profile = self.profile
            if player is not None:
                command = player.step(measured, age, dt)
                if player.done:
                    with self.lock:
                        # The path's end held as an ordinary target, the profile at rest on it.
                        if self.player is player:
                            self.player, self.profile = None, Profile(command)
            elif follower is not None:
                command = follower.step(measured, age, dt)
            else:
                command = profile.step(target, measured, dt, speed)
            self.last_command = command
            sent = self.client.set_all_joint_angles(command, mode=0, gripper=gripper)
            if gripper is not None:
                self.last_gripper = gripper
            self.record(kind="command", measured=[round(v, 2) for v in measured],
                        command=[round(v, 2) for v in command], gripper=gripper, sent=sent)

    def report(self) -> None:
        """Feedback to the client at the arm's rate."""
        last = 0.0
        while not self.stopping:
            time.sleep(0.05)
            conn = self.conn
            if conn is None:
                continue
            try:
                q = self.client.get_joint_angles()
                gripper = self.client.get_gripper_units()
            except RuntimeError:
                continue
            age = self.client.feedback_age_s
            if time.monotonic() - age <= last:
                continue
            last = time.monotonic() - age
            with self.lock:
                streaming = self.streaming
            self.send(conn, {"type": "feedback", "q": [round(v, 3) for v in q], "gripper": round(gripper, 2),
                             "age_s": round(age, 3), "streaming": streaming})

    @staticmethod
    def send(conn, msg: dict) -> None:
        try:
            conn.sendall((json.dumps(msg) + "\n").encode())
        except OSError:
            pass

    # --- the client ---------------------------------------------------------------------------------------------

    def handle(self, conn, msg: dict) -> None:
        kind = msg.get("type")
        if kind == "keepalive":
            gripper = msg.get("gripper")
            with self.lock:
                self.last_rx = time.monotonic()
                if gripper is not None and isinstance(gripper, (int, float)) and math.isfinite(gripper) \
                        and self.streaming and not self.observe:
                    self.gripper = float(gripper)
            return
        if kind == "hold":
            self.hold("the node asked")
            return
        if kind not in ("target", "path", "follow"):
            self.send(conn, {"type": "error", "detail": f"unknown message type {kind!r}"})
            return
        if self.observe:
            self.send(conn, {"type": "error", "detail": "observing only: no targets taken"})
            return
        if kind == "path":
            gripper = msg.get("gripper")
            try:
                measured = self.client.get_joint_angles()
            except RuntimeError:
                measured = None
            why = "no report from the arm" if measured is None else check_path(msg.get("q"), msg.get("dt"), measured)
            lead_max = msg.get("lead_max_deg", PATH_LEAD_MAX_DEG)
            try:
                lead_max = float(lead_max)
                if why is None and not LEAD_DEG <= lead_max <= PATH_LEAD_LIMIT_DEG:
                    why = f"a path's lead_max_deg is {LEAD_DEG}-{PATH_LEAD_LIMIT_DEG}"
            except (TypeError, ValueError):
                why = why or "a path's lead_max_deg is a number"
            if why is None and gripper is not None:
                try:
                    gripper = float(gripper)
                    why = None if math.isfinite(gripper) else "a finite gripper or null"
                except (TypeError, ValueError):
                    why = "a finite gripper or null"
            if why:
                self.send(conn, {"type": "error", "detail": why})
                self.hold("a bad path")
                return
            player = PathPlayer(msg["q"], float(msg["dt"]), lead_max=lead_max)
            with self.lock:
                self.player, self.target, self.gripper = player, [float(v) for v in player.points[-1]], gripper
                self.follower = None
                self.streaming, self.last_rx = True, time.monotonic()
            self.record(kind="path", points=len(player.points), dt=player.dt, speed_deg_s=round(player.speed, 1))
            return
        q = msg.get("q")
        gripper = msg.get("gripper")
        speed = msg.get("speed_deg_s", DEFAULT_SPEED_DEG_S) if kind == "target" else MAX_SPEED_DEG_S
        try:
            q = [float(v) for v in q]
            ok = len(q) == 6 and all(math.isfinite(v) for v in q)
            gripper = None if gripper is None else float(gripper)
            ok = ok and (gripper is None or math.isfinite(gripper))
            speed = float(speed)
            ok = ok and 0.0 < speed <= MAX_SPEED_DEG_S
        except (TypeError, ValueError):
            ok = False
        if not ok:
            self.send(conn, {"type": "error", "detail": "a target is six finite servo angles, a finite gripper or "
                                                        f"null, and 0 < speed_deg_s <= {MAX_SPEED_DEG_S}"})
            self.hold("a bad target")
            return
        if kind == "follow":
            try:
                measured = self.client.get_joint_angles()
            except RuntimeError:
                self.send(conn, {"type": "error", "detail": "no report from the arm"})
                self.hold("no report to follow from")
                return
            with self.lock:
                if self.follower is None:
                    # From the last command (a path's end, a profile's), else where the arm is.
                    self.follower = Follower(self.last_command or measured)
                    self.record(kind="follow")
                self.follower.goal = np.asarray(q, dtype=float)
                self.player, self.target, self.gripper = None, q, gripper
                self.streaming, self.last_rx = True, time.monotonic()
            return
        with self.lock:
            self.target, self.gripper, self.speed = q, gripper, speed
            self.streaming, self.last_rx = True, time.monotonic()
            followed, self.follower = self.follower is not None, None
            if self.player is not None or followed:
                # A target replaces a path: the profile picks up from the path's last command.
                self.player, self.profile = None, Profile(self.last_command or self.client.get_joint_angles())

    def serve(self, port: int) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", port))
        server.listen(1)
        log(f"listening on 127.0.0.1:{port} ({'observing only' if self.observe else 'taking targets'})")
        while not self.stopping:
            conn, _ = server.accept()
            if self.conn is not None:
                self.send(conn, {"type": "error", "detail": "one client at a time"})
                conn.close()
                continue
            self.conn = conn
            log("node connected")
            try:
                q = self.client.get_joint_angles()
            except RuntimeError:
                q = None
            self.send(conn, {"type": "hello", "observe": self.observe, "bridge": bridge_status(0.5), "q": q})
            buffer = b""
            try:
                while True:
                    chunk = conn.recv(65536)
                    if not chunk:
                        break
                    buffer += chunk
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        try:
                            msg = json.loads(line)
                        except ValueError:
                            self.send(conn, {"type": "error", "detail": "not JSON"})
                            continue
                        if isinstance(msg, dict):
                            self.handle(conn, msg)
            except OSError:
                pass
            finally:
                self.conn = None
                conn.close()
                self.hold("the node disconnected")
                log("node disconnected")


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--port", type=int, default=PORT)
    p.add_argument("--iface", default=d1_hardware.ARM_IFACE)
    p.add_argument("--observe", action="store_true", help="Feedback only; take no targets (a dry run's).")
    p.add_argument("--log", default=None, help="Append every command to this JSONL file.")
    args = p.parse_args(argv)

    status = bridge_status()
    streaming = bool(status) and status.get("connected") and status.get("mode") in ("MANUAL", "BUSY")
    if status is not None:
        log(f"P2Dingo's arm bridge: {json.dumps(status)}")
    if streaming and not args.observe:
        log("P2Dingo's arm bridge is streaming to the arm. Press E-STOP in the website's arm panel (it stops the "
            "stream and leaves the arm powered where it is) or `docker stop p2dingo-arm`, then start again.")
        return 3

    client = d1_hardware.D1Client(iface=args.iface)
    try:
        q = client.wait_for_feedback(timeout_s=5.0)
    except TimeoutError as exc:
        log(f"{exc}")
        return 4
    log(f"arm at {[round(v, 1) for v in q]}")
    link = Link(client, args.observe, Path(args.log) if args.log else None)

    def stop(signum, frame):
        link.stopping = True
        link.hold(f"signal {signum}")
        log("stopped; the arm holds where it is")
        sys.exit(0)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    threading.Thread(target=link.control, daemon=True).start()
    threading.Thread(target=link.report, daemon=True).start()
    link.serve(args.port)
    return 0


if __name__ == "__main__":
    sys.exit(main())
