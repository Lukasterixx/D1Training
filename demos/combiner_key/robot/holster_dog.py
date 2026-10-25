#!/usr/bin/env python3
"""The holster draw's arm motion on the real D1, with no holster and no key (Lukas, 2026-10-23: "I just want to see it
go through the motions in real life"). Start it on the dog with `run_holster_on_dog.sh` (`d1dogholster` on the PC).

    run_holster_on_dog.sh                 # DRY: the plan, then the motion on a simulated arm; nothing moves
    run_holster_on_dog.sh --live          # the real arm (one Enter to start; `a` + Enter holds it where it is)
    run_holster_on_dog.sh --live --step   # an Enter before every move

Live, the Go2 remote's A is Enter and its B is `a` + Enter (read from `/lf/lowstate`, as in every live program here:
`lowlevel.RemoteButtons`). After a stop, `r` + Enter or A takes the arm back to where it started (`recovery`): stopped
on the way out, it plays what it has played so far backwards; from `enter` on, it finishes the plan's way back. Either
way it only goes where the arm has been or the plan goes. `q` + Enter or Ctrl-C quits with the arm held.

The legs are not touched: sit the dog (or stand it) with the remote first. The draw is `holster.py`'s, in the base's
own frame, so the arm's motion is the same sitting or standing. The moves, as the simulation's default
(`holster_env.holster_phases`, F-189, F-190):

| move | what | how |
| --- | --- | --- |
| `turn` | Joint1 turned to the draw's side, the arm still folded, its elbow opened `holster.TURN_ELBOW_RAD` | joint space, `JOINT_SPEED_DEG_S` |
| `reach`, `lower` | unfolded to the hand 80 mm above where it waits, then down to it: 65 mm out along the holster's axis past the key's root, jaws open | joint space |
| `on` | slid along the axis onto where the key's root would be | streamed, paced at `JOINT_SPEED_RAD_S` |
| `close`, `draw`, `exit`, `hold` | jaws shut to the key's bar; drawn `holster.SLIDE_M` out along the axis, then `holster.EXIT_M` out through v5's mouth at `holster.EXIT_DEG`; held | jaws, then streamed |
| `enter`, `back`, `open`, `off` | the exit and the draw undone (the key back into the mouth and down the channel); jaws open; off the bar | streamed, jaws, streamed |
| `raise`, `unreach`, `fold` | folded back the way it came | joint space |

Through `arm_link.py` (`demos/combiner_door/robot/`): every move goes to the link as one timed path (`move_path`; a
joint-space move as a straight joint line, accelerating, cruising at `JOINT_SPEED_DEG_S` and braking), which the link
plays out by its clock at 10 Hz (`arm_link.PathPlayer`), within a lead on the arm's report that allows for the report's
age; it holds the arm on `hold`, on silence or on disconnect, and never releases it. (The first version, run live on
2026-10-23, sent a near target every 100 ms instead: the link's profile slowed to stop on each, 58% of its commands came
out under 2.5 degrees, which the firmware stops after, and on the reports' 0.4-0.6 s gaps its 10-degree lead stopped
the arm for up to 0.3 s: "it moves in microsteps, with lots of mini pauses of varying length".)

Before it moves, it refuses unless the link answers, the arm's report is fresh, and the arm is folded (each joint
within `FOLDED_DEG` of the commandable fold). Each joint-space move ends when the arm is within `ARRIVED_DEG` on every
joint, and a move the arm does not finish in `MOVE_TIMEOUT_S` holds the arm and stops. On the CPU arm model the moving
links keep 37 mm or more from the Go2's dock, rails and lidar mount, but a finger passes **8 mm** from the D1's own base
during `on` and `draw` (where the holster's key would be). That margin is the model's; the real arm's joint zeros are
not validated (F-023). Watch that corner.

The record is `logs/holster_dog/<stamp>_{dry,live}/`: `run.json` and `trace.csv` (each 100 ms: the move, the command
and the arm's report, servo degrees).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import queue
import socket
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

import d1_hardware  # noqa: E402
import d1_ik  # noqa: E402
from demos.combiner_key import holster  # noqa: E402

ARM_LINK_PORT = 8095
LOG_ROOT = ROOT / "logs/holster_dog"
#: The command period: 0.1 s (10 Hz) by default; `--hz 5` makes it 0.2 s, each command twice the step (F-202: the
#: arm paused its reports and motion 10-15 times in 20 s at 10 Hz, 3 at 5 Hz). The link sends one command a point.
PERIOD_S = 0.1
#: The joint-space moves: every joint at this pace while it moves, getting there and stopping this fast
#: (`holster.joint_move`).
JOINT_SPEED_DEG_S = math.degrees(holster.JOINT_MOVE_RAD_S)
ACCEL_DEG_S2 = math.degrees(holster.JOINT_MOVE_ACCEL_RAD_S2)
#: The streamed moves' pace on the fastest joint: 4.3 degrees a command, under the link's 45 deg/s. At 0.6 rad/s (the
#: simulation's default) Joint1 and Joint4 stepped ~2.4 degrees a command on the dog, under the ~2.5 the firmware runs
#: through without stopping (F-045); the shoulder and elbow step ~0.5 degrees whatever the pace.
JOINT_SPEED_RAD_S = 0.75
#: How far ahead of the arm's last report the link may play a path when the report is late (`arm_link.PathPlayer`):
#: more in free space, the link's default where the key would be in the holster.
LEAD_FREE_DEG = 20.0
LEAD_KEY_DEG = 15.0
#: The jaws: open round the key's bar, and shut to it (`holster.JAW_OPEN_M`, `JAW_HOLD_M`), servo 6 units through
#: `d1_hardware`'s map (unmeasured between its ends, F-063); how long each takes.
GRIPPER_OPEN_UNITS = d1_hardware.finger_travel_to_gripper_units(holster.JAW_OPEN_M)
GRIPPER_HOLD_UNITS = d1_hardware.finger_travel_to_gripper_units(holster.JAW_HOLD_M)
JAW_S = 1.5             # fully open to shut on the bar is ~65 units (the gripper's speed is not measured)
HOLD_S = 2.0
#: The commandable fold: the rest pose with Joint2 and Joint3 at the limits the arm takes commands to (as the door
#: program's `plan.FOLD_SERVO_DEG`).
FOLD_SERVO_DEG = tuple(float(np.clip(v, -90.0, 90.0)) for v in d1_hardware.REST_SERVO_DEG)
#: Starting: each joint within this of the fold. The wrist's three rest wherever they were left (on the dog, 2026-10-23:
#: Joint5 at 17 degrees against the rest pose's 4), and Joint1 where a stopped run left it (16 degrees); `turn` sends
#: them all to its goal. Joint2-3 decide whether the arm is folded at all.
FOLDED_DEG = (30.0, 10.0, 10.0, 25.0, 25.0, 25.0)
#: A joint-space move is done this close on every joint; a streamed one ends when its path does and the arm is this
#: close to its last point, or `SETTLE_S` after.
ARRIVED_DEG = 2.0
SETTLE_S = 1.5
MOVE_TIMEOUT_S = 20.0
#: The arm's report older than this: hold and stop.
STALE_S = 1.0


def log(msg: str) -> None:
    print(f"[holster_dog {time.strftime('%H:%M:%S')}] {msg}", flush=True)


# --- the plan ------------------------------------------------------------------------------------------------------

def servo(q_rad) -> np.ndarray:
    return np.asarray(d1_ik.to_servo_deg(np.asarray(q_rad, dtype=float)), dtype=float)


def plan(joint_speed: float = JOINT_SPEED_RAD_S) -> list[dict]:
    """The moves, in servo degrees: joint goals (`goal`) or streamed paths (`path`: (seconds, samples x 6) at even
    times), and the jaws (`gripper`) through each."""
    paced = holster.joint_paced(joint_speed)
    on_s, on_u, _, on_q = paced["on"]
    draw_s, draw_u, _, draw_q = paced["draw"]
    exit_s, exit_u, _, exit_q = paced["exit"]

    def even(seconds, u, q):
        """The joint path resampled at `PERIOD_S` in time (u is time over the move's length)."""
        times = np.arange(0.0, seconds + 1e-9, PERIOD_S) / seconds
        if times[-1] < 1.0 - 1e-9:
            times = np.append(times, 1.0)            # a move shorter than a command still ends where it should
        return np.stack([servo([np.interp(t, u, q[:, j]) for j in range(6)]) for t in np.clip(times, 0.0, 1.0)])

    on_path, draw_path, exit_path = even(on_s, on_u, on_q), even(draw_s, draw_u, draw_q), even(exit_s, exit_u, exit_q)
    q_pre = on_path[0]
    turned_q = d1_ik.from_servo_deg(FOLD_SERVO_DEG)[0]
    turned_q[0] = d1_ik.from_servo_deg(q_pre)[0][0]
    turned_q[2] += holster.TURN_ELBOW_RAD          # the elbow opened as Joint1 turns (`holster.unfold_stages`)
    turned = servo(turned_q)
    from position_only.workspace import load_urdf

    above = servo(holster.above_pre(load_urdf()[0], on_q[0]))
    opened, held = GRIPPER_OPEN_UNITS, GRIPPER_HOLD_UNITS
    joint = lambda a, b: holster.joint_move(a, b, JOINT_SPEED_DEG_S, ACCEL_DEG_S2, PERIOD_S)
    reach, lower = joint(turned, above), joint(above, q_pre)
    # The way back is the way out played backwards: with each joint at its own pace, a move from B to A is not the
    # move from A to B reversed, and the fresh one from the point above to the turned fold put the hand 0.2 mm off the
    # D1's base on the CPU model (2026-10-23). Reversed, every leg is one `holster.check` cleared.
    return [
        {"name": "turn", "goal": turned, "gripper": opened},
        {"name": "reach", "path": reach, "joint": True, "gripper": opened},
        {"name": "lower", "path": lower, "joint": True, "gripper": opened},
        {"name": "on", "path": on_path, "gripper": opened},
        {"name": "close", "wait": JAW_S, "gripper": held},
        {"name": "draw", "path": draw_path, "gripper": held},
        {"name": "exit", "path": exit_path, "gripper": held},
        {"name": "hold", "wait": HOLD_S, "gripper": held},
        {"name": "enter", "path": exit_path[::-1], "gripper": held},
        {"name": "back", "path": draw_path[::-1], "gripper": held},
        {"name": "open", "wait": JAW_S, "gripper": opened},
        {"name": "off", "path": on_path[::-1], "gripper": opened},
        {"name": "raise", "path": lower[::-1], "joint": True, "gripper": opened},
        {"name": "unreach", "path": reach[::-1], "joint": True, "gripper": opened},
        {"name": "fold", "reverse_of": "turn", "joint": True, "gripper": opened},
    ]


def describe(moves: list[dict]) -> list[str]:
    out = []
    for m in moves:
        if "goal" in m:
            out.append(f"  {m['name']:8s} joint space to {np.round(m['goal'], 1).tolist()} deg")
        elif "reverse_of" in m:
            out.append(f"  {m['name']:8s} `{m['reverse_of']}` played backwards, to where the arm started")
        elif m.get("joint"):
            out.append(f"  {m['name']:8s} joint space to {np.round(m['path'][-1], 1).tolist()} deg, "
                       f"{(len(m['path']) - 1) * PERIOD_S:.1f} s")
        elif "path" in m:
            path = m["path"]
            span = np.abs(path[-1] - path[0])
            out.append(f"  {m['name']:8s} streamed {len(path) * PERIOD_S:.1f} s, the most-moving joint J{int(np.argmax(span)) + 1}"
                       f" {span.max():.0f} deg")
        else:
            out.append(f"  {m['name']:8s} jaws to {m['gripper']:.1f} units, {m['wait']:.1f} s")
    return out


def move_path(m: dict, start, played: dict | None = None) -> np.ndarray:
    """The timed path (every `PERIOD_S`, servo degrees) the link plays for move `m`, from where the last one ended: a
    joint move to a goal (`holster.joint_move`: each joint at the full pace while it moves), a planned path, an earlier
    move's path backwards (`reverse_of`, from `played`), given points (`points`: a retrace, `recovery`), or the arm held
    still through a wait."""
    if "points" in m:
        return np.asarray(m["points"], dtype=float)
    if "reverse_of" in m:
        return np.asarray(played[m["reverse_of"]], dtype=float)[::-1]
    if "goal" in m:
        path = holster.joint_move(start, m["goal"], JOINT_SPEED_DEG_S, ACCEL_DEG_S2, PERIOD_S)
        return path if len(path) > 1 else np.stack([path[0], path[0]])
    if "path" in m:
        return np.asarray(m["path"], dtype=float)
    return np.repeat(np.asarray(start, dtype=float)[None], int(round(m["wait"] / PERIOD_S)) + 1, axis=0)


def recovery(moves: list[dict], played: dict, k: int, q) -> list[dict]:
    """The way back to where the arm started after a stop in move `k` (the arm at `q`, servo degrees), only over paths
    it has played or the plan's way back: on the way out (before `back`), the moves played so far backwards, the one it
    stopped in from its point nearest `q`, each with the jaws as they were before it; from `back` on, the rest of the
    plan's way back, the one it stopped in from its point nearest `q`. Each is a move for `run_move`, `of` the planned
    move it retraces or finishes."""
    way_back = [m["name"] for m in moves].index("enter")
    q = np.asarray(q, dtype=float)

    def from_nearest(path, backwards: bool) -> np.ndarray:
        i = int(np.argmin(np.abs(path - q).max(axis=1)))
        path = path[:i + 1][::-1] if backwards else path[i:]
        return path if len(path) > 1 else np.stack([path[0], path[0]])

    out = []
    if k < way_back:
        for j in range(k, -1, -1):
            m = moves[j]
            if m["name"] not in played:
                continue
            path = np.asarray(played[m["name"]], dtype=float)
            # A wait undone whole: the jaws get their time.
            points = path[::-1] if "wait" in m else from_nearest(path, backwards=True) if j == k else path[::-1]
            out.append({"name": f"undo {m['name']}", "points": points, "gripper": moves[max(j - 1, 0)]["gripper"],
                        "joint": "goal" in m or m.get("joint", False), "of": j})
        return out
    for j in range(k, len(moves)):
        m = moves[j]
        if j == k and m["name"] in played and "wait" not in m:
            out.append({"name": m["name"], "points": from_nearest(np.asarray(played[m["name"]], dtype=float), False),
                        "gripper": m["gripper"], "joint": "goal" in m or "reverse_of" in m or m.get("joint", False),
                        "of": j})
        else:
            out.append({**m, "of": j})
    return out


# --- the arm -------------------------------------------------------------------------------------------------------

class LinkArm:
    """`arm_link.py`'s socket (as the door program's `node.ArmLink`)."""

    def __init__(self, port: int = ARM_LINK_PORT):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=3.0)
        self.sock.settimeout(None)
        self.lock = threading.Lock()
        self.q, self.gripper, self.rx, self.hello = None, None, 0.0, None
        self.errors: list[str] = []
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        buffer = b""
        while True:
            try:
                chunk = self.sock.recv(65536)
            except OSError:
                return
            if not chunk:
                return
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                try:
                    msg = json.loads(line)
                except ValueError:
                    continue
                with self.lock:
                    if msg.get("type") == "feedback" and msg.get("q") is not None:
                        self.q = np.asarray(msg["q"], dtype=float)
                        self.gripper = msg.get("gripper")
                        self.rx = time.monotonic() - float(msg.get("age_s", 0.0))
                    elif msg.get("type") == "hello":
                        self.hello = msg
                    elif msg.get("type") == "error":
                        self.errors.append(str(msg.get("detail")))
                        log(f"arm link: {msg.get('detail')}")

    def latest(self):
        with self.lock:
            age = time.monotonic() - self.rx if self.rx else math.inf
            return (None if self.q is None else self.q.copy()), self.gripper, age

    def path(self, points, gripper, lead_max: float = LEAD_KEY_DEG) -> None:
        """A timed path for the link to play out at its 10 Hz (`arm_link.PathPlayer`)."""
        self.errors.clear()
        self.sock.sendall((json.dumps({"type": "path", "q": [[round(float(v), 3) for v in p] for p in points],
                                       "dt": PERIOD_S, "gripper": round(float(gripper), 2),
                                       "lead_max_deg": float(lead_max)}) + "\n").encode())

    def tick(self) -> None:
        """One 100 ms: the link told we are still here (its watchdog holds the arm after 0.5 s of silence)."""
        self.sock.sendall(b'{"type": "keepalive"}\n')
        time.sleep(PERIOD_S)
        if self.errors:
            raise Stop(f"arm link: {self.errors[-1]}")

    def hold(self) -> None:
        try:
            self.sock.sendall(b'{"type": "hold"}\n')
        except OSError:
            pass


class SimArm:
    """A dry run's arm: the link's path player (`arm_link.PathPlayer`, checked as the link checks a path) driving
    `arm_link.SimulatedArm`'s model of the servo, folded at the start."""

    def __init__(self):
        from demos.combiner_door.robot import arm_link

        self.arm_link = arm_link
        self.sim = arm_link.SimulatedArm(d1_hardware.REST_SERVO_DEG)
        self.player, self.gripper_cmd = None, None

    def latest(self):
        return self.sim.q.copy(), self.sim.gripper, 0.0

    def path(self, points, gripper, lead_max: float = LEAD_KEY_DEG) -> None:
        why = self.arm_link.check_path(points, PERIOD_S, self.sim.q)
        if why:
            raise Stop(f"arm link (simulated): {why}")
        self.player, self.gripper_cmd = self.arm_link.PathPlayer(points, PERIOD_S, lead_max=lead_max), gripper

    def tick(self) -> None:
        self.sim.step_command(self.player.step(self.sim.q, 0.0, PERIOD_S), self.gripper_cmd, PERIOD_S)

    def hold(self) -> None:
        pass


class Remote:
    """The Go2 remote's A and B (`lowlevel.RemoteButtons`) from `LowState.wireless_remote`, put on `keys` as
    "remote A" / "remote B". Reading only: a ROS node of its own, spun on a thread. None of it without ROS (a plain
    `python3`): the keyboard alone then."""

    def __init__(self, keys: queue.Queue, topic: str = "/lf/lowstate"):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from unitree_go.msg import LowState

        from demos.combiner_key.robot import lowlevel

        self.keys, self.lowlevel = keys, lowlevel
        self.buttons = lowlevel.RemoteButtons()
        # rclpy's own SIGINT handler would take Ctrl-C from the main thread, which holds the arm on it.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("holster_dog_remote")
        self.node.create_subscription(LowState, topic, self._on_state, qos_profile_sensor_data)
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def _on_state(self, msg) -> None:
        for press in self.buttons.update(self.lowlevel.remote_keys(msg.wireless_remote)):
            self.keys.put(f"remote {press}")


# --- the run -------------------------------------------------------------------------------------------------------

class Stop(Exception):
    pass


class Runner:
    def __init__(self, arm, live: bool, step: bool, run_dir: Path, moves: list[dict] | None = None):
        self.arm, self.live, self.step = arm, live, step
        self.run_dir = run_dir
        self.moves = moves or []
        #: The move in progress (or waited for), and every move's path as played: what a recovery retraces.
        self.k = 0
        self.played: dict = {}
        self.keys: queue.Queue = queue.Queue()
        self.rows: list[dict] = []
        self.t0 = time.monotonic()
        self.events: list[dict] = []
        if live:
            threading.Thread(target=self._stdin, daemon=True).start()

    def _stdin(self) -> None:
        for line in sys.stdin:
            self.keys.put(line.strip().lower())

    def event(self, **row) -> None:
        self.events.append({"t_s": round(time.monotonic() - self.t0, 2), **row})

    def _key(self, block: bool, stopped: bool = False) -> str | None:
        """The operator's next line (None: none yet), the remote's presses (`Remote`) as the lines they stand for: B
        is `a`, A is Enter, or `r` once stopped (`lowlevel.RemoteButtons`)."""
        try:
            key = self.keys.get() if block else self.keys.get_nowait()
        except queue.Empty:
            return None
        if key in ("remote A", "remote B"):
            log(f"the remote's {key[7:]}")
            return "a" if key == "remote B" else "r" if stopped else ""
        return key

    def _check_keys(self) -> None:
        while (key := self._key(block=False)) is not None:
            if key in ("a", "q"):
                raise Stop(f"operator: {key}")

    def _wait_enter(self, prompt: str) -> None:
        if not self.live:
            return
        log(prompt + " -- Enter or A to go, a + Enter or B to stop")
        while True:
            key = self._key(block=True)
            if key in ("a", "q"):
                raise Stop(f"operator: {key}")
            if key == "":
                return

    def wait_recover(self) -> bool:
        """After a stop: whether the operator asks for the way back to the start (`r` + Enter, or A), or to quit (`q` +
        Enter; Ctrl-C raises)."""
        log("stopped, the arm held. r + Enter or A: back to where it started, the way it came; q + Enter or Ctrl-C: "
            "quit with the arm held where it is")
        while True:
            key = self._key(block=True, stopped=True)
            if key == "r":
                return True
            if key == "q":
                return False

    def recover(self) -> list[dict]:
        """Back to where the arm started from wherever it stopped (`recovery`). A stop on the way raises `Stop` with
        `k` where it is, so a second recovery starts from there."""
        q = self.arm.latest()[0]
        segments = recovery(self.moves, self.played, self.k, q)
        log("back to the start: " + (" -> ".join(m["name"] for m in segments) or "nothing to undo"))
        self.where = q
        done = []
        for m in segments:
            self.k = m["of"]
            log(m["name"])
            done.append(self.run_move(m))
            self.event(kind="recover", **done[-1])
        return done

    def _tick(self, name: str, q_cmd, gripper) -> np.ndarray:
        """One 100 ms tick of a path the link is playing: the report in, a row of the trace (`q_cmd`: the path's point
        for this tick by the clock; the link's own commands are in its log)."""
        self._check_keys()
        self.arm.tick()
        q, grip, age = self.arm.latest()
        if q is None or age > STALE_S:
            raise Stop(f"the arm's report is {age:.1f} s old")
        self.rows.append({"t_s": round(time.monotonic() - self.t0, 2), "move": name,
                          **{f"cmd{j + 1}": round(float(v), 2) for j, v in enumerate(q_cmd)},
                          **{f"q{j + 1}": round(float(v), 2) for j, v in enumerate(q)},
                          "gripper_cmd": round(float(gripper), 2), "gripper": grip, "age_s": round(age, 3)})
        return q

    def run_move(self, m: dict) -> dict:
        """The move as one timed path for the link (`move_path`), then until the arm is within `ARRIVED_DEG` of its
        end (at most `SETTLE_S` more for a streamed move or a wait, `MOVE_TIMEOUT_S` in all)."""
        name, gripper = m["name"], m["gripper"]
        started = time.monotonic()
        points = move_path(m, self.where, self.played)
        self.played[name] = points
        self.arm.path(points, gripper, LEAD_KEY_DEG if name in ("draw", "exit", "enter", "back", "close", "hold", "open")
                      else LEAD_FREE_DEG)
        lag, ticks, q = 0.0, 0, None
        for point in points[1:]:
            q = self._tick(name, point, gripper)
            lag = max(lag, float(np.max(np.abs(q - point))))
            ticks += 1
        end = points[-1]
        joint = "goal" in m or m.get("joint", False)
        settle_until = ticks * PERIOD_S + (MOVE_TIMEOUT_S if joint else SETTLE_S)
        while np.max(np.abs(q - end)) >= ARRIVED_DEG:
            if ticks * PERIOD_S > min(settle_until, MOVE_TIMEOUT_S):
                if joint:
                    raise Stop(f"{name}: the arm is {np.max(np.abs(q - end)):.1f} deg from its goal after "
                               f"{ticks * PERIOD_S:.0f} s")
                break
            q = self._tick(name, end, gripper)
            ticks += 1
        self.where = end
        return {"move": name, "ticks": ticks, "planned_s": round((len(points) - 1) * PERIOD_S, 2),
                "wall_s": round(time.monotonic() - started, 2), "path_lag_max_deg": round(lag, 2),
                "end_error_deg": round(float(np.max(np.abs(q - end))), 2)}

    def run(self, moves: list[dict]) -> dict:
        done = []
        self.moves = moves
        self.where = self.arm.latest()[0]
        for self.k, m in enumerate(moves):
            if self.step:
                self._wait_enter(f"next: {m['name']}")
            log(f"{m['name']}")
            done.append(self.run_move(m))
            self.event(kind="move", **done[-1])
        return {"moves": done}


def check_start(arm) -> str | None:
    """Why the arm may not start, or None."""
    deadline = time.monotonic() + 3.0
    q = None
    while time.monotonic() < deadline:
        q, _, age = arm.latest()
        if q is not None and age < STALE_S:
            break
        time.sleep(0.1)
    if q is None:
        return "no report from the arm (is the arm link up and the arm powered?)"
    off = np.abs(q - np.asarray(FOLD_SERVO_DEG))
    over = off - np.asarray(FOLDED_DEG)
    if np.max(over) > 0.0:
        j = int(np.argmax(over))
        return (f"the arm is not folded: J{j + 1} is {off[j]:.0f} deg from the fold (at most {FOLDED_DEG[j]:.0f}; "
                f"{np.round(q, 1).tolist()}); fold it first")
    return None


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--live", action="store_true", help="move the real arm (otherwise a simulated one)")
    p.add_argument("--step", action="store_true", help="an Enter before every move")
    p.add_argument("--joint_speed", type=float, default=JOINT_SPEED_RAD_S,
                   help="the streamed moves' pace on the fastest joint, rad/s (at most 0.75: the link's 45 deg/s)")
    p.add_argument("--hz", type=float, choices=(10.0, 5.0), default=10.0,
                   help="commands a second: 10, or 5 with each step twice as large (F-202)")
    p.add_argument("--run_dir", default=None)
    args = p.parse_args(argv)
    global PERIOD_S
    PERIOD_S = 1.0 / args.hz
    if not 0.1 <= args.joint_speed <= 0.75:
        p.error("--joint_speed must be within 0.1-0.75 rad/s")

    stamp = time.strftime("%Y%m%dT%H%M%S", time.gmtime())
    run_dir = Path(args.run_dir) if args.run_dir else LOG_ROOT / f"{stamp}_{'live' if args.live else 'dry'}"
    run_dir.mkdir(parents=True, exist_ok=True)
    log(f"planning the draw on the CPU arm model (joint pace {args.joint_speed} rad/s, {args.hz:g} commands a second) ...")
    moves = plan(args.joint_speed)
    log("the moves:\n" + "\n".join(describe(moves)))
    record = {"mode": "holster_dog", "live": args.live, "step": args.step, "joint_speed_rad_s": args.joint_speed,
              "command_hz": args.hz,
              "holster_registration": holster.load()["inputs"], "fold_servo_deg": FOLD_SERVO_DEG,
              "argv": sys.argv, "status": "running"}

    if not args.live:
        # Dry: the real arm's report through the link, if it is up, for the start check the live run will make.
        try:
            real = LinkArm()
            why = check_start(real)
            record["real_arm_start_check"] = why or "ok: folded, its report fresh"
            log(f"the real arm (not moved): {record['real_arm_start_check']}")
        except OSError as exc:
            record["real_arm_start_check"] = f"no arm link: {exc}"
            log(f"no arm link ({exc}); the dry run goes on with a simulated arm")
    arm = LinkArm() if args.live else SimArm()
    if args.live and arm.hello and arm.hello.get("observe"):
        log("the arm link is in observe mode (a dry launcher?): it will not move the arm")
        return 2
    why = check_start(arm)
    if why:
        log(f"refusing: {why}")
        record.update(status="refused", why=why)
        (run_dir / "run.json").write_text(json.dumps(record, indent=2, default=str))
        return 2
    runner = Runner(arm, args.live, args.step, run_dir, moves)
    # Dry too, so a dry run shows the remote is read (B stops the simulated arm; A is logged).
    try:
        Remote(runner.keys)
        log("the remote: A is Enter (and r + Enter once stopped), B is a + Enter")
    except Exception as exc:  # noqa: BLE001 -- the remote is a convenience; the keyboard always works
        log(f"no remote ({exc!r}): the keyboard only")
    status, why = "finished", ""
    try:
        if args.live and not args.step:
            runner._wait_enter("LIVE: the arm will unfold beside its base on the robot's right and back")
        record.update(runner.run(moves))
    except (Stop, KeyboardInterrupt) as exc:
        status, why = "stopped", str(exc) or "Ctrl-C"
        arm.hold()
        log(f"stopped: {why}; the arm is held where it is (the link never releases it)")
        # Live, after a stop that was not Ctrl-C or `q`: `r` + Enter or A takes the arm back to where it started, as often as
        # it is asked (a stop on the way waits again). Its moves are the events of kind `recover`.
        try:
            while args.live and isinstance(exc, Stop) and why != "operator: q" and runner.wait_recover():
                try:
                    runner.recover()
                    status = "stopped, then back at the start"
                    break
                except Stop as again:
                    arm.hold()
                    why += f"; the way back stopped: {again}"
                    log(f"stopped on the way back: {again}; the arm is held where it is")
        except KeyboardInterrupt:
            arm.hold()
            why += "; Ctrl-C"
    finally:
        arm.hold()
        record.update(status=status, why=why, events=runner.events)
        (run_dir / "run.json").write_text(json.dumps(record, indent=2, default=str))
        if runner.rows:
            with (run_dir / "trace.csv").open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(runner.rows[0].keys()))
                writer.writeheader()
                writer.writerows(runner.rows)
        log(f"{status}; record: {run_dir}")
    return 0 if status == "finished" else 1


if __name__ == "__main__":
    raise SystemExit(main())
