#!/usr/bin/env python3
"""The general foot-press policy on the dog: the front-right foot put out, then pressed onto the operator's hand.

DRY unless `--live`: the dry run publishes nothing (no LowCmd, no sport or motion-switcher request) and runs the policy
open loop on the robot standing in sport mode, as `shadow.py` does, with the task layer's stages and prompts as they
would be live. `--rehearse` feeds the layer a pretend foot instead -- the goal itself, stopped at the plate -- so the
whole sequence of a press (lift, wait for the hand, stall, press, release, return) can be walked through with the dog
standing; the policy's targets then answer commands from a foot that is not there.

LIVE (`--live`, after a typed confirmation), with the dog standing in sport mode and the arm folded:

  1. sport mode's StandDown: the dog lies down; its standing joint angles, read just before, are the stand it returns
     to (the training started from sport-mode stands, `demos/combiner_key/sport_stands.py`);
  2. the motion switcher releases sport mode (checked by CheckMode), and nothing else may be publishing `/lowcmd`;
  3. stand-up on unitree_rl_lab's FixStand gains (`lowlevel.STAND_KP/KD`) from where it lies to that stand, held, then
     the gains blended to the policy's kp 25 / kd 0.5;
  4. the policy (`leg_policy.py`) with the press layer (`press_layer.PressTask`): Enter lifts the foot to the
     pre-contact point off the plate (`--plate`); with the hand there, Enter again presses: the foot moves in along
     the normal until it stalls on the hand, ramps to `--force_n` (at most `--max_force_n`, 5 N unless raised), holds,
     ramps off, backs out, and goes home to its footfall;
  5. back on the FixStand gains, lie down, damping (kp 0, kd 3), and -- on Enter -- sport mode selected again.

Stops: Ctrl-C, `a` + Enter, the remote's B, a tilt past `--tilt_limit_deg`, a joint past its limit, or a LowState older
than 0.1 s put every motor in damping at once (the dog sinks down) and wait for Enter to give the legs back to sport
mode. The remote's A is Enter (and `live` + Enter at the confirmation), as in every live program here
(`lowlevel.RemoteButtons`): A goes, B stops, A again gives the dog back to sport mode, where it started. `q` + Enter (or `--policy_s` running out) ends gently: the press released, the foot home, then step 5. The
decoder's estimate of the foot's force past `--force_abort_n` for 0.1 s releases the press.

Not validated: this program has run only against `fake_go2.py` on the PC (a kinematic stand-in, no dynamics); the
policy has never driven the real legs, and the press layer's numpy port is checked against the torch layer, not in a
closed loop. See the week-6 log.

    run_press_on_dog.sh --policy checkpoints/leg_press_force_r4_model_5999.npz                 # dry
    run_press_on_dog.sh --policy ... --rehearse                                               # dry, a pretend foot
    run_press_on_dog.sh --policy ... --live                                                   # the legs driven

The record is `logs/leg_press/<stamp>_leg_press[_live]/` (`press.npz`, `summary.json`, `run.json`); `d1fetch` copies it.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import queue
import shutil
import signal
import sys
import threading
import time
from pathlib import Path

for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import numpy as np  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

from demos.combiner_key.robot import leg_policy, lowlevel as ll, press_layer as pl  # noqa: E402
from demos.combiner_key.robot.shadow import ARM_LINK_PORT, ARM_STALE_S, FLUSH_S, STALE_S, ArmFeedback  # noqa: E402

#: The Go2's joint limits (the URDF's, `unifp_train.task_cfg.JOINT_POS_LIMITS`) in Unitree's order: targets are clamped
#: to them less `TARGET_MARGIN_RAD`, and a joint measured past them by `ABORT_MARGIN_RAD` stops the run.
_FRONT, _REAR = ((-1.0472, 1.0472), (-1.5708, 3.4907), (-2.7227, -0.8378)), \
    ((-1.0472, 1.0472), (-0.5236, 4.5379), (-2.7227, -0.8378))
JOINT_LIMITS = np.array(_FRONT + _FRONT + _REAR + _REAR)
TARGET_MARGIN_RAD, ABORT_MARGIN_RAD = 0.05, 0.02
#: The stand-up's and lie-down's timing (s).
STAND_S, STAND_HOLD_S, GAIN_BLEND_S = 2.5, 1.0, 1.0
#: After the blend the policy starts once no joint has moved this far over this many ticks (at most this long), and
#: only if the stand is within these of the trained settled stance. A policy exported with a `start_gate` (the stances
#: its training started from, `export_policy.start_gate`) replaces the joint bound and the pitch's by its own; roll
#: stays bounded by SETTLED_TILT_DEG.
SETTLED_MOVE_RAD, SETTLED_TICKS, SETTLE_MAX_S = 0.01, 15, 3.0
SETTLED_TOL_RAD, SETTLED_TILT_DEG = 0.20, 5.0
END_BLEND_S, END_STAND_S, LIE_S, DAMP_S = 0.5, 1.0, 2.5, 1.5
#: Sport mode's StandDown: how long to wait for the dog to lie, and the calf angle that counts as lying (rad).
LIE_WAIT_S, LYING_CALF_RAD = 6.0, -2.3
#: The motion switcher's answers are waited for this long; /lowcmd listened to this long for anyone else.
SWITCH_TIMEOUT_S, LOWCMD_LISTEN_S = 6.0, 0.4
#: The policy ends this long after the foot is back on its footfall.
FOOT_DOWN_END_S = 1.0
#: A standing robot's base is at least this high over its feet (m); the live run starts only from a stand.
STANDING_MIN_HEIGHT_M = 0.25
#: The rehearsal's pretend hand: it stops the pretend foot this close to the plate's contact point, across its normal.
REHEARSAL_HAND_M = 0.05
#: The modes in which the legs are ours (live), so a stop means damping.
DRIVEN = ("stand_up", "policy", "stand_end", "lie_end")


def log(msg: str) -> None:
    print(f"[press] {msg}", flush=True)


# --- where the foot is ------------------------------------------------------------------------------------------------

def quat_matrix(quat_wxyz) -> np.ndarray:
    w, x, y, z = (float(v) for v in quat_wxyz)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def level_rotation(quat_wxyz) -> np.ndarray:
    """The base's rotation without its yaw: base frame -> the yaw frame."""
    r = quat_matrix(quat_wxyz)
    yaw = math.atan2(r[1, 0], r[0, 0])
    c, s = math.cos(-yaw), math.sin(-yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ r


LEG_ORDER = ("FR", "FL", "RR", "RL")          # Unitree's
SUPPORT = ("FL", "RR", "RL")


def feet(q_unitree, quat_wxyz, meta) -> tuple[dict, float]:
    """Each foot sphere's centre in the yaw frame from the ground under the base, and the base's height over the
    ground, taken as the three support feet's mean (each sphere `FOOT_RADIUS_M` above it)."""
    rot = level_rotation(quat_wxyz)
    q = np.asarray(q_unitree, dtype=float)
    out = {leg: rot @ leg_policy.foot_local(leg, q[3 * k:3 * k + 3], meta) for k, leg in enumerate(LEG_ORDER)}
    height = pl.FOOT_RADIUS_M - float(np.mean([out[leg][2] for leg in SUPPORT]))
    for leg in out:
        out[leg] = out[leg] + np.array([0.0, 0.0, height])
    return out, height


def fr_foot_velocity(q_unitree, dq_unitree, quat_wxyz, meta, eps: float = 1e-4) -> np.ndarray:
    """The FR foot's velocity from its joints alone (the base taken as still), yaw frame."""
    q, dq = np.asarray(q_unitree[0:3], dtype=float), np.asarray(dq_unitree[0:3], dtype=float)
    jac = np.stack([(leg_policy.foot_local("FR", q + eps * e, meta) - leg_policy.foot_local("FR", q - eps * e, meta))
                    / (2 * eps) for e in np.eye(3)], axis=1)
    return level_rotation(quat_wxyz) @ (jac @ dq)


# --- the robot's side ------------------------------------------------------------------------------------------------

class LiveIO:
    """Everything that talks to the robot: LowCmd (through a writer thread at `hz`), the motion switcher, sport
    requests. Only built with `--live`."""

    def __init__(self, node, hz: float):
        from unitree_api.msg import Request, Response
        from unitree_go.msg import LowCmd
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        self.LowCmd, self.Request = LowCmd, Request
        # Reliable, depth 10, as unitree_ros2's low-level examples publish it (a best-effort writer would not match a
        # reliable reader); anyone else's LowCmd is listened for best-effort, which matches either.
        self.lowcmd = node.create_publisher(LowCmd, "/lowcmd", 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.switch = node.create_publisher(Request, ll.MOTION_SWITCHER_REQUEST, 10)
        self.sport = node.create_publisher(Request, ll.SPORT_REQUEST, 10)
        node.create_subscription(Response, ll.MOTION_SWITCHER_RESPONSE, self._on_response, 10)
        self.foreign_lowcmd = 0
        self.listening = False
        node.create_subscription(LowCmd, "/lowcmd", self._on_lowcmd, qos)
        self.responses: dict[int, tuple[int, str]] = {}
        self.next_id = int(time.time() * 1000) % 2 ** 40
        self.msg, self.lock, self.stop = None, threading.Lock(), threading.Event()
        self.period = 1.0 / hz
        self.sent = 0
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.writing = False
        #: Why the writer stopped, if it did on its own.
        self.error: str | None = None

    # requests
    def _id(self) -> int:
        self.next_id += 1
        return self.next_id

    def _on_response(self, msg) -> None:
        self.responses[int(msg.header.identity.id)] = (int(msg.header.status.code), str(msg.data))

    def _on_lowcmd(self, msg) -> None:
        if self.listening:
            self.foreign_lowcmd += 1

    def switcher(self, api_id: int, parameter: dict | None = None) -> int:
        req = self.Request()
        ident = self._id()
        ll.fill_request(req, api_id, ident, parameter)
        self.switch.publish(req)
        return ident

    def sport_request(self, api_id: int) -> None:
        req = self.Request()
        ll.fill_request(req, api_id, self._id())
        self.sport.publish(req)

    # LowCmd
    def command(self, q, kp, kd) -> None:
        msg = self.LowCmd()
        ll.init_lowcmd(msg)
        ll.fill_legs(msg, q, kp, kd)
        with self.lock:
            self.msg = msg
        # Out now, not at the writer's next slot (up to 2 ms later); the writer repeats it until the next command.
        try:
            self.lowcmd.publish(msg)
        except Exception as error:
            self.error = repr(error)
        if not self.writing:
            self.writing = True
            self.thread.start()

    def _write(self) -> None:
        nxt = time.monotonic()
        while not self.stop.is_set():
            with self.lock:
                msg = self.msg
            if msg is not None:
                try:
                    self.lowcmd.publish(msg)
                except Exception as error:      # the context gone: nothing more can be sent
                    self.error = repr(error)
                    return
                self.sent += 1
            nxt += self.period
            delay = nxt - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.monotonic()

    def stop_writing(self) -> None:
        self.stop.set()
        if self.writing:
            self.thread.join(timeout=1.0)


class Operator:
    """Lines typed on the terminal: Enter (go), `q` (end gently), `a` (stop: damping)."""

    def __init__(self):
        self.events: queue.Queue[str] = queue.Queue()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        for line in sys.stdin:
            self.events.put(line.strip().lower())

    def drain(self) -> list[str]:
        out = []
        while True:
            try:
                out.append(self.events.get_nowait())
            except queue.Empty:
                return out


# --- the program -----------------------------------------------------------------------------------------------------

class PressRun:
    def __init__(self, args, io: LiveIO | None, operator: Operator | None):
        self.args, self.io, self.operator = args, io, operator
        self.live = io is not None
        self.policy = leg_policy.LegPolicy(args.policy)
        self.meta = self.policy.meta
        if self.meta.get("variant") != "force":
            raise SystemExit("the press layer commands a force: it needs a force-aware export")
        self.obs = leg_policy.LegObservation(self.meta)
        self.kp, self.kd = float(self.meta["leg_kp"]), float(self.meta["leg_kd"])
        #: The policy's zero-action targets and the stance gravity settles them into in training, Unitree's order.
        self.default_q = self.obs.default[:12][self.obs.to_unitree]
        self.settled_q = np.asarray(self.meta["settled_leg_pos"], dtype=float)[self.obs.to_unitree]
        self.still_ticks = 0
        self.settle_q: list[np.ndarray] = []
        self.rest = np.asarray(self.meta["arm_rest_pose"], dtype=float)
        if args.force_n > args.max_force_n:
            raise SystemExit(f"--force_n {args.force_n} is above --max_force_n {args.max_force_n}")
        self.plate = pl.PLATES[args.plate]
        self.profile = pl.Profile(levels=[args.force_n], ramps=[args.ramp_s], holds=[args.hold_s],
                                  standoff=args.standoff, speed=args.speed)
        self.task = self._new_task()
        self.state, self.state_msg, self.state_rx, self.lock = None, None, 0.0, threading.Lock()
        self.tick_now = 0.0
        #: This tick's time from its start to the command's send (ms).
        self.send_ms = math.nan
        self.arm = None
        if args.arm_link_port:
            try:
                self.arm = ArmFeedback(args.arm_link_port)
                log(f"arm link on 127.0.0.1:{args.arm_link_port} (listening only)")
            except OSError as error:
                log(f"no arm link ({error}): the arm is taken at its rest pose")
        self.mode, self.mode_t0 = "preflight", time.monotonic()
        self.sport_name = ""
        self.pending: int | None = None
        self.pending_t = 0.0
        self.stand_q = self.lie_q = self.from_q = None
        self.from_kp = self.from_kd = None
        self.abort_reason: str | None = None
        self.end_reason: str | None = None
        self.go = False
        self.policy_ticks = 0
        self.down_for = 0.0
        self.force_high = 0
        self.rehearse_foot = None
        self.last_print = 0.0
        self.last_keys: set[str] = set()
        self.buttons = ll.RemoteButtons()
        self.max_tilt_deg = 0.0
        self.handed_back = False
        self.confirming = self.asked_handback = False
        self.release_step = self.back_step = ""
        self.typed = ""
        self.warned_pose = False
        self.reported_writer = False
        #: Sport mode released by this run and not yet selected again.
        self.released = False
        #: Each press's outcome, as the policy finishes it (`--presses`).
        self.press_results: list[dict] = []
        self.start = None
        self.rows: dict[str, list] = {k: [] for k in (
            "t", "dt", "state_age", "mode", "stage", "phase", "level", "stalled", "q", "dq", "tau_est", "quat", "gyro",
            "rpy", "foot_force", "arm_q", "arm_measured", "foot", "foot_v", "height", "commands", "frame", "action",
            "target", "kp", "kd", "sent", "estimate", "infer_ms", "goal", "offset", "go", "remote", "send_ms")}
        self.last_tick = None
        suffix = "_live" if self.live else ("_rehearse" if args.rehearse else "")
        self.run_dir = Path(args.run_dir) / f"{time.strftime('%Y%m%dT%H%M%S', time.gmtime())}_leg_press{suffix}"
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.parts, self.flushed = self.run_dir / "parts", 0
        self.events: list[dict] = []

    def _new_task(self) -> pl.PressTask:
        return pl.PressTask(self.plate, self.profile, down_goal=self.meta["key_down_goal"],
                            goal_centre=self.meta["goal_centre"], max_force_n=self.args.max_force_n,
                            hand_wait_s=self.args.hand_wait_s)

    # --- input -----------------------------------------------------------------------------------------------------

    def on_state(self, msg) -> None:
        """Keeps the message only: at 500 Hz the conversion is done once per policy step instead (`_convert`)."""
        with self.lock:
            self.state_msg, self.state_rx = msg, time.monotonic()

    @staticmethod
    def _convert(msg) -> dict:
        return {"q": np.array([msg.motor_state[i].q for i in range(12)]),
                 "dq": np.array([msg.motor_state[i].dq for i in range(12)]),
                 "tau_est": np.array([msg.motor_state[i].tau_est for i in range(12)]),
                 "quat": np.array(msg.imu_state.quaternion, dtype=float),
                 "gyro": np.array(msg.imu_state.gyroscope, dtype=float),
                 "rpy": np.array(msg.imu_state.rpy, dtype=float),
                 "foot_force": np.array(msg.foot_force, dtype=float),
                 "keys": ll.remote_keys(msg.wireless_remote)}

    def event(self, what: str, **extra) -> None:
        t = 0.0 if self.start is None else time.monotonic() - self.start
        self.events.append({"t": round(t, 3), "event": what, **extra})
        log(what + (f" {extra}" if extra else ""))

    def set_mode(self, mode: str) -> None:
        self.event(f"mode {self.mode} -> {mode}")
        self.mode, self.mode_t0 = mode, time.monotonic()

    def abort(self, why: str) -> None:
        """Damping now (live), or the dry run's end."""
        if self.abort_reason is None:
            self.abort_reason = why
        if self.live and self.mode in DRIVEN + ("damping",):
            if self.mode != "damping":
                self.event(f"STOP ({why}): damping, every motor kp 0 kd {ll.DAMP_KD}")
                self.set_mode("damping")
            self._send(self.state["q"] if self.state else np.zeros(12), 0.0, ll.DAMP_KD)
        elif self.live and self.mode in ("release", "check_lowcmd"):
            # Sport mode may be released and nothing commands the legs: give them back to it.
            self.event(f"STOP ({why}) before the legs were driven: sport mode selected again")
            self.pending = None
            self.set_mode("handback")
        elif self.live and self.mode == "handback":
            pass
        else:
            self.event(f"stop ({why})")
            self.set_mode("done")

    def _send(self, q, kp, kd) -> None:
        if self.io is not None:
            self.io.command(q, kp, kd)

    # --- the loop --------------------------------------------------------------------------------------------------

    def tick(self) -> None:
        now = self.tick_now = time.monotonic()
        with self.lock:
            msg, age = self.state_msg, now - self.state_rx
        if msg is None:
            return
        state = self.state = self._convert(msg)
        if self.start is None:
            self.start = now
            self.event("first LowState")
        keys = state["keys"]
        if keys != self.last_keys:
            if keys:
                log(f"remote: {sorted(keys)}")
            self.last_keys = keys
        if self.live and self.io.error and not self.reported_writer:
            self.reported_writer = True
            self.event(f"LowCmd can no longer be sent ({self.io.error}): the motors keep the last command; "
                       "switch the dog off or select sport mode from the app")
        for press in self.buttons.update(keys):
            if press == "B":                    # `a` + Enter
                self.abort("the remote's B")
            else:                               # Enter, and `live` + Enter at the live run's confirmation
                self.typed, self.go = "live", True
        if age > STALE_S and self.mode in DRIVEN:
            self.abort(f"LowState {age:.2f} s old")
        for line in (self.operator.drain() if self.operator else []):
            self._operator(line)
        q = state["q"]
        rp = np.degrees(leg_policy.roll_pitch(state["quat"]))
        out = (q < JOINT_LIMITS[:, 0] - ABORT_MARGIN_RAD) | (q > JOINT_LIMITS[:, 1] + ABORT_MARGIN_RAD)
        if self.live and self.mode in DRIVEN:
            self.max_tilt_deg = max(self.max_tilt_deg, float(np.abs(rp).max()))
            if np.abs(rp).max() > self.args.tilt_limit_deg:
                self.abort(f"tilt roll {rp[0]:+.1f} pitch {rp[1]:+.1f} deg")
            # The limits hold only while the policy drives: lying, sport mode folds the calves past the URDF's -2.72
            # (-2.78 to -2.85 rad on the dog, 2026-10-23), and the stand-up and lie-down ramp between measured poses.
            if self.mode == "policy" and out.any():
                self.abort(f"{leg_policy.UNITREE_LEGS[int(np.argmax(out))]} past its limit")
        elif not self.live and self.mode == "policy" and out.any() and not self.warned_pose:
            self.warned_pose = True
            log(f"dry: {leg_policy.UNITREE_LEGS[int(np.argmax(out))]} is past its limit (the dog is not standing?); "
                "the policy's outputs mean little here")
        t = now - self.mode_t0
        getattr(self, f"_{self.mode}")(state, t)
        self.last_tick = now

    def _operator(self, line: str) -> None:
        if line == "a":
            self.abort("operator: a")
        elif line == "q":
            self.end_reason = "operator: q"
            if self.mode == "policy":
                self.task.go_home("operator", self._foot_now()[0])
        else:
            self.typed, self.go = line, True

    # Each mode: one tick of it.

    def _preflight(self, state, t) -> None:
        if not self.live:
            log(f"DRY{' (rehearsal: a pretend foot)' if self.args.rehearse else ''}: nothing is sent to the robot")
            self._start_policy(state)
            return
        if self.pending is None and self.sport_name == "":
            self.pending, self.pending_t = self.io.switcher(ll.CHECK_MODE), time.monotonic()
            return
        if self.pending is not None:
            result = self.io.responses.pop(self.pending, None)
            if result is None:
                if time.monotonic() - self.pending_t > SWITCH_TIMEOUT_S:
                    self.event("no answer from the motion switcher (CheckMode)")
                    self.set_mode("done")
                return
            self.pending = None
            self.sport_name = ll.mode_name(result[1])
            if not self.sport_name:
                self.event("sport mode is already released: something else may own the legs; not starting")
                self.set_mode("done")
                return
            _, height = feet(state["q"], state["quat"], self.meta)
            if height < STANDING_MIN_HEIGHT_M:
                self.event(f"the base is {height:.3f} m up: stand the dog in sport mode first")
                self.set_mode("done")
                return
            self.stand_q = state["q"].copy()
            self.event("sport mode", name=self.sport_name, stand_height_m=round(height, 3),
                       stand_q=np.round(self.stand_q, 4).tolist())
            log("LIVE. The dog will lie down, sport mode will be released, and this program will stand it up and "
                f"drive its legs. Plate {self.args.plate} (contact {self.plate.contact.tolist()} m, normal "
                f"{np.round(self.plate.n, 3).tolist()}), {self.args.force_n} N for {self.args.hold_s} s (cap "
                f"{self.args.max_force_n} N). Stops: Ctrl-C, a+Enter, the remote's B. Type live + Enter (or the "
                "remote's A) to go.")
            self.go = False
            self.confirming = True
            return
        if self.confirming and self.go:
            self.go = False
            if self.typed != "live":
                log(f"{self.typed!r} is not 'live': nothing done. Type live + Enter (or the remote's A) to go, "
                    "or Ctrl-C")
                return
            self.confirming = False
            self.io.sport_request(ll.SPORT_STAND_DOWN)
            self.event("sport StandDown sent")
            self.set_mode("lie_down")

    def _lie_down(self, state, t) -> None:
        if t > 1.5 and (state["q"][2::3] < LYING_CALF_RAD).all():
            self.lie_q = state["q"].copy()
            self.event("lying", calf=np.round(self.lie_q[2::3], 3).tolist())
            self.pending, self.pending_t = self.io.switcher(ll.RELEASE_MODE), time.monotonic()
            self.release_step = "release"
            self.set_mode("release")
        elif t > LIE_WAIT_S:
            # Released while standing, the legs would pass from sport mode to this program mid-stand: not tried.
            self.abort(f"the dog did not lie down in {LIE_WAIT_S:.0f} s (calves {np.round(state['q'][2::3], 2).tolist()})")

    def _release(self, state, t) -> None:
        """ReleaseMode, then CheckMode until it reports no mode."""
        now = time.monotonic()
        result = self.io.responses.pop(self.pending, None) if self.pending is not None else None
        if self.release_step == "release" and (result is not None or now - self.pending_t > 1.5):
            self.event("ReleaseMode " + (f"answered (code {result[0]})" if result else "not answered; checking"))
            self.pending, self.pending_t, self.release_step = self.io.switcher(ll.CHECK_MODE), now, "check"
        elif self.release_step == "check":
            if result is not None and ll.mode_name(result[1]) == "":
                self.event("sport mode released")
                self.released = True
                self.io.foreign_lowcmd, self.io.listening = 0, True
                self.pending = None
                self.set_mode("check_lowcmd")
                return
            if result is not None or now - self.pending_t > 0.5:
                self.pending, self.pending_t = self.io.switcher(ll.CHECK_MODE), now
        if t > SWITCH_TIMEOUT_S:
            self.abort("the motion switcher did not report sport mode released")

    def _check_lowcmd(self, state, t) -> None:
        if t >= LOWCMD_LISTEN_S:
            self.io.listening = False
            if self.io.foreign_lowcmd:
                self.abort(f"{self.io.foreign_lowcmd} LowCmd from someone else on /lowcmd")
                return
            self.event("/lowcmd is quiet: standing up")
            self.from_q = state["q"].copy()
            self.set_mode("stand_up")

    def _stand_up(self, state, t) -> None:
        """Up to the policy's own default targets on the stand-up gains, held, the gains blended to the policy's with
        the targets still there, then held until the joints are still: the robot settles into the stance the training
        starts from (`leg_cfg.SETTLED_LEG_POS`, gravity against the default targets at kp 25), at rest.

        Until 2026-10-23 the stand-up went to sport mode's stance (`stand_q`) and the policy started as the gains
        reached 25: that stance cannot be held at kp 25, and the first live run's policy took over a robot sinking
        through a crouch it never trained from (rear feet 7 cm forward, 6 degrees nose up), asked 0.5-0.75 rad of its
        thighs and sat back within 0.24 s (F-188)."""
        kp = ll.STAND_KP * min(t / 0.5, 1.0)
        target = ll.ramp(self.from_q, self.default_q, t / STAND_S)
        if t < STAND_S + STAND_HOLD_S:
            self._send(target, kp, ll.STAND_KD)
            self._record(state, sent=True, target=target, kp=kp, kd=ll.STAND_KD)
            return
        s = (t - STAND_S - STAND_HOLD_S) / GAIN_BLEND_S
        kp, kd = ll.blend(ll.STAND_KP, self.kp, s), ll.blend(ll.STAND_KD, self.kd, s)
        self._send(self.default_q, kp, kd)
        self._record(state, sent=True, target=self.default_q, kp=kp, kd=kd)
        if s < 1.0:
            return
        if self.args.start_on_blend and not self.settle_q:
            # The policy takes the robot as the blend reaches its gains, moving or not, as F-192's simulated handover
            # starts it: held at kp 25 on the default targets alone the dog sinks on for ~1.3 s and rolls (0.303 ->
            # 0.231 m, roll +0.8 -> +6.3 deg on 2026-10-23 10:50 UTC), which the simulation's handover never did.
            self.settle_q = [state["q"].copy()] * SETTLED_TICKS
            self.still_ticks = SETTLED_TICKS
            self._gate(state, still=True, label="taken at the blend")
            return
        # Still: no joint moved more than SETTLED_MOVE_RAD over the last SETTLED_TICKS ticks. Not the velocities: at
        # rest on the dog they read 0.04-0.10 rad/s (2026-10-23, the second live run), past any useful threshold.
        self.settle_q.append(state["q"].copy())
        self.settle_q = self.settle_q[-SETTLED_TICKS:]
        still = len(self.settle_q) == SETTLED_TICKS and float(np.abs(self.settle_q[-1] - self.settle_q[0]).max()) < SETTLED_MOVE_RAD
        self.still_ticks = SETTLED_TICKS if still else 0
        waited = t - STAND_S - STAND_HOLD_S - GAIN_BLEND_S
        if self.still_ticks >= SETTLED_TICKS or waited > SETTLE_MAX_S:
            self._gate(state, still)

    def _gate(self, state, still: bool, label: str | None = None) -> None:
        """Start the policy if the stand is one it trained from (or the operator's bounds admit it); else lie down."""
        off = state["q"] - self.settled_q
        _, height = feet(state["q"], state["quat"], self.meta)
        rp = np.degrees(leg_policy.roll_pitch(state["quat"]))
        self.event(label or ("settled" if still else f"not still after {SETTLE_MAX_S} s"),
                   base_m=round(height, 3), roll_pitch_deg=np.round(rp, 1).tolist(),
                   q_minus_trained_settled=np.round(off, 3).tolist())
        gate = self.meta.get("start_gate") or {}
        tol = float(gate.get("max_off_settled_rad", SETTLED_TOL_RAD))
        pitch_lo, pitch_hi = gate.get("pitch_deg", (-SETTLED_TILT_DEG, SETTLED_TILT_DEG))
        # The operator's bounds, if given, replace the export's (round 4 has none, and F-192 has it pressing from the
        # dog's kp-25 rest, 0.20 rad and 8.1 degrees nose up, that the default refuses).
        if self.args.start_tol_rad is not None:
            tol = float(self.args.start_tol_rad)
        if self.args.start_pitch_deg is not None:
            pitch_lo, pitch_hi = (float(v) for v in self.args.start_pitch_deg)
        if (np.abs(off).max() > tol or abs(rp[0]) > SETTLED_TILT_DEG or not pitch_lo <= rp[1] <= pitch_hi
                or not still):
            # Not the policy's to take: lie down on the stand-up gains, not damping from a stand.
            self.abort_reason = (f"the stand is not the trained one (joints up to {np.abs(off).max():.2f} rad "
                                 f"off of {tol:.2f}, roll {rp[0]:+.1f} deg, pitch {rp[1]:+.1f} deg of "
                                 f"{pitch_lo:+.1f}..{pitch_hi:+.1f}, {'still' if still else 'moving'})")
            self.event(f"STOP: {self.abort_reason}; lying down")
            self.from_q, self.from_kp, self.from_kd = state["q"].copy(), self.kp, self.kd
            self.set_mode("stand_end")
            return
        self._start_policy(state)

    def _start_policy(self, state) -> None:
        self.obs.reset()
        self.task = self._new_task()
        self.policy_ticks, self.down_for, self.go = 0, 0.0, False
        self.rehearse_foot = None
        self.set_mode("policy")
        if self.args.presses == 0:
            log(f"the policy holds the stand for {self.args.stand_s:.0f} s (q + Enter ends it sooner)")
        else:
            log("the policy holds the stand. Enter (A): lift the foot to the pre-contact point")

    def _foot_now(self, state=None):
        state = state or self.state
        if self.args.rehearse and self.rehearse_foot is not None:
            return self.rehearse_foot, self.rehearse_v
        allf, height = feet(state["q"], state["quat"], self.meta)
        return allf["FR"], fr_foot_velocity(state["q"], state["dq"], state["quat"], self.meta)

    def _rehearsal_foot(self, goal) -> None:
        """The pretend foot: last step's goal, stopped by a hand 12 x 10 cm across at the plate when it comes onto it
        from in front."""
        p = np.asarray(goal, dtype=float).copy()
        last = p if self.rehearse_foot is None else self.rehearse_foot
        n, c = self.plate.n, self.plate.contact
        rel = p - c
        across = float(np.linalg.norm(rel - (rel @ n) * n))
        if across < REHEARSAL_HAND_M and float((last - c) @ n) >= 0.0 > float(rel @ n):
            p -= float(rel @ n) * n
        self.rehearse_v = (p - last) / pl.POLICY_DT
        self.rehearse_foot = p

    def _policy(self, state, t) -> None:
        task = self.task
        stage_before = task.stage
        arm_q, arm_age = (self.arm.latest() if self.arm is not None else (None, math.inf))
        measured = arm_q is not None and arm_age < ARM_STALE_S
        arm_q = arm_q if measured else self.rest
        allf, height = feet(state["q"], state["quat"], self.meta)
        if self.args.rehearse and self.rehearse_foot is None:
            self.rehearse_foot, self.rehearse_v = allf["FR"], np.zeros(3)
        foot, foot_v = self._foot_now(state)
        if self.policy_ticks == 0:
            commands = np.zeros(15)              # the reset's frame (leg_policy.waiting_command)
        else:
            # --presses 0: the stand only, the foot never lifted.
            commands = task.step(foot, foot_v, go=self.go and self.args.presses > 0)
        if task.stage != stage_before:
            self.go = False
            self._announce(task, stage_before)
        if self.args.rehearse:
            self._rehearsal_foot(task.goal_w_held)
        frame = self.obs.frame(state["quat"], state["gyro"], state["q"], state["dq"], arm_q, np.zeros(6), commands)
        stack = self.obs.step(frame)
        t0 = time.perf_counter()
        # The command goes out as soon as the action exists: the encoder runs once, and the estimate (for the record and
        # the force release) is decoded after the send (2026-10-23: before, both ran twice ahead of every command).
        latent = self.policy.encoder(np.atleast_2d(stack))
        action = self.policy.actor(np.concatenate((np.atleast_2d(stack)[:, -self.policy.num_single_obs:], latent), -1))[0]
        legs = self.obs.record_action(action)
        target = np.clip(self.obs.leg_targets_unitree(legs), JOINT_LIMITS[:, 0] + TARGET_MARGIN_RAD,
                         JOINT_LIMITS[:, 1] - TARGET_MARGIN_RAD)
        if self.mode == "policy":           # an abort this tick has already sent damping
            self._send(target, self.kp, self.kd)
        infer_ms = 1000.0 * (time.perf_counter() - t0)      # state to command out, this program's part
        self.send_ms = 1000.0 * (time.monotonic() - self.tick_now)
        estimate = self.policy.decoder(latent)[0]
        est = leg_policy.decode_estimates(self.meta, estimate)
        force_est = float(np.linalg.norm(est["foot_force"]))
        # Not in a rehearsal: the estimate is of the real robot, not of the pretend foot.
        if task.stage == pl.ENGAGED and task.layer.phase in (pl.PRESSING, pl.HOLDING) and not self.args.rehearse:
            self.force_high = self.force_high + 1 if force_est > self.args.force_abort_n else 0
            if self.force_high >= 5:
                self.event(f"estimated foot force {force_est:.1f} N past {self.args.force_abort_n} N: releasing")
                task.release_now(foot)
                self.force_high = 0
        self._record(state, sent=self.live and self.mode == "policy", target=target, kp=self.kp, kd=self.kd,
                     arm_q=arm_q, measured=measured, foot=foot, foot_v=foot_v, height=height, commands=commands,
                     frame=frame, action=legs, estimate=estimate, infer_ms=infer_ms)
        self.policy_ticks += 1
        now = time.monotonic()
        if now - self.last_print >= self.args.print_s:
            self.last_print = now
            self._print(state, foot, height, force_est, infer_ms, measured)
        if self.args.presses == 0 and t > self.args.stand_s and self.end_reason is None:
            self.end_reason = f"--stand_s {self.args.stand_s} s"
        if self.args.policy_s and t > self.args.policy_s and self.end_reason is None:
            self.end_reason = f"--policy_s {self.args.policy_s} s"
            task.go_home("time", foot)
        if task.stage == pl.WAITING and self.end_reason:
            self._end_policy(state)
        elif task.stage == pl.FOOT_DOWN:
            self.down_for += pl.POLICY_DT
            if self.down_for >= FOOT_DOWN_END_S:
                self._press_done(task)
                if self.end_reason is None and len(self.press_results) < self.args.presses:
                    # Another press from the stand: a new episode's layer; the policy's history runs on.
                    self.task, self.down_for, self.go = self._new_task(), 0.0, False
                    if self.args.rehearse:
                        self.rehearse_foot = None
                    log(f"press {len(self.press_results)} of {self.args.presses} done. Enter (A): lift the foot again")
                else:
                    self._end_policy(state)

    def _press_done(self, task) -> None:
        result = {"stalled": bool(task.layer.stalled), "ended_unpressed": task.ended_unpressed,
                  "engage_miss_m": None if math.isnan(task.engage_miss) else round(task.engage_miss, 4)}
        self.press_results.append(result)
        self.event(f"press {len(self.press_results)} finished", **result)

    def _announce(self, task, before) -> None:
        name = pl.STAGE_NAMES[task.stage]
        self.event(f"stage {pl.STAGE_NAMES[before]} -> {name}", **({"why": task.ended_unpressed}
                                                                 if task.ended_unpressed else {}))
        if task.stage == pl.AT_PRE:
            log(f"the foot is out. Put your hand there ({self.args.plate}), then Enter (A) to press "
                f"({self.args.hand_wait_s:.0f} s, then it goes home)")
        elif task.stage == pl.ENGAGED:
            log(f"pressing: in at {self.profile.speed} m/s until the foot stalls, then {self.profile.levels[0]} N")

    def _print(self, state, foot, height, force_est, infer_ms, measured) -> None:
        task = self.task
        rp = np.degrees(leg_policy.roll_pitch(state["quat"]))
        log(f"{self.mode} {pl.STAGE_NAMES[task.stage]}/{pl.PHASE_NAMES[task.layer.phase]}  level "
            f"{task.layer.normal_level:4.1f} N  est. foot force {force_est:5.1f} N  foot "
            f"{np.round(foot, 3).tolist()} (goal {np.round(task.commanded_goal(), 3).tolist()})  base {height:.3f} m  "
            f"roll {rp[0]:+5.1f} pitch {rp[1]:+5.1f}  {infer_ms:.1f} ms  arm {'measured' if measured else 'rest pose'}")

    def _end_policy(self, state) -> None:
        self.event("the policy is done", presses=len(self.press_results))
        if not self.live:
            self.set_mode("done")
            return
        self.from_q = state["q"].copy()
        self.set_mode("stand_end")

    def _stand_end(self, state, t) -> None:
        s = t / END_BLEND_S
        kp, kd = ll.blend(self.kp, ll.STAND_KP, s), ll.blend(self.kd, ll.STAND_KD, s)
        target = ll.ramp(self.from_q, self.default_q, t / END_STAND_S)
        self._send(target, kp, kd)
        self._record(state, sent=True, target=target, kp=kp, kd=kd)
        if t >= END_STAND_S + 0.5:
            self.set_mode("lie_end")

    def _lie_end(self, state, t) -> None:
        target = ll.ramp(self.default_q, self.lie_q, t / LIE_S)
        if t < LIE_S + 0.5:
            self._send(target, ll.STAND_KP, ll.STAND_KD)
            self._record(state, sent=True, target=target, kp=ll.STAND_KP, kd=ll.STAND_KD)
            return
        self._send(state["q"], 0.0, ll.DAMP_KD)
        self._record(state, sent=True, target=state["q"], kp=0.0, kd=ll.DAMP_KD)
        if t >= LIE_S + 0.5 + DAMP_S:
            self.set_mode("damping")

    def _damping(self, state, t) -> None:
        self._send(state["q"], 0.0, ll.DAMP_KD)
        self._record(state, sent=True, target=state["q"], kp=0.0, kd=ll.DAMP_KD)
        if not self.asked_handback:
            self.asked_handback = True
            self.go = False
            log(f"damping. Enter (A): give the legs back to sport mode (SelectMode {self.sport_name!r}); "
                "it may stand the dog up by itself")
        if self.go:
            self.go = False
            self.set_mode("handback")

    def _handback(self, state, t) -> None:
        """SelectMode with the mode CheckMode reported at the start, then CheckMode until it reports it again."""
        now = time.monotonic()
        if not self.handed_back:
            self.io.stop_writing()
            self.event("LowCmd stopped", sent=self.io.sent)
            self.pending = self.io.switcher(ll.SELECT_MODE, {"name": self.sport_name})
            self.pending_t, self.handed_back, self.back_step = now, True, "select"
            return
        result = self.io.responses.pop(self.pending, None) if self.pending is not None else None
        if self.back_step == "select" and (result is not None or now - self.pending_t > 1.5):
            self.event("SelectMode " + (f"answered (code {result[0]})" if result else "not answered; checking"))
            self.pending, self.pending_t, self.back_step = self.io.switcher(ll.CHECK_MODE), now, "check"
        elif self.back_step == "check":
            if result is not None and ll.mode_name(result[1]) == self.sport_name:
                self.event("sport mode back", name=self.sport_name)
                self.released = False
                self.set_mode("done")
                return
            if result is not None or now - self.pending_t > 0.5:
                self.pending, self.pending_t = self.io.switcher(ll.CHECK_MODE), now
        if t > SWITCH_TIMEOUT_S:
            self.event(f"no confirmation from the motion switcher: select sport mode from the app or remote "
                       f"(it was {self.sport_name!r})")
            self.set_mode("done")

    def _done(self, state, t) -> None:
        pass

    # --- the record --------------------------------------------------------------------------------------------------

    def _record(self, state, *, sent, target, kp, kd, arm_q=None, measured=False, foot=None, foot_v=None,
                height=math.nan, commands=None, frame=None, action=None, estimate=None, infer_ms=math.nan) -> None:
        now = self.tick_now                 # the tick's start: the period between rows, not including the step's work
        task = self.task
        nan3 = np.full(3, math.nan)
        row = {"t": now - self.start, "dt": (now - self.last_tick) if self.last_tick else 0.0,
               "state_age": now - self.state_rx, "mode": DRIVEN.index(self.mode) if self.mode in DRIVEN else -1,
               "stage": task.stage, "phase": task.layer.phase, "level": task.layer.normal_level,
               "stalled": float(task.layer.stalled), "q": state["q"], "dq": state["dq"], "tau_est": state["tau_est"],
               "quat": state["quat"], "gyro": state["gyro"], "rpy": state["rpy"], "foot_force": state["foot_force"],
               "arm_q": self.rest if arm_q is None else arm_q, "arm_measured": float(measured),
               "foot": nan3 if foot is None else foot, "foot_v": nan3 if foot_v is None else foot_v, "height": height,
               "commands": np.full(15, math.nan) if commands is None else commands,
               "frame": np.full(self.obs.num_single_obs, math.nan) if frame is None else frame,
               "action": np.full(12, math.nan) if action is None else action, "target": target,
               "kp": np.broadcast_to(np.asarray(kp, dtype=float), (12,)),
               "kd": np.broadcast_to(np.asarray(kd, dtype=float), (12,)), "sent": float(sent),
               "estimate": np.full(12, math.nan) if estimate is None else estimate, "infer_ms": infer_ms,
               "goal": task.commanded_goal(), "offset": task.offset, "go": float(self.go),
               "remote": float(sum(1 << k for k, name in enumerate(ll.KEYS) if name in state["keys"])),
               "send_ms": self.send_ms if self.mode == "policy" else math.nan}
        for key, value in row.items():
            self.rows[key].append(np.asarray(value, dtype=float))

    def flush(self) -> None:
        n = len(self.rows["t"])
        if n == self.flushed:
            return
        self.parts.mkdir(exist_ok=True)
        np.savez(self.parts / f"{self.flushed:07d}.npz",
                 **{key: np.array(values[self.flushed:n]) for key, values in self.rows.items()})
        self.flushed = n

    def save(self) -> Path:
        arrays = {key: np.array(values) for key, values in self.rows.items() if values}
        summary = self.summary(arrays)
        if arrays:
            np.savez_compressed(self.run_dir / "press.npz", **arrays)
        (self.run_dir / "summary.json").write_text(json.dumps(summary, indent=2, default=str))
        (self.run_dir / "events.json").write_text(json.dumps(self.events, indent=2, default=str))
        (self.run_dir / "run.json").write_text(json.dumps(
            {"mode": "leg_press_live" if self.live else "leg_press_dry", "status": "recorded",
             "published": ("LowCmd, motion switcher and sport requests" if self.live else "nothing"),
             "arguments": vars(self.args),
             "policy_meta": {k: self.meta[k] for k in ("variant", "checkpoint", "checkpoint_sha256", "checkpoint_iteration")},
             "summary": summary}, indent=2, default=str))
        shutil.rmtree(self.parts, ignore_errors=True)
        return self.run_dir

    def summary(self, a: dict) -> dict:
        task = self.task
        out = {"live": self.live, "rehearse": self.args.rehearse, "plate": self.args.plate,
               "plate_contact_m": self.plate.contact.tolist(), "plate_n": np.round(self.plate.n, 4).tolist(),
               "force_n": self.args.force_n, "max_force_n": self.args.max_force_n,
               "abort_reason": self.abort_reason, "end_reason": self.end_reason,
               "final_mode": self.mode, "sport_mode": self.sport_name, "handed_back": self.handed_back,
               "left_released": self.released,
               "presses": self.press_results, "last_stage": pl.STAGE_NAMES[task.stage],
               "max_tilt_deg": round(self.max_tilt_deg, 2), "lowcmd_sent": self.io.sent if self.io else 0,
               "events": len(self.events)}
        if a:
            pol = a["stage"] >= 0
            ticks = np.isfinite(a["infer_ms"])
            out.update({"steps": len(a["t"]), "seconds": round(float(a["t"][-1]), 2),
                        "policy_steps": int(ticks.sum()),
                        "step_period_s": {"median": round(float(np.median(a["dt"][1:])), 4) if len(a["dt"]) > 1 else None,
                                          "p99": round(float(np.quantile(a["dt"][1:], 0.99)), 4) if len(a["dt"]) > 1 else None},
                        "commanded_level_max_n": round(float(np.nanmax(a["level"][pol])), 3) if pol.any() else None})
            if ticks.any():
                out["infer_ms"] = {"median": round(float(np.nanmedian(a["infer_ms"])), 2),
                                   "p99": round(float(np.nanquantile(a["infer_ms"], 0.99)), 2)}
                pressing = ticks & (a["phase"] == pl.HOLDING)
                if pressing.any():
                    est = np.array([leg_policy.decode_estimates(self.meta, e)["foot_force"] for e in a["estimate"][pressing]])
                    out["holding_estimated_foot_force_n"] = {
                        "median": np.round(np.median(est, axis=0), 2).tolist(),
                        "max_norm": round(float(np.linalg.norm(est, axis=1).max()), 2)}
        return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--policy", required=True, help="a force-aware export (.npz), e.g. "
                        "checkpoints/leg_press_force_r4_model_5999.npz")
    parser.add_argument("--live", action="store_true", help="drive the legs (after a typed confirmation)")
    parser.add_argument("--rehearse", action="store_true", help="dry only: a pretend foot that follows the goal")
    parser.add_argument("--plate", choices=sorted(pl.PLATES), default="palm_up")
    parser.add_argument("--force_n", type=float, default=pl.DEFAULT_MAX_FORCE_N)
    parser.add_argument("--max_force_n", type=float, default=pl.DEFAULT_MAX_FORCE_N,
                        help="the cap on any force the layer commands (trained levels went to 35 N)")
    parser.add_argument("--ramp_s", type=float, default=0.5)
    parser.add_argument("--hold_s", type=float, default=3.0)
    parser.add_argument("--standoff", type=float, default=0.04, help="the pre-contact point off the plate (m, trained 0.03-0.05)")
    parser.add_argument("--speed", type=float, default=0.03, help="the approach along the normal (m/s, trained 0.03-0.06)")
    parser.add_argument("--hand_wait_s", type=float, default=15.0)
    parser.add_argument("--presses", type=int, default=1, help="presses in one run, each from the stand; 0: the "
                        "policy only holds the stand for --stand_s")
    parser.add_argument("--stand_s", type=float, default=10.0)
    parser.add_argument("--policy_s", type=float, default=90.0, help="the policy's longest run before the foot goes home")
    parser.add_argument("--tilt_limit_deg", type=float, default=15.0,
                        help="the stop on roll or pitch; round 4 crouches to ~30 degrees by design (F-194): 40 for it")
    parser.add_argument("--start_on_blend", action="store_true",
                        help="start the policy as the gains reach its own, not after the stand settles (F-192's handover)")
    parser.add_argument("--start_tol_rad", type=float, default=None,
                        help="the stand's largest joint offset from the trained settled stance the policy may start from "
                             "(default: the export's start_gate, else 0.20)")
    parser.add_argument("--start_pitch_deg", type=float, nargs=2, default=None, metavar=("LO", "HI"),
                        help="the stand's pitch range the policy may start from, nose up negative (default: the "
                             "export's start_gate, else -5 5)")
    parser.add_argument("--force_abort_n", type=float, default=25.0,
                        help="the decoder's foot-force estimate past this for 0.1 s releases the press")
    parser.add_argument("--cmd_hz", type=float, default=500.0, help="LowCmd's rate (the latest command repeated)")
    parser.add_argument("--topic", default="/lowstate")
    parser.add_argument("--arm_link_port", type=int, default=ARM_LINK_PORT, help="0: no arm link, the rest pose")
    parser.add_argument("--print_s", type=float, default=1.0)
    parser.add_argument("--run_dir", default="logs/leg_press")
    args = parser.parse_args()
    if args.live and args.rehearse:
        parser.error("--rehearse is for the dry run")
    if not 0.0 < args.max_force_n <= 35.0 or not 0.0 < args.force_n:
        parser.error("forces in (0, 35] N, the trained range")
    if not 0.03 <= args.standoff <= 0.05 or not 0.03 <= args.speed <= 0.06:
        parser.error("--standoff and --speed within their trained ranges")

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from unitree_go.msg import LowState

    from rclpy.signals import SignalHandlerOptions

    # rclpy's own SIGINT handler shuts the context down: the LowCmd writer then dies and no damping can be sent (the
    # first live run's Ctrl-C, 2026-10-23). Without it Ctrl-C is a KeyboardInterrupt, handled below.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stop.set())
    node = rclpy.create_node("leg_press")
    io = LiveIO(node, args.cmd_hz) if args.live else None
    run = PressRun(args, io, Operator())
    node.create_subscription(LowState, args.topic, run.on_state, qos_profile_sensor_data)
    log(f"{'LIVE' if args.live else 'DRY'}: {run.meta['variant']} policy {Path(run.meta['checkpoint']).name}, plate "
        f"{args.plate}, {args.force_n} N (cap {args.max_force_n} N); recording to {run.run_dir}")
    dt = float(run.meta["policy_dt"])
    next_tick = last_flush = time.monotonic()
    interrupted = False
    try:
        while rclpy.ok() and run.mode != "done":
            if stop.is_set() and not interrupted:
                interrupted = True
                run.abort("SIGTERM")
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.002, next_tick - time.monotonic())))
            now = time.monotonic()
            if now >= next_tick:
                try:
                    run.tick()
                except KeyboardInterrupt:
                    raise
                except Exception as error:          # any fault while the legs are ours: damping
                    run.abort(f"error: {error!r}")
                    if run.mode not in ("damping", "handback"):
                        raise
                next_tick += dt
                if now - next_tick > 5 * dt:
                    next_tick = now + dt
                if now - last_flush > FLUSH_S:
                    run.flush()
                    last_flush = now
            # SIGTERM (`docker stop`): damping sent above, then out, the motors left damping; a handback under way ends.
            if stop.is_set() and run.mode != "handback":
                break
    except KeyboardInterrupt:
        run.abort("Ctrl-C")
        if run.live and run.mode == "damping":
            log("damping. Enter (A): give the legs back to sport mode; Ctrl-C again leaves the motors damping")
            try:
                while rclpy.ok() and run.mode != "done":
                    rclpy.spin_once(node, timeout_sec=0.002)
                    if time.monotonic() >= next_tick:
                        run.tick()
                        next_tick = time.monotonic() + dt
            except KeyboardInterrupt:
                log("left in damping: LowCmd stops now and the motors keep its last command (kp 0, kd 3); "
                    f"select sport mode ({run.sport_name!r}) from the app or the remote")
    finally:
        if run.live and run.mode not in ("done",) and run.io is not None and run.io.writing and not run.handed_back:
            # Whatever happened, the last command out is damping.
            run.io.command(run.state["q"] if run.state else np.zeros(12), 0.0, ll.DAMP_KD)
            time.sleep(0.05)
        if run.io is not None:
            run.io.stop_writing()
        if run.released:
            # Left like this the remote does nothing (the first live run's Ctrl-C at the damping prompt, 2026-10-23).
            log(f"SPORT MODE IS STILL RELEASED: the remote will not respond. Give it back with d1dogsport "
                f"(demos/combiner_key/robot/sport_mode.py --select {run.sport_name or 'mcf'})")
        out = run.save()
        log(f"record: {out}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
