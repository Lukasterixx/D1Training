#!/usr/bin/env python3
"""The learned key insertion on the dog (item 1b, 2026-11-19): `press_live.py`'s whole-body run with the insertion policy
(`insert_policy.InsertRunner`, F-293) and the base's position from the legs (`base_odometry.LegOdometry`, F-296).

DRY unless `--live`, as `press_live.py`: the dry run sends nothing, and the policy runs on the dog standing in sport mode
with a virtual arm (`leg_policy.VirtualArm`, started at the solved start) in place of the D1, which stays folded. The
layer then meets no face, so a dry run checks the timing, the IK, the layer's free-air behaviour (no touch may be felt
in free air, S5's gate) and the targets: nothing about the press, the search or the seat.

    insert_live.py --policy checkpoints/insert_force_ki7_s62_9200.npz --lock X Y Z YAW_DEG        # dry

The dog's policy is `insert_force_ki7_s62_9200.npz` (F-303: trained with the strict keyway and this odometry in the loop,
4 keys damaged of 192 in simulation where `ki6_force_8200` damaged 26). `ki6`'s export still runs here.
    insert_live.py --policy ... --lock ... --live --insert_arm_send                              # live, the whole body

LIVE, with the dog standing in sport mode in front of the box and the arm folded (`press_live.py`'s chain):
  1. lie down, release sport mode, stand up on the FixStand gains (`press_live.PressRun`);
  2. the arm unfolds to the key's hand-in pose, 2 cm back along the keyway's axis from the start, jaws open; A shuts
     them on the key (`--reach_cloth`'s pattern);
  3. a joint move to the runner's solved start: the key's tip 17.5 mm short of the estimated face, on its axis;
  4. the policy: legs and arm, the layer pressing the key in by touch (`keylock.InsertLayer`, the old seated test);
  5. the layer's hold over (`InsertRunner.finished`): the jaws open, the key left in the lock, the hand backs out along
     the axis (`--back_out_m`), the fold, the lie-down, sport mode on Enter.

**The lock estimate** (`--lock X Y Z YAW_DEG`) is the keyway's mouth on the lock's face, in the frame the run keeps:
the ground under the base at the stand, x the base's heading then, z up (Z is the mouth's height above the ground);
YAW the lock axis's heading (0: straight ahead, into the lock along +x). The bar lies level, to the robot's right, as
training holds it. Where it comes from is the operator's (the door program's look, a tape measure): the runner only
refuses one outside what the policy trained on (`TRAINED_*`, from `leg_cfg.KEYHOLD_*`). It is read at the stand and held:
the box does not move, the base does, and the base is what the odometry follows.
How far off it may be (F-305, simulation, `ki7_s62_9200` on the strict keyway with the leg odometry; `ki6_force_8200`'s
F-302): about 5 mm across the keyway, 12 mm along it and 3 deg in roll about it. Past 3 deg of roll the seats go (4.5 deg:
94 of 192 against 147); about 1.5 deg seats more than 3, an exact roll fewer. The key's roll in the jaws counts toward the 3 deg.

**The base** is the leg odometry from the policy's first step, the IMU's attitude with its heading at the stand taken
off. Every foot is taken as planted unless `--contact_raw` gives a `LowState.foot_force` threshold: in simulation no foot
leaves the ground after the stance settles (F-296), and the dog's foot sensors are not calibrated to newtons.
A policy can learn to lean in on sliding feet that this odometry cannot see (F-301, `ki7`'s late checkpoints): the layer
then reads the hand ahead of its reference and never feels the touch. Watch the feet in the first runs.

Stops: `press_live`'s (the hard tilt, a joint past its limit, a stale LowState, Ctrl-C, `a` + Enter, the remote's B) and
the reach's whole-body ones (`--reach_soft_tilt_deg`, a leg target past `--reach_max_leg_off_rad`, `reach_arm.ArmWatch`),
which hold the arm and lie the dog down; and, the same, the layer's press past `--max_press_n` and a touch felt more than
`--air_touch_mm` short of the estimated face (an arm that cannot track its reference looks like contact, F-298).

`ArmWatch` keeps the reach's thresholds. In simulation `ki6_force_8200`'s goals lead the link's report by at most 6.8 deg in
contact (Joint2), and no episode of 192 would stop (`check_armwatch.py`, the week 10 log). The real arm's lead while
pressing is not measured.

Not done (the week 10 log, item 1b): the lock from the head camera's tag after the stand-up; a run on the dog.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from demos.combiner_key.robot import base_odometry, insert_policy, leg_policy, press_live, reach_arm  # noqa: E402
from demos.combiner_key.robot.press_live import JOINT_LIMITS, TARGET_MARGIN_RAD, log  # noqa: E402

#: What the policy trained on (`unifp_train/leg_cfg.py`'s `KEYHOLD_*`, the insertion's box): the door's distance ahead of
#: the base, the keyway's side (negative: the robot's right), its height over the ground, the door's yaw. The mouth is
#: `KEY_ADAPTER` proud of the door, a few millimetres, inside the tolerance.
TRAINED_AHEAD_M = (0.495, 0.525)
TRAINED_SIDE_M = (-0.125, -0.095)
TRAINED_HEIGHT_M = (0.442, 0.458)
TRAINED_YAW_RAD = (-0.05, 0.05)
#: How far past those an estimate may be and still run (m, rad): about the estimate's own training errors.
RANGE_TOL_M, RANGE_TOL_RAD = 0.005, math.radians(1.0)
#: The key's hand-in pose: this far back along the axis from the start, the jaws open (about 7 cm off the box's face). The
#: level hand runs Joint5 into its lower limit not far behind: from the stand-up's stand (0.30 m) 3 cm back does not
#: solve, from the policy's settled one (0.27 m) 6 cm. Any further off, a joint move into the start could swing the key into
#: the box; this one is a short straight approach.
HAND_IN_BACK_M = 0.02
#: The evaluation's start and press (`keyinsert_eval.build_plan`: 17.5 mm short of the face, `keylock.EVAL_PRESS_N`).
START_DEPTH_M = -0.0175
PRESS_N = 8.0
#: After the hold: the jaws open this long before the hand backs out.
RELEASE_S = 1.0
#: The least backing out worth a move before the fold (m).
MIN_BACK_OUT_M = 0.01
#: Corrective moves to the start solved from the settled stand before the policy, at most; and the stand held this long
#: after each (s) so the legs settle under the arm's new load.
MAX_RESTARTS, ARM_SETTLE_S = 2, 0.5


def heading(quat_wxyz) -> float:
    r = base_odometry.quat_matrix(quat_wxyz)
    return math.atan2(r[1, 0], r[0, 0])


def without_heading(quat_wxyz, yaw0: float) -> np.ndarray:
    """The IMU's attitude in the run's frame: its heading at the stand taken off."""
    c, s = math.cos(-yaw0 / 2), math.sin(-yaw0 / 2)
    w, x, y, z = (float(v) for v in quat_wxyz)
    # (c, 0, 0, s) * q
    return np.array([c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w])


def air_touch(touch_depth_m: float, limit_mm: float) -> bool:
    """A touch felt this far short of the estimated face (the layer's `touch_depth`, negative short of it) is in free air:
    training's estimate is off by at most 8 mm along the axis, and a shoulder that cannot track looks like contact (F-298)."""
    return touch_depth_m < -limit_mm / 1000.0


def lock_estimate(x: float, y: float, z: float, yaw_deg: float) -> insert_policy.LockEstimate:
    """The lock as `--lock` gives it: its axis into the lock at `yaw_deg`, the bar level to the robot's right."""
    yaw = math.radians(yaw_deg)
    axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    u = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
    return insert_policy.LockEstimate(np.array([x, y, z], dtype=float), axis, u)


def outside_training(x: float, y: float, z: float, yaw_deg: float) -> list[str]:
    """What of a lock estimate is outside what the policy trained on (empty: inside)."""
    out = []
    for name, value, (lo, hi), tol in (("ahead", x, TRAINED_AHEAD_M, RANGE_TOL_M), ("side", y, TRAINED_SIDE_M, RANGE_TOL_M),
                                       ("height", z, TRAINED_HEIGHT_M, RANGE_TOL_M),
                                       ("yaw", math.radians(yaw_deg), TRAINED_YAW_RAD, RANGE_TOL_RAD)):
        if not lo - tol <= value <= hi + tol:
            out.append(f"{name} {value:.3f} outside {lo:.3f}..{hi:.3f}")
    return out


class InsertRun(press_live.PressRun):
    """`press_live.PressRun` with the insertion's arm, policy and end."""

    def __init__(self, args, io, operator, arm_link=None):
        super().__init__(args, io, operator, arm_link)
        if self.meta.get("task") != "insert":
            raise SystemExit("an insertion export is needed (meta task 'insert')")
        self.runner = insert_policy.InsertRunner(self.meta)
        x, y, z, yaw_deg = (float(v) for v in args.lock)
        why = outside_training(x, y, z, yaw_deg)
        if why and not (args.lock_outside_training and not args.live):
            raise SystemExit("--lock outside what the policy trained on: " + "; ".join(why)
                             + (" (--lock_outside_training is for a dry run only)" if args.live else ""))
        self.lock_est = lock_estimate(x, y, z, yaw_deg)
        self.arm_send = bool(args.insert_arm_send)
        if self.arm_send and not (self.live and args.live):
            raise SystemExit("--insert_arm_send needs --live: with the legs dry the policy runs away (F-252)")
        if self.live and not self.arm_send:
            raise SystemExit("--live needs --insert_arm_send: the insertion is the whole body, never the legs alone")
        if self.arm_send:
            if self.arm is None:
                raise SystemExit("--insert_arm_send needs the arm link")
            if (self.arm.hello or {}).get("observe"):
                raise SystemExit("the arm link is observing: it takes no goal")
            self.arm_reports = reach_arm.ArmReports(self.obs.default[12:18])
            self.arm_watch = reach_arm.ArmWatch()
            self.arm_sender = reach_arm.ArmSender(self.arm.sock)
        self.virtual = None if self.arm_send else leg_policy.VirtualArm(self.meta)
        self.odo = base_odometry.LegOdometry(self.meta)
        self.yaw0 = 0.0
        self.base0 = np.zeros(3)
        self.q_start = self.q_hand_in = None
        #: The key's part: folded, hand_in (jaws open, waiting for A), closing, to_start, ready; at the end release, back_out.
        self.key_phase = "folded"
        self.key_t0 = None
        self.restarts = 0
        self.press_peak = 0.0
        self.layer_phases: list[str] = []
        stem = self.run_dir.name.replace("leg_press", "key_insert")
        new = self.run_dir.with_name(stem)
        self.run_dir.rename(new)
        self.run_dir, self.parts = new, new / "parts"
        for key in ("ins_phase", "press_n", "base_est", "ins_arm_target"):
            self.rows[key] = []

    # --- the start ------------------------------------------------------------------------------------------------------

    def _stand_frame(self, state) -> tuple[np.ndarray, np.ndarray]:
        """The base's pose in the run's frame now: over the origin at its height, the IMU's attitude less the heading
        taken at the stand (the first call)."""
        if self.q_start is None:
            self.yaw0 = heading(state["quat"])
        _, height = press_live.feet(state["q"], state["quat"], self.meta)
        return np.array([0.0, 0.0, height]), without_heading(state["quat"], self.yaw0)

    def _solve_start(self, state) -> bool:
        """The runner's start from the base's pose now, and the hand-in pose back along the axis; False if unsolved."""
        self.base0, quat = self._stand_frame(state)
        q, ok = self.runner.reset(self.lock_est, self.base0, quat, START_DEPTH_M, PRESS_N)
        if not ok:
            self.abort("the IK has no start pose for this lock from this stand")
            return False
        self.q_start = q
        back, went = self._solve_along(self.runner.goal_jaw, q, -HAND_IN_BACK_M)
        if abs(went) < HAND_IN_BACK_M - 1e-9:
            self.abort("the IK has no hand-in pose back along the axis")
            return False
        self.q_hand_in = back
        self.event("start solved", q_start=np.round(q, 3).tolist(), q_hand_in=np.round(back, 3).tolist(),
                   base_height_m=round(float(self.base0[2]), 3))
        return True

    def _solve_along(self, jaw, seed, metres: float, step_m: float = 0.005) -> tuple[np.ndarray, float]:
        """The arm's pose with the jaw centre up to `metres` along the lock's axis from `jaw` (negative: back out), the
        hand as it inserts, solved `step_m` at a time from `seed` (a long jump does not converge): the furthest solved,
        and how far that is (m, signed)."""
        q, n = np.asarray(seed, dtype=float), max(1, int(math.ceil(abs(metres) / step_m - 1e-9)))
        done = 0.0
        for k in range(1, n + 1):
            qk, ok = self.runner._solve(jaw + (metres * k / n) * self.lock_est.axis, q)
            if not ok:
                break
            q, done = qk, metres * k / n
        return q, done

    def _unfolded(self) -> bool:
        """With the legs on the stand-up gains: out to the hand-in pose, the key shut in the jaws on A, then to the start.
        True once the arm is at the start."""
        self._read_arm()
        now = time.monotonic()
        if self.key_phase == "folded":
            if self.arm_reports.stamp is None:
                self.abort("arm: no report before the unfold")
                return False
            if not self._solve_start(self.state):
                return False
            self.arm_sender.gripper = reach_arm.JAWS_OPEN_UNITS
            self._joint_move("unfolding", reach_arm.HOME_SERVO_DEG, reach_arm.to_servo_deg(self.q_hand_in))
            self.key_phase = "unfolding"
        self.arm_sender.keepalive()
        if self.key_phase == "unfolding":
            if reach_arm.at_start(self.arm_reports.q, self.q_hand_in, reach_arm.ARRIVED_DEG) is None:
                self.key_phase, self.key_t0, self.go = "hand_in", now, False
                self.event("the jaws are open at the hand-in pose: put the key in, its bar across them; A shuts them")
            elif now - self.arm_move_t0 > reach_arm.JOINT_MOVE_TIMEOUT_S:
                self.abort("arm: not at the hand-in pose in time")
            return False
        if self.key_phase == "hand_in":
            if self.end_reason is not None or now - self.key_t0 > reach_arm.CLOTH_WAIT_S:
                # Nothing is wrong with the dog: fold and lie down the normal way (the jaws open, the hand back).
                self._arm_stop(f"no key ({self.end_reason or f'no A in {reach_arm.CLOTH_WAIT_S:.0f} s'})")
                return False
            if self.go:
                self.go, self.key_phase, self.key_t0 = False, "closing", now
                self.event("the jaws closing on the key")
            return False
        if self.key_phase == "closing":
            self.arm_sender.gripper = reach_arm.jaws_closing(now - self.key_t0)
            if now - self.key_t0 >= reach_arm.CLOTH_CLOSE_S + reach_arm.CLOTH_LET_GO_S:
                self.key_phase = "to_start"
                self._joint_move("to the start", reach_arm.to_servo_deg(self.q_start))
            return False
        if self.key_phase == "to_start":
            if reach_arm.at_start(self.arm_reports.q, self.q_start, reach_arm.ARRIVED_DEG) is None:
                self.key_phase, self.arm_phase = "ready", "ready"
                self.event("arm: at the insertion's start")
                return True
            if now - self.arm_move_t0 > reach_arm.JOINT_MOVE_TIMEOUT_S:
                self.abort("arm: not at the insertion's start in time")
            return False
        return self.key_phase == "ready"

    def _start_policy(self, state) -> None:
        # The runner from the base's pose now: the stand sinks from the stand-up's (0.30 m) to the policy's settled one
        # (0.27 m) after the arm went to its start, and every training episode starts with the arm at the start solved
        # from the stand it has. A short joint move there first, the legs held on the policy's gains (`_arm_settle`).
        base, quat = self._stand_frame(state)
        q, ok = self.runner.reset(self.lock_est, base, quat, START_DEPTH_M, PRESS_N)
        if not ok:
            self.abort("the IK has no start pose for this lock from the policy's stand")
            return
        if self.arm_send:
            self._read_arm()
            if reach_arm.at_start(self.arm_reports.q, q, reach_arm.ARRIVED_DEG) is not None:
                if self.restarts >= MAX_RESTARTS:
                    self.abort(f"arm: the start still moves with the stand after {MAX_RESTARTS} corrections")
                    return
                self.restarts += 1
                self.q_start = q
                self._joint_move(f"to the start from the settled stand ({self.restarts})", reach_arm.to_servo_deg(q))
                self.set_mode("arm_settle")
                return
        super()._start_policy(state)
        self.q_start = q
        self.base0 = base
        self.odo.reset(base, quat, state["q"], np.ones(4, dtype=bool))
        if self.virtual is not None:
            self.virtual.reset()
            self.virtual.q = q.copy()
        log(f"the insertion: the lock {np.round(self.lock_est.origin, 3).tolist()} m, the press {PRESS_N} N, the runner "
            f"ends {self.runner.env.END_AFTER_DONE_S} s after the layer's hold (--policy_s {self.args.policy_s} s at most)")

    def _arm_settle(self, state, t) -> None:
        """The legs held at the policy's default targets on its gains while the arm moves to the start solved from the
        stand they settled into; then the policy (whose start re-solves once more)."""
        self._send(self.default_q, self.kp, self.kd)
        self._record(state, sent=True, target=self.default_q, kp=self.kp, kd=self.kd)
        self._read_arm()
        self.arm_sender.keepalive()
        if reach_arm.at_start(self.arm_reports.q, self.q_start, reach_arm.ARRIVED_DEG) is None and t > ARM_SETTLE_S:
            self._start_policy(state)
        elif time.monotonic() - self.arm_move_t0 > reach_arm.JOINT_MOVE_TIMEOUT_S:
            self.abort("arm: not at the start from the settled stand in time")

    # --- the policy -----------------------------------------------------------------------------------------------------

    def _contact(self, state) -> np.ndarray:
        if self.args.contact_raw is None:
            return np.ones(4, dtype=bool)
        return np.asarray(state["foot_force"], dtype=float) > float(self.args.contact_raw)

    def _policy(self, state, t) -> None:
        quat = without_heading(state["quat"], self.yaw0)
        base = self.base0 if self.policy_ticks == 0 else self.odo.step(quat, state["q"], self._contact(state))
        if self.arm_send:
            self._read_arm()
            arm_q, arm_dq = self.arm_reports.q, self.arm_reports.dq
        else:
            arm_q, arm_dq = self.virtual.q, self.virtual.dq
        t0 = time.perf_counter()
        out = self.runner.step(self.policy, quat, state["gyro"], state["q"], state["dq"], arm_q, arm_dq, base, quat)
        target = np.clip(out["legs"], JOINT_LIMITS[:, 0] + TARGET_MARGIN_RAD, JOINT_LIMITS[:, 1] - TARGET_MARGIN_RAD)
        legs_off = float(np.abs(out["action"][:12]).max()) * float(self.meta["action_scale"])
        bound = self.args.reach_max_leg_off_rad
        if self.policy_ticks * self.args_dt() < reach_arm.GRACE_S:
            bound = max(bound, reach_arm.LEG_OFF_GRACE_RAD)
        if legs_off > bound:
            self._stop(f"leg target {legs_off:.2f} rad off default, past {bound} rad")
        elif out["press_n"] > self.args.max_press_n:
            self._stop(f"the layer's press {out['press_n']:.1f} N past --max_press_n {self.args.max_press_n}")
        if self.mode == "policy":
            self._send(target, self.kp, self.kd)
        infer_ms = 1000.0 * (time.perf_counter() - t0)
        if self.mode == "policy":
            if self.arm_send:
                self._send_arm_target(out["arm"])
            else:
                self.virtual.step(out["arm"])
        self.send_ms = 1000.0 * (time.monotonic() - self.tick_now)
        self.press_peak = max(self.press_peak, float(out["press_n"]))
        if not self.layer_phases or self.layer_phases[-1] != out["phase"]:
            before = self.layer_phases[-1] if self.layer_phases else None
            self.layer_phases.append(out["phase"])
            touch = float(self.runner.layer.touch_depth[0])
            self.event(f"layer: {out['phase']}", press_n=round(float(out["press_n"]), 2), touch_depth_mm=round(1000 * touch, 1))
            if before == "approach" and out["phase"] == "search" and self.mode == "policy" and air_touch(touch, self.args.air_touch_mm):
                self._stop(f"a touch felt {-1000 * touch:.0f} mm short of the estimated face: the arm is not tracking its "
                           f"reference (F-298) or the lock estimate is off")
        self._record(state, sent=self.live and self.mode == "policy", target=target, kp=self.kp, kd=self.kd,
                     arm_q=arm_q, measured=self.arm_send, frame=self.runner.last_frame, action=out["action"][:12],
                     infer_ms=infer_ms, arm_obs=arm_q, arm_target=out["arm"])
        for key, value in (("ins_phase", insert_policy.PHASE_NAMES.index(out["phase"])), ("press_n", out["press_n"]),
                           ("base_est", base), ("ins_arm_target", out["arm"])):
            self.rows[key].append(np.asarray(value, dtype=float))
        self.policy_ticks += 1
        if self.mode != "policy":
            return
        if out["finished"] and self.end_reason is None:
            self.end_reason = "the layer's hold is over"
        if t > self.args.policy_s and self.end_reason is None:
            self.end_reason = f"--policy_s {self.args.policy_s} s"
        if self.end_reason is not None:
            self._end_policy(state)

    def _send_arm_target(self, target) -> None:
        """The runner's six arm targets (URDF rad) to the link as a goal, through `ArmWatch`, or the reason not to."""
        if not self.arm_started:
            if self.arm_reports.stamp is None:
                self.abort("no report from the arm")
                return
            why = reach_arm.at_start(self.arm_reports.q, self.q_start)
            if why:
                self.abort(why)
                return
            self.arm_started = True
            self.arm_watch.begin(time.monotonic())
            self.event("the arm's goals are going to the link")
        why = self.arm_watch.check(time.monotonic(), self.arm_reports, target)
        if why:
            self._stop(f"arm: {why}")
            return
        self.arm_sender.follow(target)

    def _stop(self, why: str) -> None:
        """A whole-body stop: the arm held, the legs to the stand-up gains, then the key let go, the hand out, fold, lie."""
        if self.arm_send:
            self._arm_stop(why)
        else:
            self.abort(why)

    # --- the end --------------------------------------------------------------------------------------------------------

    def _folded(self) -> bool:
        """Before the fold: the jaws open on the key (left in the lock) and the hand backs out along the axis."""
        if self.arm_sender is None or self.arm_phase in ("folded", "folded_again"):
            return True
        now = time.monotonic()
        self._read_arm()
        if self.key_phase in ("ready", "hand_in", "closing", "to_start", "unfolding"):
            self.arm_sender.gripper = reach_arm.JAWS_OPEN_UNITS
            self.key_phase, self.key_t0 = "release", now
            self.event("the jaws open: the key stays where it is (in the lock if the layer seated it)")
        self.arm_sender.keepalive()
        if self.key_phase == "release":
            if now - self.key_t0 < RELEASE_S:
                return False
            q_now = np.asarray(self.arm_reports.q, dtype=float)
            out, went = self._solve_along(self._jaw_now(), q_now, -self.args.back_out_m)
            if abs(went) >= MIN_BACK_OUT_M:
                self.q_out = out
                self._joint_move("backing out", reach_arm.to_servo_deg(out))
                self.key_phase = "back_out"
                self.event(f"backing out {abs(went) * 1000:.0f} mm along the axis"
                           + ("" if abs(went) >= self.args.back_out_m - 1e-9 else f" (the IK's furthest of {self.args.back_out_m * 1000:.0f})"))
            else:
                self.event("no IK to back the hand out along the axis: folding from here")
                self.key_phase = "out"
            return False
        if self.key_phase == "back_out":
            if reach_arm.at_start(self.arm_reports.q, self.q_out, reach_arm.ARRIVED_DEG) is None:
                self.key_phase = "out"
                self.event("the hand is out along the axis")
            elif now - self.arm_move_t0 > reach_arm.JOINT_MOVE_TIMEOUT_S:
                self.event("not out along the axis in time: folding from here")
                self.key_phase = "out"
            return False
        return super()._folded()

    def _jaw_now(self) -> np.ndarray:
        """The jaw centre in the run's frame by the arm's last report and the base's estimate (the runner's goal before
        any report)."""
        if self.runner._arm_measure is None:
            return self.runner.goal_jaw
        return self.runner._measure()[0]

    def _end_policy(self, state) -> None:
        self.event("the policy is done", reason=self.end_reason, layer=self.layer_phases)
        if not self.live:
            self.set_mode("done")
            return
        self.from_q = state["q"].copy()
        self.set_mode("stand_end")

    # --- the record -----------------------------------------------------------------------------------------------------

    def summary(self, a: dict) -> dict:
        out = super().summary(a)
        out["task"] = "insert"
        out["insert"] = {"lock": [float(v) for v in self.args.lock], "press_n": PRESS_N, "start_depth_m": START_DEPTH_M,
                         "layer_phases": self.layer_phases, "press_peak_n": round(self.press_peak, 2),
                         "runner_events": self.runner.events[-50:], "arm_sent": self.arm_send,
                         "contact": "all planted" if self.args.contact_raw is None else f"foot_force > {self.args.contact_raw}"}
        if self.rows["base_est"]:
            est = np.array(self.rows["base_est"])
            out["insert"]["base_moved_m"] = np.round(np.abs(est - est[0]).max(axis=0), 4).tolist()
        return out


def make_parser() -> argparse.ArgumentParser:
    parser = press_live.make_parser()
    parser.description = __doc__.split("\n\n")[0]
    parser.add_argument("--lock", type=float, nargs=4, required=True, metavar=("X", "Y", "Z", "YAW_DEG"),
                        help="the keyway's mouth in the run's frame (the ground under the base at the stand, x its heading, "
                             "z up) and the lock axis's heading; refused outside what the policy trained on")
    parser.add_argument("--lock_outside_training", action="store_true", help="dry only: run a lock outside the trained range")
    parser.add_argument("--insert_arm_send", action="store_true",
                        help="with --live: the arm unfolds, takes the key and the policy drives it (after a typed confirmation)")
    parser.add_argument("--max_press_n", type=float, default=20.0,
                        help="the layer's press past this stops the run (keylock.PRESS_LIMIT_N, 20; the key breaks near 50)")
    parser.add_argument("--contact_raw", type=float, default=None,
                        help="LowState.foot_force above this is a foot in contact (default: every foot planted)")
    parser.add_argument("--back_out_m", type=float, default=0.04, help="how far the hand backs out along the axis at the end")
    parser.add_argument("--air_touch_mm", type=float, default=15.0,
                        help="a touch felt further than this short of the estimated face stops the run (F-292, F-298)")
    parser.set_defaults(presses=0, policy_s=30.0, run_dir="logs/key_insert")
    return parser


def main() -> int:
    parser = make_parser()
    args = parser.parse_args()
    if args.rehearse:
        parser.error("--rehearse is the foot press's")
    import rclpy  # noqa: F401  (the ROS side is press_live's)

    if args.insert_arm_send:
        log("--insert_arm_send: the learned insertion on the whole body, for the first time: the D1 unfolds, takes the key and "
            "the policy drives it and the legs into the box. Nobody within the arm's reach, a hand on the E-stop.")
        try:
            if input("type 'key' to go on: ").strip() != "key":
                log("not sent")
                return 1
        except EOFError:
            log("no terminal to confirm on: not sent")
            return 1

    # press_live.main builds its run from `PressRun`: swap ours in for this process.
    press_live.PressRun = InsertRun                     # type: ignore[misc]
    press_live.make_parser = make_parser                # type: ignore[assignment]
    return press_live.main()


if __name__ == "__main__":
    raise SystemExit(main())
