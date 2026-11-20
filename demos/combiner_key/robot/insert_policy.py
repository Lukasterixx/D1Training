"""The key insertion's whole-body policy without torch, for the dog: the task layer, the arm's reference and the frame.

`export_policy.py --task insert` writes an insertion checkpoint (`ki6_force_8200`: force-aware, the v4 key, no tool input)
into one `.npz` with every constant this module needs, read from the training code (`unifp_train.keylock`,
`keyinsert_env`, `arm_kin`, `leg_cfg`), so the dog's side has none of its own. The network is `leg_policy.LegPolicy`'s
(the same encoder / actor / decoder, 18 outputs); the observation's history is `leg_policy.LegObservation`'s. What is
new here is what `keyinsert_env.Go2D1KeyInsertEnv` does around the network each 50 Hz step, in its order:

  1. `update` (every step but a reset's): the key's tip *measured* by the forward kinematics of the arm's reported
     angles -- the report the previous frame showed, as the environment measures before it reads the new one -- placed
     by the base's pose against the *estimated* lock; the task layer (`InsertLayer`, both of `keylock`'s seat tests)
     steps on that depth and lateral; its goal for the tip becomes a goal for the jaw centre (the hand on the estimated
     axis); the classical outer loop integrates the measured miss into the IK's goal; every 5th step the arm's
     reference is re-solved by IK (`ArmKinematics.solve_pose`: `arm_kin.track` then two damped least-squares polishes)
     and sag-compensated if the policy trained with sag (`ki6` did not: the stiff servo, `--arm_locked`); the actor's
     miss is the goal less the measured jaw centre in the base's frame.
  2. `frame`: the 76-wide frame -- legs and IMU as `LegObservation` builds them, the arm's reported angles and their
     report-difference rates, the commands: the foot's goal on its footfall, the miss x 20 (x 0.5 in the frame), the
     press along the estimated axis in the base's yaw frame (force-aware only). A reset's frame has all-zero commands.
  3. The policy acts on the stack (`LegPolicy.act`).
  4. `targets`: the 12 leg targets (default + 0.25 a, Unitree's order); the arm's six: the reference, the layer's roll
     on Joint6, plus 0.25 x the arm's residual with a[5] zeroed, inside 0.97 of the D1's hard range; and the press for
     the next frame. The next frame's previous actions are the masked vector.

Every value the simulator supplies is an explicit input: the estimated lock frame (`LockEstimate`), the believed key
(`reach`, `offset`), the start depth, the press, the base's pose in the lock's frame (`base_pos`, `base_quat`: any
gravity-aligned frame, z up, in which the lock estimate is also given), the IMU's quaternion and rate, the legs, the
arm's reports. On the dog those come from perception, odometry and the D1's link; here from `check_insert.py`'s replay.

The layer's state is float32 and computed in float32, as the environment's torch tensors are: its timers count 0.02 s
steps, and a float64 sum reaches 0.3 s one step before a float32 one does (15 against 16 steps), which would move every
decision a step. The geometry is float64. Checked by `tests/test_insert_policy.py` (the layer against `keylock`'s on
scripted depths, the kinematics against `arm_kin`'s) and `check_insert.py --sim` (a recorded Isaac rollout).

**One difference from the batched layer, by construction.** `keylock.InsertLayer` updates some timers inside
`if any env is searching / seating:` blocks, unmasked, so in a batch one environment's `held_for` is zeroed while
another searches. Only the located test's relocation path (back to APPROACH without clearing `held_for`) can see it;
the old test cannot. This layer is batched the same way, so with one robot it is the one-environment behaviour.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np

from demos.combiner_key.robot import leg_policy

F32 = np.float32

APPROACH, SEARCH, SEAT, HOLD, DONE = 0, 1, 2, 3, 4
PHASE_NAMES = ("approach", "search", "seat", "hold", "done")

#: `keylock`'s names for the layer's constants; the export copies each from `unifp_train.keylock`.
LAYER_CONSTANTS = (
    "FAST_SPEED_M_S", "FAST_UNTIL_M", "SLOW_SPEED_M_S", "SLACK_M", "CONTACT_LAG_M", "CONTACT_HOLD_S", "BASE_S",
    "LAG_MAX_M", "PRESS_M", "SPIRAL_PITCH_M", "SPIRAL_SPEED_M_S", "SPIRAL_R_M", "WIGGLE_DEG", "WIGGLE_PERIOD_S",
    "ENTER_M", "HELD_SIDEWAYS_M", "PROBE_S", "PROBE_M", "PROBE_HELD_M", "SEATED_SHORT_M", "STILL_ANY_S", "JIGGLE_DEG",
    "JIGGLE_PERIOD_S", "JIGGLE_S", "JIGGLE_HELD_S", "SETTLE_SPEED_M_S", "SETTLE_FILTER_S", "HOLD_S", "HOLD_PRESS_N",
    "K_AXIAL_N_PER_M", "K_LATERAL_N_PER_M", "FACE_ROLL_DEG", "ROLL_RATE_DEG_S", "LOCATE_BACK_M", "RETRIES",
    "RELOCATE_M", "RELOCATE_WAIT_S", "ROLL_SETTLE_S", "STRAIGHT_M", "KEYWAY_DEPTH_M", "APPROACH", "SEARCH", "SEAT",
    "HOLD", "DONE")
SEAT_TESTS = ("old", "located")


# --- the task layer -------------------------------------------------------------------------------------------------

class InsertLayer:
    """`keylock.InsertLayer` in numpy, line for line, for `num_envs` robots (one on the dog): the tip's reference in the
    estimated lock's coordinates (depth along the axis from the estimated face, and across it), the wrist's roll and
    the press. `constants` are `LAYER_CONSTANTS` by name (the export's `insert.layer.constants`)."""

    def __init__(self, num_envs: int, constants: dict, seat_test: str = "old", light_probe: bool = False):
        missing = [k for k in LAYER_CONSTANTS if k not in constants]
        if missing:
            raise ValueError(f"the layer's constants lack {missing}")
        for name, value in ((n, globals()[n]) for n in ("APPROACH", "SEARCH", "SEAT", "HOLD", "DONE")):
            if int(constants[name]) != value:
                raise ValueError(f"phase {name} is {constants[name]} in the export, {value} here")
        if seat_test not in SEAT_TESTS:
            raise ValueError(f"seat_test must be one of {SEAT_TESTS}, not {seat_test!r}")
        self.c = SimpleNamespace(**{k: constants[k] for k in LAYER_CONSTANTS})
        self.n = n = int(num_envs)
        self.light_probe = bool(light_probe)
        self.seat_test = seat_test
        z = lambda *shape: np.zeros(shape, F32)
        self.phase = np.full(n, DONE, np.int64)
        self.force_law = np.zeros(n, bool)
        self.ref_depth, self.ref_lateral = z(n), z(n, 2)
        self.touch_depth = z(n)
        self.press_n, self.force_cmd = z(n), z(n)
        self.roll = z(n)
        self.spiral_s, self.spiral_dir = z(n), np.ones(n, F32)
        self.wiggle_t = z(n)
        self.lag_for, self.held_for, self.still_for, self.phase_for = z(n), z(n), z(n), z(n)
        self.last_depth = z(n)
        self.seat_end = z(n)
        self.still_any = z(n)
        self.jiggle_for, self.entry_roll = z(n), z(n)
        self.probe_from = z(n, 2)
        self.lag_base = z(n)
        self.touched = np.zeros(n, bool)
        self.entered_by = z(n)
        self.max_depth = np.full(n, self.c.KEYWAY_DEPTH_M + self.c.SLACK_M, F32)
        self.locate_stage = np.zeros(n, np.int64)
        self.face_depth = z(n)
        self.retries, self.relocates = z(n), z(n)
        self.refused = np.zeros(n, bool)
        self.roll_wait = z(n)
        self.settle_for = z(n)

    def start(self, ids, start_depth, force_law, press_n) -> None:
        self.phase[ids] = APPROACH
        self.force_law[ids] = force_law
        self.ref_depth[ids], self.ref_lateral[ids] = start_depth, 0.0
        self.touch_depth[ids] = 0.0
        self.press_n[ids] = press_n
        self.force_cmd[ids] = 0.0
        self.roll[ids] = 0.0
        self.spiral_s[ids], self.spiral_dir[ids], self.wiggle_t[ids] = 0.0, 1.0, 0.0
        for buf in (self.lag_for, self.held_for, self.still_for, self.phase_for):
            buf[ids] = 0.0
        self.last_depth[ids] = start_depth
        self.seat_end[ids] = 0.0
        self.still_any[ids] = 0.0
        self.jiggle_for[ids], self.entry_roll[ids] = 0.0, 0.0
        self.probe_from[ids] = 0.0
        self.lag_base[ids] = 0.0
        self.touched[ids] = False
        self.entered_by[ids] = 0.0
        self.locate_stage[ids] = 0 if self.seat_test == "located" else 2
        self.face_depth[ids] = 0.0
        self.retries[ids], self.relocates[ids] = 0.0, 0.0
        self.refused[ids] = False
        self.roll_wait[ids] = 0.0
        self.settle_for[ids] = 0.0

    def stop(self, ids) -> None:
        self.phase[ids] = DONE
        self.force_cmd[ids] = 0.0

    def free_lead(self) -> np.ndarray:
        return self.force_cmd / self.c.K_AXIAL_N_PER_M

    def goal(self) -> tuple[np.ndarray, np.ndarray]:
        return self.ref_depth - self.free_lead(), self.ref_lateral

    def _spiral(self, s: np.ndarray) -> np.ndarray:
        b = self.c.SPIRAL_PITCH_M / (2 * math.pi)
        theta = np.sqrt(2.0 * np.maximum(s, F32(0.0)) / b)
        r = b * theta
        return np.stack((r * np.cos(theta), r * np.sin(theta)), -1)

    def _enter_seat(self, entered, depth, lateral, by: float) -> None:
        c = self.c
        self.entered_by = np.where(entered, np.full_like(self.entered_by, by), self.entered_by)
        self.ref_lateral = np.where(entered[:, None], lateral, self.ref_lateral)
        self.phase = np.where(entered, SEAT, self.phase)
        self.phase_for = np.where(entered, np.zeros_like(self.phase_for), self.phase_for)
        self.still_for = np.where(entered, np.zeros_like(self.still_for), self.still_for)
        seat_target = self.touch_depth + c.KEYWAY_DEPTH_M
        seat_end = seat_target + np.where(self.force_law, F32(0.0), F32(c.PRESS_M))
        self.seat_end = np.where(entered, np.maximum(seat_end, self.ref_depth), self.seat_end)
        self.ref_depth = np.where(entered & self.force_law, seat_target + self.free_lead(), self.ref_depth)
        self.ref_depth = np.where(entered & ~self.force_law, np.maximum(depth, self.ref_depth), self.ref_depth)
        self.still_any = np.where(entered, np.zeros_like(self.still_any), self.still_any)
        self.jiggle_for = np.where(entered, np.zeros_like(self.jiggle_for), self.jiggle_for)
        self.entry_roll = np.where(entered, self.roll, self.entry_roll)

    def step(self, depth, lateral, dt: float) -> None:
        """Advance with the tip measured at `depth` (N,) and `lateral` (N, 2), as float32 (the environment's dtype)."""
        c = self.c
        depth = np.asarray(depth, dtype=F32).reshape(self.n)
        lateral = np.asarray(lateral, dtype=F32).reshape(self.n, 2)
        active = self.phase < DONE
        if not bool(active.any()):
            return
        zeros = np.zeros_like(depth)
        filtered = self.last_depth + (depth - self.last_depth) * min(1.0, dt / c.SETTLE_FILTER_S)
        speed = (filtered - self.last_depth) / dt
        self.last_depth = filtered
        self.phase_for = np.where(active, self.phase_for + dt, self.phase_for)
        lag = self.ref_depth - depth
        located = self.seat_test == "located"
        if located:
            # Stages: 0 rolling in place, 3 approaching rolled, 1 backing off and rolling back, 4 approaching again at
            # zero roll, 2 located (`keylock.InsertLayer.step`).
            app_now = active & (self.phase == APPROACH)
            want = np.where((self.locate_stage == 0) | (self.locate_stage == 3),
                            np.full_like(self.roll, math.radians(c.FACE_ROLL_DEG)), np.zeros_like(self.roll))
            step_r = math.radians(c.ROLL_RATE_DEG_S) * dt
            turning = app_now & (((self.locate_stage == 0) & (self.phase_for >= self.roll_wait)) | (self.locate_stage == 1))
            self.roll = np.where(turning, self.roll + np.clip(want - self.roll, -step_r, step_r), self.roll)
            there = np.abs(self.roll - want) < 1e-4
            self.settle_for = np.where(turning & there, self.settle_for + dt, np.zeros_like(self.settle_for))
            settled = self.settle_for >= c.ROLL_SETTLE_S
            rolled = app_now & (self.locate_stage == 0) & settled
            unrolled = app_now & (self.locate_stage == 1) & settled
            self.locate_stage = np.where(rolled, 3, self.locate_stage)
            self.locate_stage = np.where(unrolled, 4, self.locate_stage)
            self.phase_for = np.where(rolled, np.zeros_like(self.phase_for),
                                      np.where(unrolled, np.full_like(self.phase_for, c.BASE_S), self.phase_for))
            self.lag_for = np.where(rolled | unrolled, np.zeros_like(self.lag_for), self.lag_for)
            self.lag_base = np.where(rolled, np.zeros_like(self.lag_base), self.lag_base)
        app = active & (self.phase == APPROACH) & (self.locate_stage != 0) & (self.locate_stage != 1)
        v = np.where((self.ref_depth < -c.FAST_UNTIL_M) & (self.locate_stage != 4), np.full_like(depth, c.FAST_SPEED_M_S),
                     np.full_like(depth, c.SLOW_SPEED_M_S))
        self.ref_depth = np.where(app, np.minimum(self.ref_depth + v * dt, self.max_depth), self.ref_depth)
        early = app & (self.phase_for < c.BASE_S)
        self.lag_base = np.where(early, np.clip(self.lag_base + (lag - self.lag_base) * min(1.0, dt / 0.2), 0.0, 0.005),
                                 self.lag_base)
        behind = (~early & (lag - self.lag_base > c.CONTACT_LAG_M)) | (lag > c.LAG_MAX_M)
        self.lag_for = np.where(app & behind, self.lag_for + dt, np.zeros_like(self.lag_for))
        felt = app & (self.lag_for >= c.CONTACT_HOLD_S)
        if located:
            found = felt & (self.locate_stage == 3)
            if bool(found.any()):
                self.face_depth = np.where(found, depth, self.face_depth)
                self.touched |= found
                self.ref_depth = np.where(found, depth - c.LOCATE_BACK_M, self.ref_depth)
                self.locate_stage = np.where(found, 1, self.locate_stage)
                self.phase_for = np.where(found, zeros, self.phase_for)
                self.lag_for = np.where(found, zeros, self.lag_for)
            again = felt & (self.locate_stage == 4)
            straight = again & (depth > self.face_depth + c.STRAIGHT_M)
            self.locate_stage = np.where(again, 2, self.locate_stage)
            if bool(straight.any()):
                self.touch_depth = np.where(straight, self.face_depth, self.touch_depth)
                self.force_cmd = np.where(straight & self.force_law, self.press_n, self.force_cmd)
                self.ref_depth = np.where(straight, depth + self.free_lead(), self.ref_depth)
                self._enter_seat(straight, depth, lateral, 1.0)
            felt = felt & ~found & ~straight
        if bool(felt.any()):
            self.touch_depth = np.where(felt, depth, self.touch_depth)
            self.touched |= felt
            press = np.where(self.force_law, zeros, np.full_like(depth, c.PRESS_M))
            self.force_cmd = np.where(felt & self.force_law, self.press_n, self.force_cmd)
            self.ref_depth = np.where(felt, depth + press + self.free_lead(), self.ref_depth)
            self.phase = np.where(felt, SEARCH, self.phase)
            self.phase_for = np.where(felt, zeros, self.phase_for)
            self.held_for = np.where(felt, zeros, self.held_for)
            self.probe_from = np.where(felt[:, None], lateral, self.probe_from)
        srch = active & (self.phase == SEARCH)
        if bool(srch.any()):
            probing = srch & (self.phase_for < c.PROBE_S)
            probe_ref = self.ref_lateral.copy()
            probe_ref[:, 0] = c.PROBE_M * np.minimum(self.phase_for / (0.5 * c.PROBE_S), 1.0)
            spiraling = srch & ~probing
            self.spiral_s = np.where(spiraling, self.spiral_s + self.spiral_dir * c.SPIRAL_SPEED_M_S * dt, self.spiral_s)
            r_max_s = 0.5 * c.SPIRAL_R_M ** 2 * 2 * math.pi / c.SPIRAL_PITCH_M
            self.spiral_dir = np.where(spiraling & (self.spiral_s >= r_max_s), -np.ones_like(self.spiral_dir),
                                       np.where(spiraling & (self.spiral_s <= 0.0), np.ones_like(self.spiral_dir),
                                                self.spiral_dir))
            self.ref_lateral = np.where(probing[:, None], probe_ref,
                                        np.where(spiraling[:, None], self._spiral(self.spiral_s), self.ref_lateral))
            self.wiggle_t = np.where(spiraling, self.wiggle_t + dt, self.wiggle_t)
            self.roll = np.where(spiraling, math.radians(c.WIGGLE_DEG) * np.sin(2 * math.pi * self.wiggle_t / c.WIGGLE_PERIOD_S),
                                 self.roll)
            if self.light_probe:
                eased = probing & (self.phase_for < 1.5 * dt)
                resumed = srch & (self.phase_for >= c.PROBE_S) & (self.phase_for < c.PROBE_S + 1.5 * dt)
                self.ref_depth = np.where(eased & ~self.force_law, self.touch_depth, self.ref_depth)
                self.ref_depth = np.where(resumed & ~self.force_law, self.touch_depth + c.PRESS_M, self.ref_depth)
                half = eased & self.force_law
                self.ref_depth = np.where(half, self.ref_depth - 0.5 * self.press_n / c.K_AXIAL_N_PER_M, self.ref_depth)
                self.force_cmd = np.where(half, 0.5 * self.press_n, self.force_cmd)
                full = resumed & self.force_law
                self.force_cmd = np.where(full, self.press_n, self.force_cmd)
                self.ref_depth = np.where(full, self.ref_depth + 0.5 * self.press_n / c.K_AXIAL_N_PER_M, self.ref_depth)
            probe_end = srch & (self.phase_for >= c.PROBE_S) & (self.phase_for < c.PROBE_S + 1.5 * dt)
            probe_held = probe_end & (np.abs((lateral - self.probe_from)[:, 0]) < c.PROBE_HELD_M)
            advanced = srch & (depth > self.touch_depth + c.ENTER_M)
            off = lateral - self.ref_lateral
            held = spiraling & (np.sqrt((off * off).sum(-1)) > c.HELD_SIDEWAYS_M)
            self.held_for = np.where(held, self.held_for + dt, np.zeros_like(self.held_for))
            held_long = spiraling & (self.held_for >= c.CONTACT_HOLD_S)
            entered = advanced | held_long | probe_held
            if bool(entered.any()):
                self.entered_by = np.where(entered, np.where(advanced, F32(1.0), F32(2.0)), self.entered_by)
                self.ref_lateral = np.where(entered[:, None], lateral, self.ref_lateral)
                self.phase = np.where(entered, SEAT, self.phase)
                self.phase_for = np.where(entered, zeros, self.phase_for)
                self.still_for = np.where(entered, zeros, self.still_for)
                seat_target = self.touch_depth + c.KEYWAY_DEPTH_M
                seat_end = seat_target + np.where(self.force_law, F32(0.0), F32(c.PRESS_M))
                self.seat_end = np.where(entered, np.maximum(seat_end, self.ref_depth), self.seat_end)
                self.ref_depth = np.where(entered & self.force_law, seat_target + self.free_lead(), self.ref_depth)
                self.ref_depth = np.where(entered & ~self.force_law, np.maximum(depth, self.ref_depth), self.ref_depth)
                self.still_any = np.where(entered, zeros, self.still_any)
                self.jiggle_for = np.where(entered, zeros, self.jiggle_for)
                self.entry_roll = np.where(entered, self.roll, self.entry_roll)
        seat = active & (self.phase == SEAT)
        if bool(seat.any()):
            nxt = np.minimum(self.ref_depth + c.SLOW_SPEED_M_S * dt, self.seat_end)
            self.ref_depth = np.where(seat & ~self.force_law, nxt, self.ref_depth)
            deep = depth > self.touch_depth + c.KEYWAY_DEPTH_M - c.SEATED_SHORT_M
            held_in = seat & (self.entered_by == 2.0)
            calm = seat & (np.abs(speed) < c.SETTLE_SPEED_M_S)
            self.still_for = np.where(calm & deep, self.still_for + dt, np.zeros_like(self.still_for))
            self.still_any = np.where(calm, self.still_any + dt, np.zeros_like(self.still_any))
            jiggling = seat & ~deep & (((self.still_any >= c.STILL_ANY_S) | (self.jiggle_for > 0.0))
                                       | (held_in & (self.jiggle_for < c.JIGGLE_HELD_S)))
            self.jiggle_for = np.where(jiggling, self.jiggle_for + dt, self.jiggle_for)
            wob = math.radians(c.JIGGLE_DEG) * np.sin(2 * math.pi * self.jiggle_for / c.JIGGLE_PERIOD_S)
            self.roll = np.where(jiggling, self.entry_roll + wob, np.where(seat & deep, self.entry_roll, self.roll))
            started = jiggling & (self.force_cmd <= self.press_n + 1e-6) & self.force_law
            self.force_cmd = np.where(started, 1.5 * self.press_n, self.force_cmd)
            self.ref_depth = np.where(started, self.ref_depth + 0.5 * self.press_n / c.K_AXIAL_N_PER_M, self.ref_depth)
            self.seat_end = np.where(jiggling & ~self.force_law & (self.jiggle_for <= dt + 1e-6), self.seat_end + 0.001,
                                     self.seat_end)
            if located:
                seated = seat & ((self.still_for >= c.CONTACT_HOLD_S) | ((self.still_any >= c.STILL_ANY_S) & deep))
                short = seat & ~seated & ~deep & (self.jiggle_for >= c.JIGGLE_S)
                again = short & (self.retries < c.RETRIES)
                give_up = short & ~again
                if bool(again.any()):
                    self.retries = np.where(again, self.retries + 1.0, self.retries)
                    self.force_cmd = np.where(again & self.force_law, self.press_n, self.force_cmd)
                    press = np.where(self.force_law, zeros, np.full_like(depth, c.PRESS_M))
                    self.ref_depth = np.where(again, self.touch_depth + press + self.free_lead(), self.ref_depth)
                    self.phase = np.where(again, SEARCH, self.phase)
                    self.phase_for = np.where(again, np.full_like(self.phase_for, c.PROBE_S + 2 * dt), self.phase_for)
                    self.held_for = np.where(again, zeros, self.held_for)
                    self.roll = np.where(again, self.entry_roll, self.roll)
                if bool(give_up.any()):
                    self.refused |= give_up
                    self.phase = np.where(give_up, DONE, self.phase)
                    self.force_cmd = np.where(give_up, zeros, self.force_cmd)
                    self.ref_depth = np.where(give_up, depth, self.ref_depth)
                seat = seat & ~short
            else:
                seated = seat & ((self.still_for >= c.CONTACT_HOLD_S) | (self.jiggle_for >= c.JIGGLE_S)
                                 | ((self.still_any >= c.STILL_ANY_S) & deep)
                                 | (held_in & (self.jiggle_for >= c.JIGGLE_HELD_S) & (self.still_any >= c.CONTACT_HOLD_S)))
            if bool(seated.any()):
                self.phase = np.where(seated, HOLD, self.phase)
                self.phase_for = np.where(seated, zeros, self.phase_for)
                self.force_cmd = np.where(seated & self.force_law, np.full_like(self.force_cmd, c.HOLD_PRESS_N),
                                          self.force_cmd)
                self.ref_depth = np.where(seated, depth + np.where(self.force_law, self.free_lead(), zeros), self.ref_depth)
        if located:
            lost_face = (active & ((self.phase == SEARCH) | (self.phase == SEAT)) & (self.locate_stage == 2)
                         & (depth > self.touch_depth + c.KEYWAY_DEPTH_M + c.RELOCATE_M)) \
                | (active & (self.phase == APPROACH) & (self.locate_stage == 4)
                   & (depth > self.face_depth + c.KEYWAY_DEPTH_M + c.RELOCATE_M))
            again = lost_face & (self.relocates < 1)
            if bool(again.any()):
                self.relocates = np.where(again, self.relocates + 1.0, self.relocates)
                self.locate_stage = np.where(again, 0, self.locate_stage)
                self.phase = np.where(again, APPROACH, self.phase)
                self.phase_for = np.where(again, zeros, self.phase_for)
                self.force_cmd = np.where(again, zeros, self.force_cmd)
                self.ref_depth = np.where(again, depth - c.KEYWAY_DEPTH_M - 2 * c.LOCATE_BACK_M, self.ref_depth)
                self.ref_lateral = np.where(again[:, None], lateral, self.ref_lateral)
                self.roll_wait = np.where(again, np.full_like(self.roll_wait, c.RELOCATE_WAIT_S), self.roll_wait)
                self.settle_for = np.where(again, zeros, self.settle_for)
                self.roll = np.where(again, zeros, self.roll)
                self.lag_for = np.where(again, zeros, self.lag_for)
                self.lag_base = np.where(again, zeros, self.lag_base)
            gone = lost_face & ~again
            if bool(gone.any()):
                self.refused |= gone
                self.phase = np.where(gone, DONE, self.phase)
                self.force_cmd = np.where(gone, zeros, self.force_cmd)
                self.ref_depth = np.where(gone, depth, self.ref_depth)
        hold = active & (self.phase == HOLD) & (self.phase_for >= c.HOLD_S)
        self.phase = np.where(hold, DONE, self.phase)


# --- rotations (w-first quaternions, as Isaac Lab and `interface` have them) ------------------------------------------

def quat_apply(quat, vec) -> np.ndarray:
    q = np.asarray(quat, dtype=float)
    vec = np.asarray(vec, dtype=float)
    xyz = q[1:4]
    t = 2.0 * np.cross(xyz, vec)
    return vec + q[0] * t + np.cross(xyz, t)


def quat_rotate_inverse(quat, vec) -> np.ndarray:
    q = np.asarray(quat, dtype=float)
    return quat_apply(np.array([q[0], -q[1], -q[2], -q[3]]), vec)


def quat_matrix(quat) -> np.ndarray:
    w, x, y, z = (float(v) for v in quat)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def yaw_quat(quat) -> np.ndarray:
    """The yaw-only part of `quat` (`interface.yaw_quat(interface.yaw_from_quat(q))`)."""
    w, x, y, z = (float(v) for v in quat)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)])


def hand_rotation(approach, bar, roll: float = 1.0) -> np.ndarray:
    """Link6's rotation (columns x, y, z) coming in along `approach` with the bar along `bar` (`keyhold.hand_rotation`)."""
    z = np.asarray(approach, dtype=float)
    z = z / np.linalg.norm(z)
    y = np.cross(z, np.asarray(bar, dtype=float))
    y = roll * y / np.linalg.norm(y)
    x = np.cross(y, z)
    return np.stack((x, y, z), axis=-1)


# --- the D1's kinematics (`unifp_train.arm_kin.ArmKinematics`, batched, float64) -------------------------------------

def _rodrigues(axis: np.ndarray, angle: np.ndarray) -> np.ndarray:
    x, y, z = (float(v) for v in axis)
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    s, c = np.sin(angle)[:, None, None], np.cos(angle)[:, None, None]
    return np.eye(3) + s * k + (1.0 - c) * (k @ k)


def _wrap(angle):
    return np.remainder(angle + math.pi, 2.0 * math.pi) - math.pi


def _signed_angle(a: np.ndarray, b: np.ndarray, axis: np.ndarray) -> np.ndarray:
    a = a - (a @ axis)[:, None] * axis
    b = b - (b @ axis)[:, None] * axis
    return np.arctan2(np.cross(a, b) @ axis, (a * b).sum(-1))


def _perpendicular(axis: np.ndarray) -> np.ndarray:
    other = np.zeros(3)
    other[int(np.argmin(np.abs(axis)))] = 1.0
    out = np.cross(axis, other)
    return out / np.linalg.norm(out)


def rotation_angle(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    trace = (a * b).sum(axis=(-1, -2))
    return np.arccos(np.clip((trace - 1.0) * 0.5, -1.0, 1.0))


class ArmKinematics:
    """The chain from the Go2's base to Link6 (`arm_kin.ArmKinematics`'s algorithms), from the export's
    `insert.kinematics`: each joint's origin (`xyz`), fixed rotation (`rpy`, 3x3) and axis, the mount, the wrist centre
    in Link4, the jaw centre from the wrist in Link6, the jaw centre in Link6."""

    def __init__(self, k: dict):
        self.xyz = [np.asarray(v, dtype=float) for v in k["xyz"]]
        self.rpy = [np.asarray(v, dtype=float) for v in k["rpy"]]
        self.axis = [np.asarray(v, dtype=float) for v in k["axis"]]
        self.mount = np.asarray(k["mount"], dtype=float)
        self.hard_lower, self.hard_upper = np.asarray(k["hard_lower"], dtype=float), np.asarray(k["hard_upper"], dtype=float)
        self.wrist_local = np.asarray(k["wrist_local"], dtype=float)
        self.tool_from_wrist = np.asarray(k["tool_from_wrist"], dtype=float)
        self.jaw_local = np.asarray(k["jaw_local"], dtype=float)

    def chain(self, q: np.ndarray, upto: int = 6) -> tuple[np.ndarray, np.ndarray]:
        q = np.atleast_2d(np.asarray(q, dtype=float))
        n = q.shape[0]
        rotation = np.broadcast_to(np.eye(3), (n, 3, 3))
        position = np.broadcast_to(self.mount, (n, 3))
        for k in range(upto):
            position = position + rotation @ self.xyz[k]
            rotation = rotation @ self.rpy[k] @ _rodrigues(self.axis[k], q[:, k])
        return rotation, position

    def tool(self, q: np.ndarray, offset=None) -> tuple[np.ndarray, np.ndarray]:
        """(Link6's rotation, the point `offset` in Link6 -- the jaw centre by default), base frame."""
        rotation, position = self.chain(q, 6)
        local = self.jaw_local if offset is None else np.asarray(offset, dtype=float)
        return rotation, position + rotation @ local

    def wrist(self, q: np.ndarray) -> np.ndarray:
        q = np.atleast_2d(q)
        padded = np.concatenate((q[:, :3], np.zeros((q.shape[0], 1))), axis=1)
        rotation, position = self.chain(padded, 4)
        return position + rotation @ self.wrist_local

    def _wrist_jacobian(self, q3: np.ndarray) -> np.ndarray:
        n = q3.shape[0]
        rotation = np.broadcast_to(np.eye(3), (n, 3, 3))
        position = np.broadcast_to(self.mount, (n, 3))
        axes, origins = [], []
        for k in range(3):
            position = position + rotation @ self.xyz[k]
            rotation = rotation @ self.rpy[k]
            axes.append(rotation @ self.axis[k])
            origins.append(position)
            rotation = rotation @ _rodrigues(self.axis[k], q3[:, k])
        wrist = self.wrist(q3)
        return np.stack([np.cross(a, wrist - o) for a, o in zip(axes, origins)], axis=-1)

    def solve_wrist(self, target: np.ndarray, seed: np.ndarray, iterations: int = 12) -> tuple[np.ndarray, np.ndarray]:
        q3 = np.array(seed, dtype=float)
        eye = 1e-4 * np.eye(3)
        for _ in range(iterations):
            error = target - self.wrist(q3)
            jac = self._wrist_jacobian(q3)
            jt = np.swapaxes(jac, 1, 2)
            step = np.linalg.solve(jt @ jac + eye, jt @ error[..., None])[..., 0]
            q3 = q3 + np.clip(step, -0.4, 0.4)
        error = target - self.wrist(q3)
        return q3, np.linalg.norm(error, axis=-1)

    def solve_hand(self, q3: np.ndarray, rotation: np.ndarray) -> list[np.ndarray]:
        padded = np.concatenate((q3, np.zeros((q3.shape[0], 1))), axis=1)
        link4, _ = self.chain(padded, 4)
        rel = np.swapaxes(link4, 1, 2) @ rotation
        f5, f6 = self.rpy[4], self.rpy[5]
        a4, a5, a6 = self.axis[3], self.axis[4], self.axis[5]
        h = rel @ a6
        v = f6 @ a6
        z4 = a4
        p = f5.T @ z4
        pv = float(p @ v)
        pc = float(p @ np.cross(a5, v))
        amplitude = math.hypot(pv, pc)
        phase = math.atan2(pc, pv)
        spread = np.arccos(np.clip((h @ z4) / amplitude, -1.0, 1.0))
        solutions = []
        for q5 in (phase + spread, phase - spread):
            q5 = _wrap(q5)
            rot5 = _rodrigues(a5, q5)
            h0 = (rot5 @ v) @ f5.T
            q4 = _signed_angle(h0, h, z4)
            rot4 = _rodrigues(a4, q4)
            rest = np.swapaxes(rot4 @ f5 @ rot5 @ f6, 1, 2) @ rel
            e = _perpendicular(a6)
            q6 = _signed_angle(np.broadcast_to(e, (rest.shape[0], 3)), rest @ e, a6)
            solutions.append(np.stack((q4, q5, q6), axis=-1))
        return solutions

    def track(self, position, rotation, seed, lower, upper, iterations: int = 5,
              tol_m: float = 3e-3) -> tuple[np.ndarray, np.ndarray]:
        wrist_goal = position - rotation @ self.tool_from_wrist
        q3, miss = self.solve_wrist(wrist_goal, seed[:, :3], iterations=iterations)
        first, second = self.solve_hand(q3, rotation)
        near = np.abs(first - seed[:, 3:]).sum(-1) <= np.abs(second - seed[:, 3:]).sum(-1)
        q = np.concatenate((q3, np.where(near[:, None], first, second)), axis=-1)
        return q, (miss < tol_m) & ((q >= lower) & (q <= upper)).all(axis=-1)

    def inverse(self, position, rotation, seeds, lower, upper, tol_m: float = 2e-3) -> tuple[np.ndarray, np.ndarray]:
        """`arm_kin.ArmKinematics.inverse`: every Joint1-3 seed and both wrists, the solution inside the limits nearest
        the arm's middle."""
        n = position.shape[0]
        wrist_goal = position - rotation @ self.tool_from_wrist
        best = np.zeros((n, 6))
        best_cost = np.full(n, math.inf)
        for seed in np.asarray(seeds, dtype=float):
            q3, miss = self.solve_wrist(wrist_goal, np.broadcast_to(seed, (n, 3)))
            q3_ok = (miss < tol_m) & ((q3 >= lower[:3]) & (q3 <= upper[:3])).all(axis=-1)
            for q456 in self.solve_hand(q3, rotation):
                q = np.concatenate((q3, q456), axis=-1)
                inside = q3_ok & ((q456 >= lower[3:]) & (q456 <= upper[3:])).all(axis=-1)
                r, p = self.tool(q)
                exact = (np.linalg.norm(p - position, axis=-1) < 4e-3) & (rotation_angle(r, rotation) < 0.03)
                cost = (q * q).sum(-1)
                take = inside & exact & (cost < best_cost)
                best = np.where(take[:, None], q, best)
                best_cost = np.where(take, cost, best_cost)
        return best, np.isfinite(best_cost)

    def pose_error(self, q, p_goal, r_goal) -> np.ndarray:
        """`keyinsert_env._pose_error`: the jaw centre's position error and the small-angle rotation error, (k, 6)."""
        r, p = self.tool(q)
        re = r_goal @ np.swapaxes(r, 1, 2)
        w = 0.5 * np.stack((re[:, 2, 1] - re[:, 1, 2], re[:, 0, 2] - re[:, 2, 0], re[:, 1, 0] - re[:, 0, 1]), -1)
        return np.concatenate((p_goal - p, w), -1)

    def solve_pose(self, p_b, r_b, seed, lower, upper, ik: dict) -> tuple[np.ndarray, np.ndarray]:
        """`keyinsert_env._ik`: `track` from `seed`, then `ik["polish_iterations"]` damped least-squares steps on the
        whole pose (finite differences of `ik["polish_eps"]`), inside `lower`-`upper`; solved within the tolerances."""
        q, _ = self.track(p_b, r_b, seed, lower, upper, iterations=ik["track_iterations"])
        eps, k = ik["polish_eps"], q.shape[0]
        for _ in range(ik["polish_iterations"]):
            err = self.pose_error(q, p_b, r_b)
            qq = (q[:, None, :] + np.eye(6)[None] * eps).reshape(-1, 6)
            e2 = self.pose_error(qq, np.repeat(p_b, 6, 0), np.repeat(r_b, 6, 0)).reshape(k, 6, 6)
            jac = -np.swapaxes(e2 - err[:, None, :], 1, 2) / eps
            jt = np.swapaxes(jac, 1, 2)
            step = np.linalg.solve(jt @ jac + ik["polish_damping"] * np.eye(6), jt @ err[..., None])[..., 0]
            q = np.maximum(np.minimum(q + step, upper), lower)
        err = self.pose_error(q, p_b, r_b)
        ok = (np.linalg.norm(err[:, :3], axis=-1) < ik["polish_tol_m"]) & (np.linalg.norm(err[:, 3:], axis=-1)
                                                                          < ik["polish_rot_tol_rad"])
        return q, ok


class ArmGravity:
    """`keyhold.ArmGravity.hold`: the joint torques holding the D1 still against gravity, from the export's
    `insert.gravity` (each link's mass and centre of mass, the fingers folded into Link6)."""

    def __init__(self, kin: ArmKinematics, g: dict):
        self.kin = kin
        self.mass = np.asarray(g["mass"], dtype=float)
        self.com = np.asarray(g["com"], dtype=float)

    def hold(self, q: np.ndarray, gravity_b: np.ndarray) -> np.ndarray:
        q = np.atleast_2d(q)
        n = q.shape[0]
        gravity_b = np.broadcast_to(gravity_b, (n, 3))
        axes, origins, coms = [], [], []
        for k in range(6):
            rot, pos = self.kin.chain(q, k + 1)
            axes.append(rot @ self.kin.axis[k])
            origins.append(pos)
            coms.append(pos + rot @ self.com[k])
        torque = np.zeros((n, 6))
        for j in range(6):
            for link in range(j, 6):
                force = self.mass[link] * gravity_b
                torque[:, j] -= (axes[j] * np.cross(coms[link] - origins[j], force)).sum(-1)
        return torque


# --- the runner -----------------------------------------------------------------------------------------------------

@dataclass
class LockEstimate:
    """The lock as the robot believes it, in the frame the base's pose is given in (gravity-aligned, z up): the face's
    centre on the keyway's axis, the axis into the lock, the bar's direction across it (`u`, level, to the robot's
    right as `keyinsert_env` places it) and `t` = u x axis (computed if not given)."""
    origin: np.ndarray
    axis: np.ndarray
    u: np.ndarray
    t: np.ndarray | None = None

    def __post_init__(self):
        self.origin, self.axis, self.u = (np.asarray(v, dtype=float) for v in (self.origin, self.axis, self.u))
        self.t = np.cross(self.u, self.axis) if self.t is None else np.asarray(self.t, dtype=float)


class InsertRunner:
    """One robot's insertion around an exported policy (`LegPolicy` with `meta["task"] == "insert"`).

    `reset` once per insertion; then each 50 Hz step: `update` (not after a reset), `frame`, the policy's `act` on the
    returned stack, `targets` -- or `step`, which does the four. `events` logs the layer's decisions."""

    def __init__(self, meta: dict, seat_test: str | None = None, light_probe: bool | None = None):
        if meta.get("task") != "insert":
            raise ValueError(f"not an insertion export (task {meta.get('task')!r})")
        ins = meta["insert"]
        self.meta, self.ins = meta, ins
        self.env = SimpleNamespace(**ins["env"])
        self.ik = ins["ik"]
        self.kin = ArmKinematics(ins["kinematics"])
        self.gravity = ArmGravity(self.kin, ins["gravity"])
        self.sag_on = bool(ins["sag_compensation"])
        self.arm_kp = np.asarray(ins["arm"]["kp"], dtype=float)
        self.lower, self.upper = np.asarray(ins["arm"]["lower"], dtype=float), np.asarray(ins["arm"]["upper"], dtype=float)
        layer = ins["layer"]
        self.layer = InsertLayer(1, layer["constants"], seat_test or layer["seat_test"],
                                 layer["light_probe"] if light_probe is None else light_probe)
        self.force_aware = bool(ins["force_aware"])
        self.obs = leg_policy.LegObservation(meta)
        self.dt = float(meta["policy_dt"])
        self.action_scale = float(meta["action_scale"])
        self.clip_actions = float(meta["clip_actions"])
        self.grip_roll = float(ins["key"]["grip_roll"])
        self.foot_goal = np.asarray(ins["commands"]["foot_goal_sphere"], dtype=float)
        self.miss_scale = float(ins["commands"]["miss_obs_scale"])
        self.events: list[dict] = []
        self.reset_called = False

    # --- frames -------------------------------------------------------------------------------------------------------

    def _in_base(self, point, rot=None):
        p = quat_rotate_inverse(self.base_quat, np.asarray(point, dtype=float) - self.base_pos)
        if rot is None:
            return p
        return p, quat_matrix(self.base_quat).T @ rot

    def _est_point(self, depth: float, lateral) -> np.ndarray:
        lk = self.lock
        return lk.origin + float(lateral[0]) * lk.u + float(lateral[1]) * lk.t + float(depth) * lk.axis

    def _hand(self) -> np.ndarray:
        return hand_rotation(self.lock.axis, self.lock.u, self.grip_roll)

    def _jaw_for_tip(self, tip) -> np.ndarray:
        return np.asarray(tip, dtype=float) - self._hand() @ self.tip_off

    def _sag(self, q: np.ndarray) -> np.ndarray:
        if not self.sag_on:
            return np.zeros(6)
        g_b = quat_rotate_inverse(self.base_quat, np.array([0.0, 0.0, -9.81]))
        return self.gravity.hold(q[None], g_b)[0] / self.arm_kp

    def _solve(self, jaw_goal, seed) -> tuple[np.ndarray, bool]:
        p_b, r_b = self._in_base(jaw_goal, self._hand())
        q, ok = self.kin.solve_pose(p_b[None], r_b[None], np.asarray(seed, dtype=float)[None], self.lower, self.upper,
                                    self.ik)
        return q[0], bool(ok[0])

    # --- the episode --------------------------------------------------------------------------------------------------

    def reset(self, lock: LockEstimate, base_pos, base_quat, start_depth: float, press_n: float,
              key_reach: float | None = None, key_offset=(0.0, 0.0), arm_pose=None) -> tuple[np.ndarray, bool]:
        """A new insertion: the arm's start pose (the believed key's tip `start_depth` along the estimated axis from the
        estimated face, negative short of it), solved from the base's pose now. Returns (the six joint angles, solved);
        unsolved, the runner holds the arm at `arm_pose` (the export's rest pose) and the layer never starts, as the
        environment does."""
        self.lock = lock
        self.base_pos, self.base_quat = np.asarray(base_pos, dtype=float), np.asarray(base_quat, dtype=float)
        key = self.ins["key"]
        reach = key["tip_from_jaw_m"] if key_reach is None else float(key_reach)
        self.tip_off = np.array([float(key_offset[0]), float(key_offset[1]), reach])
        self.start_depth = np.full(1, start_depth, F32)
        self.press_draw = np.full(1, press_n, F32)
        ly = self.layer
        ly.stop(slice(None))
        ly.roll[:] = 0.0
        ly.ref_depth[:] = self.start_depth
        ly.ref_lateral[:] = 0.0
        ly.force_cmd[:] = 0.0
        self.loop = np.zeros(3, F32)
        self.miss_b = np.zeros(3)
        self.since_reset = 0
        self.layer_started = False
        self.done_for = np.zeros(1, F32)
        self.ik_calls = self.ik_fail = 0
        tip = self.lock.origin + float(self.start_depth[0]) * self.lock.axis
        self.goal_jaw = self._jaw_for_tip(tip)
        q, ok = self._solve(self.goal_jaw, self.ins["ik"]["nominal"])
        if not ok:
            p_b, r_b = self._in_base(self.goal_jaw, self._hand())
            q2, _ = self.kin.inverse(p_b[None], r_b[None], self.ins["ik"]["seeds"], self.kin.hard_lower,
                                     self.kin.hard_upper)
            q, ok = self._solve(self.goal_jaw, q2[0])
        self.ins_geo = q
        self.ins_ref = q + self._sag(q)
        self.active = ok
        self.arm_pose = np.asarray(arm_pose if arm_pose is not None else self.meta["arm_rest_pose"], dtype=float)
        if ok:
            self.arm_pose = q.copy()
        self.meas_depth, self.meas_lateral = np.zeros(1, F32), np.zeros((1, 2), F32)
        self.obs.reset()
        self.press_b = np.zeros(3)
        self._fresh = True
        self._arm_measure = None
        self.events = [{"step": 0, "event": "reset", "solved": ok}]
        self.reset_called = True
        return q.copy(), ok

    def _measure(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """The jaw centre (lock frame and base frame) and the believed key's tip, by the reported joints."""
        rot_b, jaw_b = self.kin.tool(self._arm_measure[None])
        rot_b, jaw_b = rot_b[0], jaw_b[0]
        jaw = self.base_pos + quat_apply(self.base_quat, jaw_b)
        tip = self.base_pos + quat_apply(self.base_quat, jaw_b + rot_b @ self.tip_off)
        return jaw, jaw_b, tip

    def update(self, base_pos, base_quat) -> None:
        """A step's layer, goal, outer loop, reference and miss (`keyinsert_env._update_hold`), with the base's pose
        after the step and the arm measured by the previous frame's report."""
        if self._arm_measure is None:
            raise RuntimeError("update before the reset's frame")
        self.base_pos, self.base_quat = np.asarray(base_pos, dtype=float), np.asarray(base_quat, dtype=float)
        ly, e, lk, dt = self.layer, self.env, self.lock, self.dt
        _, jaw_b, tip = self._measure()
        rel = tip - lk.origin
        self.meas_depth = np.full(1, rel @ lk.axis, F32)
        self.meas_lateral = np.array([[rel @ lk.u, rel @ lk.t]], F32)
        s = self.since_reset
        if self.active and ly.phase[0] == DONE and s == e.START_STEPS and not self.layer_started:
            ly.start(slice(None), self.start_depth, self.force_aware, self.press_draw)
            self.layer_started = True
            self.events.append({"step": s + 1, "event": "start", "depth_mm": round(1e3 * float(self.meas_depth[0]), 3)})
        before = (int(ly.phase[0]), int(ly.locate_stage[0]), bool(ly.refused[0]))
        ly.step(self.meas_depth, self.meas_lateral, dt)
        depth, lateral = ly.goal()
        self.goal_jaw = self._jaw_for_tip(self._est_point(depth[0], lateral[0]))
        phase = int(ly.phase[0])
        running = self.active and self.layer_started and phase < DONE
        free_lat = running and phase <= SEARCH and float(ly.held_for[0]) <= 0.0
        if free_lat:
            self.loop[:2] = self.loop[:2] + e.LOOP_GAIN * dt * (ly.ref_lateral[0] - self.meas_lateral[0])
        free_ax = running and phase == APPROACH and float(ly.phase_for[0]) < self.layer.c.BASE_S
        if free_ax:
            self.loop[2:] = self.loop[2:] + e.LOOP_GAIN * dt * (ly.ref_depth - self.meas_depth)
        self.loop = np.clip(self.loop, -e.LOOP_MAX_M, e.LOOP_MAX_M)
        loop_w = float(self.loop[0]) * lk.u + float(self.loop[1]) * lk.t + float(self.loop[2]) * lk.axis
        if self.active and s % e.IK_EVERY_STEPS == 0:
            q, ok = self._solve(self.goal_jaw + loop_w, self.ins_geo)
            self.ik_calls += 1
            if ok:
                self.ins_geo = q
                self.ins_ref = q + self._sag(q)
            else:
                self.ik_fail += 1
        self.miss_b = (self._in_base(self.goal_jaw) - jaw_b) * float(self.active)
        self.done_for = np.where((ly.phase == DONE) & self.layer_started, self.done_for + dt, np.zeros_like(self.done_for))
        self.since_reset = s + 1
        after = (int(ly.phase[0]), int(ly.locate_stage[0]), bool(ly.refused[0]))
        if after != before:
            ev = {"step": self.since_reset, "event": f"{PHASE_NAMES[before[0]]}->{PHASE_NAMES[after[0]]}",
                  "depth_mm": round(1e3 * float(self.meas_depth[0]), 3),
                  "touch_mm": round(1e3 * float(ly.touch_depth[0]), 3), "press_n": round(float(ly.force_cmd[0]), 3)}
            if after[0] == SEAT and before[0] != SEAT:
                ev["entered_by"] = {1: "advanced", 2: "held"}.get(int(ly.entered_by[0]), "?")
            if after[1] != before[1]:
                ev["locate_stage"] = after[1]
            if after[2] and not before[2]:
                ev["refused"] = True
            self.events.append(ev)

    def frame(self, imu_quat, gyro, q_legs, dq_legs, arm_q, arm_dq, foot_goal=None) -> np.ndarray:
        """Push this step's frame and return the (2432,) stack. Legs in Unitree's order; the arm as the D1 reports it
        (URDF angles, rad) with its report-difference rates; `foot_goal` the foot's (radius, pitch, yaw) command, its
        footfall by default. The arm's report is also what the next `update` measures with."""
        commands = np.zeros(int(self.meta["num_commands"]))
        if not self._fresh:
            commands[3:6] = self.foot_goal if foot_goal is None else np.asarray(foot_goal, dtype=float)
            commands[6:9] = self.miss_b * self.miss_scale
            commands[9:12] = self.press_b
        self.commands = commands
        self.last_frame = self.obs.frame(imu_quat, gyro, q_legs, dq_legs, arm_q, arm_dq, commands)
        self._arm_measure = np.asarray(arm_q, dtype=float).copy()
        self._fresh = False
        return self.obs.step(self.last_frame)

    def targets(self, action) -> tuple[np.ndarray, np.ndarray]:
        """The 12 leg targets (Unitree's order) and the arm's six (URDF order, rad) for the policy's 18 outputs, and the
        press for the next frame (`keyinsert_env._pre_physics_step`, `_step_forces`)."""
        a = np.clip(np.asarray(action, dtype=float).reshape(-1), -self.clip_actions, self.clip_actions)
        if a.size != 18:
            raise ValueError(f"an insertion policy has 18 outputs, not {a.size}")
        legs = a[:12]
        arm = a[12:18].copy()
        arm[5] = 0.0
        arm = arm * float(self.active)
        self.obs.prev_actions = np.concatenate((legs, arm))
        leg_t = self.obs.leg_targets_unitree(legs)
        if self.active:
            ref = self.ins_ref.copy()
            ref[5] = ref[5] + float(self.layer.roll[0])
            arm_t = np.maximum(np.minimum(ref + self.action_scale * arm, self.upper), self.lower)
        else:
            arm_t = self.arm_pose.copy()
        press = float(self.layer.force_cmd[0]) * float(self.active)
        cmd_b = quat_rotate_inverse(yaw_quat(self.base_quat), press * self.lock.axis)
        self.press_b = cmd_b if self.force_aware else np.zeros(3)
        return leg_t, arm_t

    def step(self, policy, imu_quat, gyro, q_legs, dq_legs, arm_q, arm_dq, base_pos, base_quat, foot_goal=None) -> dict:
        """One 50 Hz step: the layer (unless this is the reset's frame), the frame, the policy, the targets."""
        first = self._fresh
        if not first:
            self.update(base_pos, base_quat)
        else:
            self.base_pos, self.base_quat = np.asarray(base_pos, dtype=float), np.asarray(base_quat, dtype=float)
        stack = self.frame(imu_quat, gyro, q_legs, dq_legs, arm_q, arm_dq, foot_goal)
        action = policy.act(stack)[0]
        legs, arm = self.targets(action)
        return {"action": action, "legs": legs, "arm": arm, "phase": PHASE_NAMES[int(self.layer.phase[0])],
                "press_n": float(self.layer.force_cmd[0]), "finished": self.finished}

    @property
    def finished(self) -> bool:
        """The layer's hold is over this long (`END_AFTER_DONE_S`): the environment ends the episode here."""
        return bool(self.done_for[0] >= self.env.END_AFTER_DONE_S)
