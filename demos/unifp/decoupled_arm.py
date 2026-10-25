"""The arm half of the decoupled door-pull baseline: inverse kinematics on the door pull's own script.

The UniFP door pull (`run_demo.py --task door`) hands a whole-body policy one goal per step: where the jaw
centre should be. The baseline runs the same script (`script.door_phases`) without that policy. A walking
policy that knows nothing of the arm moves the legs (`decoupled.py`), and the arm is put where the script
asks by inverse kinematics in the base's current frame, the way the scripted combiner demo puts it on the lever:
- the same IK (`d1_ik.solve` on Link6 and the jaw centre);
- the same hand orientation convention (`demos.combiner.pull.Handle`);
- the same candidate grasps (`pull.PITCHES_DEG` x `ROLLS`), tried nearest-level first (`GRASP_ORDER`).

The script gives a hand pose up to one freedom. It names the approach (into the door) and the bar's axis, and a
grip across a bar can come in from level or from above it, rotated about the bar, without changing what it
holds. That rotation is the scripted grip's `pitch_deg`, and its `roll` says which way up the jaws are. Both
are chosen once per attempt, at the first step after the spawn settles: the first grasp in `GRASP_ORDER` whose
grasp point and standoff the arm can reach, clear of the trunk and holdable against gravity. They are then held
for the attempt. The base co-rotates with the door, so the hand pose in the base's frame barely changes
through the walk.

Numpy only: it runs on the system Python, so its geometry is tested without a simulator
(`tests/test_decoupled.py`).
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

import d1_ik
from demos.combiner.geometry import GEOMETRY
from demos.combiner.pull import PITCHES_DEG, ROLLS
from demos.cup.pick_demo.grasp import JAW_CENTRE_LINK6, _seeds, solve_link6

UP = np.array([0.0, 0.0, 1.0])

#: Grasps in the order they are tried: the scripted grip's candidates, nearest level first. Not its own preference
#: (`pull.PROVEN_GRASPS`, pitch 40 then 50, held through the lever's turn): the door script enters along the level
#: approach (`script.offset` of the grasp point along `approach`), where the scripted grip enters along the pitched
#: one, so a pitched hand moving in level sweeps its fingers across the bar. The first smoke run took pitch 40 and
#: its fingers met the bar at 9-20 N before arriving (Week 6 log, 2026-10-19).
GRASP_ORDER = tuple((p, r) for p in sorted(PITCHES_DEG) for r in ROLLS)


def _unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def axis_angle(axis, angle_rad: float) -> np.ndarray:
    """Rotation matrix about a unit `axis` (Rodrigues)."""
    x, y, z = _unit(axis)
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + s * k + (1.0 - c) * (k @ k)


def hand_rotation(approach, bar_axis, pitch_deg: float, roll: int) -> np.ndarray:
    """Link6's rotation (columns: x, the jaw axis y, the approach z) holding a bar, as `pull.Handle` builds it.

    `approach` points into the door and `bar_axis` along the bar, as the script's `Command.approach` and
    `object_axis` give them. At `pitch_deg` 0 the hand comes in along `approach`, its jaws across the bar
    (`roll` +1 or -1 picks which way up). The pitch then turns the hand about the bar, tipping the approach
    down so that it comes from above. `Handle` writes this in the enclosure frame, where the bar runs along
    `hinge_side * y`. Here it is written from the two directions alone, so it is the same in any frame.
    """
    z = _unit(approach)
    bar = _unit(bar_axis)
    y = roll * _unit(np.cross(z, bar))
    x = np.cross(y, z)
    tool = np.column_stack([x, y, z])
    return axis_angle(GEOMETRY.hinge_side * bar, -math.radians(pitch_deg)) @ tool


#: The control loop's IK budget: iterations per solve, warm-started from the last solution. From a nearby target a
#: solve converges in a few; the cap is what bounds a target out of reach. Measured on the development PC
#: (2026-10-19), a solve costs about 0.12 ms an iteration, so one 10 Hz command is at most ~7 ms (pose, then
#: position).
#:
#: The first version re-ran the scripted grip's seeded search (`solve_link6`, 13 seeds x up to 300 iterations) on
#: every miss: up to 486 ms a command, which stalled the simulation whenever the base lagged the handle out of reach,
#: and could not run at 10 Hz on the robot. It opened more doors, 15 of 32 on two seeds against this loop's 9 of 32:
#: once an attempt, as the walk starts to lag, its search finds another branch where this loop falls back to the
#: jaw centre alone. Bounded versions of that search (one seed a command, at 100 or 300 iterations; the full search
#: once, at the first oriented command) did not recover it (Week 6 log, 2026-10-19). `seeded_loop` keeps it.
LOOP_ITERATIONS = 30
#: A pose solve that stops at the cap within these is used: the hand is where it should be to well under the
#: script's own tolerances.
POSE_ACCEPT_M = 0.002
POSE_ACCEPT_RAD = 0.02


@dataclass
class IKStep:
    q: np.ndarray
    #: "pose": the commanded hand pose, from the last solution. "pose_reseeded": the same, from the seeded search
    #: (`seeded_loop` only).
    #: "position": the pose has no solution, so the jaw centre alone. "hold": neither; the last q is kept.
    status: str
    position_error_m: float
    rotation_error_rad: float


class ArmIK:
    """One environment's arm: the grasp chosen for the attempt and the last solution, which seeds the next."""

    def __init__(self, joints, links, offset=JAW_CENTRE_LINK6, max_iterations: int = LOOP_ITERATIONS,
                 seeded_loop: bool = False):
        """`seeded_loop`: the first version's loop, kept to compare against (Week 6 log, 2026-10-19): 100 iterations,
        strict convergence, the seeded search on a miss, and no warm start from the grasp choice. Up to ~0.5 s a
        command; a diagnostic, not a controller."""
        self.joints, self.links = joints, links
        self.offset = np.asarray(offset, dtype=float)
        self.seeded_loop = seeded_loop
        self.max_iterations = 100 if seeded_loop else max_iterations
        self.soft = d1_ik.SOFT_LIMIT_FACTOR      # one factor or six (`d1_ik.joint_limits`)
        self.reset(np.zeros(6))

    def reset(self, q) -> None:
        self.q = np.asarray(q, dtype=float).copy()
        self.grasp: tuple[float, int] | None = None
        self.grasp_note = ""

    def choose_grasp(self, point_b, approach_b, bar_b, standoff_m: float) -> tuple[float, int]:
        """The first grasp in `GRASP_ORDER` whose grasp point and standoff solve; (0, +1) if none does.

        A planning step, once per attempt: it searches from all the scripted grip's seeds, which the control loop
        does not.
        """
        point_b = np.asarray(point_b, dtype=float)
        for pitch, roll in GRASP_ORDER:
            rot = hand_rotation(approach_b, bar_b, pitch, roll)
            standoff = point_b - standoff_m * rot[:, 2]
            solutions = [solve_link6(self.joints, self.links, target, rot, _seeds(target, [self.q]), self.offset)
                         for target in (standoff, point_b)]
            if all(s is not None for s in solutions):
                self.grasp, self.grasp_note = (pitch, roll), f"pitch {pitch:g} deg, roll {roll:+d}"
                return self.grasp
        self.grasp, self.grasp_note = (0.0, 1), "none of the scripted grip's grasps solves; pitch 0, roll +1"
        return self.grasp

    def solve(self, point_b, rot_b=None) -> IKStep:
        """Joint angles for the jaw centre at `point_b` with Link6 at `rot_b` (None: position only).

        Warm-started from the last solution, which keeps the arm on one branch from step to step, and capped at
        `max_iterations`, which bounds the cost of a target out of reach. A pose not met within `POSE_ACCEPT_*`
        falls back to the jaw centre alone, so the hand still goes to the point when the orientation cannot be met.
        At most two capped solves a command (`seeded_loop`: the first version's unbounded search, see
        `LOOP_ITERATIONS`).
        """
        point_b = np.asarray(point_b, dtype=float)
        if rot_b is not None:
            result = d1_ik.solve(self.joints, point_b, rot_b, q0=self.q, body="Link6", offset=self.offset,
                                 max_iterations=self.max_iterations, soft=self.soft)
            if result.converged or (not self.seeded_loop and result.position_error_m < POSE_ACCEPT_M
                                    and result.rotation_error_rad < POSE_ACCEPT_RAD):
                self.q = result.q
                return IKStep(result.q, "pose", result.position_error_m, result.rotation_error_rad)
            if self.seeded_loop:
                seeded = solve_link6(self.joints, self.links, point_b, rot_b, _seeds(point_b, [self.q]), self.offset,
                                     self.soft)
                if seeded is not None:
                    self.q = seeded.q
                    return IKStep(seeded.q, "pose_reseeded", seeded.position_error_m, seeded.rotation_error_rad)
        result = d1_ik.solve(self.joints, point_b, None, q0=self.q, body="Link6", offset=self.offset,
                             max_iterations=self.max_iterations, soft=self.soft)
        if result.position_error_m < 0.01:
            self.q = result.q
            return IKStep(result.q, "position", result.position_error_m, float("nan"))
        return IKStep(self.q.copy(), "hold", result.position_error_m, float("nan"))
