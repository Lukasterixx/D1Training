"""The scripted pick as a state machine over joint-angle feedback, camera frames and joint targets.

It holds no simulator or hardware handle. Each `update` takes the time, the arm's latest *feedback*
angles (what the D1 publishes at ~9 Hz, not the simulator's exact state) and world up from the IMU, and
returns a joint target and a gripper opening. Frames are pulled through a callable only when a state
needs one. The same object can therefore drive `D1Client` beside a real RealSense.

Motion follows F-035 where it can: a move between poses is one waypoint the firmware plans its own
trapezoid to, and the sequence waits for the arm to arrive and stop. A straight descent or lift is several
waypoints (`grasp.line_waypoints`), each passed once feedback is within `arrive_tolerance_rad` of it. They
are closer together than that tolerance, so in practice they stream at about the feedback rate -- the
per-cycle mode F-035 found slower and less smooth on the hardware. In simulation the jaw stayed within
4.9 mm of the planned descent line (Week 1 log, 2026-09-17).

    settle -> survey (one angled look over the floor ahead; the close scan only if it finds nothing)
    -> plan grasp -> pregrasp
    -> refine (look again from above, re-plan if the cup is elsewhere) -> descend -> close -> lift -> hold

The gripper is the plan's to choose, not the sequence's: an outside grasp descends open and closes on the
cup, a wall grasp on a cup too wide for the jaws descends shut and opens against the wall from inside
(`grasp.GraspParams.wall_grasp`). Both reach the arm the same way, in simulation and on the bench.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .grasp import (CLOSED_GAP_M, CLOSED_SPAN_M, GRIPPER_CLOSED_M, GraspParams, PlanningError, floor_point,
                    heading_to, plan_observation, plan_survey, plan_top_down_grasp)
from .perception import CupEstimate

# Floor points to look at, (x, y) in the base frame: straight ahead first, then either side and further.
LOOK_POINTS_XY = ((0.42, 0.0), (0.42, 0.15), (0.42, -0.15), (0.52, 0.0), (0.34, 0.22), (0.34, -0.22))


@dataclass
class ArmCommand:
    q: np.ndarray | None
    gripper_m: float
    state: str


@dataclass
class Timing:
    settle_s: float = 3.0
    camera_delay_s: float = 0.3        # after arriving: let the image catch up with the arm
    frame_interval_s: float = 0.15
    detect_frames: int = 8
    detections_needed: int = 3
    close_s: float = 1.5
    hold_s: float = 2.0
    arrive_tolerance_rad: float = 0.03
    stationary_window_s: float = 0.35
    stationary_rad: float = 0.004
    # How long a final waypoint may simply *stay* within tolerance instead of going still. A real arm
    # holding a pose against gravity trembles: on the bench, 2026-09-17, the arm reached its pregrasp
    # correct to 0.008 rad and then timed out for 5 s because its hold jittered by more than
    # `stationary_rad`, audibly. Being at the target and staying there is the condition that matters;
    # the spread test is only a quick way of recognising it.
    settled_s: float = 0.8
    joint_speed_rad_s: float = 1.2     # F-033, for timeouts only


class Motion:
    """Joint waypoints sent one at a time. Intermediate ones pass on arrival; the last must also be still."""

    def __init__(self, waypoints, name: str, t: float, timing: Timing, q_now):
        self.waypoints = [np.asarray(q, dtype=float) for q in waypoints]
        self.name, self.timing, self.index = name, timing, 0
        # The clock starts when the arm is first *told* where to go, not when the move is created. The
        # two are not the same instant: planning the move that follows happens in between, and a grasp
        # plan is 18 waypoints of inverse kinematics. Charging that to the arm's deadline is what made
        # the bench arm look stalled -- one command sent, "covered 0% of the move", and a timeout
        # declared before it had been asked to move at all (2026-09-17).
        self.started = self.deadline = None
        self.travel, self.within_since = 0.0, None

    def progress(self, q_fb) -> float:
        """How much of this waypoint's move the arm has covered, 0 to 1."""
        remaining = float(np.max(np.abs(np.asarray(q_fb) - self.target)))
        return 1.0 if self.travel < 1e-6 else float(np.clip(1.0 - remaining / self.travel, 0.0, 1.0))

    def _start(self, t, q_now):
        self.started = t
        self.within_since = None
        self.travel = float(np.max(np.abs(self.target - np.asarray(q_now))))
        self.deadline = (t + 3.0 + self.timing.settled_s
                         + 2.0 * self.travel / self.timing.joint_speed_rad_s)

    @property
    def target(self) -> np.ndarray:
        return self.waypoints[self.index]

    def update(self, t, q_fb, stationary: bool) -> str:
        """'moving', 'done', or 'timeout'."""
        if self.started is None:
            self._start(t, q_fb)
        error = float(np.max(np.abs(q_fb - self.target)))
        last = self.index == len(self.waypoints) - 1
        within = error < self.timing.arrive_tolerance_rad
        self.within_since = t if not within else (self.within_since if self.within_since is not None else t)
        # A trembling hold is an arrival. `stationary` is the quick way to know the arm has stopped;
        # holding inside tolerance for `settled_s` is the slow way, and it is the one a real arm under
        # load passes (bench, 2026-09-17).
        held = within and t - self.within_since >= self.timing.settled_s
        if within and (not last or stationary or held):
            if last:
                return "done"
            self.index += 1
            self._start(t, q_fb)
            return "moving"
        return "timeout" if t > self.deadline else "moving"


class PickSequence:
    def __init__(self, joints, links, camera_model, mount, perception, *, base_height_m: float,
                 look_points_xy=LOOK_POINTS_XY, grasp_params: GraspParams | None = None,
                 timing: Timing | None = None, refine_passes: int = 2, refine_min_shift_m: float = 0.004,
                 survey: bool = True):
        self.joints, self.links, self.model, self.mount, self.perception = joints, links, camera_model, mount, perception
        self.base_height = base_height_m
        self.look_points = list(look_points_xy)
        self.params = grasp_params or GraspParams()
        self.timing = timing or Timing()
        self.refine_passes, self.refine_min_shift = refine_passes, refine_min_shift_m
        self.survey = survey

        self.state, self.state_since = "settle", 0.0
        self.q_target: np.ndarray | None = None
        self.gripper = GRIPPER_CLOSED_M
        self.motion: Motion | None = None
        self.history: list[tuple[float, np.ndarray]] = []
        self.events: list[dict] = []
        self.look_index = 0
        self.surveyed = False
        self.observations: list = []
        self.cup: CupEstimate | None = None
        self.first_cup: CupEstimate | None = None
        self.plan = None
        self.refines = 0
        self.frames_tried = 0
        self.last_frame_t = -math.inf
        self.last_observation = None
        self.failure: str | None = None

    # ------------------------------------------------------------------ bookkeeping
    def log(self, t, event, **data):
        entry = {"t": round(float(t), 3), "state": self.state, "event": event}
        entry.update(data)
        self.events.append(entry)
        print(f"[pick {t:6.2f}s] {self.state}: {event}" + (f" {data}" if data else ""), flush=True)

    def _enter(self, t, state):
        self.state, self.state_since = state, t

    def _fail(self, t, reason):
        self.failure = reason
        self.log(t, "failed", reason=reason)
        self._enter(t, "failed")

    def _stationary(self, t) -> bool:
        window = [q for stamp, q in self.history if stamp >= t - self.timing.stationary_window_s]
        if len(window) < 2 or self.history[0][0] > t - self.timing.stationary_window_s:
            return False
        spread = np.max(window, axis=0) - np.min(window, axis=0)
        return float(np.max(spread)) < self.timing.stationary_rad

    def _move(self, t, waypoints, name, q_fb, next_state):
        self.motion = Motion(waypoints, name, t, self.timing, q_fb)
        self.after_motion = next_state
        self._enter(t, "moving")
        self.log(t, "move", to=name, waypoints=len(waypoints))

    def _frame_due(self, t) -> bool:
        return (t - self.state_since >= self.timing.camera_delay_s
                and t - self.last_frame_t >= self.timing.frame_interval_s)

    @property
    def done(self) -> bool:
        return self.state in ("done", "failed")

    # ------------------------------------------------------------------ the sequence
    def update(self, t: float, q_fb, up_b, frame_source) -> ArmCommand:
        q_fb = np.asarray(q_fb, dtype=float)
        up_b = np.asarray(up_b, dtype=float) / np.linalg.norm(up_b)
        self.history.append((t, q_fb))
        self.history = [(s, q) for s, q in self.history if s >= t - 2.0]

        state = self.state
        if state == "settle":
            if t - self.state_since >= self.timing.settle_s:
                self.log(t, "settled", base_height_m=round(self.base_height, 4),
                         tilt_deg=round(math.degrees(math.acos(min(1.0, abs(float(up_b[2]))))), 2))
                self._plan_observation(t, q_fb, up_b)

        elif state == "moving":
            outcome = self.motion.update(t, q_fb, self._stationary(t))
            self.q_target = self.motion.target
            if outcome == "done":
                self.log(t, "arrived", at=self.motion.name,
                         error_rad=round(float(np.max(np.abs(q_fb - self.motion.target))), 4))
                self._enter(t, self.after_motion)
            elif outcome == "timeout":
                # Say *how* it failed to arrive, not just that it did. A pose held short of the target and
                # one still crawling towards it look identical in a single worst-joint number, and the
                # bench has produced both (2026-09-17): 0.008 rad while trembling, and 0.555 rad while
                # still moving. The per-joint errors and what the arm covered separate them.
                errors = np.abs(q_fb - self.motion.target)
                covered = self.motion.progress(q_fb)
                self._fail(t, f"{self.motion.name}: no arrival by {self.motion.deadline - self.motion.started:.1f} s "
                              f"(worst joint error {float(errors.max()):.3f} rad; per joint "
                              f"{[round(float(e), 3) for e in errors]}; covered {100 * covered:.0f}% of the "
                              f"move, {'still moving' if not self._stationary(t) else 'stopped'}; "
                              f"gripper {1000 * self.gripper:.1f} mm)")

        elif state == "detect":
            if self._frame_due(t):
                self.last_frame_t = t
                self.frames_tried += 1
                frame = frame_source()
                observation = self.perception.observe(frame) if frame is not None else None
                self.last_observation = observation
                if observation is not None:
                    self.observations.append(observation)
                    self.log(t, "cup seen", confidence=round(observation.detection.confidence, 3),
                             valid_depth=round(observation.valid_depth_fraction, 3), **observation.estimate.as_dict())
                elif getattr(self.perception, "last_rejection", None):
                    # A look that found a cup but not enough of its rim. Worth a line: otherwise a run
                    # that moves through every viewpoint looks like the detector failing, when in fact it
                    # saw the cup each time and declined to measure it from there.
                    self.log(t, "look rejected", why=self.perception.last_rejection)
                if len(self.observations) >= self.timing.detections_needed:
                    self.cup = self.first_cup = consensus([o.estimate for o in self.observations])
                    self.log(t, "cup located", **self.cup.as_dict())
                    self._plan_grasp(t, q_fb, up_b)
                elif self.frames_tried >= self.timing.detect_frames:
                    self.log(t, "not enough detections", seen=len(self.observations), frames=self.frames_tried)
                    self.look_index += 1
                    self._plan_observation(t, q_fb, up_b)

        elif state == "refine":
            if self._frame_due(t):
                self.last_frame_t = t
                self.frames_tried += 1
                frame = frame_source()
                observation = self.perception.reobserve(frame, self.cup) if frame is not None else None
                self.last_observation = observation
                if observation is not None:
                    shift = _horizontal(observation.estimate.top_centre_b - self.cup.top_centre_b, up_b)
                    self.log(t, "cup seen again", method=observation.estimate.method,
                             confidence=round(observation.detection.confidence, 3),
                             label=observation.detection.label, shift_mm=[round(1000 * v, 1) for v in shift],
                             notes=observation.notes)
                    if np.linalg.norm(shift) >= self.refine_min_shift and self.refines < self.refine_passes:
                        self.refines += 1
                        self.cup = CupEstimate(self.cup.top_centre_b + shift, self.cup.radius_m, self.cup.top_height_m,
                                               self.cup.bottom_height_m, observation.estimate.points,
                                               f"refined:{observation.estimate.method}")
                        self._plan_grasp(t, q_fb, up_b)
                    else:
                        self._descend(t, q_fb)
                elif self.frames_tried >= 4:
                    self.log(t, "not seen from above; descending on the first estimate")
                    self._descend(t, q_fb)

        elif state == "close":
            # "close" is the state's name, not always the motion: an inside-out wall grasp *opens* here,
            # pressing the fingers outwards on the inside of the cup. The plan decides; the sequence only
            # knows that the gripper goes from what it held on the way down to what it holds the cup with.
            if self.gripper != self.plan.gripper_grasp_m:
                self.gripper = self.plan.gripper_grasp_m
                self.log(t, "closing" if self.plan.mode != "inside_out" else "opening onto the wall",
                         mode=self.plan.mode, finger_travel_m=round(self.gripper, 4),
                         jaw_gap_m=round(2.0 * self.gripper + CLOSED_GAP_M, 4),
                         finger_span_m=round(2.0 * self.gripper + CLOSED_SPAN_M, 4))
            if t - self.state_since >= self.timing.close_s:
                self.log(t, "closed" if self.plan.mode != "inside_out" else "pressing on the wall")
                self._move(t, self.plan.lift, "lift", q_fb, "hold")

        elif state == "hold":
            if t - self.state_since >= self.timing.hold_s:
                self.log(t, "done")
                self._enter(t, "done")

        return ArmCommand(None if self.q_target is None else self.q_target.copy(), self.gripper, self.state)

    def _plan_observation(self, t, q_fb, up_b):
        # The first look is a survey: the arm stands the camera up over the robot and tips it down at an
        # angle, holding most of a metre of floor in one frame, rather than hovering over one named point
        # at close range. Only if that finds nothing does it fall back to the close scan below, which
        # visits the floor points one at a time.
        if self.survey and not self.surveyed:
            self.surveyed = True
            heading = heading_to(floor_point(0.5, 0.0, up_b, self.base_height), up_b)
            try:
                plan = plan_survey(self.joints, self.links, self.model, self.mount, up_b, q_fb,
                                   self.base_height, heading)
            except PlanningError as exc:
                self.log(t, "no survey pose", reason=str(exc))
            else:
                self.observations, self.frames_tried = [], 0
                self.log(t, "survey", **plan.as_dict())
                self._move(t, [plan.q], "survey", q_fb, "detect")
                return
        while self.look_index < len(self.look_points):
            x, y = self.look_points[self.look_index]
            look = floor_point(x, y, up_b, self.base_height)
            try:
                plan = plan_observation(self.joints, self.links, self.model, self.mount, look, up_b, q_fb,
                                        self.base_height)
            except PlanningError as exc:
                self.log(t, "viewpoint unreachable", look_at=[x, y], reason=str(exc))
                self.look_index += 1
                continue
            self.observations, self.frames_tried = [], 0
            self.log(t, "viewpoint", **plan.as_dict())
            self._move(t, [plan.q], f"view {self.look_index}", q_fb, "detect")
            return
        self._fail(t, "no cup found from any viewpoint")

    def _plan_grasp(self, t, q_fb, up_b):
        try:
            self.plan = plan_top_down_grasp(self.joints, self.links, self.cup, up_b, q_fb, self.base_height, self.params)
        except PlanningError as exc:
            self._fail(t, f"grasp planning: {exc}")
            return
        self.gripper = self.plan.gripper_descend_m
        self.frames_tried = 0
        self.log(t, "grasp planned", **self.plan.as_dict())
        self._move(t, self.plan.approach, "pregrasp", q_fb, "refine")

    def _descend(self, t, q_fb):
        self._move(t, self.plan.descend, "descend", q_fb, "close")


def _horizontal(vector, up):
    up = np.asarray(up, dtype=float)
    return np.asarray(vector, dtype=float) - up * (np.asarray(vector) @ up)


def consensus(estimates) -> CupEstimate:
    """Median of several looks, so one bad mask cannot move the grasp."""
    centres = np.array([e.top_centre_b for e in estimates])
    return CupEstimate(np.median(centres, axis=0), float(np.median([e.radius_m for e in estimates])),
                       float(np.median([e.top_height_m for e in estimates])),
                       float(np.median([e.bottom_height_m for e in estimates])),
                       int(sum(e.points for e in estimates)), f"median of {len(estimates)}",
                       rim_coverage_deg=float(np.median([e.rim_coverage_deg or 0.0 for e in estimates])))


__all__ = ["PickSequence", "Timing", "ArmCommand", "LOOK_POINTS_XY", "consensus"]
