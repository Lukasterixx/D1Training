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

    settle -> observe (move, detect; scan other floor points if nothing) -> plan grasp -> pregrasp
    -> refine (look again from above, re-plan if the cup is elsewhere) -> descend -> close -> lift -> hold
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .grasp import (CLOSED_GAP_M, GRIPPER_CLOSED_M, GRIPPER_OPEN_M, GraspParams, PlanningError, floor_point,
                    grip_travel_m, plan_observation, plan_top_down_grasp)
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
    joint_speed_rad_s: float = 1.2     # F-033, for timeouts only


class Motion:
    """Joint waypoints sent one at a time. Intermediate ones pass on arrival; the last must also be still."""

    def __init__(self, waypoints, name: str, t: float, timing: Timing, q_now):
        self.waypoints = [np.asarray(q, dtype=float) for q in waypoints]
        self.name, self.timing, self.index = name, timing, 0
        self._start(t, q_now)

    def _start(self, t, q_now):
        self.started = t
        travel = float(np.max(np.abs(self.target - np.asarray(q_now))))
        self.deadline = t + 3.0 + 2.0 * travel / self.timing.joint_speed_rad_s

    @property
    def target(self) -> np.ndarray:
        return self.waypoints[self.index]

    def update(self, t, q_fb, stationary: bool) -> str:
        """'moving', 'done', or 'timeout'."""
        error = float(np.max(np.abs(q_fb - self.target)))
        last = self.index == len(self.waypoints) - 1
        if error < self.timing.arrive_tolerance_rad and (not last or stationary):
            if last:
                return "done"
            self.index += 1
            self._start(t, q_fb)
            return "moving"
        return "timeout" if t > self.deadline else "moving"


class PickSequence:
    def __init__(self, joints, links, camera_model, mount, perception, *, base_height_m: float,
                 look_points_xy=LOOK_POINTS_XY, grasp_params: GraspParams | None = None,
                 timing: Timing | None = None, refine_passes: int = 2, refine_min_shift_m: float = 0.004):
        self.joints, self.links, self.model, self.mount, self.perception = joints, links, camera_model, mount, perception
        self.base_height = base_height_m
        self.look_points = list(look_points_xy)
        self.params = grasp_params or GraspParams()
        self.timing = timing or Timing()
        self.refine_passes, self.refine_min_shift = refine_passes, refine_min_shift_m

        self.state, self.state_since = "settle", 0.0
        self.q_target: np.ndarray | None = None
        self.gripper = GRIPPER_CLOSED_M
        self.motion: Motion | None = None
        self.history: list[tuple[float, np.ndarray]] = []
        self.events: list[dict] = []
        self.look_index = 0
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
                self._fail(t, f"{self.motion.name}: no arrival by {self.motion.deadline - self.motion.started:.1f} s "
                              f"(worst joint error {float(np.max(np.abs(q_fb - self.motion.target))):.3f} rad)")

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
            if self.gripper == GRIPPER_OPEN_M:
                self.gripper = grip_travel_m(2.0 * self.cup.radius_m, self.params.squeeze_m)
                self.log(t, "closing", finger_travel_m=round(self.gripper, 4),
                         jaw_gap_m=round(2.0 * self.gripper + CLOSED_GAP_M, 4))
            if t - self.state_since >= self.timing.close_s:
                self.log(t, "closed")
                self._move(t, self.plan.lift, "lift", q_fb, "hold")

        elif state == "hold":
            if t - self.state_since >= self.timing.hold_s:
                self.log(t, "done")
                self._enter(t, "done")

        return ArmCommand(None if self.q_target is None else self.q_target.copy(), self.gripper, self.state)

    def _plan_observation(self, t, q_fb, up_b):
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
        self.gripper = GRIPPER_OPEN_M
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
