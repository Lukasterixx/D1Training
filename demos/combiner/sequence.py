"""Find the door's AprilTag, then open the door by its handle (or only push the lever): a state machine.

Like the cup pick (`demos/cup/pick_demo/sequence.py`), it holds no simulator or hardware handle. Each
`update` takes the time, the arm's *feedback* angles and world up from the IMU, and returns a joint
target; frames are pulled through a callable only when a state needs one. Moves between poses reuse the
pick's `Motion`: one waypoint at a time, passed on arrival.

    settle -> search (look from each stop until the tag is seen `detections_needed` times)
    -> close look (straight at the tag, `close_views_m`) -> refine (the box pose the plan uses)

then, with `method="pull"` (the default; `pull.py`):

    -> standoff, jaws open -> in onto the lever (if the arm cannot get there, back to the close look and
    the next grasp, `max_grasps` in all) -> align (take out what the joints fall short of the plan,
    from feedback) -> grip (jaws shut on the bar)
    -> turn (lever to `turn_deg`) -> turn_hold -> crack (door `crack_deg` open, lever still down)
    -> unturn (lever back up, still gripped) -> pull (door to the plan's final angle) -> open_hold
    -> let_go (jaws open) -> back off -> done

or with `method="press"` (`press.py`, F-071's runs):

    -> standoff -> over the lever -> onto it -> press (the lever commanded along its arc)
    -> hold -> release (back along the arc) -> clear -> done

Every arc -- the lever turning, the door swinging -- is streamed on a clock, not on arrival: the spring
and the door resist, and the arm is meant to be commanded past where it can get. Nothing here knows the lever's angle or whether the latch let go -- on
the robot that is what a second tag on the lever, or the door moving, would tell it. The runner measures
it from the simulator and scores the attempt.
"""
from __future__ import annotations

import math

import numpy as np

from demos.cup.pick_demo.camera import camera_pose
from demos.cup.pick_demo.grasp import CLOSED_GAP_M, GRIPPER_CLOSED_M, GraspParams, PlanningError
from demos.cup.pick_demo.sequence import ArmCommand, Motion, Timing

from .apriltag import box_pose, mean_pose, pose_error
from .press import PressParams, SearchParams, plan_close_look, plan_press, plan_search
from .pull import PullParams, arc_times, plan_pull_candidates, standoff_guess

# What "shut on the lever" commands: the jaws closing to the 2 mm the real ones reach (the pick's
# `pinch_closed_gap_m`, stated not measured), past the URDF's 17.2 mm stop. On the 18 mm bar they stall at the
# drives' limit. The runner opens the simulated stop to allow it (F-063).
GRIP_SHUT_M = min(0.0, (GraspParams().pinch_closed_gap_m - CLOSED_GAP_M) / 2.0)


class TagObservation:
    def __init__(self, box_pose_b, detection, camera_pose_b, stamp_s):
        self.box_pose_b, self.detection, self.camera_pose_b, self.stamp_s = box_pose_b, detection, camera_pose_b, stamp_s


class TurnSequence:
    def __init__(self, joints, links, mount, detector, *, method: str = "pull", press: PressParams | None = None,
                 pull: PullParams | None = None, search: SearchParams | None = None, timing: Timing | None = None,
                 detections_needed: int = 3, detect_frames: int = 8, empty_frames: int = 3, grip_s: float = 0.8,
                 release_s: float = 0.6, align_tol_rad: float = 0.003, align_wait_s: float = 0.4,
                 align_steps: int = 3, max_grasps: int = 3):
        """`empty_frames`: a search stop is left after this many frames without the tag at all (a stop that sees
        it keeps looking to `detect_frames`). `grip_s` and `release_s`: how long the jaws are given to shut on the
        bar from `PullParams.jaw_open_m`, and to open off it -- the simulated fingers' ~23 mm/s, not the real
        jaws', which are not measured."""
        if method not in ("pull", "press"):
            raise ValueError(f"method must be 'pull' or 'press', not {method!r}")
        self.joints, self.links, self.mount, self.detector = joints, links, mount, detector
        self.method, self.grip_s, self.release_s = method, grip_s, release_s
        self.align_tol_rad, self.align_wait_s, self.align_steps = align_tol_rad, align_wait_s, align_steps
        # Added to every joint target from the grasp on: what the joints fall short of the plan at the lever
        # (a steady offset under load in the simulated arm, the sag F-015 saw), measured from feedback.
        self.bias = np.zeros(6)
        self.aligned = 0
        self.max_grasps = max_grasps
        self.candidates: list = []
        self.grasps_tried = 0
        self.q_close: np.ndarray | None = None
        self.press_params = press or PressParams()
        self.pull_params = pull or PullParams()
        self.search_params = search or SearchParams()
        self.timing = timing or Timing()
        self.detections_needed, self.detect_frames, self.empty_frames = detections_needed, detect_frames, empty_frames

        self.state, self.state_since = "settle", 0.0
        self.q_target: np.ndarray | None = None
        self.motion: Motion | None = None
        self.after_motion: str | None = None
        self.history: list = []
        self.events: list[dict] = []
        self.stops: list = []
        self.stop_index = -1
        self.look_name: str | None = None
        self.observations: list[TagObservation] = []
        self.frames_tried = 0
        self.last_frame_t = -math.inf
        self.last_detections: list = []
        self.box_first: np.ndarray | None = None     # from the search look
        self.box: np.ndarray | None = None           # from the close look; the press is planned on it
        self.found_by: str | None = None
        self.plan = None
        self.commanded_lever_deg: float | None = None
        self.commanded_door_deg: float | None = None
        self.gripper = GRIPPER_CLOSED_M
        self.phase_times: dict[str, float] = {}
        self.failure: str | None = None

    # ------------------------------------------------------------------ bookkeeping
    def log(self, t, event, **data):
        entry = {"t": round(float(t), 3), "state": self.state, "event": event}
        entry.update(data)
        self.events.append(entry)
        print(f"[turn {t:6.2f}s] {self.state}: {event}" + (f" {data}" if data else ""), flush=True)

    def _enter(self, t, state):
        self.state, self.state_since = state, t
        self.phase_times.setdefault(state, round(float(t), 3))

    def _fail(self, t, reason):
        self.failure = reason
        self.log(t, "failed", reason=reason)
        self._enter(t, "failed")

    @property
    def done(self) -> bool:
        return self.state in ("done", "failed")

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

    def _command(self):
        return ArmCommand(None if self.q_target is None else self.q_target.copy(), self.gripper, self.state)

    # ------------------------------------------------------------------ looking
    def _observe(self, frame) -> TagObservation | None:
        detections = self.detector.detect(frame.rgb)
        self.last_detections = detections
        if not detections:
            return None
        detection = min(detections, key=lambda d: d.reprojection_px)
        cam = camera_pose(self.joints, frame.q, self.mount)
        return TagObservation(box_pose(cam, detection), detection, cam, frame.stamp_s)

    def _next_stop(self, t, q_fb):
        self.stop_index += 1
        if self.stop_index >= len(self.stops):
            self._fail(t, f"tag {self.detector.tag_id} not seen from any of {len(self.stops)} search stops "
                          f"({[h for h, _ in self.stops]} deg)")
            return
        heading, q = self.stops[self.stop_index]
        self.look_name = f"search {heading:+.0f} deg"
        self._move(t, [q], self.look_name, q_fb, "detect")

    def _looking(self, t, frame_source):
        if t - self.state_since < self.timing.camera_delay_s or t - self.last_frame_t < self.timing.frame_interval_s:
            return None
        self.last_frame_t = t
        self.frames_tried += 1
        frame = frame_source()
        observation = self._observe(frame) if frame is not None else None
        if observation is not None:
            self.observations.append(observation)
            d = observation.detection
            self.log(t, "tag seen", look=self.look_name, range_m=round(d.range_m, 3), side_px=round(d.side_px, 1),
                     reprojection_px=round(d.reprojection_px, 3))
        if len(self.observations) >= self.detections_needed:
            return mean_pose([o.box_pose_b for o in self.observations])
        # Searching, a stop with nothing in view at all is left early; one that has seen the tag keeps looking.
        empty = self.state == "detect" and not self.observations and self.frames_tried >= self.empty_frames
        if self.frames_tried >= self.detect_frames or empty:
            self.log(t, "not enough detections", look=self.look_name, seen=len(self.observations),
                     frames=self.frames_tried)
            return False
        return None

    def _reset_look(self):
        self.observations, self.frames_tried, self.last_frame_t = [], 0, -math.inf

    # ------------------------------------------------------------------ the sequence
    def update(self, t: float, q_fb, up_b, frame_source) -> ArmCommand:
        q_fb = np.asarray(q_fb, dtype=float)
        up_b = np.asarray(up_b, dtype=float) / np.linalg.norm(up_b)
        self.history.append((t, q_fb))
        self.history = [(s, q) for s, q in self.history if s >= t - 2.0]
        state = self.state

        if state == "settle":
            if self.q_target is None:
                self.q_target = q_fb.copy()
            if t - self.state_since >= self.timing.settle_s:
                self.stops = plan_search(self.joints, self.links, self.mount, up_b, q_fb, self.search_params)
                self.log(t, "settled", search_stops_deg=[h for h, _ in self.stops])
                self._next_stop(t, q_fb)

        elif state == "moving":
            outcome = self.motion.update(t, q_fb, self._stationary(t))
            self.q_target = self.motion.target
            if outcome == "done":
                self.log(t, "arrived", at=self.motion.name,
                         error_rad=round(float(np.max(np.abs(q_fb - self.motion.target))), 4))
                self._enter(t, self.after_motion)
                self._reset_look()
            elif outcome == "timeout":
                errors = np.abs(q_fb - self.motion.target)
                reason = (f"{self.motion.name}: no arrival (worst joint error {float(errors.max()):.3f} rad; "
                          f"per joint {[round(float(e), 3) for e in errors]})")
                if (self.after_motion == "at_handle" and self.candidates and self.grasps_tried < self.max_grasps
                        and self.q_close is not None):
                    # Something the planner does not model is in the way. Back to where the plan was made,
                    # and try the next grasp from there.
                    self.log(t, "cannot get onto the lever; backing off for the next grasp", why=reason)
                    self.gripper = GRIPPER_CLOSED_M
                    self._move(t, [self.q_close], "back to the close look", q_fb, "next_grasp")
                else:
                    self._fail(t, reason)

        elif state == "detect":
            result = self._looking(t, frame_source)
            if result is False:
                self._next_stop(t, q_fb)
            elif result is not None:
                self.box_first, self.found_by = result, self.look_name
                self.log(t, "tag located", found_by=self.found_by, box_position_b=np.round(result[:3, 3], 4).tolist())
                # Gripping, the close look turns the camera to face the way the grasp will hold the wrist, which
                # otherwise swings up to 2.2 rad between the two.
                toward = (standoff_guess(self.joints, self.links, result, q_fb, self.pull_params)
                          if self.method == "pull" else None)
                q_close = plan_close_look(self.joints, self.links, self.mount, result, up_b, q_fb, self.search_params,
                                          toward=toward)
                if q_close is None:
                    self.log(t, "no close look solves; planning from the search look")
                    self.box = result
                    self._plan_task(t, q_fb)
                else:
                    self.look_name = "close look"
                    self.q_close = q_close
                    self._move(t, [q_close], "close look", q_fb, "refine")

        elif state == "refine":
            result = self._looking(t, frame_source)
            if result is False:
                self.log(t, "close look lost the tag; planning from the search look")
                self.box = self.box_first
                self._plan_task(t, q_fb)
            elif result is not None:
                self.box = result
                self.log(t, "tag refined", shift=pose_error(result, self.box_first))
                self._plan_task(t, q_fb)

        elif state == "at_lever":
            self._enter(t, "press")
            self.log(t, "pushing", final_deg=self.plan.params.final_deg, speed_deg_s=self.plan.params.speed_deg_s)

        elif state == "press":
            p = self.plan.params
            self.commanded_lever_deg = min(p.final_deg, (t - self.state_since) * p.speed_deg_s)
            self.q_target = self._arc_q(self.commanded_lever_deg)
            if self.commanded_lever_deg >= p.final_deg:
                self._enter(t, "hold")
                self.log(t, "holding at the end of the push", lever_deg_commanded=p.final_deg, hold_s=p.hold_s)

        elif state == "hold":
            if t - self.state_since >= self.plan.params.hold_s:
                self._enter(t, "release")
                self.log(t, "releasing")

        elif state == "release":
            p = self.plan.params
            self.commanded_lever_deg = max(0.0, p.final_deg - (t - self.state_since) * p.speed_deg_s)
            self.q_target = self._arc_q(self.commanded_lever_deg)
            if self.commanded_lever_deg <= 0.0:
                self.commanded_lever_deg = None
                self._move(t, [self.plan.approach[-1], self.plan.standoff], "clear of the lever", q_fb, "done")

        elif state == "next_grasp":
            self._start_grasp(t, q_fb)

        elif state == "at_handle":
            self._enter(t, "align")

        elif state == "align":
            # Feedback runs at ~9 Hz and the arm needs a moment after each change: judge only after a wait.
            if t - self.state_since >= self.align_wait_s:
                goal = self.plan.approach[-1]
                error = goal - q_fb
                if float(np.max(np.abs(error))) > self.align_tol_rad and self.aligned < self.align_steps:
                    self.bias = np.clip(self.bias + error, -0.05, 0.05)
                    self.aligned += 1
                    self.q_target = goal + self.bias
                    self.state_since = t
                    self.log(t, "correcting the joints' shortfall at the lever",
                             shortfall_rad=[round(float(e), 4) for e in error], step=self.aligned)
                else:
                    self.gripper = GRIP_SHUT_M
                    self._enter(t, "grip")
                    self.log(t, "shutting the jaws on the lever", gripper_m=round(GRIP_SHUT_M, 4),
                             residual_rad=round(float(np.max(np.abs(error))), 4),
                             bias_rad=[round(float(b), 4) for b in self.bias])

        elif state == "grip":
            if t - self.state_since >= self.grip_s:
                self._enter(t, "turn")
                self.log(t, "turning the lever", to_deg=self.plan.params.turn_deg)

        elif state == "turn":
            self.commanded_lever_deg, self.q_target, done = self._stream(self.plan.turn, self.plan.params.turn_speed_deg_s, t)
            if done:
                self._enter(t, "turn_hold")

        elif state == "turn_hold":
            if t - self.state_since >= self.plan.params.turn_hold_s:
                self._enter(t, "crack")
                self.log(t, "cracking the door open", to_deg=self.plan.params.crack_deg)

        elif state == "crack":
            self.commanded_door_deg, self.q_target, done = self._stream(self.plan.crack, self.plan.params.door_speed_deg_s, t)
            if done:
                self._enter(t, "unturn")
                self.log(t, "letting the lever back up")

        elif state == "unturn":
            self.commanded_lever_deg, self.q_target, done = self._stream(self.plan.unturn, self.plan.params.turn_speed_deg_s, t)
            if done:
                self._enter(t, "pull")
                self.log(t, "pulling the door open", to_deg=self.plan.door_final_deg)

        elif state == "pull":
            self.commanded_door_deg, self.q_target, done = self._stream(self.plan.pull, self.plan.params.door_speed_deg_s, t)
            if done:
                self._enter(t, "open_hold")

        elif state == "open_hold":
            if t - self.state_since >= self.plan.params.open_hold_s:
                self.gripper = self.plan.params.jaw_open_m
                self._enter(t, "let_go")
                self.log(t, "letting go of the handle")

        elif state == "let_go":
            if t - self.state_since >= self.release_s:
                self.commanded_lever_deg = self.commanded_door_deg = None
                if self.plan.retreat:
                    self._move(t, [q + self.bias for q in self.plan.retreat], "back off the handle", q_fb, "done")
                else:
                    self._enter(t, "done")

        elif state == "done" and "done" not in [e["event"] for e in self.events]:
            self.log(t, "done")

        return self._command()

    def _stream(self, path, speed_deg_s, t):
        """(commanded angle, waypoint, finished) along `path` of (angle, q) since this state began, at up to
        `speed_deg_s` and the plan's joint speed (`pull.arc_times`).

        The waypoint is the last one the commanded angle has reached, whichever way the path runs."""
        times = arc_times(path, speed_deg_s, self.plan.params.joint_speed_rad_s)
        elapsed = t - self.state_since
        if elapsed >= times[-1]:
            return path[-1][0], path[-1][1] + self.bias, True
        i = max(k for k, stamp in enumerate(times) if stamp <= elapsed + 1e-9)
        share = (elapsed - times[i]) / (times[i + 1] - times[i]) if times[i + 1] > times[i] else 1.0
        angle = path[i][0] + share * (path[i + 1][0] - path[i][0])
        return angle, path[i][1] + self.bias, False

    def _arc_q(self, deg):
        """The last arc waypoint at or before `deg`: the command steps along the arc as the angle grows."""
        chosen = self.plan.arc[0][1]
        for angle, q in self.plan.arc:
            if angle <= deg + 1e-9:
                chosen = q
        return chosen.copy()

    def _plan_task(self, t, q_fb):
        if self.method == "press":
            self._plan_press(t, q_fb)
            return
        try:
            self.candidates = plan_pull_candidates(self.joints, self.links, self.box, q_fb, self.pull_params)
        except PlanningError as error:
            self._fail(t, f"pull: {error}")
            return
        self._start_grasp(t, q_fb)

    def _start_grasp(self, t, q_fb):
        if not self.candidates or self.grasps_tried >= self.max_grasps:
            self._fail(t, f"pull: none of the {self.grasps_tried} grasps tried could get onto the lever")
            return
        self.plan = self.candidates.pop(0)
        self.grasps_tried += 1
        self.log(t, "grasp planned", pitch_deg=self.plan.params.pitch_deg, roll=self.plan.params.roll,
                 door_final_deg=self.plan.door_final_deg, static_ceiling_at_45_deg_nm=self.plan.predicted_torque_nm(),
                 attempt=self.grasps_tried, notes=self.plan.notes)
        self.gripper = self.plan.params.jaw_open_m
        # The standoff was planned from the pose the first plan started at; the close look is near enough
        # for the joint move, and `path_clear` was checked from there.
        self._move(t, [self.plan.standoff, *self.plan.approach], "onto the lever, jaws open", q_fb, "at_handle")

    def _plan_press(self, t, q_fb):
        try:
            self.plan = plan_press(self.joints, self.links, self.box, q_fb, self.press_params)
        except PlanningError as error:
            self._fail(t, f"press: {error}")
            return
        ceiling = self.plan.predicted_torque_nm()
        self.log(t, "press planned", arc_waypoints=len(self.plan.arc),
                 static_ceiling_at_45_deg_nm=ceiling,
                 limiting_joint=min(self.plan.capacity, key=lambda r: abs(r["lever_deg"] - 45))["limiting_joint"])
        self._move(t, [self.plan.standoff, *self.plan.approach, *self.plan.contact], "onto the lever", q_fb, "at_lever")


__all__ = ["TurnSequence", "TagObservation"]
