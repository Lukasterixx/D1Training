"""Grasp the combiner's lever in the jaws, turn it through 45 degrees and pull the door open, planned in numpy.

The jaws close across the lever, `radius_m` out from the spindle. A bar can be held from any side, so how the
gripper meets it is free: `pitch_deg` 0 comes in level, pointing at the door, one finger above the bar and
one below; 90 comes straight down with a finger either side. `plan_pull` tries `PITCHES_DEG` and keeps the
one that opens the door furthest. It closes on the 18 mm bar, and from then on the tool and the handle move
as one. Every later pose is the grasp pose carried by the handle's two joints -- the
lever turning about its spindle, the door swinging about its hinge -- so the arm only has to follow them:

    standoff (jaws open, `standoff_m` back) -> in onto the lever -> close
    -> turn to `turn_deg` (the latch lets go at 45) -> crack the door `crack_deg` with the lever held down
    -> let the lever back up, still gripped -> pull the door to `door_deg`, or as far as the arm reaches
    -> open the jaws -> back off

Letting the lever back before the long pull is what a person does, and it keeps the wrist from carrying a
52 degree roll through the whole swing. The latch model agrees: once the door is past its 0.5 degree catch,
the lever's angle no longer matters (`latch.py`).

The grip. The jaws shut to the 2 mm the real ones close to, past the URDF's 17.2 mm stop, as the pick's
pinch does (F-063): on the 18 mm bar that is the drives' 15 N. Turning the lever loads the bar along the jaw
axis (cos(pitch) of the push), and that is the grip's weak direction in simulation: the URDF gives the two
fingers independent prismatic drives, where the real jaws are one servo (servo 6) and a gear that resists a
sideways load. In the first runs one finger was pushed fully open while turning a 0.4 N·m lever, with 3-5 N
on the fingers (Week 1 log, 2026-09-19). Coupling them with a PhysX mimic joint froze Joint7_2 and set a
wrist joint drifting in this Isaac Lab, so the fingers stay independent and the grip's limit is measured,
not modelled away.

Choosing a grasp. `plan_pull` tries `PITCHES_DEG` x `ROLLS`. Given the handle's torque (`handle_torque_nm`,
what the box is known to need), it keeps the candidates whose static turning ceiling clears it by
`torque_margin`, and of those a grasp the jaws have been seen to hold (`PROVEN_GRASPS`) first, then the one
that opens the door furthest; with none strong enough, the strongest turn. Every waypoint must hold the arm up
with `hold_fraction` of each joint's published torque to spare for the handle and the door.

Pace. Each arc is streamed as fast as its joints allow (`arc_times`), up to `turn_speed_deg_s` for the lever
and `door_speed_deg_s` for the door, so the lever turns quickly where the wrist barely moves and slowly where it
has to swing.

Everything is in the Go2 base frame, from the box pose the door's tag gives (`apriltag.box_pose`).
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from demos.cup.pick_demo.grasp import JAW_CENTRE_LINK6, PlanningError, path_clear
from position_only.workspace import EFFORT_LIMIT_NM, gravity_torques

from .geometry import GEOMETRY
from .press import _seeds, _solve, press_capacity



@dataclass(frozen=True)
class PullParams:
    radius_m: float = 0.09          # where the jaws close on the lever, out from the spindle axis
    roll: int = 1                   # +1: the camera side of the wrist towards the spindle; -1: towards the lever's end
    pitch_deg: float | None = None  # how far above level the jaws come in (0 level, 90 from above); None tries
                                    # `PITCHES_DEG` x `ROLLS`, a number takes it with `roll`
    standoff_m: float = 0.10        # back along the approach, jaws open, before moving in
    turn_deg: float = 52.0          # commanded lever angle; the latch needs 45
    turn_speed_deg_s: float = 45.0  # the lever's fastest; the joints usually set a slower pace (`arc_times`)
    turn_hold_s: float = 0.3        # let the arm catch up with the end of the turn before the door moves
    crack_deg: float = 10.0         # the door opened this far with the lever still down
    door_deg: float = 60.0          # the door's target; the plan stops where the arm stops reaching
    door_min_deg: float = 20.0      # a plan that cannot open the door this far is refused
    door_speed_deg_s: float = 30.0  # the door's fastest, likewise
    open_hold_s: float = 0.5
    step_deg: float = 2.0
    # No joint is streamed faster than this along an arc. The simulated firmware restarts its plan from rest at
    # every 10 Hz setpoint (F-045), so a streamed joint covers at most ~0.07 rad a cycle; 0.6 rad/s keeps each
    # arc's joints on their path instead of trailing it -- at a fixed 15 deg/s the wrist lagged its command by
    # 0.4 rad through the turn (Week 1 log, 2026-09-19).
    joint_speed_rad_s: float = 0.6
    # Per-finger travel from the URDF's stop while coming onto the bar: a 41 mm mouth for the 18 mm bar, 11.5 mm
    # clear either side. The pick's 30 mm (a 77 mm mouth, for cups) cost 1.3 s of closing at the simulated
    # fingers' ~23 mm/s before they touched the bar.
    jaw_open_m: float = 0.012
    handle_torque_nm: float | None = None   # what the lever needs at 45 deg, if known: picks a strong enough grasp
    torque_margin: float = 1.1
    hold_fraction: float = 0.85             # each waypoint's gravity load at most this fraction of the limits

    def as_dict(self) -> dict:
        return {k: getattr(self, k) for k in self.__dataclass_fields__}


def _rz(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def _ry(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


def _rx(deg):
    a = math.radians(deg)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])


class Handle:
    """The grasped point on the lever and the tool pose holding it, for any lever and door angle."""

    def __init__(self, box_pose_b, params: PullParams | None = None, geometry=GEOMETRY):
        self.box = np.asarray(box_pose_b, dtype=float)
        self.p = params or PullParams()
        g = self.g = geometry
        self.spindle = np.array(g.spindle)
        self.hinge = np.array(g.hinge)
        axis = np.array(g.lever_axis_point)
        self.grasp0 = axis + np.array([0.0, -self.p.radius_m, 0.0])
        z = np.array([-1.0, 0.0, 0.0])                      # approach: into the door
        y = self.p.roll * np.array([0.0, 0.0, 1.0])         # jaw axis: across the lever, vertical at rest
        # Tipped about the lever's own axis by the pitch: from level towards coming down from above.
        self.tool0 = _ry(-(self.p.pitch_deg or 0.0)) @ np.column_stack([np.cross(y, z), y, z])

    def _box_frame(self, lever_deg, door_deg):
        """(rotation, point) of the grasp in the enclosure frame. The door swings outward: -Z about the hinge."""
        door = _rz(-door_deg)
        lever = _rx(lever_deg)
        point = self.spindle + lever @ (self.grasp0 - self.spindle)
        point = self.hinge + door @ (point - self.hinge)
        return door @ lever @ self.tool0, point

    def grasp(self, lever_deg: float = 0.0, door_deg: float = 0.0):
        """(position, rotation) of the jaw centre and Link6 in the base frame."""
        rot, point = self._box_frame(lever_deg, door_deg)
        return self.box[:3, :3] @ point + self.box[:3, 3], self.box[:3, :3] @ rot

    def lever_normal(self, lever_deg: float, door_deg: float = 0.0):
        """The lever's upper side's normal in the base frame: pushing along minus this turns it down."""
        a = math.radians(lever_deg)
        return self.box[:3, :3] @ _rz(-door_deg) @ np.array([0.0, -math.sin(a), math.cos(a)])

    def door_tangent(self, door_deg: float):
        """The direction the handle moves as the door opens, in the base frame."""
        radial = self._box_frame(0.0, door_deg)[1] - self.hinge
        tangent = np.cross(np.array([0.0, 0.0, -1.0]), radial)
        return self.box[:3, :3] @ (tangent / np.linalg.norm(tangent))


@dataclass
class PullPlan:
    standoff: np.ndarray
    approach: list                  # joint waypoints from the standoff onto the lever, jaws open
    turn: list                      # (lever deg, q), 0 -> turn_deg, door shut
    crack: list                     # (door deg, q), 0 -> crack_deg, lever held at turn_deg
    unturn: list                    # (lever deg, q), turn_deg -> 0, door at crack_deg
    pull: list                      # (door deg, q), crack_deg -> door_final, lever at rest
    retreat: list                   # joint waypoints backing out along the approach, jaws open
    door_final_deg: float
    capacity: list = field(default_factory=list)
    params: PullParams = field(default_factory=PullParams)
    notes: list = field(default_factory=list)

    def predicted_torque_nm(self, deg: float = GEOMETRY.handle_release_deg) -> float:
        row = min(self.capacity, key=lambda r: abs(r["lever_deg"] - deg))
        return row["handle_torque_nm"]

    def arc_seconds(self) -> dict:
        """How long each streamed arc takes at the plan's speeds (`arc_times`)."""
        p = self.params
        speeds = {"turn": p.turn_speed_deg_s, "crack": p.door_speed_deg_s, "unturn": p.turn_speed_deg_s,
                  "pull": p.door_speed_deg_s}
        return {name: round(arc_times(getattr(self, name), speed, p.joint_speed_rad_s)[-1], 2)
                for name, speed in speeds.items()}

    def as_dict(self) -> dict:
        arc = lambda rows, key: [{key: round(d, 2), "q": np.round(q, 4).tolist()} for d, q in rows]
        return {"params": self.params.as_dict(), "door_final_deg": self.door_final_deg,
                "arc_s": self.arc_seconds(),
                "standoff_q": np.round(self.standoff, 4).tolist(),
                "turn": arc(self.turn, "lever_deg"), "crack": arc(self.crack, "door_deg"),
                "unturn": arc(self.unturn, "lever_deg"), "pull": arc(self.pull, "door_deg"),
                "static_capacity": self.capacity, "notes": self.notes}


def _angles(start, stop, step):
    """start to stop inclusive, in steps of at most `step`, either direction."""
    count = max(1, int(math.ceil(abs(stop - start) / step - 1e-9)))
    return [float(v) for v in np.linspace(start, stop, count + 1)]


def _gravity_fraction(joints, links, q) -> float:
    return float(np.max(np.abs(gravity_torques(joints, links, np.asarray(q).reshape(1, 6))[0]) / EFFORT_LIMIT_NM))


def _follow(joints, links, handle, lever_degs, door_degs, q_seed):
    """(angle, q) along a path of (lever, door) pairs, each seeded by the last; stops at the first that fails
    to solve or leaves less than `hold_fraction` of the torque limits spare."""
    out = []
    for lever, door in zip(lever_degs, door_degs):
        pos, rot = handle.grasp(lever, door)
        q = _solve(joints, links, pos, rot, _seeds(pos, [q_seed]), JAW_CENTRE_LINK6)
        if q is None or _gravity_fraction(joints, links, q) > handle.p.hold_fraction:
            break
        out.append(q)
        q_seed = q
    return out


PITCHES_DEG = (0.0, 20.0, 40.0, 50.0, 60.0, 80.0)
ROLLS = (1, -1)

# Grasps the simulated jaws have held through the turn and the pull, as (pitch, roll), in order of preference. The
# static ceiling rates every candidate strong enough, so the planner cannot see why these hold and others do not:
# this is a record, not a model (Week 1 log, 2026-09-19, F-075). At 0.2-0.4 N·m:
# - pitch 40, roll -1 opened the door in every attempt that took it, 39, with the lever turned to 47.6-52.2 deg;
# - pitch 50, roll -1 in 11 of 11, with the lever at only 45.5-47.9 deg (the latch lets go at 45), so it comes
#   second;
# - every other grasp taken -- pitch 20 roll +1 five times, pitch 60 roll -1 once -- let the lever stall at
#   43-45 deg and lost the bar.
# A grasp here goes first whenever it is strong enough, even when another would open the door further.
PROVEN_GRASPS = ((40.0, -1), (50.0, -1))


def proven_rank(params) -> int:
    """How strongly the record favours this grasp: highest for the first of `PROVEN_GRASPS`, 0 for none of them."""
    key = (params.pitch_deg, params.roll)
    return len(PROVEN_GRASPS) - PROVEN_GRASPS.index(key) if key in PROVEN_GRASPS else 0


def is_proven(params) -> bool:
    return proven_rank(params) > 0


def arc_times(path, max_deg_s: float, joint_rad_s: float) -> list:
    """When a stream should reach each (angle, q) of `path`, from 0 s. Each step takes the longer of the angle at
    `max_deg_s` and its furthest-moving joint at `joint_rad_s`: quick where the arm barely moves, slow where the
    wrist has to swing (near its Joint5 = 0 singularity the lever's 2 deg steps can cost Joint4 0.1 rad)."""
    times = [0.0]
    for (a0, q0), (a1, q1) in zip(path, path[1:]):
        joint = float(np.max(np.abs(np.asarray(q1, dtype=float) - np.asarray(q0, dtype=float))))
        times.append(times[-1] + max(abs(a1 - a0) / max_deg_s, joint / joint_rad_s))
    return times


def standoff_guess(joints, links, box_pose_b, q_seed, params: PullParams | None = None):
    """Where the arm will stand off the lever for a grasp that has held (`PROVEN_GRASPS`, or the one `params`
    fixes), from a first box estimate and before any full plan: something for the close look to face. None if
    none solves."""
    from dataclasses import replace

    p = params or PullParams()
    grasps = PROVEN_GRASPS if p.pitch_deg is None else ((p.pitch_deg, p.roll),)
    for pitch, roll in grasps:
        pos, rot = Handle(box_pose_b, replace(p, pitch_deg=pitch, roll=roll)).grasp(0.0, 0.0)
        standoff = pos - p.standoff_m * rot[:, 2]
        q = _solve(joints, links, standoff, rot, _seeds(standoff, [q_seed]), np.asarray(JAW_CENTRE_LINK6))
        if q is not None:
            return q
    return None


def plan_pull(joints, links, box_pose_b, q_start, params: PullParams | None = None) -> PullPlan:
    """The first of `plan_pull_candidates`: a grasp that has held if one is strong enough, else the one that opens
    the door furthest; `PlanningError` with each candidate's reason if none solves."""
    return plan_pull_candidates(joints, links, box_pose_b, q_start, params)[0]


def plan_pull_candidates(joints, links, box_pose_b, q_start, params: PullParams | None = None) -> list:
    """Every grasp that solves, best first (see `plan_pull`), so a sequence can fall back to the next when
    the arm cannot get onto the lever with the first: the planner models the trunk, not every contact."""
    from dataclasses import replace

    p = params or PullParams()
    pitches = PITCHES_DEG if p.pitch_deg is None else (p.pitch_deg,)
    rolls = ROLLS if p.pitch_deg is None else (p.roll,)
    plans, reasons = [], []
    for pitch in pitches:
        for roll in rolls:
            try:
                plans.append(_plan_one(joints, links, box_pose_b, q_start, replace(p, pitch_deg=pitch, roll=roll)))
            except PlanningError as error:
                reasons.append(f"pitch {pitch:.0f}, roll {roll:+d}: {error}")
    if not plans:
        raise PlanningError("no grasp solves -- " + "; ".join(reasons))
    need = None if p.handle_torque_nm is None else p.torque_margin * p.handle_torque_nm
    is_strong = [need is None or plan.predicted_torque_nm() >= need for plan in plans]
    strong = [plan for plan, ok in zip(plans, is_strong) if ok]
    weak = [plan for plan, ok in zip(plans, is_strong) if not ok]
    ranked = (sorted(strong, key=lambda plan: (proven_rank(plan.params), plan.door_final_deg,
                                               plan.predicted_torque_nm()), reverse=True)
              + sorted(weak, key=lambda plan: (plan.predicted_torque_nm(), plan.door_final_deg), reverse=True))
    if not strong:
        ranked[0].notes.append(f"no grasp's static ceiling reaches {need:.2f} N·m; took the strongest, "
                               f"{ranked[0].predicted_torque_nm():.2f}")
    for rank, plan in enumerate(ranked):
        plan.notes.append(f"pitch {plan.params.pitch_deg:.0f} deg, roll {plan.params.roll:+d}"
                          f"{' (a grasp that has held)' if is_proven(plan.params) else ''}: choice {rank + 1} of "
                          f"{len(ranked)} that solve ({len(pitches) * len(rolls)} tried, "
                          f"{len(strong)} strong enough for the handle)")
    return ranked


def _plan_one(joints, links, box_pose_b, q_start, p: PullParams) -> PullPlan:
    """One grasp's waypoints for grasp, turn, crack, unturn and pull, or `PlanningError` naming what fails."""
    handle = Handle(box_pose_b, p)
    offset = np.asarray(JAW_CENTRE_LINK6)
    pos0, rot0 = handle.grasp(0.0, 0.0)
    approach_axis = rot0[:, 2]
    standoff = pos0 - p.standoff_m * approach_axis
    q_standoff = _solve(joints, links, standoff, rot0, _seeds(standoff, [q_start]), offset)
    if q_standoff is None:
        raise PlanningError(f"no arm pose holds the open jaws {100 * p.standoff_m:.0f} cm in front of the lever")
    if not path_clear(joints, q_start, q_standoff)[0]:
        raise PlanningError("the move to the lever's standoff crosses the trunk proxy")
    approach, q_prev = [], q_standoff
    for s in np.linspace(0.0, 1.0, 4)[1:]:
        target = (1 - s) * standoff + s * pos0
        q = _solve(joints, links, target, rot0, _seeds(target, [q_prev]), offset)
        if q is None:
            raise PlanningError("the jaws cannot come in straight onto the lever")
        approach.append(q)
        q_prev = q

    turn_angles = _angles(0.0, p.turn_deg, p.step_deg)
    turn = _follow(joints, links, handle, turn_angles, [0.0] * len(turn_angles), q_prev)
    if len(turn) < len(turn_angles):
        raise PlanningError(f"the turn does not solve past {turn_angles[len(turn) - 1] if turn else 0:.0f} deg")
    crack_angles = _angles(0.0, p.crack_deg, p.step_deg)
    crack = _follow(joints, links, handle, [p.turn_deg] * len(crack_angles), crack_angles, turn[-1])
    if len(crack) < len(crack_angles):
        raise PlanningError(f"the door cannot be cracked {p.crack_deg:.0f} deg with the lever held down")
    unturn_angles = _angles(p.turn_deg, 0.0, p.step_deg)
    unturn = _follow(joints, links, handle, unturn_angles, [p.crack_deg] * len(unturn_angles), crack[-1])
    if len(unturn) < len(unturn_angles):
        raise PlanningError("the lever cannot be let back up with the door ajar")
    pull_angles = _angles(p.crack_deg, p.door_deg, p.step_deg)
    pull = _follow(joints, links, handle, [0.0] * len(pull_angles), pull_angles, unturn[-1])
    door_final = pull_angles[len(pull) - 1] if pull else p.crack_deg
    if door_final < p.door_min_deg:
        raise PlanningError(f"the arm reaches the handle only to {door_final:.0f} deg of door, "
                            f"short of {p.door_min_deg:.0f}")
    notes = [] if door_final >= p.door_deg else [f"door opened to {door_final:.0f} deg, where the arm stops reaching"]
    pos_end, rot_end = handle.grasp(0.0, door_final)
    retreat, q_prev = [], pull[-1]
    for s in (0.5, 1.0):
        target = pos_end - s * p.standoff_m * rot_end[:, 2]
        q = _solve(joints, links, target, rot_end, _seeds(target, [q_prev]), offset)
        if q is None:
            break
        retreat.append(q)
        q_prev = q
    capacity = []
    for deg, q in zip(turn_angles, turn):
        pos, _ = handle.grasp(deg, 0.0)
        ceiling = press_capacity(joints, links, q, pos, -handle.lever_normal(deg))
        capacity.append({"lever_deg": round(deg, 1), "force_n": round(ceiling["force_n"], 2),
                         "handle_torque_nm": round(ceiling["force_n"] * p.radius_m, 3),
                         "limiting_joint": ceiling["limiting_joint"]})
    return PullPlan(q_standoff, approach, list(zip(turn_angles, turn)), list(zip(crack_angles, crack)),
                    list(zip(unturn_angles, unturn)), list(zip(pull_angles[:len(pull)], pull)), retreat, door_final,
                    capacity, p, notes)


__all__ = ["PullParams", "PullPlan", "Handle", "plan_pull", "plan_pull_candidates", "arc_times", "PROVEN_GRASPS",
           "is_proven", "proven_rank", "standoff_guess"]
