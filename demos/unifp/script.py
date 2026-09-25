"""The demos as a clock-driven sequence of tool-tip goals and jaw commands.

This is the high-level task sequence the plan puts above the controller
(`docs/thesis_b_plan.md`: "a programmed high-level task sequence supplying pose and force
commands"). It holds no simulator handle and no policy. Each phase names where the **jaw centre**
should be, how far open the jaws should be, and -- for the runs allowed to servo the wrist -- which
way the hand should face; the environment turns that into the goal UniFP is actually commanded and
into joint targets for the two finger drives.

Two conventions, both taken from the scripted demos so the two are comparable:

  * **Every arc runs on a clock, not on arrival** (F-076). Against a stiff spring or a heavy cup
    the arm falls short, and how far short is the measurement. A phase that waited for arrival
    would convert a tracking failure into a longer run and hide it.
  * **The sequence never reads the thing it is working.** The lever's angle, the latch, the door
    and the cup's pose are not inputs here. Phases that follow the lever down or the door out
    follow the *commanded* arc, exactly as `demos/combiner/pull.py` does. The runner reads the
    truth from the simulator afterwards and records the gap.

Goals are interpolated in Cartesian space. UniFP's own generator interpolates in the spherical
coordinates the command is expressed in, and over these short moves the two differ by millimetres,
but it is a difference: a demo path is a task-space straight line, not an arc of the goal sphere.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Callable

from demos.cup.pick_demo.grasp import GRIPPER_OPEN_M
from demos.combiner.geometry import GEOMETRY
from demos.combiner.sequence import GRIP_SHUT_M
from demos.combiner.pull import PullParams

from . import props

Vec3 = tuple[float, float, float]

#: Where the arm is asked to wait before and after a task, in the robot's yaw frame. This is
#: UniFP's own episode-opening goal (`EE_GOAL_INIT_START`, radius 0.50 at pitch 36 degrees) in
#: Cartesian form, so the demo starts the policy on the goal every training episode starts it on --
#: and it is above the table rather than through it.
HOME_POINT: Vec3 = (0.50 * math.cos(math.pi / 5), 0.0,
                    0.49 + 0.50 * math.sin(math.pi / 5))

#: Jaw travel per finger. The gap between the pads is `CLOSED_GAP_M + 2 * travel`.
JAW_WIDE_M = GRIPPER_OPEN_M              # 77.2 mm gap: the pick's open jaws, over a 55 mm cup
JAW_LEVER_OPEN_M = PullParams().jaw_open_m   # 41.2 mm gap: the scripted grip's opening on the bar
JAW_CUP_SHUT_M = 0.0                     # the URDF's stop; the 55 mm cup arrests the pads long before
JAW_BAR_SHUT_M = GRIP_SHUT_M             # past the stop, to squeeze the 18 mm bar (F-063)


def lerp(a: Vec3, b: Vec3, u: float) -> Vec3:
    return (a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u)


def offset(point: Vec3, direction: Vec3, distance: float) -> Vec3:
    return (point[0] - direction[0] * distance,
            point[1] - direction[1] * distance,
            point[2] - direction[2] * distance)


@dataclass(frozen=True)
class Phase:
    """One timed leg of a demo.

    `target` gives the point the jaw centre should reach by the end, interpolated from wherever
    the previous phase left it. `path` replaces that with an explicit function of the phase's
    progress, for the arcs that follow a lever or a door. Exactly one of the two is given.
    """
    name: str
    duration_s: float
    target: Callable[[dict], Vec3] | None = None
    path: Callable[[dict, float], Vec3] | None = None
    jaw_m: float = JAW_WIDE_M
    approach: Callable[[dict], Vec3] | None = None
    object_axis: Callable[[dict], Vec3] | None = None
    #: Phases whose point the runner should score arrival against. A transit phase is not a
    #: tracking measurement -- the goal is moving and the tip is meant to be behind it.
    measure: bool = False
    #: Whether a `--wrist` run should hold the hand's orientation through this phase. Off while
    #: the arm is still crossing the room: the servo costs tracking, and the only orientation
    #: that matters is the one the jaws close from. On from the standoff hold onward, so the
    #: wrist turns and the policy re-converges before the fine approach rather than during it.
    servo: bool = False


@dataclass
class Command:
    """What the environment should do this step."""
    point: Vec3
    jaw_m: float
    approach: Vec3
    object_axis: Vec3
    servo: bool
    phase: str
    progress: float
    elapsed_s: float
    finished: bool


@dataclass
class DemoScript:
    """Runs a phase list against a clock. One instance per attempt."""
    phases: list[Phase]
    index: int = 0
    elapsed_s: float = 0.0
    _start_point: Vec3 | None = None
    _start_jaw: float | None = None
    log: list[dict] = field(default_factory=list)

    def reset(self, point: Vec3, jaw_m: float) -> None:
        self.index, self.elapsed_s = 0, 0.0
        self._start_point, self._start_jaw = point, jaw_m
        self.log = []

    @property
    def finished(self) -> bool:
        return self.index >= len(self.phases)

    @property
    def total_s(self) -> float:
        return sum(phase.duration_s for phase in self.phases)

    def phase_end_point(self, phase: Phase, state: dict) -> Vec3:
        return phase.path(state, 1.0) if phase.path is not None else phase.target(state)

    def update(self, dt: float, state: dict) -> Command:
        """Advance the clock and return this step's command.

        Called once per policy step. Phases roll over inside a single call rather than skipping a
        step, so a phase shorter than `dt` still has its endpoint issued.
        """
        if self.finished:
            phase = self.phases[-1]
            point = self.phase_end_point(phase, state)
            return Command(point, phase.jaw_m, self._approach(phase, state),
                           self._object_axis(phase, state), phase.servo, "done", 1.0,
                           self.elapsed_s, True)

        self.elapsed_s += dt
        while not self.finished:
            phase = self.phases[self.index]
            if self.elapsed_s <= phase.duration_s or phase.duration_s <= 0.0:
                break
            self.elapsed_s -= phase.duration_s
            self._start_point = self.phase_end_point(phase, state)
            self._start_jaw = phase.jaw_m
            self.log.append({"phase": phase.name, "end_point_m": list(self._start_point)})
            self.index += 1
        if self.finished:
            return self.update(0.0, state)

        phase = self.phases[self.index]
        u = 1.0 if phase.duration_s <= 0 else min(1.0, self.elapsed_s / phase.duration_s)
        if phase.path is not None:
            point = phase.path(state, u)
        else:
            point = lerp(self._start_point, phase.target(state), u)
        # The jaws ramp with the phase rather than stepping: the finger drives are position
        # controlled and a step target closes them as fast as the drive allows, which on the bar
        # is a hammer blow rather than a grip.
        jaw = self._start_jaw + (phase.jaw_m - self._start_jaw) * u
        return Command(point, jaw, self._approach(phase, state), self._object_axis(phase, state),
                       phase.servo, phase.name, u, self.elapsed_s, False)

    def _approach(self, phase: Phase, state: dict) -> Vec3:
        return phase.approach(state) if phase.approach is not None else (1.0, 0.0, 0.0)

    def _object_axis(self, phase: Phase, state: dict) -> Vec3:
        return phase.object_axis(state) if phase.object_axis is not None else (0.0, 0.0, 1.0)


# --- the two demos ------------------------------------------------------------------------------

@dataclass(frozen=True)
class CupTiming:
    stand_s: float = 1.5
    approach_s: float = 2.0
    #: Hold at the standoff before going in. A transit leaves the tip behind its goal -- measured
    #: at 3 cm on a 1.3 s move, against 1 cm once the goal stops -- and the jaws clear a 55 mm cup
    #: by 11 mm a side, so entering with the transit's error still on the hand sweeps the cup over.
    #: These two holds are what the position error has to come out of.
    settle_s: float = 2.0
    enter_s: float = 2.5
    arrive_s: float = 1.2
    close_s: float = 0.8
    lift_s: float = 2.0
    hold_s: float = 1.5
    #: How far back along the approach the hand waits before coming in.
    standoff_m: float = 0.10


def cup_phases(timing: CupTiming = CupTiming()) -> list[Phase]:
    """Reach a cup standing on a table, close on its wall and lift it.

    An *outside* grasp: the jaws open wider than the 55 mm cup and close across it. F-065's wall
    grasp -- descending shut and opening against the inside -- exists for a cup too wide for the
    jaws, and is not what this cup needs.
    """
    site = lambda st: st["cup_site"]
    grasp = lambda st: site(st).grasp_point_m
    approach = lambda st: site(st).approach_unit
    axis = lambda st: site(st).cup_axis_unit
    return [
        Phase("stand", timing.stand_s, target=lambda st: HOME_POINT, jaw_m=JAW_WIDE_M,
              approach=approach, object_axis=axis),
        Phase("approach", timing.approach_s, jaw_m=JAW_WIDE_M, approach=approach, object_axis=axis,
              target=lambda st: offset(grasp(st), approach(st), timing.standoff_m)),
        Phase("settle", timing.settle_s, jaw_m=JAW_WIDE_M, approach=approach, object_axis=axis,
              target=lambda st: offset(grasp(st), approach(st), timing.standoff_m), measure=True, servo=True),
        Phase("enter", timing.enter_s, target=grasp, jaw_m=JAW_WIDE_M,
              approach=approach, object_axis=axis, servo=True),
        Phase("arrive", timing.arrive_s, target=grasp, jaw_m=JAW_WIDE_M,
              approach=approach, object_axis=axis, measure=True, servo=True),
        Phase("close", timing.close_s, target=grasp, jaw_m=JAW_CUP_SHUT_M,
              approach=approach, object_axis=axis, measure=True, servo=True),
        Phase("lift", timing.lift_s, target=lambda st: site(st).lift_point_m, jaw_m=JAW_CUP_SHUT_M,
              approach=approach, object_axis=axis, servo=True),
        Phase("hold", timing.hold_s, target=lambda st: site(st).lift_point_m, jaw_m=JAW_CUP_SHUT_M,
              approach=approach, object_axis=axis, measure=True, servo=True),
    ]


@dataclass(frozen=True)
class CombinerTiming:
    stand_s: float = 1.5
    approach_s: float = 2.0
    settle_s: float = 2.0
    enter_s: float = 2.5
    arrive_s: float = 1.2
    grip_s: float = 0.8
    turn_s: float = 2.5
    crack_s: float = 1.2
    ease_s: float = 0.8
    pull_s: float = 3.0
    release_s: float = 0.8
    back_s: float = 1.5
    standoff_m: float = 0.12
    #: Lever angle commanded. Past `GEOMETRY.handle_release_deg` (45) as the scripted pull's is,
    #: so arriving a few degrees short still releases the latch.
    turn_deg: float = props.LEVER_TURN_DEG
    #: Door angle the lever is held down through, before the lever is let back up.
    crack_deg: float = 10.0
    #: Lever angle the hand eases back to while still holding the bar and the door.
    ease_deg: float = 5.0
    door_deg: float = props.DOOR_OPEN_DEG


def combiner_phases(timing: CombinerTiming = CombinerTiming()) -> list[Phase]:
    """Grip the combiner's lever, turn it past the latch, and pull the door open.

    The same four moves the scripted grip-and-pull makes (`demos/combiner/pull.py`): onto the bar,
    turn it down, crack the door with the lever still down, then let the lever up and pull. What
    is different is only what drives the arm there.
    """
    site = lambda st: st["box_site"]
    approach = lambda st, door=0.0: site(st).approach_unit(door)
    axis = lambda st, handle=0.0, door=0.0: site(st).lever_axis_unit(handle, door)
    at = lambda st, handle=0.0, door=0.0: site(st).grasp_point_m(handle, door)
    return [
        Phase("stand", timing.stand_s, target=lambda st: HOME_POINT, jaw_m=JAW_LEVER_OPEN_M,
              approach=lambda st: approach(st), object_axis=lambda st: axis(st)),
        Phase("approach", timing.approach_s, jaw_m=JAW_LEVER_OPEN_M,
              approach=lambda st: approach(st), object_axis=lambda st: axis(st),
              target=lambda st: offset(at(st), approach(st), timing.standoff_m)),
        Phase("settle", timing.settle_s, jaw_m=JAW_LEVER_OPEN_M, measure=True, servo=True,
              approach=lambda st: approach(st), object_axis=lambda st: axis(st),
              target=lambda st: offset(at(st), approach(st), timing.standoff_m)),
        Phase("enter", timing.enter_s, target=lambda st: at(st), jaw_m=JAW_LEVER_OPEN_M, servo=True,
              approach=lambda st: approach(st), object_axis=lambda st: axis(st)),
        Phase("arrive", timing.arrive_s, target=lambda st: at(st), jaw_m=JAW_LEVER_OPEN_M,
              approach=lambda st: approach(st), object_axis=lambda st: axis(st),
              measure=True, servo=True),
        Phase("grip", timing.grip_s, target=lambda st: at(st), jaw_m=JAW_BAR_SHUT_M,
              approach=lambda st: approach(st), object_axis=lambda st: axis(st),
              measure=True, servo=True),
        # Down the lever's arc. The grasp point moves in as well as down, because the lever turns
        # about its spindle rather than sliding.
        Phase("turn", timing.turn_s, jaw_m=JAW_BAR_SHUT_M, servo=True,
              path=lambda st, u: at(st, handle=timing.turn_deg * u),
              approach=lambda st: approach(st),
              object_axis=lambda st: axis(st, handle=timing.turn_deg)),
        # Crack the door with the lever still held past the release, so the latch cannot re-engage.
        Phase("crack", timing.crack_s, jaw_m=JAW_BAR_SHUT_M, servo=True,
              path=lambda st, u: at(st, handle=timing.turn_deg, door=timing.crack_deg * u),
              approach=lambda st: approach(st, timing.crack_deg),
              object_axis=lambda st: axis(st, timing.turn_deg, timing.crack_deg)),
        # Let the lever back up while keeping hold of it: pulling a door by a lever held at 52
        # degrees drags the jaws along the bar.
        Phase("ease", timing.ease_s, jaw_m=JAW_BAR_SHUT_M, servo=True,
              path=lambda st, u: at(st, handle=timing.turn_deg + (timing.ease_deg - timing.turn_deg) * u,
                                    door=timing.crack_deg),
              approach=lambda st: approach(st, timing.crack_deg),
              object_axis=lambda st: axis(st, timing.ease_deg, timing.crack_deg)),
        Phase("pull", timing.pull_s, jaw_m=JAW_BAR_SHUT_M, measure=True, servo=True,
              path=lambda st, u: at(st, handle=timing.ease_deg,
                                    door=timing.crack_deg + (timing.door_deg - timing.crack_deg) * u),
              approach=lambda st: approach(st, timing.door_deg),
              object_axis=lambda st: axis(st, timing.ease_deg, timing.door_deg)),
        Phase("release", timing.release_s, jaw_m=JAW_LEVER_OPEN_M, servo=True,
              target=lambda st: at(st, handle=timing.ease_deg, door=timing.door_deg),
              approach=lambda st: approach(st, timing.door_deg),
              object_axis=lambda st: axis(st, timing.ease_deg, timing.door_deg)),
        # Back to the ready pose rather than straight out of the open door: with the door at
        # `door_deg` its normal points across the robot, and retreating along it walks the goal
        # out of the trained yaw range.
        Phase("back", timing.back_s, jaw_m=JAW_LEVER_OPEN_M, target=lambda st: HOME_POINT,
              approach=lambda st: approach(st, timing.door_deg),
              object_axis=lambda st: axis(st, timing.ease_deg, timing.door_deg)),
    ]


def latch_would_release(turn_deg: float) -> bool:
    """Whether a lever commanded to `turn_deg` clears the latch, if the arm gets it there."""
    return turn_deg >= GEOMETRY.handle_release_deg
