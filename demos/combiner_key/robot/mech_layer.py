"""The held-mechanism (T1) policy without torch, for the dog: the reach to the handle, when to grasp it, the reference
along its path, the force law and the goal held on the handle. numpy only. Nothing here has run on the dog.

`export_policy.py --task mechanism` writes an S4b or S4c checkpoint (`unifp_train.mech_env.Go2D1MechEnv`: UniFP's network and
actor observation, 18 outputs, trained from `66600` with the law in the loop) into one `.npz` with every constant this
module needs, read from the training code (`mech_cfg`, `hook_cfg`, `unifp_isaaclab.interface`, `mech_env`'s source) and
the training run's `run.json`, so the dog's side has none of its own. The network is `leg_policy.LegPolicy`'s, the
observation's history `leg_policy.LegObservation`'s, the arm's forward kinematics `insert_policy.ArmKinematics`'s.

What the environment does around the network each 50 Hz step, as `run_unifp_train.py mech_eval --force_law
--force_law_bleed 1.0` evaluates it, in its order, and what this module does in its place:

  1. `update` (every step but a reset's; `Go2D1MechEnv._get_rewards`): the step count; the grasp, once the count has
     passed the approach's slide plus a delay (15 steps at evaluation) and the tool has slowed below 0.10 m/s, or 50
     steps after that regardless (`_engage_due`), and, with the export's `grasp_within_m` (S4c), only with the tool
     within that of its goal (the grasp paragraph below); the reference along the path (`mechanism.TravelGoal` on the
     evaluation's plan: one target, the whole travel, at 0.10 m/s, held to the end), stepped once grasped; the goal:
     before the grasp UniFP's slide from its opening goal to the handle's placement (`EeGoalTrajectory` in `HeldGoals`),
     after it the handle at the measured travel `s`, in the goal sphere of the base now (`_hold_goals_on_path` with the
     law on); the roll channels, (sin 2 phi, cos 2 phi) at phi = 0, if the policy trained with the roll objective.
  2. `frame`: the 76-wide frame -- legs and IMU as `LegObservation` builds them, the arm's reported angles and their
     report-difference rates, the previous actions, the gait phase pinned at 0, and the commands: zero velocity, the
     goal, the roll, the law's force in the base's yaw frame (computed by the previous step's `targets`). A reset's frame
     has all-zero commands.
  3. The policy acts on the stack (`LegPolicy.act`).
  4. `targets` (`_pre_physics_step`, `_step_forces`): the 12 leg targets and the arm's six (default + 0.25 x action; the
     arm's clamped to the training limits as `reach_arm.ArmTargets` clamps what goes on the wire, which the simulator's
     targets are not); and the law on the latest update's measurement: lag = ref - s while grasped, the integral with
     anti-windup (zeroed whenever it would push against the lag) and, with `bleed_s`, decaying by exp(-dt / bleed_s) a
     step while |lag| < `arrived_m`; F = clamp(kp lag + ki integral, +-max) along the path's tangent at `s`, rotated
     into the base's yaw frame, for the next frame. So a frame's force is one step older than its goal, as in training.

**Every input is explicit**: the handle's placement at the reset (goal-sphere coordinates as the evaluation's plan gives
them, or a world point); its path (`MechPath`: a slide or a hinge, from perception on the dog); the measured travel `s`;
the base's pose (`base_pos`, `base_quat`) in a gravity-aligned world frame, z up and z = 0 the ground under the base, the
frame the path is in (the goal sphere's centre is the base's x, y at that z = 0, plus 0.49 m up, `interface`); the
tool's speed, for the grasp's timing, and with the arrival gate its position; the IMU, the legs and the arm's reports. On the dog those come from perception,
odometry and the D1's link; in `check_mech.py` from an Isaac rollout, and in `mech_gate.py` from recorded stands.

**How the dog would measure `s`** (`measure_travel`; **not validated on the dog**). In simulation the law reads the
mechanism's true `s` (`Mechanism.s`), which the dog has no sensor for. The dog would take the controlled point (Link6
plus `tool_offset_m`, the jaw centre) by the forward kinematics of the arm's reported joints (`ArmKinematics.tool`),
place it in the world by the base's pose (position from `base_odometry.LegOdometry`, attitude from the IMU), and project
it onto the perceived path (`MechPath.project`): along a slide's direction from its start, or as the arc length of its
angle about a hinge's axis, clamped to the travel. What that reading carries that the simulator's does not:
  - the grasp is not the handle. In simulation the handle trails the tool by the grasp spring's stretch (2,000 N/m at
    evaluation: 1 cm per 20 N along the path), so a hand-based `s` reads ahead of the handle's and the law asks less;
    on the dog the claw's and the handle's own play take that place, unmeasured;
  - the reports: 9 Hz, 77 ms behind, quantised to 0.1 deg, and blind to the links' flex under load;
  - the base estimate: feet that slip, the IMU's yaw drift (`base_odometry`'s list);
  - the path's perception: an error in the perceived path moves `s` along it; the distance off it is returned to show
    when the hand and the path disagree;
  - the CAD jaw centre, not a point measured on the arm (F-013's caveat for the CAD tip).
`check_mech.py` measures the first three against an Isaac rollout (the stretch, the reports' lag, the odometry); the
last two need the dog.

**The tool's speed** for the grasp's timing is the simulator's true speed of the controlled point. `tool_speed_estimate`
is what the reports give: the forward kinematics' change along the reported rates, the arm's part only (the base's own
motion is left out), and the rates are report differences at 9 Hz. Not validated.

**The grasp on the dog is not the simulator's.** There the claw is a 3-D spring put wherever the tool is when the rule
fires, and the mechanism is built around that point, so a grasp cannot miss. On the dog the handle is where it is: the
jaws have to be shut on it, which takes time (`reach_arm.CLOTH_CLOSE_S` is 2 s), and the path starts at the perceived
handle, not at the tool. `MechRunner.update(grasp_now=...)` lets the dog program make the grasp when the jaws are shut,
after the rule (`engage_due`) says the tool has settled; left None, the grasp is the rule's, as in training.

**The arrival gate** (S4c, `mech_env`'s `grasp_within_m`, `--mech_grasp_within_m`; the export's
`approach.grasp_within_m`, None for S4b and before, and then the rule is the settle rule alone, bit for bit; `MechLayer`
takes another). With it the rule, the fallback included, also needs the tool within that radius of its goal, as
`_engage_due` measures it: against `_goal_world()` when the rule runs, which is the goal the policy was given on the
frame it just acted on (before this step's slide), placed in the world by the goal sphere of the base now. `update` takes
the controlled point in the path's world frame (`tool_pos`; on the dog the forward kinematics of the reports placed by
the base's estimate, `MechRunner.hand`) and measures it against its own goal so (`tool_to_goal`), or the distance itself
(`tool_to_goal_m`, a replay that has the environment's), compared in float32 with `<=` as the environment compares it.
Where the two part: once UniFP's hold ends before a grasp the environment's goal moves on (not reproduced, below), and
its gate then measures the tool against that moving goal, not the handle, so an S4c grasp in simulation can close well
away from the handle (`check_mech.py`'s `grasps_after_goal_moved_on`); this layer's goal, and so its gate, stays on
the handle.

The layer's reference and integral are float32 and computed in float32 as the environment's tensors are, so a threshold
(|lag| < `arrived_m`, the anti-windup's sign, the gate's radius) flips on the same step; the geometry is float64. Checked by
`tests/test_mech_layer.py` (against `mechanism.TravelGoal`, `Mechanism`'s path, `EeGoalTrajectory` and `mech_env`'s
`_step_forces`, `_hold_goals_on_path` and `_engage_due` compiled from its source) and `check_mech.py --sim` (a recorded
Isaac rollout). Not reproduced: UniFP's goal resampling before the grasp (the environment's goal moves on if the hold
ends before the tool settles; this layer holds the handle), the other kinds of training draws, a torn grasp (an episode's
end in training; on the dog a lost grip ends the run).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from demos.combiner_key.robot import insert_policy, leg_policy, reach_arm

F32 = np.float32


def _f32(x) -> np.float32:
    return np.float32(x)


# --- the path ----------------------------------------------------------------------------------------------------------

@dataclass
class MechPath:
    """A handle's path in the world frame (`mechanism.Mechanism`'s geometry, float64): a slide, ``start + s direction``,
    or a hinge, ``centre + cos(s/r) radial + sin(s/r) (axis x radial)`` with ``radial = start - centre``, ``r = |radial|``;
    `s` in metres from closed, 0 to `travel`."""
    hinge: bool
    start: np.ndarray
    direction: np.ndarray
    axis: np.ndarray
    radial: np.ndarray
    centre: np.ndarray
    radius: float
    travel: float

    @classmethod
    def from_grasp(cls, hinge: bool, point, opening, axis, radius: float, travel: float) -> "MechPath":
        """As `Mechanism.grasp` builds it around the handle at `point`: `opening` the handle's first motion; for a hinge,
        `axis` perpendicular to it, the centre ``point - radius (opening x axis)``, so the swing starts along `opening`."""
        point = np.asarray(point, dtype=float)
        opening = np.asarray(opening, dtype=float)
        opening = opening / np.linalg.norm(opening)
        axis = np.asarray(axis, dtype=float)
        axis = axis / max(float(np.linalg.norm(axis)), 1e-9)
        radial = float(radius) * np.cross(opening, axis)
        return cls(bool(hinge), point, opening, axis, radial, point - radial, float(radius) if hinge else 1.0,
                   float(travel))

    @classmethod
    def slide(cls, start, opening, travel: float) -> "MechPath":
        return cls.from_grasp(False, start, opening, (0.0, 0.0, 1.0), 1.0, travel)

    @classmethod
    def swing(cls, start, opening, axis, radius: float, travel: float) -> "MechPath":
        return cls.from_grasp(True, start, opening, axis, radius, travel)

    def _angle(self, s: float) -> float:
        return float(s) / max(self.radius, 1e-6)

    def position(self, s: float) -> np.ndarray:
        """The handle at `s` (`mechanism.path_position`)."""
        if not self.hinge:
            return self.start + float(s) * self.direction
        a = self._angle(s)
        return self.centre + math.cos(a) * self.radial + math.sin(a) * np.cross(self.axis, self.radial)

    def tangent(self, s: float) -> np.ndarray:
        """The unit direction of increasing `s` at `s` (`Mechanism.tangent`)."""
        if not self.hinge:
            return self.direction.copy()
        a = self._angle(s)
        return (-math.sin(a) * self.radial + math.cos(a) * np.cross(self.axis, self.radial)) / max(self.radius, 1e-6)

    def project(self, point) -> tuple[float, float, float]:
        """(s, the distance off the path, s unclamped) of the path's point nearest `point`: along a slide's direction from
        its start, or the arc length of `point`'s angle about a hinge's axis from `radial`. `s` is clamped to the travel,
        as the mechanism's stops clamp it, and the distance is to the path's point at that `s`. Geometry only; how good a
        reading of the handle it is depends on what `point` is (`measure_travel`)."""
        p = np.asarray(point, dtype=float)
        if not self.hinge:
            raw = float((p - self.start) @ self.direction)
        else:
            v = p - self.centre
            r = max(self.radius, 1e-6)
            raw = r * math.atan2(float(v @ np.cross(self.axis, self.radial)) / r, float(v @ self.radial) / r)
        s = min(max(raw, 0.0), self.travel)
        return s, float(np.linalg.norm(p - self.position(s))), raw


# --- the dog's measurements (not validated on the dog) ---------------------------------------------------------------

def hand_point(kin: insert_policy.ArmKinematics, arm_q, base_pos, base_quat, tool_offset) -> np.ndarray:
    """The controlled point (Link6 + `tool_offset`) in the world, by the forward kinematics of `arm_q` (URDF radians)
    placed by the base's pose."""
    _, p_b = kin.tool(np.asarray(arm_q, dtype=float)[None], np.asarray(tool_offset, dtype=float))
    return np.asarray(base_pos, dtype=float) + insert_policy.quat_apply(base_quat, p_b[0])


def measure_travel(path: MechPath, kin: insert_policy.ArmKinematics, arm_q, base_pos, base_quat,
                   tool_offset) -> tuple[float, float]:
    """The dog's reading of the travel: the hand (`hand_point` of the reported joints, the base's estimated pose)
    projected on the perceived path. Returns (s, the hand's distance off the path, m). Not validated on the dog; see the
    module's docstring for what it carries that the simulator's true `s` does not."""
    s, off, _ = path.project(hand_point(kin, arm_q, base_pos, base_quat, tool_offset))
    return s, off


def tool_speed_estimate(kin: insert_policy.ArmKinematics, arm_q, arm_dq, base_quat, tool_offset,
                        h: float = 1e-3) -> float:
    """The controlled point's speed from the arm's reports, m/s: the forward kinematics' change along the reported rates
    (report differences), rotated into the world by the IMU's attitude. The arm's part only: the base's own velocity and
    rotation are left out. Not validated."""
    q, dq = np.asarray(arm_q, dtype=float), np.asarray(arm_dq, dtype=float)
    _, p0 = kin.tool(q[None], np.asarray(tool_offset, dtype=float))
    _, p1 = kin.tool((q + h * dq)[None], np.asarray(tool_offset, dtype=float))
    return float(np.linalg.norm(insert_policy.quat_apply(base_quat, (p1[0] - p0[0]) / h)))


# --- the pieces of the environment's step -----------------------------------------------------------------------------

def goal_sphere_centre(base_pos, base_quat, centre_offset) -> np.ndarray:
    """`unifp_isaaclab.task.goal_sphere_center`: the base's x, y at z = 0, plus the offset in the base's yaw frame."""
    base = np.asarray(base_pos, dtype=float)
    return np.array([base[0], base[1], 0.0]) + insert_policy.quat_apply(insert_policy.yaw_quat(base_quat),
                                                                        np.asarray(centre_offset, dtype=float))


def world_to_sphere(point, base_pos, base_quat, centre_offset) -> np.ndarray:
    """A world point as the policy's goal: (radius, pitch, yaw) of the point less the goal sphere's centre, in the base's
    yaw frame (`interface.cart2sphere` of `quat_rotate_inverse(base_yaw_quat, point - centre)`)."""
    local = insert_policy.quat_rotate_inverse(insert_policy.yaw_quat(base_quat),
                                              np.asarray(point, dtype=float)
                                              - goal_sphere_centre(base_pos, base_quat, centre_offset))
    return leg_policy.cart_to_sphere(local)


def sphere_to_world(sphere, base_pos, base_quat, centre_offset) -> np.ndarray:
    """The inverse of `world_to_sphere`: where a goal-sphere coordinate is in the world for the base's pose."""
    return goal_sphere_centre(base_pos, base_quat, centre_offset) + insert_policy.quat_apply(
        insert_policy.yaw_quat(base_quat), leg_policy.sphere_to_cart(*(float(v) for v in sphere)))


def _lerp(start: np.ndarray, end: np.ndarray, weight: np.float32) -> np.ndarray:
    """`torch.lerp` in float32, as its kernels compute it (the two halves of the weight's range differently)."""
    if weight < F32(0.5):
        return (start + weight * (end - start)).astype(F32)
    return (end - (end - start) * (F32(1.0) - weight)).astype(F32)


class ApproachGoal:
    """`EeGoalTrajectory` for one robot from a reset, as `Go2D1MechEnv` aims it at a handle: from UniFP's opening goal
    (`interface.EE_GOAL_INIT_START`) to the handle's placement over `traj_steps` policy steps, then held there. float32.
    UniFP's resample once the hold ends (`total_steps`) is not reproduced: this goal stays on the handle."""

    def __init__(self, init_start):
        self.init_start = np.asarray(init_start, dtype=F32)
        self.reset(self.init_start, 1.0)

    def reset(self, goal_sphere, traj_steps: float) -> None:
        self.start = self.init_start.copy()
        self.goal = np.asarray(goal_sphere, dtype=F32).copy()
        self.current = self.start.copy()
        self.timer = F32(0.0)
        self.traj_steps = F32(traj_steps)

    def step(self) -> np.ndarray:
        t = F32(min(max(self.timer / self.traj_steps, F32(0.0)), F32(1.0)))
        self.current = _lerp(self.start, self.goal, t)
        self.timer = F32(self.timer + F32(1.0))
        return self.current


class TravelPlan:
    """`mechanism.TravelGoal` under `use_plan(fraction, speed)` for one robot: once started, a reference `ref` along the
    path moving at `speed` m/s toward `fraction` of the travel and held there to the end. float32."""

    def __init__(self, fraction: float, speed_m_s: float, dt: float):
        self.fraction, self.speed, self.dt = F32(fraction), F32(speed_m_s), F32(dt)
        self.ref = self.target = F32(0.0)
        self.started = False

    def reset(self) -> None:
        self.ref = self.target = F32(0.0)
        self.started = False

    def start(self, travel: float) -> None:
        self.ref = F32(0.0)
        self.target = F32(self.fraction * F32(travel))
        self.started = True

    def step(self, active: bool) -> np.float32:
        if active:
            change = F32(min(max(F32(self.target - self.ref), F32(-1.0)), F32(1.0)))
            most = F32(self.speed * self.dt)
            change = F32(max(min(change, most), F32(-most)))
            self.ref = F32(self.ref + change)
        return self.ref


class ForceLaw:
    """`Go2D1MechEnv._step_forces`'s law for one robot, float32: lag = ref - s while held; the integral of the lag, zeroed
    whenever it would push against the lag (anti-windup); with `bleed_s`, decaying by exp(-dt / bleed_s) a step while
    |lag| < `arrived_m`; F = clamp(kp lag + ki integral, -max, max), N along the path's tangent."""

    def __init__(self, kp: float, ki: float, max_n: float, bleed_s: float | None, arrived_m: float, dt: float):
        self.kp, self.ki, self.max = F32(kp), F32(ki), F32(max_n)
        self.bleed_s = None if not bleed_s else float(bleed_s)
        self.decay = None if self.bleed_s is None else F32(math.exp(-float(dt) / self.bleed_s))
        self.arrived, self.dt = F32(arrived_m), F32(dt)
        self.reset()

    def reset(self) -> None:
        self.integral = F32(0.0)
        self.lag = F32(0.0)
        self.force = F32(0.0)

    def step(self, ref, s, held: bool) -> np.float32:
        h = F32(1.0) if held else F32(0.0)
        lag = F32(F32(F32(ref) - F32(s)) * h)
        integral = F32(self.integral + F32(lag * self.dt))
        if F32(integral * lag) < F32(0.0):
            integral = F32(0.0)
        if self.decay is not None and abs(lag) < self.arrived:
            integral = F32(integral * self.decay)
        self.integral = F32(integral * h)
        force = F32(F32(self.kp * lag) + F32(self.ki * self.integral))
        self.lag, self.force = lag, F32(max(min(force, self.max), F32(-self.max)))
        return self.force


# --- the layer ---------------------------------------------------------------------------------------------------------

class MechLayer:
    """The task layer of one held mechanism, from the export's `mechanism` block: the approach's goal, the grasp's rule,
    the travel plan, the law and the goal on the handle, stepped in the environment's order (`update`, then `law` after
    the policy has acted). `plan_fraction`, `speed_m_s`, `gains` (kp, ki, max) and `bleed_s` default to the export's
    evaluation values; `engage_delay_steps` to the evaluation's delay; `grasp_within_m` (the arrival gate's radius, m;
    None: off) to the export's `approach.grasp_within_m`, off when the export has none."""

    def __init__(self, mech: dict, *, plan_fraction: float | None = None, speed_m_s: float | None = None,
                 gains=None, bleed_s: float | None | str = "export", engage_delay_steps: int | None = None,
                 grasp_within_m: float | None | str = "export", dt: float = 0.02):
        self.m = mech
        law, plan, approach = mech["law"]["evaluation"], mech["plan"], mech["approach"]
        kp, ki, mx = (law["kp"], law["ki"], law["max_n"]) if gains is None else (float(g) for g in gains)
        self.law_cfg = {"kp": float(kp), "ki": float(ki), "max_n": float(mx),
                        "bleed_s": law["bleed_s"] if bleed_s == "export" else bleed_s}
        self.dt = float(dt)
        self.law = ForceLaw(kp, ki, mx, self.law_cfg["bleed_s"], law["arrived_m"], self.dt)
        self.plan = TravelPlan(plan["fraction"] if plan_fraction is None else plan_fraction,
                               plan["speed_m_s"] if speed_m_s is None else speed_m_s, self.dt)
        self.approach = ApproachGoal(approach["init_start"])
        self.engage_delay = int(approach["engage_delay_steps"] if engage_delay_steps is None else engage_delay_steps)
        self.settle_speed = F32(approach["settle_speed_m_s"])
        self.max_wait = int(approach["max_wait_steps"])
        within = approach.get("grasp_within_m") if grasp_within_m == "export" else grasp_within_m
        #: The arrival gate's radius, float32 as the environment compares it; None: off (the settle rule alone).
        self.grasp_within = None if within is None else F32(within)
        self.centre_offset = np.asarray(mech["goal_sphere"]["centre_offset"], dtype=float)
        self.roll_channels = np.asarray(mech["commands"]["roll_value"], dtype=float)
        self.reset(mech["approach"]["init_start"], 2.0 / self.dt, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))

    def reset(self, goal_sphere, traj_steps: float, base_pos, base_quat) -> None:
        """A new episode: the approach from UniFP's opening goal to `goal_sphere` (the handle's placement) over
        `traj_steps` steps (a float, as `EeGoalTrajectory` draws it), the grasp due `int(traj_steps) + delay` steps in."""
        self.approach.reset(goal_sphere, traj_steps)
        self.engage_step = int(F32(traj_steps)) + self.engage_delay
        self.since_reset = 0
        self.grasped = self.ever_grasped = False
        self.grasp_step: int | None = None
        self.rule_due = False
        self.settle_due = False
        self.tool_to_goal_m: float | None = None
        self.path: MechPath | None = None
        self.s = F32(0.0)
        self.off_path = 0.0
        self.base_pos, self.base_quat = np.asarray(base_pos, dtype=float), np.asarray(base_quat, dtype=float)
        self.goal = np.asarray(self.approach.current, dtype=float).copy()
        self.hold_goal = np.zeros(3)
        self.plan.reset()
        self.law.reset()
        self.force_w = np.zeros(3)
        self.force_b = np.zeros(3)

    def settle_rule(self, tool_speed: float) -> bool:
        """`_engage_due`'s settle rule, with this step's count: past the approach and the delay, and the tool slower than
        the settle speed, or the longest wait over (the whole rule without the arrival gate)."""
        waited = self.since_reset - self.engage_step
        return (not self.ever_grasped and waited >= 0
                and (F32(tool_speed) < self.settle_speed or waited >= self.max_wait))

    def within_reach(self, tool_to_goal_m: float | None) -> bool:
        """The arrival gate: the tool within `grasp_within` of its goal, compared in float32 (`<=`, as `_engage_due`);
        always True with the gate off. A distance is needed only with the gate on."""
        if self.grasp_within is None:
            return True
        if tool_to_goal_m is None:
            raise ValueError("the arrival gate (grasp_within_m) needs the tool's position or its distance to the goal")
        return bool(F32(tool_to_goal_m) <= self.grasp_within)

    def engage_due(self, tool_speed: float, tool_to_goal_m: float | None = None) -> bool:
        """`_engage_due`'s rule, with this step's count: the settle rule and, with the gate on, the tool within its radius
        of its goal (`tool_to_goal_m`, m, needed until the grasp)."""
        if self.grasp_within is not None and not self.ever_grasped and tool_to_goal_m is None:
            raise ValueError("the arrival gate (grasp_within_m) needs the tool's distance to its goal until the grasp")
        return self.settle_rule(tool_speed) and self.within_reach(tool_to_goal_m)

    def goal_world(self, base_pos=None, base_quat=None) -> np.ndarray:
        """The layer's goal now (`goal`, goal-sphere coordinates) in the world for the base's pose (default the latest
        update's): `_goal_world()`."""
        return sphere_to_world(self.goal, self.base_pos if base_pos is None else base_pos,
                               self.base_quat if base_quat is None else base_quat, self.centre_offset)

    def tool_to_goal(self, tool_pos, base_pos=None, base_quat=None) -> float:
        """The controlled point's distance from `goal_world`, m. Called inside `update` before the goal steps, so with the
        goal the policy was just given and the base now, as `_engage_due` measures it."""
        return float(np.linalg.norm(np.asarray(tool_pos, dtype=float) - self.goal_world(base_pos, base_quat)))

    def update(self, base_pos, base_quat, tool_speed: float, s=None, path: MechPath | None = None,
               grasp_now: bool | None = None, tool_pos=None, tool_to_goal_m: float | None = None) -> None:
        """One step's layer (`_get_rewards`' part), with the state after the step: the base's pose, the tool's speed, the
        measured travel `s` (read once grasped) and the path (needed at the grasp). `grasp_now` None grasps when the rule
        says (training's); True or False is the caller's grasp (the dog's jaws shut, or a replay's), the rule still
        evaluated into `rule_due`. With the arrival gate on, the rule also needs `tool_pos` (the controlled point in the
        path's world frame, measured against this layer's goal: `tool_to_goal`) or `tool_to_goal_m` (the distance itself,
        used in place of `tool_pos`'s) on every step until the grasp; with it off they decide nothing. The distance is
        kept in `tool_to_goal_m` (None without either)."""
        self.base_pos, self.base_quat = np.asarray(base_pos, dtype=float), np.asarray(base_quat, dtype=float)
        self.since_reset += 1
        if tool_to_goal_m is None and tool_pos is not None:
            tool_to_goal_m = self.tool_to_goal(tool_pos)          # the goal not yet stepped: the one just acted on
        self.tool_to_goal_m = None if tool_to_goal_m is None else float(tool_to_goal_m)
        if self.grasp_within is not None and not self.ever_grasped and self.tool_to_goal_m is None:
            raise ValueError("the arrival gate (grasp_within_m) needs tool_pos or tool_to_goal_m on every step until the "
                             "grasp")
        self.settle_due = self.settle_rule(tool_speed)
        self.rule_due = self.settle_due and self.within_reach(self.tool_to_goal_m)
        grasp = self.rule_due if grasp_now is None else (bool(grasp_now) and not self.ever_grasped)
        if grasp:
            if path is None:
                raise ValueError("a grasp needs the handle's path")
            self.path = path
            self.plan.start(path.travel)
            self.grasped = self.ever_grasped = True
            self.grasp_step = self.since_reset
        if self.grasped:
            if s is None:
                raise ValueError("a held handle needs its measured travel s")
            self.s = F32(s)
        self.plan.step(self.grasped)
        current = self.approach.step()
        if self.grasped:
            point = self.path.position(float(self.s))
            self.hold_goal = world_to_sphere(point, self.base_pos, self.base_quat, self.centre_offset)
            self.goal = self.hold_goal.copy()
        else:
            self.goal = np.asarray(current, dtype=float).copy()

    def law_step(self) -> np.ndarray:
        """The law on the latest update's state (`_step_forces`, after the policy acts): the force command for the next
        frame, N in the base's yaw frame."""
        held = self.grasped
        force = self.law.step(self.plan.ref, self.s, held)
        if held:
            self.force_w = float(force) * self.path.tangent(float(self.s))
        else:
            self.force_w = np.zeros(3)
        self.force_b = insert_policy.quat_rotate_inverse(insert_policy.yaw_quat(self.base_quat), self.force_w)
        return self.force_b

    def commands(self, num_commands: int, fresh: bool) -> np.ndarray:
        """The 15 commands of the next frame: all zero for a reset's frame; then no walking, the goal, the roll channels,
        the law's force."""
        out = np.zeros(int(num_commands))
        if fresh:
            return out
        c = self.m["commands"]
        out[c["goal"][0]:c["goal"][1]] = self.goal
        out[c["roll"][0]:c["roll"][1]] = self.roll_channels
        out[c["force"][0]:c["force"][1]] = self.force_b
        return out


# --- the runner --------------------------------------------------------------------------------------------------------

class MechRunner:
    """One robot's held mechanism around an exported policy (`LegPolicy` with `meta["task"] == "mechanism"`).

    `reset` once per episode; then each 50 Hz step: `update` (not after a reset), `frame`, the policy's `act` on the
    returned stack, `targets` -- or `step`, which does the four. Keyword arguments override the export's evaluation
    plan, law and delay (`MechLayer`)."""

    def __init__(self, meta: dict, **layer_kw):
        if meta.get("task") != "mechanism":
            raise ValueError(f"not a mechanism export (task {meta.get('task')!r})")
        self.meta, self.mech = meta, meta["mechanism"]
        self.dt = float(meta["policy_dt"])
        self.layer = MechLayer(self.mech, dt=self.dt, **layer_kw)
        self.kin = insert_policy.ArmKinematics(self.mech["kinematics"])
        self.tool_offset = np.asarray(self.mech["tool"]["offset_m"], dtype=float)
        self.obs = leg_policy.LegObservation(meta)
        limits = np.asarray(meta["arm_joint_limits"], dtype=float)
        self.arm = reach_arm.ArmTargets(np.asarray(meta["default_dof_pos"], dtype=float)[12:18], meta["action_scale"],
                                        meta["clip_actions"], limits[:, 0], limits[:, 1])
        self.default = np.asarray(meta["default_dof_pos"], dtype=float)[:18]
        self.events: list[dict] = []
        self._fresh = True

    def reset(self, handle, base_pos, base_quat, traj_steps: float | None = None, handle_is: str = "sphere") -> None:
        """A new episode. `handle` is the handle's placement: goal-sphere coordinates (radius, pitch, yaw) about the
        centre of the base now (`handle_is="sphere"`, as the evaluation's plan gives it), or a world point
        (`handle_is="world"`, converted with the base's pose now and then held in the base's goal sphere as training
        holds it). `traj_steps` is the approach's slide in policy steps, by default `approach.default_slide_s`."""
        if traj_steps is None:
            traj_steps = float(self.mech["approach"]["default_slide_s"]) / self.dt
        sphere = (np.asarray(handle, dtype=float) if handle_is == "sphere"
                  else world_to_sphere(handle, base_pos, base_quat, self.layer.centre_offset))
        self.layer.reset(sphere, traj_steps, base_pos, base_quat)
        self.handle_sphere = sphere
        self.obs.reset()
        self._fresh = True
        self.commands = np.zeros(int(self.meta["num_commands"]))
        self.raw_targets = self.default.copy()
        self.events = [{"step": 0, "event": "reset", "handle_sphere": [round(float(v), 5) for v in sphere],
                        "engage_step": self.layer.engage_step}]

    def update(self, base_pos, base_quat, tool_speed: float, s=None, path: MechPath | None = None,
               grasp_now: bool | None = None, tool_pos=None, tool_to_goal_m: float | None = None) -> None:
        """`MechLayer.update`, logging the grasp (with the tool's distance from its goal when the layer had one)."""
        before = self.layer.grasped
        self.layer.update(base_pos, base_quat, tool_speed, s, path, grasp_now, tool_pos=tool_pos,
                          tool_to_goal_m=tool_to_goal_m)
        if self.layer.grasped and not before:
            event = {"step": self.layer.since_reset, "event": "grasp", "rule": bool(self.layer.rule_due),
                     "tool_speed_m_s": round(float(tool_speed), 4),
                     "travel_m": round(float(self.layer.path.travel), 4), "hinge": self.layer.path.hinge}
            if self.layer.tool_to_goal_m is not None:
                event["tool_to_goal_m"] = round(self.layer.tool_to_goal_m, 5)
            self.events.append(event)

    def frame(self, imu_quat, gyro, q_legs, dq_legs, arm_q, arm_dq) -> np.ndarray:
        """Push this step's frame and return the (2432,) stack. Legs in Unitree's order; the arm as the D1 reports it
        (URDF angles, rad) with its report-difference rates."""
        self.commands = self.layer.commands(int(self.meta["num_commands"]), self._fresh)
        self.last_frame = self.obs.frame(imu_quat, gyro, q_legs, dq_legs, arm_q, arm_dq, self.commands)
        self._fresh = False
        return self.obs.step(self.last_frame)

    def targets(self, action) -> tuple[np.ndarray, np.ndarray]:
        """The 12 leg targets (Unitree's order) and the arm's six (URDF order, rad, clamped to the training limits) for
        the policy's 18 outputs; then the law, for the next frame's force. `raw_targets` keeps the 18 unclamped, UniFP's
        order, as the environment computes them."""
        a = np.clip(np.asarray(action, dtype=float).reshape(-1), -float(self.meta["clip_actions"]),
                    float(self.meta["clip_actions"]))
        if a.size != 18:
            raise ValueError(f"a mechanism policy has 18 outputs, not {a.size}")
        self.obs.record_action(a)
        self.raw_targets = self.default + float(self.meta["action_scale"]) * a
        legs = self.obs.leg_targets_unitree(a[:12])
        arm = self.arm.target(a[12:18])
        self.layer.law_step()
        return legs, arm

    def step(self, policy, imu_quat, gyro, q_legs, dq_legs, arm_q, arm_dq, base_pos, base_quat, tool_speed: float,
             s=None, path: MechPath | None = None, grasp_now: bool | None = None, tool_pos=None) -> dict:
        """One 50 Hz step: the layer (unless this is the reset's frame), the frame, the policy, the targets. With the
        arrival gate on and `tool_pos` None, the tool is the forward kinematics of `arm_q` placed by the base's pose
        (`hand`), as the dog reads it."""
        if not self._fresh:
            if tool_pos is None and self.layer.grasp_within is not None:
                tool_pos = self.hand(arm_q, base_pos, base_quat)
            self.update(base_pos, base_quat, tool_speed, s, path, grasp_now, tool_pos=tool_pos)
        stack = self.frame(imu_quat, gyro, q_legs, dq_legs, arm_q, arm_dq)
        action = policy.act(stack)[0]
        legs, arm = self.targets(action)
        ly = self.layer
        return {"action": action, "legs": legs, "arm": arm, "grasped": ly.grasped, "ref": float(ly.plan.ref),
                "s": float(ly.s), "force_n": float(ly.law.force), "goal": ly.goal.copy()}

    def hand(self, arm_q, base_pos, base_quat) -> np.ndarray:
        """The controlled point in the world by the reported joints (`hand_point`)."""
        return hand_point(self.kin, arm_q, base_pos, base_quat, self.tool_offset)

    def measure(self, path: MechPath, arm_q, base_pos, base_quat) -> tuple[float, float]:
        """The dog's reading of the travel (`measure_travel`). Not validated on the dog."""
        return measure_travel(path, self.kin, arm_q, base_pos, base_quat, self.tool_offset)
