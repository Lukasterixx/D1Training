"""Hand poses to command that the arm can reach: UniFP's goal generator, drawn in joint space.

UniFP commands a position. `EeGoalTrajectory` draws it in the goal sphere and slides it there. A *pose* cannot be drawn the
same way: over that sphere 11% of uniformly random orientations are reachable (F-099, `pose_reach.py`), so a reward on them
would mostly score what no action attains. This draws the other way round. It picks **joint angles** and takes the pose
they give through the D1's forward kinematics, so every goal, and every point of the slide to it, is a pose the arm takes
by construction, with the base where it stands. The pose is all the policy is told; the angles stay here.

Three choices make the answer unique, which matters because the policy has to learn this inverse:

  * The elbow has one branch: Joint3's limits stop it short of straight, so there is no elbow-down solution in reach.
  * The wrist has two, `(J4, J5, J6)` and `(J4 + pi, -J5, J6 + pi)`. `JOINT_BOX` holds Joint4 to about +/-90 degrees, which
    keeps to one and still points the hand over the whole half-sphere about the forearm.
  * Goals are slid along the straight line **in joint space**, at a joint speed the real arm can make (`SPEED_RAD_S`
    against the Follower's 45 deg/s), so the commanded pose never leaves the reachable set and never asks for a wrist flip.

**The command.** Position stays the sphere's (radius, pitch, yaw), so nothing downstream of it changes. Orientation goes in
`CMD_EE_ORN_R/P/Y`, three channels declared by the contract and zero in every policy before this one, as the rotation
vector taking the hand's all-zero attitude to the commanded one, in the yaw-only frame the position is in. It is
continuous wherever the turn from neutral is under a half turn, and `MAX_TURN_RAD` rejects any goal past that.

**Primitives** (`mix`). A straight line in joint space moves position and orientation together, always. A task does not: an
approach slides the hand along itself with its attitude fixed, a lever swings the hand round an axis outside it. A policy
trained on joint lines alone swings 30 degrees through a 12 cm straight approach it has never seen (2026-11-07, door pull). So a
share of goals are Cartesian segments from wherever the last ended, each checked reachable by solving the arm at points along it:

    translate   p + s dp, attitude fixed; half of them along the hand's own approach
    rotate      turned about an axis through the jaw centre, position fixed
    arc         turned about an axis `ARC_RADIUS_M` off the hand, position and attitude together: a lever, a small door

No Isaac imports (`tests/test_arm_kin.py`).
"""
from __future__ import annotations

import math

import torch

from unifp_isaaclab import interface

from . import arm_kin

#: Joint draw, rad, URDF order: inside the soft limits, Joint1 to the sphere's yaw, Joint4 to one wrist branch.
JOINT_BOX = ((-1.40, 1.40), (-1.35, 1.35), (-1.35, 1.35), (-1.60, 1.60), (-1.40, 1.40), (-2.10, 2.10))
#: Largest single move of any joint, rad: goals further than this are drawn nearer along the same line.
MAX_MOVE_RAD = 1.5
#: Slide speed of the fastest joint, drawn per goal, rad/s. The arm link's cap is 0.785 (45 deg/s).
SPEED_RAD_S = (0.25, 0.60)
#: Seconds a slide takes at least, and a goal is held after it.
MIN_SLIDE_S = 1.0
HOLD_S = (0.5, 2.0)
#: Base height the goals are drawn for, m (`task_cfg.BASE_HEIGHT_TARGET`).
BASE_HEIGHT_M = 0.30
#: Where the position may be, as the sphere's: the upper radius is the arm's with a level hand (`pose_reach.py`).
RADIUS_RANGE = (0.30, 0.62)
PITCH_RANGE = interface.EE_GOAL_PITCH_RANGE
YAW_RANGE = interface.EE_GOAL_YAW_RANGE
#: A goal's turn from the neutral hand, rad. The rotation vector is discontinuous at pi.
MAX_TURN_RAD = 0.92 * math.pi
#: Candidates drawn per goal, of which the first that passes is kept.
CANDIDATES = 12
#: Fractions along a slide checked against the body.
PATH_CHECKS = (0.25, 0.5, 0.75, 1.0)
#: `mix`: the share of goals of each kind. None: joint lines alone, as the frozen `unifp_pose_*` sets were drawn.
TASK_MIX = {"joint": 0.40, "translate": 0.30, "rotate": 0.12, "arc": 0.18}
#: A translation's length, m; a turn's angle, rad; an arc's pivot from the jaw centre, m.
TRANSLATE_M = (0.04, 0.22)
ROTATE_RAD = (0.2, 1.2)
ARC_RAD = (0.3, 1.2)
ARC_RADIUS_M = (0.06, 0.40)
#: Speeds a Cartesian segment is drawn at. The script's approach is 5 cm/s; its fastest turn 0.6 rad/s.
LINEAR_SPEED_M_S = (0.04, 0.16)
ANGULAR_SPEED_RAD_S = (0.2, 0.6)
#: Draws of a Cartesian segment before a robot takes a joint line instead: most random directions from a pose run out of reach.
CARTESIAN_TRIES = 6
#: With primitives, goals are drawn every this many policy steps, for everyone then due: one batch of solves in place of five, at
#: the cost of up to 80 ms longer on a hold.
RESAMPLE_EVERY = 5
#: Fractions of a Cartesian segment solved for the arm.
SEGMENT_CHECKS = (1.0 / 3.0, 2.0 / 3.0, 1.0)


def rotvec_from_matrix(rotation: torch.Tensor) -> torch.Tensor:
    """The rotation vector of (N, 3, 3) matrices, angles under pi."""
    trace = rotation[:, 0, 0] + rotation[:, 1, 1] + rotation[:, 2, 2]
    angle = torch.acos(((trace - 1.0) * 0.5).clamp(-1.0, 1.0))
    axis = torch.stack((rotation[:, 2, 1] - rotation[:, 1, 2], rotation[:, 0, 2] - rotation[:, 2, 0],
                        rotation[:, 1, 0] - rotation[:, 0, 1]), dim=-1)
    # angle / (2 sin angle), with its limit of a half at zero.
    scale = torch.where(angle < 1e-4, torch.full_like(angle, 0.5), angle / (2.0 * torch.sin(angle).clamp(min=1e-6)))
    return axis * scale.unsqueeze(-1)


def matrix_from_rotvec(rotvec: torch.Tensor) -> torch.Tensor:
    """(N, 3, 3) matrices of rotation vectors (N, 3): Rodrigues."""
    angle = rotvec.norm(dim=-1, keepdim=True).clamp(min=1e-9)
    x, y, z = (rotvec / angle).unbind(-1)
    zero = torch.zeros_like(x)
    k = torch.stack((zero, -z, y, z, zero, -x, -y, x, zero), dim=-1).view(-1, 3, 3)
    s, c = torch.sin(angle)[..., None], torch.cos(angle)[..., None]
    eye = torch.eye(3, dtype=rotvec.dtype, device=rotvec.device)
    return eye + s * k + (1.0 - c) * (k @ k)


def matrix_from_quat(quat_wxyz: torch.Tensor) -> torch.Tensor:
    w, x, y, z = quat_wxyz.unbind(-1)
    return torch.stack((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
                        2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
                        2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)), dim=-1).view(-1, 3, 3)


class PoseGoalTrajectory:
    """Drop-in for `EeGoalTrajectory`: the same `step`, `reset`, `current`, `start` and `goal`, plus an orientation.

    `current` is (radius, pitch, yaw) of the jaw centre. `current_rotvec` is what goes in the three orientation channels and
    `current_rotation` the same as a matrix, in the yaw-only base frame. `naive` draws the position as UniFP does and the
    orientation uniformly over SO(3): F-099's objective, kept as the control that says what the joint-space draw buys.
    """

    def __init__(self, num_envs: int, device, *, generator: torch.Generator | None = None, naive: bool = False,
                 box=JOINT_BOX, speed=SPEED_RAD_S, hold_s=HOLD_S, max_move=MAX_MOVE_RAD, mix: dict | None = None):
        self.num_envs, self.device, self.generator, self.naive = num_envs, torch.device(device), generator, naive
        self.mix = mix
        self.kin = arm_kin.ArmKinematics(device=device)
        zero = torch.zeros(1, 6, device=self.device)
        self.neutral = self.kin.tool(zero)[0][0].clone()
        self.low = torch.tensor([b[0] for b in box], device=self.device)
        self.high = torch.tensor([b[1] for b in box], device=self.device)
        self.speed, self.hold, self.max_move = speed, hold_s, max_move
        z = lambda *shape: torch.zeros(*shape, device=self.device)
        self.q_start, self.q_goal, self.q_current = z(num_envs, 6), z(num_envs, 6), z(num_envs, 6)
        self.start, self.goal, self.current = z(num_envs, 3), z(num_envs, 3), z(num_envs, 3)
        self.current_rotvec = z(num_envs, 3)
        self.current_rotation = self.neutral.expand(num_envs, 3, 3).clone()
        self.rot_start = self.current_rotation.clone()
        self.rot_goal = self.current_rotation.clone()
        self.timer, self.traj_steps, self.total_steps = z(num_envs), torch.ones(num_envs, device=self.device), z(num_envs)
        #: Goals kept and given up on, for the log.
        self.draws = self.misses = 0
        #: The Cartesian segment each environment is on: `cartesian` False means a joint line. Pose along it, for s in 0-1:
        #: `pivot + Rot(axis, s theta) (p0 - pivot) + s dp`, attitude `Rot(axis, s theta) R0`, in the commanded (base) frame.
        self.cartesian = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.seg_p0, self.seg_pivot, self.seg_dp, self.seg_axis = z(num_envs, 3), z(num_envs, 3), z(num_envs, 3), z(num_envs, 3)
        self.seg_axis[:, 2] = 1.0
        self.seg_r0 = self.current_rotation.clone()
        self.seg_theta = z(num_envs)
        self.steps = 0
        #: Goals of each kind kept, for the log.
        self.kinds = {"joint": 0, "translate": 0, "rotate": 0, "arc": 0}
        #: Goals drawn this episode: 0 through the opening hold on the all-zero pose, which any policy passes by standing still.
        self.goals_drawn = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        self.reset()

    def _uniform(self, span, shape) -> torch.Tensor:
        return torch.rand(*shape, device=self.device, generator=self.generator) * (span[1] - span[0]) + span[0]

    def pose_of(self, q: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Joint angles (N, 6) -> ((radius, pitch, yaw), rotation) as commanded, for a level base at `BASE_HEIGHT_M`."""
        rotation, position = self.kin.tool(q)
        centre = torch.tensor((0.0, 0.0, interface.EE_GOAL_CENTER_OFFSET[2] - BASE_HEIGHT_M), device=q.device)
        return interface.cart2sphere(position - centre), rotation

    def reset(self, env_ids: torch.Tensor | None = None) -> None:
        """Every selected robot on the arm's all-zero pose, as an episode starts, held briefly before its first goal."""
        index = slice(None) if env_ids is None else env_ids
        count = self.num_envs if env_ids is None else len(env_ids)
        zero = torch.zeros(count, 6, device=self.device)
        sphere, rotation = self.pose_of(zero)
        self.q_start[index], self.q_goal[index], self.q_current[index] = zero, zero, zero
        self.start[index], self.goal[index], self.current[index] = sphere, sphere, sphere
        self.rot_start[index], self.rot_goal[index], self.current_rotation[index] = rotation, rotation, rotation
        self.current_rotvec[index] = rotvec_from_matrix(self.neutral.T @ rotation)
        self.timer[index] = 0.0
        self.goals_drawn[index] = 0
        self.cartesian[index] = False
        self.traj_steps[index] = 1.0
        self.total_steps[index] = self._uniform(self.hold, (count,)) / interface.POLICY_DT

    def _body_clear(self, q: torch.Tensor) -> torch.Tensor:
        """True where the hand, wrist and jaw centre stay out of `interface`'s keep-out box round the robot."""
        rotation, position = self.kin.link6(q)
        points = torch.stack((position, self.kin.wrist(q), position + rotation @ self.kin.jaw_local), dim=1)
        centre = torch.tensor((0.0, 0.0, interface.EE_GOAL_CENTER_OFFSET[2] - BASE_HEIGHT_M), device=q.device)
        points = points - centre
        upper = torch.as_tensor(interface.EE_GOAL_COLLISION_UPPER, device=q.device)
        lower = torch.as_tensor(interface.EE_GOAL_COLLISION_LOWER, device=q.device)
        inside = ((points < upper) & (points > lower)).all(dim=-1).any(dim=-1)
        return ~inside & (points[..., 2] > interface.EE_GOAL_UNDERGROUND_LIMIT).all(dim=-1)

    def _acceptable(self, q0: torch.Tensor, q: torch.Tensor) -> torch.Tensor:
        sphere, rotation = self.pose_of(q)
        ok = (sphere[:, 0] > RADIUS_RANGE[0]) & (sphere[:, 0] < RADIUS_RANGE[1])
        ok &= (sphere[:, 1] > PITCH_RANGE[0]) & (sphere[:, 1] < PITCH_RANGE[1])
        ok &= (sphere[:, 2] > YAW_RANGE[0]) & (sphere[:, 2] < YAW_RANGE[1])
        ok &= rotvec_from_matrix(self.neutral.T @ rotation).norm(dim=-1) < MAX_TURN_RAD
        for fraction in PATH_CHECKS:
            ok &= self._body_clear(torch.lerp(q0, q, fraction))
        return ok

    def _resample_joints(self, ids: torch.Tensor) -> None:
        count = len(ids)
        q0 = self.q_goal[ids]
        draw = self.low + (self.high - self.low) * torch.rand(count, CANDIDATES, 6, device=self.device, generator=self.generator)
        step = draw - q0.unsqueeze(1)
        largest = step.abs().amax(dim=-1, keepdim=True).clamp(min=1e-6)
        draw = q0.unsqueeze(1) + step * (self.max_move / largest).clamp(max=1.0)
        flat = self._acceptable(q0.repeat_interleave(CANDIDATES, dim=0), draw.reshape(-1, 6)).view(count, CANDIDATES)
        found = flat.any(dim=1)
        first = flat.float().argmax(dim=1)
        chosen = torch.where(found.unsqueeze(-1), draw[torch.arange(count, device=self.device), first], q0)
        self.draws += count
        self.misses += int((~found).sum())
        self.q_start[ids] = q0
        self.q_goal[ids] = chosen
        self.start[ids] = self.goal[ids]
        self.goal[ids] = self.pose_of(chosen)[0]
        speed = self._uniform(self.speed, (count,))
        slide = ((chosen - q0).abs().amax(dim=-1) / speed).clamp(min=MIN_SLIDE_S)
        self.traj_steps[ids] = slide / interface.POLICY_DT
        self.total_steps[ids] = self.traj_steps[ids] + self._uniform(self.hold, (count,)) / interface.POLICY_DT
        self.timer[ids] = 0.0

    def _segment_pose(self, s: torch.Tensor, p0, pivot, dp, axis, theta, r0) -> tuple[torch.Tensor, torch.Tensor]:
        """Position and attitude at fraction `s` (N,) along a segment, base frame."""
        turn = matrix_from_rotvec(axis * (s * theta).unsqueeze(-1))
        arm = (turn @ (p0 - pivot).unsqueeze(-1)).squeeze(-1)
        return pivot + arm + s.unsqueeze(-1) * dp, turn @ r0

    def _unit(self, count: int) -> torch.Tensor:
        v = torch.randn(count, 3, device=self.device, generator=self.generator)
        return v / v.norm(dim=-1, keepdim=True).clamp(min=1e-6)

    def _resample_cartesian(self, ids: torch.Tensor, kinds: torch.Tensor) -> torch.Tensor:
        """Try `CARTESIAN_TRIES` segments from each robot's last goal at once, keep its first that solves. `kinds` 1 translate, 2 rotate,
        3 arc. Returns who got one.

        All in one batch: a sequential retry launched thousands of small kernels a policy step and ran training at 32 s an iteration.
        """
        count = len(ids)
        tries = CARTESIAN_TRIES
        rows = count * tries
        q0 = self.q_goal[ids].repeat_interleave(tries, dim=0)
        kind = kinds.repeat_interleave(tries)
        r0, p0 = self.kin.tool(q0)
        translate, rotate, arc = kind == 1, kind == 2, kind == 3
        # Half the translations run along the hand's own approach, either way: that is what entering a handle is.
        direction = self._unit(rows)
        along = torch.rand(rows, device=self.device, generator=self.generator) < 0.5
        sign = torch.where(torch.rand(rows, device=self.device, generator=self.generator) < 0.5, 1.0, -1.0).unsqueeze(-1)
        direction = torch.where(along.unsqueeze(-1), r0[:, :, 2] * sign, direction)
        length = self._uniform(TRANSLATE_M, (rows,))
        dp = torch.where(translate.unsqueeze(-1), direction * length.unsqueeze(-1), torch.zeros_like(direction))
        # An arc turns about an axis off the hand. Levers and hinges are perpendicular to what they swing, so the axis is
        # drawn across the pivot's offset.
        axis = self._unit(rows)
        offset = self._unit(rows)
        offset = offset - (offset * axis).sum(-1, keepdim=True) * axis
        offset = offset / offset.norm(dim=-1, keepdim=True).clamp(min=1e-6)
        radius = self._uniform(ARC_RADIUS_M, (rows,))
        pivot = torch.where(arc.unsqueeze(-1), p0 + offset * radius.unsqueeze(-1), p0)
        theta = torch.where(rotate, self._uniform(ROTATE_RAD, (rows,)), self._uniform(ARC_RAD, (rows,)))
        theta = torch.where(translate, torch.zeros_like(theta), theta * torch.where(
            torch.rand(rows, device=self.device, generator=self.generator) < 0.5, 1.0, -1.0))
        ok = torch.ones(rows, dtype=torch.bool, device=self.device)
        q = q0
        for fraction in SEGMENT_CHECKS:
            p, r = self._segment_pose(torch.full((rows,), fraction, device=self.device), p0, pivot, dp, axis, theta, r0)
            q, good = self.kin.track(p, r, q, self.low, self.high, iterations=3)
            sphere, _ = self.pose_of(q)
            ok &= good & self._body_clear(q)
            ok &= (sphere[:, 0] > RADIUS_RANGE[0]) & (sphere[:, 0] < RADIUS_RANGE[1])
            ok &= (sphere[:, 1] > PITCH_RANGE[0]) & (sphere[:, 1] < PITCH_RANGE[1])
            ok &= (sphere[:, 2] > YAW_RANGE[0]) & (sphere[:, 2] < YAW_RANGE[1])
            ok &= rotvec_from_matrix(self.neutral.T @ r).norm(dim=-1) < MAX_TURN_RAD
        ok &= (q - q0).abs().amax(dim=-1) < self.max_move
        grid = ok.view(count, tries)
        found = grid.any(dim=1)
        rows_kept = (torch.arange(count, device=self.device) * tries + grid.float().argmax(dim=1))[found]
        got = ids[found]
        if len(got) == 0:
            return found
        linear = self._uniform(LINEAR_SPEED_M_S, (len(got),))
        angular = self._uniform(ANGULAR_SPEED_RAD_S, (len(got),))
        q_new, q_old = q[rows_kept], q0[rows_kept]
        slide = torch.maximum(dp[rows_kept].norm(dim=-1) / linear, theta[rows_kept].abs() / angular)
        slide = torch.maximum(slide, (q_new - q_old).abs().amax(dim=-1) / self.speed[1]).clamp(min=MIN_SLIDE_S)
        self.q_start[got] = q_old
        self.q_goal[got] = q_new
        self.cartesian[got] = True
        self.seg_p0[got], self.seg_pivot[got], self.seg_dp[got] = p0[rows_kept], pivot[rows_kept], dp[rows_kept]
        self.seg_axis[got], self.seg_theta[got], self.seg_r0[got] = axis[rows_kept], theta[rows_kept], r0[rows_kept]
        self.start[got] = self.goal[got]
        self.goal[got] = self.pose_of(q_new)[0]
        self.traj_steps[got] = slide / interface.POLICY_DT
        self.total_steps[got] = self.traj_steps[got] + self._uniform(self.hold, (len(got),)) / interface.POLICY_DT
        self.timer[got] = 0.0
        for index, name in ((1, "translate"), (2, "rotate"), (3, "arc")):
            self.kinds[name] += int((kinds[found] == index).sum())
        return found

    def _resample_mixed(self, ids: torch.Tensor) -> None:
        names = ("joint", "translate", "rotate", "arc")
        weights = torch.tensor([self.mix[n] for n in names], device=self.device)
        kinds = torch.multinomial(weights, len(ids), replacement=True, generator=self.generator)
        self.cartesian[ids] = False
        done = torch.zeros(len(ids), dtype=torch.bool, device=self.device)
        wanted = kinds > 0
        if bool(wanted.any()):
            done[wanted] = self._resample_cartesian(ids[wanted], kinds[wanted])
        # Whatever found no reachable segment takes a joint line, which always has one.
        if bool((~done).any()):
            self._resample_joints(ids[~done])
            self.kinds["joint"] += int((~done).sum())

    def _resample_naive(self, ids: torch.Tensor) -> None:
        count = len(ids)
        self.start[ids] = self.current[ids]
        self.rot_start[ids] = self.current_rotation[ids]
        self.goal[ids] = torch.stack((self._uniform(interface.EE_GOAL_RADIUS_RANGE, (count,)),
                                      self._uniform(interface.EE_GOAL_PITCH_RANGE, (count,)),
                                      self._uniform(interface.EE_GOAL_YAW_RANGE, (count,))), dim=-1)
        quat = torch.randn(count, 4, device=self.device, generator=self.generator)
        self.rot_goal[ids] = matrix_from_quat(quat / quat.norm(dim=-1, keepdim=True))
        self.traj_steps[ids] = self._uniform(interface.EE_GOAL_TRAJ_TIME_S, (count,)) / interface.POLICY_DT
        self.total_steps[ids] = self.traj_steps[ids] + self._uniform(self.hold, (count,)) / interface.POLICY_DT
        self.timer[ids] = 0.0

    def step(self) -> torch.Tensor:
        """Advance one policy step and return (radius, pitch, yaw); orientation is in `current_rotvec`."""
        t = torch.clip(self.timer / self.traj_steps, 0.0, 1.0).unsqueeze(-1)
        if self.naive:
            self.current = torch.lerp(self.start, self.goal, t)
            turn = rotvec_from_matrix(self.rot_start.transpose(1, 2) @ self.rot_goal)
            self.current_rotation = self.rot_start @ matrix_from_rotvec(turn * t)
        else:
            self.q_current = torch.lerp(self.q_start, self.q_goal, t)
            self.current, self.current_rotation = self.pose_of(self.q_current)
            if bool(self.cartesian.any()):
                ids = self.cartesian.nonzero(as_tuple=False).flatten()
                position, rotation = self._segment_pose(t[ids, 0], self.seg_p0[ids], self.seg_pivot[ids], self.seg_dp[ids],
                                                        self.seg_axis[ids], self.seg_theta[ids], self.seg_r0[ids])
                centre = torch.tensor((0.0, 0.0, interface.EE_GOAL_CENTER_OFFSET[2] - BASE_HEIGHT_M), device=self.device)
                self.current[ids] = interface.cart2sphere(position - centre)
                self.current_rotation[ids] = rotation
        self.current_rotvec = rotvec_from_matrix(self.neutral.T @ self.current_rotation)
        self.timer += 1
        self.steps += 1
        if self.mix and self.steps % RESAMPLE_EVERY:
            return self.current
        due = (self.timer > self.total_steps).nonzero(as_tuple=False).flatten()
        if len(due) > 0:
            (self._resample_naive if self.naive else self._resample_mixed if self.mix else self._resample_joints)(due)
            self.goals_drawn[due] += 1
        return self.current
