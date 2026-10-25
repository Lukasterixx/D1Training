"""The approach: a box the look sees but that is out of a task's range is walked to, not refused (Lukas, 2026-10-24: "make
it so that the box is never out of range as long as the dog sees the qr code"). numpy only.

From the look's fix, the task's own stance (the door pull's, the key's push, the insertion's) is moved straight back
along its facing until the box's root is `STAGE_ROOT_M` ahead: the **staging pose**, square to the box, in front of the
part the task works on, where the front camera saw the tag on the dog (`STAGE_ROOT_M`'s note; a look there that finds no
tag walks on back to `STAGE_FAR_ROOT_M` once, `node.py`). The dog walks there on the odometry, forward or back,
sideways and turning at once (`ApproachAttempt`: the key's walk up, `key_plan.WalkUp`, with a longer reach), the arm held
where it is. Then the node looks again from there (`node.py`), and the task's own walk in, its refit and its range check
go on as from any look. Up to `MAX_APPROACHES` times; a look that still finds the box out of range after them stops.

The same walk takes the dog back after the key (`door_after_key`, `node.py --then_door`): once the lock is turned the
door swings open by up to ~10 degrees, so the door pull does not go on from the key's fix. The dog backs up to the door
pull's staging pose from where the key left it, looks again, and the door pull goes on from that look as from any.

Not walked to: a tag the lidar says is not on a door the box's size (P2Dingo's charging dock carries the same tag on a
stand in front of a wall); a box more than `MAX_APPROACH_M` off. Nothing checks the way there for obstacles: the
approach walks in a straight line, which is the dog's operator's to keep clear.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from demos.combiner_door.robot import plan
from demos.combiner_key.robot import key_plan
from demos.unifp import script as demo_script

#: The staging pose: the box's root this far straight ahead, for every look the program walks to (the approach, the look
#: after the key, at the open door for the close, and after the close for the lock). Lukas, 2026-10-25: "the offset the
#: dog uses to look at the door once it opens is a good distance that I want re-used for the initial look at the closed
#: door, and then also the other look it does once the robot closes it". The close's look from 0.30 m nearer than the
#: 1.05 before saw the tag at once in both runs (02:37 and 05:04 UTC; the dog stopped ~13 cm short of it on the odometry,
#: the look's walk in 0.11-0.14 m). The door pull's stance has the root 0.75 m ahead, so a door pull from there needs
#: ~no walk in; the insertion's (0.60) a 0.15 m walk up. By the calibrated lens model the tag's top is 77 px above the
#: image at 0.75 (22 px of room at 0.85, where the dog has looked from); not validated at 0.75 itself.
STAGE_ROOT_M = 0.75
#: A look from `STAGE_ROOT_M` that finds no whole tag walks on back to here, once, and looks again (`node.py`): the
#: staging pose before 2026-10-25, the front camera's band by the lens model (`plan.FRONT_LOOK_MIN_ROOT_M`).
STAGE_FAR_ROOT_M = 1.05
#: The staging pose has the box's root at most this far to its side: the tag, on the door's centreline, in the front
#: camera's view. The door pull's stance has it 0.10 m to the side, from where every look saw the tag; the key's push and
#: the insertion's stance stand in front of the lock, 0.35 m to the side, and from there, the root 0.75 m ahead, the tag
#: is at the image's edge or past it (the lens model: columns 1107-1249 of 1280 at 0.88 m), and the look after the close
#: saw none (2026-10-25 05:49 UTC). Their walk up takes the rest sideways (0.25 m, `key_plan.WALK_UP_SIDE_M` 0.40).
LOOK_SIDE_M = 0.10
#: Close enough to the staging pose (a look follows, and the task's own walk takes out the rest).
ACCEPT_M = 0.08
ACCEPT_YAW_DEG = 6.0
#: How far the approach may take the dog, and how long it may walk. The gait's 0.25 m/s cap and turns: ~15 s for 3 m.
MAX_APPROACH_M = 3.0
TIMEOUT_S = 40.0
MAX_APPROACHES = 3
#: After the key (`door_after_key`): the dog stands this long before its first step back. Sport mode is put back into
#: classic walk as the walk begins (`node.py`): the insertion's StandDown, StandUp and BalanceStand leave it in the
#: default gait, and the door pull's walk in after the insertion went in that gait (Lukas, 2026-10-25).
GAIT_SETTLE_S = 1.5


def task_stance(task: str, box) -> tuple[float, float, float]:
    """The stance (x, y, yaw) a task works from, in the frame `box` (the box's 4x4 pose) is given in: the door pull's
    (square to the door, the root `plan.STANCE_AHEAD_M` ahead and the dog `plan.STANCE_RIGHT_M` toward the hinge), the
    key's push (`key_plan.push_stance`),
    the insertion's (`insert_plan.insert_stance`)."""
    if task == "key":
        return key_plan.push_stance(box)
    if task == "insert":
        from demos.combiner_key.robot import insert_plan

        return insert_plan.insert_stance(box)
    from demos.combiner.geometry import GEOMETRY
    from demos.combiner_key import geometry as kg

    box = np.asarray(box)
    normal = box[:3, :3] @ np.asarray(kg.OUT_OF_DOOR_BOX)
    normal = normal[:2] / np.linalg.norm(normal[:2])
    hinge = box[:3, :3] @ np.array([0.0, GEOMETRY.hinge_side, 0.0])     # toward the hinge, along the door
    hinge = hinge[:2] / np.linalg.norm(hinge[:2])
    x, y = box[:2, 3] + plan.STANCE_AHEAD_M * normal + plan.STANCE_RIGHT_M * hinge
    return float(x), float(y), math.atan2(-normal[1], -normal[0])


def staging_pose(task: str, box, root_m: float = STAGE_ROOT_M) -> tuple[float, float, float]:
    """The staging pose (x, y, yaw), the frame `box` is given in: facing as the task's stance does, the box's root
    `root_m` (`STAGE_ROOT_M`) ahead, or the stance's own distance if that is further, and no more than `LOOK_SIDE_M` to
    the side, on the stance's side (the door pull's: its stance moved straight back)."""
    x, y, yaw = task_stance(task, box)
    facing = np.array([math.cos(yaw), math.sin(yaw)])
    left = np.array([-facing[1], facing[0]])
    root = np.asarray(box)[:2, 3]
    rel = root - np.array([x, y])
    ahead = max(root_m, float(rel @ facing))
    side = max(-LOOK_SIDE_M, min(LOOK_SIDE_M, float(rel @ left)))
    px, py = root - ahead * facing - side * left
    return float(px), float(py), yaw


def staging_fix(fix: plan.Fix, staging_odom) -> plan.Fix:
    """A plain fix with its script frame at `staging_odom` (x, y, yaw in the odometry; `odometry_target` of the staging
    pose): the approach walks to its origin."""
    plain = plan.Fix(fix.script_odom, fix.site, fix.pose_script, fix.fit, fix.detections, fix.lidar_points,
                     fix.tag_only_yaw_deg, 0.0, fix.tag_tilt_deg, fix.beyond_edges)
    x, y, yaw = staging_odom
    return plan.reanchor(plain, plan.base_pose(x, y, yaw))


def odometry_target(start, staging) -> tuple[float, float, float]:
    """Where to send the dog on the odometry to arrive at `staging` (x, y, yaw), from `start`: the way there shortened
    by the odometry's under-count (`plan.ODOM_WALK_SCALE`)."""
    sx, sy, _ = start
    x, y, yaw = staging
    k = plan.ODOM_WALK_SCALE
    return sx + k * (x - sx), sy + k * (y - sy), yaw


def door_after_key(fix: plan.Fix, start) -> "ApproachAttempt":
    """After the key's turn (`node.py --then_door`): back to the door pull's staging pose from the key's fix, from
    `start` (x, y, yaw in the odometry), standing `GAIT_SETTLE_S` first. The key's fix is only good for where to stand
    and look from: the unlocked door has swung open since, and the look there gives the door pull its own fix."""
    staging = staging_pose("door", fix.box_odom)
    return ApproachAttempt(staging_fix(fix, odometry_target(start, staging)), "walk", settle_s=GAIT_SETTLE_S)


@dataclass
class ApproachAttempt(key_plan.WalkUp, plan.Attempt):
    """The walk to the staging pose (the script frame's origin, facing +x): the key's walk up on all three axes
    (`key_plan.WalkUp`), further and with a looser finish (`ACCEPT_*`); the arm left where it is (nothing is sent to it).
    One phase, `walk_in`; after the key a `pause` before it (`settle_s`), standing."""
    settle_s: float = 0.0

    reads_door = False
    joint_phases = frozenset()
    dwell_phases = frozenset()
    position_only_phases = {"walk_in"}
    holding_phases = frozenset()
    at_the_lever_phases = frozenset()
    walking_phases = {"walk_in"}
    release_held_phases = frozenset()
    walk_up_timeout_s = TIMEOUT_S
    walk_up_max_steps = 3
    walk_in_timeout_s = TIMEOUT_S + 5.0

    def _make_phases(self) -> list:
        home = lambda st, u: plan.home_at(st["base"])  # noqa: E731
        phases = [demo_script.Phase("walk_in", plan.WALK_IN_S, path=home, jaw_m=demo_script.JAW_LEVER_OPEN_M,
                                    base=lambda st, u: (0.0, 0.0, 0.0))]
        if self.settle_s > 0.0:
            phases.insert(0, demo_script.Phase("pause", self.settle_s, path=home, jaw_m=demo_script.JAW_LEVER_OPEN_M))
        return phases

    def _prepare(self) -> None:
        pass

    def _close_enough(self, base) -> bool:
        x, y, yaw = base
        return math.hypot(x, y) <= ACCEPT_M and abs(math.degrees(plan.wrap(yaw))) <= ACCEPT_YAW_DEG

    def _walk_up_note(self, actual) -> dict:
        return {}

    def _goal_servo(self, cmd, base_s, measured, st):
        self.status = "hold"
        return np.asarray(measured, dtype=float)

    def step(self, obs: plan.Observation, dt: float = 0.1) -> plan.Command:
        out = super().step(obs, dt)
        out.arm_servo_deg, out.gripper_units = None, None          # the arm left where it is
        return out
