"""Cartesian (IK) control for the welded D1 arm.

Ported from Rescue's `d1_ik_controller.py`. The solver, the 10 Hz command rate,
the DLS method, the smoothing and the antipodal-quaternion guard are unchanged --
what changed is the plumbing underneath.

Rescue streams joint angles over the D1's real CycloneDDS protocol so the same
loop drives sim and hardware. This repo is a locomotion testbed, so that layer
is gone and `arm_client` is a `DirectD1` that writes straight to PhysX. The
`KinematicsBackend` seam is kept anyway: it is the only Isaac-specific piece,
and keeping it means the solver stays portable if this ever needs to talk to a
real arm again.

The other change is that `arm` and `robot` are now the same articulation. The
weld merged them, so the arm's joints are simply the tail of the Go2's joint
list and the Jacobian comes out of the same PhysX view.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import torch

from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.utils.math import quat_apply, quat_inv, quat_mul

_RAD2DEG = 180.0 / math.pi
_DEG2RAD = math.pi / 180.0

# The real D1 accepts joint angles at 10 Hz and nothing else. Held here so what
# you see in sim is the motion quality the hardware could actually deliver.
COMMAND_RATE_HZ = 10.0


@dataclass
class KinSnapshot:
    """A self-consistent kinematic state: EE pose, Jacobian and the joint angles
    they were evaluated at. Keeping them together is what lets the solver stay
    correct regardless of which backend produced them."""

    ee_pos: torch.Tensor      # (N, 3) world
    ee_quat: torch.Tensor     # (N, 4) world, wxyz
    jacobian: torch.Tensor    # (N, 6, num_joints)
    joint_pos: torch.Tensor   # (N, num_joints) radians


class KinematicsBackend:
    """Supplies forward kinematics and a Jacobian for the arm."""

    def snapshot(self, reported_angles_rad: torch.Tensor) -> KinSnapshot:
        raise NotImplementedError

    def to_world(self, rel_pos: torch.Tensor, rel_quat: torch.Tensor):
        """Map a target expressed in the arm's command frame into the frame the
        snapshot uses."""
        raise NotImplementedError

    def to_local(self, world_pos: torch.Tensor, world_quat: torch.Tensor):
        """Inverse of to_world. Used to re-derive a Cartesian target from where
        the arm actually is."""
        raise NotImplementedError


class IsaacKinematics(KinematicsBackend):
    """Reads kinematics straight from PhysX.

    PhysX evaluates the Jacobian at the articulation's *live* configuration, so
    `reported_angles_rad` is ignored and the live joint positions are returned
    alongside it. The snapshot stays internally consistent, which is what the
    solver needs; the cost is that it does not model the arm's 10 Hz encoder
    latency.

    Since the weld, `robot` is the merged Go2+D1 articulation: `joint_ids` index
    the arm's six joints within its 20, and `to_world` maps from the quadruped's
    base -- which is both the teleop frame and what the arm is bolted to.
    """

    def __init__(self, robot, arm_joint_names: list[str], ee_body_name: str = "Link6"):
        self._robot = robot
        self._ee_body_idx = robot.find_bodies(ee_body_name)[0][0]
        # Arm joints only: the legs are the policy's business and the gripper
        # fingers are not part of the IK chain.
        self._joint_ids = [robot.find_joints(name)[0][0] for name in arm_joint_names]

    @property
    def joint_ids(self) -> list[int]:
        return self._joint_ids

    def snapshot(self, reported_angles_rad: torch.Tensor) -> KinSnapshot:
        robot = self._robot
        jac = robot.root_physx_view.get_jacobians()
        # A fixed base drops the root body from the Jacobian's row indexing.
        idx = self._ee_body_idx - 1 if robot.is_fixed_base else self._ee_body_idx
        jacobian = jac[:, idx, :, :]
        if not robot.is_fixed_base:
            jacobian = jacobian[:, :, 6:]  # strip the floating-base DOF columns
        jacobian = jacobian[:, :, self._joint_ids]

        return KinSnapshot(
            ee_pos=robot.data.body_pos_w[:, self._ee_body_idx, :],
            ee_quat=robot.data.body_quat_w[:, self._ee_body_idx, :],
            jacobian=jacobian,
            joint_pos=robot.data.joint_pos[:, self._joint_ids],
        )

    def to_world(self, rel_pos: torch.Tensor, rel_quat: torch.Tensor):
        root_pos = self._robot.data.root_state_w[:, :3]
        root_quat = self._robot.data.root_state_w[:, 3:7]
        return (
            root_pos + quat_apply(root_quat, rel_pos),
            quat_mul(root_quat, rel_quat),
        )

    def to_local(self, world_pos: torch.Tensor, world_quat: torch.Tensor):
        root_pos = self._robot.data.root_state_w[:, :3]
        root_quat = self._robot.data.root_state_w[:, 3:7]
        root_inv = quat_inv(root_quat)
        return (
            quat_apply(root_inv, world_pos - root_pos),
            quat_mul(root_inv, world_quat),
        )


class D1CartesianController:
    """Holds a Cartesian target and streams joint angles to the arm.

    `update()` may be called every simulator step but only emits on the 10 Hz
    boundary, matching the arm's real command rate.
    """

    def __init__(
        self,
        arm_client,
        backend: KinematicsBackend,
        num_envs: int,
        device,
        rate_hz: float = COMMAND_RATE_HZ,
        smoothing: float = 0.05,
    ):
        self._client = arm_client
        self._backend = backend
        self._device = device
        self._num_envs = num_envs
        self._period = 1.0 / rate_hz
        self._alpha = smoothing
        self._time_since_send = 0.0
        self._suspended = False

        cfg = DifferentialIKControllerCfg(command_type="pose", ik_method="dls")
        self._ik = DifferentialIKController(cfg, num_envs=num_envs, device=device)

        # The smoothed target the solver actually chases, base-relative.
        self.current_rel_pos = torch.zeros((num_envs, 3), device=device, dtype=torch.float32)
        self.current_rel_rot = torch.tensor(
            [[1.0, 0.0, 0.0, 0.0]], device=device, dtype=torch.float32
        ).repeat(num_envs, 1)

    def reset(self, rel_pos, rel_rot) -> None:
        self.current_rel_pos[:] = torch.tensor(
            rel_pos, device=self._device, dtype=torch.float32
        ).repeat(self._num_envs, 1)
        self.current_rel_rot[:] = torch.tensor(
            rel_rot, device=self._device, dtype=torch.float32
        ).repeat(self._num_envs, 1)
        self._time_since_send = 0.0
        self._suspended = False
        try:
            self._ik.reset()
        except Exception:
            pass

    @property
    def is_suspended(self) -> bool:
        return self._suspended

    def suspend(self) -> None:
        """Stop streaming joint commands.

        Required before any non-streaming command (return-to-zero): this loop
        rewrites all six joints every 100 ms, so it would otherwise overwrite
        such a command before the arm visibly moved.
        """
        self._suspended = True

    def resume_from_arm(self):
        """Re-engage streaming from wherever the arm actually is.

        Returns the arm's current pose as a base-relative (pos, quat) so the
        caller can adopt it as the teleop target. Without this the arm would
        snap back to the pre-suspend target the moment streaming resumed.
        """
        snap = self._backend.snapshot(self._reported_rad())
        rel_pos, rel_rot = self._backend.to_local(snap.ee_pos, snap.ee_quat)
        self.current_rel_pos[:] = rel_pos
        self.current_rel_rot[:] = rel_rot
        self._suspended = False
        self._time_since_send = 0.0
        try:
            self._ik.reset()
        except Exception:
            pass
        pos = rel_pos[0].detach().cpu().numpy().tolist()
        rot = rel_rot[0].detach().cpu().numpy().tolist()
        return pos, rot

    def _reported_rad(self) -> torch.Tensor:
        """The arm's reported joint angles, in radians."""
        self._client.poll()
        return torch.tensor(
            [[a * _DEG2RAD for a in self._client.get_joint_angles()]],
            device=self._device,
            dtype=torch.float32,
        ).repeat(self._num_envs, 1)

    def update(self, target_rel_pos, target_rel_rot, dt: float) -> bool:
        """Advance the controller. Returns True if a command went out this tick."""
        target_pos = torch.tensor(
            np.asarray(target_rel_pos, dtype=np.float32), device=self._device
        ).repeat(self._num_envs, 1)
        target_rot = torch.tensor(
            np.asarray(target_rel_rot, dtype=np.float32), device=self._device
        ).repeat(self._num_envs, 1)

        # q and -q are the same rotation, so lerping toward an opposite-signed
        # quaternion takes the long way round -- and an exactly antipodal one
        # collapses the norm to zero and yields NaN joint targets.
        flip = (self.current_rel_rot * target_rot).sum(dim=-1, keepdim=True) < 0.0
        target_rot = torch.where(flip, -target_rot, target_rot)

        # Ease toward the target so a keypress does not become a step input.
        self.current_rel_pos += self._alpha * (target_pos - self.current_rel_pos)
        self.current_rel_rot += self._alpha * (target_rot - self.current_rel_rot)
        self.current_rel_rot /= torch.norm(self.current_rel_rot, dim=-1, keepdim=True)

        reported_rad = self._reported_rad()

        if self._suspended:
            return False

        self._time_since_send += dt
        if self._time_since_send < self._period:
            return False
        self._time_since_send = 0.0

        snap = self._backend.snapshot(reported_rad)
        world_pos, world_rot = self._backend.to_world(
            self.current_rel_pos, self.current_rel_rot
        )

        self._ik.set_command(torch.cat([world_pos, world_rot], dim=-1))
        joint_targets = self._ik.compute(
            ee_pos=snap.ee_pos,
            ee_quat=snap.ee_quat,
            jacobian=snap.jacobian,
            joint_pos=snap.joint_pos,
        )

        angles_deg = (joint_targets[0] * _RAD2DEG).detach().cpu().numpy()
        self._client.set_all_joint_angles(angles_deg)
        return True
