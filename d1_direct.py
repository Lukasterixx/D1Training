"""Direct PhysX drive for the D1 arm -- the stand-in for Rescue's DDS stack.

Rescue routes every arm command through CycloneDDS: a `D1SimServer` speaks the
real arm's wire protocol and a `D1Arm` client drives it, so the same code path
runs against hardware. This repo is a locomotion testbed and does not need that,
so `DirectD1` collapses the client and server into one object that writes joint
targets to the articulation. It exposes the subset of the client's surface that
`D1CartesianController` calls, so the controller port stayed almost untouched.

The threading split is inherited from Rescue and still load-bearing. Isaac's
keyboard callback runs on its own thread; the tensors belong to the sim loop.
Commands therefore land here as plain Python floats under a lock, and `apply()`
-- called only from the sim loop -- is the single place they become tensors.
Skipping that and calling `set_joint_position_target` from the key handler races
PhysX.
"""
from __future__ import annotations

import threading

import torch

# The protocol advertises a 65 mm jaw, but this URDF's fingers travel 30 mm
# each, so a full-open command saturates at 60 mm of real jaw.
GRIPPER_STROKE_MM = 65.0
GRIPPER_FINGER_TRAVEL_M = 0.03

_RAD2DEG = 57.29577951308232
_DEG2RAD = 0.017453292519943295


def gripper_mm_to_finger_m(stroke_mm: float) -> float:
    """Jaw opening (mm, as the protocol speaks) -> per-finger travel (m).

    The two fingers mirror each other, so each covers half the opening.
    """
    return min((stroke_mm / 2.0) / 1000.0, GRIPPER_FINGER_TRAVEL_M)


def finger_m_to_gripper_mm(finger_m: float) -> float:
    """Per-finger travel (m) -> jaw opening (mm). Inverse of the above."""
    return abs(finger_m) * 2.0 * 1000.0


class DirectD1:
    """Buffers arm commands and pushes them into PhysX from the sim loop."""

    def __init__(
        self,
        robot,
        arm_joint_names: list[str],
        gripper_joint_names: list[str],
        device,
        arm_stiffness: float,
        arm_damping: float,
        gripper_stiffness: float,
        gripper_damping: float,
    ):
        self._robot = robot
        self._device = device
        # Indices into the merged 20-joint articulation.
        self._arm_ids = [robot.find_joints(n)[0][0] for n in arm_joint_names]
        self._grip_ids = [robot.find_joints(n)[0][0] for n in gripper_joint_names]

        self._base_stiffness = [arm_stiffness] * len(self._arm_ids) + [gripper_stiffness] * len(self._grip_ids)
        self._base_damping = [arm_damping] * len(self._arm_ids) + [gripper_damping] * len(self._grip_ids)

        self._lock = threading.Lock()
        self._target_deg = [0.0] * len(self._arm_ids)
        self._gripper_mm = 0.0
        self._measured_deg = [0.0] * len(self._arm_ids)
        self._powered = True
        self._gains_dirty = True

    @property
    def arm_joint_ids(self) -> list[int]:
        return list(self._arm_ids)

    @property
    def gripper_joint_ids(self) -> list[int]:
        return list(self._grip_ids)

    # ------------------------------------------------ command side (any thread)

    def set_all_joint_angles(self, angles_deg) -> None:
        with self._lock:
            self._target_deg = [float(a) for a in angles_deg]

    def set_gripper(self, stroke_mm: float) -> None:
        with self._lock:
            self._gripper_mm = max(0.0, min(GRIPPER_STROKE_MM, float(stroke_mm)))

    def return_to_zero(self) -> None:
        """Home the arm to its folded zero pose, as the real D1 does."""
        with self._lock:
            self._target_deg = [0.0] * len(self._arm_ids)

    def set_motor_power(self, on: bool) -> None:
        with self._lock:
            self._powered = bool(on)
            self._gains_dirty = True

    def is_powered(self) -> bool:
        with self._lock:
            return self._powered

    # ----------------------------------------------------------- feedback side

    def poll(self) -> None:
        """No-op. Exists because the controller polls a DDS client here; the
        encoders are refreshed by `refresh()` from the sim loop instead."""

    def get_joint_angles(self) -> list[float]:
        with self._lock:
            return list(self._measured_deg)

    def get_gripper_mm(self) -> float:
        """The jaws' measured opening, in mm. Sim loop only (reads PhysX)."""
        finger_m = float(self._robot.data.joint_pos[0, self._grip_ids[0]].detach().cpu())
        return finger_m_to_gripper_mm(finger_m)

    # -------------------------------------------------------- sim loop only

    def refresh(self) -> None:
        """Read the simulated encoders. Sim loop only."""
        measured_rad = self._robot.data.joint_pos[0, self._arm_ids].detach().cpu().numpy()
        with self._lock:
            self._measured_deg = [float(a) * _RAD2DEG for a in measured_rad]

    def apply(self) -> None:
        """Push the buffered commands into PhysX. Sim loop only."""
        with self._lock:
            target_deg = list(self._target_deg)
            gripper_mm = self._gripper_mm
            powered = self._powered
            gains_dirty = self._gains_dirty
            self._gains_dirty = False

        n = self._robot.num_instances

        joint_targets = torch.tensor(
            [[a * _DEG2RAD for a in target_deg]], device=self._device, dtype=torch.float32
        ).repeat(n, 1)
        self._robot.set_joint_position_target(joint_targets, joint_ids=self._arm_ids)

        # Joint7_1 opens positive, Joint7_2 negative -- the URDF mirrors them.
        finger = gripper_mm_to_finger_m(gripper_mm)
        gripper_targets = torch.tensor(
            [[finger, -finger]], device=self._device, dtype=torch.float32
        ).repeat(n, 1)
        self._robot.set_joint_position_target(gripper_targets, joint_ids=self._grip_ids)

        if not gains_dirty:
            return

        # Cutting motor power releases the drives. Unlike Rescue -- where the
        # arm's root was teleported every tick and so could never build up any
        # falling motion -- the welded arm genuinely goes limp and sags here,
        # and the dog feels it do so.
        ids = self._arm_ids + self._grip_ids
        scale = 1.0 if powered else 0.0
        stiff = torch.tensor(
            [[b * scale for b in self._base_stiffness]], device=self._device, dtype=torch.float32
        ).repeat(n, 1)
        damp = torch.tensor(
            [[b * scale for b in self._base_damping]], device=self._device, dtype=torch.float32
        ).repeat(n, 1)
        self._robot.write_joint_stiffness_to_sim(stiff, joint_ids=ids)
        self._robot.write_joint_damping_to_sim(damp, joint_ids=ids)
        print(f"[D1] Motor power {'ON' if powered else 'OFF (e-stop): the arm will sag'}.")
