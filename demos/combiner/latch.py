"""Latch approximation: constrain the door hinge until the measured lever turns.

This models the task sequence with a joint-limit constraint, not a simulated cam
and sliding bolt. The lever itself is a physical joint with a return spring.
"""
from dataclasses import dataclass
import math

from .geometry import GEOMETRY


@dataclass
class DoorLatch:
    latched: bool = True

    def reset(self, door_deg=0.0, handle_deg=0.0):
        # An explicitly open initial inspection pose starts outside the strike.
        self.latched = door_deg <= GEOMETRY.latch_capture_deg and handle_deg < GEOMETRY.handle_release_deg

    def update(self, door_deg, handle_deg):
        if not all(math.isfinite(v) for v in (door_deg, handle_deg)):
            raise ValueError("Non-finite door or handle angle")
        if handle_deg >= GEOMETRY.handle_release_deg:
            self.latched = False
        elif not self.latched and door_deg <= GEOMETRY.latch_capture_deg and handle_deg <= GEOMETRY.handle_reset_deg:
            self.latched = True
        return self.latched

    @property
    def door_upper_limit_deg(self):
        return GEOMETRY.latch_capture_deg if self.latched else GEOMETRY.open_limit_deg


class LatchController:
    """Single-box Isaac Lab adapter, called before each 200 Hz physics step.

    H/O/C/F are optional inspection drives. Without them, only the latch constraint
    and the handle's return spring act; contacts can depress the lever and pull the door.
    Joint indices are resolved by name because PhysX ordering is not a contract.

    The spring is the handle's implicit drive: its target is the rest angle below the stop
    (the preload), or `handle_hold_deg` while H holds the lever down. `set_spring` resizes
    it between attempts, which is how a torque sweep runs without rebuilding the scene.
    """

    def __init__(self, box):
        import torch

        self.torch = torch
        self.box = box
        self.door_id = box.joint_names.index("DoorHinge")
        self.handle_id = box.joint_names.index("HandleJoint")
        self.latch = DoorLatch()
        self.enabled = False
        self.handle_held = False
        self.door_target_deg = None
        self.transitions = []
        self.time_s = 0.0
        self.last_latched = None
        self.spring_torque_nm = None

    def angles(self):
        q = self.box.data.joint_pos[0].detach().cpu().tolist()
        return math.degrees(q[self.door_id]), math.degrees(q[self.handle_id])

    def _apply_limit(self):
        if self.last_latched == self.latch.latched:
            return
        limits = self.torch.tensor([[[0.0, math.radians(self.latch.door_upper_limit_deg)]]], device=self.box.device)
        self.box.write_joint_position_limit_to_sim(limits, joint_ids=[self.door_id], warn_limit_violation=False)
        self.last_latched = self.latch.latched
        self.transitions.append({"time_s": self.time_s, "latched": self.latch.latched})

    def reset(self, door_deg=0.0, handle_deg=0.0):
        with self.torch.inference_mode():
            self.enabled = False
            self.handle_held = False
            self.door_target_deg = None
            self.time_s = 0.0
            self.transitions = []
            self.last_latched = None
            self.latch.reset(door_deg, handle_deg)
            self._apply_limit()
            q = self.box.data.default_joint_pos.clone()
            q[:, self.door_id] = math.radians(door_deg)
            q[:, self.handle_id] = math.radians(handle_deg)
            zeros = self.torch.zeros_like(q)
            self.box.write_joint_state_to_sim(q, zeros)
            rest = zeros.clone()
            rest[:, self.handle_id] = math.radians(GEOMETRY.spring_rest_deg)
            self.box.set_joint_position_target(rest)
            self.box.set_joint_velocity_target(zeros)
            self.box.set_joint_effort_target(zeros)
            self.box.reset()
            self.box.write_data_to_sim()
            self.enabled = True

    def set_spring(self, torque_nm):
        """Resize the return spring to need `torque_nm` at 45 degrees (`GEOMETRY.spring_stiffness`)."""
        stiffness = GEOMETRY.spring_stiffness(torque_nm)
        self.box.write_joint_stiffness_to_sim(stiffness, joint_ids=[self.handle_id])
        self.box.write_joint_effort_limit_to_sim(GEOMETRY.spring_effort_limit(torque_nm), joint_ids=[self.handle_id])
        self.spring_torque_nm = torque_nm
        return stiffness

    def command(self, key):
        if key == "H":
            self.handle_held = not self.handle_held
        elif key == "O":
            if self.latch.latched:
                return False
            self.door_target_deg = 90.0
        elif key == "C":
            self.door_target_deg = 0.0
        elif key == "F":
            self.handle_held = False
            self.door_target_deg = None
        else:
            raise ValueError(f"Unknown inspection key: {key}")
        return True

    def on_physics_step(self, dt):
        if not self.enabled:
            return
        with self.torch.inference_mode():
            door_deg, handle_deg = self.angles()
            self.latch.update(door_deg, handle_deg)
            self._apply_limit()
            # The return spring targets its rest angle below the stop; holding H depresses it past the release.
            target_deg = GEOMETRY.handle_hold_deg if self.handle_held else GEOMETRY.spring_rest_deg
            target = self.torch.full((1, 1), math.radians(target_deg), device=self.box.device)
            self.box.set_joint_position_target(target, joint_ids=[self.handle_id])
            effort = 0.0
            if self.door_target_deg is not None:
                velocity = float(self.box.data.joint_vel[0, self.door_id])
                error = math.radians(self.door_target_deg - door_deg)
                # A bounded inspection pull, not an instantaneous pose write.
                effort = max(-0.6, min(0.6, 1.5 * error - 0.3 * velocity))
                tolerance = GEOMETRY.latch_capture_deg if self.door_target_deg == 0 else 1.0
                if abs(math.degrees(error)) <= tolerance and abs(velocity) < 0.05:
                    effort = 0.0
                    self.door_target_deg = None
            self.box.set_joint_effort_target(self.torch.full((1, 1), effort, device=self.box.device),
                                             joint_ids=[self.door_id])
            # InteractiveScene wrote its targets before the physics callback. Flush our
            # changes now so they apply to this substep, rather than one control step later.
            self.box.write_data_to_sim()
            self.time_s += dt
