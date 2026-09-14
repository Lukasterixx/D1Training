"""Unitree motor model as an Isaac Lab actuator: explicit PD, delay buffer, measured envelope.

Adapted from unitreerobotics/unitree_rl_lab (Apache-2.0; `assets/robots/unitree_actuators.py`,
as shipped in mairo-rl-lab-rinam commit a179aa0). Changes: the envelope and friction maths
call `motor_model.py`, and only the Go2 motor configuration is kept. Licence:
third_party/unitree_rl_lab/LICENCE.

Compared with Isaac Lab's stock Go2 `DCMotor` (23.5 N·m falling linearly to zero at 30 rad/s),
this keeps full torque up to the measured 13.5 rad/s knee and distinguishes driving from
braking torque. Import only after Isaac Sim has started.
"""
from __future__ import annotations

from dataclasses import MISSING

import torch
from isaaclab.actuators import DelayedPDActuator, DelayedPDActuatorCfg
from isaaclab.utils import configclass
from isaaclab.utils.types import ArticulationActions

from motor_model import GO2_HV, clip_unitree_effort, joint_friction


class UnitreeActuator(DelayedPDActuator):
    cfg: "UnitreeActuatorCfg"

    def __init__(self, cfg: "UnitreeActuatorCfg", *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)
        self._joint_vel = torch.zeros_like(self.computed_effort)
        self._effort_y1 = self._parse_joint_parameter(cfg.Y1, 1e9)
        self._effort_y2 = self._parse_joint_parameter(cfg.Y2, cfg.Y1)
        self._velocity_x1 = self._parse_joint_parameter(cfg.X1, 1e9)
        self._velocity_x2 = self._parse_joint_parameter(cfg.X2, 1e9)
        self._friction_static = self._parse_joint_parameter(cfg.Fs, 0.0)
        self._friction_dynamic = self._parse_joint_parameter(cfg.Fd, 0.0)
        self._activation_vel = self._parse_joint_parameter(cfg.Va, 0.01)

    def compute(self, control_action: ArticulationActions, joint_pos, joint_vel) -> ArticulationActions:
        self._joint_vel[:] = joint_vel  # `_clip_effort` needs the speed; the base class does not pass it.
        control_action = super().compute(control_action, joint_pos, joint_vel)
        self.applied_effort -= joint_friction(
            joint_vel, self._friction_static, self._friction_dynamic, self._activation_vel)
        control_action.joint_positions = None
        control_action.joint_velocities = None
        control_action.joint_efforts = self.applied_effort
        return control_action

    def _clip_effort(self, effort):
        return clip_unitree_effort(effort, self._joint_vel, self._effort_y1, self._effort_y2,
                                   self._velocity_x1, self._velocity_x2)


@configclass
class UnitreeActuatorCfg(DelayedPDActuatorCfg):
    class_type: type = UnitreeActuator
    X1: float = 1e9
    """Speed at which full torque starts to fall (rad/s)."""
    X2: float = 1e9
    """No-load speed (rad/s)."""
    Y1: float = MISSING
    """Peak torque with torque and speed in the same direction (N·m)."""
    Y2: float | None = None
    """Peak torque with torque opposing speed (N·m); defaults to Y1."""
    Fs: float = 0.0
    """Static friction (N·m)."""
    Fd: float = 0.0
    """Viscous friction (N·m·s/rad)."""
    Va: float = 0.01
    """Speed at which static friction is fully active (rad/s)."""


@configclass
class UnitreeGo2HVActuatorCfg(UnitreeActuatorCfg):
    X1 = GO2_HV.X1
    X2 = GO2_HV.X2
    Y1 = GO2_HV.Y1
    Y2 = GO2_HV.Y2
