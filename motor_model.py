"""Joint motor limits as plain torch maths, so they can be checked without Isaac Sim.

The Unitree envelope and friction terms are adapted from unitreerobotics/unitree_rl_lab
(Apache-2.0; `assets/robots/unitree_actuators.py`, as shipped in mairo-rl-lab-rinam
commit a179aa0). The code is restructured into functions; the maths is unchanged.
Licence: third_party/unitree_rl_lab/LICENCE. `unitree_actuators.py` wraps these
functions in an Isaac Lab actuator.

Unitree envelope (torque limit against joint speed)::

    Y2 ──┐                        Y1: peak torque, torque and speed in the same direction
         │─────────── Y1          Y2: peak torque, torque opposing the speed
         │           │\           X1: speed at which full torque starts to fall
         │           │ \          X2: no-load speed (limit reaches zero)
         └───────────┴──┴──> |speed|
                    X1  X2
"""
from __future__ import annotations

from dataclasses import dataclass

import torch


@dataclass(frozen=True)
class UnitreeMotorSpec:
    """Envelope parameters in N·m and rad/s; friction in N·m and N·m·s/rad."""

    Y1: float
    Y2: float
    X1: float
    X2: float
    Fs: float = 0.0
    Fd: float = 0.0
    Va: float = 0.01


# Go2 hip/thigh/calf motor, values from unitree_rl_lab `UnitreeActuatorCfg_Go2HV`.
GO2_HV = UnitreeMotorSpec(Y1=20.2, Y2=23.4, X1=13.5, X2=30.0)


def unitree_effort_limit(effort, joint_vel, y1, y2, x1, x2):
    """Magnitude of the torque available at `joint_vel` for a demand of `effort`."""
    peak = torch.where(joint_vel * effort > 0, y1, y2)
    falling = (-peak / (x2 - x1) * (joint_vel.abs() - x1) + peak).clip(min=0.0)
    return torch.where(joint_vel.abs() < x1, peak, falling)


def clip_unitree_effort(effort, joint_vel, y1, y2, x1, x2):
    limit = unitree_effort_limit(effort, joint_vel, y1, y2, x1, x2)
    return torch.clip(effort, -limit, limit)


def joint_friction(joint_vel, static, dynamic, activation_vel):
    """Torque lost to friction: smoothed Coulomb term plus viscous term."""
    return static * torch.tanh(joint_vel / activation_vel) + dynamic * joint_vel


def dc_motor_effort_bounds(joint_vel, saturation_effort, effort_limit, velocity_limit):
    """(min, max) torque of Isaac Lab's `DCMotor`, which the stock Go2 config uses.

    Mirrors `isaaclab.actuators.DCMotor._clip_effort`, for comparison with the Unitree envelope.
    """
    vel_at_limit = velocity_limit * (1.0 + effort_limit / saturation_effort)
    vel = joint_vel.clip(-vel_at_limit, vel_at_limit)
    top = (saturation_effort * (1.0 - vel / velocity_limit)).clip(max=effort_limit)
    bottom = (saturation_effort * (-1.0 - vel / velocity_limit)).clip(min=-effort_limit)
    return bottom, top
