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


# ---------------------------------------------------------------------------- D1-550 arm
# Unitree publishes no torque-speed curve for the D1's servos, so the D1 model is the published
# torque limits, the speed limits the URDF carries, and the SDK's interface timing. Each value
# is labelled with its source; replace them once the arm is measured. flat_env_cfg applies the
# limits to force drives: the URDF import's acceleration drives let the arm sag (F-010).

D1_ARM_JOINTS = [f"Joint{i}" for i in range(1, 7)]

# Unitree D1-550 published joint torques (README, "The arm's mass"): first two joints 3.3 N·m, last four 1.7.
D1_EFFORT_LIMIT_NM = dict(zip(D1_ARM_JOINTS, (3.3, 3.3, 1.7, 1.7, 1.7, 1.7)))

# From d1_arm/d1.urdf. NOT Unitree data: Unitree's d1_description ships velocity="0"; these were
# filled in when Rescue ported the arm (2026-07-16) with no recorded source.
D1_VELOCITY_LIMIT_RAD_S = dict(zip(D1_ARM_JOINTS, (1.05, 1.05, 1.05, 1.73, 1.73, 1.73)))

# D1 SDK interface (Rescue d1_sdk, reproduced from the SDK's headers and samples): the arm publishes
# joint angles only, at 10 Hz, and takes streamed joint-angle setpoints at about 10 Hz.
D1_FEEDBACK_HZ = 10.0
D1_COMMAND_HZ = 10.0

# Go2 legs: unitree_rl_lab's controller publishes LowCmd from a 1 kHz thread that picks up the 50 Hz
# policy's latest action. Isaac Lab counts actuator delay in physics steps (5 ms here); 0-2 steps
# covers pickup, state age, inference and transport. An estimate from the loop structure, not a measurement.
GO2_LEG_DELAY_PHYSICS_STEPS = (0, 2)


def interface_timing(latency: str, policy_hz: float, leg_actuator: str) -> dict:
    """Interface timing for a latency profile, in the units the config uses."""
    if latency == "none":
        return {"leg_delay_physics_steps": (0, 0), "arm_command_hold_steps": 1, "arm_feedback_period_steps": 1}
    if latency != "estimated":
        raise ValueError(f"Unknown latency profile: {latency!r}")
    return {
        # Isaac Lab's delay buffer lives in the explicit actuator; the stock DCMotor legs get none.
        "leg_delay_physics_steps": GO2_LEG_DELAY_PHYSICS_STEPS if leg_actuator == "unitree" else (0, 0),
        "arm_command_hold_steps": round(policy_hz / D1_COMMAND_HZ),
        "arm_feedback_period_steps": round(policy_hz / D1_FEEDBACK_HZ),
    }
