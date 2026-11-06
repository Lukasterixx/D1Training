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

# Go2 knee (calf joint): DERIVED, not measured. GO2_HV with its torques multiplied and its speeds divided by the knee's
# reduction, read off Unitree's Go2 URDF (`description/go2_d1.urdf`): the calf 45.43 N·m at 15.70 rad/s against the hip's
# and thigh's 23.7 at 30.1, the same factor 1.917 in torque and in speed, i.e. the same motor behind a geared knee.
# unitree_rl_lab and Isaac Lab's stock Go2 config give the knee the hip's limits; Unitree's Go2 page gives ~45 N·m as the
# largest joint motor's peak. Used by the leg task's actuator diagnostics (`unifp_train/leg_actuators.py`) until the
# knee's torque is measured on the dog.
GO2_KNEE_RATIO = 45.43 / 23.7
GO2_HV_KNEE = UnitreeMotorSpec(Y1=GO2_HV.Y1 * GO2_KNEE_RATIO, Y2=GO2_HV.Y2 * GO2_KNEE_RATIO,
                               X1=GO2_HV.X1 / GO2_KNEE_RATIO, X2=GO2_HV.X2 / GO2_KNEE_RATIO)


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

# Joint1 and Joint4-6: Unitree D1-550 published joint torques (README, "The arm's mass"): 3.3 N·m, then 1.7.
# Joint2 and Joint3: 3.33 N·m, URDF-unverified. Unitree's D1-550 URDF gives both 3.33, and in Unitree's CAD the shoulder
# and elbow servos have identical housings (F-137), so Lukas took them for the same motor (2026-10-09). The published
# table gives Joint3 1.7, which this used until then. Lukas confirmed on the arm (2026-10-18) that both are a Fashion Star
# RX18-U100H-M, published at 3.14 N·m rated, 4.90 continuous, 9.80 stall at 12 V (F-142). What the firmware lets the
# servo deliver on the arm is not measured.
D1_EFFORT_LIMIT_NM = dict(zip(D1_ARM_JOINTS, (3.3, 3.33, 3.33, 1.7, 1.7, 1.7)))

# MEASURED on the arm, 2026-09-16 (F-033): a 30 deg step on each joint in turn, commanded through
# funcode 2 mode 0 (the fast path -- mode 1 slews at a fifth of this, F-031), peak rate taken from
# consecutive feedback samples.
#
# The URDF's previous 1.05/1.05/1.05/1.73/1.73/1.73 was NOT Unitree data -- d1_description ships
# velocity="0" and the numbers were filled in when Rescue ported the arm with no recorded source. The
# measurement shows no such split: every joint tops out between 1.20 and 1.29 rad/s, which reads as one
# controller-wide speed ceiling rather than a per-joint mechanical limit. The old values were therefore
# wrong in both directions -- too low for Joint1-3 and, more dangerously, too high for Joint4-6, where
# simulation allowed the arm 40% more speed than the hardware delivers.
#
# These are LOWER bounds: the 111 ms feedback cycle (F-020) means a 30 deg move spans only ~4 samples,
# so the true peak is at least this. Unloaded, one posture per joint, one arm.
D1_VELOCITY_LIMIT_RAD_S = dict(zip(D1_ARM_JOINTS, (1.25, 1.29, 1.23, 1.21, 1.25, 1.25)))

# D1 interface timing. The arm publishes joint angles only -- no torque, no velocity.
#
# Measured on the physical arm, 2026-09-16 (F-020): over 120 s of DDS capture with the arm at rest,
# `current_servo_angle` and the `funcode 1` stream on `rt/arm_Feedback` both ran at a 111.0 ms period
# (8.999 Hz, stdev 0.5 ms), quantised to 0.1 deg. The 10 Hz the SDK headers implied is real but belongs
# to the `funcode 3` status stream (enable/power/error, 100.5 ms), not to the angles.
D1_FEEDBACK_HZ = 9.0  # measured: 8.9986 Hz over 120 s

# NOT measured. The d1_sdk samples and the VIP-Rescue driver both stream setpoints at 10 Hz, and that
# is what this models, but the arm's maximum accepted command rate was never tested -- only that single
# commands are honoured.
D1_COMMAND_HZ = 10.0

# What the UniFP task's arm link draws from (`unifp_train/arm_link.py`). Lukas, 2026-11-06: "the real arm takes commands
# at 9-10 hz". Stated, not measured here; `d1_hardware.MAX_COMMAND_HZ` is the ceiling, because past it the arm stops
# responding (F-032).
D1_COMMAND_HZ_RANGE = (9.0, 10.0)

# MEASURED (F-020): the joint angles are reported to 0.1 deg.
D1_FEEDBACK_QUANTUM_DEG = 0.1

# Under a stream of commands the arm drops them for 0.3-0.7 s on occasion (Lukas, 2026-11-06), and its angle reports stop
# with them (F-200). Duration: stated, and F-200's gaps agree (to 0.71 s). How often is ESTIMATED from one 39 s capture at
# 10 Hz (19 gaps, 8.1 s of 38.8, F-200; fewer at lower rates, F-202): these ranges give that capture's 21% of the time.
# Lukas's "on occasion" may be rarer, so this interval is the frequent end. What the arm does in one is read from commands
# against the next reports, not measured: it moved ~5 deg through a gap asked for 15-20 (F-200), about one 4 deg command,
# so the model finishes the move it has and takes no new one.
D1_STALL_INTERVAL_S = (1.0, 2.8)
D1_STALL_DURATION_S = (0.3, 0.7)

# The locked stream (2026-11-06, F-238/F-239 in the robot program's numbering, run on the other PC): one command 20 ms after
# each `current_servo_angle` report, so a true 111 ms stream at the reports' own rate. MEASURED on the arm: 3 skipped reports
# in 2,059 cycles, each ~0.21 s, cause unknown (three events do not fit a rate: light noise), against 73 in 710 free-running.
# Commands 98-106 ms after a report are what skip.
D1_LOCK_OFFSET_S = 0.02
D1_LOCKED_SKIP_PROBABILITY = 0.0015
# The arm in that stream. FITTED 2026-11-06 to the raw reports of five locked Joint1 triangle blocks (+-10 deg at 36 deg/s, 4 deg
# a command, ~900 reports), driven by the commands as sent: a first-order lag behind a speed-limited target, 0.43 deg RMS, 0.35
# held out. `unifp_train/arm_link_check.py` is the comparison. An earlier fit to summary figures gave lag 80, dead 20, age 40 ms,
# 1.95 deg RMS: it matched a 150 ms lag on a measure that reads 98 on the arm. F-045's planner with 19 ms of report age is close
# behind at 0.53 and fits the 30 deg steps far better (0.34 deg against 0.95): this profile is for streams, F-045 for big steps.
# Acceleration, deceleration and velocity retention add nothing here, so the plan only limits speed. Delay and lag trade off
# (dead + age 0: lag 82 ms, 0.433; 20 ms: 63-65, 0.45), so training draws those ranges. Fitted to Joint1, unloaded, dog lying down.
# HELD OUT on the six-joint holster draw with the key in the jaws: 0.23-0.57 deg RMS. There the shoulder and elbow fit best at 65-70
# ms and the wrist at 50-55, so this one lag is about right for the first and slow for the second.
D1_LOCKED_SERVO_LAG_S = 0.077
D1_LOCKED_SERVO_LAG_RANGE_S = (0.06, 0.09)
D1_LOCKED_REPORT_AGE_S = 0.0
D1_LOCKED_REPORT_AGE_RANGE_S = (0.0, 0.02)
D1_LOCKED_DEAD_TIME_S = 0.005
D1_LOCKED_PLAN_ACCEL_RAD_S2 = 400.0   # the step fit's bound: with the servo lag the plan only limits speed

# The D1 firmware's commanded motion: what happens between a setpoint arriving and the joint getting
# there. FITTED to recorded hardware, 2026-09-17 (F-045), by `position_only/arm_response.py`, which
# simulates `core.TrapezoidTracker` -- the implementation the task uses -- against the raw 9 Hz samples
# of the six 30 deg single-joint steps (F-033 sweeps, 12 legs, 234 samples, 0.34 deg RMS) and the F-035
# streaming cap sweep.
#
# These replace two figures that were artefacts of the 111 ms feedback cycle, not properties of the arm:
# F-021's "~127 ms command-to-motion delay" is mostly the wait for the next sample (a 127 ms dead time
# fits the steps 7x worse), and F-035's "two cycles to reach cruise" is how a ~80 ms ramp looks when it
# is sampled every 111 ms.
#
# Dead time and acceleration trade off on single steps -- every dead time from 0 to 40 ms fits within
# 0.02 deg RMS -- so the streaming sweep decides: 0-10 ms reproduce it best, and 10 ms is taken as the
# best streaming fit among the step ties. Treat 0-10 ms as the supported range, not 10 ms as a
# measurement. The feedback transport age is folded into it, so the loop's dominant real latency remains
# the 111 ms feedback period, which `D1_FEEDBACK_HZ` already models.
D1_COMMAND_DEAD_TIME_S = 0.010        # fitted range 0-10 ms
D1_PLAN_ACCEL_RAD_S2 = 15.5           # fitted at that dead time (12.5 at 0 ms)
D1_PLAN_DECEL_RAD_S2 = 17.4           # fitted
# Fraction of planned speed a new setpoint keeps. FITTED to the F-035 cap sweep: streaming a waypoint
# per cycle covered 4.9-5.3 deg per 111 ms where a speed-preserving planner (1.0) gives 7.8; restarting
# from rest (0.0) gives 5.0. Provisional: that sweep ran a Cartesian loop that re-solved IK from each
# fresh sample, and at a 5 deg cap the model covers 4.3 deg/cycle against 2.5 measured.
D1_REPLAN_VELOCITY_RETENTION = 0.0

# Go2 legs: unitree_rl_lab's controller publishes LowCmd from a 1 kHz thread that picks up the 50 Hz
# policy's latest action. Isaac Lab counts actuator delay in physics steps (5 ms here); 0-2 steps
# covers pickup, state age, inference and transport. An estimate from the loop structure, not a measurement.
GO2_LEG_DELAY_PHYSICS_STEPS = (0, 2)


def arm_trajectory(profile: str) -> dict | None:
    """The D1 firmware's motion planner for a profile: "measured" (F-045), "locked" (a stream of small steps, with
    `D1_LOCKED_SERVO_LAG_S` behind it) or "none" (setpoints act at once).

    Joint speed ceilings are not repeated here; they are `D1_VELOCITY_LIMIT_RAD_S`, which the planner
    and PhysX share.
    """
    if profile == "none":
        return None
    if profile == "locked":
        return {"dead_time_s": D1_LOCKED_DEAD_TIME_S, "accel_rad_s2": D1_LOCKED_PLAN_ACCEL_RAD_S2,
                "decel_rad_s2": D1_LOCKED_PLAN_ACCEL_RAD_S2, "replan_velocity_retention": D1_REPLAN_VELOCITY_RETENTION}
    if profile != "measured":
        raise ValueError(f"Unknown arm trajectory profile: {profile!r}")
    return {
        "dead_time_s": D1_COMMAND_DEAD_TIME_S,
        "accel_rad_s2": D1_PLAN_ACCEL_RAD_S2,
        "decel_rad_s2": D1_PLAN_DECEL_RAD_S2,
        "replan_velocity_retention": D1_REPLAN_VELOCITY_RETENTION,
    }


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
