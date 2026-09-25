"""The pieces the combiner demo needs to run the mechanism-trained policy: a claw, a force law, a roll.

The policy (`unifp_train/mech_env.py`, `--force_law`, F-108) was trained holding a handle through a
3-D spring -- a claw, not a friction grip -- and given two things by a task layer: a goal *on the
handle*, and a force command along the mechanism's path from a PI law on how far the handle lags its
reference. This module is those three things for the real box, in plain torch, so they can be
checked without a simulator (`tests/test_mech_demo.py`):

  * `ClawCoupling`: the claw. Once it hooks the lever bar, a spring-damper joins the jaw centre to a
    point fixed on the lever, and tears out above a grip limit -- the training's grasp exactly,
    with its evaluation constants (`unifp_train.mech_cfg.GRASP_*`, `GRIP_N`). It is an assumption
    about the tool (Lukas: "assume I will turn the pincers into claws"), not a model of the D1's
    pads.
  * `PlaneForceLaw`: the task layer's law. Training had one degree of freedom and a tangent; the box
    has two -- the lever about its spindle and the door about its hinge -- so the lag is projected
    onto the plane the handle can move in (both joints' directions at the measured angles) before
    the PI law acts. Along the directions the box is rigid nothing is commanded, so the integral
    cannot wind up against the enclosure.
  * `jaw_roll`: the roll command that puts the jaws across the bar, measured the way the training
    measures roll (`unifp_train.rewards.tool_roll`).
"""
from __future__ import annotations

import math

import torch


def unit(vector: torch.Tensor) -> torch.Tensor:
    return vector / vector.norm(dim=-1, keepdim=True).clamp(min=1e-9)


def jaw_roll(approach: torch.Tensor, jaw_axis: torch.Tensor, forward: torch.Tensor) -> torch.Tensor:
    """Roll of `jaw_axis` about `approach`, from the horizontal, wrapped to (-pi/2, pi/2].

    The same construction as `rewards.tool_roll`, so a roll computed here and commanded is the
    quantity the policy was trained to track: reference = up x approach (the horizontal direction
    across the approach), falling back to forward x approach where the approach is vertical.
    """
    up = torch.zeros_like(approach)
    up[:, 2] = 1.0
    reference = torch.cross(up, approach, dim=-1)
    degenerate = reference.norm(dim=-1, keepdim=True) < 1e-4
    reference = torch.where(degenerate, torch.cross(forward, approach, dim=-1), reference)
    reference = unit(reference)
    second = torch.cross(approach, reference, dim=-1)
    angle = torch.atan2((jaw_axis * second).sum(-1), (jaw_axis * reference).sum(-1))
    return torch.remainder(angle + math.pi / 2, math.pi) - math.pi / 2


class ClawCoupling:
    """A claw on a lever: a spring-damper from the jaw centre to a point fixed on the lever.

    `engage` fixes the anchor where the jaw centre is at that moment, in the lever's own frame, so
    there is no initial pull. Above `grip_n` of spring force the lever is torn out of the claw and the
    coupling lets go for the rest of the attempt.
    """

    def __init__(self, num_envs: int, device, *, stiffness: float, damping: float, grip_n: float):
        self.stiffness, self.damping, self.grip_n = stiffness, damping, grip_n
        self.device = torch.device(device)
        self.engaged = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.torn = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        #: Anchor in the lever body's frame, m.
        self.anchor_local = torch.zeros(num_envs, 3, device=self.device)
        #: Force on the tool as of the last `force`, world frame, N; and the spring's part of it.
        self.force_on_tool = torch.zeros(num_envs, 3, device=self.device)
        self.spring_n = torch.zeros(num_envs, device=self.device)

    def engage(self, ids: torch.Tensor, anchor_local: torch.Tensor) -> None:
        self.engaged[ids] = True
        self.torn[ids] = False
        self.anchor_local[ids] = anchor_local

    def release(self, ids: torch.Tensor) -> None:
        self.engaged[ids] = False
        self.force_on_tool[ids] = 0.0
        self.spring_n[ids] = 0.0

    def reset(self, ids: torch.Tensor) -> None:
        self.release(ids)
        self.torn[ids] = False

    def force(self, tool_pos: torch.Tensor, tool_vel: torch.Tensor, anchor_pos: torch.Tensor,
              anchor_vel: torch.Tensor) -> torch.Tensor:
        """Force on the tool (N, 3), world frame; the reaction goes on the lever at the anchor."""
        spring = -self.stiffness * (tool_pos - anchor_pos)
        force = spring - self.damping * (tool_vel - anchor_vel)
        magnitude = spring.norm(dim=-1)
        torn = self.engaged & (magnitude > self.grip_n)
        self.torn |= torn
        self.engaged &= ~torn
        on = self.engaged.unsqueeze(-1)
        self.force_on_tool = torch.where(on, force, torch.zeros_like(force))
        self.spring_n = torch.where(self.engaged, magnitude, torch.zeros_like(magnitude))
        return self.force_on_tool


def project_onto(vector: torch.Tensor, jacobian: torch.Tensor) -> torch.Tensor:
    """The part of each `vector` (N, 3) in the span of `jacobian`'s columns (N, 3, k), least squares."""
    jt = jacobian.transpose(1, 2)
    gram = jt @ jacobian + 1e-9 * torch.eye(jacobian.shape[-1], device=jacobian.device)
    coeff = torch.linalg.solve(gram, jt @ vector.unsqueeze(-1))
    return (jacobian @ coeff).squeeze(-1)


class PlaneForceLaw:
    """`F = kp lag + ki integral(lag)` on the lag projected onto the mechanism's directions.

    The training law (`unifp_train.mech_env.Go2D1MechEnv._step_forces`) in vector form: the integral
    is dropped wherever it points against the lag (anti-windup), decays with time constant `bleed_s`
    once the handle is within `arrived_m` of its reference (without it, the integral holds its force
    against a stop -- F-108's torn buttons), and the result is clamped to `max_n` in magnitude.
    """

    def __init__(self, num_envs: int, device, *, kp: float, ki: float, max_n: float,
                 bleed_s: float | None, arrived_m: float = 0.005):
        self.kp, self.ki, self.max_n, self.bleed_s, self.arrived_m = kp, ki, max_n, bleed_s, arrived_m
        self.integral = torch.zeros(num_envs, 3, device=torch.device(device))
        self.command = torch.zeros(num_envs, 3, device=torch.device(device))

    def reset(self, ids: torch.Tensor) -> None:
        self.integral[ids] = 0.0
        self.command[ids] = 0.0

    def step(self, lag: torch.Tensor, active: torch.Tensor, dt: float) -> torch.Tensor:
        """Advance one policy step; `lag` (N, 3) already projected. Returns the command, world, N."""
        on = active.unsqueeze(-1)
        integral = self.integral + lag * dt
        against = (integral * lag).sum(-1, keepdim=True) < 0.0
        integral = torch.where(against, torch.zeros_like(integral), integral)
        if self.bleed_s:
            arrived = (lag.norm(dim=-1, keepdim=True) < self.arrived_m)
            integral = torch.where(arrived, integral * math.exp(-dt / self.bleed_s), integral)
        self.integral = torch.where(on, integral, torch.zeros_like(integral))
        force = self.kp * lag + self.ki * self.integral
        magnitude = force.norm(dim=-1, keepdim=True)
        force = force * (self.max_n / magnitude.clamp(min=1e-9)).clamp(max=1.0)
        self.command = torch.where(on, force, torch.zeros_like(force))
        return self.command
