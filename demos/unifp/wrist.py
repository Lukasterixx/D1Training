"""A wrist servo for the demos that are allowed to override UniFP's last actions.

UniFP's command vector carries a tool-tip position and no orientation: `CMD_EE_ORN_R/P/Y` are
declared in `interface.py` and the task never writes them. The arm has six joints and the goal
constrains three numbers, so three degrees of freedom are unconstrained and the policy has no term
that holds them. Measured on the deliverable checkpoint at 105 held goals, the jaw axis sits
anywhere between 4 and 87 degrees from horizontal and swings a median of 20 degrees (worst 93)
while the goal does not move at all. A grasp cannot be closed on that.

This is the smallest thing that fixes it without retraining, and *which* joints it takes matters
more than anything else about it:

  * **`Joint6` is the roll about the approach axis.** Rolling it sweeps the jaw axis through every
    direction perpendicular to the approach, so it can always bring the jaws level -- and the jaw
    centre lies on that axis, so rolling barely moves the point the grasp is placed by. It costs
    the policy almost nothing.
  * **`Joint5` is the wrist pitch**, and it swings the tool point about 20 cm per radian. Taking it
    from the policy takes away part of how the policy was reaching, and the measured price is
    large: 8 cm of tracking error where the policy alone had 3.

So the default is the roll alone. With it, the desired jaw axis is built from the hand's
*measured* approach rather than the one the script wanted, because a roll cannot change the
approach and asking it to is asking for an error it can only answer with the wrong rotation.

**This is a modification, not the policy.** The substituted actions are fed back into the
observation, so the policy sees what was applied rather than what it asked for, and compensates
through the joints it still owns. Any result from this path is evidence about "UniFP plus a wrist
servo" and must not be reported as UniFP.
"""
from __future__ import annotations

import torch

#: Indices in UniFP's 18-action vector (12 legs, then Joint1..6).
JOINT5_ACTION_INDEX = 16
JOINT6_ACTION_INDEX = 17

#: Joint axes in each child link's own frame, from `d1_arm/d1.urdf`. Both are its -z.
JOINT_LOCAL_AXES = {JOINT5_ACTION_INDEX: (0.0, 0.0, -1.0),
                    JOINT6_ACTION_INDEX: (0.0, 0.0, -1.0)}

#: Fraction of the remaining orientation error commanded each step. The joint is driven by
#: UniFP's PD -- 40 N·m/rad, clipped at the URDF's 1.7 N·m -- so a target that leads the measured
#: angle by more than 0.043 rad already saturates the torque and the joint moves as fast as it
#: can. Commanding a rate instead (error x gain x dt) leads by a thousandth of that and the wrist
#: barely turns, which is what a first version of this did.
DEFAULT_GAIN = 1.0

#: Levenberg damping on the normal equations. Bounds the correction where the joint axes cannot
#: produce the rotation asked of them.
DEFAULT_DAMPING = 0.05

#: Largest angle step commanded in one policy step, rad. Not a rate limit -- the PD and its effort
#: limit set the rate -- but a bound on how far ahead of the arm a target may be placed.
MAX_STEP_RAD = 0.6

#: How much the approach term counts against the jaw term when the pitch is servoed too. A grasp
#: is decided by the jaw axis: both pads have to meet the cup's wall at the same height, or the
#: lever's bar on opposite sides. Two joints cannot service three rotational degrees of freedom,
#: so asking equally for both spends the wrist on the half that does not close the grasp.
APPROACH_WEIGHT = 0.2


def unit(vector: torch.Tensor) -> torch.Tensor:
    return vector / vector.norm(dim=-1, keepdim=True).clamp(min=1e-9)


def desired_jaw_axis(approach: torch.Tensor, object_axis: torch.Tensor) -> torch.Tensor:
    """The axis the jaws should open along: across both the approach and the object.

    Falls back to any perpendicular of the approach where the two are parallel -- coming straight
    down a cup's own axis, say, where every jaw angle is as good as every other.
    """
    axis = torch.cross(approach, object_axis, dim=-1)
    degenerate = axis.norm(dim=-1) < 1e-6
    if degenerate.any():
        up = torch.tensor([0.0, 0.0, 1.0], device=approach.device).expand_as(approach)
        forward = torch.tensor([1.0, 0.0, 0.0], device=approach.device).expand_as(approach)
        fallback = torch.cross(approach, up, dim=-1)
        fallback = torch.where(fallback.norm(dim=-1, keepdim=True) < 1e-6,
                               torch.cross(approach, forward, dim=-1), fallback)
        axis = torch.where(degenerate.unsqueeze(-1), fallback, axis)
    return unit(axis)


def orientation_error(approach_cur: torch.Tensor, jaw_cur: torch.Tensor,
                      jaw_des: torch.Tensor, approach_des: torch.Tensor | None = None,
                      approach_weight: float = APPROACH_WEIGHT) -> torch.Tensor:
    """Rotation vector taking the current hand frame to the desired one, small-angle.

    The jaw axis is an *axis*, not a direction -- the two fingers are interchangeable -- so the
    desired one is taken in whichever sense is nearer the current one. Without that the servo
    spends a whole phase rolling the wrist 180 degrees to reach a pose it was already in.

    `approach_des` is omitted when only the roll is being servoed: the roll cannot change the
    approach, and an error it cannot answer only pulls the roll off the jaw axis it can fix.
    """
    sign = torch.sign((jaw_cur * jaw_des).sum(-1, keepdim=True))
    sign = torch.where(sign == 0, torch.ones_like(sign), sign)
    error = 0.5 * torch.cross(jaw_cur, sign * jaw_des, dim=-1)
    if approach_des is not None:
        error = error + 0.5 * approach_weight * torch.cross(approach_cur, approach_des, dim=-1)
    return error


def joint_correction(error: torch.Tensor, axes: torch.Tensor, gain: float = DEFAULT_GAIN,
                     damping: float = DEFAULT_DAMPING, max_step: float = MAX_STEP_RAD
                     ) -> torch.Tensor:
    """Least-squares joint angles producing as much of `gain * error` as the axes can.

    `axes` is (N, 3, k): each servoed joint's rotation axis in the world, so the rotation a set of
    angle steps produces is `axes @ steps`. The damped normal equations of that k-column system
    are solved directly -- cheaper and better conditioned here than a batched `lstsq`, and the
    damping is what keeps the answer finite when two axes coincide.
    """
    target = gain * error
    ata = axes.transpose(-1, -2) @ axes
    eye = torch.eye(axes.shape[-1], device=axes.device, dtype=axes.dtype).expand_as(ata)
    atb = axes.transpose(-1, -2) @ target.unsqueeze(-1)
    steps = torch.linalg.solve(ata + damping * eye, atb).squeeze(-1)
    return steps.clamp(-max_step, max_step)


def joint_actions(current_q: torch.Tensor, steps: torch.Tensor, limits: torch.Tensor,
                  action_scale: float) -> torch.Tensor:
    """Turn an angle step into the actions UniFP's control law reads as that position target.

    UniFP's law is `target = action * ACTION_SCALE + default`, and every arm joint's default is
    zero, so the action for an angle is that angle over the scale. The target is built from the
    *measured* angle rather than integrated in the servo, which keeps it a feedback loop: an arm
    held off its target by the object does not have the next command run away from where it is.
    """
    target = (current_q + steps).clamp(limits[..., 0], limits[..., 1])
    return target / action_scale
