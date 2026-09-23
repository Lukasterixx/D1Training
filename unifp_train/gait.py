"""UniFP's gait clock, and the leg reference pose its rewards are scored against.

Three quantities come off one scalar phase per robot, and all three are needed by the training
port: the **stance mask** (which feet the gait says should be down), the **reference joint
positions** (the trot posture the legs are rewarded for tracking) and the sin/cos pair that enters
the observation. `unifp_isaaclab.interface` owns the phase itself and the observation encoding;
this module owns what the *task* does with it, which the playback port never needed.

Ported from `_get_gait_phase` and `compute_ref_state` in UniFP's Go2+D1 environment, and checked
against a recorded rollout of that environment in `tests/test_unifp_train.py`.

The diagonal pairing is upstream's and is worth stating because it is easy to transcribe wrong:
**FL and RR** swing together against **FR and RL**, and the two halves use *different* comparisons
(`>= 0` against `< 0`) around a threshold offset of ±`TARGET_JOINT_POS_THD`, which is what creates
the double-support overlap rather than a clean 50/50 split.
"""
from __future__ import annotations

import math

import torch

from unifp_isaaclab import interface

#: Amplitude of the reference trot, radians at the thigh (the calf gets twice this).
TARGET_JOINT_POS_SCALE = 0.17

#: Phase offset that creates the double-support overlap. Both halves of the gait are shifted by
#: this before their sign is taken, so for |sin| < this value *both* diagonals read as stance.
TARGET_JOINT_POS_THD = 0.5

#: Indices into the 12 leg joints, in UniFP's DOF order (FL, FR, RL, RR) x (hip, thigh, calf).
FL_THIGH, FL_CALF = 1, 2
FR_THIGH, FR_CALF = 4, 5
RL_THIGH, RL_CALF = 7, 8
RR_THIGH, RR_CALF = 10, 11

#: Foot order for the stance and contact masks: FL, FR, RL, RR.
FEET = ("FL_foot", "FR_foot", "RL_foot", "RR_foot")


def stance_mask(phase: torch.Tensor) -> torch.Tensor:
    """Which feet the gait wants on the ground, (N, 4) of 1.0 (stance) and 0.0 (swing).

    Column order is FL, FR, RL, RR. FL and RR share a phase; FR and RL share the other.
    """
    sin_pos = torch.sin(2 * math.pi * phase)
    left = sin_pos + TARGET_JOINT_POS_THD
    right = sin_pos - TARGET_JOINT_POS_THD
    mask = torch.zeros(phase.shape[0], 4, device=phase.device, dtype=torch.float)
    mask[:, 0] = (left >= 0).float()
    mask[:, 3] = (left >= 0).float()
    mask[:, 1] = (right < 0).float()
    mask[:, 2] = (right < 0).float()
    return mask


def reference_leg_pos(phase: torch.Tensor) -> torch.Tensor:
    """The trot posture the leg-tracking reward scores against, (N, 12) in UniFP's DOF order.

    Only the thigh and calf joints move; the hips hold their default. Upstream clamps the *stance*
    half of each diagonal to zero deflection — the `* 0.0` in its `compute_ref_state` — so the
    reference only ever pushes a leg through its swing, never against its stance. That multiply
    looks like dead code and is not: removing it makes the reference a full sinusoid and changes
    the gait the legs are rewarded for.
    """
    default = torch.as_tensor(interface.DEFAULT_DOF_POS[:12], dtype=phase.dtype, device=phase.device)
    reference = default.unsqueeze(0).repeat(phase.shape[0], 1)

    sin_pos = torch.sin(2 * math.pi * phase)
    left = sin_pos + TARGET_JOINT_POS_THD
    right = sin_pos - TARGET_JOINT_POS_THD
    left = torch.where(left > 0, torch.zeros_like(left), left)
    right = torch.where(right < 0, torch.zeros_like(right), right)

    scale_1 = TARGET_JOINT_POS_SCALE / (1 - TARGET_JOINT_POS_THD)
    scale_2 = scale_1 * 2

    reference[:, FL_THIGH] -= left * scale_1
    reference[:, FL_CALF] += left * scale_2
    reference[:, RR_THIGH] -= left * scale_1
    reference[:, RR_CALF] += left * scale_2

    reference[:, FR_THIGH] += right * scale_1
    reference[:, FR_CALF] -= right * scale_2
    reference[:, RL_THIGH] += right * scale_1
    reference[:, RL_CALF] -= right * scale_2
    return reference
