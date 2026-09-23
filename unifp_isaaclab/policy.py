"""Load a UniFP checkpoint as plain PyTorch, with no Isaac Gym and no `b2_gym_learn` on the path.

UniFP's checkpoints are saved by its vendored rsl_rl fork, but what they contain is an ordinary
`state_dict` of `nn.Linear` weights -- so the inference half of the network can be rebuilt from
the shapes alone. That matters here: the training stack is Python 3.8 with Isaac Gym Preview 4,
and Isaac Lab is Python 3.11. Nothing in this module imports either.

Three modules make up inference (`ActorCritic.act_student` in the training stack):

    latent  = adaptation_encoder(obs_2432)        # 32 frames of history -> 64 latent
    action  = actor_body(cat(obs_2432[-76:], latent))
    obs_pred = adaptation_decoder(latent)         # 12: base velocity, ee position, ee force, base force

`obs_pred` is the *estimate* the policy forms of quantities it cannot observe. It does not feed
the action -- it is a supervised head -- but it is the only window onto what the policy thinks the
end-effector force is, so `estimates()` unpacks and unscales it.

The critic is not rebuilt: it reads 459 privileged observations that do not exist outside training.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import torch
from torch import nn

from . import interface

#: Layout of the adaptation decoder's 12 outputs, and the observation scale each was trained
#: against. Dividing by the scale puts them back in SI units.
ESTIMATE_LAYOUT = (
    ("base_lin_vel", slice(0, 3), 2.0),     # m/s, body frame
    ("ee_pos_sphere", slice(3, 6), None),   # (radius, pitch, yaw); per-axis scales, see below
    ("ee_force", slice(6, 9), 0.01),        # N, yaw-frame
    ("base_force", slice(9, 12), 0.01),     # N, yaw-frame
)
EE_SPHERE_SCALES = (0.5, 1.0, 1.3)


def _mlp(state: dict, prefix: str) -> nn.Sequential:
    """Rebuild an ELU MLP from the `Linear` weights saved under `prefix`.

    UniFP builds every one of these as `Linear, ELU, Linear, ELU, ..., Linear`, so the module
    indices in the state dict are even and the activations sit between them. Reading the shapes
    rather than hardcoding them means a checkpoint from a differently-sized run still loads, and
    a checkpoint that does not match this pattern raises here instead of producing a network that
    is quietly the wrong shape.
    """
    indices = sorted({int(key[len(prefix) + 1:].split(".")[0]) for key in state if key.startswith(prefix + ".")})
    if indices != list(range(0, 2 * len(indices), 2)):
        raise ValueError(f"{prefix}: expected Linear layers at even indices, found {indices}")
    layers: list[nn.Module] = []
    for position, index in enumerate(indices):
        weight = state[f"{prefix}.{index}.weight"]
        layers.append(nn.Linear(weight.shape[1], weight.shape[0]))
        if position < len(indices) - 1:
            layers.append(nn.ELU())
    module = nn.Sequential(*layers)
    module.load_state_dict({key[len(prefix) + 1:]: value for key, value in state.items()
                            if key.startswith(prefix + ".")})
    return module


@dataclass
class Estimates:
    """What the adaptation decoder believes, in SI units."""

    base_lin_vel: torch.Tensor      # (N, 3) m/s, body frame
    ee_pos_sphere: torch.Tensor     # (N, 3) radius m, pitch rad, yaw rad
    ee_force: torch.Tensor          # (N, 3) N, yaw-aligned world frame
    base_force: torch.Tensor        # (N, 3) N, yaw-aligned world frame


class UniFPPolicy:
    """`model_<iteration>.pt`, loaded for inference.

    Call it with the stacked observation (N, 2432) -- oldest frame first, newest last, which is
    the order `ObsHistory` produces -- and it returns (N, 18) actions. Deterministic: this is the
    distribution mean, as UniFP's own play path uses, not a sample.
    """

    def __init__(self, checkpoint: str | Path, device: str = "cpu"):
        self.path = Path(checkpoint)
        payload = torch.load(self.path, map_location="cpu", weights_only=False)
        state = payload["model_state_dict"]
        self.encoder = _mlp(state, "adaptation_encoder_module").to(device).eval()
        self.decoder = _mlp(state, "adaptation_decoder_module").to(device).eval()
        self.actor = _mlp(state, "actor_body").to(device).eval()
        self.device = device

        self.num_obs = self.encoder[0].in_features
        self.num_latent = self.encoder[-1].out_features
        self.num_actions = self.actor[-1].out_features
        self.num_single_obs = self.actor[0].in_features - self.num_latent

        if self.num_single_obs != interface.NUM_SINGLE_OBS or self.num_obs != interface.NUM_OBS:
            raise ValueError(
                f"{self.path.name} expects {self.num_obs} observations of {self.num_single_obs}, "
                f"but the interface describes {interface.NUM_OBS} of {interface.NUM_SINGLE_OBS}")
        if self.num_actions != interface.NUM_ACTIONS:
            raise ValueError(f"{self.path.name} has {self.num_actions} actions, "
                             f"the interface describes {interface.NUM_ACTIONS}")

        self._latent: torch.Tensor | None = None

    @property
    def iteration(self) -> int | None:
        """The training iteration this checkpoint was written at, from its filename.

        The `iter` field inside the payload is 0 in these runs (upstream writes it before the
        counter is advanced), so the filename is the only honest source.
        """
        stem = self.path.stem
        return int(stem.split("_")[-1]) if stem.startswith("model_") and stem.split("_")[-1].isdigit() else None

    @torch.no_grad()
    def act(self, obs: torch.Tensor) -> torch.Tensor:
        """(N, 2432) stacked observation -> (N, 18) actions."""
        obs = obs.to(self.device)
        self._latent = self.encoder(obs)
        return self.actor(torch.cat((obs[:, -self.num_single_obs:], self._latent), dim=-1))

    @torch.no_grad()
    def estimates(self) -> Estimates:
        """Unscale the adaptation decoder's prediction from the most recent `act()`."""
        if self._latent is None:
            raise RuntimeError("call act() before estimates()")
        pred = self.decoder(self._latent)
        sphere_scale = torch.as_tensor(EE_SPHERE_SCALES, dtype=pred.dtype, device=pred.device)
        return Estimates(
            base_lin_vel=pred[:, 0:3] / 2.0,
            ee_pos_sphere=pred[:, 3:6] / sphere_scale,
            ee_force=pred[:, 6:9] / 0.01,
            base_force=pred[:, 9:12] / 0.01,
        )


class ObsHistory:
    """The 32-frame ring the adaptation encoder reads.

    UniFP keeps a `deque(maxlen=32)` of single observations and flattens it oldest-first. On reset
    every frame is zeroed rather than filled with the current observation, so the first 32 policy
    steps of an episode are fed a history that is mostly zeros -- that is what the policy was
    trained against and it is reproduced here deliberately. A history pre-filled with the reset
    pose would be a different input distribution than training, and the difference shows up as a
    lurch in the first half second.

    Verified frame-for-frame against a recorded Isaac Gym rollout from step 1 onward (exactly
    0.0 difference; `tests/test_unifp_interface.py`). Step 0 differs, and the difference is
    upstream's: a freshly constructed UniFP environment returns an all-zero `obs_buf` from
    `get_observations()` even though its history already holds one real frame, so UniFP's own
    play loop feeds the policy 2432 zeros on its first control step. That is an artefact of the
    play path rather than part of the trained contract -- during training the environment is
    stepped continuously and never presents it -- so it is not reproduced here.
    """

    def __init__(self, num_envs: int, device: str = "cpu",
                 frame_stack: int = interface.FRAME_STACK,
                 num_single_obs: int = interface.NUM_SINGLE_OBS):
        self.frame_stack = frame_stack
        self.num_single_obs = num_single_obs
        self.buffer = torch.zeros(num_envs, frame_stack, num_single_obs, device=device)

    def reset(self, env_ids: torch.Tensor | None = None) -> None:
        """Clear the history for these environments, without touching what `append` handed out.

        The rebinding is the whole point and is not a style choice. `append` returns a *view* of
        this buffer, and `DirectRLEnv.step()` resets terminated environments **after** the policy
        has acted on that view but **before** the rollout storage copies it. Zeroing in place
        therefore reaches back and blanks an observation that has already been used: the update
        then sees 2432 zeros where the rollout saw a real state, for exactly the samples whose
        episode ended. Measured on 2026-09-21 that was 0.05-0.11% of samples per iteration, and it
        drove the mean KL to 0.04-1.0 -- against a target of 0.01 -- with the optimizer frozen.
        Upstream is immune because it rebuilds its stack with `torch.stack` every step.
        """
        if env_ids is None:
            self.buffer = torch.zeros_like(self.buffer)
        else:
            self.buffer = self.buffer.clone()
            self.buffer[env_ids] = 0.0

    def append(self, obs: torch.Tensor) -> torch.Tensor:
        """Push one (N, 76) frame and return the (N, 2432) stack, newest frame last."""
        self.buffer = torch.roll(self.buffer, shifts=-1, dims=1)
        self.buffer[:, -1] = obs
        return self.buffer.reshape(self.buffer.shape[0], -1)
