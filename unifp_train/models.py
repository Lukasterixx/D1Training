"""UniFP's actor: an MLP behind a learned encoder that reads 32 frames of history.

The policy is not a plain MLP over the stacked observation. It is:

    encoder:  2432 (32 frames x 76)        -> [512, 256, 128] -> 64 latent
    actor:    76 (newest frame) + 64 latent -> [512, 256, 128] -> 18 actions
    decoder:  64 latent                     -> [128, 64]       -> 12 estimates

so the history reaches the action only through a 64-wide bottleneck, and that bottleneck is
*supervised*: the decoder is trained to reproduce the four privileged quantities the actor is not
given -- base linear velocity, where the tool actually is, and the two external forces. That is
the "adaptation module", and it is why this task can be trained with forces the actor never sees.
The decoder's loss lives in `algorithm.py`; what is here is the architecture.

Shapes are the trained contract, not a choice. The latent is 64 because upstream computes it as
`(num_obs / num_single_obs) * 2` -- two numbers per stacked frame -- and a checkpoint from
`model_48800` has to load into this and vice versa.

This subclasses rsl-rl's `MLPModel` rather than replacing it, which buys the distribution, the
normalizer, the KL machinery and the storage integration for free. Two hooks do the work:
`_get_latent_dim()` tells the base class how wide the actor MLP's input is, and `get_latent()`
builds it. `self.mlp` is therefore upstream's `actor_body`.

**Observation normalization is off** in the config that goes with this, because UniFP does not
normalize: it feeds raw scaled observations clipped at +/-100. Turning it on would train a
different policy -- a defensible one, but not this one.
"""
from __future__ import annotations

import torch
import torch.nn as nn
from tensordict import TensorDict

from rsl_rl.models import MLPModel
from rsl_rl.modules import MLP, HiddenState

from unifp_isaaclab import interface

from . import observations

#: Upstream's `AC_Args.adaptation_module_encoder_hidden_dims` / `..._decoder_hidden_dims`.
ENCODER_HIDDEN_DIMS = (512, 256, 128)
DECODER_HIDDEN_DIMS = (128, 64)


class UniFPActor(MLPModel):
    """Encoder + actor + supervised decoder, as one `MLPModel`."""

    def __init__(
        self,
        obs: TensorDict,
        obs_groups: dict[str, list[str]],
        obs_set: str,
        output_dim: int,
        hidden_dims: tuple[int, ...] | list[int] = (512, 256, 128),
        activation: str = "elu",
        obs_normalization: bool = False,
        distribution_cfg: dict | None = None,
        encoder_hidden_dims: tuple[int, ...] | list[int] = ENCODER_HIDDEN_DIMS,
        decoder_hidden_dims: tuple[int, ...] | list[int] = DECODER_HIDDEN_DIMS,
        num_single_obs: int = interface.NUM_SINGLE_OBS,
        num_latent: int = interface.NUM_LATENT,
        estimate_dim: int = observations.NUM_ESTIMATES,
    ) -> None:
        # Set before `super().__init__()`: it calls `_get_latent_dim()`, which needs both.
        self.num_single_obs = num_single_obs
        self.num_latent = num_latent
        super().__init__(obs, obs_groups, obs_set, output_dim, hidden_dims, activation,
                         obs_normalization, distribution_cfg)

        if self.obs_dim % num_single_obs != 0:
            raise ValueError(
                f"The actor observation is {self.obs_dim} wide, which is not a whole number of "
                f"{num_single_obs}-wide frames. The encoder reads a frame stack; check obs_groups.")

        self.encoder = MLP(self.obs_dim, num_latent, list(encoder_hidden_dims), activation)
        self.decoder = MLP(num_latent, estimate_dim, list(decoder_hidden_dims), activation)

    # --- the two hooks the base class calls ---------------------------------------------------

    def _get_latent_dim(self) -> int:
        """Width of `self.mlp`'s input: the newest frame, plus the latent."""
        return self.num_single_obs + self.num_latent

    def get_latent(self, obs: TensorDict, masks: torch.Tensor | None = None,
                   hidden_state: HiddenState = None) -> torch.Tensor:
        stacked = self._stacked(obs)
        latent = self.encoder(stacked)
        # The newest frame is *last* in the stack, which is `ObsHistory`'s convention and the
        # checkpoint's. Slicing the other end would feed the policy a 32-step-old observation and
        # would not fail anywhere -- it would just train badly.
        return torch.cat((stacked[:, -self.num_single_obs:], latent), dim=-1)

    # --- the adaptation module ------------------------------------------------------------------

    def estimate(self, obs: TensorDict) -> torch.Tensor:
        """The decoder's prediction of the privileged quantities, (N, `estimate_dim`)."""
        return self.decoder(self.encoder(self._stacked(obs)))

    def estimator_parameters(self):
        """Encoder and decoder only -- the parameters the estimator loss is allowed to move.

        Upstream hands its estimator optimizer *every* parameter, which reads as a mistake and is
        not one: the estimator loss only reaches the encoder and the decoder, so the rest never
        accumulate a gradient and Adam leaves them where they are. Naming them here says the same
        thing without depending on that argument holding.
        """
        return list(self.encoder.parameters()) + list(self.decoder.parameters())

    # --- plumbing ---------------------------------------------------------------------------------

    def _stacked(self, obs: TensorDict) -> torch.Tensor:
        latent = torch.cat([obs[group] for group in self.obs_groups], dim=-1)
        return self.obs_normalizer(latent)

    #: Why export is refused rather than inherited: the base class's exporters copy
    #: `obs_normalizer` and `mlp` and nothing else, so an exported `UniFPActor` would be the actor
    #: body fed 140 numbers it never sees -- a shape error at best and a silently wrong policy at
    #: worst. Exporting this network means exporting the encoder with it, which is work nothing
    #: here needs yet.
    _NO_EXPORT = ("UniFPActor puts an encoder in front of the MLP and the base class's exporter "
                  "does not copy it. Write an exporter that includes the encoder before using one.")

    def as_jit(self) -> nn.Module:
        raise NotImplementedError(self._NO_EXPORT)

    def as_onnx(self, verbose: bool) -> nn.Module:
        raise NotImplementedError(self._NO_EXPORT)
