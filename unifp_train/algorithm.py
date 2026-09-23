"""PPO plus UniFP's estimator loss, on rsl-rl 5.x.

UniFP trains two things at once. PPO trains the actor and the critic in the usual way. A second,
much slower optimizer trains the encoder and decoder of `models.UniFPActor` to reproduce the
privileged quantities the actor is not given -- base linear velocity, the tool's actual position,
and the two external forces. The encoder is the only part both touch, and that is the point: the
latent the policy acts on is pushed towards being a state estimate.

Upstream interleaves them per mini-batch -- PPO gradient step, then one estimator step on the same
batch, twenty times per iteration (5 epochs x 4 mini-batches). That interleaving is reproduced
exactly, and without reimplementing PPO: the mini-batch generator is wrapped so that it runs the
estimator step when the consumer comes back for the next batch, which is precisely after the PPO
step for the batch just yielded.

Three things are deliberately not upstream's:

  * The estimator optimizer is given the encoder and decoder rather than every parameter in the
    model. See `models.UniFPActor.estimator_parameters` for why those are the same thing.
  * The reported losses are means over mini-batches. Upstream divides its by an extra factor of
    `adaptation_batch_size` (64) that has nothing to do with how many terms were summed, so its
    logged numbers are 64x smaller than the losses it optimises. They are not comparable to these.
  * RND, symmetry augmentation and recurrent models are refused rather than half-supported. None
    are part of UniFP and all three would interact with the wrapped generator.
"""
from __future__ import annotations

import torch
import torch.nn.functional as F
import torch.optim as optim
from collections.abc import Generator
from itertools import chain
from tensordict import TensorDict

from rsl_rl.algorithms import PPO
from rsl_rl.env import VecEnv
from rsl_rl.models import MLPModel
from rsl_rl.storage import RolloutStorage
from rsl_rl.extensions import resolve_rnd_config, resolve_symmetry_config
from rsl_rl.utils import resolve_callable, resolve_obs_groups

#: Upstream's `AC_Args.adaptation_labels` / `adaptation_dims` / `adaptation_weights`. The blocks
#: are consecutive slices of the estimate vector, in this order, and the weights say how much each
#: matters: the two forces ten times more than velocity or tool position, which is the whole
#: argument for the architecture -- the forces are what the actor most needs and least can see.
ESTIMATE_BLOCKS = (
    ("base_velocity", 3, 0.2),
    ("gripper_pos", 3, 0.2),
    ("force_ee", 3, 1.0),
    ("force_base", 3, 1.0),
)

#: Upstream's `Adaptation_Args`. Two orders of magnitude below PPO's learning rate: the estimator
#: is meant to follow the policy, not steer it.
ESTIMATOR_LEARNING_RATE = 1.0e-5
ESTIMATOR_SUBSTEPS = 1


class ScaledAdam(torch.optim.Adam):
    """Adam whose effective step is a fixed fraction of the rate it is handed.

    rsl-rl's adaptive schedule clamps the learning rate to `[1e-5, 1e-2]` and those bounds are
    hard-coded inside `PPO.update()`, so when the floor binds the schedule has nowhere left to go
    and stops regulating. This class moves the whole usable range down without touching the
    schedule's logic.

    **Why it did not help, and why it is kept.** It was written on 2026-09-21 to answer a KL
    sitting at 0.017 against a target of 0.01 *at* the floor. F-091 later found the reason: most
    of that KL was not produced by learning at all but by environments whose stored observation
    had been blanked after the policy acted on it, so the controller was saturated by a corrupted
    input and scaling the optimizer could not move it. With that fixed the rate settles near
    2e-04, nowhere near the floor, and nothing here is needed. It stays because the clamp is still
    hard-coded and a future configuration may genuinely want a rate below 1e-5 -- but reach for it
    only after `Loss/kl_first_minibatch` has been confirmed to be ~0, not before.

    Scaling what the optimizer does with the rate moves the whole usable range down without
    touching the schedule's logic: at `scale = 0.25` the clamp `[1e-5, 1e-2]` becomes an effective
    `[2.5e-6, 2.5e-3]`, and the schedule regulates inside it exactly as before. `scale = 1.0` is
    the untouched behaviour.
    """

    def __init__(self, params, scale: float = 1.0, **kwargs) -> None:
        super().__init__(params, **kwargs)
        self.scale = scale

    def step(self, closure=None):  # type: ignore[override]
        if self.scale == 1.0:
            return super().step(closure)
        saved = [group["lr"] for group in self.param_groups]
        for group in self.param_groups:
            group["lr"] = group["lr"] * self.scale
        try:
            return super().step(closure)
        finally:
            for group, rate in zip(self.param_groups, saved):
                group["lr"] = rate


class AdaptationRolloutStorage(RolloutStorage):
    """Rollout storage that calls back once the consumer has finished with each mini-batch.

    The hook exists so the estimator step lands where upstream puts it -- after the PPO gradient
    step for that batch -- without copying rsl-rl's `PPO.update()`. A generator resumes after its
    `yield` only when the consumer asks for the next item, which is exactly that moment.
    """

    #: Set by `UniFPPPO`. `None` makes this behave as the base class.
    after_batch = None

    def mini_batch_generator(self, num_mini_batches: int,
                             num_epochs: int = 8) -> Generator[RolloutStorage.Batch, None, None]:
        for batch in super().mini_batch_generator(num_mini_batches, num_epochs):
            yield batch
            if self.after_batch is not None:
                self.after_batch(batch)


class UniFPPPO(PPO):
    """PPO with the adaptation module's supervised loss alongside it."""

    #: Set before `super().__init__()` so the `learning_rate` property below is already armed
    #: when the base class assigns to it.
    _fixed_learning_rate: float | None = None

    @property
    def learning_rate(self) -> float:
        return self._learning_rate

    @learning_rate.setter
    def learning_rate(self, value: float) -> None:
        """Hold the rate at `fixed_learning_rate`, if one was asked for.

        rsl-rl's `schedule="fixed"` would do this, but it skips the whole adaptive block --
        including the KL computation -- so a fixed-rate run would report no KL at all, and the KL
        is the quantity being measured. Refusing the schedule's writes instead leaves the KL
        computed and logged exactly as in a faithful run, while the rate it is trying to control
        stays put. Every mini-batch then re-asserts the fixed value on the optimizer.
        """
        self._learning_rate = value if self._fixed_learning_rate is None else self._fixed_learning_rate

    def __init__(self, actor: MLPModel, critic: MLPModel, storage: RolloutStorage,
                 estimator_learning_rate: float = ESTIMATOR_LEARNING_RATE,
                 estimator_substeps: int = ESTIMATOR_SUBSTEPS,
                 estimate_group: str = "estimates",
                 learning_rate_scale: float = 1.0,
                 fixed_learning_rate: float | None = None,
                 **kwargs) -> None:
        self._fixed_learning_rate = fixed_learning_rate
        super().__init__(actor, critic, storage, **kwargs)
        if fixed_learning_rate is not None:
            for group in self.optimizer.param_groups:
                group["lr"] = fixed_learning_rate

        self.learning_rate_scale = learning_rate_scale
        if learning_rate_scale != 1.0:
            # Replace the optimizer the base class just built, keeping its parameter set and
            # starting rate. See ScaledAdam for why the hard-coded rate floor is a problem here.
            self.optimizer = ScaledAdam(chain(actor.parameters(), critic.parameters()),
                                        lr=self.learning_rate, scale=learning_rate_scale)

        if self.rnd is not None or self.symmetry is not None:
            raise ValueError("UniFPPPO does not support RND or symmetry; both are absent from "
                             "UniFP and both would interact with the wrapped mini-batch generator.")
        if actor.is_recurrent or critic.is_recurrent:
            raise ValueError("UniFPPPO expects feedforward models: the adaptation module reads a "
                             "frame stack, not a hidden state.")
        if not hasattr(actor, "estimate"):
            raise TypeError(f"UniFPPPO needs an actor with an adaptation module; got "
                            f"{type(actor).__name__}, which has no `estimate()`.")

        self.estimate_group = estimate_group
        self.estimator_substeps = estimator_substeps
        self.estimator_optimizer = optim.Adam(actor.estimator_parameters(),
                                              lr=estimator_learning_rate)

        self._blocks, start = [], 0
        for label, width, weight in ESTIMATE_BLOCKS:
            self._blocks.append((label, start, start + width, weight))
            start += width
        self._estimate_dim = start

        self._losses: dict[str, float] = {}
        self._batches = 0
        #: Mean KL divergence over the last update's mini-batches, recorded in `update()`.
        self.kl: float | None = None

        if isinstance(storage, AdaptationRolloutStorage):
            storage.after_batch = self._estimator_step
        else:
            raise TypeError("UniFPPPO needs an AdaptationRolloutStorage to place the estimator "
                            "step after each PPO step; see construct_algorithm().")

    # --- the extra loss ---------------------------------------------------------------------

    def _estimator_step(self, batch: RolloutStorage.Batch) -> None:
        """One supervised step on the encoder and decoder, using the batch PPO just used."""
        obs: TensorDict = batch.observations  # type: ignore
        target = obs[self.estimate_group].detach()
        if target.shape[-1] != self._estimate_dim:
            raise ValueError(f"Observation group '{self.estimate_group}' is {target.shape[-1]} "
                             f"wide; the estimate blocks sum to {self._estimate_dim}.")

        for _ in range(self.estimator_substeps):
            prediction = self.actor.estimate(obs)  # type: ignore[attr-defined]
            total = None
            for label, start, stop, weight in self._blocks:
                # The weight is applied to both sides before the MSE rather than to the loss, so
                # it scales the error *quadratically*. That is upstream's arrangement and it is
                # what the relative weights mean: 1.0 against 0.2 is a factor of 25, not 5.
                loss = F.mse_loss(prediction[:, start:stop] * weight, target[:, start:stop] * weight)
                total = loss if total is None else total + loss
                self._losses[label] = self._losses.get(label, 0.0) + loss.item()

            self.estimator_optimizer.zero_grad()
            total.backward()  # type: ignore[union-attr]
            self.estimator_optimizer.step()
            self._losses["estimator"] = self._losses.get("estimator", 0.0) + total.item()  # type: ignore[union-attr]
        self._batches += 1

    def diagnose_storage(self) -> None:
        """Check that the policy the update sees is the policy that produced the rollout.

        Recomputes the action mean from each stored observation and compares it with the mean
        stored alongside it. Offset 0 is the pairing the update uses; the others are there to say
        what a mismatch *is*, because "off by one step" and "wrong environment" look nothing alike
        in this table and neither looks like numerical noise.
        """
        st = self.storage
        steps = st.num_transitions_per_env
        with torch.inference_mode():
            recomputed = []
            for t in range(steps):
                self.actor(st.observations[t], masks=None, hidden_state=None,
                           stochastic_output=True)
                recomputed.append(self.actor.output_distribution_params[0].clone())
            stored = [st.distribution_params[0][t] for t in range(steps)]
            sigma = st.distribution_params[1][0]
            print(f"[diagnose] {steps} steps x {st.num_envs} envs")
            print(f"[diagnose] {'offset':>7}{'mean |dmu|':>14}{'max |dmu|':>13}{'implied KL':>13}")
            for off in (0, 1, -1):
                diffs, kls = [], []
                for t in range(steps):
                    u = t + off
                    if not 0 <= u < steps:
                        continue
                    d = recomputed[t] - stored[u]
                    diffs.append(d.abs().mean().item())
                    kls.append(((d ** 2) / (2 * sigma ** 2)).sum(-1).mean().item())
                print(f"[diagnose] {off:>7}{sum(diffs)/len(diffs):>14.6g}"
                      f"{max(diffs):>13.6g}{sum(kls)/len(kls):>13.6g}")
            # Where does the disagreement live -- every environment a little, or a few a lot?
            bad = torch.zeros(0, device=st.dones.device, dtype=torch.bool)
            done = torch.zeros(0, device=st.dones.device, dtype=torch.bool)
            zeroed = torch.zeros(0, device=st.dones.device, dtype=torch.bool)
            for t in range(steps):
                b = (recomputed[t] - stored[t]).abs().mean(-1) > 1e-5
                bad = torch.cat((bad, b))
                done = torch.cat((done, st.dones[t].squeeze(-1).bool()))
                zeroed = torch.cat((zeroed, st.observations[t]["policy"].abs().sum(-1) == 0))
            n = bad.numel()
            print(f"[diagnose] samples disagreeing at offset 0 : {bad.sum().item()}/{n} "
                  f"({bad.float().mean().item()*100:.2f}%)")
            print(f"[diagnose] samples whose episode ended here: {done.sum().item()}/{n} "
                  f"({done.float().mean().item()*100:.2f}%)")
            print(f"[diagnose] stored actor observation all zero: {zeroed.sum().item()}/{n}")
            print(f"[diagnose] disagree AND done: {(bad & done).sum().item()}   "
                  f"disagree NOT done: {(bad & ~done).sum().item()}   "
                  f"done NOT disagree: {(done & ~bad).sum().item()}")

    def update(self) -> dict[str, float]:
        """`PPO.update()`, with the KL divergence recorded on the way past.

        rsl-rl computes the KL to drive its adaptive learning rate and then throws it away, which
        leaves the rate as the only visible trace of it. That is not enough to debug with: a rate
        pinned at the 1e-5 floor is consistent with a KL of 0.006, sitting harmlessly inside the
        schedule's dead band, and with a KL of 0.5, which would mean the policy is being thrown
        across the space every iteration. Those call for opposite responses and the rate alone
        cannot tell them apart.

        Rather than reimplement the update loop to get at it, the actor's KL function is wrapped
        for the duration of the call. The schedule still sees exactly what it saw before; the
        values are copied out on the way through.
        """
        if getattr(self, "diagnose_storage_requested", False):
            self.diagnose_storage()

        self._losses, self._batches = {}, 0
        original = self.actor.get_kl_divergence
        samples: list[float] = []

        def recording(old_params, new_params):
            kl = original(old_params, new_params)
            samples.append(float(kl.mean()))
            return kl

        self.actor.get_kl_divergence = recording  # type: ignore[method-assign]
        try:
            loss_dict = super().update()
        finally:
            self.actor.get_kl_divergence = original  # type: ignore[method-assign]

        if self._batches:
            for label, value in self._losses.items():
                loss_dict[label] = value / (self._batches * self.estimator_substeps)
        if samples:
            # The two fractions are what the schedule actually acts on: it raises the rate only
            # below `desired_kl / 2` and lowers it only above `desired_kl * 2`. A run where both
            # are zero is one whose learning rate is frozen because every mini-batch landed in
            # the dead band -- which looks identical, from the rate alone, to one that is stuck.
            self.kl = sum(samples) / len(samples)
            loss_dict["kl"] = self.kl
            # The first mini-batch of the first epoch is evaluated before any gradient step has
            # been taken since the rollout, so the policy it measures is bit-for-bit the policy
            # that produced the data. Its KL is therefore the implementation's numerical floor,
            # and nothing else: a large value here cannot be caused by learning, and means the
            # update is not seeing the policy that acted. Logged on every run as a standing check.
            loss_dict["kl_first_minibatch"] = samples[0]
            loss_dict["effective_lr"] = self.learning_rate * self.learning_rate_scale
            loss_dict["kl_above_upper"] = sum(
                1 for k in samples if k > self.desired_kl * 2.0) / len(samples)
            loss_dict["kl_below_lower"] = sum(
                1 for k in samples if k < self.desired_kl / 2.0) / len(samples)
        return loss_dict

    # --- saving -------------------------------------------------------------------------------

    def save(self) -> dict:
        saved = super().save()
        saved["estimator_optimizer_state_dict"] = self.estimator_optimizer.state_dict()
        return saved

    def load(self, loaded_dict: dict, load_cfg: dict | None, strict: bool) -> bool:
        resume_iteration = super().load(loaded_dict, load_cfg, strict)
        wanted = True if load_cfg is None else load_cfg.get("optimizer", False)
        if wanted and "estimator_optimizer_state_dict" in loaded_dict:
            self.estimator_optimizer.load_state_dict(loaded_dict["estimator_optimizer_state_dict"])
        return resume_iteration

    # --- construction ---------------------------------------------------------------------------

    @staticmethod
    def construct_algorithm(obs: TensorDict, env: VecEnv, cfg: dict, device: str) -> "UniFPPPO":
        """`PPO.construct_algorithm` with `AdaptationRolloutStorage` in place of the storage.

        This is a copy of the base implementation rather than a call to it, because the storage is
        built inside it and reallocating a second one would briefly double the largest allocation
        in the run (24 steps x 4096 envs x 2903 floats is about 1.1 GB). Everything else is the
        library's own resolution helpers, so a version bump shows up as a loud failure here rather
        than as drift in the learning maths -- which stays untouched in `PPO.update()`.
        """
        alg_class = resolve_callable(cfg["algorithm"].pop("class_name"))
        actor_class = resolve_callable(cfg["actor"].pop("class_name"))
        critic_class = resolve_callable(cfg["critic"].pop("class_name"))

        cfg["obs_groups"] = resolve_obs_groups(obs, cfg["obs_groups"], ["actor", "critic"])
        cfg["algorithm"] = resolve_rnd_config(cfg["algorithm"], obs, cfg["obs_groups"], env)
        cfg["algorithm"] = resolve_symmetry_config(cfg["algorithm"], env)

        actor = actor_class(obs, cfg["obs_groups"], "actor", env.num_actions, **cfg["actor"]).to(device)
        print(f"Actor Model: {actor}")
        critic = critic_class(obs, cfg["obs_groups"], "critic", 1, **cfg["critic"]).to(device)
        print(f"Critic Model: {critic}")

        storage = AdaptationRolloutStorage(
            "rl", env.num_envs, cfg["num_steps_per_env"], obs, [env.num_actions], device)

        return alg_class(actor, critic, storage, device=device, **cfg["algorithm"],
                         multi_gpu_cfg=cfg["multi_gpu"])
