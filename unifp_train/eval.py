"""Run a policy against a frozen manifest in Isaac Lab, and summarise what it did.

The point of this module is a comparison the sim-to-sim work could not make. F-088 measured the
Isaac Gym policy in Isaac Lab and found it stands but cannot walk; F-089 rebuilt the task so a
policy could be trained here instead. The question left over is whether training natively is
actually better than porting the weights — and that question is only answerable if **both**
policies are measured **in the same simulator, on the same frozen episodes**.

So this evaluates, in Isaac Lab:

  * `model_48800`, trained on Isaac Gym and ported (its loading is verified exactly, F-087);
  * a checkpoint trained here;
  * zero actions, because without the baseline a tracking number says nothing — the resting tool
    tip is already somewhere and some goals are near it.

**Why this needs its own manifest.** `results/manifests/unifp_validation.json` freezes its episode
set by one seed and one environment count, and verifies each episode by a digest of the schedule
the environment realised. That works because every draw comes from the global torch RNG in a
fixed order — which is a property of the *Isaac Gym* environment's code path. This environment
draws the same quantities from different code in a different order, so the same seed gives
different episodes and those digests can never match. A separate manifest is honest about that;
reusing the file and ignoring the mismatch would not be.

Metric definitions are deliberately identical to `unifp_go2d1/evaluate.py` so the two records read
alike, and what each number is and is not is set out there at length. In brief: goal tracking is
measured only on steps where no force is commanded or applied, unified tracking is the thing the
task actually rewards, force realisation is a displacement through UniFP's admittance formulation
rather than anything a sensor measured, and estimator error is the adaptation module's own
prediction scored rather than trained.
"""
from __future__ import annotations

import hashlib
import statistics as st

import torch

from unifp_go2d1 import eval_manifest as _em
from unifp_isaaclab import interface

from . import observations, task_cfg

#: Distinct from `unifp_go2d1_eval_manifest_v1` on purpose: an Isaac Lab manifest cannot be run in
#: Isaac Gym or the reverse, and a format that says so fails early rather than at the digest check.
FORMAT = "unifp_isaaclab_eval_manifest_v1"

content_hash = _em.content_hash
save = _em.save
compare_conditions = _em.compare_conditions


def build(role: str, episode_count: int, seed: int, conditions: dict, episodes=None) -> dict:
    manifest = _em.build(role, episode_count, seed, conditions, episodes)
    manifest["format"] = FORMAT
    manifest["note"] = (
        "Frozen evaluation cases for the Isaac Lab build of UniFP's Go2+D1 position/force task. "
        "The set is one seeded batch of episode_count environments and each episode is verified "
        "by a digest of the schedule the environment realised, so a policy that perturbed the "
        "schedule, or an environment that has drifted, shows up as a mismatch rather than as a "
        "quietly different experiment. This is NOT interchangeable with the Isaac Gym manifest of "
        "the same task: the two simulators draw the same quantities in different orders, so a "
        "shared seed does not give shared episodes.")
    manifest["content_sha256"] = content_hash(manifest)
    return manifest


def load(path: str) -> dict:
    manifest = _em.json.load(open(path))
    if manifest.get("format") != FORMAT:
        raise ValueError(f"{path}: not a {FORMAT} manifest (got {manifest.get('format')!r}). "
                         f"An Isaac Gym manifest cannot be run here; its episodes do not exist "
                         f"in this environment.")
    recomputed = content_hash(manifest)
    if recomputed != manifest["content_sha256"]:
        raise ValueError(f"{path}: content_sha256 does not match its contents. The file has been "
                         f"edited by hand; rebuild it instead.")
    return manifest


def conditions_of(env) -> dict:
    """What this environment is, in the terms a manifest freezes."""
    cfg = env.cfg
    return {
        "task": "go2d1_pos_force",
        "simulator": "isaaclab",
        "episode_length_s": float(cfg.episode_length_s),
        "policy_hz": round(1.0 / interface.POLICY_DT, 3),
        "num_envs": int(env.num_envs),
        "noise": False,
        "domain_randomisation": False,
        "force_start_step": int(cfg.force_start_step),
        "ee_force_range_n": list(task_cfg.GRIPPER_FORCE_RANGE_N),
        "gripper_force_kp": float(env._gripper_force_kp[0, 0]),
        "goal_sphere_centre_m": list(interface.EE_GOAL_CENTER_OFFSET),
        "action_scale": interface.ACTION_SCALE,
        "tool_body": interface.TOOL_BODY,
        "tool_offset_m": list(interface.TOOL_OFFSET_M),
        "terrain": "flat plane",
        "solver_iterations": [cfg.robot.spawn.articulation_props.solver_position_iteration_count,
                              cfg.robot.spawn.articulation_props.solver_velocity_iteration_count],
        "arm_armature": cfg.robot.actuators["unifp"].armature,
    }


def _finite(values):
    return [v for v in values if v is not None and v == v]


def _stats(values):
    vals = _finite(values)
    if not vals:
        return None
    ordered = sorted(vals)
    return {
        "n": len(vals),
        "mean": st.mean(vals),
        "median": st.median(vals),
        "p90": ordered[min(int(0.9 * len(ordered)), len(ordered) - 1)],
        "max": ordered[-1],
    }


def _pool(records, key, field="median"):
    return _stats([r[key][field] for r in records if r.get(key)])


def run_episodes(env, manifest, policy=None, progress=None) -> list[dict]:
    """One record per manifest episode. `policy=None` runs the zero-action baseline.

    All episodes run at once, one per environment, for the full episode length, with termination
    **recorded but not acted upon** — the same protocol as the Isaac Gym evaluator, and for the
    same two reasons. Resetting a fallen environment mid-batch would redraw its commands and shift
    the random stream for every episode after it, and it would leave the schedule realised only to
    whatever prefix that policy survived. A fallen robot's tracking numbers stop meaning anything,
    so the metrics are cut at the step it fell; the schedule generator carries on.
    """
    episodes = manifest["episodes"]
    if env.num_envs != len(episodes):
        raise ValueError(f"manifest has {len(episodes)} episodes but the environment has "
                         f"{env.num_envs}; they must match, because the schedule is drawn in "
                         f"batches across environments")

    n = env.num_envs
    max_steps = int(env.max_episode_length)
    fell = torch.zeros(n, dtype=torch.bool, device=env.device)

    original_dones = env._get_dones

    def dones_without_reset():
        terminated, truncated = original_dones()
        fell.logical_or_(terminated)
        # Returning all-False keeps `DirectRLEnv.step` from resetting anyone.
        return torch.zeros_like(terminated), torch.zeros_like(truncated)

    torch.manual_seed(manifest["seed"])
    obs, _ = env.reset()
    env.episode_length_buf[:] = 0
    env._get_dones = dones_without_reset

    fell_at: list[int | None] = [None] * n
    digests = [hashlib.sha256() for _ in range(n)]
    trace = {k: [[] for _ in range(n)] for k in
             ("goal_err", "unified_err", "force_cmd", "force_applied", "force_along_cmd",
              "est_err", "base_vel_err", "base_z")}

    try:
        with torch.no_grad():
            for step in range(max_steps):
                if policy is None:
                    action = torch.zeros(n, interface.NUM_ACTIONS, device=env.device)
                    estimate = None
                else:
                    action = policy.act(obs["policy"])
                    estimate = policy.estimate(obs["policy"])
                obs, _, _, _, _ = env.step(action)

                tip = env._tip_pos()
                goal = env._goal_world()
                goal_sphere = env._goals.current
                applied = env._ee_force_w
                cmd_local = env._commands[:, interface.CMD_EE_FORCE]
                vel_cmd = env._commands[:, :3]
                stiffness = env._gripper_force_kp

                cmd_world = interface.quat_apply(env._base_yaw_quat(), cmd_local)
                unified_target = goal + (applied + cmd_world) / stiffness

                goal_err = torch.norm(tip - goal, dim=1)
                unified_err = torch.norm(tip - unified_target, dim=1)
                cmd_mag = torch.norm(cmd_world, dim=1)
                applied_mag = torch.norm(applied, dim=1)
                direction = cmd_world / cmd_mag.clamp(min=1e-6).unsqueeze(1)
                realised = ((tip - goal) * direction).sum(dim=1) * stiffness[:, 0]
                vel_err = torch.norm(env._robot.data.root_lin_vel_b[:, :2] - vel_cmd[:, :2], dim=1)
                base_z = env._robot.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2]

                if estimate is not None:
                    predicted = torch.norm(
                        estimate[:, 6:9] / observations.OBS_SCALE_EE_FORCE, dim=1)
                    est_err = (predicted - applied_mag).abs().tolist()
                else:
                    est_err = [None] * n

                for i in fell.nonzero().flatten().tolist():
                    if fell_at[i] is None:
                        fell_at[i] = step

                sph, fcl, vcm = goal_sphere.tolist(), cmd_local.tolist(), vel_cmd.tolist()
                ge, ue = goal_err.tolist(), unified_err.tolist()
                cm, am, rl = cmd_mag.tolist(), applied_mag.tolist(), realised.tolist()
                ve, bz = vel_err.tolist(), base_z.tolist()
                for i in range(n):
                    # Hashed in the policy-independent frames: the goal in arm-frame spherical
                    # coordinates, which is what the trajectory is generated in and what the
                    # observation carries, and the force command in the base-yaw frame.
                    digests[i].update(",".join(
                        f"{v:.4f}" for v in (sph[i] + fcl[i] + vcm[i])).encode())
                    if fell_at[i] is None:
                        trace["goal_err"][i].append(ge[i])
                        trace["unified_err"][i].append(ue[i])
                        trace["force_cmd"][i].append(cm[i])
                        trace["force_applied"][i].append(am[i])
                        trace["force_along_cmd"][i].append(rl[i] if cm[i] > 1e-6 else None)
                        trace["est_err"][i].append(est_err[i])
                        trace["base_vel_err"][i].append(ve[i])
                        trace["base_z"][i].append(bz[i])
    finally:
        env._get_dones = original_dones

    records = []
    for i, spec in enumerate(episodes):
        quiet = {j for j, (c, a) in enumerate(zip(trace["force_cmd"][i], trace["force_applied"][i]))
                 if c < 0.1 and a < 0.1}
        pushed = [j for j in range(len(trace["force_cmd"][i])) if j not in quiet]
        records.append({
            "index": spec["index"],
            "steps": fell_at[i] if fell_at[i] is not None else max_steps,
            "fell": fell_at[i] is not None,
            "timed_out": fell_at[i] is None,
            "schedule_sha256": digests[i].hexdigest(),
            "goal_tracking_quiet_m": _stats([trace["goal_err"][i][j] for j in sorted(quiet)]),
            "unified_tracking_m": _stats(trace["unified_err"][i]),
            "unified_tracking_pushed_m": _stats([trace["unified_err"][i][j] for j in pushed]),
            "force_cmd_n": _stats([trace["force_cmd"][i][j] for j in pushed]),
            "force_realised_n": _stats([trace["force_along_cmd"][i][j] for j in pushed]),
            "estimator_err_n": _stats(trace["est_err"][i]),
            "base_vel_err_m_s": _stats(trace["base_vel_err"][i]),
            "base_z_m": _stats(trace["base_z"][i]),
        })
        if progress:
            progress(records[-1])
    return records


def summarise(records, manifest, controller, conditions=None) -> dict:
    """Pool the per-episode records. Same shape as the Isaac Gym evaluator's summary."""
    episodes = len(records)
    falls = sum(1 for r in records if r["fell"])
    expected = {e["index"]: e.get("schedule_sha256") for e in manifest["episodes"]}
    drifted = [r["index"] for r in records
               if expected.get(r["index"]) and expected[r["index"]] != r["schedule_sha256"]]
    return {
        "controller": controller,
        "simulator": "isaaclab",
        "manifest_role": manifest["role"],
        "manifest_sha256": manifest["content_sha256"],
        "episodes": episodes,
        "falls": {"count": falls, "rate": falls / episodes if episodes else None},
        "survived_to_time_limit": sum(1 for r in records if r["timed_out"]),
        "schedule_mismatches": drifted,
        "goal_tracking_quiet_m": _pool(records, "goal_tracking_quiet_m"),
        "unified_tracking_m": _pool(records, "unified_tracking_m"),
        "unified_tracking_pushed_m": _pool(records, "unified_tracking_pushed_m"),
        "force_cmd_n": _pool(records, "force_cmd_n", "mean"),
        "force_realised_n": _pool(records, "force_realised_n", "mean"),
        "estimator_err_n": _pool(records, "estimator_err_n"),
        "base_vel_err_m_s": _pool(records, "base_vel_err_m_s"),
        "base_z_m": _pool(records, "base_z_m"),
        "condition_mismatches": conditions or [],
    }


class EvaluablePolicy:
    """A checkpoint from either stack, exposing the same two calls the evaluator needs.

    Two formats have to load here and they are not interchangeable. A UniFP checkpoint keeps the
    encoder, actor and decoder in one `nn.Module` under `model_state_dict`; an rsl-rl 5.x one
    splits the actor and critic into separate models under `actor_state_dict`. The mapping between
    them is the same one `tests/test_unifp_train.py` checks exactly, where a loaded `UniFPActor`
    reproduces the verified reference loader's actions to 0.0 — so a policy trained in either
    stack is run here by the same code path, and any difference in the numbers below is the
    policy rather than the plumbing.
    """

    #: UniFP's module names against rsl-rl's. `actor_body` becomes `mlp` because it *is* the
    #: `MLPModel`'s own network; the encoder is what `get_latent()` puts in front of it.
    RENAME = {"adaptation_encoder_module": "encoder", "adaptation_decoder_module": "decoder",
              "actor_body": "mlp"}

    def __init__(self, checkpoint: str, device: str = "cuda:0") -> None:
        from tensordict import TensorDict

        from .models import UniFPActor

        raw = torch.load(checkpoint, map_location="cpu", weights_only=False)
        probe = TensorDict(
            {"policy": torch.zeros(1, interface.NUM_OBS),
             "critic": torch.zeros(1, observations.NUM_CRITIC_OBS)}, batch_size=[1])
        self.actor = UniFPActor(
            probe, {"actor": ["policy"], "critic": ["critic"]}, "actor", interface.NUM_ACTIONS,
            distribution_cfg={"class_name": "GaussianDistribution", "init_std": 1.0,
                              "std_type": "scalar"})

        if "actor_state_dict" in raw:
            self.actor.load_state_dict(raw["actor_state_dict"], strict=True)
            self.source, self.iteration = "isaaclab", raw.get("iter")
        else:
            mapped = {}
            for key, value in raw["model_state_dict"].items():
                head, _, rest = key.partition(".")
                if head == "std":
                    mapped["distribution.std_param"] = value
                elif head in self.RENAME:
                    mapped[f"{self.RENAME[head]}.{rest}"] = value
            self.actor.load_state_dict(mapped, strict=True)
            self.source = "isaacgym"
            self.iteration = raw.get("iter") or raw.get("infos")
        self.actor.to(device).eval()
        self.device = device
        self._groups = {"actor": ["policy"], "critic": ["critic"]}

    def _wrap(self, stacked: torch.Tensor):
        from tensordict import TensorDict

        return TensorDict({"policy": stacked}, batch_size=[stacked.shape[0]])

    def act(self, stacked: torch.Tensor) -> torch.Tensor:
        """Deterministic action: the distribution mean, as both stacks' play paths use."""
        return self.actor(self._wrap(stacked))

    def estimate(self, stacked: torch.Tensor) -> torch.Tensor:
        """The adaptation module's prediction, in the scaled units the decoder outputs."""
        return self.actor.estimate(self._wrap(stacked))
