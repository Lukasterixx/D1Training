"""The Isaac Lab training port's task logic, checked against the environment that trained the policy.

`tests/data/unifp_task_48800.npz` is 160 policy steps recorded out of the running Isaac Gym
environment by `unifp_go2d1/dump_interface.py`: the full state each step, the gait reference and
stance mask, and **each of the 27 reward terms separately**, taken as the change in the
environment's own `episode_sums` so that nothing is re-evaluated (`_reward_feet_air_time` mutates
state, so calling it twice would change the run being recorded).

Porting 27 reward terms by reading them is how a sign or an index error gets in and is never seen
again -- it disappears into a weighted sum and comes back as a policy that trains to something
slightly wrong. These tests compare each term against its own source.

Two conventions the fixture encodes, both established by measurement rather than assumed:

  * **A reward is computed from post-step state.** The reward recorded at step `t` uses the state
    recorded at index `t+1`. Checked by scanning the offset: at `t+1` the error is 1e-10, at any
    neighbouring shift it is 1e-2.
  * **`feet_air_time` and `last_contacts` are the exception.** They are mutated inside the reward
    itself, so the value recorded at index `t` is exactly what step `t`'s reward starts from.
"""
from __future__ import annotations

import os
from pathlib import Path
import unittest

import numpy as np
import torch

from unifp_isaaclab import interface
from unifp_train import forces, gait, observations, rewards, task_cfg

FIXTURE = Path(__file__).resolve().parent / "data/unifp_task_48800.npz"

#: The trained checkpoint (24 MB, not in the repository). The model tests skip without it.
CHECKPOINT = Path(os.environ.get(
    "UNIFP_CHECKPOINT",
    os.path.expanduser("~/thesis_b_legacy/UniFP/logs/go2d1_pos_force/Sep18_11-10-15_/model_48800.pt")))

#: float32 rounding. The reference came off a GPU and is rebuilt on the CPU.
TOLERANCE = 2e-6

#: Terms that never fire in the reference rollout: the robot never collides, never reaches a joint
#: limit, never lifts a foot past 20 cm, and is never commanded to stand. They are ported and
#: their arithmetic is exercised, but agreeing with a column of zeros is not evidence that they
#: are right. Anything relying on them needs a rollout that provokes them.
NEVER_EXERCISED = ("collision", "dof_pos_limits", "feet_height_high", "stand_still")


def load():
    return np.load(FIXTURE, allow_pickle=True)


class TestWeights(unittest.TestCase):
    """The reward weights, against the ones the environment was running."""

    def setUp(self):
        self.data = load()
        self.recorded = {str(n): float(s)
                         for n, s in zip(self.data["reward_names"], self.data["reward_scales"])}

    def test_every_term_is_accounted_for(self):
        self.assertEqual(set(self.recorded), set(task_cfg.REWARD_WEIGHTS))
        self.assertEqual(set(self.recorded), set(rewards.TERMS))

    def test_weights_match_after_the_dt_multiply(self):
        # `_prepare_reward_function` folds dt into every scale on startup. Applying it twice is a
        # silent 50x error in every term at once, so it is checked rather than trusted.
        ours = task_cfg.scaled_weights()
        for name, recorded in self.recorded.items():
            with self.subTest(term=name):
                self.assertAlmostEqual(ours[name], recorded, places=6)


class TestGait(unittest.TestCase):
    """The gait clock's two derived quantities."""

    def setUp(self):
        self.data = load()
        self.phase = torch.as_tensor(self.data["gait_indices"], dtype=torch.float32)

    def test_stance_mask(self):
        expected = torch.as_tensor(self.data["stance_mask"])
        self.assertLess(float((gait.stance_mask(self.phase) - expected).abs().max()), TOLERANCE)

    def test_reference_leg_pose(self):
        expected = torch.as_tensor(self.data["ref_dof_pos"])
        self.assertLess(float((gait.reference_leg_pos(self.phase) - expected).abs().max()), TOLERANCE)

    def test_diagonals_move_together(self):
        # FL pairs with RR, FR with RL. A transposed pairing still produces a plausible gait, so
        # it is worth asserting rather than eyeballing.
        mask = gait.stance_mask(torch.linspace(0, 1, 50))
        self.assertTrue(torch.equal(mask[:, 0], mask[:, 3]))
        self.assertTrue(torch.equal(mask[:, 1], mask[:, 2]))

    def test_double_support_exists(self):
        # Both diagonals in stance at once, which is what the threshold offset is for.
        mask = gait.stance_mask(torch.linspace(0, 1, 200))
        self.assertTrue(bool((mask.sum(dim=1) == 4).any()))


class TestRewardTerms(unittest.TestCase):
    """Every reward term, against the value the environment accumulated for it."""

    @classmethod
    def setUpClass(cls):
        data = load()
        scales = {str(n): float(s) for n, s in zip(data["reward_names"], data["reward_scales"])}
        count = len(data["rew_buf"])
        low, high = 2, count - 2

        def at(key, index):
            return torch.as_tensor(data[key][index], dtype=torch.float32).unsqueeze(0)

        air = at("feet_air_time", low)
        contacts = at("last_contacts", low).bool()
        predicted = {name: [] for name in task_cfg.REWARD_WEIGHTS}
        for step in range(low, high):
            phase = torch.as_tensor(data["gait_indices"][step + 1:step + 2], dtype=torch.float32)
            quat = data["base_yaw_quat"][step + 1]      # Isaac Gym stores w last
            state = rewards.TaskState(
                base_lin_vel_b=at("base_lin_vel", step + 1),
                base_ang_vel_b=at("base_ang_vel", step + 1),
                root_pos_w=at("root_states", step + 1)[:, :3],
                dof_pos=at("dof_pos", step + 1), dof_vel=at("dof_vel", step + 1),
                last_dof_vel=at("dof_vel", step), torques=at("torques", step),
                actions=at("action", step), last_actions=at("action", step - 1),
                commands=at("commands", step + 1), gait_phase=phase,
                ref_leg_pos=gait.reference_leg_pos(phase),
                stance_mask=at("stance_mask", step + 1), contact_mask=at("contact_mask", step + 1),
                feet_pos_w=at("feet_pos", step + 1),
                feet_contact_forces=at("feet_contact_forces", step + 1),
                feet_air_time=air, penalised_contact_forces=torch.zeros(1, 1, 3),
                ee_pos_w=at("ee_pos", step + 1), ee_goal_w=at("ee_goal_cart_world", step + 1),
                thigh_pos_w=at("thigh_pos", step + 1), feet_vel_w=at("feet_vel", step + 1),
                last_contacts=contacts,
                base_yaw_quat=torch.as_tensor(
                    np.r_[quat[3], quat[:3]], dtype=torch.float32).unsqueeze(0),
                ee_force_measured=at("ee_force_measured", step + 1),
                ee_force_cmd=at("ee_force_cmd", step + 1),
                base_force_measured=at("base_force_measured", step + 1),
                base_force_cmd=at("base_force_cmd", step + 1),
                gripper_force_kp=at("gripper_force_kp", step + 1),
                base_force_kd=at("base_force_kd", step + 1))
            for name, function in rewards.TERMS.items():
                predicted[name].append(float(function(state)[0]) * scales[name])
            # feet_air_time advances these in place; carry them as the environment does.
            air, contacts = state.feet_air_time, state.last_contacts

        cls.predicted = {name: np.asarray(values) for name, values in predicted.items()}
        cls.recorded = {name: data[f"reward_{name}"][low:high] for name in task_cfg.REWARD_WEIGHTS}

    def test_each_term(self):
        for name in sorted(task_cfg.REWARD_WEIGHTS):
            with self.subTest(term=name):
                worst = float(np.abs(self.predicted[name] - self.recorded[name]).max())
                self.assertLess(worst, TOLERANCE, f"{name} differs by {worst}")

    def test_the_two_objectives_are_actually_exercised(self):
        # These are the terms the task is about; a zero column here would make the whole
        # comparison vacuous.
        for name in ("tracking_ee_force_world", "tracking_lin_vel_force_world"):
            with self.subTest(term=name):
                self.assertGreater(float(np.abs(self.recorded[name]).mean()), 1e-3)

    def test_stateful_air_time_tracks_across_steps(self):
        # feet_air_time is carried across steps rather than read per step, so a drift would show
        # up late in the window rather than at the start.
        recorded, predicted = self.recorded["feet_air_time"], self.predicted["feet_air_time"]
        self.assertGreater(float(np.abs(recorded).max()), 0.0)
        second_half = slice(len(recorded) // 2, None)
        self.assertLess(float(np.abs(predicted[second_half] - recorded[second_half]).max()), TOLERANCE)

    def test_unexercised_terms_are_declared(self):
        # Guards the honesty of the suite: if a term stops being all-zero in a future fixture,
        # this fails and NEVER_EXERCISED should shrink.
        for name in NEVER_EXERCISED:
            with self.subTest(term=name):
                self.assertEqual(float(np.abs(self.recorded[name]).max()), 0.0)



def upstream_push_gripper(state, episode_length, cfg):
    """A transcription of UniFP's `_push_gripper`, one channel, for differential testing.

    Line for line from `legged_gym/envs/go2d1/legged_robot_go2d1_pos_force.py`, reduced to the
    command channel and to plain tensors. It exists so `forces.PushSchedule` is checked against
    the code it was ported from rather than against a restatement of the same idea by the same
    author -- which is what a hand-written expected waveform would be.

    `state` is a dict of the buffers upstream keeps on `self`; it is mutated in place.
    """
    num_envs = episode_length.shape[0]
    env_ids_all = torch.arange(num_envs)
    generator = cfg["generator"]

    def rand_float(lower, upper, shape):
        return (upper - lower) * torch.rand(*shape, generator=generator) + lower

    new_selected = env_ids_all[(episode_length % state["push_interval"]) == 0]
    if new_selected.nelement() > 0:
        count = len(new_selected)
        state["freed"][new_selected] = torch.rand(
            count, generator=generator) > cfg["forced_prob"]
        low, high = cfg["force_range"]
        for axis in range(3):
            state["force_target"][new_selected, axis] = rand_float(low, high, (count, 1)).view(count)
        duration = rand_float(cfg["duration_min"], cfg["duration_max"], (count, 1)).view(count)
        duration = torch.clip(
            duration, max=(state["push_interval"][new_selected] - cfg["settling"]) / 2)
        state["push_end_time"][new_selected] = episode_length[new_selected] + duration
        state["push_duration"][new_selected] = duration
        state["selected"][new_selected] = 1

    if episode_length[state["selected"] == 1].nelement() > 0:
        subset = env_ids_all[state["selected"] == 1]

        step1 = subset[episode_length[state["selected"] == 1]
                       < (state["push_end_time"][state["selected"] == 1]).type(torch.int32)]
        if step1.nelement() > 0:
            span = state["push_duration"][step1].unsqueeze(-1)
            state["current"][step1, :3] = (state["force_target"][step1, :3] / span) * (
                torch.clamp(episode_length[step1].unsqueeze(-1)
                            - (state["push_end_time"][step1].unsqueeze(-1) - span),
                            torch.zeros_like(span), span))

        step2 = subset[episode_length[state["selected"] == 1]
                       > (state["push_end_time"][state["selected"] == 1]
                          + cfg["settling"]).type(torch.int32)]
        if step2.nelement() > 0:
            span = state["push_duration"][step2].unsqueeze(-1)
            state["current"][step2, :3] = state["force_target"][step2, :3] - (
                state["force_target"][step2, :3] / span) * (
                torch.clamp(episode_length[step2].unsqueeze(-1)
                            - (state["push_end_time"][step2].unsqueeze(-1) + cfg["settling"]),
                            torch.zeros_like(span), span))

        finished = subset[episode_length[state["selected"] == 1]
                          >= (state["push_end_time"][state["selected"] == 1] + cfg["settling"]
                              + state["push_duration"][state["selected"] == 1]).type(torch.int32)]
        if finished.nelement() > 0:
            state["selected"][finished] = 0
            state["force_target"][finished, :3] = 0.0
            state["current"][finished, :3] = 0.0
            state["push_end_time"][finished] = 0.0
            state["push_duration"][finished] = 0.0
            state["push_interval"][finished] = torch.randint(
                int(cfg["interval_min"]), int(cfg["interval_max"]), (len(finished), 1),
                generator=generator)[:, 0]

    freed = state["freed"]
    state["selected"][freed] = 0
    state["force_target"][freed, :3] = 0.0
    state["current"][freed, :3] = 0.0
    state["push_end_time"][freed] = 0.0
    state["push_duration"][freed] = 0.0
    return state["current"]


class TestForceSchedule(unittest.TestCase):
    """`forces.PushSchedule` against a transcription of the code it was ported from."""

    ENVS = 12
    STEPS = 1200

    def setUp(self):
        self.ported = forces.PushSchedule(
            self.ENVS, "cpu", clear_on_finish=True,
            generator=torch.Generator().manual_seed(7))
        reference_generator = torch.Generator().manual_seed(7)
        self.reference_cfg = {
            "generator": reference_generator,
            "forced_prob": task_cfg.GRIPPER_FORCED_PROB,
            "force_range": task_cfg.GRIPPER_FORCE_RANGE_N,
            "duration_min": float(forces.steps_from_seconds(task_cfg.PUSH_DURATION_S[0])),
            "duration_max": float(forces.steps_from_seconds(task_cfg.PUSH_DURATION_S[1])),
            "settling": float(forces.steps_from_seconds(task_cfg.SETTLING_TIME_GRIPPER_S)),
            "interval_min": forces.steps_from_seconds(task_cfg.PUSH_INTERVAL_S[0]),
            "interval_max": forces.steps_from_seconds(task_cfg.PUSH_INTERVAL_S[1]),
        }
        # `PushSchedule.__init__` draws the first intervals from its generator, so the reference
        # has to make the same draw or the two random streams are offset by it for the rest of the
        # run -- which is how the first version of this test failed, with the reference's force
        # targets lagging the ported one by exactly one draw.
        initial_intervals = torch.randint(
            self.reference_cfg["interval_min"], self.reference_cfg["interval_max"],
            (self.ENVS,), generator=reference_generator)
        self.assertTrue(torch.equal(initial_intervals, self.ported.interval))
        self.reference = {
            "push_interval": initial_intervals.clone(),
            "force_target": torch.zeros(self.ENVS, 3),
            "current": torch.zeros(self.ENVS, 3),
            "push_duration": torch.zeros(self.ENVS),
            "push_end_time": torch.zeros(self.ENVS),
            "selected": torch.zeros(self.ENVS, dtype=torch.long),
            "freed": torch.zeros(self.ENVS, dtype=torch.bool),
        }

    def run_both(self):
        """Step both implementations together and return (ported, reference) traces."""
        episode_length = torch.zeros(self.ENVS, dtype=torch.long)
        ours, theirs = [], []
        for _ in range(self.STEPS):
            ours.append(self.ported.step(episode_length).clone())
            theirs.append(
                upstream_push_gripper(self.reference, episode_length, self.reference_cfg).clone())
            episode_length = episode_length + 1
        return torch.stack(ours), torch.stack(theirs)

    def test_matches_upstream_step_for_step(self):
        ours, theirs = self.run_both()
        self.assertLess(float((ours - theirs).abs().max()), 1e-6)

    def test_the_schedule_actually_pushes(self):
        # Guards the test above from passing because both sides did nothing.
        ours, _ = self.run_both()
        magnitude = ours.norm(dim=-1)
        self.assertGreater(float(magnitude.max()), 1.0)
        active = float((magnitude > 1e-6).float().mean())
        self.assertGreater(active, 0.2, "a schedule that is almost never pushing is not a schedule")
        self.assertLess(active, 0.95, "and one that never stops is not one either")

    def test_peak_force_stays_inside_the_commanded_band(self):
        ours, _ = self.run_both()
        low, high = task_cfg.GRIPPER_FORCE_RANGE_N
        self.assertGreaterEqual(float(ours.min()), low)
        self.assertLessEqual(float(ours.max()), high)

    def test_a_reset_clears_the_force(self):
        episode_length = torch.zeros(self.ENVS, dtype=torch.long)
        for _ in range(200):
            self.ported.step(episode_length)
            episode_length = episode_length + 1
        self.ported.reset(torch.arange(self.ENVS))
        self.assertEqual(float(self.ported.current.abs().max()), 0.0)

    def test_ramps_are_linear_and_symmetric(self):
        """The shape of one push: up over `duration`, hold, down over `duration`, then nothing."""
        ours, _ = self.run_both()
        trace = ours[:, 0, :]
        magnitude = trace.norm(dim=-1)
        active = (magnitude > 1e-6).nonzero().flatten()
        self.assertGreater(len(active), 0)
        start = int(active[0])
        # Within a push the direction never changes: only the magnitude is ramped.
        direction = trace[start + 5] / trace[start + 5].norm()
        for step in range(start + 5, start + 20):
            if magnitude[step] < 1e-6:
                break
            with self.subTest(step=step):
                self.assertLess(
                    float((trace[step] / magnitude[step] - direction).abs().max()), 1e-5)
        # And the ramp is linear: equal increments.
        increments = magnitude[start + 1:start + 20] - magnitude[start:start + 19]
        self.assertLess(float(increments.std()), 1e-5)


class TestEstimateTarget(unittest.TestCase):
    """The adaptation module's supervised target against the privileged observation layout."""

    def test_blocks_are_the_first_four_privileged_blocks(self):
        from unifp_train.algorithm import ESTIMATE_BLOCKS

        widths = [width for _, width, _ in ESTIMATE_BLOCKS]
        self.assertEqual(widths, [width for _, width in observations.PRIVILEGED_LAYOUT[:4]])
        self.assertEqual(sum(widths), observations.NUM_ESTIMATES)

    def test_the_forces_are_weighted_above_the_kinematics(self):
        # Not a transcription check -- a statement of what the weights are *for*. If someone
        # flattens them, the architecture stops being about forces and this should fail.
        from unifp_train.algorithm import ESTIMATE_BLOCKS

        weights = {label: weight for label, _, weight in ESTIMATE_BLOCKS}
        self.assertGreater(weights["force_ee"], weights["base_velocity"])
        self.assertGreater(weights["force_base"], weights["gripper_pos"])



@unittest.skipUnless(CHECKPOINT.is_file(), f"checkpoint not present at {CHECKPOINT}")
class TestAdaptationActor(unittest.TestCase):
    """`models.UniFPActor` against the network `model_48800` actually is.

    The architecture is not something a shape assertion can settle -- an encoder whose output is
    concatenated on the wrong side of the newest frame has the same shapes and a different policy.
    So the trained weights are loaded into the new model with `strict=True`, which fails if a
    single tensor is the wrong size or missing, and the result is compared against
    `unifp_isaaclab.policy.UniFPPolicy`, which is itself checked against the running Isaac Gym
    environment to 9.5e-06 (F-076). Agreement means the two are the same network, not a network
    of the same size.
    """

    #: UniFP's names for the three MLPs, against rsl-rl's. `actor_body` becomes `mlp` because it
    #: *is* the `MLPModel`'s own network -- the encoder is what `get_latent()` puts in front of it.
    RENAME = {"adaptation_encoder_module": "encoder",
              "adaptation_decoder_module": "decoder",
              "actor_body": "mlp"}

    @classmethod
    def setUpClass(cls):
        from tensordict import TensorDict

        from unifp_train.models import UniFPActor

        state = torch.load(CHECKPOINT, map_location="cpu",
                           weights_only=False)["model_state_dict"]
        cls.envs = 4
        torch.manual_seed(0)
        cls.obs = TensorDict(
            {"policy": torch.randn(cls.envs, interface.NUM_OBS),
             "critic": torch.zeros(cls.envs, observations.NUM_CRITIC_OBS)},
            batch_size=[cls.envs])
        cls.actor = UniFPActor(
            cls.obs, {"actor": ["policy"], "critic": ["critic"]}, "actor", interface.NUM_ACTIONS,
            distribution_cfg={"class_name": "GaussianDistribution", "init_std": 1.0,
                              "std_type": "scalar"})
        mapped = {}
        for key, value in state.items():
            head, _, rest = key.partition(".")
            if head == "std":
                mapped["distribution.std_param"] = value
            elif head in cls.RENAME:
                mapped[f"{cls.RENAME[head]}.{rest}"] = value
        cls.missing = cls.actor.load_state_dict(mapped, strict=True)
        cls.state = state

    def test_every_trained_tensor_has_a_home(self):
        """The checkpoint splits cleanly into this actor and a plain `MLPModel` critic.

        `strict=True` in `setUpClass` already refuses anything missing or unexpected on the actor
        side. The remaining group is `critic_body`, which is a separate model here: legged_gym
        keeps the actor and critic in one `nn.Module` and rsl-rl 5.x keeps them in two. Loading it
        is what shows the split is exactly along that seam and nothing was left over.
        """
        from rsl_rl.models import MLPModel

        groups = {k.partition(".")[0] for k in self.state}
        self.assertEqual(groups, set(self.RENAME) | {"std", "critic_body"})

        critic = MLPModel(self.obs, {"actor": ["policy"], "critic": ["critic"]}, "critic", 1,
                          hidden_dims=[512, 256, 128], activation="elu")
        critic.load_state_dict(
            {f"mlp.{key.partition('.')[2]}": value for key, value in self.state.items()
             if key.startswith("critic_body.")}, strict=True)

    def test_actions_match_the_verified_loader(self):
        from unifp_isaaclab.policy import UniFPPolicy

        reference = UniFPPolicy(CHECKPOINT, device="cpu")
        with torch.no_grad():
            ours = self.actor(self.obs)
        theirs = reference.act(self.obs["policy"])
        self.assertLess(float((ours - theirs).abs().max()), 1e-6)

    def test_estimates_match_the_verified_loader(self):
        from unifp_isaaclab.policy import EE_SPHERE_SCALES, UniFPPolicy

        reference = UniFPPolicy(CHECKPOINT, device="cpu")
        reference.act(self.obs["policy"])
        estimate = reference.estimates()
        # `UniFPPolicy.estimates()` returns SI units; the decoder's own output is scaled, which is
        # what the adaptation loss is computed against, so the scales go back on.
        scale = torch.as_tensor(EE_SPHERE_SCALES)
        theirs = torch.cat((estimate.base_lin_vel * interface.COMMANDS_SCALE[0],
                            estimate.ee_pos_sphere * scale,
                            estimate.ee_force * observations.OBS_SCALE_EE_FORCE,
                            estimate.base_force * observations.OBS_SCALE_BASE_FORCE), dim=-1)
        with torch.no_grad():
            ours = self.actor.estimate(self.obs)
        self.assertLess(float((ours - theirs).abs().max()), 1e-6)

    def test_the_newest_frame_reaches_the_actor_directly(self):
        """Changing only the newest frame must change the action; only the oldest, less so.

        The encoder sees all 32 frames, so nothing is invariant -- but the newest frame is also
        fed straight to the actor body, which is the whole point of the architecture. Reversing
        the stack would swap these two and break nothing else.
        """
        from tensordict import TensorDict

        def perturbed(index):
            stack = self.obs["policy"].clone().view(self.envs, interface.FRAME_STACK, -1)
            stack[:, index, :] += 1.0
            return TensorDict({"policy": stack.reshape(self.envs, -1),
                               "critic": self.obs["critic"]}, batch_size=[self.envs])

        with torch.no_grad():
            base = self.actor(self.obs)
            newest = float((self.actor(perturbed(-1)) - base).abs().mean())
            oldest = float((self.actor(perturbed(0)) - base).abs().mean())
        self.assertGreater(newest, oldest)


if __name__ == "__main__":
    unittest.main()


class TestDebugVisualisation(unittest.TestCase):
    """The viewer overlay's geometry, which is plain torch and needs no simulator."""

    def test_trajectory_endpoints_match_the_goal_generator(self):
        """The drawn path must start and end where the goal itself starts and ends."""
        from unifp_isaaclab import task
        from unifp_train import debug_vis

        goals = task.EeGoalTrajectory(4, device="cpu")
        goals.start, goals.goal = torch.rand(4, 3), torch.rand(4, 3)
        yaw = interface.yaw_quat(torch.rand(4) * 6.283)
        centre = torch.rand(4, 3)
        samples = debug_vis.TRAJECTORY_SAMPLES
        drawn = task.trajectory_samples(goals, samples, yaw, centre)
        self.assertEqual(tuple(drawn.shape), (4, samples, 3))
        for spherical, index in ((goals.start, 0), (goals.goal, samples - 1)):
            expected = centre + interface.quat_apply(yaw, interface.sphere2cart(spherical))
            self.assertLess(float((drawn[:, index] - expected).abs().max()), 1e-6)

    def test_trajectory_is_interpolated_in_spherical_coordinates(self):
        """Every drawn point must be a point the goal will actually pass through.

        Lerping the Cartesian endpoints would agree at the ends and nowhere else, so the check is
        against the generator's own interpolation rather than against a straight line.
        """
        from unifp_isaaclab import task
        from unifp_train import debug_vis

        goals = task.EeGoalTrajectory(3, device="cpu")
        goals.start, goals.goal = torch.rand(3, 3), torch.rand(3, 3)
        yaw = interface.yaw_quat(torch.rand(3) * 6.283)
        centre = torch.rand(3, 3)
        samples = debug_vis.TRAJECTORY_SAMPLES
        drawn = task.trajectory_samples(goals, samples, yaw, centre)
        for i in range(samples):
            t = i / (samples - 1)
            current = torch.lerp(goals.start, goals.goal, t)
            expected = centre + interface.quat_apply(yaw, interface.sphere2cart(current))
            self.assertLess(float((drawn[:, i] - expected).abs().max()), 1e-6)

    def test_arrow_pose_points_along_the_force(self):
        from unifp_train import debug_vis

        vectors = torch.tensor([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0],
                                [-1.0, 0, 0], [3.0, 4.0, 0.0]])
        quat, length = debug_vis.arrow_pose(vectors)
        unit_x = torch.zeros_like(vectors)
        unit_x[:, 0] = 1.0
        rotated = interface.quat_apply(quat, unit_x)
        norms = torch.norm(vectors, dim=-1)
        self.assertLess(float((rotated - vectors / norms[:, None]).abs().max()), 1e-5)
        # a length, not a whole scale: it multiplies the prototype's X and leaves Y and Z alone
        self.assertEqual(tuple(length.shape), (len(vectors),))
        self.assertLess(float((length - norms).abs().max()), 1e-5)

    def test_arrow_pose_collapses_a_zero_force(self):
        """A commanded force of zero must not leave a unit arrow pointing somewhere arbitrary."""
        from unifp_train import debug_vis

        quat, length = debug_vis.arrow_pose(torch.zeros(2, 3))
        self.assertLess(float(length.max()), 1e-3)
        self.assertFalse(bool(torch.isnan(quat).any()), "antiparallel/zero fallback divided by zero")

    def test_arrow_keeps_the_prototype_lateral_scale(self):
        """The length must multiply the prototype's X scale, not replace the whole scale.

        `visualize()` takes an absolute scale, so returning `(length, 1, 1)` would quietly drop
        the config's slim 0.02 cross-section and draw arrows a metre wide -- which is what the
        first version of this overlay did.
        """
        from unifp_train import debug_vis

        self.assertEqual(debug_vis.ARROW_SCALE[0], 1.0,
                         "X must be 1.0 so the force length scales it directly")
        self.assertTrue(all(c < 0.1 for c in debug_vis.ARROW_SCALE[1:]),
                        "the cross-section must stay slim enough not to swallow the robot")
