"""S4c (2026-11-19, the week 10 log "S4c"): the arrival-gated grasp (`--mech_grasp_within_m`, `--mech_engage_max_wait_s`)
and the mechanism evaluation's new per-episode fields (the grasp error, the closest approach, J1-J5's lead over the D1
link's report), on `test_mech_layer`'s CPU stand-in: the real `Go2D1MechEnv` compiled from its source onto a scripted
base environment.

`S4C_MECH_ENV_SRC` / `S4C_MECH_EVAL_SRC` point the tests at other copies of `unifp_train/mech_env.py` / `mech_eval.py`
(to test an edit before it replaces the live file while a queue imports it)."""
from __future__ import annotations

import importlib.util
import os
import types
import unittest
from pathlib import Path

import numpy as np
import torch

from tests import test_mech_layer as tml
from unifp_isaaclab import interface
from unifp_train import mech_cfg

ENV_SRC = Path(os.environ.get("S4C_MECH_ENV_SRC", tml.MECH_SRC))
EVAL_SRC = Path(os.environ.get("S4C_MECH_EVAL_SRC", tml.REPO / "unifp_train" / "mech_eval.py"))
NS = tml.MechEnv._engage_due.__globals__           # the namespace test_mech_layer compiles the env in
MechEnvNew = tml._source_class(ENV_SRC, "Go2D1MechEnv", dict(NS))


def _mech_eval():
    spec = importlib.util.spec_from_file_location("unifp_train.mech_eval_s4c", EVAL_SRC)   # its relative imports
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _env(cls, steps=330, seed=3, **cfg):
    torch.manual_seed(seed)
    placements = mech_cfg.s4_set("validation", mech_cfg.S4B_MANIFEST)[0][:2]
    n = len(tml.KINDS) * len(tml.LEVELS) * len(placements)
    env = cls(tml._cfg(**cfg), None, num_envs=n, episode_steps=steps, seed=seed)
    env.set_evaluation(*tml.mech_eval.build_plan(tml.KINDS, tml.LEVELS, "cpu", placements),
                       conditions=dict(mech_cfg.DEV_CONDITIONS))
    env.set_force_law(400.0, 1000.0, 80.0, bleed_s=1.0)
    return env


def _roll(env, steps):
    """Zero actions for `steps` policy steps; each step's grasp step, grasp error and tool-goal distance."""
    env.reset()
    act = torch.zeros(env.num_envs, interface.NUM_ACTIONS)
    out = {"grasp_step": [], "grasp_error": [], "dist": [], "ever": [], "goal": []}
    for _ in range(steps):
        env.step(act)
        out["grasp_step"].append(env._grasp_step.clone())
        out["grasp_error"].append(env._grasp_error.clone())
        out["dist"].append((env._tip_pos() - env._goal_world()).norm(dim=-1))
        out["ever"].append(env._ever_grasped.clone())
        out["goal"].append(env._goals.current.clone())
    return {k: torch.stack(v) for k, v in out.items()}


class GraspGate(unittest.TestCase):
    STEPS = 300

    def test_off_is_the_old_rule_bit_for_bit(self):
        old = _roll(_env(tml.MechEnv), self.STEPS)
        for extra in ({}, {"grasp_within_m": None, "engage_max_wait_steps": None}):
            new = _roll(_env(MechEnvNew, **extra), self.STEPS)
            for key in old:
                self.assertTrue(torch.equal(old[key], new[key]), f"{key} with {extra}")

    def test_grasps_close_only_within(self):
        # The stand-in's scripted tool arrives within about 1 cm, so the radius is taken from the ungated grasps' own
        # errors: under it some of them, over it the rest.
        old = _roll(_env(tml.MechEnv), self.STEPS)
        old_err = old["grasp_error"][-1][old["ever"][-1]]
        r = float(old_err.median())
        self.assertGreater(float(old_err.max()), r, "without the gate some grasp closes further out")
        g = _roll(_env(MechEnvNew, grasp_within_m=r), self.STEPS)
        ever = g["ever"][-1]
        self.assertTrue(bool(ever.any()), "the stand-in's tool reaches the radius in some episodes")
        err = g["grasp_error"][-1][ever]
        self.assertTrue(bool((err <= r + 1e-6).all()), f"grasp errors {err.tolist()} past {r}")

    def test_never_within_never_grasps(self):
        g = _roll(_env(MechEnvNew, grasp_within_m=1e-9, engage_max_wait_steps=5), self.STEPS)
        self.assertFalse(bool(g["ever"].any()))

    def test_longer_wait_moves_only_fallback_grasps(self):
        old = _roll(_env(tml.MechEnv), self.STEPS)
        new = _roll(_env(MechEnvNew, engage_max_wait_steps=10_000), self.STEPS)
        both = old["ever"][-1] & new["ever"][-1]
        self.assertTrue(bool((new["grasp_step"][-1][both] >= old["grasp_step"][-1][both]).all()))


class Approach(unittest.TestCase):
    def test_off_adds_nothing(self):
        for extra in ({}, {"approach_weight": 0.0}):
            env = _env(MechEnvNew, **extra)
            self.assertNotIn("mech_approach", env._weights)
            self.assertNotIn("mech_approach", env._terms)

    def test_pays_the_approach_then_its_full_value_once_grasped(self):
        env = _env(MechEnvNew, approach_weight=2.0)
        self.assertAlmostEqual(env._weights["mech_approach"], 2.0 * interface.POLICY_DT)
        env.reset()
        act = torch.zeros(env.num_envs, interface.NUM_ACTIONS)
        seen_before = seen_after = False
        for _ in range(250):
            env.step(act)
            state = types.SimpleNamespace(ee_pos_w=env._tip_pos(), ee_goal_w=env._goal_world())
            value = env._terms["mech_approach"](state)
            dist = (env._tip_pos() - env._goal_world()).norm(dim=-1)
            before = env._mech_env & ~env._ever_grasped
            want = torch.where(env._ever_grasped, torch.ones_like(dist), torch.exp(-dist / 0.04)) * env._mech_env.float()
            self.assertTrue(bool((value <= 1.0 + 1e-6).all()))
            self.assertTrue(torch.allclose(value, want, atol=1e-6))
            seen_before |= bool(before.any())
            seen_after |= bool((~before).any())
        self.assertTrue(seen_before and seen_after, "the roll has steps before and after grasps")


class GraspedFraction(unittest.TestCase):
    def _fraction(self, **cfg):
        env = _env(MechEnvNew, **cfg)
        env.reset()
        env.step(torch.zeros(env.num_envs, interface.NUM_ACTIONS))
        for _ in range(250):
            env.step(torch.zeros(env.num_envs, interface.NUM_ACTIONS))
        env.extras["log"] = {}
        env._update_curriculum(torch.arange(env.num_envs))
        return env.extras["log"].get("Mechanism/grasped_fraction")

    def test_logged(self):
        self.assertEqual(self._fraction(), 1.0)                                       # the old rule grasps everywhere
        self.assertEqual(self._fraction(grasp_within_m=1e-9, engage_max_wait_steps=5), 0.0)


class TraceKeeps(unittest.TestCase):
    def test_the_loop_keeps_every_new_field(self):
        # The evaluation loop (`run_unifp_train.mech_evaluate`) keeps only `mech_eval.FIELDS` of what `record_step`
        # returns; S4c-0's first runs lost the new fields that way.
        me = _mech_eval()
        for key in ("tool_goal_m", "grasp_error_m", "arm_lead_deg"):
            self.assertIn(key, me.FIELDS)

    def test_record_step_returns_every_field(self):
        me = _mech_eval()
        env = _env(MechEnvNew)
        env.reset()
        env.step(torch.zeros(env.num_envs, interface.NUM_ACTIONS))
        none = torch.zeros(env.num_envs, dtype=torch.bool)
        try:
            step = me.record_step(env, None, none, none)
        except AttributeError as err:      # the stand-in lacks a buffer only Isaac has
            self.skipTest(f"stand-in: {err}")
        self.assertEqual(set(me.FIELDS) - set(step), set())

    def test_the_approach_counts_from_the_engage_step(self):
        # The goal starts at the hand (distance 0 at the reset), so the closest approach is read only once a grasp may close.
        me = _mech_eval()
        env = _env(MechEnvNew, grasp_within_m=1e-9)
        env.reset()
        none = torch.zeros(env.num_envs, dtype=torch.bool)
        before = after = 0
        for _ in range(200):
            env.step(torch.zeros(env.num_envs, interface.NUM_ACTIONS))
            d = me.record_step(env, None, none, none)["tool_goal_m"]
            due = (env._since_reset >= env._engage_step).numpy()
            self.assertTrue(np.isnan(d[~due]).all())
            self.assertTrue(np.isfinite(d[due]).all())
            before, after = before + int((~due).sum()), after + int(due.sum())
        self.assertTrue(before > 0 and after > 0)


class HandleHeld(unittest.TestCase):
    """With the gate, the goal stays on the handle from the approach's arrival until the grasp (S4c-1's first run measured
    the tool against UniFP's next goal once the hold had ended)."""

    def _check(self, **cfg):
        env = _env(MechEnvNew, steps=330, **cfg)
        env.reset()
        act = torch.zeros(env.num_envs, interface.NUM_ACTIONS)
        waiting = 0
        for _ in range(320):
            env.step(act)
            g = env._goals
            pre = env._mech_env & ~env._ever_grasped & (env._since_reset > env._engage_step)
            if bool(pre.any()):
                self.assertTrue(torch.allclose(g.current[pre], env._handle_sphere[pre], atol=1e-6))
                waiting += int(pre.sum())
        return env, waiting

    def test_never_grasping_keeps_the_handle(self):
        # 320 steps is well past every approach and its longest hold (3 s + 2 s at 50 Hz is 250 steps).
        env, waiting = self._check(grasp_within_m=1e-9, engage_max_wait_steps=5)
        self.assertGreater(waiting, 1000)
        self.assertFalse(bool(env._ever_grasped.any()))

    def test_grasps_are_at_the_handle(self):
        old = _roll(_env(tml.MechEnv), 300)
        r = float(old["grasp_error"][-1][old["ever"][-1]].median())
        env = _env(MechEnvNew, grasp_within_m=r)
        env.reset()
        act = torch.zeros(env.num_envs, interface.NUM_ACTIONS)
        handle_err = []
        for _ in range(300):
            before = env._ever_grasped.clone()
            seen = env._goals.current.clone()          # the goal `_engage_due` measures against on this step
            env.step(act)
            new = env._ever_grasped & ~before
            if bool(new.any()):
                self.assertTrue(torch.allclose(seen[new], env._handle_sphere[new], atol=1e-6))
                handle_err.append(env._grasp_error[new])
        errs = torch.cat(handle_err) if handle_err else torch.zeros(0)
        self.assertGreater(len(errs), 0)
        self.assertTrue(bool((errs <= r + 1e-6).all()))

    def test_off_never_holds_before_the_grasp(self):
        env = _env(MechEnvNew)
        env.reset()
        act = torch.zeros(env.num_envs, interface.NUM_ACTIONS)
        for _ in range(300):
            env.step(act)
            self.assertTrue(bool((env._goals.hold_mask == env._ever_grasped).all()))


class EpisodeFields(unittest.TestCase):
    def _trace(self, T=200, lead=None, dist=None, grasp_at=80, err=0.031):
        nan = np.full((T, 1), np.nan)
        grasped = np.zeros((T, 1)); grasped[grasp_at:] = 1.0
        s = np.zeros((T, 1)); s[grasp_at:, 0] = np.linspace(0.0, 0.12, T - grasp_at)
        tr = {"grasped": grasped, "released": np.zeros((T, 1)), "s": s, "v": np.zeros((T, 1)), "ref": s.copy(),
              "travel": np.full((T, 1), 0.12), "drive": np.full((T, 1), 10.0), "force": np.full((T, 1), 10.0),
              "arm_load": np.full((T, 1), 0.2), "arm_loads": np.full((T, 1, 6), 0.2), "pitch": np.zeros((T, 1)),
              "height": np.full((T, 1), 0.29), "base_x": np.zeros((T, 1)), "base_y": np.zeros((T, 1)),
              "estimate_drive": nan.copy(), "arm_contact": np.zeros((T, 1)), "body_contact": np.zeros((T, 1)),
              "torn": np.zeros((T, 1)), "fallen": np.zeros((T, 1)), "tilt_stop": np.zeros((T, 1))}
        if lead is not None:
            tr["arm_lead_deg"] = lead.reshape(T, 1)
            tr["tool_goal_m"] = dist.reshape(T, 1)
            ge = np.zeros((T, 1)); ge[grasp_at:] = err
            tr["grasp_error_m"] = ge
        return tr

    def test_old_traces_score_as_before(self):
        me = _mech_eval()
        live = tml.mech_eval
        tr = self._trace()
        self.assertEqual(me.score_episode(tr, 0), live.score_episode(tr, 0))

    def test_new_fields(self):
        me = _mech_eval()
        T = 200
        lead = np.full(T, 5.0); lead[60:100] = 30.0; lead[150:] = np.nan     # ended at 150
        dist = np.linspace(0.2, 0.01, T)
        tr = self._trace(T, lead, dist)
        base = me.score_episode(self._trace(T), 0)
        out = me.score_episode(tr, 0)
        for key, value in base.items():
            self.assertEqual(out[key], value, key)
        self.assertEqual(out["grasp_error_m"], 0.031)
        self.assertEqual(out["lead_deg_max_after_1s"], 30.0)
        late = lead[int(round(1.0 / interface.POLICY_DT)):150]
        self.assertAlmostEqual(out["lead_over15_share_after_1s"], round(float(np.mean(late > 15.0)), 3))
        self.assertEqual(out["lead_deg_median_held"], 5.0)
        self.assertAlmostEqual(out["closest_to_goal_m"], round(float(dist[:81].min()), 4))   # up to the grasp (step 80)

    def test_no_grasp_still_reports_the_approach(self):
        me = _mech_eval()
        T = 120
        tr = self._trace(T, np.full(T, 4.0), np.full(T, 0.07), grasp_at=T)
        out = me.score_episode(tr, 0)
        self.assertIsNone(out["grasp_step"])
        self.assertEqual(out["closest_to_goal_m"], 0.07)
        self.assertEqual(out["lead_deg_max_after_1s"], 4.0)
        self.assertNotIn("grasp_error_m", out)


if __name__ == "__main__":
    unittest.main()
