"""The foot-press fine-tune's starts from the dog's handover (2026-10-23): the dog's recorded rest, the spawn library
built from it and from the simulated handover (`unifp_train/leg_handover.py`,
`demos/combiner_key/assets/dog_stand_20261023/build_starts.py`), and the start gate the export hands the dog
(`export_policy.start_gate`, read by `press_live._stand_up`). No simulator.
"""
import importlib.util
import json
import math
import tempfile
import types
import unittest
from pathlib import Path

import numpy as np

from demos.combiner_key.robot import export_policy, leg_policy, press_layer as pl, press_live
from unifp_isaaclab import interface
from unifp_train import leg_cfg, leg_handover as lh

REPO = Path(__file__).resolve().parents[1]
STARTS = REPO / "demos" / "combiner_key" / "assets" / "dog_stand_20261023"
POLICY = REPO / "checkpoints" / "leg_press_force_r4_model_5999.npz"
DOG_RUN = REPO / "results" / "week_06" / "runs" / "20261023T020839_leg_press_live"


class Orders(unittest.TestCase):
    def test_unitree_from_unifp(self):
        perm = lh.UNITREE_FROM_UNIFP
        self.assertEqual([interface.DOF_NAMES[k] for k in perm], list(leg_policy.UNITREE_LEGS))
        np.testing.assert_array_equal(perm[perm], np.arange(12))      # the same permutation both ways


class TheDogsRest(unittest.TestCase):
    def test_is_the_settle_gates_event(self):
        events = json.loads((DOG_RUN / "events.json").read_text())
        gate = next(e for e in events if "q_minus_trained_settled" in e)
        self.assertEqual(gate["q_minus_trained_settled"], lh.DOG["rest"]["q_minus_settled_unitree"])
        self.assertEqual(gate["roll_pitch_deg"], [lh.DOG["rest"]["roll_deg"], lh.DOG["rest"]["pitch_deg"]])
        self.assertEqual(gate["base_m"], lh.DOG["rest"]["base_m"])
        settled = np.asarray(leg_cfg.SETTLED_LEG_POS)[lh.UNITREE_FROM_UNIFP]
        np.testing.assert_allclose(lh.dog_rest_unifp()[lh.UNITREE_FROM_UNIFP] - settled,
                                   lh.DOG["rest"]["q_minus_settled_unitree"], atol=1e-12)

    @unittest.skipUnless(POLICY.exists(), "no exported press policy")
    def test_spawn_state(self):
        rest = json.loads((STARTS / "kp25_rest.json").read_text())[0]
        np.testing.assert_allclose(rest["legs"], lh.dog_rest_unifp(), atol=1e-5)
        self.assertAlmostEqual(math.degrees(rest["roll"]), 1.4, places=2)
        self.assertAlmostEqual(math.degrees(rest["pitch"]), -8.1, places=2)
        # Every foot clears the ground at the spawn height, the lowest by 2 mm.
        meta = leg_policy.LegPolicy(POLICY).meta
        q = np.asarray(rest["legs"])[lh.UNITREE_FROM_UNIFP]
        quat = np.array([math.cos(rest["roll"] / 2) * math.cos(rest["pitch"] / 2),
                         math.sin(rest["roll"] / 2) * math.cos(rest["pitch"] / 2),
                         math.cos(rest["roll"] / 2) * math.sin(rest["pitch"] / 2),
                         -math.sin(rest["roll"] / 2) * math.sin(rest["pitch"] / 2)])
        out, height = press_live.feet(q, quat, meta)
        lowest = min(v[2] for v in out.values()) + (rest["base_z"] - height)
        self.assertAlmostEqual(lowest - pl.FOOT_RADIUS_M, 0.002, places=4)
        self.assertAlmostEqual(height, lh.DOG["rest"]["base_m"], places=3)


class TheLibrary(unittest.TestCase):
    def test_entries_and_shares(self):
        spec = importlib.util.spec_from_file_location("build_starts", STARTS / "build_starts.py")
        build = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(build)
        library = json.loads((STARTS / "ft_starts.json").read_text())
        counts = {"sim": 0, "dog": 0, "round4": 0}
        for s in library:
            self.assertEqual(len(s["legs"]), 12)
            self.assertTrue(0.2 < s["base_z"] < 0.35, s["base_z"])
            self.assertLess(abs(s["roll"]), math.radians(3.0))
            self.assertTrue(math.radians(-16.0) < s["pitch"] <= 0.0, s["pitch"])
            if s["source"].startswith("leg_handover"):
                counts["sim"] += 1
                self.assertEqual((len(s["joint_vel"]), len(s["root_vel"])), (12, 6))
            elif s["source"].startswith("dog kp-25 rest path"):
                counts["dog"] += 1
            else:
                counts["round4"] += 1
        for name, share in build.SHARES.items():
            self.assertAlmostEqual(counts[name] / len(library), share, delta=0.03)


class TiltTerm(unittest.TestCase):
    def test_hinge_past_the_threshold(self):
        import torch
        from unifp_train import leg_rewards
        tilts = torch.tensor([0.0, 5.0, 10.0, 25.0, -25.0])
        g = torch.stack((torch.sin(torch.deg2rad(tilts)), torch.zeros(5), -torch.cos(torch.deg2rad(tilts))), dim=-1)
        state = types.SimpleNamespace(projected_gravity_b=g)
        saved = leg_cfg.TILT_EXCESS_RAD
        try:
            leg_cfg.TILT_EXCESS_RAD = math.radians(10.0)
            out = leg_rewards.tilt_excess(state)
        finally:
            leg_cfg.TILT_EXCESS_RAD = saved
        np.testing.assert_allclose(out.numpy(), np.radians([0.0, 0.0, 0.0, 15.0, 15.0]), atol=1e-5)
        self.assertIn("tilt_excess", leg_rewards.TERMS)


class StartGate(unittest.TestCase):
    def test_from_the_runs_library(self):
        settled = np.asarray(leg_cfg.SETTLED_LEG_POS)
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            states = [{"legs": (settled + d).tolist(), "base_z": 0.27, "roll": 0.0, "pitch": math.radians(p)}
                      for d, p in zip(np.linspace(0.0, 0.4, 21), np.linspace(-12.0, -2.0, 21))]
            (tmp / "lib.json").write_text(json.dumps(states))
            (tmp / "run.json").write_text(json.dumps({"leg": {"spawn_states": str(tmp / "lib.json")}}))
            gate = export_policy.start_gate(tmp / "model_1.pt")
            self.assertAlmostEqual(gate["max_off_settled_rad"], float(np.percentile(np.linspace(0.0, 0.4, 21), 95)),
                                   places=3)
            self.assertEqual(gate["pitch_deg"], [-13.0, 1.0])               # the settled stance's 0 included
            (tmp / "run.json").write_text(json.dumps({"leg": {}}))
            self.assertIsNone(export_policy.start_gate(tmp / "model_1.pt"))


if __name__ == "__main__":
    unittest.main()
