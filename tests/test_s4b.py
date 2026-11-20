"""S4b's pieces that need no simulator: the dog stance the base environment can start from, and the frozen T1 sets.

The stance must be the one `demos/unifp/run_plan_probe.py --real_stance` hands over from (F-259's evaluations), and the
manifests on disk must be what their builders draw, so a later edit to `mech_cfg` cannot silently change a frozen set.
"""
import ast
import json
import math
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

try:
    import torch  # noqa: F401  (mech_cfg's draws use it)
    from unifp_train import mech_cfg, task_cfg
except Exception:  # pragma: no cover - the system Python has no torch or Isaac
    mech_cfg = task_cfg = None


def _probe_constants() -> dict:
    """run_plan_probe.py's REAL_* assignments, read without importing it (it starts Isaac)."""
    tree = ast.parse((ROOT / "demos/unifp/run_plan_probe.py").read_text())
    out = {}
    for node in tree.body:
        if isinstance(node, ast.Assign):
            names = [t.id for t in node.targets if isinstance(t, ast.Name)]
            names += [e.id for t in node.targets if isinstance(t, ast.Tuple) for e in t.elts]
            if any(n.startswith("REAL_") for n in names):
                value = ast.literal_eval(node.value)
                if len(names) == 1:
                    out[names[0]] = value
                else:
                    out.update(dict(zip(names, value)))
    return out


@unittest.skipIf(task_cfg is None, "needs the Isaac environment's torch")
class DogStanceTest(unittest.TestCase):
    def test_the_stance_is_the_plan_probes(self):
        real = _probe_constants()
        self.assertEqual(tuple(real["REAL_LEG_OFFSETS"]), task_cfg.DOG_STANCE_LEG_OFFSETS)
        self.assertEqual(real["REAL_BASE_M"], task_cfg.DOG_STANCE_BASE_M)
        self.assertEqual(real["REAL_PITCH_DEG"], task_cfg.DOG_STANCE_PITCH_DEG)
        self.assertEqual(real["REAL_ROLL_DEG"], task_cfg.DOG_STANCE_ROLL_DEG)

    def test_the_quaternion_is_nose_up_by_3_7_deg(self):
        w, x, y, z = task_cfg.dog_stance_quat()
        self.assertAlmostEqual(w * w + x * x + y * y + z * z, 1.0, places=12)
        # Body x (the nose) in the world: z component positive means nose up.
        nose_z = 2.0 * (x * z - w * y)
        self.assertAlmostEqual(math.degrees(math.asin(nose_z)), 3.7, places=2)
        # Body y (the left side) in the world: tilted out of the horizontal by the 0.4 deg roll.
        left_z = 2.0 * (y * z + w * x)
        self.assertAlmostEqual(abs(math.degrees(math.asin(left_z))), 0.4, places=2)


@unittest.skipIf(mech_cfg is None, "needs the Isaac environment's torch")
class FrozenSetsTest(unittest.TestCase):
    def test_the_manifests_on_disk_are_their_builders(self):
        for path, build in ((mech_cfg.S4_MANIFEST, mech_cfg.s4_manifest), (mech_cfg.S4B_MANIFEST, mech_cfg.s4b_manifest)):
            on_disk = json.loads((ROOT / path).read_text())
            self.assertEqual(on_disk["sets"], json.loads(json.dumps(build()["sets"])), path)

    def test_s4b_draws_new_placements_with_s4s_conditions(self):
        v2 = json.loads((ROOT / mech_cfg.S4_MANIFEST).read_text())["sets"]
        v3 = json.loads((ROOT / mech_cfg.S4B_MANIFEST).read_text())["sets"]
        for name in ("validation", "test"):
            self.assertEqual(v2[name]["conditions"], v3[name]["conditions"])
            self.assertEqual(len(v2[name]["placements"]), len(v3[name]["placements"]))
            self.assertFalse(set(map(tuple, v2[name]["placements"])) & set(map(tuple, v3[name]["placements"])))
        self.assertEqual(mech_cfg.s4_set("test", mech_cfg.S4B_MANIFEST)[0][0], tuple(v3["test"]["placements"][0]))


if __name__ == "__main__":
    unittest.main()
