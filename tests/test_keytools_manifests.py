"""The insertion's frozen lock and key sets on disk are what `unifp_train/keytools.py` draws, so a later edit to the
builders cannot silently change a set that has been scored. Every manifest the code can write is checked, and new sets
must not repeat a lock already scored."""
import json
import unittest

from unifp_train import keytools


BUILDERS = ((keytools.MANIFEST, keytools.manifest), (keytools.TWINS_MANIFEST, keytools.twins_manifest),
            (keytools.S1B_MANIFEST, keytools.s1b_manifest), (keytools.S1C_MANIFEST, keytools.s1c_manifest),
            (keytools.KI7_MANIFEST, keytools.ki7_manifest), (keytools.CHAIN_MANIFEST, keytools.chain_manifest))


class Manifests(unittest.TestCase):
    def test_each_manifest_on_disk_is_its_builder(self):
        for path, build in BUILDERS:
            if not path.exists():
                continue
            with self.subTest(path.name):
                self.assertEqual(keytools.load(path), json.loads(json.dumps(build())))

    def test_the_chain_draws_locks_no_set_before_it_drew(self):
        seen = set()
        for path, build in BUILDERS[:-1]:
            for lk in build()["locks"].values():
                seen |= {(r["door_ahead"], r["axis_side"], r["door_yaw"]) for r in lk["rows"]}
        chain = keytools.chain_manifest()
        for name, lk in chain["locks"].items():
            with self.subTest(name):
                self.assertFalse({(r["door_ahead"], r["axis_side"], r["door_yaw"]) for r in lk["rows"]} & seen)
        self.assertEqual(sorted(chain["sets"]), ["spawn", "test", "validation"])


if __name__ == "__main__":
    unittest.main()
