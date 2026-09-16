"""Frozen manifests and the G1a metric rules (CPU only, no simulator).

These test the places where a bug would quietly flatter a result: a truncated episode counted as a
success, a failed episode dropped from the denominator, a missing tail filled with the last good
value, or a dwell that is not actually continuous.
"""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from position_only import manifest as mf
from position_only.evaluate import _episode_record, longest_run, summarise, wilson

STEP_DT = 0.02  # 50 Hz
DWELL_STEPS = 50  # 1 s
FINAL_STEPS = 100  # 2 s
TOTAL_STEPS = 500  # 10 s
RADIUS = 0.05


def trace(errors, height=0.27, tilt=2.0, at_limit=0.0, arm_at_limit=1.0, leg_sat=0.0,
          leg_torque=5.0, arm_reaction=1.1):
    """A (steps, 8) trace with the given error series and constant everything else."""
    errors = np.asarray(errors, dtype=float)
    columns = [errors, np.full_like(errors, height), np.full_like(errors, tilt),
               np.full_like(errors, at_limit), np.full_like(errors, arm_at_limit),
               np.full_like(errors, leg_sat), np.full_like(errors, leg_torque),
               np.full_like(errors, arm_reaction)]
    return np.stack(columns, axis=-1)


def record(errors, ended_step=-1, fell=False, timed_out=True, index=0):
    return _episode_record(index=index, target=(0.4, 0.0, 0.55), trace=trace(errors),
                           ended_step=ended_step, total_steps=TOTAL_STEPS, step_dt=STEP_DT,
                           fell=fell, timed_out=timed_out, radius=RADIUS,
                           dwell_steps=DWELL_STEPS, final_steps=FINAL_STEPS)


class LongestRun(unittest.TestCase):
    def test_counts_only_consecutive_values(self):
        self.assertEqual(longest_run([]), 0)
        self.assertEqual(longest_run([False, False]), 0)
        self.assertEqual(longest_run([True] * 5), 5)
        # Three separate runs of 2 are not a run of 6.
        self.assertEqual(longest_run([True, True, False, True, True, False, True, True]), 2)
        self.assertEqual(longest_run([False, True, True, True, False]), 3)


class Wilson(unittest.TestCase):
    def test_interval_is_usable_at_the_extremes(self):
        low, high = wilson(0, 100)
        self.assertEqual(low, 0.0)
        self.assertGreater(high, 0.0)  # not a degenerate [0, 0]
        low, high = wilson(100, 100)
        self.assertLess(low, 1.0)
        self.assertAlmostEqual(high, 1.0)
        self.assertLessEqual(high, 1.0)  # never reports above 100%

    def test_interval_brackets_the_estimate(self):
        low, high = wilson(33, 256)
        self.assertLess(low, 33 / 256)
        self.assertGreater(high, 33 / 256)


class SuccessRule(unittest.TestCase):
    def test_one_continuous_second_inside_the_radius_succeeds(self):
        errors = np.full(TOTAL_STEPS, 0.20)
        errors[200:200 + DWELL_STEPS] = 0.01
        self.assertTrue(record(errors)["success"])

    def test_dwell_one_step_short_fails(self):
        errors = np.full(TOTAL_STEPS, 0.20)
        errors[200:200 + DWELL_STEPS - 1] = 0.01
        result = record(errors)
        self.assertFalse(result["success"])
        self.assertAlmostEqual(result["max_dwell_s"], (DWELL_STEPS - 1) * STEP_DT)

    def test_dwell_must_be_continuous_not_cumulative(self):
        # 100 steps inside the radius in total, but never more than 25 in a row.
        errors = np.full(TOTAL_STEPS, 0.20)
        for start in (100, 200, 300, 400):
            errors[start:start + 25] = 0.01
        self.assertFalse(record(errors)["success"])

    def test_exactly_on_the_radius_counts_as_inside(self):
        errors = np.full(TOTAL_STEPS, 0.20)
        errors[100:100 + DWELL_STEPS] = RADIUS
        self.assertTrue(record(errors)["success"])

    def test_a_terminated_episode_cannot_succeed_however_close_it_got(self):
        errors = np.full(300, 0.001)  # parked on the target the whole time
        result = record(errors, ended_step=300, fell=True, timed_out=False)
        self.assertFalse(result["success"])
        self.assertTrue(result["fell"])
        self.assertTrue(result["truncated"])
        self.assertFalse(result["survived"])


class TruncationRule(unittest.TestCase):
    def test_a_truncated_trace_is_not_padded(self):
        result = record(np.full(137, 0.2), ended_step=137, fell=True, timed_out=False)
        self.assertEqual(result["recorded_steps"], 137)
        self.assertAlmostEqual(result["recorded_s"], 137 * STEP_DT)
        self.assertEqual(result["ended_step"], 137)

    def test_a_short_trace_reports_no_final_window(self):
        # Fewer samples than the 2 s window: there is no final-2 s figure to report.
        result = record(np.full(40, 0.2), ended_step=40, fell=True, timed_out=False)
        self.assertIsNone(result["final_2s_error"])
        self.assertIsNotNone(result["transient_error"])

    def test_a_full_episode_splits_transient_from_the_final_two_seconds(self):
        errors = np.concatenate([np.full(TOTAL_STEPS - FINAL_STEPS, 0.30), np.full(FINAL_STEPS, 0.02)])
        result = record(errors)
        self.assertAlmostEqual(result["transient_error"]["mean_m"], 0.30, places=6)
        self.assertAlmostEqual(result["final_2s_error"]["mean_m"], 0.02, places=6)
        # The pooled RMS is dominated by the transient, so it must not look like the final value.
        self.assertGreater(result["rms_error_m"], 0.2)


class EpisodeMetrics(unittest.TestCase):
    def test_time_to_reach_is_the_first_entry_not_the_dwell_start(self):
        errors = np.full(TOTAL_STEPS, 0.20)
        errors[60] = 0.01                       # one early touch
        errors[300:300 + DWELL_STEPS] = 0.01    # the qualifying dwell
        result = record(errors)
        self.assertAlmostEqual(result["time_to_reach_s"], 60 * STEP_DT)
        self.assertTrue(result["success"])

    def test_time_to_reach_is_none_when_never_reached(self):
        self.assertIsNone(record(np.full(TOTAL_STEPS, 0.2))["time_to_reach_s"])

    def test_rms_and_p95_come_from_the_error_series(self):
        errors = np.concatenate([np.full(400, 0.10), np.full(100, 0.30)])
        result = record(errors)
        self.assertAlmostEqual(result["rms_error_m"], float(np.sqrt(np.mean(errors ** 2))), places=9)
        self.assertAlmostEqual(result["p95_error_m"], float(np.percentile(errors, 95)), places=9)


class TorqueReporting(unittest.TestCase):
    def test_commanded_and_measured_arm_torque_are_kept_apart(self):
        # The arm's commanded estimate saturates while standing still; the reaction torque does not.
        result = record(np.full(TOTAL_STEPS, 0.2))
        self.assertAlmostEqual(result["arm_commanded_effort_at_limit_frac"], 1.0)
        self.assertAlmostEqual(result["peak_arm_joint_torque_nm"], 1.1)
        summary = summarise([result], mf.build("development", episodes=1), controller="test")
        self.assertIn("not a PhysX measurement", summary["saturation"]["note"])
        self.assertAlmostEqual(summary["saturation"]["arm_commanded_effort_at_limit_frac"], 1.0)
        self.assertAlmostEqual(summary["saturation"]["peak_arm_joint_torque_nm"], 1.1)


class Aggregation(unittest.TestCase):
    def manifest(self):
        return mf.build("development", episodes=10)

    def test_failed_episodes_stay_in_the_success_denominator(self):
        good = np.full(TOTAL_STEPS, 0.20)
        good[100:100 + DWELL_STEPS] = 0.01
        records = [record(good, index=i) for i in range(6)]
        records += [record(np.full(50, 0.01), index=6 + i, ended_step=50, fell=True, timed_out=False)
                    for i in range(4)]
        result = summarise(records, self.manifest(), controller="test")
        self.assertEqual(result["episodes"], 10)
        self.assertEqual(result["success"]["count"], 6)
        self.assertAlmostEqual(result["success"]["rate"], 0.6)   # not 6/6
        self.assertAlmostEqual(result["falls"]["rate"], 0.4)
        self.assertEqual(result["truncated"]["count"], 4)

    def test_surviving_only_error_is_reported_separately_from_the_headline(self):
        # Survivors track well; the failures were far away when they died.
        good = np.full(TOTAL_STEPS, 0.02)
        bad = np.full(100, 0.40)
        records = [record(good, index=i) for i in range(5)]
        records += [record(bad, index=5 + i, ended_step=100, fell=True, timed_out=False) for i in range(5)]
        result = summarise(records, self.manifest(), controller="test")
        headline = result["error_all_episodes"]["rms_m"]
        survivors = result["error_surviving_episodes_only"]["rms_m"]
        self.assertAlmostEqual(survivors, 0.02, places=6)
        self.assertGreater(headline, survivors)  # the headline must not hide the failures

    def test_g1a_thresholds(self):
        good = np.full(TOTAL_STEPS, 0.20)
        good[100:100 + DWELL_STEPS] = 0.01
        # 90% success, no falls: passes.
        records = [record(good, index=i) for i in range(9)] + [record(np.full(TOTAL_STEPS, 0.3), index=9)]
        result = summarise(records, self.manifest(), controller="test")
        self.assertTrue(result["g1a"]["passed"])
        # Same success rate, but one of them fell: 10% falls breaks the ≤1% limit.
        records = [record(good, index=i) for i in range(9)]
        records += [record(np.full(80, 0.3), index=9, ended_step=80, fell=True, timed_out=False)]
        result = summarise(records, self.manifest(), controller="test")
        self.assertAlmostEqual(result["success"]["rate"], 0.9)
        self.assertFalse(result["g1a"]["passed"])
        self.assertTrue(result["g1a"]["success_rate_ok"])
        self.assertFalse(result["g1a"]["fall_rate_ok"])

    def test_condition_mismatches_are_reported(self):
        manifest = self.manifest()
        result = summarise([record(np.full(TOTAL_STEPS, 0.2))], manifest, controller="test",
                           conditions={"spawn_height_m": 0.42, "robustness": "unitree"})
        self.assertEqual(len(result["condition_mismatches"]), 2)
        self.assertTrue(any("spawn_height_m" in line for line in result["condition_mismatches"]))
        self.assertTrue(any("robustness" in line for line in result["condition_mismatches"]))

    def test_matching_conditions_report_nothing(self):
        manifest = self.manifest()
        result = summarise([record(np.full(TOTAL_STEPS, 0.2))], manifest, controller="test",
                           conditions=dict(manifest["conditions"]))
        self.assertEqual(result["condition_mismatches"], [])


class Manifests(unittest.TestCase):
    def test_is_deterministic_in_its_arguments(self):
        a, b = mf.build("test", episodes=100), mf.build("test", episodes=100)
        self.assertEqual(a["content_sha256"], b["content_sha256"])
        self.assertEqual(a["episode_count"], 100)
        self.assertEqual(len(a["episodes"]), 100)

    def test_the_episode_count_is_not_shadowed_by_the_episode_list(self):
        manifest = mf.build("test", episodes=7)
        self.assertIsInstance(manifest["episode_count"], int)
        self.assertIsInstance(manifest["episodes"], list)

    def test_roles_draw_different_episodes(self):
        hashes = {role: mf.build(role, episodes=100)["content_sha256"] for role in mf.ROLES}
        self.assertEqual(len(set(hashes.values())), 3)
        dev = mf.targets(mf.build("development", episodes=100))
        test = mf.targets(mf.build("test", episodes=100))
        self.assertFalse(np.allclose(dev, test))

    def test_targets_lie_inside_the_declared_box(self):
        manifest = mf.build("test", episodes=200)
        points = mf.targets(manifest)
        for axis, (low, high) in enumerate(manifest["conditions"]["target_box_env_frame_m"]):
            self.assertGreaterEqual(points[:, axis].min(), low)
            self.assertLessEqual(points[:, axis].max(), high)

    def test_changing_the_box_changes_the_hash(self):
        a = mf.build("test", episodes=10)
        b = mf.build("test", episodes=10, ranges=((0.2, 0.3), (-0.05, 0.05), (0.6, 0.7)))
        self.assertNotEqual(a["content_sha256"], b["content_sha256"])

    def test_changing_a_condition_changes_the_hash(self):
        a = mf.build("test", episodes=10)
        b = mf.build("test", episodes=10, robustness="unitree")
        self.assertNotEqual(a["content_sha256"], b["content_sha256"])

    def test_round_trip_through_a_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = mf.write(mf.build("validation", episodes=25), Path(tmp) / "validation.json")
            loaded = mf.load(path)
            self.assertEqual(loaded["role"], "validation")
            self.assertEqual(len(loaded["episodes"]), 25)

    def test_an_edited_manifest_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "test.json"
            mf.write(mf.build("test", episodes=10), path)
            edited = json.loads(path.read_text())
            edited["episodes"][0]["target_env_frame_m"][2] = 0.76  # move one target onto the resting tip
            path.write_text(json.dumps(edited))
            with self.assertRaises(ValueError) as caught:
                mf.load(path)
            self.assertIn("content_sha256", str(caught.exception))

    def test_unknown_role_is_rejected(self):
        with self.assertRaises(ValueError):
            mf.build("final", episodes=10)


if __name__ == "__main__":
    unittest.main()
