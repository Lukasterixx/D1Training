"""The combiner scene's wrist RealSense and its reach-console feed, without Isaac."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from demos.combiner import wrist_camera
from demos.combiner.run_combiner_demo import parse_args
from demos.cup.d1_ui.sim_feed import ARM_JOINTS, FINGER_JOINTS, GO2_MOTOR_ORDER, SimFeedClient
from demos.cup.pick_demo.camera import DEFAULT_MOUNT_PATH

ROOT = Path(__file__).resolve().parents[3]
CALIBRATION = ROOT / "demos/cup/pick_demo/assets/calibration/d435i_238222076237_640x480.json"


class _Tensor(np.ndarray):
    """Enough of a torch tensor for `publish`: `.cpu().numpy()`."""

    def cpu(self):
        return self

    def numpy(self):
        return np.asarray(self)


def _tensor(values):
    return np.asarray(values, dtype=float).view(_Tensor)


class _Namespace:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class ArgumentTests(unittest.TestCase):
    def parse_error(self, argv):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(argv)

    def test_defaults_match_the_cup_pick(self):
        args = parse_args([])
        self.assertEqual((args.camera, args.calibration, args.mount), ("d435", None, None))
        self.assertTrue(args.camera_body)
        self.assertEqual(args.ui_feed_port, 8765)

    def test_both_spellings_hide_the_housing(self):
        self.assertFalse(parse_args(["--no_camera_body"]).camera_body)
        self.assertFalse(parse_args(["--no-camera_body"]).camera_body)

    def test_files_are_checked_before_the_simulator_starts(self):
        self.parse_error(["--calibration", "/nonexistent/calibration.json"])
        self.parse_error(["--mount", "/nonexistent/mount.json"])
        self.parse_error(["--ui_feed_port", "70000"])
        self.assertEqual(parse_args(["--mount", "none"]).mount, "none")
        self.assertEqual(parse_args(["--ui_feed_port", "0"]).ui_feed_port, 0)
        with tempfile.TemporaryDirectory() as directory:
            mount = Path(directory) / "mount.json"
            mount.write_text("{}")
            self.assertEqual(parse_args(["--mount", str(mount)]).mount, str(mount.resolve()))


class ResolveTests(unittest.TestCase):
    def test_preset_and_saved_mount_are_recorded_with_their_provenance(self):
        model, mount, record = wrist_camera.resolve(parse_args([]))
        self.assertEqual((model.name, model.width, model.height), ("d435", 640, 480))
        self.assertIn("datasheet-derived, not a calibration", record["model_source"])
        if DEFAULT_MOUNT_PATH.is_file():
            self.assertEqual(record["mount_file"], str(DEFAULT_MOUNT_PATH.resolve()))
        self.assertTrue(record["body_drawn"])
        json.dumps(record)      # run.json takes it as it is

    def test_mount_none_is_the_placeholder(self):
        _, mount, record = wrist_camera.resolve(parse_args(["--mount", "none", "--no_camera_body"]))
        self.assertEqual(tuple(mount.pos_link6), (-0.055, 0.0, 0.035))
        self.assertIsNone(record["mount_file"])
        self.assertIn("placeholder", record["mount_source"])
        self.assertFalse(record["body_drawn"])

    def test_a_calibration_replaces_the_preset(self):
        model, _, record = wrist_camera.resolve(parse_args(["--calibration", str(CALIBRATION)]))
        self.assertIn(str(CALIBRATION), record["model_source"])
        self.assertEqual((model.width, model.height), (640, 480))


class FeedTests(unittest.TestCase):
    """What the combiner publishes is what the reach console reads."""

    def setUp(self):
        self.assertIsNone(wrist_camera.open_feed(parse_args(["--ui_feed_port", "0"]), None))
        from demos.cup.d1_ui.sim_feed import SimFeed

        # open_feed binds the fixed port a real run uses; the test takes a free one with the same source name.
        self.feed = SimFeed(wrist_camera.FEED_SOURCE, camera=True, port=0, log=lambda *_: None)
        self.client = SimFeedClient(self.feed.url)
        names = ARM_JOINTS + FINGER_JOINTS + GO2_MOTOR_ORDER
        self.q = np.linspace(-0.5, 0.5, len(names))
        self.robot = _Namespace(joint_names=names, data=_Namespace(
            joint_pos=_tensor([self.q]), root_pos_w=_tensor([[1.0, 2.0, 0.31]])))
        rgb = np.zeros((1, 480, 640, 4))
        rgb[..., 1] = 200
        self.wrist = _Namespace(data=_Namespace(output={"rgb": _tensor(rgb)}))
        self.origin = _tensor([1.0, 2.0, 0.2])

    def tearDown(self):
        self.feed.close()

    def test_nothing_is_copied_until_the_console_reads(self):
        wrist_camera.publish(None, self.robot, self.wrist, self.origin, 1.0, "idle")   # no feed: no-op
        wrist_camera.publish(self.feed, self.robot, self.wrist, self.origin, 1.0, "unread")
        self.assertFalse(self.client.poll())
        self.assertIsNone(self.client.fetch_frame())

    def test_joints_status_and_wrist_frame_reach_the_console(self):
        self.client.poll()
        self.client.fetch_frame()           # the console asking is what turns publishing on
        wrist_camera.publish(self.feed, self.robot, self.wrist, self.origin, 2.5, "combiner scene, episode 3")
        self.assertTrue(self.client.poll())
        state = self.client.state
        self.assertEqual(state["source"], "combiner_demo")
        self.assertEqual(state["status"], "combiner scene, episode 3")
        self.assertAlmostEqual(state["base_height_m"], 0.11, places=4)
        np.testing.assert_allclose(state["arm_q_rad"], self.q[:6], atol=1e-5)
        frame = self.client.fetch_frame()
        self.assertIsNotNone(frame)
        rgb, sim_time = frame
        self.assertEqual(rgb.shape, (480, 640, 3))
        self.assertEqual(int(rgb[0, 0, 1]), 200)
        self.assertEqual(sim_time, 2.5)


if __name__ == "__main__":
    unittest.main()
