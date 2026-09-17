"""Tests for the browser console's server side. Numpy only; no DDS, no browser, no simulator.

The page builds its scene graph from `/model.json` by composing, per joint,
translate(xyz) · R(rpy) · Rot(axis, q). A browser cannot run here, so the same
composition is done in numpy from the same JSON and checked against the solver's
FK -- if these agree, the rendered arm and the IK share one model.
"""
import json
import math
import socket
import sys
import tempfile
import threading
import time
import unittest
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import d1_ik
from d1_ui import camera_feed, server, sim_feed
from pick_demo.perception import Detection
from position_only.workspace import rpy_matrix, sample_configs

try:
    import cv2
except ImportError:     # the system Python; the Isaac env has it
    cv2 = None

QUIET = lambda *args, **kwargs: None  # noqa: E731


def _closed_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def _until(predicate, timeout_s=5.0):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        value = predicate()
        if value:
            return value
        time.sleep(0.02)
    return predicate()


# Articulation order as Isaac reports it: legs grouped by part, then the arm. Deliberately not the SDK order.
SIM_NAMES = ([f"{leg}_hip_joint" for leg in ("FL", "FR", "RL", "RR")] + [f"{leg}_thigh_joint" for leg in ("FL", "FR", "RL", "RR")]
             + [f"{leg}_calf_joint" for leg in ("FL", "FR", "RL", "RR")] + [f"Joint{i}" for i in range(1, 7)]
             + ["Joint7_1", "Joint7_2"])


def _rot_axis(axis, q):
    axis = np.asarray(axis, dtype=float); axis /= np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(q) * k + (1 - math.cos(q)) * (k @ k)


def _page_fk(spec, q_by_joint):
    """What the page does: walk the joints, composing fixed origin then joint rotation."""
    frames = {spec["root"]: (np.eye(3), np.asarray(spec.get("mount", [0, 0, 0]), dtype=float))}
    pending = list(spec["joints"])
    while pending:
        for j in list(pending):
            if j["parent"] not in frames:
                continue
            R_p, p_p = frames[j["parent"]]
            p = p_p + R_p @ np.asarray(j["xyz"])
            R = R_p @ rpy_matrix(*j["rpy"])
            if j["type"] == "revolute":
                R = R @ _rot_axis(j["axis"], q_by_joint.get(j["name"], 0.0))
            frames[j["child"]] = (R, p)
            pending.remove(j)
    return frames


class ModelJsonTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = server.build_model()
        cls.joints, _ = d1_ik.load_urdf()

    def test_page_scene_graph_matches_solver_fk(self):
        """The decisive check: the page's transform composition equals workspace.forward."""
        rng = np.random.default_rng(7)
        for q in sample_configs(self.joints, 6, rng):
            frames = _page_fk(self.model["d1"], dict(zip(d1_ik.ARM_JOINTS, q)))
            R, p = frames[self.model["tool"]["body"]]
            page_tool = p + R @ np.asarray(self.model["tool"]["offset"])
            solver_tool, solver_R = d1_ik.tool_pose(self.joints, q)
            np.testing.assert_allclose(page_tool, solver_tool, atol=1e-9)
            np.testing.assert_allclose(R, solver_R, atol=1e-9)

    def test_d1_chain_is_complete_and_mounted(self):
        d1 = self.model["d1"]
        self.assertEqual(d1["arm_joints"], [f"Joint{i}" for i in range(1, 7)])
        self.assertEqual(d1["mount"], [0.0, 0.0, 0.08])
        names = {j["name"] for j in d1["joints"]}
        self.assertTrue({"Joint1", "Joint6", "Joint7_1", "Joint7_2"} <= names)
        for link in ("base_link", "Link1", "Link6", "Link7_1"):
            self.assertTrue(d1["links"][link], f"{link} has no visual")
            self.assertTrue(d1["links"][link][0]["mesh"].startswith("/meshes/d1/"))

    def test_go2_chain_excludes_the_arm_and_covers_twelve_motors(self):
        go2 = self.model["go2"]
        names = {j["name"] for j in go2["joints"]}
        self.assertFalse(any(n.startswith("Joint") for n in names))
        self.assertNotIn("arm_mount_joint", names)
        self.assertTrue(set(go2["motor_order"]) <= names)
        self.assertEqual(len(go2["motor_order"]), 12)
        self.assertTrue(go2["links"]["base_link"][0]["mesh"].endswith("base.dae"))

    def test_servo_signs_and_limits_are_the_measured_ones(self):
        self.assertEqual(self.model["d1"]["servo_sign"], [-1.0, 1.0, 1.0, -1.0, 1.0, 1.0])
        lows, highs = self.model["d1"]["servo_limits_deg"]
        self.assertTrue(all(lo < hi for lo, hi in zip(lows, highs)))

    def test_sphere_sits_above_the_mount(self):
        sph = self.model["sphere"]
        self.assertGreater(sph["min_z"], self.model["d1"]["mount"][2])
        self.assertGreater(sph["center"][2], sph["min_z"])


class TargetValidationTests(unittest.TestCase):
    """validate_target is pure on the model, so it is exercised without an ArmServer."""

    def setUp(self):
        self.srv = server.ArmServer.__new__(server.ArmServer)
        self.srv.model = server.build_model(0.40)

    def test_rejects_below_the_mount_plane(self):
        ok, why = self.srv.validate_target([0.2, 0.0, 0.05])
        self.assertFalse(ok); self.assertIn("mount plane", why)

    def test_rejects_outside_the_sphere(self):
        c = self.srv.model["sphere"]["center"]
        ok, why = self.srv.validate_target([c[0] + 0.9, c[1], c[2] + 0.1])
        self.assertFalse(ok); self.assertIn("sphere", why)

    def test_rejects_garbage(self):
        self.assertFalse(self.srv.validate_target([1.0, float("nan"), 0.3])[0])
        self.assertFalse(self.srv.validate_target([1.0, 2.0])[0])

    def test_accepts_a_point_on_the_upper_sphere(self):
        c = self.srv.model["sphere"]["center"]; r = self.srv.model["sphere"]["radius"]
        ok, why = self.srv.validate_target([c[0] + r * 0.6, c[1], c[2] + r * 0.8])
        self.assertTrue(ok, why)


class SimFeedTests(unittest.TestCase):
    def setUp(self):
        self.feed = sim_feed.SimFeed("test", camera=True, port=0, log=QUIET)
        self.client = sim_feed.SimFeedClient(self.feed.url)

    def tearDown(self):
        self.feed.close()

    def test_health_identifies_a_simulator(self):
        health = self.client.health()
        self.assertTrue(health["sim"])
        self.assertEqual(health["source"], "test")
        self.assertTrue(health["camera"])

    def test_nothing_is_copied_until_someone_reads(self):
        self.assertFalse(self.feed.wants_state())
        self.assertFalse(self.feed.wants_frame())
        self.assertFalse(self.client.poll())            # asks, finds nothing yet
        self.assertIsNone(self.client.fetch_frame())
        self.assertTrue(self.feed.wants_state())
        self.assertTrue(self.feed.wants_frame())

    def test_joints_arrive_as_servo_degrees_with_legs_in_sdk_order(self):
        rng = np.random.default_rng(3)
        positions = rng.uniform(-1.0, 1.0, len(SIM_NAMES))
        self.feed.publish_joints(SIM_NAMES, positions, sim_time_s=1.25, base_height_m=0.085, status="detect")
        self.assertTrue(self.client.poll())
        by_name = dict(zip(SIM_NAMES, positions))
        expected = d1_ik.to_servo_deg([by_name[f"Joint{i}"] for i in range(1, 7)])
        np.testing.assert_allclose(self.client.get_joint_angles(), expected, atol=2e-3)
        state = self.client.state
        np.testing.assert_allclose(state["legs_q_rad"], [by_name[n] for n in sim_feed.GO2_MOTOR_ORDER], atol=1e-5)
        np.testing.assert_allclose(state["finger_m"], [by_name["Joint7_1"], by_name["Joint7_2"]], atol=1e-5)
        self.assertEqual((state["sim_time_s"], state["base_height_m"], state["status"]), (1.25, 0.085, "detect"))
        self.assertFalse(self.client.poll())            # nothing newer
        self.assertLess(self.client.feedback_age_s, 1.0)

    def test_a_robot_without_the_arm_publishes_no_arm(self):
        legs_only = SIM_NAMES[:12]
        self.feed.publish_joints(legs_only, np.zeros(12), sim_time_s=0.0)
        self.assertTrue(self.client.poll())
        self.assertIsNone(self.client.state["arm_q_rad"])
        with self.assertRaises(RuntimeError):
            self.client.get_joint_angles()

    def test_frames_round_trip_once(self):
        rgb = np.random.default_rng(1).integers(0, 255, (48, 64, 3), dtype=np.uint8)
        self.client.fetch_frame()
        self.feed.publish_frame(rgb, sim_time_s=2.0)
        frame, sim_time = self.client.fetch_frame()
        np.testing.assert_array_equal(frame, rgb)
        self.assertEqual(sim_time, 2.0)
        self.assertIsNone(self.client.fetch_frame())    # already seen

    def test_a_restarted_simulator_is_not_mistaken_for_old_frames(self):
        rgb = np.zeros((4, 4, 3), dtype=np.uint8)
        self.client.fetch_frame()
        for _ in range(5):
            self.feed.publish_frame(rgb, sim_time_s=0.0)
        self.assertIsNotNone(self.client.fetch_frame())        # the client has now seen frame 5
        port = self.feed.server.server_address[1]
        self.feed.close()
        self.feed = sim_feed.SimFeed("test", camera=True, port=port, log=QUIET)   # the next run, same port
        self.client.fetch_frame()
        self.feed.publish_frame(rgb + 7, sim_time_s=0.0)       # its frame 1
        frame = self.client.fetch_frame()
        self.assertIsNotNone(frame, "a new run's frame 1 was taken for one already seen")
        self.assertEqual(int(frame[0][0, 0, 0]), 7)

    def test_a_simulation_without_a_camera_serves_no_frames(self):
        feed = sim_feed.SimFeed("teleop", camera=False, port=0, log=QUIET)
        try:
            client = sim_feed.SimFeedClient(feed.url)
            self.assertIsNone(client.fetch_frame())
            self.assertFalse(feed.wants_frame())
            self.assertFalse(client.health()["camera"])
        finally:
            feed.close()

    def test_a_taken_port_turns_the_feed_off_instead_of_raising(self):
        messages = []
        other = sim_feed.SimFeed("second", camera=True, port=self.feed.server.server_address[1], log=messages.append)
        self.assertFalse(other.active)
        self.assertFalse(other.wants_state())
        other.publish_joints(SIM_NAMES, np.zeros(len(SIM_NAMES)), sim_time_s=0.0)   # a no-op, not an error
        self.assertIn("unavailable", messages[0])

    def test_nothing_listening_reads_as_no_simulator(self):
        client = sim_feed.SimFeedClient(f"http://127.0.0.1:{_closed_port()}", timeout_s=0.2)
        self.assertIsNone(client.health())
        self.assertFalse(client.poll(timeout_s=0.05))
        self.assertEqual(client.feedback_age_s, float("inf"))


class ModeDetectionTests(unittest.TestCase):
    def setUp(self):
        self.net = tempfile.TemporaryDirectory()
        self.net_dir = Path(self.net.name)
        self.nobody = f"http://127.0.0.1:{_closed_port()}"

    def tearDown(self):
        self.net.cleanup()

    def test_a_simulator_feed_means_sim_even_on_the_dog(self):
        (self.net_dir / "enP8p1s0").mkdir()
        feed = sim_feed.SimFeed("pick_demo", camera=True, port=0, log=QUIET)
        try:
            mode, why = server.detect_mode("auto", feed.url, "enP8p1s0", self.net_dir)
        finally:
            feed.close()
        self.assertEqual(mode, "sim")
        self.assertIn("pick_demo", why)

    def test_the_arm_interface_without_a_simulator_means_hardware(self):
        (self.net_dir / "enP8p1s0").mkdir()
        self.assertEqual(server.detect_mode("auto", self.nobody, "enP8p1s0", self.net_dir)[0], "hardware")

    def test_neither_means_sim_waiting_for_one(self):
        mode, why = server.detect_mode("auto", self.nobody, "enP8p1s0", self.net_dir)
        self.assertEqual(mode, "sim")
        self.assertIn("waiting", why)

    def test_an_explicit_mode_is_not_second_guessed(self):
        self.assertEqual(server.detect_mode("hardware", self.nobody, "enP8p1s0", self.net_dir)[0], "hardware")


class _StillSource:
    name = "still"

    def __init__(self):
        self.frame = np.full((120, 160, 3), 90, dtype=np.uint8)

    def open(self):
        pass

    def read(self, timeout_s=1.0):
        time.sleep(0.01)
        return self.frame

    def close(self):
        pass


class _BoxDetector:
    names = {0: "person", 41: "cup"}
    weights_name = "fake.pt"
    device = "cpu"

    def detect(self, rgb, labels=("cup",)):
        return [Detection("cup", 0.87, (20.0, 30.0, 100.0, 90.0), None)] if "cup" in labels else []


def _missing_ultralytics():
    raise RuntimeError("ultralytics is not installed here")


@unittest.skipIf(cv2 is None, "needs OpenCV")
class CameraPipelineTests(unittest.TestCase):
    def test_boxes_are_drawn_on_the_frame_they_were_detected_in(self):
        pipeline = camera_feed.CameraPipeline(_StillSource(), _BoxDetector, targets=("cup",), max_fps=60, log=QUIET).start()
        try:
            status = _until(lambda: (lambda s: s if s["detections"] else None)(pipeline.status()))
            self.assertTrue(status and status["available"] and status["detector_ready"])
            self.assertEqual(status["detections"][0]["label"], "cup")
            seq, jpeg = pipeline.wait_jpeg(0, timeout_s=5.0)
            image = cv2.imdecode(np.frombuffer(pipeline.latest_jpeg(), np.uint8), cv2.IMREAD_COLOR)
        finally:
            pipeline.stop()
        self.assertEqual(image.shape, (120, 160, 3))
        # The box's left edge is green (BGR), well away from the grey frame; the middle is untouched.
        b, g, r = (int(v) for v in image[60, 20])
        self.assertGreater(g, 150)
        self.assertLess(abs(int(image[60, 60, 1]) - 90), 12)

    def test_frames_still_flow_without_a_detector(self):
        pipeline = camera_feed.CameraPipeline(_StillSource(), _missing_ultralytics, max_fps=60, log=QUIET).start()
        try:
            self.assertIsNotNone(pipeline.wait_jpeg(0, timeout_s=5.0))
            status = _until(lambda: (lambda s: s if "unavailable" in str(s["detector"]) else None)(pipeline.status()))
        finally:
            pipeline.stop()
        self.assertIn("ultralytics", status["detector"])
        self.assertFalse(status["detector_ready"])
        self.assertTrue(status["available"])

    def test_a_simulator_without_a_camera_says_so(self):
        feed = sim_feed.SimFeed("teleop", camera=False, port=0, log=QUIET)
        pipeline = camera_feed.CameraPipeline(camera_feed.SimFrameSource(sim_feed.SimFeedClient(feed.url)), None,
                                              log=QUIET).start()
        try:
            status = _until(lambda: (lambda s: s if "no wrist camera" in str(s["message"]) else None)(pipeline.status()))
        finally:
            pipeline.stop()
            feed.close()
        self.assertFalse(status["available"])


class SimModeServerTests(unittest.TestCase):
    """The console against a live SimFeed: what the page is sent, what it may do, and the camera endpoints."""

    def setUp(self):
        self.feed = sim_feed.SimFeed("pick_demo", camera=True, port=0, log=QUIET)
        self.arm = server.ArmServer(iface="none", legs=True, sphere_radius_m=0.40, step_deg=5.0, log_path=None,
                                    mode="sim", mode_reason="test", client=sim_feed.SimFeedClient(self.feed.url))
        self.arm._log = QUIET

    def tearDown(self):
        self.feed.close()

    def _publish_until_seen(self, positions):
        def seen():
            if self.feed.wants_state():
                self.feed.publish_joints(SIM_NAMES, positions, sim_time_s=4.0, base_height_m=0.085, status="moving")
            snap = self.arm._snapshot()
            return snap if snap["connected"] and snap["go2_q_rad"] is not None else None
        return _until(seen)

    def test_before_the_simulator_publishes_the_page_is_told_it_is_waiting(self):
        snap = self.arm._snapshot()
        self.assertEqual(snap["mode"], "sim")
        self.assertFalse(snap["connected"])
        self.assertIsNone(snap["servo_deg"])
        self.assertIsNone(snap["tool_m"])
        self.assertFalse(self.arm.preview([0.3, 0.0, 0.4])["ok"])
        json.dumps(snap)

    def test_the_page_follows_the_simulator(self):
        by_name = {name: 0.0 for name in SIM_NAMES}
        by_name.update(Joint2=0.6, Joint3=-0.9, Joint7_1=0.012, Joint7_2=-0.012, FR_thigh_joint=1.1, RL_calf_joint=-2.5)
        snap = self._publish_until_seen([by_name[n] for n in SIM_NAMES])
        self.assertIsNotNone(snap, "the server never picked up the simulator's state")
        np.testing.assert_allclose(snap["q_rad"], [by_name[f"Joint{i}"] for i in range(1, 7)], atol=1e-4)
        tool, _ = d1_ik.tool_pose(d1_ik.load_urdf()[0], np.asarray(snap["q_rad"]))
        np.testing.assert_allclose(snap["tool_m"], tool, atol=2e-4)
        self.assertEqual(snap["finger_m"], [0.012, -0.012])
        self.assertEqual(snap["go2_q_rad"][sim_feed.GO2_MOTOR_ORDER.index("FR_thigh_joint")], 1.1)
        self.assertEqual(snap["go2_q_rad"][sim_feed.GO2_MOTOR_ORDER.index("RL_calf_joint")], -2.5)
        self.assertEqual(snap["base_height_m"], 0.085)
        self.assertEqual(snap["sim"], {"source": "pick_demo", "sim_time_s": 4.0, "status": "moving"})
        self.assertIsNone(snap["gripper_units"])
        json.dumps(snap)

    def test_nothing_that_moves_an_arm_is_allowed(self):
        self.assertIsNotNone(self._publish_until_seen(np.zeros(len(SIM_NAMES))))
        for name, result in (("move", self.arm.move_to([0.3, 0.0, 0.4])), ("park", self.arm.park()),
                             ("release", self.arm.release(force=True)), ("live", self.arm.set_live(True))):
            self.assertFalse(result[0], name)
            self.assertIn("sim mode", result[1])
        self.assertFalse(self.arm.live.is_set())
        self.assertIsNone(self.arm.state["busy"])

    @unittest.skipIf(cv2 is None, "needs OpenCV")
    def test_camera_endpoints_serve_the_annotated_frame(self):
        camera = camera_feed.CameraPipeline(_StillSource(), _BoxDetector, max_fps=60, log=QUIET).start()
        self.arm.camera = camera
        previous = server.Handler.arm
        server.Handler.arm = self.arm
        httpd = ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
        httpd.daemon_threads = True
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        base = f"http://127.0.0.1:{httpd.server_address[1]}"
        try:
            self.assertIsNotNone(camera.wait_jpeg(0, timeout_s=5.0))
            with urllib.request.urlopen(base + "/camera.jpg", timeout=5) as response:
                self.assertEqual(response.headers["Content-Type"], "image/jpeg")
                self.assertEqual(response.read(2), b"\xff\xd8")
            with urllib.request.urlopen(base + "/camera.mjpg", timeout=5) as response:
                self.assertIn("multipart/x-mixed-replace", response.headers["Content-Type"])
                head = response.read(64)
            self.assertTrue(head.startswith(b"--frame\r\nContent-Type: image/jpeg"))
            request = urllib.request.Request(base + "/arm", data=b'{"live": true}', method="POST",
                                             headers={"Content-Type": "application/json"})
            with self.assertRaises(urllib.error.HTTPError) as refused:
                urllib.request.urlopen(request, timeout=5)
            self.assertEqual(refused.exception.code, 409)
            self.assertIn(camera.status()["source"], "still")
        finally:
            httpd.shutdown()
            httpd.server_close()
            camera.stop()
            server.Handler.arm = previous


if __name__ == "__main__":
    unittest.main()
