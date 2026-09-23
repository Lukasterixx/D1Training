"""Tests for the browser console's server side. Numpy only; no DDS, no browser, no simulator.

The page builds its scene graph from `/model.json` by composing, per joint,
translate(xyz) · R(rpy) · Rot(axis, q). A browser cannot run here, so the same
composition is done in numpy from the same JSON and checked against the solver's
FK -- if these agree, the rendered arm and the IK share one model.
"""
import json
import math
import shutil
import socket
import sys
import tempfile
import threading
import time
import types
import unittest
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))   # the repository root

import d1_ik
from demos.cup.d1_ui import camera_feed, server, sim_feed
from demos.cup.pick_demo.perception import Detection
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


class _TagSource(_StillSource):
    """The combiner door's tag, 96 px across its black square, square-on to a camera that knows its K."""
    name = "tag"

    def __init__(self, with_k=True):
        from demos.combiner.apriltag import tag_image

        self.frame = np.full((480, 640, 3), 128, dtype=np.uint8)
        self.frame[150:270, 200:320] = tag_image(0, 12)[..., None]
        self.with_k = with_k

    def intrinsic_matrix(self):
        return np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]]) if self.with_k else None


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

    def test_apriltags_are_outlined_and_ranged_when_the_intrinsics_are_known(self):
        pipeline = camera_feed.CameraPipeline(_TagSource(), None, max_fps=60, log=QUIET, tag_size_m=0.06).start()
        try:
            status = _until(lambda: (lambda s: s if s["tags"] else None)(pipeline.status()))
            image = cv2.imdecode(np.frombuffer(pipeline.latest_jpeg(), np.uint8), cv2.IMREAD_COLOR)
        finally:
            pipeline.stop()
        self.assertTrue(status["tags_enabled"])
        tag = status["tags"][0]
        self.assertEqual(tag["id"], 0)
        # 600 px focal length, 60 mm across 96 px: 0.375 m square-on, a little more off the optical axis.
        self.assertTrue(0.36 < tag["range_m"] < 0.40, tag)
        self.assertIn("with range", status["tag_detector"])
        # The outline is magenta (BGR 255, 0, 255) on the black square's left edge, where the frame is grey.
        b, g, r = (int(v) for v in image[210, 212])
        self.assertGreater(b, 180)
        self.assertGreater(r, 180)
        self.assertLess(g, 90)

    def test_apriltags_without_intrinsics_are_outlined_without_a_range(self):
        pipeline = camera_feed.CameraPipeline(_TagSource(with_k=False), None, max_fps=60, log=QUIET,
                                              tag_size_m=0.06).start()
        try:
            status = _until(lambda: (lambda s: s if s["tags"] else None)(pipeline.status()))
        finally:
            pipeline.stop()
        self.assertEqual(status["tags"][0]["id"], 0)
        self.assertIsNone(status["tags"][0]["range_m"])
        self.assertIn("no range", status["tag_detector"])

    def test_tags_are_off_unless_asked_for(self):
        pipeline = camera_feed.CameraPipeline(_TagSource(), None, max_fps=60, log=QUIET).start()
        try:
            self.assertIsNotNone(pipeline.wait_jpeg(0, timeout_s=5.0))
            status = pipeline.status()
        finally:
            pipeline.stop()
        self.assertFalse(status["tags_enabled"])
        self.assertEqual(status["tags"], [])

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


class MountFileTests(unittest.TestCase):
    """The wrist mount as six numbers and as a file. No arm, no browser, no Isaac."""

    def setUp(self):
        from demos.cup.pick_demo import camera

        self.camera = camera
        self.dir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def test_the_placeholder_mount_reads_as_six_interpretable_numbers(self):
        """The editor seeds itself from whatever mount is loaded, so that had better be legible."""
        xyz, rpy = self.camera.mount_as_xyz_rpy(self.camera.WristMount())
        np.testing.assert_allclose(xyz, (-0.055, 0.0, 0.035), atol=1e-9)
        np.testing.assert_allclose(rpy, (-20.0, 0.0, -90.0), atol=1e-9)

    def test_xyz_rpy_round_trips_through_the_matrix(self):
        """A number typed in the console, stored, and used by the solver must mean one thing."""
        for rpy in ((-20.0, 0.0, -90.0), (12.5, -7.25, 143.0), (0.0, 0.0, 0.0), (179.0, 31.0, -6.0)):
            with self.subTest(rpy=rpy):
                mount = self.camera.mount_from_xyz_rpy((0.01, -0.02, 0.03), rpy)
                back_xyz, back_rpy = self.camera.mount_as_xyz_rpy(mount)
                np.testing.assert_allclose(back_xyz, (0.01, -0.02, 0.03), atol=1e-12)
                np.testing.assert_allclose(back_rpy, rpy, atol=1e-9)

    def test_the_rpy_convention_is_the_one_the_urdf_and_the_page_use(self):
        """`rpy_matrix_zyx` has to agree with the solver's, or the console and the arm disagree."""
        for rpy in ((0.3, -0.2, 1.1), (0.0, 0.0, 0.0), (-1.4, 0.9, 0.2)):
            with self.subTest(rpy=rpy):
                np.testing.assert_allclose(self.camera.rpy_matrix_zyx(*rpy), rpy_matrix(*rpy), atol=1e-12)

    def test_a_saved_mount_loads_back_as_the_same_transform(self):
        path = Path(self.dir) / "m.json"
        mount = self.camera.mount_from_xyz_rpy((-0.05, 0.004, 0.036), (-18.0, 2.0, -88.0))
        self.camera.save_mount(path, mount, source="unit test", method="typed in")
        loaded = self.camera.load_mount(path)
        np.testing.assert_allclose(loaded.pose, mount.pose, atol=1e-9)

    def test_a_saved_mount_says_it_was_not_measured(self):
        """The console can only align by eye. A file that does not say so would be read as a calibration."""
        path = Path(self.dir) / "m.json"
        self.camera.save_mount(path, self.camera.WristMount(), source="console", method="aligned by eye")
        data = json.loads(path.read_text())
        self.assertFalse(data["measured"])
        self.assertIn("aligned by eye", data["method"])
        self.assertIn("not measured", self.camera.load_mount(path).source)

    def test_a_file_edited_in_one_place_only_is_refused(self):
        """Both parameterisations are stored; if they come apart the file is wrong, not merely stale."""
        path = Path(self.dir) / "m.json"
        self.camera.save_mount(path, self.camera.WristMount(), source="test")
        data = json.loads(path.read_text())
        data["xyz_m"] = [0.5, 0.5, 0.5]                 # moved here but not in pose_link6
        path.write_text(json.dumps(data))
        with self.assertRaises(ValueError) as bad:
            self.camera.load_mount(path)
        self.assertIn("disagree", str(bad.exception))

    def test_a_foreign_file_is_refused_by_schema(self):
        path = Path(self.dir) / "m.json"
        path.write_text(json.dumps({"schema": "something/else", "pose_link6": np.eye(4).tolist()}))
        with self.assertRaises(ValueError):
            self.camera.load_mount(path)


class MountEditorTests(unittest.TestCase):
    """The server side of the console's mount editor: what it accepts, refuses and writes."""

    def setUp(self):
        from demos.cup.pick_demo import camera

        self.camera = camera
        # A client that answers nothing: the mount editor touches only pick_cfg, and building a real
        # D1Client here would open a DDS participant for no reason.
        self.arm = server.ArmServer(iface="none", legs=True, sphere_radius_m=0.40, step_deg=5.0, log_path=None,
                                    mode="sim", mode_reason="test",
                                    client=sim_feed.SimFeedClient("http://127.0.0.1:1"))
        self.arm._log = QUIET
        # Pinned to the placeholder rather than whatever mount happens to be saved in the repo, so these
        # tests say the same thing on a machine that has one and on a machine that does not.
        mount = camera.WristMount()
        self.perception = types.SimpleNamespace(mount=mount)
        self.arm.pick_cfg = {"mount": mount, "perception": self.perception,
                             "mount_source": "assumed", "camera_source": "test"}
        self.arm.mount, self.arm.mount_source, self.arm.mount_file = mount, "assumed", None

    def test_without_a_pick_the_editor_still_works_but_drives_nothing(self):
        """A workstation has no arm and no camera, and is exactly where the simulator's mount is set."""
        self.arm.pick_cfg = None
        status = self.arm.mount_status()
        self.assertTrue(status["available"])
        self.assertFalse(status["drives_perception"])
        self.assertIn("edits a file only", status["note"])
        ok, why = self.arm.set_mount([-0.05, 0.0, 0.04], [-18.0, 0.0, -90.0])
        self.assertTrue(ok, why)
        np.testing.assert_allclose(self.arm.mount_status()["xyz_m"], (-0.05, 0.0, 0.04), atol=1e-6)

    def test_the_status_seeds_the_editor_from_the_loaded_mount(self):
        status = self.arm.mount_status()
        self.assertTrue(status["available"])
        np.testing.assert_allclose(status["xyz_m"], (-0.055, 0.0, 0.035), atol=1e-6)
        np.testing.assert_allclose(status["rpy_deg"], (-20.0, 0.0, -90.0), atol=1e-4)
        self.assertEqual(len(status["mesh_to_optical"]), 4)
        self.assertIn("not a hand-eye calibration", status["note"])

    def test_setting_the_mount_moves_the_frame_perception_uses(self):
        """The editor is only useful if the next frame is deprojected through the new mount."""
        ok, why = self.arm.set_mount([-0.05, 0.002, 0.04], [-18.0, 1.0, -88.0])
        self.assertTrue(ok, why)
        expected = self.camera.mount_from_xyz_rpy([-0.05, 0.002, 0.04], [-18.0, 1.0, -88.0])
        np.testing.assert_allclose(self.perception.mount.pose, expected.pose, atol=1e-9)
        self.assertIs(self.perception.mount, self.arm.pick_cfg["mount"])
        self.assertIs(self.perception.mount, self.arm.mount)
        self.assertTrue(self.arm.mount_status()["drives_perception"])

    def test_a_mount_that_buries_the_camera_in_the_wrist_is_reported(self):
        """The editor has to say when the case is inside Link6, or it is a way to draw a bad bracket."""
        self.assertFalse(self.arm.mount_status()["clearance"]["intersects_shell"])
        ok, why = self.arm.set_mount([-0.048, 0.006, 0.042], [-16.0, 2.0, -87.0])
        self.assertTrue(ok, why)
        clearance = self.arm.mount_status()["clearance"]
        self.assertTrue(clearance["intersects_shell"], clearance)
        self.assertGreater(max(clearance["overlap_m"]), 0.0)

    def test_nonsense_is_refused_rather_than_stored(self):
        before = self.arm.pick_cfg["mount"].pose.copy()
        for xyz, rpy in (([0.0, 0.0], [0, 0, 0]),                 # too few
                         (["a", 0.0, 0.0], [0, 0, 0]),            # not numbers
                         ([float("nan"), 0.0, 0.0], [0, 0, 0]),   # not finite
                         ([5.0, 0.0, 0.0], [0, 0, 0])):           # metres confused for millimetres
            with self.subTest(xyz=xyz):
                ok, why = self.arm.set_mount(xyz, rpy)
                self.assertFalse(ok)
                self.assertTrue(why)
        np.testing.assert_allclose(self.arm.pick_cfg["mount"].pose, before)

    def test_the_mount_cannot_move_under_a_running_pick(self):
        self.arm.pick_state["running"] = True
        ok, why = self.arm.set_mount([-0.05, 0.0, 0.04], [-18.0, 0.0, -90.0])
        self.assertFalse(ok)
        self.assertIn("pick is running", why)
        self.assertFalse(self.arm.mount_status()["editable"])

    def test_saving_writes_a_loadable_file_that_admits_what_it_is(self):
        root = Path(server.PICK_ASSETS) / "mounts"
        name = "unittest_tmp_mount"
        path = root / f"{name}.json"
        try:
            self.arm.set_mount([-0.05, 0.002, 0.04], [-18.0, 1.0, -88.0])
            ok, where = self.arm.save_mount(name)
            self.assertTrue(ok, where)
            self.assertEqual(Path(where), path)
            loaded = self.camera.load_mount(path)
            np.testing.assert_allclose(loaded.pose, self.arm.pick_cfg["mount"].pose, atol=1e-9)
            self.assertFalse(json.loads(path.read_text())["measured"])
            self.assertIn(name, self.arm.mount_status()["saved_to"])
        finally:
            path.unlink(missing_ok=True)

    def test_a_save_name_cannot_escape_the_mounts_directory(self):
        root = Path(server.PICK_ASSETS) / "mounts"
        ok, where = self.arm.save_mount("../../etc/passwd")
        try:
            self.assertTrue(ok, where)
            self.assertEqual(Path(where).parent, root)
        finally:
            Path(where).unlink(missing_ok=True)



class MountDefaultTests(unittest.TestCase):
    """Which mount a launch uses. One saved file is meant to be enough, with nothing to pass."""

    def setUp(self):
        from demos.cup.pick_demo import camera

        self.camera = camera
        self.dir = Path(tempfile.mkdtemp())
        self.saved = self.dir / "wrist_mount.json"
        self.previous = camera.DEFAULT_MOUNT_PATH
        camera.DEFAULT_MOUNT_PATH = self.saved

    def tearDown(self):
        self.camera.DEFAULT_MOUNT_PATH = self.previous
        shutil.rmtree(self.dir, ignore_errors=True)

    def _save(self):
        mount = self.camera.mount_from_xyz_rpy((-0.061, 0.034, 0.067), (0.0, 0.0, -90.0))
        self.camera.save_mount(self.saved, mount, source="console", method="aligned by eye")
        return mount

    def test_with_nothing_saved_a_launch_uses_the_placeholder_and_says_so(self):
        mount, source, _ = self.camera.resolve_mount(None)
        np.testing.assert_allclose(mount.pose, self.camera.WristMount().pose, atol=1e-12)
        self.assertIn("no saved mount", source)

    def test_a_saved_mount_is_used_with_nothing_passed(self):
        """The whole point: save it in the console once, and every launch after that is accurate."""
        saved = self._save()
        mount, source, _ = self.camera.resolve_mount(None)
        np.testing.assert_allclose(mount.pose, saved.pose, atol=1e-9)
        self.assertIn("saved default", source)

    def test_the_source_carries_the_files_own_provenance(self):
        """A run log has to be able to say the mount was aligned by eye without knowing where it came from."""
        self._save()
        _, source, _ = self.camera.resolve_mount(None)
        self.assertIn("aligned by eye, not measured", source)

    def test_an_explicit_file_beats_the_saved_default(self):
        self._save()
        other = self.dir / "other.json"
        elsewhere = self.camera.mount_from_xyz_rpy((0.01, 0.02, 0.03), (1.0, 2.0, 3.0))
        self.camera.save_mount(other, elsewhere, source="explicit")
        mount, source, _ = self.camera.resolve_mount(str(other))
        np.testing.assert_allclose(mount.pose, elsewhere.pose, atol=1e-9)
        self.assertIn("other.json", source)

    def test_none_forces_the_placeholder_even_when_one_is_saved(self):
        """A run has to be able to reproduce the assumed geometry deliberately."""
        self._save()
        mount, source, _ = self.camera.resolve_mount("none")
        np.testing.assert_allclose(mount.pose, self.camera.WristMount().pose, atol=1e-12)
        self.assertIn("--mount none", source)

    def test_a_caller_may_supply_its_own_fallback(self):
        other = self.camera.WristMount(pos_link6=(0.0, 0.0, 0.1), pitch_deg=5.0)
        mount, _, _ = self.camera.resolve_mount("none", fallback=other)
        np.testing.assert_allclose(mount.pose, other.pose, atol=1e-12)


class PickAvailabilityTests(unittest.TestCase):
    """Why the PICK button is on or off, and what turns it on without relaunching the console."""

    def setUp(self):
        from demos.cup.pick_demo import camera

        self.camera = camera
        self.arm = server.ArmServer(iface="none", legs=True, sphere_radius_m=0.40, step_deg=5.0, log_path=None,
                                    mode="hardware", mode_reason="test",
                                    client=sim_feed.SimFeedClient("http://127.0.0.1:1"))
        self.arm._log = QUIET
        self.camera_stub = types.SimpleNamespace(has_depth=lambda: True, status=lambda: {"source": "test"})
        self.arm.camera = self.camera_stub
        mount = camera.WristMount()
        self.arm.pick_cfg = {"mount": mount, "perception": types.SimpleNamespace(mount=mount),
                             "mount_source": "assumed", "camera_source": "test"}
        self.arm.mount = mount

    def test_a_configured_console_offers_the_pick_with_nothing_to_set_first(self):
        """There used to be one more thing to type: how high the mount sits above the table. Nothing needs
        it now -- every pose is placed from the arm's own mount -- so a bench console with an arm, a camera
        and depth is ready as it stands."""
        status = self.arm.pick_status()
        self.assertTrue(status["configured"])
        self.assertTrue(status["available"], status["refusal"])
        self.assertEqual(status["refusal"], "")
        self.assertNotIn("base_height_m", status)

    def test_the_pick_uses_the_mount_the_editor_holds(self):
        """'Use the extrinsics that were loaded': one object, so the two cannot drift apart."""
        self.arm.set_mount([-0.061, 0.034, 0.067], [0.0, 0.0, -90.0])
        expected = self.camera.mount_from_xyz_rpy([-0.061, 0.034, 0.067], [0.0, 0.0, -90.0])
        np.testing.assert_allclose(self.arm.pick_cfg["mount"].pose, expected.pose, atol=1e-9)
        np.testing.assert_allclose(self.arm.pick_cfg["perception"].mount.pose, expected.pose, atol=1e-9)
        self.assertIs(self.arm.pick_cfg["mount"], self.arm.mount)

    def test_without_depth_it_still_refuses_because_the_pick_needs_it(self):
        """Not every refusal is a configuration slip; this one is physics and must survive."""
        self.camera_stub.has_depth = lambda: False
        self.assertIn("no depth", self.arm.pick_status()["refusal"])


@unittest.skipUnless(shutil.which("bash"), "no bash here")
class BesideSimTests(unittest.TestCase):
    """demos/cup/d1_ui/beside_sim.sh: the console the simulator launchers start and stop."""

    ROOT = Path(__file__).resolve().parents[3]
    LAUNCHERS = ("demos/cup/run_pick_demo.sh", "demos/combiner/run_combiner_demo.sh")

    def split(self, *argv):
        import subprocess

        script = ('set -euo pipefail; . demos/cup/d1_ui/beside_sim.sh; console_args "$@"; '
                  'printf "%s\\n" "$CONSOLE" "$HEADLESS" "$FEED_PORT" ${SIM_ARGS[@]+"${SIM_ARGS[@]}"}')
        out = subprocess.run(["bash", "-c", script, "bash", *argv], cwd=self.ROOT, capture_output=True,
                             text=True, check=True, env={"PATH": "/usr/bin:/bin"}).stdout.splitlines()
        return out[0], out[1], out[2], out[3:]

    def test_launcher_flags_are_read_and_the_rest_forwarded(self):
        self.assertEqual(self.split(), ("1", "0", "8765", []))
        self.assertEqual(self.split("--headless", "--ui_feed_port", "9001", "--seed", "3"),
                         ("1", "1", "9001", ["--headless", "--ui_feed_port", "9001", "--seed", "3"]))
        self.assertEqual(self.split("--no_console", "--ui_feed_port=0", "--box_range", "0.6", "0.7"),
                         ("0", "0", "0", ["--ui_feed_port=0", "--box_range", "0.6", "0.7"]))
        self.assertEqual(self.split("--help")[0], "0")

    def test_launchers_keep_the_shell_that_stops_the_console(self):
        # `exec python ...` would replace the shell whose EXIT trap stops the console, leaving it orphaned.
        for launcher in self.LAUNCHERS:
            with self.subTest(launcher=launcher):
                text = (self.ROOT / launcher).read_text()
                self.assertIn(". demos/cup/d1_ui/beside_sim.sh", text)
                self.assertIn("console_start ", text)
                self.assertNotRegex(text, r"(?m)^\s*exec\s+python")


if __name__ == "__main__":
    unittest.main()
