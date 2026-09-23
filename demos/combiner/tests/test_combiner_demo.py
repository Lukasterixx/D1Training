import contextlib
import io
import math
from pathlib import Path
import random
import tempfile
import unittest
from unittest.mock import patch
import types

from demos.combiner.geometry import DEFAULT_HANDLE_TORQUE_NM, GEOMETRY, Placement, sample_placement, validate_region
from demos.combiner.run_combiner_demo import parse_args
from demos.combiner.latch import DoorLatch, LatchController
from demos.common.resting import LYING_LEG_POSE, make_resting_cfg

try:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
except ImportError:
    Usd = None

try:
    import torch
except ImportError:
    torch = None


class PlacementTests(unittest.TestCase):
    def test_seed_replays_the_entire_reset_sequence(self):
        first, replay, other = random.Random(7), random.Random(7), random.Random(8)
        expected = [sample_placement(first) for _ in range(20)]
        self.assertEqual(expected, [sample_placement(replay) for _ in range(20)])
        self.assertNotEqual(expected, [sample_placement(other) for _ in range(20)])
        self.assertEqual(len(set(expected)), 20)

    def test_default_positions_face_dog_and_stay_in_requested_sector(self):
        rng = random.Random(42)
        bearings = []
        for _ in range(1000):
            pose = sample_placement(rng)
            x, y, z = pose.position
            radius = math.hypot(x, y)
            self.assertTrue(0.62 <= radius <= 0.70)
            bearing = math.degrees(math.atan2(y, x))
            bearings.append(bearing)
            self.assertTrue(-45 <= bearing <= 45)
            self.assertEqual(z, 0)
            self.assertLessEqual(abs(pose.yaw_deg - bearing - 180), 10)
            self.assertAlmostEqual(sum(v * v for v in pose.quaternion), 1)
            handle = pose.world_point(GEOMETRY.handle)
            self.assertLess(math.hypot(*handle[:2]), radius)
            self.assertAlmostEqual(handle[2], 0.30)
        self.assertLess(min(bearings), -44)
        self.assertGreater(max(bearings), 44)

    def test_full_circle_covers_every_quadrant(self):
        rng = random.Random(7)
        quadrants = set()
        for _ in range(100):
            pose = sample_placement(rng, sweep_deg=180)
            quadrants.add(tuple(v > 0 for v in pose.position[:2]))
        self.assertEqual(len(quadrants), 4)

    def test_zero_sweep_and_jitter_give_exact_front_facing_pose(self):
        pose = sample_placement(random.Random(1), distance=(0.65, 0.65), sweep_deg=0, yaw_jitter_deg=0)
        self.assertEqual(pose, Placement((0.65, 0.0, 0.0), 180.0))
        handle = pose.world_point(GEOMETRY.handle)
        self.assertAlmostEqual(handle[0], 0.489)
        self.assertAlmostEqual(handle[1], -0.055)

    def test_bad_sampling_bounds_are_rejected(self):
        for distance, sweep, jitter in (((0.5, 0.7), 45, 10), ((0.8, 0.6), 45, 10),
                                       ((0.6, math.inf), 45, 10), ((0.6, 0.7), 181, 10),
                                       ((0.6, 0.7), -1, 10), ((0.6, 0.7), 45, 31),
                                       ((0.6, 0.7), math.nan, 10)):
            with self.subTest(distance=distance, sweep=sweep, jitter=jitter), self.assertRaises(ValueError):
                validate_region(distance, sweep, jitter)


class RunnerTests(unittest.TestCase):
    def test_viewer_and_headless_defaults(self):
        viewer, headless = parse_args([]), parse_args(["--headless"])
        self.assertEqual(viewer.episodes, 0)
        self.assertTrue(viewer.realtime)
        self.assertEqual(headless.episodes, 1)
        self.assertFalse(headless.realtime)
        self.assertEqual(parse_args(["--episodes", "3"]).episodes, 3)
        from demos.combiner.run_combiner_demo import grasp_params

        self.assertEqual(grasp_params(viewer), {})
        self.assertEqual(grasp_params(parse_args(["--grasp", "50", "-1"])), {"pitch_deg": 50.0, "roll": -1})

    def test_invalid_options_fail_before_simulator_launch(self):
        for args in (["--headless", "--episodes", "0"], ["--episodes", "-1"],
                     ["--door_angle_deg", "111"], ["--door_angle_deg", "nan"],
                     ["--handle_angle_deg", "61"], ["--handle_angle_deg", "nan"],
                     ["--episode_s", "0"], ["--episode_s", "nan"], ["--seed", "-1"],
                     ["--grasp", "40", "0"], ["--grasp", "95", "-1"], ["--grasp", "40"]):
            with self.subTest(args=args), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(args)

    def launch(self, *argv):
        """What `run_combiner_demo.sh` hands Python, with conda and python stubbed and the console off."""
        import os
        import stat
        import subprocess

        root = Path(__file__).resolve().parents[3]
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            (base / "bin").mkdir()
            (base / "etc/profile.d").mkdir(parents=True)
            (base / "etc/profile.d/conda.sh").write_text("conda() { :; }\n")
            for name, body in (("conda", "exit 0"), ("python", 'printf "%s\\n" "$@"')):
                path = base / "bin" / name
                path.write_text(f"#!/bin/sh\n{body}\n")
                path.chmod(path.stat().st_mode | stat.S_IEXEC)
            env = {"PATH": f"{base / 'bin'}:/usr/bin:/bin", "CONDA_EXE": str(base / "bin/conda"), "D1_UI_CONSOLE": "0",
                   "HOME": os.environ.get("HOME", tmp)}
            out = subprocess.run([str(root / "demos/combiner/run_combiner_demo.sh"), *argv], cwd=tmp, env=env,
                                 capture_output=True, text=True, check=True).stdout.splitlines()
        return [line for line in out if not line.startswith("[combiner]")]

    def test_launcher_without_arguments_grips_and_pulls_against_0_3_nm(self):
        forwarded = self.launch()
        self.assertEqual(forwarded, ["demos/combiner/run_combiner_demo.py", "--turn", "--handle_torque_nm", "0.3"])
        args = parse_args(forwarded[1:])
        self.assertTrue(args.turn)
        self.assertEqual(args.method, "pull")
        self.assertEqual(args.handle_torque_nm, [0.3])
        self.assertEqual(args.episodes, 0)   # the viewer, as a bare launch always was
        # Any argument hands the choice back, including one that is only the launcher's.
        self.assertEqual(self.launch("--seed", "42")[1:], ["--seed", "42"])
        self.assertEqual(self.launch("--no_console")[1:], [])

    def test_shared_resting_setup_preserves_the_cup_model(self):
        cfg = types.SimpleNamespace(
            scene=types.SimpleNamespace(robot=types.SimpleNamespace(init_state=types.SimpleNamespace())),
            actions=types.SimpleNamespace(arm=types.SimpleNamespace()),
            terminations=types.SimpleNamespace(time_out="unchanged"),
            commands=types.SimpleNamespace(ee_position=types.SimpleNamespace()))
        module = types.ModuleType("position_only.env_cfg")
        calls = []

        def make_cfg(*args, **kwargs):
            calls.append((args, kwargs))
            return cfg

        module.make_cfg = make_cfg
        with patch.dict("sys.modules", {"position_only.env_cfg": module}):
            result = make_resting_cfg("robot.usd", seed=7, episode_s=99)
        self.assertIs(result, cfg)
        self.assertEqual(calls[0][1]["spawn_height"], 0.18)
        self.assertEqual(calls[0][1]["arm_actuator"], "d1_servo")
        self.assertEqual(calls[0][1]["arm_trajectory"], "measured")
        self.assertEqual(calls[0][1]["leg_actuator"], "unitree")
        self.assertEqual(cfg.scene.robot.init_state.joint_pos,
                         {**LYING_LEG_POSE, "Joint[1-6]": 0.0, "Joint7_.*": 0.0})
        self.assertEqual(cfg.actions.arm.scale, math.pi)
        self.assertEqual(cfg.episode_length_s, 99)
        self.assertIsNone(cfg.terminations.low_base)
        self.assertEqual(cfg.terminations.time_out, "unchanged")


@unittest.skipUnless(Usd is not None, "USD/pxr is not on this Python's path")
class AssetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from demos.combiner.asset import build_combiner_usd

        cls.directory = tempfile.TemporaryDirectory()
        cls.path = Path(cls.directory.name) / "combiner.usda"
        cls.info = build_combiner_usd(cls.path)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def setUp(self):
        self.stage = Usd.Stage.Open(str(self.path))
        self.stage.SetEditTarget(self.stage.GetSessionLayer())

    def assertVectorNear(self, actual, expected):
        for a, e in zip(actual, expected):
            self.assertAlmostEqual(a, e, places=6)

    def test_articulation_has_door_hinge_and_spring_return_handle_joint(self):
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(self.stage), 1.0)
        root = self.stage.GetDefaultPrim()
        self.assertTrue(root.HasAPI(UsdPhysics.ArticulationRootAPI))
        bodies = [p.GetName() for p in self.stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        self.assertEqual(sorted(bodies), ["Door", "Enclosure", "Handle"])
        hinge = UsdPhysics.RevoluteJoint.Get(self.stage, "/CombinerBox/DoorHinge")
        self.assertEqual(hinge.GetLowerLimitAttr().Get(), 0)
        self.assertEqual(hinge.GetUpperLimitAttr().Get(), 110)
        self.assertFalse(hinge.GetCollisionEnabledAttr().Get())
        self.assertEqual(UsdPhysics.DriveAPI(hinge.GetPrim(), "angular").GetStiffnessAttr().Get(), 0)
        axis = Gf.Rotation(hinge.GetLocalRot0Attr().Get()).TransformDir(Gf.Vec3d(0, 0, 1))
        self.assertVectorNear(axis, (0, 0, -1))
        for target in (hinge.GetBody0Rel(), hinge.GetBody1Rel()):
            self.assertTrue(self.stage.GetPrimAtPath(target.GetTargets()[0]).HasAPI(UsdPhysics.RigidBodyAPI))
        handle_joint = UsdPhysics.RevoluteJoint.Get(self.stage, "/CombinerBox/HandleJoint")
        self.assertEqual(handle_joint.GetAxisAttr().Get(), "X")
        self.assertEqual(handle_joint.GetLowerLimitAttr().Get(), 0)
        self.assertEqual(handle_joint.GetUpperLimitAttr().Get(), 60)
        self.assertEqual(str(handle_joint.GetBody0Rel().GetTargets()[0]), "/CombinerBox/Door")
        self.assertEqual(str(handle_joint.GetBody1Rel().GetTargets()[0]), "/CombinerBox/Handle")
        spring = UsdPhysics.DriveAPI(handle_joint.GetPrim(), "angular")
        self.assertEqual(spring.GetTargetPositionAttr().Get(), GEOMETRY.spring_rest_deg)
        # Per degree in USD: the default spring needs its stated torque at 45 degrees.
        per_deg = spring.GetStiffnessAttr().Get()
        self.assertAlmostEqual(per_deg * (45 - GEOMETRY.spring_rest_deg), DEFAULT_HANDLE_TORQUE_NM, places=5)
        self.assertEqual(self.info["handle_spring"]["torque_at_45_deg_nm"], DEFAULT_HANDLE_TORQUE_NM)

    def test_grasp_follows_both_joints_and_lever_rotates_down(self):
        door = UsdGeom.Xform.Get(self.stage, "/CombinerBox/Door")
        handle = UsdGeom.Xform.Get(self.stage, "/CombinerBox/Handle")
        handle.ClearXformOpOrder()
        pose_op = handle.AddTransformOp()
        grasp = self.stage.GetPrimAtPath("/CombinerBox/Handle/HandleGrasp")
        joint = UsdPhysics.RevoluteJoint.Get(self.stage, "/CombinerBox/HandleJoint")
        anchor = Gf.Matrix4d(1).SetTranslate(Gf.Vec3d(joint.GetLocalPos0Attr().Get()))
        # USD bodies are siblings; evaluate the joint chain just as a physics solver
        # does, using the authored joint frames rather than assuming prim parenting.
        for door_angle in (0, 30, 90, 110):
            door.GetOrderedXformOps()[1].Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, -1), door_angle).GetQuat()))
            for lever_angle in (0, 30, 60):
                lever_rotation = Gf.Matrix4d(1).SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), lever_angle))
                door_world = UsdGeom.XformCache().GetLocalToWorldTransform(door.GetPrim())
                pose_op.Set(lever_rotation * anchor * door_world)
                cache = UsdGeom.XformCache()
                self.assertVectorNear(cache.GetLocalToWorldTransform(grasp).ExtractTranslation(),
                                      GEOMETRY.handle_at_angle(door_angle, lever_angle))
                self.assertVectorNear(door_world.ExtractTranslation(), GEOMETRY.hinge)
        self.assertGreater(GEOMETRY.handle_at_angle(90)[0], GEOMETRY.handle[0])
        self.assertLess(GEOMETRY.handle_at_angle(0, 30)[2], GEOMETRY.handle[2] - 0.03)

    def test_hollow_shell_and_handle_gap_are_not_filled_by_colliders(self):
        self.assertFalse(self.stage.GetPrimAtPath("/CombinerBox/Enclosure").HasAPI(UsdPhysics.CollisionAPI))
        self.assertFalse(self.stage.GetPrimAtPath("/CombinerBox/Door").HasAPI(UsdPhysics.CollisionAPI))
        for path in ("Door/Panel", "Door/HandleRosette", "Handle/Spindle", "Handle/Lever"):
            self.assertTrue(self.stage.GetPrimAtPath(f"/CombinerBox/{path}").HasAPI(UsdPhysics.CollisionAPI))
        # A grasp point between bar and door must be outside EVERY primitive collider.
        gap = Gf.Vec3d(GEOMETRY.handle[0] - 0.025, GEOMETRY.handle[1], GEOMETRY.handle[2])
        cache = UsdGeom.XformCache()
        for prim in self.stage.Traverse():
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            point = cache.GetLocalToWorldTransform(prim).GetInverse().Transform(gap)
            if prim.IsA(UsdGeom.Cube):
                half = UsdGeom.Cube(prim).GetSizeAttr().Get() / 2
                inside = all(abs(v) <= half for v in point)
            else:
                cylinder = UsdGeom.Cylinder(prim)
                axis = "XYZ".index(cylinder.GetAxisAttr().Get())
                inside = (abs(point[axis]) <= cylinder.GetHeightAttr().Get() / 2 and
                          sum(point[i] ** 2 for i in range(3) if i != axis) <= cylinder.GetRadiusAttr().Get() ** 2)
            self.assertFalse(inside, prim.GetPath())

    def test_referenced_asset_preserves_anchors_at_arbitrary_spawn(self):
        stage = Usd.Stage.CreateInMemory()
        instance = UsdGeom.Xform.Define(stage, "/World/Box")
        instance.GetPrim().GetReferences().AddReference(str(self.path))
        pose = Placement((0.5, -0.5, 0), 135)
        instance.AddTranslateOp().Set(Gf.Vec3d(*pose.position))
        instance.AddOrientOp().Set(Gf.Quatf(*pose.quaternion))
        cache = UsdGeom.XformCache()
        hinge = UsdPhysics.RevoluteJoint.Get(stage, "/World/Box/DoorHinge")
        anchor0 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(hinge.GetBody0Rel().GetTargets()[0]))
        anchor1 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(hinge.GetBody1Rel().GetTargets()[0]))
        self.assertVectorNear(anchor0.Transform(Gf.Vec3d(hinge.GetLocalPos0Attr().Get())),
                              anchor1.Transform(Gf.Vec3d(hinge.GetLocalPos1Attr().Get())))
        mount = UsdPhysics.FixedJoint.Get(stage, "/World/Box/WorldMount")
        anchors = [cache.GetLocalToWorldTransform(stage.GetPrimAtPath(rel.GetTargets()[0])).ExtractTranslation()
                   for rel in (mount.GetBody0Rel(), mount.GetBody1Rel())]
        self.assertVectorNear(anchors[0], anchors[1])
        lever_joint = UsdPhysics.RevoluteJoint.Get(stage, "/World/Box/HandleJoint")
        lever0 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(lever_joint.GetBody0Rel().GetTargets()[0]))
        lever1 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(lever_joint.GetBody1Rel().GetTargets()[0]))
        self.assertVectorNear(lever0.Transform(Gf.Vec3d(lever_joint.GetLocalPos0Attr().Get())),
                              lever1.Transform(Gf.Vec3d(lever_joint.GetLocalPos1Attr().Get())))
        grasp = stage.GetPrimAtPath("/World/Box/Handle/HandleGrasp")
        self.assertVectorNear(cache.GetLocalToWorldTransform(grasp).ExtractTranslation(), pose.world_point(GEOMETRY.handle))


class LatchTests(unittest.TestCase):
    def test_requires_rotation_to_release_not_door_motion(self):
        latch = DoorLatch()
        release = GEOMETRY.handle_release_deg
        self.assertEqual(release, 45.0)  # Lukas, 2026-09-19: the handle must turn 45 degrees
        for handle in (0, 10, 30, release - 0.1):
            self.assertTrue(latch.update(0, handle))
        # Even a numerical limit overshoot must not release a still-latched door.
        self.assertTrue(latch.update(2, 0))
        self.assertLess(latch.door_upper_limit_deg, 1)
        self.assertFalse(latch.update(0, release))
        self.assertEqual(latch.door_upper_limit_deg, 110)

    def test_releasing_handle_while_open_does_not_lock_door_in_midair(self):
        latch = DoorLatch()
        latch.update(0, 45)
        for door in (5, 90, 20, 1):
            self.assertFalse(latch.update(door, 0))
        self.assertTrue(latch.update(0.4, 0))

    def test_release_has_hysteresis_and_relocks_if_door_was_never_pulled(self):
        latch = DoorLatch()
        latch.update(0, GEOMETRY.handle_release_deg + 1)
        for handle in (44, 46, 20, 10.1):
            self.assertFalse(latch.update(0, handle))
        self.assertTrue(latch.update(0, 10))

    def test_door_does_not_latch_closed_while_handle_is_held_down(self):
        latch = DoorLatch()
        latch.update(0, 45)
        latch.update(90, 45)
        self.assertFalse(latch.update(0, 45))
        self.assertTrue(latch.update(0, 0))

    def test_reset_supports_open_inspection_poses_and_restores_latch(self):
        latch = DoorLatch()
        latch.reset(90, 0)
        self.assertFalse(latch.latched)
        latch.reset(0, 45)
        self.assertFalse(latch.latched)
        latch.reset()
        self.assertTrue(latch.latched)

    def test_nonfinite_feedback_is_an_error(self):
        for angles in ((math.nan, 0), (0, math.inf)):
            with self.assertRaises(ValueError):
                DoorLatch().update(*angles)


@unittest.skipUnless(torch is not None, "Controller adapter checks need torch (use env_isaaclab)")
class LatchControllerTests(unittest.TestCase):
    def setUp(self):
        # Deliberately reversed order catches accidental hard-coded joint indices.
        class Box:
            joint_names = ["HandleJoint", "DoorHinge"]
            device = "cpu"

            def __init__(self):
                self.data = types.SimpleNamespace(default_joint_pos=torch.zeros((1, 2)),
                                                   joint_pos=torch.zeros((1, 2)), joint_vel=torch.zeros((1, 2)))
                self.limits = torch.tensor([[[0., math.pi / 3], [0., math.radians(110)]]])
                self.target = torch.zeros((1, 2))
                self.effort = torch.zeros((1, 2))
                self.state_writes = 0

            def write_joint_position_limit_to_sim(self, limits, joint_ids, **kwargs):
                self.limits[:, joint_ids] = limits

            def write_joint_state_to_sim(self, q, dq):
                self.data.joint_pos.copy_(q)
                self.data.joint_vel.copy_(dq)
                self.state_writes += 1

            def set_joint_position_target(self, value, joint_ids=None):
                self.target[:, slice(None) if joint_ids is None else joint_ids] = value

            def set_joint_effort_target(self, value, joint_ids=None):
                self.effort[:, slice(None) if joint_ids is None else joint_ids] = value

            def set_joint_velocity_target(self, value):
                pass

            def write_joint_stiffness_to_sim(self, value, joint_ids):
                self.stiffness = (value, joint_ids)

            def write_joint_effort_limit_to_sim(self, value, joint_ids):
                self.effort_limit = (value, joint_ids)

            def reset(self):
                pass

            def write_data_to_sim(self):
                pass

        self.box = Box()
        self.controller = LatchController(self.box)
        self.controller.reset()

    def test_open_is_blocked_until_measured_handle_turns_and_never_teleports(self):
        c, box = self.controller, self.box
        self.assertFalse(c.command("O"))
        c.command("H")
        c.on_physics_step(0.005)
        self.assertAlmostEqual(float(box.target[0, 0]), math.radians(GEOMETRY.handle_hold_deg))
        self.assertGreater(GEOMETRY.handle_hold_deg, GEOMETRY.handle_release_deg)
        self.assertFalse(c.command("O"))  # target alone cannot release it
        box.data.joint_pos[0, 0] = math.radians(GEOMETRY.handle_release_deg - 1)
        c.on_physics_step(0.005)
        self.assertFalse(c.command("O"))  # 44 degrees is not enough
        box.data.joint_pos[0, 0] = math.radians(GEOMETRY.handle_release_deg + 1)
        c.on_physics_step(0.005)
        self.assertTrue(c.command("O"))
        c.on_physics_step(0.005)
        self.assertAlmostEqual(float(box.limits[0, 1, 1]), math.radians(110))
        self.assertGreater(float(box.effort[0, 1]), 0)
        self.assertLessEqual(float(box.effort[0, 1]), 0.600001)
        self.assertEqual(box.state_writes, 1)  # only reset writes poses

    def test_return_close_relatch_and_reset_clear_old_commands(self):
        c, box = self.controller, self.box
        box.data.joint_pos[0] = torch.tensor([math.radians(45), math.radians(90)])
        c.on_physics_step(0.005)
        c.command("C")
        c.on_physics_step(0.005)
        self.assertLess(float(box.effort[0, 1]), 0)
        box.data.joint_pos.zero_()
        c.on_physics_step(0.005)
        self.assertTrue(c.latch.latched)
        self.assertAlmostEqual(float(box.limits[0, 1, 1]), math.radians(0.5))
        self.assertIsNone(c.door_target_deg)
        c.command("H")
        c.reset()
        self.assertFalse(c.handle_held)
        self.assertTrue(c.latch.latched)
        # The spring's target is its rest angle below the stop: the preload. The door has no target.
        self.assertAlmostEqual(float(box.target[0, 0]), math.radians(GEOMETRY.spring_rest_deg))
        self.assertEqual(float(box.target[0, 1]), 0)
        self.assertTrue(bool((box.effort == 0).all()))

    def test_free_control_releases_targets_but_preserves_latch_constraint(self):
        c, box = self.controller, self.box
        c.command("H")
        c.command("C")
        c.command("F")
        c.on_physics_step(0.005)
        self.assertAlmostEqual(float(box.target[0, 0]), math.radians(GEOMETRY.spring_rest_deg))
        self.assertEqual(float(box.effort[0, 1]), 0)
        self.assertTrue(c.latch.latched)

    def test_spring_is_resized_on_the_handle_joint_by_name(self):
        stiffness = self.controller.set_spring(0.25)
        self.assertAlmostEqual(stiffness, GEOMETRY.spring_stiffness(0.25))
        self.assertEqual(self.box.stiffness, (stiffness, [0]))   # HandleJoint is index 0 in this double
        self.assertEqual(self.box.effort_limit, (GEOMETRY.spring_effort_limit(0.25), [0]))
        self.assertEqual(self.controller.spring_torque_nm, 0.25)

    def test_reset_open_pose_is_not_clamped_to_closed_limit(self):
        self.controller.reset(90, 0)
        self.assertFalse(self.controller.latch.latched)
        self.assertAlmostEqual(float(self.box.limits[0, 1, 1]), math.radians(110))
        self.assertAlmostEqual(float(self.box.data.joint_pos[0, 1]), math.pi / 2)


if __name__ == "__main__":
    unittest.main()
