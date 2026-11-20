"""The dog's numpy insertion runner (`demos/combiner_key/robot/insert_policy.py`) against the training code, no simulator.

The task layer against `unifp_train.keylock.InsertLayer` on scripted depths (both seat tests, whatever the torch layer
does at test time); the kinematics against `unifp_train.arm_kin`; the IK and the start solve against
`keyinsert_env`'s own methods, compiled from its source (that module imports Isaac Lab); the gravity model against
`keyhold.ArmGravity`. A recorded Isaac rollout is checked by `check_insert.py --sim`. Runs on the system Python (torch,
numpy); the export of a random actor needs rsl-rl and is skipped without it.
"""
from __future__ import annotations

import __future__
import ast
import importlib.util
import json
import math
import tempfile
import types
import unittest
from pathlib import Path

import numpy as np
import torch

from demos.combiner_key.robot import export_policy, insert_policy, leg_policy
from unifp_isaaclab import interface
from unifp_train import arm_kin, keyhold, keylock, leg_cfg

REPO = Path(__file__).resolve().parents[1]
ENV_SRC = REPO / "unifp_train" / "keyinsert_env.py"
HOLD_SRC = REPO / "unifp_train" / "keyhold_env.py"


def _source_functions(path: Path, names, namespace: dict) -> dict:
    """The functions `names` (module level or in any class) of `path`, compiled in `namespace`."""
    tree = ast.parse(path.read_text())
    found = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name in names and node.name not in found:
            code = compile(ast.Module(body=[node], type_ignores=[]), str(path), "exec",
                           flags=__future__.annotations.compiler_flag, dont_inherit=True)
            scope = dict(namespace)
            exec(code, scope)
            found[node.name] = scope[node.name]
    missing = [n for n in names if n not in found]
    if missing:
        raise AssertionError(f"{path.name} has no {missing}")
    return found


def _env_constants() -> dict:
    return export_policy.source_constants(ENV_SRC, export_policy.INSERT_ENV_CONSTANTS,
                                          {"keylock": keylock, "leg_cfg": leg_cfg, "math": math})


def _layer_constants() -> dict:
    return {name: getattr(keylock, name) for name in insert_policy.LAYER_CONSTANTS}


# --- a scripted plant for the layer --------------------------------------------------------------------------------

SCENARIOS = (
    # (true face depth from the estimate m, the keyway's centre across (u, t) m, a jam short of the floor m or None,
    #  a stall in free air (start s, end s) or None)
    (0.000, (0.0020, -0.0010), None, None),       # off centre: the spiral finds it
    (0.004, (0.0000, 0.0000), None, None),        # lined up: straight in
    (-0.005, (0.0005, 0.0005), 0.004, None),      # jammed 4 mm in
    (0.006, (0.0030, 0.0020), None, None),        # far off: a long search
    (0.005, (0.0000, 0.0010), None, (0.8, 2.0)),  # the arm stalls in free air: a face felt where it is not
    (0.030, (0.0000, 0.0000), None, None),        # no face within the reference's reach
)


def _plant(n: int, seed: int):
    """Per environment, a hand that follows the layer's reference with a lag and stops on the face, entering the keyway
    only lined up and with the wrist within 6 deg (the wards); measured with a little noise. Returns step(ref, ...)."""
    rng = np.random.default_rng(seed)
    sc = [SCENARIOS[i % len(SCENARIOS)] for i in range(n)]
    face = np.array([s[0] for s in sc])
    hole = np.array([s[1] for s in sc])
    stop = np.array([s[0] + (s[2] if s[2] is not None else keylock.KEYWAY_DEPTH_M) for s in sc])
    stall = [s[3] for s in sc]
    state = {"d": np.full(n, -0.0175), "lat": np.zeros((n, 2)), "inside": np.zeros(n, bool), "t": 0.0}

    def step(ref_depth, ref_lateral, roll, dt):
        d, lat, inside = state["d"], state["lat"], state["inside"]
        moving = np.array([not (w is not None and w[0] <= state["t"] < w[1]) for w in stall])
        d = np.where(moving, d + 0.3 * (ref_depth - d), d)
        lat = np.where(moving[:, None], lat + 0.3 * (ref_lateral - lat), lat)
        lined = (np.linalg.norm(lat - hole, axis=-1) < 0.0006) & (np.abs(roll) < math.radians(6.0))
        inside = inside | (lined & (d > face - 0.0003))
        d = np.where(inside, np.minimum(d, stop), np.minimum(d, face))
        lat = np.where(inside[:, None], hole + 0.3 * (lat - hole), lat)
        state.update(d=d, lat=lat, inside=inside, t=state["t"] + dt)
        noise = rng.normal(0.0, 2e-5, n), rng.normal(0.0, 2e-5, (n, 2))
        return (d + noise[0]).astype(np.float32), (lat + noise[1]).astype(np.float32)

    return step


def _run_both(seat_test: str, light_probe: bool = False, n: int = 12, steps: int = 1600):
    """The torch and the numpy layer, fed the same measured depths (the plant driven by the torch layer)."""
    tl = keylock.InsertLayer(n, "cpu")
    tl.seat_test, tl.light_probe = seat_test, light_probe
    nl = insert_policy.InsertLayer(n, _layer_constants(), seat_test, light_probe)
    force = torch.arange(n) % 2 == 0
    start = torch.full((n,), -0.0175)
    press = torch.full((n,), keylock.EVAL_PRESS_N)
    ids = torch.arange(n)
    tl.start(ids, start, force, press)
    nl.start(ids.numpy(), start.numpy(), force.numpy(), press.numpy())
    plant = _plant(n, seed=7 if seat_test == "old" else 8)
    dt = interface.POLICY_DT
    worst, phases = {}, set()
    for k in range(steps):
        depth, lateral = plant(tl.ref_depth.numpy().astype(float), tl.ref_lateral.numpy().astype(float),
                               tl.roll.numpy().astype(float), dt)
        tl.step(torch.from_numpy(depth), torch.from_numpy(lateral), dt)
        nl.step(depth, lateral, dt)
        for name, value in vars(tl).items():
            if not isinstance(value, torch.Tensor):
                continue
            mine = getattr(nl, name)
            ref = value.numpy()
            if ref.dtype == np.float32:
                assert mine.dtype == np.float32, (name, mine.dtype)
                worst[name] = max(worst.get(name, 0.0), float(np.abs(mine - ref).max()))
            else:
                np.testing.assert_array_equal(mine.astype(ref.dtype), ref, err_msg=f"{name} at step {k}")
        phases |= set(tl.phase.tolist())
    return worst, phases


class Layer(unittest.TestCase):
    def test_every_layer_constant_is_exported(self):
        """Every module constant `InsertLayer` reads is in `LAYER_CONSTANTS` (a constant added there fails here)."""
        tree = ast.parse((REPO / "unifp_train" / "keylock.py").read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "InsertLayer")
        used = {n.id for n in ast.walk(cls) if isinstance(n, ast.Name) and n.id.isupper() and hasattr(keylock, n.id)}
        self.assertEqual(used - set(insert_policy.LAYER_CONSTANTS), set())

    def test_numpy_layer_is_the_torch_layer(self):
        for seat_test, light in (("old", False), ("old", True), ("located", False)):
            with self.subTest(seat_test=seat_test, light_probe=light):
                worst, phases = _run_both(seat_test, light)
                # The phases, stages and flags are equal every step (asserted inside); the floats to float32's sin.
                self.assertLess(max(worst.values()), 1e-6, worst)
                self.assertGreater(len(phases), 1, "the script never left the approach")

    def test_float32_timers(self):
        """Why the layer is float32: 15 steps of 0.02 s reach 0.3 s in float64, not in float32 (torch's)."""
        t32, t64 = np.zeros(1, np.float32), 0.0
        for _ in range(15):
            t32, t64 = t32 + 0.02, t64 + 0.02
        tt = torch.zeros(1)
        for _ in range(15):
            tt = tt + 0.02
        self.assertTrue(t64 >= 0.3)
        self.assertFalse(bool(t32[0] >= 0.3))
        self.assertFalse(bool(tt[0] >= 0.3))
        self.assertEqual(t32.tobytes(), tt.numpy().tobytes())


# --- kinematics ------------------------------------------------------------------------------------------------------

class Kinematics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        consts = export_policy.arm_constants(_env_constants()["LIMIT_SHARE"], wrist_payload=True)
        cls.consts = consts
        cls.np_kin = insert_policy.ArmKinematics(consts["kinematics"])
        cls.t64 = arm_kin.ArmKinematics("cpu", torch.float64)
        cls.t32 = arm_kin.ArmKinematics("cpu", torch.float32)
        rng = np.random.default_rng(3)
        nominal = np.asarray(leg_cfg.KEYHOLD_IK_NOMINAL)
        cls.q = nominal + rng.uniform(-0.3, 0.3, (64, 6))
        cls.lower, cls.upper = np.asarray(consts["arm"]["lower"]), np.asarray(consts["arm"]["upper"])
        ns = {"torch": torch, "keylock": keylock, "hand_rotation": keyhold.hand_rotation, "quat_matrix": keyhold.quat_matrix,
              "interface": interface, "leg_cfg": leg_cfg, **_env_constants()}
        ns.update(_source_functions(ENV_SRC, ["_pose_error"], ns))
        cls.src = _source_functions(ENV_SRC, ["_ik", "_solve_start", "_jaw_for_tip", "_tip_off"], ns)
        cls.src.update(_source_functions(HOLD_SRC, ["_in_base", "_sag"], ns))

    def test_forward(self):
        r, p = self.np_kin.tool(self.q)
        rt, pt = self.t64.tool(torch.as_tensor(self.q))
        np.testing.assert_allclose(p, pt.numpy(), atol=1e-12)
        np.testing.assert_allclose(r, rt.numpy(), atol=1e-12)
        np.testing.assert_allclose(self.np_kin.wrist(self.q), self.t64.wrist(torch.as_tensor(self.q)).numpy(), atol=1e-12)

    def _goals(self, k: int = 32):
        r, p = self.np_kin.tool(self.q[:k])
        seed = self.q[:k] + np.random.default_rng(4).normal(0.0, 0.05, (k, 6))
        return p, r, seed

    def test_track_and_inverse(self):
        p, r, seed = self._goals()
        lo, hi = self.lower, self.upper
        q, ok = self.np_kin.track(p, r, seed, lo, hi, iterations=10)
        f = lambda x: torch.as_tensor(x)
        qt, okt = self.t64.track(f(p), f(r), f(seed), f(lo), f(hi), iterations=10)
        np.testing.assert_allclose(q, qt.numpy(), atol=1e-9)
        np.testing.assert_array_equal(ok, okt.numpy())
        seeds = np.asarray(leg_cfg.KEYHOLD_IK_SEEDS)
        qi, oki = self.np_kin.inverse(p, r, seeds, self.np_kin.hard_lower, self.np_kin.hard_upper)
        qit, okit = self.t64.inverse(f(p), f(r), f(seeds), soft=False)
        np.testing.assert_array_equal(oki, okit.numpy())
        np.testing.assert_allclose(qi[oki], qit.numpy()[oki], atol=1e-9)

    def _fake_env(self, kin, dtype, n: int):
        env = types.SimpleNamespace()
        for name, fn in self.src.items():
            setattr(env, name, types.MethodType(fn, env))
        mid, half = 0.5 * (kin.hard_lower + kin.hard_upper), 0.5 * (kin.hard_upper - kin.hard_lower)
        share = _env_constants()["LIMIT_SHARE"]
        env._kin, env._lim_lo_cpu, env._lim_hi_cpu = kin, mid - share * half, mid + share * half
        env._kin_nominal = torch.as_tensor(leg_cfg.KEYHOLD_IK_NOMINAL, dtype=dtype)
        env._kin_seeds = torch.as_tensor(leg_cfg.KEYHOLD_IK_SEEDS, dtype=dtype)
        env._tool_bel = {"reach": torch.full((n,), keylock.TIP_FROM_JAW_M, dtype=dtype), "offset": torch.zeros(n, 2, dtype=dtype)}
        env.device = "cpu"
        return env

    def test_polished_ik_is_the_environments(self):
        """`solve_pose` against `keyinsert_env._ik` itself, in float64 (the algorithm) and float32 (as the env runs it)."""
        p, r, seed = self._goals()
        ik = {"track_iterations": 10, "polish_iterations": _env_constants()["POLISH_ITERATIONS"],
              "polish_damping": _env_constants()["POLISH_DAMPING"], "polish_tol_m": _env_constants()["POLISH_TOL_M"],
              "polish_eps": 1e-4, "polish_rot_tol_rad": 0.01}
        self.assertEqual({k: ik[k] for k in export_policy.IK_LITERALS}, export_policy.IK_LITERALS)
        p_off = p + np.array([0.002, -0.001, 0.0015])          # a goal the seed's pose misses, as the IK's are
        q, ok = self.np_kin.solve_pose(p_off, r, seed, self.lower, self.upper, ik)
        self.assertGreater(int(ok.sum()), len(ok) // 2)
        self.assertLess(int(ok.sum()), len(ok))           # some goals lie past the limits: unsolved, held there
        for dtype, kin, tol in ((torch.float64, self.t64, 1e-9), (torch.float32, self.t32, 2e-6)):
            env = self._fake_env(kin, dtype, len(p))
            qt, okt = env._ik(torch.as_tensor(p_off, dtype=dtype), torch.as_tensor(r, dtype=dtype),
                              torch.as_tensor(seed, dtype=dtype))
            with self.subTest(dtype=str(dtype)):
                np.testing.assert_array_equal(ok, okt.numpy())
                # Solved, the float32 environment is within its own rounding of float64 (6e-7 rad); unsolved (pinned on
                # a limit) its polish wanders ~5e-5 rad per call in float32 -- the environment discards those.
                rows = slice(None) if dtype == torch.float64 else ok
                np.testing.assert_allclose(q[rows], qt.numpy()[rows], atol=tol)

    def test_gravity(self):
        g = insert_policy.ArmGravity(self.np_kin, self.consts["gravity"])
        gt = keyhold.ArmGravity(self.t64, wrist_payload=True)
        g_b = np.array([0.3, -0.2, -9.7])
        np.testing.assert_allclose(g.hold(self.q, g_b), gt.hold(torch.as_tensor(self.q), torch.as_tensor(g_b).expand(64, 3)).numpy(),
                                   atol=1e-12)

    def test_rotations(self):
        rng = np.random.default_rng(5)
        for _ in range(20):
            quat = rng.normal(size=4)
            quat /= np.linalg.norm(quat)
            v = rng.normal(size=3)
            tq, tv = torch.as_tensor(quat)[None], torch.as_tensor(v)[None]
            np.testing.assert_allclose(insert_policy.quat_apply(quat, v), interface.quat_apply(tq, tv)[0].numpy(), atol=1e-12)
            np.testing.assert_allclose(insert_policy.quat_rotate_inverse(quat, v),
                                       interface.quat_rotate_inverse(tq, tv)[0].numpy(), atol=1e-12)
            np.testing.assert_allclose(insert_policy.quat_matrix(quat), keyhold.quat_matrix(tq)[0].numpy(), atol=1e-12)
            np.testing.assert_allclose(insert_policy.yaw_quat(quat),
                                       interface.yaw_quat(interface.yaw_from_quat(tq))[0].numpy(), atol=1e-12)
            a, u = rng.normal(size=3), rng.normal(size=3)
            np.testing.assert_allclose(insert_policy.hand_rotation(a, u),
                                       keyhold.hand_rotation(torch.as_tensor(a)[None], torch.as_tensor(u)[None])[0].numpy(),
                                       atol=1e-12)


# --- the runner --------------------------------------------------------------------------------------------------

def _meta(tmp: Path, sag: bool = False) -> dict:
    """An insertion export's meta without a network: a fake training run beside a stand-in checkpoint file."""
    run = tmp / "run"
    run.mkdir(exist_ok=True)
    (run / "model_1.pt").write_bytes(b"not a network")
    (run / "run.json").write_text(json.dumps({"arguments": {"key_insert": True, "leg_variant": "force", "arm_servo": not sag,
                                                             "payload": "physical", "hold_arm": "policy"}, "leg": {}}))
    return json.loads(json.dumps(export_policy.meta_for("force", run / "model_1.pt", 1, interface.NUM_ACTIONS,
                                                        {"encoder": 4, "actor": 4, "decoder": 3}, task="insert",
                                                        train_run=run)))


def _scene():
    """A lock ahead of a standing base and to its right, as the key hold's frozen set places them (env frame)."""
    base_pos = np.array([0.0, 0.0, 0.30])
    yaw, pitch = 0.03, -0.02
    base_quat = np.array([math.cos(yaw / 2) * math.cos(pitch / 2), -math.sin(yaw / 2) * math.sin(pitch / 2),
                          math.cos(yaw / 2) * math.sin(pitch / 2), math.sin(yaw / 2) * math.cos(pitch / 2)])
    axis = np.array([1.0, 0.0, 0.0])
    u = np.array([0.0, -1.0, 0.0])
    lock = insert_policy.LockEstimate(np.array([0.505, -0.11, 0.45]), axis, u)
    return lock, base_pos, base_quat


def _kin_fixture():
    if not hasattr(Kinematics, "src"):
        Kinematics.setUpClass()
    return Kinematics


class Runner(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.meta = _meta(Path(cls.tmp.name))

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_meta_carries_the_training_constants(self):
        ins = self.meta["insert"]
        self.assertEqual(self.meta["task"], "insert")
        self.assertEqual(ins["env"]["INSERT_OBS_SCALE"], leg_cfg.KEYHOLD_OBS_SCALE)
        self.assertEqual(ins["env"]["IK_EVERY_STEPS"], leg_cfg.KEYHOLD_IK_EVERY_STEPS)
        self.assertAlmostEqual(ins["key"]["tip_from_jaw_m"], keylock.TIP_FROM_JAW_M)
        self.assertEqual(ins["layer"]["seat_test"], "old")
        self.assertFalse(ins["sag_compensation"])
        np.testing.assert_allclose(ins["commands"]["foot_goal_sphere"], leg_policy.waiting_command(self.meta)[3:6])

    def test_start_solve_is_the_environments(self):
        """`reset`'s start pose against `keyinsert_env._solve_start` (float32, as the env runs it)."""
        lock, base_pos, base_quat = _scene()
        runner = insert_policy.InsertRunner(self.meta)
        q, ok = runner.reset(lock, base_pos, base_quat, -0.0175, 8.0)
        self.assertTrue(ok)
        k = _kin_fixture()
        env = k._fake_env(k, k.t32, torch.float32, 1)
        f = lambda x: torch.as_tensor(np.asarray(x), dtype=torch.float32)[None]
        env._est_o, env._est_a, env._est_u = f(lock.origin), f(lock.axis), f(lock.u)
        env._start_depth, env._goal_jaw = torch.full((1,), -0.0175), torch.zeros(1, 3)
        env.scene = types.SimpleNamespace(env_origins=torch.zeros(1, 3))
        env._robot = types.SimpleNamespace(data=types.SimpleNamespace(root_pos_w=f(base_pos), root_quat_w=f(base_quat)))
        qt, okt = env._solve_start(torch.arange(1))
        self.assertTrue(bool(okt[0]))
        np.testing.assert_allclose(q, qt[0].numpy(), atol=2e-5)
        np.testing.assert_allclose(runner.goal_jaw, env._goal_jaw[0].numpy(), atol=1e-6)

    def test_sag_is_the_environments(self):
        folder = Path(self.tmp.name) / "sag"
        folder.mkdir()
        meta = _meta(folder, sag=True)
        lock, base_pos, base_quat = _scene()
        runner = insert_policy.InsertRunner(meta)
        self.assertTrue(runner.sag_on)
        q, _ = runner.reset(lock, base_pos, base_quat, -0.0175, 8.0)
        k = _kin_fixture()
        env = k._fake_env(k, k.t32, torch.float32, 1)
        env.cfg = types.SimpleNamespace(sag_compensation=True, arm_servo=False)
        env._gravity = keyhold.ArmGravity(k.t32, wrist_payload=True)
        env._arm_kp = torch.as_tensor(interface.P_GAINS[12:18], dtype=torch.float32)
        env._robot = types.SimpleNamespace(data=types.SimpleNamespace(
            root_quat_w=torch.as_tensor(base_quat, dtype=torch.float32)[None]))
        sag = env._sag(torch.as_tensor(q, dtype=torch.float32)[None])[0].numpy()
        np.testing.assert_allclose(runner.ins_ref - runner.ins_geo, sag, atol=1e-6)
        self.assertGreater(float(np.abs(sag).max()), 1e-3)

    def test_a_still_robot_waits_then_approaches(self):
        """Through the reset's frame and 60 steps, a robot that does not move: the layer starts at step 41 (the
        environment's `START_STEPS` 40 after the reset), the press stays zero until a touch, the frame is finite."""
        lock, base_pos, base_quat = _scene()
        runner = insert_policy.InsertRunner(self.meta)
        q, ok = runner.reset(lock, base_pos, base_quat, -0.0175, 8.0)
        legs = np.asarray(leg_cfg.SETTLED_LEG_POS)[runner.obs.to_unitree]
        zero_policy = types.SimpleNamespace(act=lambda stack: np.zeros((1, 18)))
        phases = []
        for k in range(61):
            out = runner.step(zero_policy, base_quat, np.zeros(3), legs, np.zeros(12), q, np.zeros(6), base_pos, base_quat)
            phases.append(out["phase"])
            self.assertTrue(np.isfinite(runner.last_frame).all())
            np.testing.assert_allclose(out["arm"], np.clip(runner.ins_ref + np.r_[0, 0, 0, 0, 0, runner.layer.roll[0]],
                                                           runner.lower, runner.upper))
        self.assertEqual(phases[40], "done")
        self.assertEqual(phases[41], "approach")
        self.assertEqual(runner.events[1]["event"], "start")
        self.assertEqual(float(runner.layer.force_cmd[0]), 0.0)
        np.testing.assert_allclose(runner.commands[9:12], 0.0)
        # The reset's frame has all-zero commands; the next ones carry the footfall.
        np.testing.assert_allclose(runner.commands[3:6], runner.foot_goal)

    def test_no_torch_on_the_dog(self):
        tree = ast.parse((REPO / "demos" / "combiner_key" / "robot" / "insert_policy.py").read_text())
        imported = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
        imported |= {n.module.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.module}
        self.assertLessEqual(imported, {"__future__", "math", "dataclasses", "types", "numpy", "demos"})


class Gate(unittest.TestCase):
    """`insert_gate.py`'s new pieces: the frozen set's boxes, the lock placed against them, the recorded attitude, and
    the closed-arm replay on synthetic frames."""

    @classmethod
    def setUpClass(cls):
        from demos.combiner_key.robot import insert_gate
        cls.G = insert_gate
        cls.tmp = tempfile.TemporaryDirectory()
        cls.meta = _meta(Path(cls.tmp.name))

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_the_key_hold_plan_and_no_side_effect(self):
        before = {k: v for k, v in vars(leg_cfg).items() if k.isupper()}
        boxes = self.G.key_hold_plan()
        self.assertEqual({k: v for k, v in vars(leg_cfg).items() if k.isupper()}, before)
        self.assertEqual(len(boxes["door_ahead"]), 96)
        for value in boxes["door_ahead"]:
            self.assertTrue(any(abs(value - v) < 1e-6 for v in leg_cfg.KEYHOLD_EVAL_DOOR_AHEAD_M))
        for value in boxes["height"]:
            self.assertTrue(any(abs(value - v) < 1e-6 for v in leg_cfg.KEYHOLD_EVAL_AXIS_HEIGHT_M))
        self.assertLessEqual(float(np.abs(boxes["door_yaw"]).max()), leg_cfg.KEYHOLD_EVAL_DOOR_YAW_RAD + 1e-6)

    def test_lock_estimate(self):
        box = {"door_ahead": 0.51, "axis_side": -0.11, "height": 0.45, "door_yaw": 0.0}
        lock = self.G.lock_estimate(box, keylock.LOCK_FACE_OFF_DOOR_M)
        np.testing.assert_allclose(lock.axis, [1.0, 0.0, 0.0], atol=1e-12)        # into the door
        np.testing.assert_allclose(lock.u, [0.0, -1.0, 0.0], atol=1e-12)           # the bar to the robot's right
        np.testing.assert_allclose(lock.origin, [0.51 - keylock.LOCK_FACE_OFF_DOOR_M, -0.11, 0.45], atol=1e-12)
        turned = self.G.lock_estimate(dict(box, door_yaw=0.04), keylock.LOCK_FACE_OFF_DOOR_M)
        basis = np.stack((turned.u, turned.t, turned.axis))
        np.testing.assert_allclose(basis @ basis.T, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(float(turned.origin[1]), -0.11 + keylock.LOCK_FACE_OFF_DOOR_M * math.sin(0.04) * -1, 12)

    def test_attitude(self):
        for roll, pitch, yaw in ((0.03, -0.06, 0.0), (-0.02, 0.05, 0.07)):
            q = self.G.quat_rpy(roll, pitch, yaw)
            np.testing.assert_allclose(leg_policy.roll_pitch(q), (roll, pitch), atol=1e-12)
            np.testing.assert_allclose(insert_policy.yaw_quat(q)[[0, 3]], (math.cos(yaw / 2), math.sin(yaw / 2)), atol=1e-12)
        # At zero yaw, `leg_env`'s spawn quaternion.
        r, p = 0.03, -0.06
        cr, sr, cp, sp = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2)
        np.testing.assert_allclose(self.G.quat_rpy(r, p, 0.0), (cr * cp, sr * cp, cr * sp, -sr * sp), atol=1e-12)

    def test_replay_on_a_still_stand(self):
        """A still settled stand, a policy that asks nothing: the start solves, every IK call solves, the layer starts at
        step 41 and approaches past the face without a touch, the frame's legs and IMU are the recorded ones."""
        n = 260
        frame = np.zeros((n, 76))
        frame[:, 5:17] = np.asarray(leg_cfg.SETTLED_LEG_POS) - np.asarray(interface.DEFAULT_DOF_POS[:12])
        rows = {"frame": frame, "rpy": np.zeros((n, 3))}
        state = self.G.frames_state(self.meta, rows)
        policy = types.SimpleNamespace(meta=self.meta, act=lambda stack: np.zeros((1, 18)))
        box = {"name": "median", "door_ahead": 0.51, "axis_side": -0.11, "height": 0.45, "door_yaw": 0.0}
        out = self.G.replay(policy, state, box)
        self.assertEqual(out["ik_fail"], 0)
        self.assertGreater(out["ik_calls"], 40)
        self.assertEqual(out["touches_short_of_face"], [])
        self.assertEqual(out["decisions"][1]["event"], "start")
        self.assertEqual(out["decisions"][1]["step"], 41)
        self.assertIsNotNone(out["ref_passes_face_s"])
        self.assertLess(out["replay_frame_err"], 1e-12)
        self.assertEqual(out["legs_rad_max_first_s"], 0.0)
        self.assertTrue(out["pass"])


@unittest.skipUnless(importlib.util.find_spec("rsl_rl") is not None, "the export loads checkpoints through rsl-rl")
class Export(unittest.TestCase):
    def test_export_of_a_random_insertion_actor(self):
        from tensordict import TensorDict

        from unifp_train import observations
        from unifp_train.models import UniFPActor

        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            meta = _meta(tmp)                      # writes the fake run
            run = tmp / "run"
            torch.manual_seed(5)
            probe = TensorDict({"policy": torch.zeros(1, interface.NUM_OBS),
                                "critic": torch.zeros(1, observations.NUM_CRITIC_OBS)}, batch_size=[1])
            actor = UniFPActor(probe, {"actor": ["policy"], "critic": ["critic"]}, "actor", 18,
                               distribution_cfg={"class_name": "GaussianDistribution", "init_std": 1.0, "std_type": "scalar"})
            ckpt = run / "model_1.pt"
            torch.save({"actor_state_dict": actor.state_dict(), "iter": 1}, ckpt)
            result = export_policy.export(ckpt, "force", tmp / "p.npz", task="insert", train_run=run)
            self.assertLess(result["action_max_abs_err"], 1e-5)
            policy = leg_policy.LegPolicy(result["out"])
            self.assertEqual(policy.meta["task"], "insert")
            self.assertEqual(policy.meta["insert"]["layer"], meta["insert"]["layer"])
            with self.assertRaises(ValueError):
                export_policy.export(ckpt, "position", tmp / "q.npz", task="insert", train_run=run)


if __name__ == "__main__":
    unittest.main()
