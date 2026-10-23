"""The dog's foot-press program (`demos/combiner_key/robot/press_layer.py`, `lowlevel.py`, `press_live.py`) against the
training code and Unitree's reference, without a simulator or ROS.

The press layer's numpy port runs beside `unifp_train.foot_press.PressLayer` on the same closed-loop foot paths; the
program runs whole on a fake clock against a kinematic stand-in for the robot and its motion switcher, live and dry.
None of this is the policy driving a robot: the stand-in has no dynamics.
"""
import ast
import json
import math
import re
import tempfile
import types
import unittest
from pathlib import Path

import numpy as np
import torch

from demos.combiner_key.robot import leg_policy, lowlevel as ll, press_layer as pl, press_live
from unifp_train import foot_press, foot_press_cfg as pc, handle, leg_cfg, task_cfg
from unifp_isaaclab import interface

REPO = Path(__file__).resolve().parents[1]
ROBOT = REPO / "demos" / "combiner_key" / "robot"
POLICY = REPO / "checkpoints" / "leg_press_force_r4_model_5999.npz"


class RepeatedConstants(unittest.TestCase):
    def test_against_their_sources(self):
        self.assertEqual(pl.POLICY_DT, interface.POLICY_DT)
        self.assertEqual(pl.FOOT_FORCE_KP, leg_cfg.FOOT_FORCE_KP)
        self.assertEqual((pl.LIFT_S, pl.OVER_S, pl.LIFT_FIRST_SHARE), (leg_cfg.LIFT_S, leg_cfg.OVER_S, leg_cfg.LIFT_FIRST_SHARE))
        self.assertEqual((pl.FOOT_CORRECTION_GAIN, pl.FOOT_CORRECTION_MAX_M),
                         (leg_cfg.FOOT_CORRECTION_GAIN, leg_cfg.FOOT_CORRECTION_MAX_M))
        self.assertEqual((pl.ENGAGE_CONVERGED_M, pl.ENGAGE_SETTLE_SPEED_M_S, pl.ENGAGE_NEAR_M, pl.ENGAGE_MAX_WAIT_STEPS),
                         (leg_cfg.ENGAGE_CONVERGED_M, leg_cfg.ENGAGE_SETTLE_SPEED_M_S, leg_cfg.ENGAGE_NEAR_M,
                          leg_cfg.ENGAGE_MAX_WAIT_STEPS))
        self.assertEqual(pl.FOOT_RADIUS_M, handle.FOOT_RADIUS_M)
        self.assertEqual((pl.IDLE, pl.PRESSING, pl.HOLDING, pl.RELEASING, pl.DONE),
                         (handle.IDLE, handle.PRESSING, handle.HOLDING, handle.RELEASING, handle.DONE))
        self.assertEqual((pl.CMD_EE_RADIUS, pl.CMD_EE_FORCE), (interface.CMD_EE_RADIUS, interface.CMD_EE_FORCE))
        # leg_env imports Isaac Lab: its constants are read from the source.
        src = (REPO / "unifp_train" / "leg_env.py").read_text()
        line = re.search(r"^WAITING, LIFTING, OVER, AT_PRE, ENGAGED, MISSED, RETURNING, FOOT_DOWN = (.*)$", src, re.M)
        self.assertEqual(tuple(int(v) for v in line.group(1).split(",")),
                         (pl.WAITING, pl.LIFTING, pl.OVER, pl.AT_PRE, pl.ENGAGED, pl.MISSED, pl.RETURNING, pl.FOOT_DOWN))
        self.assertIn("self._start_delay[ids] = 25", src)
        self.assertEqual(pl.EVAL_START_DELAY_STEPS, 25)

    def test_joint_limits_against_task_cfg(self):
        names = list(interface.DOF_NAMES)
        for k, name in enumerate(leg_policy.UNITREE_LEGS):
            self.assertEqual(tuple(press_live.JOINT_LIMITS[k]), task_cfg.JOINT_POS_LIMITS[names.index(name)], name)


def run_layers(n, *, plate_k=None, plate_h=0.025, levels=(8.0,), ramps=(0.5,), holds=(1.0,), steps=400, max_force=35.0,
               press_without_stall=True, cap_start=False):
    """Torch's layer and the numpy one, each driving its own kinematic foot toward goal + force / k (stopped by a plane
    `plate_h` in from the pre-contact point: rigid if `plate_k` is None, else a spring of `plate_k`)."""
    n = pl.unit(n)
    pre = np.array([0.33, -0.2, 0.15])
    # In float64: torch's layer keeps its timers in float32, where 75 steps of 0.02 s fall short of 1.5 s and a hold
    # ends one step later than numpy's (the dog runs float64).
    dtype = torch.get_default_dtype()
    torch.set_default_dtype(torch.float64)
    try:
        return _run_layers(n, pre, plate_k, plate_h, levels, ramps, holds, steps, max_force, press_without_stall, cap_start)
    finally:
        torch.set_default_dtype(dtype)


def _run_layers(n, pre, plate_k, plate_h, levels, ramps, holds, steps, max_force, press_without_stall, cap_start):
    standoff, speed = 0.04, 0.03
    t_layer = foot_press.PressLayer(1, "cpu")
    k = foot_press.MAX_SEGMENTS
    pad = lambda v: torch.tensor([list(v) + [v[-1]] * (k - len(v))], dtype=torch.float64)
    f = lambda v: torch.tensor([v], dtype=torch.float64)
    t_layer.start(torch.tensor([0]), pre=f(pre), n=f(n), standoff=torch.tensor([standoff]), speed=torch.tensor([speed]),
                  levels=pad(levels), ramps=pad(ramps), holds=pad(holds), segments=torch.tensor([len(levels)]),
                  tan_ratio=torch.zeros(1, k), tan_dir=torch.zeros(1, 3), slide=torch.tensor([False]),
                  slide_dir=torch.zeros(1, 3), slide_speed=torch.zeros(1), slide_dist=torch.zeros(1),
                  force_law=torch.tensor([True]), foot=f(pre))
    n_layer = pl.PressLayer(max_force_n=max_force, press_without_stall=press_without_stall, force_cap_start=cap_start)
    n_layer.start(pre=pre, n=n, profile=pl.Profile(levels=list(levels), ramps=list(ramps), holds=list(holds),
                                                   standoff=standoff, speed=speed), foot=pre)
    face = pre - plate_h * n
    feet = {"torch": pre.astype(np.float64).copy(), "numpy": pre.copy()}
    out = {"torch": [], "numpy": []}

    def advance(name, goal, force):
        p = feet[name]
        target = goal + force / pl.FOOT_FORCE_KP
        p = p + 0.3 * (target - p)
        h = float((p - face) @ n)
        if h < 0.0:
            if plate_k is None:
                p = p - h * n
            else:   # the spring pushes back: the foot settles where k_s * depth balances the commanded force
                depth = min(-h, float(np.linalg.norm(force)) / plate_k)
                p = p - (h + depth) * n
        feet[name] = p

    for _ in range(steps):
        g, fc = t_layer.step(torch.tensor(np.array([feet["torch"]]), dtype=torch.float64), pl.POLICY_DT)
        g, fc = g[0].double().numpy(), fc[0].double().numpy()
        out["torch"].append((g, fc, int(t_layer.phase[0]), float(t_layer.normal_level[0])))
        advance("torch", g, fc)
        g2, fc2 = n_layer.step(feet["numpy"], pl.POLICY_DT)
        out["numpy"].append((g2, fc2, n_layer.phase, n_layer.normal_level))
        advance("numpy", g2, fc2)
    return out, t_layer, n_layer


class LayerAgainstTorch(unittest.TestCase):
    def check(self, out, tol=1e-9):
        for k, ((g1, f1, p1, l1), (g2, f2, p2, l2)) in enumerate(zip(out["torch"], out["numpy"])):
            self.assertEqual(p1, p2, f"phase at step {k}")
            np.testing.assert_allclose(g2, g1, atol=tol, err_msg=f"goal at step {k}")
            np.testing.assert_allclose(f2, f1, atol=1e-9, err_msg=f"force at step {k}")
            self.assertAlmostEqual(l1, l2, delta=1e-9)

    def test_rigid_plate_facing_up(self):
        out, t, n = run_layers((0, 0, 1))
        self.check(out)
        self.assertTrue(bool(t.stalled[0]) and n.stalled)
        self.assertEqual(n.phase, pl.DONE)

    def test_wall_three_segments(self):
        out, t, n = run_layers((-1, -0.06, 0), levels=(5.0, 12.0, 4.0), ramps=(0.3, 0.6, 0.4), holds=(0.8, 1.0, 0.5),
                               steps=500)
        self.check(out)
        self.assertTrue(n.stalled)

    def test_sprung_plate(self):
        out, t, n = run_layers((0, 0, 1), plate_k=1000.0, levels=(16.0,))
        self.check(out)

    def test_nothing_there_presses_in_training_but_not_on_the_dog(self):
        out, t, n = run_layers((0, 0, 1), plate_h=1.0)        # the plate far beyond the travel
        self.check(out)
        self.assertFalse(bool(t.stalled[0]))
        self.assertGreater(max(r[3] for r in out["torch"]), 7.9)  # the trained layer presses the air at the end
        dog = pl.PressLayer()
        dog.start(pre=[0.33, -0.2, 0.15], n=[0, 0, 1], profile=pl.Profile(), foot=[0.33, -0.2, 0.15])
        levels = []
        for _ in range(300):
            dog.step(dog.goal, pl.POLICY_DT)         # the foot exactly on its goal: nothing stops it
            levels.append(dog.normal_level)
        self.assertTrue(dog.ran_out)
        self.assertEqual(max(levels), 0.0)
        self.assertEqual(dog.phase, pl.DONE)

    def test_the_cap(self):
        out, _, n = run_layers((0, 0, 1), levels=(20.0,), max_force=5.0, cap_start=True, press_without_stall=False)
        levels = [r[3] for r in out["numpy"]]
        forces = [float(np.linalg.norm(r[1])) for r in out["numpy"]]
        self.assertLessEqual(max(levels), 5.0 + 1e-9)
        self.assertLessEqual(max(forces), 5.0 + 1e-9)
        self.assertTrue(n.stalled)


class Plates(unittest.TestCase):
    def test_the_hand_plates_are_table_rows(self):
        table = foot_press.PlateTable().columns
        for name, plate in pl.PLATES.items():
            self.assertEqual(pl.check_plate(plate), [], name)
            kind = {"palm_up": 2, "palm_facing": 1}[name]
            rows = table["kind"] == kind
            near = np.linalg.norm(table["contact"][rows] - plate.contact, axis=1) < 0.02
            aligned = table["n"][rows][near] @ plate.n > math.cos(math.radians(5))
            self.assertGreaterEqual(int(aligned.sum()), 5, name)

    def test_pushing_up_is_refused(self):
        plate = pl.Plate(contact=(0.33, -0.2, 0.2), n=(0, 0, -1))
        self.assertTrue(any("push up" in p for p in pl.check_plate(plate)))
        with self.assertRaises(ValueError):
            pl.PressTask(plate, pl.Profile(), down_goal=leg_cfg.KEY_DOWN_GOAL, goal_centre=leg_cfg.GOAL_CENTRE)

    def test_out_of_reach_is_refused(self):
        self.assertTrue(pl.check_plate(pl.Plate(contact=(0.6, -0.2, 0.1), n=(0, 0, 1))))


def kinematic_episode(task, *, hand=True, go_at=(30, None), steps=1500, lag=0.25):
    """A foot that follows its commanded goal (plus force / k) with a lag, stopped by the operator's hand at the plate."""
    foot = np.array(leg_cfg.KEY_DOWN_GOAL, dtype=float)
    face, n = task.plate.contact, task.plate.n
    history = []
    at_pre_since = None
    for k in range(steps):
        go = False
        if task.stage == pl.WAITING:
            go = k >= go_at[0]
        elif task.stage == pl.AT_PRE:
            at_pre_since = k if at_pre_since is None else at_pre_since
            go = go_at[1] is not None and k - at_pre_since >= go_at[1]
        last = foot.copy()
        cmd = task.step(foot, (foot - history[-1]["foot"]) / pl.POLICY_DT if history else np.zeros(3), go=go)
        goal = task.commanded_goal()
        target = goal + cmd[pl.CMD_EE_FORCE] / pl.FOOT_FORCE_KP
        foot = foot + lag * (target - foot)
        if hand and (foot - face) @ n < 0.0:
            foot = foot - float((foot - face) @ n) * n
        history.append({"foot": last, "cmd": cmd, "stage": task.stage, "phase": task.layer.phase,
                        "level": task.layer.normal_level, "force": task.layer.force_cmd.copy()})
        if task.stage == pl.FOOT_DOWN:
            break
    return history


class Task(unittest.TestCase):
    def new(self, **kw):
        return pl.PressTask(pl.PLATES["palm_up"], pl.Profile(), down_goal=leg_cfg.KEY_DOWN_GOAL,
                            goal_centre=leg_cfg.GOAL_CENTRE, **kw)

    def test_a_press_on_the_hand(self):
        task = self.new()
        h = kinematic_episode(task, go_at=(30, 50))
        self.assertEqual(task.stage, pl.FOOT_DOWN)
        self.assertTrue(task.layer.stalled)
        self.assertIsNone(task.ended_unpressed)
        stages = [r["stage"] for r in h]
        for s in (pl.WAITING, pl.LIFTING, pl.OVER, pl.AT_PRE, pl.ENGAGED, pl.RETURNING):
            self.assertIn(s, stages)
        levels = [r["level"] for r in h]
        self.assertAlmostEqual(max(levels), 5.0, places=6)
        self.assertLessEqual(max(np.linalg.norm(r["force"]) for r in h), 5.0 + 1e-9)
        # The force pushes down onto the palm, and the frame carries it one step behind the layer.
        held = [r for r in h if r["phase"] == pl.HOLDING]
        self.assertTrue(all(r["force"][2] < 0 and abs(r["force"][0]) < 1e-9 for r in held))
        for a, b in zip(h, h[1:]):
            np.testing.assert_allclose(b["cmd"][pl.CMD_EE_FORCE], a["force"])

    def test_waiting_command_is_the_policy_exports(self):
        task = self.new()
        cmd = task.step(np.array(leg_cfg.KEY_DOWN_GOAL), np.zeros(3))
        meta = {"key_down_goal": list(leg_cfg.KEY_DOWN_GOAL), "goal_centre": list(leg_cfg.GOAL_CENTRE), "num_commands": 15,
                "cmd_ee_radius": interface.CMD_EE_RADIUS}
        np.testing.assert_allclose(cmd, leg_policy.waiting_command(meta), atol=1e-12)

    def test_no_go_brings_the_foot_home(self):
        task = self.new(hand_wait_s=2.0)
        kinematic_episode(task, go_at=(30, None))
        self.assertEqual(task.stage, pl.FOOT_DOWN)
        self.assertEqual(task.ended_unpressed, "no go")

    def test_no_hand_backs_out_unpressed(self):
        task = self.new()
        h = kinematic_episode(task, hand=False, go_at=(30, 10))
        self.assertEqual(task.stage, pl.FOOT_DOWN)
        self.assertEqual(task.ended_unpressed, "no stall")
        self.assertEqual(max(r["level"] for r in h), 0.0)

    def test_release_now(self):
        # Driven to the hold, then stopped.
        task = self.new()
        hist = []
        foot = np.array(leg_cfg.KEY_DOWN_GOAL, dtype=float)
        k_at = None
        for k in range(1500):
            go = (task.stage == pl.WAITING and k > 30) or task.stage == pl.AT_PRE
            cmd = task.step(foot, np.zeros(3), go=go)
            foot = foot + 0.25 * (task.commanded_goal() + cmd[pl.CMD_EE_FORCE] / 400.0 - foot)
            n = task.plate.n
            if (foot - task.plate.contact) @ n < 0:
                foot = foot - float((foot - task.plate.contact) @ n) * n
            if task.layer.phase == pl.HOLDING and task.layer.normal_level >= 4.9 and k_at is None:
                k_at = k
                task.release_now(foot)
            hist.append(task.layer.normal_level)
            if task.stage == pl.FOOT_DOWN:
                break
        self.assertIsNotNone(k_at)
        self.assertEqual(task.stage, pl.FOOT_DOWN)
        self.assertLess(hist[k_at + round(pc.RELEASE_FORCE_S / pl.POLICY_DT) + 2], 1e-9)


class Crc(unittest.TestCase):
    # Unitree's crc32_core (unitree_ros2 example/src/src/common/motor_crc.cpp) compiled with gcc on 2026-10-23.
    def lcg(self):
        s, out = 12345, []
        for _ in range(202):
            s = (s * 1103515245 + 12345) & 0xFFFFFFFF
            out.append(s)
        return out

    def test_against_unitrees_c(self):
        self.assertEqual(ll.crc32_words(self.lcg()), 279564839)
        self.assertEqual(ll.crc32_words([0] * 202), 1264208000)
        rng = np.random.default_rng(1)
        words = [int(v) for v in rng.integers(0, 2 ** 32, 50, dtype=np.uint64)]
        self.assertEqual(ll.crc32_words(words), ll.crc32_core_unitree(words))

    def test_a_lowcmd_against_unitrees_get_crc(self):
        # The same message through motor_crc.h's struct and crc32_core in C++: 736096623.
        msg = fake_lowcmd()
        ll.init_lowcmd(msg)
        ll.fill_legs(msg, [0.25 * i for i in range(12)], 25.0, 0.5)
        self.assertEqual(len(ll.pack_lowcmd(msg)), 812)
        self.assertEqual(msg.crc, 736096623)

    def test_remote_and_modes(self):
        raw = [0] * 40
        raw[3] = 0b10                 # B
        self.assertEqual(ll.remote_keys(raw), {"B"})
        raw[2] = 0b100000             # L2
        self.assertEqual(ll.remote_keys(raw), {"B", "L2"})
        self.assertEqual(ll.mode_name('{"name":"normal","form":"0"}'), "normal")
        self.assertEqual(ll.mode_name('{"name":""}'), "")
        self.assertEqual(ll.mode_name(""), "")


def fake_lowcmd():
    N = types.SimpleNamespace
    return N(head=[0, 0], level_flag=0, frame_reserve=0, sn=[0, 0], version=[0, 0], bandwidth=0,
             motor_cmd=[N(mode=0, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0, reserve=[0, 0, 0]) for _ in range(20)],
             bms_cmd=N(off=0, reserve=[0, 0, 0]), wireless_remote=[0] * 40, led=[0] * 12, fan=[0, 0], gpio=0,
             reserve=0, crc=0)


SETTLED_UNITREE = np.array(leg_cfg.SETTLED_LEG_POS)[[interface.DOF_NAMES.index(n) for n in leg_policy.UNITREE_LEGS]]


class Kinematics(unittest.TestCase):
    def setUp(self):
        self.meta = leg_policy.LegPolicy(POLICY).meta if POLICY.exists() else None
        if self.meta is None:
            self.skipTest("no exported press policy")

    def test_the_settled_stance(self):
        quat = (math.cos(0.4), 0.0, 0.0, math.sin(0.4))               # a yaw changes nothing
        feet, height = press_live.feet(SETTLED_UNITREE, quat, self.meta)
        self.assertAlmostEqual(height, leg_cfg.SETTLED_BASE_HEIGHT_M, delta=0.01)
        np.testing.assert_allclose(feet["FR"], leg_cfg.KEY_DOWN_GOAL, atol=0.02)

    def test_a_pitched_base(self):
        a = 0.1                                                         # nose down 0.2 rad about y
        quat = (math.cos(a), 0.0, math.sin(a), 0.0)
        feet, height = press_live.feet(SETTLED_UNITREE, quat, self.meta)
        self.assertAlmostEqual(float(np.mean([feet[l][2] for l in press_live.SUPPORT])), pl.FOOT_RADIUS_M, places=9)

    def test_foot_velocity(self):
        q, dq = SETTLED_UNITREE.copy(), np.zeros(12)
        dq[0:3] = (0.3, -0.5, 0.8)
        v = press_live.fr_foot_velocity(q, dq, (1, 0, 0, 0), self.meta)
        h = 1e-6
        fd = (leg_policy.foot_local("FR", q[0:3] + h * dq[0:3], self.meta) - leg_policy.foot_local("FR", q[0:3], self.meta)) / h
        np.testing.assert_allclose(v, fd, atol=1e-5)


class OnlyLiveIOPublishes(unittest.TestCase):
    def test_source(self):
        tree = ast.parse((ROBOT / "press_live.py").read_text())
        for cls in [n for n in tree.body if isinstance(n, ast.ClassDef)]:
            for node in ast.walk(cls):
                if isinstance(node, ast.Attribute) and node.attr in ("create_publisher", "publish"):
                    self.assertEqual(cls.name, "LiveIO", f"{node.attr} in {cls.name}")
        for fn in [n for n in tree.body if isinstance(n, ast.FunctionDef)]:
            for node in ast.walk(fn):
                if isinstance(node, ast.Attribute) and node.attr in ("create_publisher", "publish"):
                    self.fail(f"{node.attr} in {fn.name}")
        # Ctrl-C must stay a KeyboardInterrupt: rclpy's own handler shuts the context and the writer with it.
        self.assertIn("rclpy.init(signal_handler_options=SignalHandlerOptions.NO)", (ROBOT / "press_live.py").read_text())
        for name in ("press_live.py", "press_layer.py", "lowlevel.py"):
            for node in ast.walk(ast.parse((ROBOT / name).read_text())):
                if isinstance(node, (ast.Import, ast.ImportFrom)):
                    mods = [a.name for a in node.names] if isinstance(node, ast.Import) else [node.module or ""]
                    self.assertFalse(any(m.split(".")[0] == "torch" for m in mods), name)


# --- the program on a fake clock -------------------------------------------------------------------------------------

class Clock:
    def __init__(self):
        import time as real
        self.real, self.now = real, 1000.0

    def monotonic(self):
        return self.now

    def __getattr__(self, name):
        return getattr(self.real, name)


#: The dog lying in sport mode, read from its LowState on 2026-10-23: the calves past the URDF's -2.7227.
DOG_LYING_Q = np.array([-0.046, 1.253, -2.79, 0.081, 1.247, -2.78, -0.346, 1.271, -2.805, 0.302, 1.295, -2.847])


class FakeRobot:
    """Joints that follow the commanded targets (first order, faster with more kp), sag under damping, lie down on
    sport mode's StandDown; a motion switcher that releases and selects; the operator's hand under the FR foot."""

    def __init__(self, meta, plate=None, sag=1.0):
        self.meta, self.plate = meta, plate
        # Gravity as the policy's PD meets it: at kp 25 the default targets settle into the trained settled stance, at
        # other gains the joints sag by that torque over kp (`sag` scales the load).
        obs = leg_policy.LegObservation(meta)
        self.load = sag * 25.0 * (np.asarray(meta["settled_leg_pos"])[obs.to_unitree] - obs.default[:12][obs.to_unitree])
        self.error = None
        self.q = SETTLED_UNITREE.copy() + np.tile([0.0, 0.05, 0.0], 4)
        self.dq = np.zeros(12)
        self.mode, self.lying = "normal", False
        self.target, self.kp, self.kd = None, np.zeros(12), np.zeros(12)
        self.responses, self.sent, self.writing, self.listening, self.foreign_lowcmd = {}, 0, False, False, 0
        self.lowcmd_while_sport = 0
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.keys = []
        self.ids = 0
        self.log = []

    # The LiveIO interface.
    def switcher(self, api_id, parameter=None):
        self.ids += 1
        if api_id == ll.RELEASE_MODE:
            self.mode = ""
        elif api_id == ll.SELECT_MODE:
            self.mode = parameter["name"]
        self.responses[self.ids] = (0, json.dumps({"name": self.mode}))
        self.log.append(("switcher", api_id, parameter))
        return self.ids

    def sport_request(self, api_id):
        self.log.append(("sport", api_id))
        if api_id == ll.SPORT_STAND_DOWN:
            self.lying = True

    def command(self, q, kp, kd):
        if self.mode:
            self.lowcmd_while_sport += 1
        msg = fake_lowcmd()
        ll.init_lowcmd(msg)
        ll.fill_legs(msg, q, kp, kd)
        assert msg.crc == ll.lowcmd_crc(msg)
        self.target, self.kp, self.kd = np.asarray(q, float), np.broadcast_to(kp, (12,)), np.broadcast_to(kd, (12,))
        self.sent += 1
        self.writing = True

    def stop_writing(self):
        self.writing = False

    def step(self, dt=0.02):
        before = self.q.copy()
        if self.mode and self.lying:
            self.q += (DOG_LYING_Q - self.q) * 0.1
        elif self.mode:
            pass                                   # sport mode holds its stand
        elif self.target is not None and self.kp.max() > 0:
            rate = np.clip(self.kp / 25.0 * 0.3, 0.0, 0.9)
            self.q += (self.target + self.load / np.maximum(self.kp, 1e-6) - self.q) * rate
        else:
            self.q += (DOG_LYING_Q - self.q) * 0.02  # damping: it sinks
        if self.plate is not None:
            feet, _ = press_live.feet(self.q, self.quat, self.meta)
            if (feet["FR"] - self.plate.contact) @ self.plate.n < 0.0:
                self.q[0:3] = before[0:3]          # the hand stops the foot
        self.dq = (self.q - before) / dt

    def msg(self):
        N = types.SimpleNamespace
        raw = [0] * 40
        for key in self.keys:
            bit = ll.KEYS.index(key)
            raw[2 + bit // 8] |= 1 << (bit % 8)
        return N(motor_state=[N(q=float(self.q[i]), dq=float(self.dq[i]), tau_est=0.0) for i in range(12)],
                 imu_state=N(quaternion=list(self.quat), gyroscope=[0.0, 0.0, 0.0], rpy=[0.0, 0.0, 0.0]),
                 foot_force=[20, 20, 20, 20], wireless_remote=raw)


class FakeOperator:
    def __init__(self):
        self.lines = []

    def drain(self):
        out, self.lines = self.lines, []
        return out


def args(**kw):
    base = dict(policy=str(POLICY), live=False, rehearse=False, plate="palm_up", force_n=5.0, max_force_n=5.0,
                ramp_s=0.5, hold_s=1.0, standoff=0.04, speed=0.03, hand_wait_s=15.0, presses=1, policy_s=90.0,
                tilt_limit_deg=15.0, force_abort_n=1e9, stand_s=10.0, cmd_hz=500.0, topic="/lowstate", arm_link_port=0, print_s=1e9,
                run_dir=None)
    base.update(kw)
    return types.SimpleNamespace(**base)


@unittest.skipUnless(POLICY.exists(), "no exported press policy")
class Program(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.clock = Clock()
        self.saved_time = press_live.time
        press_live.time = self.clock

    def tearDown(self):
        press_live.time = self.saved_time
        self.tmp.cleanup()

    def drive(self, run, robot, operator, *, until, steps=6000, script=None):
        for k in range(steps):
            robot.step()
            run.on_state(robot.msg())
            if script:
                script(k, run, robot, operator)
            run.tick()
            self.clock.now += 0.02
            if until(run):
                return k
        self.fail(f"not reached: mode {run.mode}, stage {pl.STAGE_NAMES[run.task.stage]}")

    @staticmethod
    def enter_on(run, operator, states):
        """Enter whenever the program waits on one of `states` (mode or task stage names)."""
        waiting = run.mode in states or (run.mode == "policy" and pl.STAGE_NAMES[run.task.stage] in states)
        if waiting and not run.go:
            operator.lines.append("live" if run.confirming else "")

    def test_dry_rehearsal_walks_a_press(self):
        robot, op = FakeRobot(leg_policy.LegPolicy(POLICY).meta), FakeOperator()
        run = press_live.PressRun(args(rehearse=True, run_dir=self.tmp.name), None, op)
        self.drive(run, robot, op, until=lambda r: r.mode == "done",
                   script=lambda k, r, rb, o: k % 30 == 0 and self.enter_on(r, o, ("waiting", "at_pre")))
        self.assertEqual(robot.sent, 0)
        self.assertEqual(len(run.press_results), 1)
        self.assertTrue(run.press_results[0]["stalled"])
        out = run.save()
        summary = json.loads((out / "summary.json").read_text())
        self.assertEqual(summary["lowcmd_sent"], 0)
        self.assertAlmostEqual(summary["commanded_level_max_n"], 5.0, places=6)
        self.assertTrue((out / "press.npz").exists())

    def test_live_whole_run(self):
        meta = leg_policy.LegPolicy(POLICY).meta
        robot, op = FakeRobot(meta), FakeOperator()
        run = press_live.PressRun(args(live=True, run_dir=self.tmp.name), robot, op)
        modes = []
        self.drive(run, robot, op, until=lambda r: r.mode == "done",
                   script=lambda k, r, rb, o: (modes.append(r.mode) if not modes or modes[-1] != r.mode else None,
                                               k % 30 == 0 and self.enter_on(r, o, ("preflight", "damping", "waiting", "at_pre"))))
        self.assertEqual(modes[:6], ["preflight", "lie_down", "release", "check_lowcmd", "stand_up", "policy"])
        self.assertEqual(modes[-4:], ["stand_end", "lie_end", "damping", "handback"])
        self.assertEqual(run.mode, "done")
        self.assertEqual(robot.lowcmd_while_sport, 0)          # never a LowCmd with sport mode on
        self.assertEqual(robot.mode, "normal")                 # given back
        self.assertIsNone(run.abort_reason)              # lying past the calves' limit stops nothing
        self.assertIn("settled", [e["event"] for e in run.events])
        self.assertEqual(len(run.press_results), 1)
        self.assertEqual([e[1] for e in robot.log if e[0] == "switcher"][:2], [ll.CHECK_MODE, ll.RELEASE_MODE])
        a = {k: np.array(v) for k, v in run.rows.items()}
        self.assertLessEqual(float(np.nanmax(a["level"])), 5.0 + 1e-9)
        self.assertTrue(np.all(a["kp"][a["mode"] == press_live.DRIVEN.index("policy")] == 25.0))

    def test_live_stand_only(self):
        robot, op = FakeRobot(leg_policy.LegPolicy(POLICY).meta), FakeOperator()
        run = press_live.PressRun(args(live=True, presses=0, stand_s=3.0, run_dir=self.tmp.name), robot, op)
        self.drive(run, robot, op, until=lambda r: r.mode == "done",
                   script=lambda k, r, rb, o: k % 30 == 0 and self.enter_on(r, o, ("preflight", "waiting", "damping")))
        self.assertIsNone(run.abort_reason)
        self.assertEqual(run.end_reason, "--stand_s 3.0 s")
        stages = set(np.array(run.rows["stage"])[np.array(run.rows["mode"]) == press_live.DRIVEN.index("policy")])
        self.assertEqual(stages, {pl.WAITING})              # Enter never lifts the foot
        self.assertEqual(robot.mode, "normal")

    def test_a_stand_not_the_trained_one_lies_down(self):
        robot, op = FakeRobot(leg_policy.LegPolicy(POLICY).meta, sag=2.0), FakeOperator()   # twice the load
        run = press_live.PressRun(args(live=True, run_dir=self.tmp.name), robot, op)
        modes = []
        self.drive(run, robot, op, until=lambda r: r.mode == "done",
                   script=lambda k, r, rb, o: (modes.append(r.mode) if not modes or modes[-1] != r.mode else None,
                                               k % 30 == 0 and self.enter_on(r, o, ("preflight", "damping"))))
        self.assertNotIn("policy", modes)
        self.assertIn("not the trained one", run.abort_reason)
        self.assertEqual(modes[-4:], ["stand_end", "lie_end", "damping", "handback"])

    def test_live_needs_the_word(self):
        robot, op = FakeRobot(leg_policy.LegPolicy(POLICY).meta), FakeOperator()
        run = press_live.PressRun(args(live=True, run_dir=self.tmp.name), robot, op)
        for _ in range(200):
            robot.step()
            run.on_state(robot.msg())
            if run.confirming and not op.lines:
                op.lines.append("")              # a bare Enter is not the confirmation
            run.tick()
            self.clock.now += 0.02
        self.assertEqual(run.mode, "preflight")
        self.assertFalse(robot.lying)
        self.assertEqual(robot.sent, 0)

    def stop_case(self, inject):
        robot, op = FakeRobot(leg_policy.LegPolicy(POLICY).meta), FakeOperator()
        run = press_live.PressRun(args(live=True, run_dir=self.tmp.name), robot, op)

        def script(k, r, rb, o):
            if k % 30 == 0:
                self.enter_on(r, o, ("preflight", "waiting"))
            if r.mode == "policy" and pl.STAGE_NAMES[r.task.stage] == "at_pre":
                inject(rb, o)
        self.drive(run, robot, op, until=lambda r: r.mode == "damping", script=script)
        self.assertIsNotNone(run.abort_reason)
        self.assertEqual(float(robot.kp.max()), 0.0)
        self.assertEqual(float(robot.kd.max()), ll.DAMP_KD)
        return run

    def test_stop_on_the_remotes_b(self):
        run = self.stop_case(lambda rb, o: setattr(rb, "keys", ["B"]))
        self.assertIn("B", run.abort_reason)

    def test_stop_on_tilt(self):
        a = math.radians(25) / 2
        run = self.stop_case(lambda rb, o: setattr(rb, "quat", np.array([math.cos(a), math.sin(a), 0.0, 0.0])))
        self.assertIn("tilt", run.abort_reason)

    def test_stop_on_a(self):
        run = self.stop_case(lambda rb, o: o.lines.append("a"))
        self.assertIn("operator", run.abort_reason)

    def test_already_released_does_not_start(self):
        robot, op = FakeRobot(leg_policy.LegPolicy(POLICY).meta), FakeOperator()
        robot.mode = ""
        run = press_live.PressRun(args(live=True, run_dir=self.tmp.name), robot, op)
        self.drive(run, robot, op, until=lambda r: r.mode == "done", steps=50)
        self.assertEqual(robot.sent, 0)
        self.assertFalse(robot.lying)


if __name__ == "__main__":
    unittest.main()
