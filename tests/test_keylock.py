"""The cam lock's contact and the insertion's task layer (`unifp_train/keylock.py`), on the CPU with no simulator."""
from __future__ import annotations

import math
import unittest

import torch

from unifp_train import keylock


def _frame(n: int):
    """A lock facing a robot that looks along +x: the axis +x into the lock, the bar to the robot's right (-y)."""
    lock = keylock.KeyLock(n, "cpu")
    ids = torch.arange(n)
    axis = torch.tensor([[1.0, 0.0, 0.0]]).expand(n, 3)
    e_u = torch.tensor([[0.0, -1.0, 0.0]]).expand(n, 3)
    full = lambda v: torch.full((n,), float(v))
    lock.place(ids, torch.zeros(n, 3), axis, e_u, full(0.0002), full(0.0005), full(0.0005), full(0.3), full(30.0))
    return lock


def _hand(n: int, tip_depth: float, lateral=(0.0, 0.0), roll: float = 0.0, tilt: float = 0.0):
    """Link6 holding the key level along +x, its tip `tip_depth` into the lock, rolled and tilted (about the lock's u)."""
    # Link6's x is the bar (u), y is -t, z is the axis.
    u = torch.tensor([0.0, -1.0, 0.0])
    t = torch.tensor([0.0, 0.0, 1.0])
    a = torch.tensor([1.0, 0.0, 0.0])
    ur = math.cos(roll) * u + math.sin(roll) * t
    tr = torch.cross(ur, a, dim=0)
    # A tilt about u turns the axis toward t.
    ar = math.cos(tilt) * a + math.sin(tilt) * tr
    tr = torch.cross(ur, ar, dim=0)
    rot = torch.stack((ur, -tr, ar), dim=-1).expand(n, 3, 3).contiguous()
    tip_link6 = keylock.KEY_BACK_LINK6_Z_M + keylock.KEY_LENGTH_M
    tip = tip_depth * a + lateral[0] * u + lateral[1] * t
    pos = (tip - tip_link6 * ar).expand(n, 3).contiguous()
    return pos, rot


def _step(lock, pos, rot, vel=(0.0, 0.0, 0.0)):
    n = pos.shape[0]
    lock.step(pos, rot, torch.tensor(vel).expand(n, 3), torch.zeros(n, 3), 0.005, torch.ones(n, dtype=torch.bool))


class ContactTests(unittest.TestCase):
    def test_in_front_of_the_face_nothing(self):
        lock = _frame(1)
        _step(lock, *_hand(1, -0.002))
        self.assertEqual(float(lock.force_w.norm()), 0.0)

    def test_lined_up_key_goes_in_free_and_stops_on_the_floor(self):
        lock = _frame(1)
        _step(lock, *_hand(1, 0.005))
        self.assertLess(float(lock.force_w.norm()), 1e-6)
        self.assertTrue(bool(lock.tip_inside[0]))
        _step(lock, *_hand(1, keylock.KEYWAY_DEPTH_M + 0.0005))
        f = lock.force_w[0]
        # The floor pushes it straight back out: the tip's face points at 0.5 mm.
        expect = keylock.CONTACT_K_N_PER_M / keylock.POINTS_SHARE * lock.face_points * 0.0005
        self.assertAlmostEqual(float(-f[0]), expect, delta=0.5)
        self.assertLess(float(f[1:].norm()), 0.05)
        self.assertTrue(bool(lock.seated[0]))

    def test_off_the_keyway_the_face_holds_it(self):
        lock = _frame(1)
        # Clear of the keyway, the whole tip on the face: 35 points at 0.5 mm.
        _step(lock, *_hand(1, 0.0005, lateral=(0.012, 0.0)))
        f = lock.force_w[0]
        expect = keylock.CONTACT_K_N_PER_M / keylock.POINTS_SHARE * lock.face_points * 0.0005
        self.assertAlmostEqual(float(-f[0]), expect, delta=0.5)
        # 3 mm off, most of the tip is over the hole, and the rest on the face holds it.
        lock = _frame(1)
        _step(lock, *_hand(1, 0.0005, lateral=(0.003, 0.0)))
        self.assertGreater(float(-lock.force_w[0, 0]), 1.0)
        self.assertFalse(bool(lock.tip_inside[0]))

    def test_rolled_past_the_wards_clearance_it_stops_on_the_face(self):
        for deg, blocked in ((0.5, False), (6.0, True)):
            lock = _frame(1)
            for depth in torch.arange(0.0001, 0.0013, 0.0001).tolist():
                _step(lock, *_hand(1, depth, roll=math.radians(deg)))
            pushed_back = float(-lock.force_w[0, 0])
            self.assertEqual(pushed_back > 0.3, blocked, f"roll {deg} deg: {pushed_back:.2f} N back")

    def test_the_edge_break_pushes_toward_the_keyway(self):
        lock = _frame(1)
        # 0.5 mm off along +u (the robot's right, -y), the tip 0.8 mm in: the key's outer edge on the lock's break.
        _step(lock, *_hand(1, 0.0008, lateral=(0.0005, 0.0)))
        f = lock.force_w[0]
        self.assertGreater(float(f[1]), 0.05, "pushed back toward -u, i.e. +y")

    def test_pressed_deep_beside_the_keyway_it_is_not_pulled_in(self):
        lock = _frame(1)
        # Onto the face 0.3 mm outside the hole's edge, pressed on in steps: the face holds it, nothing pulls it across.
        for depth in (0.0002, 0.0006, 0.0012, 0.0020):
            _step(lock, *_hand(1, depth, lateral=(0.0098, 0.0)))
        f = lock.force_w[0]
        self.assertGreater(float(-f[0]), 0.5)
        self.assertLess(abs(float(f[1])), 0.1 * float(-f[0]))

    def test_friction_opposes_sliding_on_the_face(self):
        lock = _frame(1)
        _step(lock, *_hand(1, 0.0005, lateral=(0.012, 0.0)), vel=(0.0, 0.0, 0.01))
        f = lock.force_w[0]
        self.assertLess(float(f[2]), 0.0)
        self.assertLessEqual(float(-f[2]), 0.3 * float(-f[0]) + 1e-6)

    def test_a_hard_axial_push_slides_the_key_in_the_jaws(self):
        lock = _frame(1)
        for _ in range(200):
            _step(lock, *_hand(1, 0.003, lateral=(0.012, 0.0)))
        self.assertGreater(float(lock.slip_m[0]), 0.0)


class LayerTests(unittest.TestCase):
    def _run(self, force_law: bool, face_at: float, hole_at=(0.0, 0.0), steps=900):
        """A crude hand that follows the layer's goal (plus the force-aware lead) and is stopped by a flat face with a
        hole, so the layer's phases can be checked without the simulator."""
        layer = keylock.InsertLayer(1, "cpu")
        ids = torch.tensor([0])
        layer.start(ids, torch.tensor([-0.015]), torch.tensor([force_law]), torch.tensor([8.0]))
        depth, lateral = torch.tensor([-0.015]), torch.zeros(1, 2)
        for _ in range(steps):
            goal_d, goal_l = layer.goal()
            want = goal_d + layer.free_lead()
            over = (lateral - torch.tensor([hole_at])).norm(dim=-1) < 0.0008
            limit = torch.where(over, torch.tensor([face_at + keylock.KEYWAY_DEPTH_M]), torch.tensor([face_at]))
            new_depth = torch.minimum(depth + (want - depth).clamp(-0.0004, 0.0004), limit)
            # In the keyway its walls hold the hand across; elsewhere it follows the goal.
            in_keyway = over & (depth > face_at + 0.0005)
            lateral = torch.where(in_keyway.unsqueeze(-1), lateral, lateral + (goal_l - lateral) * 0.5)
            depth = new_depth
            layer.step(depth, lateral, 0.02)
            if int(layer.phase[0]) == keylock.DONE:
                break
        return layer, depth

    def test_touch_search_seat_position(self):
        layer, depth = self._run(False, 0.004, hole_at=(0.0015, -0.001))
        self.assertEqual(int(layer.phase[0]), keylock.DONE)
        self.assertAlmostEqual(float(layer.touch_depth[0]), 0.004, delta=0.001)
        self.assertGreater(float(depth[0]), 0.004 + keylock.KEYWAY_DEPTH_M - 0.001)

    def test_straight_in_is_felt_by_the_probe(self):
        # Over the keyway from the start: the key goes to the floor, the touch is the floor, the probe finds it held.
        for force_law in (False, True):
            layer, depth = self._run(force_law, 0.002, hole_at=(0.0, 0.0))
            self.assertEqual(int(layer.phase[0]), keylock.DONE, f"force_law {force_law}")
            self.assertEqual(float(layer.entered_by[0]), 2.0)
            self.assertGreater(float(depth[0]), 0.002 + keylock.KEYWAY_DEPTH_M - 0.001)

    def test_touch_search_seat_force(self):
        layer, depth = self._run(True, -0.005, hole_at=(-0.001, 0.0015))
        self.assertEqual(int(layer.phase[0]), keylock.DONE)
        self.assertGreater(float(depth[0]), -0.005 + keylock.KEYWAY_DEPTH_M - 0.001)


if __name__ == "__main__":
    unittest.main()
