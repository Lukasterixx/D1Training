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

    def _pressed_in(self, deg: float, strict: bool, to: float = 0.005, aligned_first: bool = False):
        """The axial push back after a key rolled `deg` is moved in 0.1 mm steps to `to` (or in aligned, then rolled)."""
        lock = _frame(1)
        lock.strict_roll = strict
        for depth in torch.arange(0.0001, to + 1e-9, 0.0001).tolist():
            _step(lock, *_hand(1, depth, roll=0.0 if aligned_first else math.radians(deg)))
        if aligned_first:
            for k in range(1, 11):
                _step(lock, *_hand(1, to, roll=math.radians(deg) * k / 10))
        return lock, float(-lock.force_w[0, 0])

    def test_strict_keyway_stops_a_key_rolled_past_the_wards_clearance(self):
        # The default contact lets a 4 deg misrolled key slide to 5 mm against little; the strict one holds it.
        _, lax = self._pressed_in(4.0, strict=False)
        lock, strict = self._pressed_in(4.0, strict=True)
        self.assertLess(lax, 1.0, f"lax {lax:.2f} N")
        self.assertGreater(strict, 5.0, f"strict {strict:.2f} N")
        self.assertAlmostEqual(abs(math.degrees(float(lock.roll_err[0]))), 4.0, places=3)

    def test_strict_keyway_lets_a_key_within_the_clearance_in(self):
        # Play 0.2 mm: the clearance is atan(0.2 / 7.5) = 1.53 deg; 0.5 deg goes in as on the default contact.
        _, lax = self._pressed_in(0.5, strict=False)
        _, strict = self._pressed_in(0.5, strict=True)
        self.assertLess(strict, 0.05)
        self.assertAlmostEqual(strict, lax, places=6)

    def test_strict_keyway_keeps_the_wall_for_a_key_that_went_in_aligned(self):
        # In aligned to 5 mm, then turned 4 deg: the wards press the keyway's walls sideways, nothing holds it back.
        _, strict = self._pressed_in(4.0, strict=True, aligned_first=True)
        self.assertLess(strict, 1.0, f"{strict:.2f} N")

    def test_strict_keyway_lets_a_seated_key_be_jiggled(self):
        # Seated aligned, then rolled 3 deg either way and pushed on to the floor: the back of the wards follows the front
        # into the slot, and nothing holds it at the mouth (the layer's jiggle, `JIGGLE_DEG`).
        for strict in (False, True):
            lock = _frame(1)
            lock.strict_roll = strict
            for depth in torch.arange(0.0001, keylock.KEYWAY_DEPTH_M + 1e-9, 0.0001).tolist():
                _step(lock, *_hand(1, depth))
            worst = 0.0
            for deg in (1.0, 2.0, 3.0, 2.0, 0.0, -1.0, -2.0, -3.0, -2.0, 0.0):
                _step(lock, *_hand(1, keylock.KEYWAY_DEPTH_M + 0.0003, roll=math.radians(deg)))
                worst = max(worst, float(lock.force_w[0].norm()))
            if not strict:
                lax = worst
        self.assertLess(worst, lax + 1.0, f"strict {worst:.1f} N against {lax:.1f} N")

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


class ToolTests(unittest.TestCase):
    """S1's per-environment key (`KeyLock.set_tool`, `keylock.descriptor`)."""

    def _hand_for(self, reach: float, offset, tip_depth: float):
        """Link6 holding a key with this reach and shaft offset level along +x, its tip `tip_depth` into the lock on
        the keyway's axis."""
        u, t, a = torch.tensor([0.0, -1.0, 0.0]), torch.tensor([0.0, 0.0, 1.0]), torch.tensor([1.0, 0.0, 0.0])
        rot = torch.stack((u, -t, a), dim=-1)
        tip_link6 = torch.tensor([offset[0], offset[1], keylock.JAW_LINK6_Z_M + reach])
        pos = tip_depth * a - rot @ tip_link6
        return pos, rot

    def test_v4_through_set_tool_is_the_default_key(self):
        lock = _frame(1)
        before = lock.points.clone()
        lock.set_tool(torch.tensor([0]), torch.tensor([keylock.TIP_FROM_JAW_M]), torch.zeros(1, 2), torch.zeros(1))
        self.assertLess(float((lock.points - before).abs().max()), 1e-9)
        self.assertLess(float(keylock.descriptor(torch.tensor([keylock.TIP_FROM_JAW_M]), torch.zeros(1, 2),
                                                 torch.zeros(1), torch.tensor([keylock.NOMINAL_MASS_KG])).abs().max()), 1e-6)

    def test_two_keys_seat_at_their_own_tips(self):
        """Two environments, two keys (reach 24 mm offset 1.5 mm along x; reach 45 mm offset 1.5 mm along -y): each
        hand placed for its own key's tip seats; the same hand placed for the other key's tip does not."""
        keys = [(0.024, (0.0015, 0.0)), (0.045, (0.0, -0.0015))]
        lock = _frame(2)
        lock.set_tool(torch.arange(2), torch.tensor([k[0] for k in keys]), torch.tensor([k[1] for k in keys]),
                      torch.zeros(2))
        seat = keylock.KEYWAY_DEPTH_M - 0.0002

        def run(placed_for):
            for _ in range(3):
                hands = [self._hand_for(*keys[j], seat) for j in placed_for]
                pos = torch.stack([h[0] for h in hands])
                rot = torch.stack([h[1] for h in hands])
                lock.step(pos, rot, torch.zeros(2, 3), torch.zeros(2, 3), 0.005, torch.ones(2, dtype=torch.bool))
            return lock.seated.tolist(), lock.tip_lateral.norm(dim=-1).tolist()

        seated, lateral = run([0, 1])
        self.assertEqual(seated, [True, True])
        self.assertLess(max(lateral), 1e-5)
        lock.place(torch.arange(2), torch.zeros(2, 3), torch.tensor([[1.0, 0.0, 0.0]] * 2),
                   torch.tensor([[0.0, -1.0, 0.0]] * 2), torch.full((2,), 0.0002), torch.full((2,), 0.0005),
                   torch.full((2,), 0.0005), torch.full((2,), 0.3), torch.full((2,), 30.0))
        seated, lateral = run([1, 0])
        self.assertEqual(seated, [False, False])
        self.assertGreater(min(lateral), 0.001)

    def test_a_compliant_key_gives_by_its_compliance_and_stays_put(self):
        """The softest manifest key (v2's H-high, 0.12 mm/N) pressed on the face: it gives back by about compliance x
        force, and the give settles instead of ringing."""
        lock = _frame(1)
        comp = 0.12e-3
        lock.set_tool(torch.tensor([0]), torch.tensor([keylock.TIP_FROM_JAW_M]), torch.zeros(1, 2), torch.tensor([comp]))
        pos, rot = _hand(1, 0.0015, lateral=(0.012, 0.0))
        gives = []
        for _ in range(400):
            _step(lock, pos, rot)
            gives.append(float(lock.give[0, 2]))
        f_axial = float(lock.force_w[0, 0])                      # the face pushes back along -x, Link6's -z
        self.assertLess(f_axial, -1.0)
        self.assertAlmostEqual(gives[-1], comp * f_axial, delta=0.05 * abs(comp * f_axial))
        self.assertLess(max(abs(g - gives[-1]) for g in gives[-50:]), 1e-6)
        # The tip is where the key gave to: in front of where the rigid key's would be.
        self.assertLess(float(lock.tip_local()[0, 2]), keylock.KEY_BACK_LINK6_Z_M + keylock.KEY_LENGTH_M)


class S1bKeyTests(unittest.TestCase):
    """S1b's continuous training region (`keytools.S1B_RANGES`): drawn keys stay inside it and out of its held-out reach
    band, and every held-out key lies outside the region."""

    def test_drawn_keys_avoid_the_band_and_held_out_keys_avoid_the_region(self):
        from unifp_train import keytools
        r_lo, r_hi, g_lo, g_hi, off_max, comp_hi, m_lo, m_hi = keytools.S1B_RANGES
        inside = lambda k: (r_lo * 1e3 <= k[0] <= r_hi * 1e3 and not g_lo * 1e3 < k[0] < g_hi * 1e3
                            and math.hypot(k[1], k[2]) <= off_max * 1e3 + 1e-6 and 0 <= k[3] <= comp_hi * 1e3 + 1e-9
                            and m_lo * 1e3 <= k[4] <= m_hi * 1e3)
        drawn = keytools.s1b_draw_keys(400, 1)
        self.assertTrue(all(inside(k) for k in drawn.values()))
        reach = [k[0] for k in drawn.values()]
        self.assertGreater(min(reach), r_lo * 1e3 - 1e-6)
        self.assertTrue(any(r < g_lo * 1e3 for r in reach) and any(r > g_hi * 1e3 for r in reach))
        self.assertFalse(any(inside(k) for k in keytools.S1B_HELD_OUT.values()))


class LayerTests(unittest.TestCase):
    def _run(self, force_law: bool, face_at: float, hole_at=(0.0, 0.0), steps=900, seat_test="old", floor=None,
             wards_deg=None):
        """A crude hand that follows the layer's goal (plus the force-aware lead) and is stopped by a flat face with a
        hole, so the layer's phases can be checked without the simulator. With `wards_deg` the hole lets the key in only
        with the wrist within that of its roll (the wards); it stops the key `floor` past the face (the keyway's depth: a
        key jammed short of it otherwise)."""
        layer = keylock.InsertLayer(1, "cpu")
        layer.seat_test = seat_test
        floor = keylock.KEYWAY_DEPTH_M if floor is None else floor
        ids = torch.tensor([0])
        layer.start(ids, torch.tensor([-0.015]), torch.tensor([force_law]), torch.tensor([8.0]))
        depth, lateral = torch.tensor([-0.015]), torch.zeros(1, 2)
        for _ in range(steps):
            goal_d, goal_l = layer.goal()
            want = goal_d + layer.free_lead()
            over = (lateral - torch.tensor([hole_at])).norm(dim=-1) < 0.0008
            if wards_deg is not None:
                over = over & (layer.roll.abs() < math.radians(wards_deg))
            over = over | (depth > face_at + 0.0005)
            limit = torch.where(over, torch.tensor([face_at + floor]), torch.tensor([face_at]))
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

    def test_located_straight_in_seats_from_the_face(self):
        # Over the keyway from the start: rolled, the key stops on the face, which is located there; rolled back, it goes
        # straight in, and the depth past the located face says it is seated.
        for force_law in (False, True):
            layer, depth = self._run(force_law, 0.002, hole_at=(0.0, 0.0), seat_test="located", wards_deg=5.0)
            self.assertEqual(int(layer.phase[0]), keylock.DONE, f"force_law {force_law}")
            self.assertFalse(bool(layer.refused[0]))
            self.assertAlmostEqual(float(layer.face_depth[0]), 0.002, delta=0.0008)
            self.assertAlmostEqual(float(layer.touch_depth[0]), 0.002, delta=0.0008)
            self.assertGreater(float(depth[0]), 0.002 + keylock.KEYWAY_DEPTH_M - 0.001)

    def test_a_jam_short_of_the_floor_is_refused_where_the_old_layer_held_it(self):
        # Straight in and jammed 4 mm in: the old layer, which touched the jam, held it there and finished; located, the
        # layer measures it 4 mm past the face, retries once and refuses it.
        for force_law in (False, True):
            old, _ = self._run(force_law, 0.002, hole_at=(0.0, 0.0), floor=0.004, steps=1500, wards_deg=5.0)
            self.assertEqual(int(old.phase[0]), keylock.DONE)
            self.assertFalse(bool(old.refused[0]))
            new, depth = self._run(force_law, 0.002, hole_at=(0.0, 0.0), floor=0.004, steps=1500, seat_test="located",
                                   wards_deg=5.0)
            self.assertEqual(int(new.phase[0]), keylock.DONE, f"force_law {force_law}")
            self.assertTrue(bool(new.refused[0]), f"force_law {force_law}")
            self.assertEqual(float(new.retries[0]), float(keylock.RETRIES))

    def test_touch_search_seat_force(self):
        layer, depth = self._run(True, -0.005, hole_at=(-0.001, 0.0015))
        self.assertEqual(int(layer.phase[0]), keylock.DONE)
        self.assertGreater(float(depth[0]), -0.005 + keylock.KEYWAY_DEPTH_M - 0.001)


if __name__ == "__main__":
    unittest.main()
