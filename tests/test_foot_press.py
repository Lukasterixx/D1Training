"""The foot-press task's plain-torch pieces (`unifp_train/foot_press.py`): the leg's kinematics, the plate table, the
plate's contact and the press layer. No simulator."""
from __future__ import annotations

import math
import unittest

import numpy as np
import torch

from unifp_train import foot_press, foot_press_cfg as pc, handle, leg_cfg


class KinematicsTest(unittest.TestCase):
    def test_legs_match_the_stance_solver(self):
        from demos.combiner_key import classic_stance

        q = np.array(leg_cfg.SETTLED_LEG_POS).reshape(4, 3)
        rng = np.random.default_rng(0)
        for k, leg in enumerate(classic_stance.LEGS):
            for qq in [q[k]] + [q[k] + rng.uniform(-0.4, 0.4, 3) for _ in range(5)]:
                np.testing.assert_allclose(foot_press.leg_points_base(leg, qq)[2], classic_stance.foot(leg, qq), atol=1e-9)

    def test_settled_feet_where_recorded(self):
        q = np.array(leg_cfg.SETTLED_LEG_POS).reshape(4, 3)
        for k, leg in enumerate(("FL", "FR", "RL", "RR")):
            foot = foot_press.leg_points_base(leg, q[k])[2]
            self.assertAlmostEqual(foot[0], leg_cfg.NOMINAL_FEET_B[k][0], delta=0.045)
            self.assertAlmostEqual(foot[1], leg_cfg.NOMINAL_FEET_B[k][1], delta=0.005)


class TableTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.table = foot_press.PlateTable(size=3000, seed=1)

    def test_size_and_kinds(self):
        c = self.table.columns
        self.assertEqual(self.table.size, 3000)
        self.assertEqual(set(np.unique(c["kind"]).tolist()), {0, 1, 2, 3})

    def test_contacts_reached_and_plates_clear(self):
        c = self.table.columns
        reach = [foot_press.ReachVoxels(h) for h in pc.REACH_HEIGHTS_M]
        for r in reach:
            self.assertTrue(r(c["contact"]).all())
            self.assertTrue(r(c["contact"] - pc.PUSH_ROOM_M * c["n"]).all())
        self.assertTrue((c["contact"][:, 2] >= pc.MIN_CONTACT_HEIGHT_M).all())
        # The contact point is the foot's centre a foot radius off the face, inside the rectangle.
        rel = c["contact"] - c["face"]
        np.testing.assert_allclose((rel * c["n"]).sum(-1), handle.FOOT_RADIUS_M, atol=1e-9)
        self.assertTrue((np.abs((rel * c["t1"]).sum(-1)) <= c["half"][:, 0] + 1e-9).all())
        self.assertTrue((np.abs((rel * c["t2"]).sum(-1)) <= c["half"][:, 1] + 1e-9).all())
        np.testing.assert_allclose(np.linalg.norm(c["n"], axis=-1), 1.0, atol=1e-9)
        np.testing.assert_allclose((c["n"] * c["t1"]).sum(-1), 0.0, atol=1e-9)
        # Corners above the floor; walls horizontal, level plates straight up or down.
        for a in (-1, 1):
            for b in (-1, 1):
                corner = c["face"] + a * c["half"][:, :1] * c["t1"] + b * c["half"][:, 1:] * c["t2"]
                self.assertTrue((corner[:, 2] >= -1e-9).all())
        np.testing.assert_allclose(c["n"][c["kind"] == 1][:, 2], 0.0, atol=1e-9)
        np.testing.assert_allclose(np.abs(c["n"][c["kind"] >= 2][:, 2]), 1.0, atol=1e-9)

    def test_tilted_normals_face_the_shoulder(self):
        c = self.table.columns
        tilted = c["kind"] == 0
        d0 = np.array(pc.SHOULDER_M) - c["contact"][tilted]
        d0 /= np.linalg.norm(d0, axis=-1, keepdims=True)
        angle = np.degrees(np.arccos(np.clip((d0 * c["n"][tilted]).sum(-1), -1, 1)))
        self.assertLessEqual(angle.max(), math.degrees(pc.NORMAL_TILT_MAX_RAD) + 1e-6)

    def test_reproducible(self):
        again = foot_press.PlateTable(size=3000, seed=1)
        np.testing.assert_array_equal(again.columns["face"], self.table.columns["face"])


def wall(n_envs=1, k_soft=handle.CONTACT_K_N_PER_M, travel=0.0, mu=0.6):
    """A plate facing -x (toward a robot behind it) with its face at x = 0.4, 20 x 20 cm."""
    plates = foot_press.PlateSet(n_envs, "cpu")
    ids = torch.arange(n_envs)
    face = torch.tensor([[0.4, 0.0, 0.3]]).expand(n_envs, 3)
    n = torch.tensor([[-1.0, 0.0, 0.0]]).expand(n_envs, 3)
    t1 = torch.tensor([[0.0, 1.0, 0.0]]).expand(n_envs, 3)
    plates.place(ids, face=face, n=n, t1=t1, half=torch.full((n_envs, 2), 0.1), k_soft=torch.full((n_envs,), k_soft),
                 travel=torch.full((n_envs,), travel), mu=torch.full((n_envs,), mu))
    return plates


class PlateTest(unittest.TestCase):
    def _at(self, plates, x, y=0.0, v=(0.0, 0.0, 0.0)):
        foot = torch.tensor([[x, y, 0.3]])
        return plates.step(foot, torch.tensor([v]), 0.005)

    def test_rigid_spring(self):
        plates = wall()
        d = 0.002
        f = self._at(plates, 0.4 - handle.FOOT_RADIUS_M + d)
        self.assertAlmostEqual(float(f[0, 0]), -handle.CONTACT_K_N_PER_M * d, places=3)
        self.assertTrue(bool(plates.on_bar[0]))

    def test_sprung_then_stop(self):
        plates = wall(k_soft=1000.0, travel=0.02)
        f = self._at(plates, 0.4 - handle.FOOT_RADIUS_M + 0.01)
        self.assertAlmostEqual(float(-f[0, 0]), 10.0, places=3)
        plates = wall(k_soft=1000.0, travel=0.02)
        f = self._at(plates, 0.4 - handle.FOOT_RADIUS_M + 0.022)
        self.assertAlmostEqual(float(-f[0, 0]), 1000.0 * 0.02 + handle.CONTACT_K_N_PER_M * 0.002, places=3)

    def test_no_contact_clear_of_the_slab(self):
        plates = wall()
        f = self._at(plates, 0.39, y=0.1 + handle.FOOT_RADIUS_M + 0.005)
        self.assertEqual(float(f.norm()), 0.0)
        f = self._at(plates, 0.4 + pc.PLATE_THICKNESS_M + handle.FOOT_RADIUS_M + 0.005)
        self.assertEqual(float(f.norm()), 0.0)

    def test_an_edge_pushes_sideways_and_builds_up(self):
        """A foot coming in from the side at the face's height meets the edge, the force growing with the overlap."""
        plates = wall()
        forces = []
        for gap in (0.020, 0.015, 0.010, 0.005):
            plates = wall()
            f = self._at(plates, 0.41, y=0.1 + gap)                  # 1 cm into the slab's depth, beside its edge
            forces.append(float(f[0, 1]))
        self.assertTrue(all(b > a for a, b in zip(forces, forces[1:])), forces)
        self.assertAlmostEqual(forces[-1], handle.CONTACT_K_N_PER_M * (handle.FOOT_RADIUS_M - 0.005), places=3)

    def test_face_only_plate_as_before(self):
        """`edges=False`, rounds 1-5's plate: nothing beside the rectangle, the whole radius once the centre crosses."""
        plates = foot_press.PlateSet(1, "cpu", edges=False)
        plates.place(torch.arange(1), face=torch.tensor([[0.4, 0.0, 0.3]]), n=torch.tensor([[-1.0, 0.0, 0.0]]),
                     t1=torch.tensor([[0.0, 1.0, 0.0]]), half=torch.full((1, 2), 0.1), k_soft=torch.full((1,), 8000.0),
                     travel=torch.zeros(1), mu=torch.full((1,), 0.6))
        self.assertEqual(float(self._at(plates, 0.41, y=0.105).norm()), 0.0)
        self.assertAlmostEqual(float(-self._at(plates, 0.40, y=0.095)[0, 0]), 8000.0 * min(handle.FOOT_RADIUS_M, handle.MAX_OVERLAP_M), places=2)

    def test_tangential_sticks_then_slips(self):
        plates = wall(mu=0.5)
        x = 0.4 - handle.FOOT_RADIUS_M + 0.0025           # 20 N normal
        self._at(plates, x)
        f = self._at(plates, x, y=0.001)                  # 1 mm sideways: 4 N back, under 10 N of friction
        self.assertAlmostEqual(float(f[0, 1]), -pc.TANGENT_K_N_PER_M * 0.001, places=2)
        f = self._at(plates, x, y=0.01)                   # 10 mm: past the cap, held at mu x normal
        self.assertAlmostEqual(float(-f[0, 1]), 0.5 * 20.0, places=2)
        f = self._at(plates, x, y=0.0095)                 # back a little: the anchor moved, the force eases
        self.assertLess(float(-f[0, 1]), 10.0)

    def test_knee_and_head(self):
        plates = wall()
        foot = torch.tensor([[0.2, 0.0, 0.3]])
        zero = torch.zeros(1, 3)
        plates.step(foot, zero, 0.005, knee=torch.tensor([[0.4 - 0.02, 0.0, 0.3]]), knee_v=zero,
                    head=torch.tensor([[0.4 - 0.06, 0.0, 0.3]]), head_v=zero, head_r=0.065)
        self.assertAlmostEqual(float(plates.knee_n[0]), handle.CONTACT_K_N_PER_M * (handle.KNEE_RADIUS_M - 0.02), places=3)
        self.assertAlmostEqual(float(plates.head_n[0]), handle.CONTACT_K_N_PER_M * 0.005, places=3)
        self.assertGreater(float(plates.knee_door_n[0]), 0.0)


def press(force_law: bool, levels=(10.0, 20.0, 5.0), segments=1, slide=False, slide_dist=0.02):
    layer = foot_press.PressLayer(1, "cpu")
    ids = torch.arange(1)
    n = torch.tensor([[-1.0, 0.0, 0.0]])
    contact = torch.tensor([[0.4 - handle.FOOT_RADIUS_M, 0.0, 0.3]])
    pre = contact + 0.04 * n
    one = lambda v: torch.tensor([float(v)])
    layer.start(ids, pre=pre, n=n, standoff=one(0.04), speed=one(0.05), levels=torch.tensor([levels]),
                ramps=torch.tensor([[0.5, 0.5, 0.5]]), holds=torch.tensor([[1.0, 1.0, 1.0]]),
                segments=torch.tensor([segments]), tan_ratio=torch.zeros(1, 3), tan_dir=torch.tensor([[0.0, 1.0, 0.0]]),
                slide=torch.tensor([slide]), slide_dir=torch.tensor([[0.0, 1.0, 0.0]]), slide_speed=one(0.03),
                slide_dist=one(slide_dist), force_law=torch.tensor([force_law]), foot=pre)
    return layer, contact


def run(layer, contact, steps=600, dt=0.02, lag=0.3):
    """A foot that follows the goal with a first-order lag and cannot pass the wall at `contact`'s x."""
    foot = layer.pre.clone()
    log = []
    for _ in range(steps):
        goal, force = layer.step(foot, dt)
        foot = foot + (goal - foot) * (dt / lag)
        foot[:, 0] = foot[:, 0].clamp(max=float(contact[0, 0]))
        log.append((int(layer.phase[0]), float(layer.normal_level[0]), goal.clone(), force.clone(), foot.clone()))
    return log


class LayerTest(unittest.TestCase):
    def test_force_law_finds_the_wall_and_runs_the_profile(self):
        layer, contact = press(True, segments=2)
        log = run(layer, contact)
        phases = [p for p, *_ in log]
        self.assertIn(handle.HOLDING, phases)
        self.assertEqual(phases[-1], handle.DONE)
        self.assertTrue(bool(layer.stalled[0]))
        # The origin is where the foot stalled: at the wall.
        self.assertAlmostEqual(float(layer.origin[0, 0]), float(contact[0, 0]), delta=0.003)
        holding = [(lvl, g, f) for p, lvl, g, f, _ in log if p == handle.HOLDING]
        levels = [lvl for lvl, _, _ in holding]
        self.assertAlmostEqual(max(levels), 20.0, places=4)
        for lvl, g, f in holding:
            # The force is commanded into the wall at the profile's level, the goal held at the origin.
            self.assertAlmostEqual(float(f[0, 0]), lvl, places=4)
            self.assertAlmostEqual(float(g[0, 0]), float(layer.origin[0, 0]), places=6)
        self.assertEqual(float(log[-1][3].norm()), 0.0)

    def test_force_goal_follows_a_sprung_plate(self):
        """A foot that sinks 1.5 cm past where contact was called carries the force-aware goal with it."""
        layer, contact = press(True)
        foot = layer.pre.clone()
        for _ in range(150):
            layer.step(foot, 0.02)
            foot = torch.minimum(foot + torch.tensor([[0.004, 0.0, 0.0]]), contact)
            if int(layer.phase[0]) == handle.HOLDING:
                break
        self.assertEqual(int(layer.phase[0]), handle.HOLDING)
        sunk = contact + torch.tensor([[0.015, 0.0, 0.0]])
        for _ in range(30):
            goal, _ = layer.step(sunk, 0.02)
        self.assertAlmostEqual(float(goal[0, 0]), float(sunk[0, 0]), delta=0.001)

    def test_target_continuous_when_contact_is_called(self):
        """The force-aware target (goal + command / k) and the position-only goal do not jump at the stall."""
        for law in (True, False):
            layer, contact = press(law)
            foot = layer.pre.clone()
            last = None
            for _ in range(300):
                goal, force = layer.step(foot, 0.02)
                target = goal + force / leg_cfg.FOOT_FORCE_KP
                if int(layer.phase[0]) == handle.HOLDING and last is not None and last[1] == handle.PRESSING:
                    self.assertAlmostEqual(float(target[0, 0]), float(last[0][0, 0]), delta=0.004)
                    break
                last = (target.clone(), int(layer.phase[0]))
                foot = foot + (goal - foot) * 0.05
                foot[:, 0] = foot[:, 0].clamp(max=float(contact[0, 0]))
            else:
                self.fail("no contact called")

    def test_position_layer_asks_for_the_displaced_goal(self):
        layer, contact = press(False)
        log = run(layer, contact)
        holding = [(lvl, g, f) for p, lvl, g, f, _ in log if p == handle.HOLDING]
        self.assertTrue(holding)
        for lvl, g, f in holding:
            self.assertEqual(float(f.norm()), 0.0)
            self.assertAlmostEqual(float(g[0, 0] - layer.origin[0, 0]), lvl / leg_cfg.FOOT_FORCE_KP, places=5)

    def test_reference_runs_out_without_a_wall(self):
        layer, _ = press(True)
        far = torch.tensor([[5.0, 0.0, 0.3]])
        log = run(layer, far, steps=400)
        self.assertFalse(bool(layer.stalled[0]))
        self.assertIn(handle.HOLDING, [p for p, *_ in log])

    def test_no_stall_called_off_the_line(self):
        """A foot that does not follow (stuck 5 cm to the side) never has contact called; the reference runs out."""
        layer, contact = press(True)
        foot = layer.pre + torch.tensor([[0.0, 0.05, 0.0]])
        for _ in range(200):
            layer.step(foot, 0.02)
        self.assertFalse(bool(layer.stalled[0]))
        self.assertGreaterEqual(int(layer.phase[0]), handle.HOLDING)

    def test_release_backs_out_from_a_sagged_foot(self):
        """A foot 10 cm further out than the press point at the end of the profile: through the release the goal (with
        the force-aware command's displacement, its target) only moves out along the normal, starting from the foot."""
        for law in (True, False):
            layer, contact = press(law)
            foot = layer.pre.clone()
            out, at_release = [], None
            for _ in range(600):
                goal, force = layer.step(foot, 0.02)
                target = goal + force / leg_cfg.FOOT_FORCE_KP
                if int(layer.phase[0]) == handle.HOLDING:
                    foot = layer.origin + 0.10 * layer.n                   # sagged 10 cm out (n points out of the plate)
                elif int(layer.phase[0]) == handle.RELEASING:
                    if at_release is None:
                        at_release = float((foot * layer.n).sum())
                    out.append(float((target * layer.n).sum()))
                elif int(layer.phase[0]) == handle.DONE:
                    break
                else:
                    foot = foot + (goal - foot) * 0.05
                    foot[:, 0] = foot[:, 0].clamp(max=float(contact[0, 0]))
            self.assertEqual(int(layer.phase[0]), handle.DONE)
            self.assertTrue(all(b >= a - 1e-6 for a, b in zip(out, out[1:])), "the release moved in")
            # It starts from the foot, less the force still asked for (10 N over 400 N/m), not 10 cm in at the plate.
            self.assertGreater(out[0], at_release - 10.0 / leg_cfg.FOOT_FORCE_KP - 1e-3)

    def test_release_does_not_draw_a_slipped_foot_back(self):
        """A foot 10 cm along the plate from the press point at the end of the profile (it slid off): the release backs
        out from where it is, not back toward the press point."""
        for law in (True, False):
            layer, contact = press(law)
            foot = layer.pre.clone()
            along = torch.tensor([[0.0, 1.0, 0.0]])
            seen = []
            for _ in range(600):
                goal, _ = layer.step(foot, 0.02)
                if int(layer.phase[0]) == handle.HOLDING:
                    foot = layer.origin + 0.10 * along
                elif int(layer.phase[0]) == handle.RELEASING:
                    seen.append(float(goal[0, 1]))
                elif int(layer.phase[0]) == handle.DONE:
                    break
                else:
                    foot = foot + (goal - foot) * 0.05
                    foot[:, 0] = foot[:, 0].clamp(max=float(contact[0, 0]))
            self.assertTrue(seen)
            self.assertAlmostEqual(min(seen), float(foot[0, 1]), delta=1e-5)

    def test_slide_moves_the_goal_along_the_plate(self):
        layer, contact = press(True, slide=True)
        log = run(layer, contact)
        ys = [float(g[0, 1]) for p, _, g, _, _ in log if p == handle.HOLDING]
        self.assertAlmostEqual(max(ys) - min(ys), 0.02, places=4)


class BatchTest(unittest.TestCase):
    def test_layer_and_plates_batch(self):
        """Several environments at once, in different phases (a single one would broadcast past a missing axis)."""
        n = 5
        plates = foot_press.PlateSet(n, "cpu")
        ids = torch.arange(n)
        face = torch.tensor([[0.4, 0.0, 0.3]]).repeat(n, 1)
        face[:, 0] += torch.linspace(0.0, 0.04, n)
        normal = torch.tensor([[-1.0, 0.0, 0.0]]).repeat(n, 1)
        plates.place(ids, face=face, n=normal, t1=torch.tensor([[0.0, 1.0, 0.0]]).repeat(n, 1),
                     half=torch.full((n, 2), 0.1), k_soft=torch.full((n,), 8000.0), travel=torch.zeros(n),
                     mu=torch.full((n,), 0.6))
        layer = foot_press.PressLayer(n, "cpu")
        start = torch.tensor([[0.33, 0.0, 0.3]]).repeat(n, 1)
        full = lambda v: torch.full((n,), float(v))
        layer.start(ids[:3], pre=start[:3], n=normal[:3], standoff=full(0.04)[:3], speed=full(0.05)[:3],
                    levels=torch.full((3, 3), 15.0), ramps=torch.full((3, 3), 0.5), holds=torch.full((3, 3), 1.0),
                    segments=torch.ones(3, dtype=torch.long), tan_ratio=torch.full((3, 3), 0.2),
                    tan_dir=torch.tensor([[0.0, 1.0, 0.0]]).repeat(3, 1), slide=torch.tensor([True, False, True]),
                    slide_dir=torch.tensor([[0.0, 0.0, 1.0]]).repeat(3, 1), slide_speed=full(0.03)[:3],
                    slide_dist=full(0.02)[:3], force_law=torch.tensor([True, False, True]), foot=start[:3])
        foot = start.clone()
        phases = set()
        for _ in range(400):
            goal, force = layer.step(foot, 0.02)
            self.assertEqual(goal.shape, (n, 3))
            self.assertEqual(force.shape, (n, 3))
            f = plates.step(foot, torch.zeros(n, 3), 0.005)
            self.assertEqual(f.shape, (n, 3))
            foot = foot + (goal - foot) * 0.06
            foot[:, 0] = torch.minimum(foot[:, 0], face[:, 0] - handle.FOOT_RADIUS_M + 0.002)
            phases |= set(layer.phase.tolist())
        self.assertEqual(layer.phase[3:].tolist(), [handle.IDLE, handle.IDLE])
        self.assertTrue({handle.HOLDING, handle.DONE} <= phases)


if __name__ == "__main__":
    unittest.main()
