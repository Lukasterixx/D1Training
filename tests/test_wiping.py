"""The wiping task's surface, path and task layer (`unifp_train/wiping.py`), without a simulator.

Each contact case is pinned -- a pad pressed flat, a tilted pad on its rim, friction, the panel's edge, going through
it -- then the path's resampling, then the task layer wiping a surface it has estimated right, too near, too far and
tilted, against a stand-in hand that tracks UniFP's own target: the goal displaced by (measured + commanded force) / k.
"""
from __future__ import annotations

import math
import unittest

import torch

from unifp_train import wiping

NORMAL = torch.tensor([[-1.0, 0.0, 0.0]])     # a wall ahead of the robot, facing back toward it
INTO = -NORMAL
K = wiping.CONTACT_K_N_PER_M


def surface(n=1, x=0.0, normal=NORMAL, half=0.2, mu=0.4):
    s = wiping.Surface(n, "cpu")
    s.place(torch.arange(n), origin=torch.tensor([[x, 0.0, 0.0]]).expand(n, 3), normal=normal.expand(n, 3),
            half_a=torch.full((n,), half), half_b=torch.full((n,), half), mu=torch.full((n,), mu))
    return s


def tilted_face(deg):
    """A pad face pointing into the wall, tilted `deg` about the vertical."""
    t = math.radians(deg)
    return torch.tensor([[math.cos(t), math.sin(t), 0.0]])


class SurfaceTest(unittest.TestCase):
    def test_a_flat_pad_presses_with_its_whole_face_at_its_centre(self):
        s = surface()
        force = s.step(torch.tensor([[0.001, 0.0, 0.0]]), INTO, torch.zeros(1, 3), torch.zeros(1, 3))
        self.assertAlmostEqual(float(force[0, 0]), -K * 0.001, places=3)
        self.assertLess(float((s.pressure_centre[0] - torch.tensor([0.001, 0.0, 0.0])).norm()), 1e-6)
        self.assertTrue(bool(s.in_contact[0]))

    def test_a_tilted_pad_rides_on_its_rim(self):
        s = surface()
        s.step(torch.tensor([[0.0, 0.0, 0.0]]), tilted_face(10.0), torch.zeros(1, 3), torch.zeros(1, 3))
        # Only the rim on the leading side is past the surface; the contact sits there, off the centre, and is far
        # softer than a flat pad pressed as deep (three of nine points bite).
        self.assertGreater(float(s.normal_n[0]), 0.0)
        self.assertLess(float(s.normal_n[0]), 0.5 * K * float(s.depth[0]))
        self.assertGreater(float(s.pressure_centre[0].norm()), 0.8 * wiping.PAD_RADIUS_M)
        self.assertAlmostEqual(float(s.depth[0]), wiping.PAD_RADIUS_M * math.sin(math.radians(10.0)), delta=2e-4)

    def test_friction_opposes_sliding_along_the_surface(self):
        s = surface(mu=0.5)
        force = s.step(torch.tensor([[0.001, 0.0, 0.0]]), INTO, torch.tensor([[0.0, 0.0, 0.05]]), torch.zeros(1, 3))
        self.assertLess(float(force[0, 2]), 0.0)
        self.assertAlmostEqual(-float(force[0, 2]), 0.5 * float(s.normal_n[0]) * math.tanh(0.05 / 0.02), delta=0.05)

    def test_past_the_panels_edge_nothing_is_touched(self):
        s = surface(half=0.1)
        force = s.step(torch.tensor([[0.002, 0.2, 0.0]]), INTO, torch.zeros(1, 3), torch.zeros(1, 3))
        self.assertEqual(float(force.norm()), 0.0)

    def test_pushing_through_the_panel_is_damage(self):
        s = surface()
        s.step(torch.tensor([[wiping.FACE_DEPTH_M + 0.001, 0.0, 0.0]]), INTO, torch.zeros(1, 3), torch.zeros(1, 3))
        self.assertTrue(bool(s.pierced[0]))


class PathTest(unittest.TestCase):
    def test_a_line_is_resampled_evenly_end_to_end(self):
        points, arc = wiping.resample(wiping.line_path(torch.tensor([0.16]), torch.tensor([0.0])))
        self.assertEqual(points.shape, (1, wiping.PATH_SAMPLES, 2))
        self.assertAlmostEqual(float(arc[0, -1]), 0.16, places=6)
        self.assertAlmostEqual(float(points[0, 0, 0]), -0.08, places=6)
        self.assertAlmostEqual(float(points[0, -1, 0]), 0.08, places=6)
        steps = (points[0, 1:] - points[0, :-1]).norm(dim=-1)
        self.assertLess(float(steps.max() - steps.min()), 1e-6)

    def test_a_zigzag_is_its_strokes_and_the_steps_between(self):
        vertices = wiping.zigzag_path(torch.tensor([0.12]), torch.tensor([0.03]), strokes=3)
        points, arc = wiping.resample(vertices)
        self.assertAlmostEqual(float(arc[0, -1]), 3 * 0.12 + 2 * 0.03, places=5)
        self.assertAlmostEqual(float(points[0, 0, 1]), 0.03, places=6)      # starts on the top stroke
        self.assertAlmostEqual(float(points[0, -1, 1]), -0.03, places=6)    # ends on the bottom one

    def test_point_at_interpolates_and_gives_the_tangent(self):
        points, arc = wiping.resample(wiping.line_path(torch.tensor([0.2]), torch.tensor([math.pi / 2])))
        p, t = wiping.point_at(points, arc, torch.tensor([0.05]))
        self.assertAlmostEqual(float(p[0, 1]), -0.05, places=5)
        self.assertAlmostEqual(float(t[0, 1]), 1.0, places=5)


class WipeTaskLayerTest(unittest.TestCase):
    def run_loop(self, *, offset=0.0, tilt_deg=0.0, force_n=10.0, kind="line", seconds=20.0, mu=0.4):
        """The estimate: a wall at x = 0 facing -x. The truth: `offset` further (positive) and tilted `tilt_deg` about
        the vertical. The hand tracks UniFP's target -- goal + (contact + command) / 200 -- slowly along the wall's
        normal (0.5% per 5 ms step, the admittance the insertion tests use) and faster across it (3%)."""
        dt = 0.005
        t = math.radians(tilt_deg)
        normal = torch.tensor([[-math.cos(t), math.sin(t), 0.0]])
        s = surface(x=offset, normal=normal, mu=mu)
        layer = wiping.WipeTaskLayer(1, "cpu")
        vertices = (wiping.line_path(torch.tensor([0.16]), torch.tensor([0.0])) if kind == "line"
                    else wiping.zigzag_path(torch.tensor([0.10]), torch.tensor([0.03])))
        points, arc = wiping.resample(vertices)
        layer.start(torch.arange(1), origin=torch.zeros(1, 3), normal=NORMAL, points=points, arc=arc,
                    standoff=torch.tensor([0.015]), overdrive=torch.tensor([0.015]), force_n=torch.tensor([force_n]),
                    speed=torch.tensor([0.03]), approach_speed=torch.tensor([0.02]))
        pad = layer.to_world(points[:, 0]) - 0.015 * INTO
        vel = torch.zeros(1, 3)
        wiped = torch.zeros(1, wiping.PATH_SAMPLES, dtype=torch.bool)
        in_motion = in_contact = 0
        forces, pushes = [], []
        for _ in range(int(seconds / dt)):
            goal, command = layer.step(pad, dt)
            contact = s.step(pad, INTO, vel, torch.zeros(1, 3))
            target = goal + (contact + command) / 200.0
            miss = target - pad
            along = (miss * INTO).sum(-1, keepdim=True) * INTO
            new = pad + 0.005 * along + 0.03 * (miss - along)
            vel = (new - pad) / dt
            pad = new
            wiped |= wiping.wiped_samples(points, layer.in_plane(s.pressure_centre), s.in_contact)
            if int(layer.phase[0]) == wiping.WIPING:
                in_motion += 1
                in_contact += int(s.in_contact[0])
                forces.append(float(s.normal_n[0]))
                pushes.append(float(layer.push[0]))
        coverage = float(wiped.float().mean())
        contact_share = in_contact / max(1, in_motion)
        rms = math.sqrt(sum((f - p) ** 2 for f, p in zip(forces, pushes)) / max(1, len(forces)))
        return layer, s, coverage, contact_share, rms, forces

    def test_a_well_estimated_wall_is_wiped_end_to_end_at_the_force(self):
        layer, s, coverage, contact, rms, forces = self.run_loop()
        self.assertGreaterEqual(coverage, 0.9)
        self.assertGreaterEqual(contact, 0.8)
        self.assertLess(rms, 1.5)
        self.assertEqual(int(layer.phase[0]), wiping.DONE)
        self.assertFalse(bool(s.in_contact[0]))                   # lifted off at the end

    def test_a_zigzag_is_wiped_too(self):
        _, _, coverage, contact, _, _ = self.run_loop(kind="zigzag", seconds=30.0)
        self.assertGreaterEqual(coverage, 0.9)
        self.assertGreaterEqual(contact, 0.8)

    def test_a_wall_further_or_nearer_than_estimated_is_found_and_pressed_at_the_force(self):
        for offset in (0.008, -0.008):
            layer, _, coverage, contact, rms, forces = self.run_loop(offset=offset)
            self.assertGreaterEqual(coverage, 0.9, offset)
            self.assertGreaterEqual(contact, 0.8, offset)
            self.assertAlmostEqual(sorted(forces)[len(forces) // 2], 10.0, delta=1.5)

    def test_a_tilted_wall_keeps_its_contact(self):
        _, _, coverage, contact, _, _ = self.run_loop(tilt_deg=4.0)
        self.assertGreaterEqual(coverage, 0.9)
        self.assertGreaterEqual(contact, 0.8)

    def test_the_push_never_exceeds_the_wiping_force(self):
        layer = wiping.WipeTaskLayer(1, "cpu")
        points, arc = wiping.resample(wiping.line_path(torch.tensor([0.16]), torch.tensor([0.0])))
        layer.start(torch.arange(1), origin=torch.zeros(1, 3), normal=NORMAL, points=points, arc=arc,
                    standoff=torch.tensor([0.015]), overdrive=torch.tensor([0.015]), force_n=torch.tensor([6.0]),
                    speed=torch.tensor([0.03]), approach_speed=torch.tensor([0.05]))
        pad = torch.tensor([[-0.05, 0.0, 0.0]])                      # a hand held back: blocked, as far as it knows
        for _ in range(1000):
            _, command = layer.step(pad, 0.005)
            self.assertLessEqual(float(layer.push[0]), 6.0 + 1e-6)
        self.assertAlmostEqual(float(layer.push[0]), 6.0, places=4)

    def test_a_slow_hand_in_free_air_is_not_taken_for_a_touch(self):
        layer = wiping.WipeTaskLayer(1, "cpu")
        points, arc = wiping.resample(wiping.line_path(torch.tensor([0.16]), torch.tensor([0.0])))
        layer.start(torch.arange(1), origin=torch.zeros(1, 3), normal=NORMAL, points=points, arc=arc,
                    standoff=torch.tensor([0.015]), overdrive=torch.tensor([0.015]), force_n=torch.tensor([10.0]),
                    speed=torch.tensor([0.03]), approach_speed=torch.tensor([0.02]))
        pad = torch.tensor([[-0.015, 0.0, 0.0]])
        for _ in range(100):                                          # following its reference at 5 mm/s, pushed lightly
            layer.step(pad, 0.005)
            pad = pad + torch.tensor([[0.005 * 0.005, 0.0, 0.0]])
        self.assertLess(float(layer.push[0]), 9.0)
        self.assertFalse(bool(layer.touched[0]))

    def test_the_reference_waits_for_a_pad_that_lags(self):
        layer = wiping.WipeTaskLayer(1, "cpu")
        points, arc = wiping.resample(wiping.line_path(torch.tensor([0.16]), torch.tensor([0.0])))
        layer.start(torch.arange(1), origin=torch.zeros(1, 3), normal=NORMAL, points=points, arc=arc,
                    standoff=torch.tensor([0.015]), overdrive=torch.tensor([0.015]), force_n=torch.tensor([6.0]),
                    speed=torch.tensor([0.05]), approach_speed=torch.tensor([0.05]))
        layer.touched[:] = True
        layer.phase[:] = wiping.WIPING
        pad = layer.to_world(points[:, 0])                          # stuck at the start
        for _ in range(400):
            layer.step(pad, 0.005)
        self.assertLessEqual(float(layer.path_ref[0] - layer.progress[0]), layer.lead_m + 1e-6)
        self.assertGreater(float(layer.force_cmd[0].norm()), 0.0)  # and pushes along the path to free it


class EvaluationTest(unittest.TestCase):
    """The frozen set and the scoring, on synthetic traces."""

    def test_the_frozen_set_is_the_size_it_was_frozen_at(self):
        from unifp_train import wipe_eval
        plan, rows = wipe_eval.build_plan("cpu")
        self.assertEqual(len(rows), 36)
        self.assertEqual(plan["points"].shape, (36, wiping.PATH_SAMPLES, 2))
        self.assertEqual(int(plan["kind"].sum()), 18)                    # half of them zigzags
        self.assertAlmostEqual(float(plan["arc"][plan["kind"] == 0][0, -1]), 0.16, places=5)

    def test_a_loss_counts_only_past_the_debounce(self):
        from unifp_train import wipe_eval
        import numpy as np
        contact = np.array([0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0], dtype=bool)
        self.assertEqual(wipe_eval.loss_events(contact, 5), 1)          # the 1-step gap is not an event
        self.assertEqual(wipe_eval.loss_events(contact, 1), 3)          # nor is the wait before the first touch

    def trace(self, steps=300):
        import numpy as np
        from unifp_train import wipe_eval
        return {key: np.zeros((steps, 1)) for key in wipe_eval.FIELDS}

    def test_success_needs_both_criteria(self):
        from unifp_train import wipe_eval
        tr = self.trace()
        tr["engaged"][10:] = 1.0
        tr["phase"][40:250] = float(wiping.WIPING)
        tr["phase"][250:] = float(wiping.LIFTING)
        tr["in_contact"][40:250] = 1.0
        tr["coverage"][:, 0] = 0.95
        self.assertEqual(wipe_eval.score_episode(tr, 0)["reason"], "success")
        tr["in_contact"][100:190] = 0.0                                  # 90 of 210 steps lost
        result = wipe_eval.score_episode(tr, 0)
        self.assertEqual(result["reason"], "contact lost")
        self.assertEqual(result["loss_events"], 1)
        tr["in_contact"][40:250] = 1.0
        tr["coverage"][:, 0] = 0.8
        self.assertEqual(wipe_eval.score_episode(tr, 0)["reason"], "incomplete")
        tr["phase"][:] = float(wiping.APPROACHING)
        self.assertEqual(wipe_eval.score_episode(tr, 0)["reason"], "no contact")
