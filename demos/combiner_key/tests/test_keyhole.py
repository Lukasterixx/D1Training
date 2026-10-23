"""The cam lock's bezel found in the wrist camera's image (`keyhole.py`), on rendered frames from the insertion's view
pose: the bracket's CAD mount 30 degrees below the approach, the key 85 mm out, the lock 17 cm off. numpy only."""
from __future__ import annotations

import unittest

import numpy as np

from demos.combiner_door import perception
from demos.combiner_door.robot import plan
from demos.combiner_key import geometry as kg
from demos.combiner_key import keyhole as kh
from demos.combiner_key.robot import insert_plan as ip
from demos.cup.pick_demo.grasp import JAW_CENTRE_LINK6
from demos.unifp import decoupled_arm, props

K = np.array([[907.1, 0.0, 640.0], [0.0, 906.0, 360.0], [0.0, 0.0, 1.0]])
SIZE = (1280, 720)


def view():
    """(camera pose, the bezel's model centre, out of the door, u, t) at the insertion's view pose, base frame."""
    box = perception.site_pose(props.BoxSite(0.70, 0.0, 180.0))
    _, out, u, t = ip._frames(box)
    hand = decoupled_arm.hand_rotation(-out, u, 0.0, kg.GRASP_ROLL)
    link6 = plan.pose(hand, ip.jaw_for(box, ip.VIEW_OFF_M) - hand @ np.asarray(JAW_CENTRE_LINK6))
    centre = (box @ np.array([kg.LOCK[0] + ip.LOCK_FACE_OFF_DOOR_M, kg.LOCK[1], kg.LOCK[2], 1.0]))[:3]
    return link6 @ plan.wrist_mount(), centre, out, u, t


class BezelTests(unittest.TestCase):
    def test_it_finds_the_bezel_mm_off_the_model_with_the_key_in_front_of_it(self):
        camera, centre, out, u, t = view()
        for a, b in ((0.003, -0.002), (-0.008, 0.005), (0.0, 0.0)):
            with self.subTest(offset_mm=(1000 * a, 1000 * b)):
                image = kh.render(K, camera, SIZE, centre + a * u + b * t, out, u, t,
                                  occluders=[(860, 380, 1000, 720, 90)], seed=2)
                found = kh.find_bezel(image, K, camera, centre, out, u, t)
                self.assertTrue(found.ok, found.why)
                self.assertAlmostEqual(found.a, a, delta=2e-4)
                self.assertAlmostEqual(found.b, b, delta=2e-4)

    def test_a_door_off_the_models_plane_is_put_right_along_the_cameras_ray(self):
        camera, centre, out, u, t = view()
        for depth in (0.003, -0.003):
            truth = centre + 0.002 * u + depth * out
            found = kh.find_bezel(kh.render(K, camera, SIZE, truth, out, u, t, seed=4), K, camera, centre, out, u, t)
            self.assertTrue(found.ok, found.why)
            self.assertLess(np.linalg.norm(found.at_depth(out, depth) - truth), 2e-4)

    def test_a_plain_door_and_a_rim_mostly_hidden_are_refused(self):
        camera, centre, out, u, t = view()
        plain = np.clip(190.0 + np.random.default_rng(1).normal(0.0, 4.0, SIZE[::-1]), 0, 255)
        self.assertFalse(kh.find_bezel(plain, K, camera, centre, out, u, t).ok)
        hidden = kh.render(K, camera, SIZE, centre, out, u, t, occluders=[(0, 0, 1280, 720, 90)])
        self.assertFalse(kh.find_bezel(hidden, K, camera, centre, out, u, t).ok)

    def test_a_second_dark_disc_nearby_does_not_take_its_place(self):
        """A dark round thing 25 mm away (a screw head, a tag's square): the search starts at the model's place."""
        camera, centre, out, u, t = view()
        image = kh.render(K, camera, SIZE, centre + 0.002 * u, out, u, t, seed=5)
        decoy = kh.render(K, camera, SIZE, centre + 0.025 * t, out, u, t, seed=5)
        image = np.minimum(image, decoy)
        found = kh.find_bezel(image, K, camera, centre, out, u, t)
        self.assertTrue(found.ok, found.why)
        self.assertAlmostEqual(found.a, 0.002, delta=2e-4)
        self.assertAlmostEqual(found.b, 0.0, delta=2e-4)


if __name__ == "__main__":
    unittest.main()
