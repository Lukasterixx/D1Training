"""The cam lock's bezel found in the wrist camera's image alone (`bezel_image.py`), on frames of the hand-held wrist
clips (`results/week_06/keyhole_clips/`, truth from there), and `keyhole.find_bezel`'s image-first search on rendered
frames with the model's place out by more than its own search. cv2 + numpy."""
from __future__ import annotations

import math
import unittest
from pathlib import Path

import cv2
import numpy as np

from demos.combiner_key import bezel_image as bi
from demos.combiner_key import keyhole as kh
from demos.combiner_key.tests.test_keyhole import K, SIZE, view

DATA = Path(__file__).parent / "data"
#: (frame, the truth's ellipse centre and semi-axes, px): daylight close (480), motion blur (190), the gripper's jaw
#: beside it (380), far beside the tag (820), dim red light close (7000_300) and far (7000_020, 7000_560).
FRAMES = {
    "wrist_1585_190.jpg": (470.8, 236.1, 58.8, 52.4),
    "wrist_1585_380.jpg": (863.9, 483.6, 67.3, 64.9),
    "wrist_1585_480.jpg": (602.7, 342.3, 77.9, 76.6),
    "wrist_1585_820.jpg": (839.2, 50.3, 21.9, 19.7),
    "wrist_7000_020.jpg": (839.7, 47.7, 22.1, 20.0),
    "wrist_7000_300.jpg": (909.6, 353.2, 76.9, 73.9),
    "wrist_7000_560.jpg": (550.4, 59.5, 21.3, 19.7),
}


def load(name):
    return cv2.imread(str(DATA / name))


class ImageTests(unittest.TestCase):
    def test_it_finds_the_bezel_in_daylight_dim_light_blur_and_beside_the_gripper(self):
        for name, (cx, cy, a, b) in FRAMES.items():
            with self.subTest(frame=name):
                image = load(name)
                found, _ = bi.find(image, mask=bi.gripper_mask(image.shape))
                self.assertTrue(found.ok, found.why)
                r = math.sqrt(a * b)
                self.assertLess(math.hypot(found.cx - cx, found.cy - cy), 0.1 * r)
                self.assertAlmostEqual(found.r / r, 1.0, delta=0.1)

    def test_the_logo_and_the_tag_are_not_taken_for_it_with_the_lock_out_of_view(self):
        """The box's "shoals": its "o" is the bezel's shape and size, with letters either side of it."""
        image = load("wrist_1585_030.jpg")
        found, _ = bi.find(image, mask=bi.gripper_mask(image.shape))
        self.assertFalse(found.ok, found.as_dict())

    def test_a_prior_picks_between_two_that_pass(self):
        """Two bezels in one picture: each prior gets its own."""
        image = load("wrist_1585_480.jpg")
        twin = image.copy()
        cx, cy, a, _ = FRAMES["wrist_1585_480.jpg"]
        h = int(2.4 * a)
        twin[250 - h:250 + h, 250 - h:250 + h] = image[int(cy) - h:int(cy) + h, int(cx) - h:int(cx) + h]
        for u, v in ((cx, cy), (250.0, 250.0)):
            found, _ = bi.find(twin, prior=(u, v, a, 60.0, 0.3))
            self.assertTrue(found.ok, found.why)
            self.assertLess(math.hypot(found.cx - u, found.cy - v), 10.0)

    def test_the_gripper_mask_covers_the_lower_right(self):
        mask = bi.gripper_mask((720, 1280))
        self.assertTrue(mask[700, 1270] and mask[600, 1100])
        self.assertFalse(mask[100, 100] or mask[360, 640])


class WideSearchTests(unittest.TestCase):
    def test_a_model_out_by_more_than_its_own_search_is_put_right_by_the_image(self):
        camera, centre, out, u, t = view()
        for a, b in ((0.02, 0.015), (-0.03, 0.02), (-0.025, -0.01)):   # 20-30 mm off, the bezel in the picture
            with self.subTest(offset_mm=(1000 * a, 1000 * b)):
                image = kh.render(K, camera, SIZE, centre + a * u + b * t, out, u, t, seed=3)
                narrow = kh.find_bezel(image, K, camera, centre, out, u, t)
                self.assertFalse(narrow.ok and abs(narrow.a - a) < 1e-3, "the model's own search reached it")
                wide = kh.find_bezel(image, K, camera, centre, out, u, t, wide_m=kh.WIDE_SEARCH_M)
                self.assertTrue(wide.ok, wide.why)
                self.assertEqual(wide.how, "image")
                self.assertAlmostEqual(wide.a, a, delta=3e-4)
                self.assertAlmostEqual(wide.b, b, delta=3e-4)

    def test_close_to_the_model_it_agrees_with_the_models_own_search(self):
        camera, centre, out, u, t = view()
        image = kh.render(K, camera, SIZE, centre + 0.003 * u - 0.002 * t, out, u, t,
                          occluders=[(860, 380, 1000, 720, 90)], seed=2)
        narrow = kh.find_bezel(image, K, camera, centre, out, u, t)
        wide = kh.find_bezel(image, K, camera, centre, out, u, t, wide_m=kh.WIDE_SEARCH_M)
        self.assertTrue(narrow.ok and wide.ok)
        self.assertLess(np.hypot(wide.a - narrow.a, wide.b - narrow.b), 2e-4)

    def test_a_disc_beyond_the_window_is_not_taken_and_it_falls_back_to_the_model(self):
        camera, centre, out, u, t = view()
        image = kh.render(K, camera, SIZE, centre + 0.002 * u, out, u, t, seed=5)
        decoy = kh.render(K, camera, SIZE, centre + 0.07 * t, out, u, t, seed=5)
        found = kh.find_bezel(np.minimum(image, decoy), K, camera, centre, out, u, t, wide_m=kh.WIDE_SEARCH_M)
        self.assertTrue(found.ok, found.why)
        self.assertAlmostEqual(found.a, 0.002, delta=3e-4)


if __name__ == "__main__":
    unittest.main()
