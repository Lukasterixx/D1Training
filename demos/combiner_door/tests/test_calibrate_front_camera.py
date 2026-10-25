"""The front camera's board calibration (`calibrate_front_camera.py`): the foreign-tag filter and the fit itself."""
import unittest

import numpy as np

from demos.combiner_door import calibrate_front_camera as cal, front_camera


def _board_image(px_per_mm=2, margin=200):
    board = cal.charuco_board()
    img = board.generateImage((int(cal.COLS * cal.SQUARE_MM * px_per_mm), int(cal.ROWS * cal.SQUARE_MM * px_per_mm)),
                              marginSize=0, borderBits=1)
    import cv2
    return cv2.copyMakeBorder(img, margin, margin, margin, margin + 300, cv2.BORDER_CONSTANT, value=255)


class ForeignTags(unittest.TestCase):
    def test_the_dock_tag_beside_the_board_no_longer_hides_it(self):
        """Another tag36h11 id 0 beside the board (the dock's, 2026-10-24) made OpenCV's own detector find no corner;
        with the filter all 40 are found."""
        import cv2

        image = _board_image()
        tag = cv2.aruco.generateImageMarker(cal.charuco_board().getDictionary(), 0, 60, borderBits=1)
        h, w = image.shape
        image[h // 2 - 30:h // 2 + 30, w - 200:w - 140] = tag
        _, ids, _, _ = cv2.aruco.CharucoDetector(cal.charuco_board()).detectBoard(image)
        self.assertTrue(ids is None or len(ids) == 0)
        found, (seen, kept) = cal.Detector()(image)
        self.assertEqual(len(found[2]), (cal.COLS - 1) * (cal.ROWS - 1))
        self.assertEqual(seen - kept, 1)

    def test_a_lone_foreign_tag_is_not_taken_for_the_board(self):
        import cv2

        canvas = np.full((400, 400), 255, np.uint8)
        canvas[150:250, 150:250] = cv2.aruco.generateImageMarker(cal.charuco_board().getDictionary(), 0, 100,
                                                                 borderBits=1)
        found, (seen, kept) = cal.Detector()(canvas)
        self.assertIsNone(found)
        self.assertEqual((seen, kept), (1, 0))


class Fit(unittest.TestCase):
    def test_it_gives_back_a_known_lens(self):
        """Board corners projected through a known fisheye from 30 poses, 0.2 px of noise: the fit returns its rays to
        0.1 degrees over the image (0.25 at worst)."""
        import cv2

        truth = front_camera.CALIBRATED
        board = cal.charuco_board()
        obj = np.asarray(board.getChessboardCorners(), dtype=np.float64)
        obj = obj - obj.mean(axis=0)
        rng = np.random.default_rng(0)
        views = []
        while len(views) < 30:
            rvec = rng.normal(scale=0.4, size=3)
            tvec = np.array([rng.uniform(-0.4, 0.4), rng.uniform(-0.2, 0.2), rng.uniform(0.45, 0.9)])
            cam = (cv2.Rodrigues(rvec)[0] @ obj.T).T + tvec
            uv = truth.project(cam)
            ok = front_camera.in_view(truth, cam, margin_px=5.0)
            if ok.sum() < 20:
                continue
            views.append({"clip": "synthetic", "frame": len(views), "obj": obj[ok],
                          "img": uv[ok] + rng.normal(scale=0.2, size=(ok.sum(), 2))})
        rms, K, D, used, dropped = cal.fit_rejecting(cv2, views, (truth.width, truth.height), front_camera.ONE_STAND)
        self.assertLess(rms, 0.3)
        model = cal.model_from(K, D, (truth.width, truth.height), "")
        rays = cal.ray_difference_deg(model, truth)
        self.assertLess(rays["median"], 0.15)
        self.assertLess(rays["max"], 0.3)


if __name__ == "__main__":
    unittest.main()
