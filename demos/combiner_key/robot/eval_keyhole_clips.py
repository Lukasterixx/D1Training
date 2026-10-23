"""The keyhole finder scored on wrist RealSense clips against their ground truth (`results/week_06/keyhole_clips/`).

Two scores, per clip:

1. **The image alone** (`bezel_image.find`, the gripper masked, no prior), every frame: right (centre within 0.3 radii
   of the truth's, radius within 25%), wrong (found elsewhere), missed; and in the frames with the lock out of view
   (`absent`), how many found something.
2. **`keyhole.find_bezel` with a model that is wrong by E mm** (`--errors`), every `--every`th frame with a truth: the
   clips have no arm pose, so each frame gets one made up from the truth itself: the camera at the origin, the bezel's
   circle where its ellipse puts it (depth f R / a, tilted acos(b / a) about the major axis), and the "model's" place
   E mm from it in a random direction on that plane. Found right (its centre projects within 0.3 radii of the truth's
   centre) with the model's search alone (`wide_m` 0) and with the image's first (`wide_m` `WIDE_SEARCH_M`).

    python3 -m demos.combiner_key.robot.eval_keyhole_clips [--clips logs/keyhole_clips] [--every 5] [--video]

`--video` also writes each clip with the image's find drawn (green found, the truth thin yellow) beside it as
`<clip>_found.mp4`.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np

from demos.combiner_key import bezel_image, keyhole

ROOT = Path(__file__).resolve().parents[3]
TRUTH = ROOT / "results/week_06/keyhole_clips/ground_truth.json"
#: The D435i's colour stream at 1280x720 (the one the clips were recorded from; the tests' too).
K = np.array([[907.1, 0.0, 640.0], [0.0, 906.0, 360.0], [0.0, 0.0, 1.0]])


def frames(path: Path):
    capture = cv2.VideoCapture(str(path))
    i = 0
    while True:
        ok, image = capture.read()
        if not ok:
            return
        yield i, image
        i += 1


def matches(e, truth) -> bool:
    r = math.sqrt(truth["a"] * truth["b"])
    return math.hypot(e.cx - truth["cx"], e.cy - truth["cy"]) < 0.3 * r and 0.75 < e.r / r < 1.33


def circle_from(truth) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """(centre, normal out of the door, u, t) of a 13.75 mm circle projecting as the truth's ellipse, camera frame."""
    a, b, angle = truth["a"], truth["b"], math.radians(truth["ang"])
    depth = K[0, 0] * keyhole.BEZEL_RADIUS_M / a
    ray = np.linalg.inv(K) @ np.array([truth["cx"], truth["cy"], 1.0])
    centre = ray / ray[2] * depth
    major = np.array([math.cos(angle), math.sin(angle), 0.0])
    tilt = math.acos(min(1.0, b / a))
    towards = -centre / np.linalg.norm(centre)
    c, s = math.cos(tilt), math.sin(tilt)               # Rodrigues: `towards` turned by `tilt` about `major`
    normal = towards * c + np.cross(major, towards) * s + major * float(major @ towards) * (1 - c)
    u = np.cross(normal, [0.0, 1.0, 0.0])
    u /= np.linalg.norm(u)
    return centre, normal, u, np.cross(normal, u)


def draw(image, best, gt) -> np.ndarray:
    """A frame with the truth (thin yellow) and the image's find (green, or why not in red)."""
    image = image.copy()
    if gt is not None:
        cv2.ellipse(image, ((gt["cx"], gt["cy"]), (2 * gt["a"], 2 * gt["b"]), gt["ang"]), (0, 220, 255), 1, cv2.LINE_AA)
    if best.ok:
        cv2.polylines(image, [best.points().astype(np.int32)], True, (60, 220, 60), 2, cv2.LINE_AA)
        label = f"found r {best.r:.0f} px  support {best.support:.2f} contrast {best.contrast:.2f}"
    else:
        label = f"not found: {best.why}"
    cv2.putText(image, label, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (60, 220, 60) if best.ok else (60, 60, 230), 2)
    return image


def run(clips: Path, truth_path: Path, every: int, errors_mm, out: Path | None, video: bool = False) -> dict:
    truth = json.loads(truth_path.read_text())
    report = {"image": {}, "find_bezel": {}}
    rng = np.random.default_rng(0)
    for clip, name in truth["clips"].items():
        absent = {i for lo, hi in truth["absent"].get(clip, []) for i in range(lo, hi + 1)}
        s = dict(frames=0, truth=0, right=0, wrong=0, missed=0, absent=0, found_absent=0, unlabelled=0,
                 found_unlabelled=0)
        fb = {str(e): {"model_only": 0, "image_first": 0, "image_first_how_image": 0, "n": 0} for e in errors_mm}
        times, mask, writer = [], None, None
        for i, image in frames(clips / name):
            if mask is None:
                mask = bezel_image.gripper_mask(image.shape)
                if video:
                    path = clips / f"{Path(name).stem}_found.mp4"
                    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 15.0,
                                             (image.shape[1], image.shape[0]))
            s["frames"] += 1
            t0 = time.perf_counter()
            best, _ = bezel_image.find(image, mask=mask)
            times.append(time.perf_counter() - t0)
            gt = truth["frames"].get(f"{clip}:{i}")
            if writer is not None:
                writer.write(draw(image, best, gt))
            if gt is not None:
                s["truth"] += 1
                if not best.ok:
                    s["missed"] += 1
                elif matches(best, gt):
                    s["right"] += 1
                else:
                    s["wrong"] += 1
            elif i in absent:
                s["absent"] += 1
                s["found_absent"] += int(best.ok)
            else:
                s["unlabelled"] += 1
                s["found_unlabelled"] += int(best.ok)
            if gt is None or i % every:
                continue
            centre, normal, u, t = circle_from(gt)
            r = math.sqrt(gt["a"] * gt["b"])
            rgb = image[..., ::-1]
            for e in errors_mm:
                ang = rng.uniform(0, 2 * np.pi)
                model = centre + 0.001 * e * (math.cos(ang) * u + math.sin(ang) * t)
                row = fb[str(e)]
                row["n"] += 1
                for key, wide in (("model_only", 0.0), ("image_first", keyhole.WIDE_SEARCH_M)):
                    found = keyhole.find_bezel(rgb, K, np.eye(4), model, normal, u, t, wide_m=wide, mask=mask)
                    uv = K @ found.centre
                    right = found.ok and math.hypot(uv[0] / uv[2] - gt["cx"], uv[1] / uv[2] - gt["cy"]) < 0.3 * r
                    row[key] += int(right)
                    if key == "image_first" and right and found.how == "image":
                        row["image_first_how_image"] += 1
        if writer is not None:
            writer.release()
        s["ms_median"] = round(1000 * float(np.median(times)), 1)
        s["ms_p95"] = round(1000 * float(np.percentile(times, 95)), 1)
        report["image"][clip] = s
        report["find_bezel"][clip] = fb
        print(clip, name, json.dumps(s))
        for e, row in fb.items():
            print(f"  model {e:>2} mm off: right {row['model_only']}/{row['n']} model only, "
                  f"{row['image_first']}/{row['n']} image first ({row['image_first_how_image']} from the image's find)")
    if out is not None:
        out.write_text(json.dumps(report, indent=1))
    return report


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--clips", type=Path, default=ROOT / "logs/keyhole_clips")
    p.add_argument("--truth", type=Path, default=TRUTH)
    p.add_argument("--every", type=int, default=5, help="find_bezel on every Nth frame with a truth")
    p.add_argument("--errors", type=float, nargs="+", default=[0, 10, 20, 30, 40], help="the model's error, mm")
    p.add_argument("--out", type=Path, default=TRUTH.parent / "eval.json")
    p.add_argument("--video", action="store_true", help="also write each clip with the find drawn, beside it")
    a = p.parse_args()
    run(a.clips, a.truth, a.every, a.errors, a.out, a.video)


if __name__ == "__main__":
    main()
