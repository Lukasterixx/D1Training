"""Calibrates the Go2 front camera's lens from videos of the A3 ChArUco board: `cv2.fisheye`, the model
`front_camera.FisheyeModel` measures with, so the result replaces `front_camera.ONE_STAND` directly.

    python -m demos.combiner_door.calibrate_front_camera board charuco_9x6_40mm_A3.pdf
    python -m demos.combiner_door.calibrate_front_camera calibrate logs/front_camera_calibration/raw/*.mp4

**The board** (`board`): 9x6 squares of 40 mm, tag36h11 markers of 30 mm (OpenCV's `DICT_APRILTAG_36h11`, ids 0-26,
OpenCV >= 4.6's layout), on A3 landscape at exact scale with two 100 mm bars to check the print. A scale error the same
on both axes changes no intrinsic, only the fitted distances, so the nominal 40 mm is used unless `--square_mm` says
otherwise; a different scale on the two axes would bias the fit, which is what the bars are for.

**The videos** come from P2Dingo's mobile web app (Camera page, Go2 front camera, Record), which writes the head's
1280x720 stream as `go2front_<ms>.mp4` in the p2dingo container's `photos/web/`. The board need not move: the dog can
walk around a board taped flat to something, as only the relative motion counts.

**Foreign tags.** The dock's tag is tag36h11 id 0 as well, and the combiner box's tag a tag36h11 too: one in view beside
the board gives two id 0s, and `CharucoDetector` then finds no corner at all (2026-10-24: 0 corners in every frame with
the dock tag in view, 40 in the frames without it). `board_markers` keeps a marker only when a detected marker the board
puts next to it sits where the board says, at the right spacing for the two markers' size in the image.

**The fit** takes every `--stride`th frame with at least `--min_corners` corners, thins them to `--max_views` evenly in
time, fits k1-k4, f_x, f_y and the principal point (skew 0), and drops views whose error is over 3x the median (blur:
the dog moves the camera, ~14 fps), refitting until none is. Each clip is also held out in turn: intrinsics fitted on
the others, each held-out view's pose alone fitted on it (`solvePnP` through the fixed lens), and its error reported
beside `ONE_STAND`'s through the same steps. A view's pose absorbs much of a wrong lens, so these errors separate the
lenses weakly; how far the held-out fits' rays are from the full fit's is the better measure of what the data pins down.

Writes `run.json` (the model, its errors, the held-out table, how far `ONE_STAND`'s rays were off) and `views.csv` to
`--out` (default `logs/front_camera_calibration/<time>_calibrate/`), and a coverage image there.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

from . import front_camera

COLS, ROWS = 9, 6
SQUARE_MM, MARKER_MM = 40.0, 30.0
DICTIONARY = "DICT_APRILTAG_36h11"
PAGE_MM = (420.0, 297.0)  # A3 landscape
REPO = Path(__file__).resolve().parents[2]


def charuco_board(square_mm: float = SQUARE_MM):
    import cv2
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICTIONARY))
    return cv2.aruco.CharucoBoard((COLS, ROWS), square_mm / 1000.0, square_mm * MARKER_MM / SQUARE_MM / 1000.0, dictionary)


# --- the print ------------------------------------------------------------------------------------------------------

def write_board_pdf(path: Path) -> None:
    """The board on A3 at exact scale: a raster board at 4 px/mm (square 160 px, tag cell 15 px, all whole) placed by
    its extent in mm, so the PDF scales it without resampling."""
    import cv2
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    px_per_mm = 4
    board = charuco_board()
    img = board.generateImage((int(COLS * SQUARE_MM * px_per_mm), int(ROWS * SQUARE_MM * px_per_mm)), marginSize=0,
                              borderBits=1)
    padded = cv2.copyMakeBorder(img, 80, 80, 80, 80, cv2.BORDER_CONSTANT, value=255)
    _, ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(padded)
    expected = (COLS - 1) * (ROWS - 1)
    if ids is None or len(ids) != expected:
        raise RuntimeError(f"the rendered board shows {0 if ids is None else len(ids)} of {expected} corners")

    page_w, page_h = PAGE_MM
    fig = plt.figure(figsize=(page_w / 25.4, page_h / 25.4))
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_xlim(0, page_w)
    ax.set_ylim(page_h, 0)
    ax.axis("off")
    bw, bh = COLS * SQUARE_MM, ROWS * SQUARE_MM
    x0, y0 = (page_w - bw) / 2, 18.0
    ax.imshow(img, cmap="gray", vmin=0, vmax=255, interpolation="nearest", extent=(x0, x0 + bw, y0 + bh, y0))
    # 100 mm caliper checks, both axes, clear of the board.
    ys = y0 + bh + 10
    ax.add_patch(Rectangle((x0, ys), 100, 1.0, color="k", lw=0))
    for x in (x0, x0 + 100):
        ax.add_patch(Rectangle((x - 0.25, ys - 3), 0.5, 7, color="k", lw=0))
    ax.text(x0 + 104, ys + 0.8, "100 mm", va="center", fontsize=8)
    xs = x0 - 10
    ax.add_patch(Rectangle((xs, y0), 1.0, 100, color="k", lw=0))
    for y in (y0, y0 + 100):
        ax.add_patch(Rectangle((xs - 3, y - 0.25), 7, 0.5, color="k", lw=0))
    ax.text(xs + 0.5, y0 + 104, "100 mm", ha="center", va="top", fontsize=8, rotation=90)
    info = (f"ChArUco {COLS}x{ROWS} squares | square {SQUARE_MM:.0f} mm | marker {MARKER_MM:.0f} mm | "
            f"{DICTIONARY}, ids 0-{len(board.getIds()) - 1} | OpenCV >= 4.6 layout (not legacy) | "
            f"board {bw:.0f} x {bh:.0f} mm\n"
            "Print on A3 at 100% / actual size (no 'fit to page'). Check both 100 mm bars and a full row/column of "
            f"squares ({bw:.0f} / {bh:.0f} mm) with calipers;\n"
            "use the measured square size in the calibration. Mount flat on rigid board, matte paper. "
            "For the Go2 front camera's intrinsics (cv2.fisheye.calibrate).")
    ax.text(x0 + 125, ys - 4, info, va="top", fontsize=6.5, linespacing=1.5)
    fig.savefig(path, format="pdf", dpi=600)


# --- detection ------------------------------------------------------------------------------------------------------

def marker_centres_mm(board) -> dict:
    """Each marker id's centre on the board, mm."""
    return {int(i): np.asarray(c, dtype=float).reshape(4, 3)[:, :2].mean(axis=0) * 1000.0
            for i, c in zip(board.getIds().ravel(), board.getObjPoints())}


def board_markers(corners, ids, centres_mm: dict, marker_mm: float):
    """The detections that belong to the board (module docstring, "Foreign tags"): a marker is kept when some other
    detected marker within 1.5 squares of it on the board sits at 0.6-1.6x the board's spacing, measured in the two
    markers' mean side length in the image. An id seen twice keeps only the copy that passes, or neither if both do."""
    if ids is None or len(ids) == 0:
        return [], np.zeros((0, 1), np.int32)
    ids = ids.ravel().astype(int)
    pts = [np.asarray(c, dtype=float).reshape(4, 2) for c in corners]
    centre = [p.mean(axis=0) for p in pts]
    side = [np.mean(np.linalg.norm(p - np.roll(p, 1, axis=0), axis=1)) for p in pts]
    reach = 1.5 * marker_mm * SQUARE_MM / MARKER_MM + 1e-6
    keep = []
    for a in range(len(ids)):
        if ids[a] not in centres_mm:
            keep.append(False)
            continue
        ok = False
        for b in range(len(ids)):
            if b == a or ids[b] == ids[a] or ids[b] not in centres_mm:
                continue
            board_mm = np.linalg.norm(centres_mm[ids[a]] - centres_mm[ids[b]])
            if board_mm > reach:
                continue
            ratio = (np.linalg.norm(centre[a] - centre[b]) / (0.5 * (side[a] + side[b]))) / (board_mm / marker_mm)
            if 0.6 <= ratio <= 1.6:
                ok = True
                break
        keep.append(ok)
    for i in set(ids.tolist()):
        copies = [k for k in range(len(ids)) if ids[k] == i and keep[k]]
        if len(copies) > 1:
            for k in copies:
                keep[k] = False
    sel = [k for k in range(len(ids)) if keep[k]]
    return [corners[k] for k in sel], ids[sel].reshape(-1, 1).astype(np.int32)


class Detector:
    def __init__(self, square_mm: float = SQUARE_MM):
        import cv2
        self.cv2 = cv2
        self.board = charuco_board(square_mm)
        self.aruco = cv2.aruco.ArucoDetector(self.board.getDictionary())
        self.charuco = cv2.aruco.CharucoDetector(self.board)
        self.centres_mm = marker_centres_mm(self.board)
        self.marker_mm = square_mm * MARKER_MM / SQUARE_MM

    def __call__(self, gray):
        """(object points (N, 3) m, image points (N, 2) px, corner ids (N,)) or None, and (markers seen, kept)."""
        corners, ids, _ = self.aruco.detectMarkers(gray)
        seen = 0 if ids is None else len(ids)
        corners, ids = board_markers(corners, ids, self.centres_mm, self.marker_mm)
        if len(ids) == 0:
            return None, (seen, 0)
        ch_corners, ch_ids, _, _ = self.charuco.detectBoard(gray, markerCorners=corners, markerIds=ids)
        if ch_ids is None or len(ch_ids) == 0:
            return None, (seen, len(ids))
        obj, img = self.board.matchImagePoints(ch_corners, ch_ids)
        return (obj.reshape(-1, 3).astype(np.float64), img.reshape(-1, 2).astype(np.float64),
                ch_ids.ravel().astype(int)), (seen, len(ids))


def scan(path: Path, detector: Detector, stride: int, min_corners: int) -> tuple[list, dict]:
    cv2 = detector.cv2
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {path}")
    views, stats = [], {"frames": 0, "scanned": 0, "with_markers": 0, "with_corners": 0, "kept": 0, "foreign_dropped": 0}
    fps = cap.get(cv2.CAP_PROP_FPS)
    i = 0
    still = None
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if i % stride == 0:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, (seen, kept) = detector(gray)
            stats["scanned"] += 1
            stats["with_markers"] += seen > 0
            stats["foreign_dropped"] += seen - kept
            if found is not None:
                stats["with_corners"] += 1
                if len(found[2]) >= min_corners:
                    views.append({"clip": path.name, "frame": i, "obj": found[0], "img": found[1]})
                    stats["kept"] += 1
            if still is None and found is not None:
                still = frame
        i += 1
    stats["frames"] = i
    stats["fps"] = round(fps, 2)
    stats["size"] = [int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))]
    return views, stats | {"_still": still}


# --- the fit --------------------------------------------------------------------------------------------------------

def _shape(views):
    return ([v["obj"].reshape(-1, 1, 3) for v in views], [v["img"].reshape(-1, 1, 2) for v in views])


def fit(cv2, views, size, guess: front_camera.FisheyeModel):
    """cv2.fisheye.calibrate from `guess`: (rms, K, D, per-view rms)."""
    obj, img = _shape(views)
    K = guess.intrinsic_matrix.copy()
    D = np.asarray(guess.distortion, dtype=float).reshape(4, 1)
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS
    crit = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-9)
    rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(obj, img, tuple(size), K, D, flags=flags, criteria=crit)
    errs = []
    for o, i, r, t in zip(obj, img, rvecs, tvecs):
        p, _ = cv2.fisheye.projectPoints(o, r, t, K, D)
        errs.append(float(np.sqrt(np.mean(np.sum((p - i) ** 2, axis=2)))))
    return float(rms), K, D.ravel(), np.array(errs)


def fit_rejecting(cv2, views, size, guess, factor: float = 3.0, floor_px: float = 1.0, rounds: int = 5):
    used = list(views)
    dropped = []
    for _ in range(rounds):
        rms, K, D, errs = fit(cv2, used, size, guess)
        limit = max(factor * float(np.median(errs)), floor_px)
        bad = errs > limit
        if not bad.any():
            break
        dropped += [v | {"err": float(e)} for v, e, b in zip(used, errs, bad) if b]
        used = [v for v, b in zip(used, bad) if not b]
    for v, e in zip(used, errs):
        v["err"] = float(e)
    return rms, K, D, used, dropped


def pose_error(cv2, view, K, D) -> float:
    """A view's error through a fixed lens, its pose alone fitted: rays through the lens, PnP on them, back through it."""
    img = view["img"].reshape(-1, 1, 2)
    rays = cv2.fisheye.undistortPoints(img, K, D)
    ok, r, t = cv2.solvePnP(view["obj"], rays.reshape(-1, 2), np.eye(3), None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return float("nan")
    p, _ = cv2.fisheye.projectPoints(view["obj"].reshape(-1, 1, 3), r, t, K, D)
    return float(np.sqrt(np.mean(np.sum((p - img) ** 2, axis=2))))


def model_from(K, D, size, source: str) -> front_camera.FisheyeModel:
    return front_camera.FisheyeModel(width=int(size[0]), height=int(size[1]), fx=float(K[0, 0]), fy=float(K[1, 1]),
                                     cx=float(K[0, 2]), cy=float(K[1, 2]), distortion=tuple(float(x) for x in D),
                                     source=source, calibrated=True)


def ray_difference_deg(a: front_camera.FisheyeModel, b: front_camera.FisheyeModel, step: int = 16) -> dict:
    """The angle between the two models' rays through the same pixels: centre, image edges' midpoints, corners, max."""
    u, v = np.meshgrid(np.arange(0, a.width + 1, step, dtype=float), np.arange(0, a.height + 1, step, dtype=float))
    uv = np.column_stack((u.ravel(), v.ravel()))
    ang = np.degrees(np.arccos(np.clip(np.sum(a.rays(uv) * b.rays(uv), axis=1), -1.0, 1.0)))

    def at(points):
        return [round(float(x), 3) for x in np.degrees(np.arccos(np.clip(
            np.sum(a.rays(points) * b.rays(points), axis=1), -1.0, 1.0)))]

    w, h = a.width, a.height
    return {"centre": at([[w / 2, h / 2]])[0], "edge_midpoints_lrtb": at([[0, h / 2], [w, h / 2], [w / 2, 0], [w / 2, h]]),
            "corners": at([[0, 0], [w, 0], [0, h], [w, h]]), "max": round(float(ang.max()), 3),
            "median": round(float(np.median(ang)), 3)}


def coverage_image(cv2, still, used, dropped, path: Path):
    canvas = (0.5 * still).astype(np.uint8) if still is not None else np.zeros((720, 1280, 3), np.uint8)
    for v in dropped:
        for x, y in v["img"]:
            cv2.circle(canvas, (int(round(x)), int(round(y))), 2, (0, 0, 255), -1)
    for v in used:
        for x, y in v["img"]:
            cv2.circle(canvas, (int(round(x)), int(round(y))), 2, (0, 255, 0), -1)
    cv2.putText(canvas, f"{len(used)} views used (green), {len(dropped)} dropped (red)", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    cv2.imwrite(str(path), canvas)


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def calibrate(args) -> int:
    import cv2
    detector = Detector(args.square_mm)
    guess = front_camera.ONE_STAND
    clips, views, still = {}, [], None
    for path in args.videos:
        v, stats = scan(Path(path), detector, args.stride, args.min_corners)
        still = still if still is not None else stats.pop("_still")
        stats.pop("_still", None)
        clips[Path(path).name] = stats | {"sha256": sha256(Path(path)), "bytes": Path(path).stat().st_size}
        views += v
        print(f"{Path(path).name}: {stats['frames']} frames, {stats['scanned']} scanned, markers in "
              f"{stats['with_markers']}, corners in {stats['with_corners']}, {stats['kept']} with >= {args.min_corners}; "
              f"{stats['foreign_dropped']} foreign marker detections dropped")
    size = next(iter(clips.values()))["size"]
    if any(c["size"] != size for c in clips.values()):
        raise SystemExit("the clips differ in size")
    if len(views) > args.max_views:
        views = [views[k] for k in np.linspace(0, len(views) - 1, args.max_views).round().astype(int)]
    rms, K, D, used, dropped = fit_rejecting(cv2, views, size, guess)
    model = model_from(K, D, size, "")
    print(f"fit: {len(used)} views ({len(dropped)} dropped), rms {rms:.3f} px; fx {K[0, 0]:.2f} fy {K[1, 1]:.2f} "
          f"cx {K[0, 2]:.2f} cy {K[1, 2]:.2f}; k1-k4 {', '.join(f'{x:+.5f}' for x in D)}")

    held_out, held_models = {}, {}
    if len(clips) > 1:
        for name in clips:
            train = [v for v in used if v["clip"] != name]
            test = [v for v in used if v["clip"] == name]
            if len(train) < 5 or not test:
                continue
            _, Kt, Dt, _ = fit(cv2, train, size, guess)
            held_models[name] = model_from(Kt, Dt, size, "")
            new = np.array([pose_error(cv2, v, Kt, Dt) for v in test])
            old = np.array([pose_error(cv2, v, guess.intrinsic_matrix, np.asarray(guess.distortion, float))
                            for v in test])
            held_out[name] = {"views": len(test), "fitted_on_views": len(train),
                              "fx_fy_cx_cy": [round(float(x), 2) for x in (Kt[0, 0], Kt[1, 1], Kt[0, 2], Kt[1, 2])],
                              "k1_k4": [round(float(x), 5) for x in Dt],
                              "rms_px_median": round(float(np.nanmedian(new)), 3),
                              "rms_px_p90": round(float(np.nanpercentile(new, 90)), 3),
                              "one_stand_rms_px_median": round(float(np.nanmedian(old)), 3),
                              "one_stand_rms_px_p90": round(float(np.nanpercentile(old, 90)), 3),
                              "rays_against_the_full_fit_deg": ray_difference_deg(model, held_models[name])}
            print(f"held out {name}: {len(test)} views, rms median {held_out[name]['rms_px_median']} px "
                  f"(ONE_STAND {held_out[name]['one_stand_rms_px_median']})")

    rays = ray_difference_deg(model, guess)
    print(f"ONE_STAND's rays against the fit: centre {rays['centre']} deg, corners {rays['corners']}, max {rays['max']}")

    out = Path(args.out) if args.out else (
        REPO / "logs/front_camera_calibration" / f"{dt.datetime.now().strftime('%Y%m%dT%H%M%S')}_calibrate")
    out.mkdir(parents=True, exist_ok=True)
    all_img = np.concatenate([v["img"] for v in used])
    run = {
        "mode": "front_camera_calibration",
        "status": "finished",
        "command": " ".join([Path(sys.executable).name, "-m", "demos.combiner_door.calibrate_front_camera"] + sys.argv[1:]),
        "board": {"cols": COLS, "rows": ROWS, "square_mm": args.square_mm, "marker_mm": args.square_mm * MARKER_MM / SQUARE_MM,
                  "dictionary": DICTIONARY, "print": "charuco_9x6_40mm_A3.pdf"},
        "clips": clips,
        "settings": {"stride": args.stride, "min_corners": args.min_corners, "max_views": args.max_views,
                     "reject": "views over max(3x median, 1 px), refit until none", "initial_guess": "ONE_STAND",
                     "flags": "RECOMPUTE_EXTRINSIC | FIX_SKEW | USE_INTRINSIC_GUESS"},
        "summary": {
            "views_used": len(used), "views_dropped": len(dropped), "corners_used": int(len(all_img)),
            "rms_px": round(rms, 4),
            "view_rms_px_median": round(float(np.median([v["err"] for v in used])), 3),
            "view_rms_px_max": round(float(np.max([v["err"] for v in used])), 3),
            "fx": round(float(K[0, 0]), 3), "fy": round(float(K[1, 1]), 3),
            "cx": round(float(K[0, 2]), 3), "cy": round(float(K[1, 2]), 3),
            "k1": float(D[0]), "k2": float(D[1]), "k3": float(D[2]), "k4": float(D[3]),
            "max_angle_deg": round(float(np.degrees(model.max_angle_rad())), 2),
            "corner_coverage_px": {"u": [round(float(all_img[:, 0].min()), 1), round(float(all_img[:, 0].max()), 1)],
                                   "v": [round(float(all_img[:, 1].min()), 1), round(float(all_img[:, 1].max()), 1)]},
            "held_out": held_out,
            "one_stand_ray_error_deg": rays,
        },
    }
    (out / "run.json").write_text(json.dumps(run, indent=2) + "\n")
    with open(out / "views.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["clip", "frame", "corners", "rms_px", "used"])
        rows = [(v, 1) for v in used] + [(v, 0) for v in dropped]
        for v, is_used in sorted(rows, key=lambda r: (r[0]["clip"], r[0]["frame"])):
            w.writerow([v["clip"], v["frame"], len(v["img"]), round(v["err"], 4), is_used])
    coverage_image(cv2, still, used, dropped, out / "coverage.png")
    print(f"wrote {out}")
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("board", help="write the A3 print")
    b.add_argument("pdf", type=Path)
    c = sub.add_parser("calibrate", help="fit the lens to videos of the board")
    c.add_argument("videos", nargs="+")
    c.add_argument("--square_mm", type=float, default=SQUARE_MM, help="the printed square, as measured")
    c.add_argument("--stride", type=int, default=3)
    c.add_argument("--min_corners", type=int, default=12)
    c.add_argument("--max_views", type=int, default=600)
    c.add_argument("--out", default=None)
    args = parser.parse_args(argv)
    if args.cmd == "board":
        write_board_pdf(args.pdf)
        print(f"wrote {args.pdf}")
        return 0
    return calibrate(args)


if __name__ == "__main__":
    raise SystemExit(main())
