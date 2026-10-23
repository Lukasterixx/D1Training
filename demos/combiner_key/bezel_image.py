"""The cam lock's bezel found in the wrist camera's image from the IMAGE ALONE: a dark disc with light door all round
it, anywhere in the picture, at any size. cv2 + numpy.

`keyhole.find_bezel` fits the bezel's 13.75 mm circle on the door's plane round where the model puts it, ±12 mm: it is
precise, but it only looks there, and the model's place is only as good as the look and the wrist camera's mount (a
mount 3-5 cm out is what moved the box's look onto the front camera). This finds the bezel first wherever it is in the
frame, so the plane fit can start on it.

**The method.**
1. **Candidates**: at each radius (9-240 px, steps of 1.19x), on the pyramid level where that radius is 2.5-5 px (so
   the cost does not grow with it), a ring just outside (1.12-1.5 r) and a band just inside (0.6-0.92 r, clear of the
   chrome core at 0.58 r) are each cut into 8 sectors. The response is the second-darkest outer sector less the
   second-lightest inner one, over the outer: a dark disc with light ALL round, one sector of each allowed lost (a
   finger, the image's edge). A mean round the ring instead let clutter and the letters of the box's logo outrank a
   small bezel; a straight dark edge or a dark disc beside other dark things scores nothing this way.
2. **Refinement**: along 90 rays from each candidate, the rim point where the mean just outside less the mean just
   inside (over the outside) peaks; a RANSAC ellipse through them (perspective makes the circle an ellipse).
3. **Verification**, on the ellipse: at 72 angles, the door 1.2-1.4 radii out against the band 0.7-0.86 in.
   `support`: the share of the angles where it is lighter outside by `EDGE_RATIO`; `contrast`: the median ratio;
   `isolation`: the share of the surroundings 1.6-2.2 radii out that is lit like the door (the "o" of the box's logo
   is the bezel's shape and size, with letters on both sides of it); `seen`: the share of the rim in the image and off
   the gripper; `residual`: the rim points' median distance from the ellipse, over its radius.

**Measured** (`robot/eval_keyhole_clips.py`, 2026-10-23): two hand-held wrist clips past the box (1470 frames, 1280x720,
daylight and dim red room light, heavy motion blur in places, the bezel 9-90 px in radius). Ground truth from the
bezel tracked through each clip and checked frame by frame. With these gates and no prior: 1319 of 1338 frames
found right, 5 wrong (a far, blurred bezel ~10 px in radius losing to a tag's cell or clutter), 0 found in 91 frames
with the lock out of view, the "shoals" logo in them; 14 missed (blur, the rim off the image's edge). ~60 ms a
frame on the dev PC searching every radius; a prior's radius range makes it a few scales.

The gripper's jaw is always in the lower right of the wrist camera's image (`GRIPPER`, fitted to those clips): its
rounded end against the door is a big dark disc, and next to the bezel it spoils the isolation. `gripper_mask` turns
it into "not seen" for every test.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

#: Radii searched, full-resolution pixels: 9 px is the bezel ~1.4 m off at the D435's 907 px focal length.
R_MIN_PX, R_MAX_PX, R_STEP = 9.0, 240.0, 1.19
#: Each radius is filtered on the pyramid level where it is LEVEL_R to 2 LEVEL_R px.
LEVEL_R = 2.5
SECTORS = 8
CANDIDATE_MIN = 0.10
CANDIDATES = 20
RAYS = 90
#: A rim angle supports the ellipse when the door outside is this much lighter, as a share of itself, than the band.
EDGE_RATIO = 0.2
#: Accepted: from the two clips, every gate round the 5th percentile of the bezel's own values and beyond most others.
SUPPORT_MIN = 0.9
CONTRAST_MIN = 0.5
ISOLATION_MIN = 0.9
SEEN_MIN = 0.6
#: b/a: 0.45 is the circle seen 63 degrees off its axis.
AXIS_MIN = 0.45
RESIDUAL_MAX = 0.05
#: The gripper's jaw in the wrist camera's image (fractions of width, height), fitted to the two clips: the region dark
#: in more than 85% of their frames, lower right.
GRIPPER = ((0.997, 0.728), (0.997, 0.994), (0.716, 0.994), (0.753, 0.700), (0.781, 0.611), (0.859, 0.600))


@dataclass
class Ellipse:
    """A verified (or refused) dark disc: centre, semi-axes (px) and the first axis's angle (deg, cv2's convention);
    its tests (module docstring), its score and, if refused, why."""
    ok: bool
    cx: float = 0.0
    cy: float = 0.0
    a: float = 0.0
    b: float = 0.0
    angle: float = 0.0
    support: float = 0.0
    contrast: float = 0.0
    isolation: float = 0.0
    seen: float = 0.0
    residual: float = 0.0
    score: float = 0.0
    why: str = ""

    @property
    def r(self) -> float:
        return math.sqrt(self.a * self.b)

    def points(self, scale: float = 1.0, n: int = 72) -> np.ndarray:
        """(n, 2) points round the ellipse scaled by `scale`."""
        ang = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
        return np.stack(_ellipse_xy(self, scale, ang), axis=-1)

    def as_dict(self) -> dict:
        return {"ok": self.ok, "centre_px": [round(self.cx, 1), round(self.cy, 1)],
                "axes_px": [round(self.a, 1), round(self.b, 1)], "angle_deg": round(self.angle, 1),
                "support": round(self.support, 2), "contrast": round(self.contrast, 2),
                "isolation": round(self.isolation, 2), "seen": round(self.seen, 2),
                "residual": round(self.residual, 3), "score": round(self.score, 3), "why": self.why}


def gripper_mask(shape, poly=GRIPPER, grow: float = 0.08) -> np.ndarray:
    """True on the gripper's jaw in an image of `shape`: `poly` grown `grow` about its own centre."""
    h, w = shape[:2]
    p = np.asarray(poly, float)
    p = p.mean(axis=0) + (p - p.mean(axis=0)) * (1.0 + grow)
    mask = np.zeros((h, w), np.uint8)
    cv2.fillPoly(mask, [np.round(p * [w, h]).astype(np.int32)], 1)
    return mask.astype(bool)


def grey(image) -> np.ndarray:
    """RGB or BGR (or grey) as float32 grey: the channels' mean, which does not care about their order."""
    image = np.asarray(image)
    return (image.mean(axis=2) if image.ndim == 3 else image).astype(np.float32)


_KERNELS: dict = {}


def _ring_kernel(r0: float, r1: float, sector: int) -> np.ndarray:
    """The normalised kernel of sector `sector` of `SECTORS` of the ring r0 <= d < r1."""
    key = (round(r0, 3), round(r1, 3), sector)
    if key not in _KERNELS:
        R = int(math.ceil(r1))
        y, x = np.mgrid[-R:R + 1, -R:R + 1]
        d = np.hypot(x, y)
        s = np.floor((np.arctan2(y, x) + np.pi) / (2 * np.pi) * SECTORS).astype(int) % SECTORS
        k = ((d >= r0) & (d < r1) & (s == sector)).astype(np.float32)
        _KERNELS[key] = k / max(float(k.sum()), 1.0)
    return _KERNELS[key]


def candidates(g: np.ndarray, r_min: float = R_MIN_PX, r_max: float = R_MAX_PX, n: int = CANDIDATES) -> list:
    """(x, y, r, response) of the best `n` dark discs in grey image `g` (full-resolution pixels), best first."""
    out, levels = [], {}
    r = r_min
    while r <= r_max:
        level = max(0, int(math.floor(math.log2(r / LEVEL_R))))
        if level not in levels:
            img = g
            for _ in range(level):
                img = cv2.pyrDown(img)
            levels[level] = img
        img, s = levels[level], 2 ** level
        rl = r / s
        inner = np.stack([cv2.filter2D(img, -1, _ring_kernel(0.6 * rl, 0.92 * rl, k), borderType=cv2.BORDER_REFLECT)
                          for k in range(SECTORS)])
        outer = np.stack([cv2.filter2D(img, -1, _ring_kernel(1.12 * rl, 1.5 * rl, k), borderType=cv2.BORDER_REFLECT)
                          for k in range(SECTORS)])
        inner.sort(axis=0)
        outer.sort(axis=0)
        response = (outer[1] - inner[-2]) / (outer[1] + 8.0)
        size = max(3, int(rl) | 1)
        peak = cv2.dilate(response, np.ones((size, size), np.uint8))
        for y, x in zip(*np.nonzero((response >= peak) & (response > CANDIDATE_MIN))):
            out.append(((x + 0.5) * s - 0.5, (y + 0.5) * s - 0.5, r, float(response[y, x])))
        r *= R_STEP
    out.sort(key=lambda c: -c[3])
    kept = []
    for c in out:
        if all(math.hypot(c[0] - k[0], c[1] - k[1]) > 0.7 * max(c[2], k[2]) or not 0.6 < c[2] / k[2] < 1.67
               for k in kept):
            kept.append(c)
            if len(kept) >= n:
                break
    return kept


def _sample(img: np.ndarray, x, y) -> np.ndarray:
    """Bilinear samples; NaN off the image (and wherever `img` is NaN: the mask)."""
    return cv2.remap(img, np.asarray(x, np.float32), np.asarray(y, np.float32), cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_CONSTANT, borderValue=float("nan"))


def _ellipse_xy(e: Ellipse, scale: float, ang: np.ndarray):
    th = math.radians(e.angle)
    x, y = e.a * scale * np.cos(ang), e.b * scale * np.sin(ang)
    return e.cx + math.cos(th) * x - math.sin(th) * y, e.cy + math.sin(th) * x + math.cos(th) * y


def _residual(pts, cx, cy, A, B, angle) -> np.ndarray:
    th = math.radians(angle)
    dx, dy = pts[:, 0] - cx, pts[:, 1] - cy
    u = (math.cos(th) * dx + math.sin(th) * dy) / max(A, 1e-6)
    v = (-math.sin(th) * dx + math.cos(th) * dy) / max(B, 1e-6)
    return np.abs(np.hypot(u, v) - 1.0) * math.sqrt(A * B)


def _fit(pts):
    """cv2's ellipse through `pts` as (cx, cy, major semi-axis, minor, the major axis's angle)."""
    (cx, cy), (w, h), angle = cv2.fitEllipse(pts)
    A, B = w / 2, h / 2
    if A < B:
        A, B, angle = B, A, angle + 90.0
    return cx, cy, A, B, angle % 180.0


def refine(g: np.ndarray, x0: float, y0: float, r0: float, seed: int = 0) -> Ellipse:
    """The ellipse through the rim round the candidate (x0, y0, r0), verified."""
    ang = np.linspace(0, 2 * np.pi, RAYS, endpoint=False)
    step = max(0.5, r0 / 40)
    s = np.arange(0.4 * r0, 1.8 * r0, step)
    X = x0 + np.cos(ang)[:, None] * s[None]
    Y = y0 + np.sin(ang)[:, None] * s[None]
    profile = _sample(g, X, Y)
    w = max(2, int(round(0.12 * r0 / step)))
    n = profile.shape[1]
    if n <= 2 * w:
        return Ellipse(False, why="too small")
    cs = np.concatenate([np.zeros((RAYS, 1)), np.cumsum(np.nan_to_num(profile, nan=0.0), axis=1)], axis=1)
    idx = np.arange(w, n - w)
    outside = (cs[:, idx + w] - cs[:, idx]) / w
    inside = (cs[:, idx] - cs[:, idx - w]) / w
    valid = np.isfinite(profile[:, idx - w]) & np.isfinite(profile[:, idx + w - 1])
    across = np.where(valid, (outside - inside) / (outside + 8.0), -1.0)
    j = np.argmax(across, axis=1)
    rows = np.arange(RAYS)
    good = across[rows, j] > 0.15
    if good.sum() < 12:
        return Ellipse(False, why="few rim points")
    pts = np.stack([X[rows, idx[j]], Y[rows, idx[j]]], axis=1)[good].astype(np.float32)
    rng = np.random.default_rng(seed)
    tol = max(1.5, 0.05 * r0)
    best, best_n = None, 0
    for _ in range(80):
        try:
            cx, cy, A, B, angle = _fit(pts[rng.choice(len(pts), 6, replace=False)])
        except cv2.error:
            continue
        if not np.isfinite([cx, cy, A, B]).all() or B < 2 or A > 3 * r0:
            continue
        inliers = _residual(pts, cx, cy, A, B, angle) < tol
        if inliers.sum() > best_n:
            best, best_n = inliers, int(inliers.sum())
    if best is None or best_n < 10:
        return Ellipse(False, why="no ellipse")
    fit = _fit(pts[best])
    inliers = _residual(pts, *fit) < tol
    if inliers.sum() >= 10:
        fit = _fit(pts[inliers])
    else:
        inliers = best
    e = Ellipse(True, *[float(v) for v in fit])
    e.residual = float(np.median(_residual(pts[inliers], *fit)) / max(e.r, 1e-6))
    return verify(g, e)


def verify(g: np.ndarray, e: Ellipse) -> Ellipse:
    """`e`'s tests (module docstring) on grey image `g` (NaN where masked), its score, and whether it passes."""
    ang = np.linspace(0, 2 * np.pi, 72, endpoint=False)
    ring = lambda scales: np.array([_sample(g, *_ellipse_xy(e, k, ang)).ravel() for k in scales])  # noqa: E731
    out, band, far = ring((1.2, 1.3, 1.4)), ring((0.7, 0.78, 0.86)), ring((1.6, 1.9, 2.2))
    out_ok, band_ok = np.isfinite(out).any(axis=0), np.isfinite(band).any(axis=0)
    seen = out_ok & band_ok
    e.seen = float(seen.mean())
    if seen.sum() < 8:
        e.ok, e.why = False, "out of the image"
        return e
    with np.errstate(invalid="ignore"):
        door = np.nanmean(np.where(np.isfinite(out), out, np.nan)[:, seen], axis=0)
        level = np.nanmean(np.where(np.isfinite(band), band, np.nan)[:, seen], axis=0)
        ratio = (door - level) / (door + 8.0)
        e.support = float(np.mean(ratio > EDGE_RATIO))
        e.contrast = float(np.median(ratio))
        door_m, level_m = float(np.median(door)), float(np.median(level))
        lit = far[np.isfinite(far)] > level_m + 0.6 * (door_m - level_m)
    e.isolation = float(lit.mean()) if lit.size else 0.0
    e.score = e.support * e.contrast * e.isolation ** 2
    axis = e.b / max(e.a, 1e-6)
    for name, value, bound, low in (("seen", e.seen, SEEN_MIN, True), ("support", e.support, SUPPORT_MIN, True),
                                    ("contrast", e.contrast, CONTRAST_MIN, True),
                                    ("isolation", e.isolation, ISOLATION_MIN, True), ("axis", axis, AXIS_MIN, True),
                                    ("residual", e.residual, RESIDUAL_MAX, False)):
        if (value < bound) if low else (value > bound):
            e.ok, e.why = False, f"{name} {value:.2f}"
            break
    return e


def find(image, mask=None, r_range=None, prior=None) -> tuple[Ellipse, list]:
    """The bezel in `image` (RGB, BGR or grey): (the best verified `Ellipse`, or a refused one saying why; every
    candidate's `Ellipse`). `mask`: True where the image is not the scene (`gripper_mask`). `r_range`: (min, max) radius
    in px. `prior`: (column, row, radius, sigma_px, sigma_log_r): where and how big the bezel is expected; it scales
    each score by a Gaussian of the distance and of the log radius ratio: between two that pass, the one there wins."""
    g = cv2.GaussianBlur(grey(image), (0, 0), 1.0)
    r_min, r_max = (R_MIN_PX, R_MAX_PX) if r_range is None else (max(4.0, r_range[0]), r_range[1])
    if mask is not None and mask.any():
        light = float(np.percentile(g[~mask], 90)) if (~mask).any() else 255.0
        found = candidates(np.where(mask, light, g).astype(np.float32), r_min, r_max)
        g = np.where(mask, np.nan, g).astype(np.float32)
    else:
        found = candidates(g, r_min, r_max)
    every = []
    for x, y, r, _ in found:
        e = refine(g, x, y, r)
        if e.ok and prior is not None:
            u, v, rp, sigma_px, sigma_r = prior
            e.score *= math.exp(-0.5 * (math.hypot(e.cx - u, e.cy - v) / sigma_px) ** 2
                                - 0.5 * (math.log(e.r / rp) / sigma_r) ** 2)
        every.append(e)
    passed = [e for e in every if e.ok]
    if passed:
        return max(passed, key=lambda e: e.score), every
    why = "no dark disc" if not every else "none passed: " + ", ".join(sorted({e.why.split()[0] for e in every}))
    return Ellipse(False, why=why), every
