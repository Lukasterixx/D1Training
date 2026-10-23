"""The cam lock's keyhole found in the wrist camera's image, to put the key on it. numpy only.

What is found is the lock's **bezel**: a black disc 27.5 mm across (Lukas's callipers, 2026-10-21) round the chrome
cylinder, on the box's light grey door. Its rim is the strongest, most constant edge near the keyway: dark inside,
light outside, the same size from every side. The keyway itself, a 15 mm slot in chrome, is small, specular and lit by
whatever the wrist sees; its angle is not needed (the hand holds the key at the lock's 45 degrees). The cylinder is
concentric with the bezel, so the bezel's centre is the keyway's.

**The method: the known circle on the known plane.** The door's plane and the lock's place on it come from the look
(the tag and the lidar); the camera's pose from the arm's encoders. So the bezel can be one thing only: a 13.75 mm
circle on that plane near where the model puts it. Candidate centres on the plane (and a few mm along its normal, for
the plane's own error) are each projected as that circle, and scored by the image's contrast across it: at rim points
round it, the intensity just outside less just inside (`RIM_STEPS_M` either side, narrowing as it refines). The rim of the real bezel scores
high all round; a circle off it scores on a few points at most. A coarse grid over `SEARCH_M` and then finer ones
refine it. It needs no training data and no thresholds tuned to the lighting (the score is relative), it is unaffected
by perspective (the circle is projected, not assumed round in the image), and it tolerates a rim partly hidden: the
key, the fingers and the bar in front of it only take their points out of the score.

**Accepted** only if enough of the rim is seen dark-inside (`MIN_SUPPORT`) and the best centre stands clear of the
best one a few mm away (`MIN_PEAK_RATIO`): a plain door, a missing lock or a shadow fails one of them.

**Found in the image first** (`wide_m`, `bezel_image.py`, cv2): the search above covers +-`SEARCH_M` round the model's
place, and the model is only as good as the look and the wrist camera's mount. With `wide_m` > 0 the bezel is first
looked for in the image alone, inside the projection of +-`wide_m` round the model's place and at the size the model's
distance gives it (`WIDE_RADIUS_RANGE`); the ray through the ellipse's centre meets the model's plane, and the search
above is centred there. If the image finds nothing it falls back to the model's place; the result says which
(`Bezel.how`). Measured on hand-held wrist clips (`bezel_image.py`), the image search alone finds it in 99% of frames
in daylight and dim red light with nothing false found.

Not handled: lens distortion (the D435's colour stream is near rectilinear; at the image's edge a pixel or two),
motion blur (the attempt holds the arm still to look), and a door far off the plane the look gives (more than
`DEPTH_SEARCH_M`).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

#: The bezel's radius (27.5 mm across).
BEZEL_RADIUS_M = 0.01375
#: The contrast is sampled this far inside and outside the rim, on the plane: wide for the coarse search (it finds the
#: rim from 2 mm off), narrow for the last (it peaks on the edge itself; any circle within the step of it scores the
#: same).
RIM_STEPS_M = (0.002, 0.001, 0.0004)
RIM_POINTS = 72
#: The search: the bezel's centre this far either way of the model's on the plane. Not along its normal: seen 30 degrees
#: off it, a circle a few mm nearer and shifted projects almost as the true one (its size changes 3.5% over 6 mm at
#: 170 mm), so depth and place trade off. The found centre is on the model's plane; `Bezel.at_depth` carries it along
#: the camera's ray to another depth when one is measured (the key's touch).
SEARCH_M = 0.012
DEPTH_SEARCH_M = 0.0
#: A rim point supports the circle when it is at least this much lighter outside than in (grey levels, 0-255).
EDGE_LEVELS = 15.0
#: Accepted: at least this share of the rim's points support it, and its score is this many times the best a few mm off.
MIN_SUPPORT = 0.40
MIN_PEAK_RATIO = 1.5
PEAK_EXCLUSION_M = 0.004
#: The image's own search (`find_bezel(wide_m=...)`): this far round the model's place on the plane (the wrist mount's
#: error is 3-5 cm), the radius within this factor of the model's (the plane's depth and its tilt), and the fit on the
#: plane must land within this share of the ellipse's radius of its centre.
WIDE_SEARCH_M = 0.045
WIDE_RADIUS_RANGE = (0.6, 1.6)
WIDE_AGREE = 0.35
#: The depth the image's find may put the bezel off the model's plane by its size (`_image_find`), and the share of
#: the range it must disagree by to be used: on renders the size's range is good to 0.1-0.5% at 12-25 cm and 2-3% long
#: at 0.48-0.6 m (the ellipse ~0.5 px small there), a few % more off-axis and tilted.
WIDE_DEPTH_M = 0.06
WIDE_DEPTH_TRUST = 0.06


@dataclass
class Bezel:
    """What was found: its centre (frame of the inputs), its offset on the plane from where the model put it (along
    `u`, along `t`, m) and along the normal, the share of the rim that supports it, its score and the runner-up's."""
    ok: bool
    centre: np.ndarray = field(default_factory=lambda: np.zeros(3))
    a: float = 0.0
    b: float = 0.0
    d: float = 0.0
    support: float = 0.0
    score: float = 0.0
    runner_up: float = 0.0
    why: str = ""
    camera: np.ndarray = field(default_factory=lambda: np.zeros(3))
    how: str = "model"          # "image": the search was centred on the image's own find; "model": on the model's place
    image: dict | None = None   # the image's find (`bezel_image.Ellipse.as_dict`), or why it found none

    def at_depth(self, normal, delta_m: float) -> np.ndarray:
        """The centre moved along the camera's ray through it to a plane `delta_m` further along `normal`."""
        ray = self.centre - self.camera
        normal = np.asarray(normal, float)
        return self.centre + ray * (delta_m / float(ray @ normal))

    def as_dict(self) -> dict:
        return {"ok": self.ok, "offset_mm": [round(1000 * self.a, 2), round(1000 * self.b, 2), round(1000 * self.d, 2)],
                "support": round(self.support, 3), "score": round(self.score, 2), "runner_up": round(self.runner_up, 2),
                "why": self.why, "how": self.how, "image": self.image}


def grey(image) -> np.ndarray:
    """An RGB (or grey) image as float grey levels."""
    image = np.asarray(image, dtype=float)
    return image.mean(axis=2) if image.ndim == 3 else image


def _blur(image: np.ndarray) -> np.ndarray:
    """A 3x3 box blur: the rim's contrast without the pixel noise."""
    padded = np.pad(image, 1, mode="edge")
    out = np.zeros_like(image)
    for dy in range(3):
        for dx in range(3):
            out += padded[dy:dy + image.shape[0], dx:dx + image.shape[1]]
    return out / 9.0


def _sample(image: np.ndarray, uv: np.ndarray) -> np.ndarray:
    """Bilinear samples at continuous pixel coordinates (..., 2: column, row); NaN outside the image."""
    h, w = image.shape
    u, v = uv[..., 0], uv[..., 1]
    inside = (u >= 0) & (v >= 0) & (u <= w - 1.001) & (v <= h - 1.001)
    u0 = np.clip(np.floor(u).astype(int), 0, w - 2)
    v0 = np.clip(np.floor(v).astype(int), 0, h - 2)
    fu, fv = np.clip(u - u0, 0, 1), np.clip(v - v0, 0, 1)
    top = image[v0, u0] * (1 - fu) + image[v0, u0 + 1] * fu
    bottom = image[v0 + 1, u0] * (1 - fu) + image[v0 + 1, u0 + 1] * fu
    return np.where(inside, top * (1 - fv) + bottom * fv, np.nan)


def _project(K, world_to_camera, points) -> np.ndarray:
    """Points (..., 3) to pixel coordinates (..., 2); NaN behind the camera."""
    p = points @ world_to_camera[:3, :3].T + world_to_camera[:3, 3]
    z = p[..., 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        uv = np.stack([K[0, 0] * p[..., 0] / z + K[0, 2], K[1, 1] * p[..., 1] / z + K[1, 2]], axis=-1)
    return np.where((z > 0.02)[..., None], uv, np.nan)


def _scores(image, K, world_to_camera, centres, u, t, step: float) -> tuple[np.ndarray, np.ndarray]:
    """(score, support) of each candidate centre (N, 3): the mean over the rim's points of the contrast across it,
    `step` either side, 0 where not lighter outside or not in the image; the share that supports it."""
    angles = np.linspace(0.0, 2 * np.pi, RIM_POINTS, endpoint=False)
    ring = np.cos(angles)[:, None] * u + np.sin(angles)[:, None] * t              # (P, 3)
    inner = centres[:, None, :] + (BEZEL_RADIUS_M - step) * ring[None]
    outer = centres[:, None, :] + (BEZEL_RADIUS_M + step) * ring[None]
    contrast = _sample(image, _project(K, world_to_camera, outer)) - _sample(image, _project(K, world_to_camera, inner))
    contrast = np.nan_to_num(contrast, nan=0.0)
    score = np.clip(contrast, 0.0, 255.0).mean(axis=1)
    support = (contrast >= EDGE_LEVELS).mean(axis=1)
    return score, support


def find_bezel(image, K, camera_pose, centre, normal, u, t, search_m: float = SEARCH_M,
               depth_search_m: float = DEPTH_SEARCH_M, wide_m: float = 0.0, mask=None) -> Bezel:
    """The bezel in `image` (RGB or grey), seen through intrinsics `K` (3x3) from `camera_pose` (the optical frame,
    ROS axes, in the frame the rest are given in). `centre`: where the model puts the bezel's centre, on its front;
    `normal`: out of the door; `u`, `t`: two unit directions on the door's plane. `wide_m` > 0: look for it in the image
    first, that far round the model's place (`WIDE_SEARCH_M`), `mask` (True: not the scene, e.g.
    `bezel_image.gripper_mask`) left out. Returns a `Bezel`."""
    a0 = b0 = d0 = 0.0
    how, seen = "model", None
    if wide_m > 0:
        abd, seen = _image_find(image, K, camera_pose, centre, normal, u, t, wide_m, mask)
        if abd is not None:
            (a0, b0, d0), how = abd, "image"
    img = _blur(grey(image))
    K = np.asarray(K, dtype=float)
    w2c = np.linalg.inv(np.asarray(camera_pose, dtype=float))
    centre, normal = np.asarray(centre, float), np.asarray(normal, float)
    u, t = np.asarray(u, float), np.asarray(t, float)

    def grid(a0, b0, d0, half, step, dhalf, dstep, rim):
        a = np.arange(-half, half + 1e-9, step) + a0
        b = np.arange(-half, half + 1e-9, step) + b0
        d = np.arange(-dhalf, dhalf + 1e-9, dstep) + d0 if dhalf > 0 else np.array([d0])
        A, B, D = np.meshgrid(a, b, d, indexing="ij")
        abd = np.stack([A.ravel(), B.ravel(), D.ravel()], axis=-1)
        points = centre + abd[:, :1] * u + abd[:, 1:2] * t + abd[:, 2:3] * normal
        score, support = _scores(img, K, w2c, points, u, t, rim)
        return abd, score, support

    abd, score, support = grid(a0, b0, d0, search_m, 0.001, depth_search_m, 0.003, RIM_STEPS_M[0])
    coarse = (abd, score)
    best = abd[int(np.argmax(score))]
    for half, step, dhalf, dstep, rim in ((0.002, 0.0005, depth_search_m / 2, 0.001, RIM_STEPS_M[1]),
                                          (0.0008, 0.0001, depth_search_m / 4, 0.0005, RIM_STEPS_M[2])):
        abd, score, support = grid(*best, half, step, dhalf, dstep, rim)
        i = int(np.argmax(score))
        best, best_score, best_support = abd[i], float(score[i]), float(support[i])
    # The runner-up on the coarse grid, a few mm off; the best's own coarse score to compare it with.
    far = np.hypot(coarse[0][:, 0] - best[0], coarse[0][:, 1] - best[1]) > PEAK_EXCLUSION_M
    runner = float(coarse[1][far].max()) if far.any() else 0.0
    peak = float(coarse[1].max())
    found = Bezel(False, centre + best[0] * u + best[1] * t + best[2] * normal, float(best[0]), float(best[1]),
                  float(best[2]), best_support, best_score, runner, camera=np.asarray(camera_pose, float)[:3, 3].copy(),
                  how=how, image=seen)
    off = r_px = 0.0
    if how == "image":
        # The fit on the plane against the image's ellipse: the same bezel, or the fit wandered onto something else.
        uv = _project(K, w2c, found.centre[None])[0]
        off = float(np.hypot(uv[0] - seen["centre_px"][0], uv[1] - seen["centre_px"][1]))
        r_px = float(np.sqrt(seen["axes_px"][0] * seen["axes_px"][1]))
    if off > WIDE_AGREE * r_px:
        found.why = f"the rim fit left the image's bezel ({off:.0f} px off its centre, radius {r_px:.0f} px)"
    elif best_support < MIN_SUPPORT:
        found.why = f"only {best_support:.0%} of the rim seen dark inside (needs {MIN_SUPPORT:.0%})"
    # Found by the image, the bezel's distinctness is the image's own tests (the light all round it). This one, against
    # a centre 4 mm off, refused the bezel from the folded arm's 0.48 m (83 against 65, Lukas's run 2026-10-23), where
    # 4 mm is a fifth of its radius in the picture.
    elif how == "model" and peak < MIN_PEAK_RATIO * runner:
        found.why = f"not distinct: {peak:.1f} against {runner:.1f} {1000 * PEAK_EXCLUSION_M:.0f} mm away"
    else:
        found.ok = True
    return found


def _image_find(image, K, camera_pose, centre, normal, u, t, wide_m: float, mask):
    """The bezel found in the image alone (`bezel_image.find`) near where the model puts it: ((a, b, d): the point on
    the ray through its centre at the depth its size gives, on `u`, `t` and `normal` from `centre`, d within
    +-`WIDE_DEPTH_M`; its `as_dict`), or (None, why not). The rim search starts there: a model whose door is
    centimetres out in depth (the sit-back) put the circle it fits 20% too big and refused the bezel (the dry test,
    2026-10-23); at `key_view`'s ~17 cm a pixel of the bezel's ~70 px radius is ~2.5 mm of range."""
    from demos.combiner_key import bezel_image

    K = np.asarray(K, float)
    pose = np.asarray(camera_pose, float)
    w2c = np.linalg.inv(pose)
    centre, normal = np.asarray(centre, float), np.asarray(normal, float)
    u, t = np.asarray(u, float), np.asarray(t, float)
    angles = np.linspace(0.0, 2 * np.pi, 36, endpoint=False)
    ring = _project(K, w2c, centre + BEZEL_RADIUS_M * (np.cos(angles)[:, None] * u + np.sin(angles)[:, None] * t))
    middle = _project(K, w2c, centre[None])[0]
    if not np.isfinite(ring).all():
        return None, {"ok": False, "why": "the model's bezel is behind the camera"}
    x, y = ring[:, 0], ring[:, 1]
    r_px = float(np.sqrt(abs(0.5 * np.sum(x * np.roll(y, 1) - np.roll(x, 1) * y)) / np.pi))
    depth = float((w2c[:3, :3] @ centre + w2c[:3, 3])[2])
    window_px = K[0, 0] * wide_m / max(depth, 1e-3)
    lo, hi = WIDE_RADIUS_RANGE
    # Only the window (and a ring of door round a bezel at its edge) is searched: the whole frame cost ~1.3 s inside the
    # dog's node, where the window is a fraction of it.
    image = np.asarray(image)
    h, w = image.shape[:2]
    reach = window_px + 2.4 * hi * r_px
    # Its corner on a multiple of 32 px: the pyramid's levels then sample the frame as they would uncropped.
    x0, x1 = int(max(0, middle[0] - reach)) // 32 * 32, int(min(w, middle[0] + reach + 1))
    y0, y1 = int(max(0, middle[1] - reach)) // 32 * 32, int(min(h, middle[1] + reach + 1))
    if x1 - x0 < 16 or y1 - y0 < 16:
        return None, {"ok": False, "why": "the image search's window is off the image"}
    best, every = bezel_image.find(image[y0:y1, x0:x1], mask=None if mask is None else mask[y0:y1, x0:x1],
                                   r_range=(lo * r_px, hi * r_px),
                                   prior=(middle[0] - x0, middle[1] - y0, r_px, 0.5 * window_px, 0.25))
    for e in every:
        e.cx, e.cy = e.cx + x0, e.cy + y0
    origin, rotation = pose[:3, 3], pose[:3, :3]

    def on_plane(e):
        ray = rotation @ (np.linalg.inv(K) @ np.array([e.cx, e.cy, 1.0]))
        denom = float(ray @ normal)
        if abs(denom) < 1e-6:
            return None
        p = origin + ray * (float((centre - origin) @ normal) / denom)
        return float((p - centre) @ u), float((p - centre) @ t)

    near = []
    for e in every:
        ab = on_plane(e) if e.ok else None
        if ab is not None and np.hypot(*ab) <= wide_m:
            near.append((e.score, ab, e))
    if not near:
        why = best.why if not best.ok else f"the image's bezel is more than {1000 * wide_m:.0f} mm from the model's"
        return None, {"ok": False, "why": why}
    _, ab, e = max(near, key=lambda n: n[0])
    # How far out of the model's plane its size puts it: the depth along its ray at which the 13.75 mm circle, on the
    # model's door orientation, projects to the area of the ellipse found (tilt and off-axis stretch included).
    ray = rotation @ (np.linalg.inv(K) @ np.array([e.cx, e.cy, 1.0]))
    ring_ut = np.cos(angles)[:, None] * u + np.sin(angles)[:, None] * t

    def at_depth(d):
        return origin + ray * (float((centre + d * normal - origin) @ normal) / float(ray @ normal))

    def r_at(d):
        pts = _project(K, w2c, at_depth(d) + BEZEL_RADIUS_M * ring_ut)
        x, y = pts[:, 0], pts[:, 1]
        return float(np.sqrt(abs(0.5 * np.sum(x * np.roll(y, 1) - np.roll(x, 1) * y)) / np.pi))

    depths = np.linspace(-WIDE_DEPTH_M, WIDE_DEPTH_M, 49)
    sizes = np.array([r_at(d) for d in depths])
    order = np.argsort(sizes)
    sized = float(np.interp(e.r, sizes[order], depths[order]))     # clipped to the ends: at most `WIDE_DEPTH_M`
    model_range = float(np.linalg.norm(centre - origin))
    # Used only when it disagrees with the model's plane by more than size can be trusted to (`WIDE_DEPTH_TRUST` of
    # the range): within that the plane, whose rim fit is the more precise, is kept.
    d = sized if abs(sized) > WIDE_DEPTH_TRUST * model_range else 0.0
    at = at_depth(d)
    found = e.as_dict()
    found.update(out_mm=round(1000 * sized, 1), out_used=d != 0.0, range_m=round(float(np.linalg.norm(at - origin)), 4),
                 model_range_m=round(model_range, 4), model_r_px=round(r_px, 1))
    return (float((at - centre) @ u), float((at - centre) @ t), d), found


def render(K, camera_pose, size, centre, normal, u, t, door: float = 190.0, bezel: float = 25.0, chrome: float = 150.0,
           occluders=(), noise: float = 4.0, seed: int = 0) -> np.ndarray:
    """A synthetic grey image for tests: the door's plane `door` grey, the bezel's disc `bezel` round a chrome disc of
    8 mm `chrome`, each pixel ray-cast onto the plane; `occluders` are (column0, row0, column1, row1, grey) boxes in
    front; Gaussian noise. `size` (width, height)."""
    w, h = size
    K = np.asarray(K, float)
    cols, rows = np.meshgrid(np.arange(w) + 0.0, np.arange(h) + 0.0)
    rays = np.stack([(cols - K[0, 2]) / K[0, 0], (rows - K[1, 2]) / K[1, 1], np.ones_like(cols)], axis=-1)
    pose = np.asarray(camera_pose, float)
    origin, rays_w = pose[:3, 3], rays @ pose[:3, :3].T
    normal = np.asarray(normal, float)
    denom = rays_w @ normal
    with np.errstate(divide="ignore", invalid="ignore"):
        s = ((np.asarray(centre, float) - origin) @ normal) / denom
    hit = origin + s[..., None] * rays_w
    rel = hit - np.asarray(centre, float)
    r = np.hypot(rel @ np.asarray(u, float), rel @ np.asarray(t, float))
    image = np.where(r <= 0.008, chrome, np.where(r <= BEZEL_RADIUS_M, bezel, door))
    for c0, r0, c1, r1, g in occluders:
        image[int(r0):int(r1), int(c0):int(c1)] = g
    rng = np.random.default_rng(seed)
    return np.clip(image + rng.normal(0.0, noise, image.shape), 0, 255)
