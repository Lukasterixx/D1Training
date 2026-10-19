#!/usr/bin/env python3
"""The Livox MID-360 and Lukas's mount for it, from his Go2 CAD, put in the Go2's `base_link` frame.

    python description/mid360_from_cad.py ~/Downloads/mount_and_mid360_only.stl
    python description/mid360_from_cad.py "~/Downloads/Go2 v115.stl" ~/Downloads/Go2_with_mid360.stl   # the first way

Lukas's Fusion 360 model of the Go2 exists in two versions (STL, metres, exported 2026-10-17 and 2026-10-19): `Go2 v115`
and `Go2_with_mid360`. The second has the lidar and its mount on the front of the trunk (and not v115's head D435i).
Those whole-robot exports are coarsely tessellated, which drew the lidar's dome and the cage as visible facets, so
Lukas exported the two on their own at full resolution (`mount_and_mid360_only.stl`, 2026-10-19, 174,238 triangles,
the same frame). Given one file it is that; given two, it is their difference. What it does:

  1. **Isolate.** One file: it is already the lidar and its mount. Two files: triangles in the second and not in the
     first are.
  2. **Register.** The STL's frame is not `base_link`. The URDF's trunk mesh (`meshes/go2/base.dae`, in `base_link`)
     is fitted into the second file by ICP. It fits to 0.34 mm median with 0.14 deg of rotation, so the frames differ
     by a translation only, `STL_FROM_BASE_LINK_M` (2026-10-19). The translation is applied; the rotation is not.
  3. **Split.** Connected parts inside the lidar's 65 mm footprint and above its base are the lidar; the rest (the
     plate on the trunk, the cage over the lidar, the screws, and the connector on the lidar's back, which sits
     outside the footprint) is the mount.
  4. **Decimate** each connected part as far as it goes while staying within `--tolerance_mm` of the original, both
     ways, and keep it whole where nothing does (`fast_simplification`, if importable; without it nothing is
     decimated). Quadric decimation rounds sharp edges fast: at 0.3 mm the screws come down to about a third and
     the lidar's housing, the plate and the cage stay whole. Then write
       meshes/go2/mid360.stl         the lidar
       meshes/go2/mid360_mount.stl   the mount
     in `base_link`, metres.

Needs numpy and trimesh (the Isaac environment has both). `payload.py` reads the outputs.
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT_DIR = HERE / "meshes" / "go2"
#: `base_link`'s origin in the STL's frame, from the trunk-mesh ICP fit (residual 0.34 mm median, 0.87 mm p90).
STL_FROM_BASE_LINK_M = (-0.0445, 0.0003, 0.1787)
#: The lidar's footprint and floor in `base_link` (its housing is 65 mm square): parts inside it are the lidar.
LIDAR_BOX_M = ((0.1425, 0.2095), (-0.0335, 0.0335), (0.0955, 0.20))


def _key(triangles: np.ndarray) -> set:
    rounded = np.round(triangles, 5)
    return {tuple(map(tuple, sorted(t.tolist()))) for t in rounded}


def trunk_points() -> np.ndarray:
    """The URDF trunk mesh's vertices in `base_link` (the .dae's node matrix applied)."""
    text = (HERE / "meshes" / "go2" / "base.dae").read_text()
    values = re.search(r'<float_array id="base_004-mesh-positions-array" count="\d+">([^<]*)</float_array>', text)
    matrix = np.array(re.search(r'<matrix sid="transform">([^<]*)</matrix>', text).group(1).split(), float)
    points = np.array(values.group(1).split(), float).reshape(-1, 3)
    m = matrix.reshape(4, 4)
    return points @ m[:3, :3].T + m[:3, 3]


def register(target, seed: int = 1) -> np.ndarray:
    """ICP of the trunk mesh into `target` (the trunk's region of the STL); returns the 4x4 fit."""
    import trimesh

    centres = target.triangles_center
    trunk = target.submesh([np.where((centres[:, 2] > 0.10) & (centres[:, 2] < 0.30) & (np.abs(centres[:, 1]) < 0.12))[0]],
                           append=True)
    points = trunk_points()
    sample = points[np.random.default_rng(seed).choice(len(points), 4000, replace=False)]
    init = np.eye(4)
    init[:3, 3] = STL_FROM_BASE_LINK_M
    fit, _, _ = trimesh.registration.icp(sample, trunk, initial=init, threshold=1e-9, max_iterations=40,
                                         reflection=False, scale=False)
    return fit


#: Fractions of a part's triangles tried, fewest first; the first within tolerance is kept.
KEEP_STEPS = (0.05, 0.1, 0.2, 0.3, 0.5, 0.75)


def deviation_m(original, reduced, samples: int = 3000, seed: int = 1) -> float:
    """The larger of the two one-way distances: points sampled on `original` to `reduced`, and `reduced`'s vertices
    to `original`. Sampled, so a bound on what was looked at, not a Hausdorff distance."""
    import trimesh

    points = trimesh.sample.sample_surface(original, samples, seed=seed)[0]
    there = trimesh.proximity.closest_point(reduced, points)[1]
    vertices = reduced.vertices
    if len(vertices) > samples:
        vertices = vertices[np.random.default_rng(seed).choice(len(vertices), samples, replace=False)]
    back = trimesh.proximity.closest_point(original, vertices)[1]
    return float(max(there.max(), back.max()))


def decimate(mesh, tolerance_m: float):
    """Each connected part decimated as far as `KEEP_STEPS` go within `tolerance_m`, or left whole."""
    try:
        import fast_simplification
    except ImportError:
        print("fast_simplification is not importable: nothing decimated")
        return mesh
    import trimesh

    out = []
    for part in mesh.split(only_watertight=False):
        best = part
        for keep in KEEP_STEPS:
            if len(part.faces) * keep < 12:
                continue
            points, faces = fast_simplification.simplify(part.vertices.astype(np.float32),
                                                         part.faces.astype(np.int32), target_reduction=1.0 - keep)
            reduced = trimesh.Trimesh(points, faces, process=True)
            if len(reduced.faces) and deviation_m(part, reduced) <= tolerance_m:
                best = reduced
                break
        out.append(best)
    return trimesh.util.concatenate(out)


def main(argv=None) -> int:
    import trimesh

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("stl", type=Path, nargs="+",
                        help="mount_and_mid360_only.stl; or Go2 v115.stl and Go2_with_mid360.stl, to diff")
    parser.add_argument("--tolerance_mm", type=float, default=0.3,
                        help="Largest sampled deviation a decimated part may have from the original.")
    parser.add_argument("--check_registration", action="store_true",
                        help="Re-run the trunk ICP and print it (needs the whole-robot STL, so the two-file form).")
    args = parser.parse_args(argv)
    if len(args.stl) > 2:
        parser.error("one STL (the lidar and mount alone) or two (the robot without and with them)")

    meshes = [trimesh.load(path.expanduser(), force="mesh", process=False) for path in args.stl]
    if len(meshes) == 1:
        payload = meshes[0]
        payload.merge_vertices(digits_vertex=6)
    else:
        before, after = meshes
        if args.check_registration:
            fit = register(after)
            angle = np.degrees(np.arccos(np.clip((np.trace(fit[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)))
            print(f"trunk fit: base_link at {np.round(fit[:3, 3], 4).tolist()} in the STL, rotated {angle:.2f} deg "
                  f"(used: {STL_FROM_BASE_LINK_M})")
        known = _key(before.triangles)
        added = np.array([i for i, t in enumerate(np.round(after.triangles, 5))
                          if tuple(map(tuple, sorted(t.tolist()))) not in known])
        payload = after.submesh([added], append=True)
        payload.merge_vertices(digits_vertex=5)
    payload.apply_translation(-np.asarray(STL_FROM_BASE_LINK_M))

    lidar, mount = [], []
    (lx, ly, lz) = LIDAR_BOX_M
    for part in payload.split(only_watertight=False):
        (x0, y0, z0), (x1, y1, z1) = part.bounds
        inside = lx[0] <= x0 and x1 <= lx[1] and ly[0] <= y0 and y1 <= ly[1] and lz[0] <= z0 and z1 <= lz[1]
        (lidar if inside else mount).append(part)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for name, parts in (("mid360", lidar), ("mid360_mount", mount)):
        whole = trimesh.util.concatenate(parts)
        mesh = decimate(whole, args.tolerance_mm / 1000.0)
        mesh.export(OUT_DIR / f"{name}.stl")
        print(f"{name}.stl: {len(mesh.faces)} triangles (of {len(whole.faces)}), "
              f"bounds {np.round(mesh.bounds, 4).tolist()} m in base_link")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
