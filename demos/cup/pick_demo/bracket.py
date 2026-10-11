"""The wrist camera's bracket, from its CAD: where the RealSense sits on the D1's Link6.

Lukas designed the bracket in Fusion 360 on Unitree's D1-550 CAD and exported it on 2026-10-09 as three STL files,
in metres, in the STEP's own frame (the arm lying along -y, as `d1_arm/compare_cad.py` describes it):

* `mount_only.stl` -- the bracket alone, three bodies;
* `d435i_with_mount.stl` -- the bracket with a D435i model in it;
* `d435i_with_mount_and_arm.stl` -- both on Unitree's arm.

`register` turns them into the two things the simulator and the hardware use, and `main` writes them:

1. **The mount** (`camera.DEFAULT_MOUNT_PATH`): the colour optical frame in Link6, saved as the default so that
   every launch loads it with nothing to pass (`camera.resolve_mount`).
2. **The bracket in Link6** (`BRACKET_MESH_PATH`), drawn by the scenes as a visual-only child of Link6
   (`camera_asset.build_bracket_usd`, `scene.bracket_cfg`). It is fixed to Link6, not to the mount: the bracket
   is bolted to the gripper, so a later hand-eye calibration that moves the camera does not move the bracket.

How each frame is found, and from what:

* **Link6 in the CAD.** Its z axis is the wrist roll, Joint6: the common axis of the wrist flange's rings. Its y
  axis is the jaw axis, which in the CAD lies along x (the two fingers sit at the same height either side of the
  roll line), so the roll is zero by construction. Its origin along the roll line is where `d1.urdf`'s own Link6
  mesh fits the CAD's gripper best, a one-parameter trimmed nearest-surface fit. That mesh is the older
  `d1_description` hand (F-137), not the CAD's, so the fit is about 1.4 mm median and its minimum is shallow:
  this is the least certain number here. It is the right reference for the simulator, which draws that mesh,
  and for the bracket, which bolts to that body.
* **The camera.** The D435i case's faces are planes. The front's normal is pitched 30.00 deg below the approach
  axis, with no yaw. The bracket holds the case by its back: a plate lies on the case's back face, with two M3
  clearance holes 45.000 mm apart, which are the D435's two M3 mounting holes. Intel's own case
  (`camera_body.MESH_PATH`, the mesh the simulator draws) is placed with its back on that plate and its holes on
  the bracket's, and `camera_body.MESH_TO_OPTICAL` carries it to the colour optical frame.
  The D435i model in the export is not Intel's: it measures 90.74 x 24.74 x 24.10 mm against Intel's
  89.91 x 25.00 x 25.05, and its lens marks are a millimetre or two out. It decides the orientation and which end
  the colour lens is at (its projector and left imager at -11 and +18 mm along the case, Intel's at -11.5 and
  +17.5), and nothing else.

**This is the design, not a measurement.** It says where the camera sits if the bracket is printed and assembled
as drawn. A hand-eye calibration on the real arm is what would measure it. The saved file says so: `measured` is
false and its `method` starts "cad".

The registration needs `trimesh` and `scipy` (the Isaac environment has both); loading the results does not.

    python -m demos.cup.pick_demo.bracket ~/Downloads/mount_only.stl ~/Downloads/d435i_with_mount.stl \\
        ~/Downloads/d435i_with_mount_and_arm.stl            # report only
    ... --write                                             # and write the mount and the bracket mesh
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from . import camera, camera_body

ROOT = Path(__file__).resolve().parents[3]
ASSET_DIR = Path(__file__).resolve().parent / "assets" / "bracket"
BRACKET_MESH_PATH = ASSET_DIR / "wrist_bracket_link6.stl"
REGISTRATION_PATH = ASSET_DIR / "registration.json"
LINK6_MESH_PATH = ROOT / "d1_arm" / "meshes" / "Link6.STL"

# The CAD's wrist roll line (x, z) in metres, near enough to pick out the flange's rings; `compare_cad.py` has
# the same seed in millimetres.
ROLL_LINE_SEED_XZ = (0.0004, 0.1665)
# The D435's M3 mounting holes on its back, along the case either side of its centre (Intel's mesh: 45.000 mm).
CASE_HOLE_X_M = 0.0225
CAMERA_PITCH_DEG = 30.0          # the bracket's design angle; the case's front normal measures 30.003 deg
MESH_METHOD = ("cad: Lukas's bracket (Fusion 360, 2026-10-09) on Unitree's D1-550 CAD. Link6 from the wrist "
               "flange's roll axis, the jaw axis and a fit of d1.urdf's Link6 mesh; the camera as Intel's D435 case "
               "with its back on the bracket's plate and its M3 holes on the plate's (demos/cup/pick_demo/bracket.py)")


# --------------------------------------------------------------------------- the CAD export

def _sha256(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _parts(path):
    import trimesh

    return trimesh.load(str(path), force="mesh").split(only_watertight=False)


def _key(part):
    return len(part.faces), tuple(np.round(part.bounds.mean(axis=0), 4))


def split_export(mount_stl, camera_stl, arm_stl):
    """(bracket parts, camera parts, arm parts): the three files share a frame, so a body present in a smaller
    export is the same body, with the same triangles at the same place, in a larger one."""
    bracket = _parts(mount_stl)
    known = {_key(p) for p in bracket}
    with_camera = _parts(camera_stl)
    cam = [p for p in with_camera if _key(p) not in known]
    known |= {_key(p) for p in with_camera}
    arm = [p for p in _parts(arm_stl) if _key(p) not in known]
    if len(cam) + len(bracket) != len(with_camera):
        raise ValueError("the camera export does not contain the bracket export's bodies unchanged")
    return bracket, sorted(cam, key=lambda p: -len(p.faces)), arm


def circle_fit(xy):
    """Least-squares circle through 2D points: (centre, radius)."""
    xy = np.asarray(xy, dtype=float)
    a = np.c_[2.0 * xy, np.ones(len(xy))]
    sol, *_ = np.linalg.lstsq(a, (xy ** 2).sum(axis=1), rcond=None)
    return sol[:2], float(np.sqrt(sol[2] + sol[:2] @ sol[:2]))


def _hole(points_xy, guess, radius_window=(0.0010, 0.0022), band=0.00015):
    """The circle of vertices around `guess`: a hole's edge, refined on its own ring."""
    d = np.linalg.norm(points_xy - guess, axis=1)
    centre, radius = circle_fit(points_xy[(d > radius_window[0]) & (d < radius_window[1])])
    for _ in range(3):
        on = np.abs(np.linalg.norm(points_xy - centre, axis=1) - radius) < band
        centre, radius = circle_fit(points_xy[on])
    return centre, radius


def _transform(rot, pos):
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = rot, pos
    return out


def roll_line(arm) -> tuple:
    """The wrist roll axis (x, z) in the CAD: the mean centre of the wrist flange's rings, and their spread."""
    seed = np.asarray(ROLL_LINE_SEED_XZ)
    centres = []
    for part in arm:
        xz = part.vertices[:, [0, 2]]
        lo, hi = part.bounds
        if np.linalg.norm((lo[[0, 2]] + hi[[0, 2]]) / 2.0 - seed) > 0.0006 or not (-0.02 < (lo[1] + hi[1]) / 2 < 0.012):
            continue
        radius = np.linalg.norm(xz - seed, axis=1)
        for r in np.unique(np.round(radius, 4)):
            ring = np.abs(radius - r) < 0.00006
            if ring.sum() >= 24 and r > 0.004:
                centres.append(circle_fit(xz[ring])[0])
    if len(centres) < 3:
        raise ValueError("no wrist flange rings found about the roll line")
    centres = np.array(centres)
    return centres.mean(axis=0), float(np.ptp(centres, axis=0).max()), len(centres)


def _is_finger(part):
    centre, extent = part.bounds.mean(axis=0), part.extents
    return centre[1] > 0.078 and abs(centre[0]) < 0.03 and extent[0] < 0.03


def link6_in_cad(arm, samples: int = 40000, seed: int = 0) -> dict:
    """Link6's frame in the CAD: on the roll line, jaw axis along CAD x, origin from the Link6 mesh fit."""
    import trimesh
    from scipy.optimize import minimize_scalar
    from scipy.spatial import cKDTree

    axis_xz, spread, rings = roll_line(arm)
    hand = trimesh.util.concatenate([p for p in arm if p.bounds[1][1] > 0.005 and p.bounds[0][1] < 0.09
                                     and p.bounds[0][2] > 0.11 and not _is_finger(p)])
    tree = cKDTree(trimesh.sample.sample_surface(hand, 400000, seed=seed)[0])
    link6 = trimesh.load(str(LINK6_MESH_PATH), force="mesh")
    points = trimesh.sample.sample_surface(link6, samples, seed=seed)[0]
    # Link6 z along the CAD's +y (out along the gripper), its x along -z (with the gripper level, -x is up) and
    # its y along -x: the fingers travel along Link6 y, and along CAD x.
    rot = np.column_stack([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])

    def frame(y0):
        return _transform(rot, [axis_xz[0], y0, axis_xz[1]])

    def cost(y0, cap=0.002):
        t = frame(y0)
        d, _ = tree.query(points @ t[:3, :3].T + t[:3, 3])
        return float(np.mean(np.minimum(d, cap) ** 2))

    fit = minimize_scalar(cost, bounds=(0.0, 0.025), method="bounded", options={"xatol": 1e-6})
    best = frame(fit.x)
    d, _ = tree.query(points @ best[:3, :3].T + best[:3, 3])
    return {"T_cad_link6": best, "roll_line_xz_m": axis_xz.tolist(), "flange_rings": rings,
            "ring_centre_spread_m": spread, "origin_along_roll_m": float(fit.x),
            "fit_median_m": float(np.median(d)), "fit_within_1mm": float(np.mean(d < 0.001)),
            "cost_ratio_at_pm1mm": [round(cost(fit.x - 0.001) / fit.fun, 3), round(cost(fit.x + 0.001) / fit.fun, 3)]}


def camera_in_cad(bracket, cam) -> dict:
    """Intel's D435 case bolted to the bracket: its colour optical frame in the CAD."""
    import trimesh

    case = cam[0]
    normals, areas = case.face_normals, case.area_faces
    pitch = np.radians(CAMERA_PITCH_DEG)
    forward = np.array([0.0, np.cos(pitch), -np.sin(pitch)])
    on_front = normals @ forward > 0.995
    front = (normals[on_front] * areas[on_front, None]).sum(axis=0)
    front /= np.linalg.norm(front)
    measured_pitch = float(np.degrees(np.arctan2(-front[2], front[1])))
    measured_yaw = float(np.degrees(np.arctan2(front[0], front[1])))
    # Intel's mesh axes in the CAD: x along the case (towards the colour lens, which is at CAD -x), y up, z out
    # of the front. The upright choice is the export's lens layout's; see the module docstring.
    rot = np.column_stack([[-1.0, 0.0, 0.0], [0.0, np.sin(pitch), np.cos(pitch)], forward])
    plate = max(bracket, key=lambda p: len(p.faces))
    plate_local = plate.vertices @ rot
    case_local = case.vertices @ rot
    back_on_plate = float(plate_local[:, 2].max())
    across = case_local[:, 1].mean()
    holes = [_hole(plate_local[:, :2], np.array([s * CASE_HOLE_X_M, across])) for s in (-1.0, 1.0)]
    intel = trimesh.load(str(camera_body.MESH_PATH), force="mesh")
    back = intel.vertices[:, 2].min()
    near_back = intel.vertices[intel.vertices[:, 2] < back + 0.002]
    intel_holes = [_hole(near_back[:, :2], np.array([s * CASE_HOLE_X_M, 0.0])) for s in (-1.0, 1.0)]
    shift = np.r_[(holes[0][0] + holes[1][0]) / 2.0 - (intel_holes[0][0] + intel_holes[1][0]) / 2.0,
                  back_on_plate - back]
    t_cad_mesh = _transform(rot, rot @ shift)
    return {"T_cad_optical": t_cad_mesh @ np.linalg.inv(camera_body.MESH_TO_OPTICAL), "T_cad_case_mesh": t_cad_mesh,
            "front_pitch_deg": measured_pitch, "front_yaw_deg": measured_yaw,
            "case_back_plane_m": float(case_local[:, 2].min()), "plate_face_m": back_on_plate,
            "bracket_hole_spacing_m": float(np.linalg.norm(holes[0][0] - holes[1][0])),
            "bracket_hole_radius_m": [h[1] for h in holes],
            "intel_hole_spacing_m": float(np.linalg.norm(intel_holes[0][0] - intel_holes[1][0])),
            "export_case_size_m": (case_local.max(axis=0) - case_local.min(axis=0)).tolist(),
            "intel_case_size_m": intel.extents.tolist()}


def register(mount_stl, camera_stl, arm_stl) -> dict:
    """Everything the CAD says about the mount; nothing is written."""
    bracket, cam, arm = split_export(mount_stl, camera_stl, arm_stl)
    link6 = link6_in_cad(arm)
    cam_fit = camera_in_cad(bracket, cam)
    to_link6 = np.linalg.inv(link6["T_cad_link6"])
    import trimesh

    bracket_mesh = trimesh.util.concatenate(bracket)
    bracket_mesh.apply_transform(to_link6)
    return {"mount_pose_link6": to_link6 @ cam_fit["T_cad_optical"], "bracket_link6": bracket_mesh,
            "link6": link6, "camera": cam_fit,
            "inputs": {Path(p).name: {"sha256": _sha256(p), "bytes": Path(p).stat().st_size}
                       for p in (mount_stl, camera_stl, arm_stl)},
            "bodies": {"bracket": len(bracket), "camera": len(cam), "arm": len(arm)}}


# --------------------------------------------------------------------------- the committed results

def bracket_mesh_link6() -> tuple:
    """The bracket in the Link6 frame, as (points (N, 3), triangles (T, 3)). Needs `trimesh`."""
    import trimesh

    mesh = trimesh.load(str(BRACKET_MESH_PATH), force="mesh")
    return np.asarray(mesh.vertices, dtype=float), np.asarray(mesh.faces, dtype=np.int64)


def _rounded(value, places=9):
    return np.round(np.asarray(value, dtype=float), places).tolist()


def write(result, previous=None) -> dict:
    """Write the bracket mesh, the registration record and the default mount file."""
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    result["bracket_link6"].export(str(BRACKET_MESH_PATH))
    mount = camera.MeasuredMount.from_pose(result["mount_pose_link6"], "bracket CAD")
    saved = camera.save_mount(camera.DEFAULT_MOUNT_PATH, mount, source="the bracket's CAD (bracket.py)",
                              measured=False, method=MESH_METHOD, camera="D435i")
    link6, cam_fit = result["link6"], result["camera"]
    record = {
        "what": "the wrist camera's bracket registered to Link6 and Intel's D435 case; see bracket.py",
        "inputs": result["inputs"], "bodies": result["bodies"],
        "link6_in_cad": {k: (_rounded(v) if isinstance(v, (np.ndarray, list, tuple)) else v)
                         for k, v in link6.items()},
        "camera_in_cad": {k: (_rounded(v) if isinstance(v, (np.ndarray, list, tuple)) else v)
                          for k, v in cam_fit.items()},
        "mount_pose_link6": _rounded(result["mount_pose_link6"]),
        "mount_xyz_m": saved["xyz_m"], "mount_rpy_deg": saved["rpy_deg"],
        "bracket_mesh": {"path": str(BRACKET_MESH_PATH.relative_to(ROOT)), "sha256": _sha256(BRACKET_MESH_PATH),
                         "frame": "Link6 (d1.urdf), metres"},
        "replaced": previous,
    }
    REGISTRATION_PATH.write_text(json.dumps(record, indent=2) + "\n")
    return record


def report(result, previous_mount=None) -> list:
    link6, cam_fit = result["link6"], result["camera"]
    pose = result["mount_pose_link6"]
    xyz, rpy = camera.mount_as_xyz_rpy(camera.MeasuredMount.from_pose(pose, ""))
    lines = [
        f"bodies: bracket {result['bodies']['bracket']}, camera {result['bodies']['camera']}, "
        f"arm {result['bodies']['arm']}",
        f"roll line (x, z) {np.round(np.array(link6['roll_line_xz_m']) * 1000, 3)} mm from "
        f"{link6['flange_rings']} flange rings (centres within {link6['ring_centre_spread_m'] * 1000:.3f} mm)",
        f"Link6 origin along the roll line: y = {link6['origin_along_roll_m'] * 1000:.2f} mm; mesh fit median "
        f"{link6['fit_median_m'] * 1000:.2f} mm, {link6['fit_within_1mm'] * 100:.0f}% within 1 mm; cost x"
        f"{link6['cost_ratio_at_pm1mm']} at -/+1 mm",
        f"camera front pitch {cam_fit['front_pitch_deg']:.3f} deg, yaw {cam_fit['front_yaw_deg']:.3f} deg",
        f"bracket holes {cam_fit['bracket_hole_spacing_m'] * 1000:.3f} mm apart (r "
        f"{np.round(np.array(cam_fit['bracket_hole_radius_m']) * 1000, 3)} mm); Intel's "
        f"{cam_fit['intel_hole_spacing_m'] * 1000:.3f} mm",
        f"plate face {cam_fit['plate_face_m'] * 1000:.3f} mm, export case back {cam_fit['case_back_plane_m'] * 1000:.3f} mm",
        f"mount: xyz {np.round(np.array(xyz) * 1000, 2)} mm, rpy {np.round(rpy, 3)} deg",
        f"optical axis in Link6 {np.round(pose[:3, 2], 4)}: {np.degrees(np.arccos(pose[2, 2])):.2f} deg off the approach",
    ]
    if previous_mount is not None:
        old = np.asarray(previous_mount.pose)
        turn = old[:3, :3].T @ pose[:3, :3]
        lines.append(f"against the previous default: moved {np.linalg.norm(pose[:3, 3] - old[:3, 3]) * 1000:.1f} mm, "
                     f"turned {np.degrees(np.arccos(np.clip((np.trace(turn) - 1) / 2, -1, 1))):.1f} deg")
    return lines


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mount_stl")
    parser.add_argument("camera_stl")
    parser.add_argument("arm_stl")
    parser.add_argument("--write", action="store_true",
                        help="Write the bracket mesh, the registration record and the default mount file.")
    args = parser.parse_args(argv)
    previous, previous_record = None, None
    if camera.DEFAULT_MOUNT_PATH.is_file():
        previous = camera.load_mount(camera.DEFAULT_MOUNT_PATH)
        previous_record = json.loads(camera.DEFAULT_MOUNT_PATH.read_text())
    result = register(args.mount_stl, args.camera_stl, args.arm_stl)
    for line in report(result, previous):
        print(f"[bracket] {line}")
    if args.write:
        write(result, previous_record)
        print(f"[bracket] wrote {BRACKET_MESH_PATH.relative_to(ROOT)}, {REGISTRATION_PATH.relative_to(ROOT)} and "
              f"{camera.DEFAULT_MOUNT_PATH.relative_to(ROOT)}")
    return 0


__all__ = ["register", "write", "report", "bracket_mesh_link6", "BRACKET_MESH_PATH", "REGISTRATION_PATH",
           "CAMERA_PITCH_DEG"]

if __name__ == "__main__":
    raise SystemExit(main())
