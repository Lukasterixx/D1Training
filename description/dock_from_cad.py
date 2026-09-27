#!/usr/bin/env python3
"""The Go2 EDU's Jetson expansion dock and its rails, as meshes, from Unitree's CAD.

    python3 description/dock_from_cad.py <path to GO2简化-带拓展坞-0911.stp>

The STEP file is the one `mount_from_cad.py` reads; its docstring says where it comes from. This needs `cascadio`
(Open CASCADE's STEP reader and tessellator, as a wheel) and `trimesh`, which the Isaac Lab environment does not have:

    python3 -m venv /tmp/cad && /tmp/cad/bin/pip install cascadio trimesh
    /tmp/cad/bin/python description/dock_from_cad.py <stp>

It writes two binary STLs, in the Go2's `base_link` frame, metres, so whatever draws them needs no placement:

  meshes/go2/expansion_dock.stl        the dock's body, from the trunk's top (z = 57 mm) up to under the rails
  meshes/go2/expansion_dock_rails.stl  the two T-slot rails and the screws that fasten them; their tops at 85.5 mm

The CAD is registered to `base_link` exactly as the mount was (`mount_from_cad.register`, the hip motors' axes). It
is tessellated to 0.5 mm and 0.5 rad: a drawing's accuracy, not a measurement's. The parts are picked by where they are
-- wholly above the trunk, inside the dock's footprint -- and the rails are told from the body by reaching highest.
The script fails unless it finds exactly those two, with the rails' tops at the mount's height
(`mount.ARM_MOUNT_POS_M`, measured from the same file). Left out: a small red part (6 x 60 mm) just behind the trunk at
x = -130 mm, whose belonging to the dock the CAD does not say.
"""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
from mount import ARM_MOUNT_POS_M  # noqa: E402
from mount_from_cad import base_from_cad, read_step, register  # noqa: E402

OUT_DIR = HERE / "meshes" / "go2"
BODY_STL, RAILS_STL = "expansion_dock.stl", "expansion_dock_rails.stl"
TOL_LINEAR_MM, TOL_ANGULAR_RAD = 0.5, 0.5
# Where the dock can be, in base_link, m: above the trunk's top, within the rails' and the dock's footprint.
DOCK_REGION_M = ((-0.125, -0.06, 0.050), (0.075, 0.06, 0.090))


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 1
    import cascadio
    import trimesh

    step = Path(sys.argv[1])
    to_base = base_from_cad(register(*read_step(step)))
    with tempfile.TemporaryDirectory() as tmp:
        glb = Path(tmp) / "go2.glb"
        cascadio.step_to_glb(str(step), str(glb), tol_linear=TOL_LINEAR_MM, tol_angular=TOL_ANGULAR_RAD)
        scene = trimesh.load(glb, force="scene")
    lo, hi = (np.array(v) for v in DOCK_REGION_M)
    parts = []
    for node in scene.graph.nodes_geometry:
        transform, name = scene.graph[node]
        mesh = scene.geometry[name].copy()
        mesh.apply_transform(to_base @ transform)
        if np.all(mesh.bounds[0] >= lo) and np.all(mesh.bounds[1] <= hi):
            parts.append(mesh)
    if len(parts) != 2:
        raise SystemExit(f"expected the dock's body and its rails, found {len(parts)} parts in {DOCK_REGION_M}")
    body, rails = sorted(parts, key=lambda m: m.bounds[1][2])
    if abs(rails.bounds[1][2] - ARM_MOUNT_POS_M[2]) > 0.0005:
        raise SystemExit(f"the rails' tops are at {rails.bounds[1][2]:.4f} m, the mount at {ARM_MOUNT_POS_M[2]} m")
    for mesh, name in ((body, BODY_STL), (rails, RAILS_STL)):
        mesh.merge_vertices()
        mesh.export(OUT_DIR / name)
        lo_mm, hi_mm = (np.round(1000 * b, 1).tolist() for b in mesh.bounds)
        print(f"{name}: {len(mesh.faces)} triangles, x {lo_mm[0]} to {hi_mm[0]}, y {lo_mm[1]} to {hi_mm[1]}, "
              f"z {lo_mm[2]} to {hi_mm[2]} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
