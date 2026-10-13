"""Where the D1's base sits on the Go2: the one number every model of the robot is built from.

`ARM_MOUNT_POS_M` is the D1 base frame's origin -- the middle of its bottom face -- in the Go2's `base_link`, metres.
The arm is square to the dog (its x forward). `weld.py` welds the sim's arm there, `position_only.workspace.MOUNT_B`
solves IK from it, `description/build_go2_d1_urdf.py` joins the RViz model there. The dock and rails it stands on are
drawn from the same CAD (`dock.py`).

Measured, not guessed. On the Go2 EDU the D1 stands on the Jetson expansion dock's two T-slot rails, whose tops are
85.5 mm above `base_link` in Unitree's own CAD (F-117, 2026-09-26). The rails are fastened to the dog by four M3 screws
at x = -84.5 and +46.6 mm, y = +/-29 mm, 131 mm apart fore-aft. Lukas measured the real arm's holes 130 mm apart along
the rail, so they sit over those screws and the base is centred at their midpoint, x = -19 mm (F-125, 2026-09-30).
Unitree's base mesh has them 108 mm apart, which is why F-117 had to choose one pair of screws; the real base is a
larger part (F-126, `d1_arm/measured_base.py`). `description/mount_from_cad.py` reproduces the rails from the CAD.

No imports, so plain-Python tools can read it.
"""

#: The D1 base's origin in the Go2's `base_link`, m: on the rails' tops, midway between the rails' screws (F-125).
ARM_MOUNT_POS_M = (-0.019, 0.0, 0.0855)

#: F-117's mount, 2026-09-26 to 2026-10-05 on this branch: Unitree's 108 mm hole pattern with its front pair 1 cm ahead
#: of the rails' front screws. Kept to name what results in that window were built on.
DOCK_CAD_ARM_MOUNT_POS_M = (0.0025, 0.0, 0.0855)

#: The weld every run before 2026-09-26 used (Rescue's), kept to name what older results were built on.
LEGACY_ARM_MOUNT_POS_M = (0.0, 0.0, 0.08)

#: Joint1 above the D1 base's bottom face, m: the base Lukas measured (F-126, `d1_arm/d1.urdf`), and Unitree's model
#: of it, which every scene before 2026-09-30 was set against. `tests/test_workspace.py` checks the first against the URDF.
JOINT1_HEIGHT_M = 0.0760
LEGACY_JOINT1_HEIGHT_M = 0.0533

#: How far the arm's base moved in the Go2's `base_link` from the arm the scripted scenes were set against (the legacy
#: mount on Unitree's base) to the measured one: (-0.019, 0, +0.0282) m. A scene re-placed by it is where it was
#: relative to the arm (Week 4, 2026-10-06); one standing on the real floor takes only the horizontal part. The arm
#: above its base then took the CAD's link lengths (2026-10-09, F-139): the shoulder 1.5 mm lower, the forearm 6.8 mm
#: longer, Link6 (+5.1, +0.9, -1.5) mm off at the zero pose. The scenes were not moved again for that.
ARM_SHIFT_FROM_LEGACY_M = (ARM_MOUNT_POS_M[0] - LEGACY_ARM_MOUNT_POS_M[0],
                           ARM_MOUNT_POS_M[1] - LEGACY_ARM_MOUNT_POS_M[1],
                           ARM_MOUNT_POS_M[2] - LEGACY_ARM_MOUNT_POS_M[2] + JOINT1_HEIGHT_M - LEGACY_JOINT1_HEIGHT_M)
