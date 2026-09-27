"""Where the D1's base sits on the Go2: the one number every model of the robot is built from.

`ARM_MOUNT_POS_M` is the D1 base frame's origin -- the middle of its bottom face -- in the Go2's `base_link`, metres.
The arm is square to the dog (its x forward). `weld.py` welds the sim's arm there, `position_only.workspace.MOUNT_B`
solves IK from it, `description/build_go2_d1_urdf.py` joins the RViz model there. The dock and rails it stands on are
drawn from the same CAD (`dock.py`).

Measured, not guessed (F-117, 2026-09-26): on the Go2 EDU the D1 stands on the Jetson expansion dock's two T-slot rails,
whose tops are 85.5 mm above `base_link` in Unitree's own CAD, its four M4 holes (108 x 58 mm) over the rails' centrelines
(y = +/-29 mm). The rails are fastened to the dog by four M3 screws at x = -84.5 and +46.5 mm, 131 mm apart, so the D1's
108 mm cannot put all four of its screws the same distance from them; Lukas placed it with its **front** screws 1 cm ahead
of the rails' front screws (2026-09-26), which puts its origin 2.5 mm ahead of `base_link`. In the CAD the rails end at
x = +53.0 mm, 3.5 mm short of those front screws -- a difference between the CAD and the robot to check on the robot.
`description/mount_from_cad.py` reproduces the numbers from the CAD. Until 2026-09-26 the weld was (0, 0, 0.08), Rescue's
guess; every policy trained before then was trained on that.

No imports, so plain-Python tools can read it.
"""

#: The D1 base's origin in the Go2's `base_link`, m: 1 cm ahead of the rails' front screws, on the rails' tops.
ARM_MOUNT_POS_M = (0.0025, 0.0, 0.0855)

#: The weld every run before 2026-09-26 used (Rescue's), kept to name what older results were built on.
LEGACY_ARM_MOUNT_POS_M = (0.0, 0.0, 0.08)
