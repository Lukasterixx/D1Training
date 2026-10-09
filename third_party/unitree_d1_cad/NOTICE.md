# Unitree D1 arm (CAD and URDF)

Downloaded 2026-10-08 from the links on Unitree's D1 developer page,
<https://support.unitree.com/home/en/developer/D1Arm_services>. That page is behind a firewall that refuses scripted
requests, so its content was read from the documentation API the page itself calls,
`https://robot-api.unitree.com/doc?space=developer&locale=en` (the D1 page: `https://doc-cdn.unitree.com/6/109/en/6_109_en`,
updated 2026-07-20). The page names the arm the D1-550: 3152 g, 550 mm without the jaws, 670 mm with them.

| Archive on the page | URL (`https://oss-global-cdn.unitree.com/static/…`) | sha256 of the archive | What is in it |
| --- | --- | --- | --- |
| URDF, gripper version | `90b2525be5d84531ab9814f48b2f86a7.zip` | `76cbe772…346a7164` | `d1_description`: the URDF `d1_arm/d1.urdf` descends from |
| URDF, grip-free version | `02c95ece8e354143b874a3c963241467.zip` | `07d285ae…eea38c83` | the same, the jaws fixed |
| STEP simplified model | `5bade30b2a454376978ab2853589e5f7.zip` | `8e7a5695…d73d7b68` | `D1-舵机臂.STEP`: the same arm, parts and pose as the D1-550 STEP |
| D1-550 STEP | `fc33175f0fbd4fb0842e588080c2b3d9.zip` | `d0f0f8c2…80d41785` | `D1舵机臂-简化B(1).stp` (sha256 `1e3d6381…ac61750e`): 74 bodies in one frame |
| D1-550 URDF | `9b20252a26374d50aa369532657d0143.zip` | `71abcc4c…8e42faa5` | `d1_550_description`: different link names, frames and lengths from `d1_description` |
| D1-550 gripper open-source files | `6f0d89dcedde40f1a59272b71c5363a2.zip` | `a59eba61…161ff549` | four STEP plastic gripper parts |

None of these files is in this repository. `d1_arm/compare_cad.py` reads the D1-550 STEP and URDF from wherever they
were unpacked and commits nothing derived from them but numbers.

**Licence.** Both URDF packages' `package.xml` say `BSD` with `TODO` for author and maintainer (the SolidWorks
exporter's template); the STEP archives state none. `d1_arm/d1.urdf` and its meshes were already taken from
`d1_description` before this note (Rescue's copy); this records where it came from.
