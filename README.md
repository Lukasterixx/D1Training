# D1Training

A flat testing area for a Unitree Go2 carrying a **hard-attached** D1 arm, in Isaac Sim.

Thesis B work now starts in the [testing and training plan](docs/thesis_b_plan.md),
[codebase investigation](docs/codebase_investigation.md), and
[position-only task guide](docs/position_only_environment.md). The new
`run_position_only.py` entry point is a separate whole-body reaching prototype;
its physics and training still need GPU validation. The existing playback
instructions and preliminary results below describe the original testbed.

Results are kept week by week in [`results/`](results/README.md): notes,
findings, decision gates, recorded runs and screenshots. Browse them with
`./dashboard.py`, which serves <http://localhost:8765>, and copy a finished run
in as evidence with `./dashboard.py record logs/position_only/<run>`.

The question this repo exists to answer: *does the Go2's walking policy still work
with an arm bolted to its back?* It is a payload the policy was never trained on,
so it should show up as a disturbance.

## How this differs from Rescue

Rescue mounts the arm by teleporting its root onto the dog's back every physics
step (`omniverse_sim.py`, the "pin the arm" block). That keeps the arm in place
visually, but tells the walking policy nothing: the teleport rewrites the arm's
root pose and zeroes its velocity every tick, so no reaction force ever reaches
the quadruped. Dynamically the arm is a ghost, and the gait cannot notice it.

Here the arm is **welded**. `weld.py` composes a USD that references Isaac Lab's
stock `go2.usd` and the URDF-imported D1, joins them with a
`UsdPhysics.FixedJoint`, and strips the D1's `ArticulationRootAPI` so PhysX folds
the arm's links into the Go2's articulation rather than parsing a second one.
The result is a single 20-joint articulation whose mass matrix includes the arm.

That is the whole point, and it has one consequence worth understanding: the
robot no longer has exactly the 12 joints the policy was trained on. Every
joint-space observation and action term in `flat_env_cfg.py` is therefore scoped
to `LEG_JOINTS`. This is safe because `SceneEntityCfg` resolves joint names in
ascending articulation order, and subsetting cannot reorder the legs relative to
each other — the 12 values reach the policy in exactly the order it expects,
even though PhysX interleaves `Joint1` between the thigh and calf joints:

```
FL_hip FR_hip RL_hip RR_hip FL_thigh FR_thigh RL_thigh RR_thigh
Joint1 FL_calf FR_calf RL_calf RR_calf Joint2 Joint3 Joint4 Joint5 Joint6 Joint7_1 Joint7_2
        ^^^^^^ arm joints interleaved, legs keep their relative order
```

The other simplification: no CycloneDDS, and no arm wire protocol. Rescue drives
the arm over the real D1's protocol so the same code path runs against hardware;
`d1_direct.py` replaces that with a direct PhysX write. The `KinematicsBackend`
seam in `d1_ik_controller.py` is kept, so the solver stays portable if this ever
needs to talk to a real arm again.

The ROS 2 side is back, though, in the smaller shape described under
[Watching it in RViz](#watching-it-in-rviz).

## Running

```bash
./run_sim.sh                      # teleop, arm at its real published 3.152 kg
./run_sim.sh --no_arm             # bare-Go2 baseline
./run_sim.sh --arm_mass 6.0       # heavier base cylinder
./run_sim.sh --headless --selftest 10   # walk 10 s, print gait + arm + gripper stats
./run_sim.sh --no_ros2            # no lidar, no /joint_states (see below)
```

Needs the `env_isaaclab` conda env from Rescue's setup (Isaac Sim 5.1, Isaac Lab
2.3.x, Python 3.11). `go2.usd` is fetched from NVIDIA's cloud assets on first
run; the D1 URDF is re-imported and the weld rebuilt into `generated/` on every
run, so that directory is disposable and gitignored.

## Watching it in RViz

The sim publishes the same front **Unitree 4D L1 lidar** P2Dingo simulates —
`PointCloud2` on `/utlidar/cloud`, frame `utlidar_lidar`, mounted per the Go2
URDF's `radar_joint` — plus `/joint_states`, `/odom`, `/clock` and TF. So the
welded robot can be watched from outside the viewport, arm included:

```bash
./run_sim.sh          # terminal 1
./run_rviz.sh         # terminal 2 -- robot_state_publisher + RViz 2
```

`run_rviz.sh` needs `/opt/ros/humble` (`ros-humble-desktop`); `run_sim.sh` must
*not* see it, which is why they are separate scripts and why neither sources the
other's environment. Both pin Fast DDS on `ROS_DOMAIN_ID` 0 — a middleware
mismatch here shows up as topics that simply never arrive.

| Topic | Contents |
| --- | --- |
| `/utlidar/cloud` | L1 point cloud, ~12 Hz, xyz only, frame `utlidar_lidar` |
| `/joint_states` | all 20 joints: 12 legs, 6 arm, 2 jaws |
| `/odom` + `/tf` | `odom -> base_link`, and `base_link -> utlidar_lidar` |
| `/clock` | Isaac's timeline; everything downstream runs `use_sim_time` |

The robot model itself comes from `description/go2_d1.urdf`, a **generated** merge
of P2Dingo's Go2 description and `d1_arm/d1.urdf`, joined at the same
`ARM_MOUNT_Z` the weld uses, with the Go2's meshes copied in beside the arm's.
See [description/README.md](description/README.md) — including how to regenerate
it, and why the arm draws in orange.

Two things worth knowing:

- **The L1 needs the renderer, and Isaac Lab only runs it on request.** Isaac Lab
  renders during `env.step()` only if `sim.has_gui() or sim.has_rtx_sensors()`,
  and the latter is a carb setting that its *own* `Camera` sensor sets. `LidarRtx`
  is created outside Isaac Lab, so `ros2.py` sets it explicitly. Without that, a
  headless run publishes one cloud and then freezes `/clock` at 0.1 s — which
  looks like a DDS problem and is not one.
- **`--no_arm` still draws the arm**, folded at its zero pose: the URDF describes
  the fitted robot, and joints missing from `/joint_states` just hold at zero.

## Keybinds

Identical to Rescue, minus `T` — that resets Rescue's odom origin onto the robot,
and odom here is just the sim's world frame, with nothing to re-anchor.

| Key | Action |
| --- | --- |
| `W` `A` `S` `D` | walk forward / strafe left / back / strafe right |
| `Q` `E` | yaw left / right |
| arrows | move the arm's Cartesian IK target in X / Y |
| `1` `0` | move the IK target in Z |
| `,` `.` | close / open the gripper (5 mm per press, 0–65 mm) |
| `Z` | return the arm to its zero pose (suspends IK; any arm key re-engages) |
| `P` | toggle motor power (e-stop) |
| `R` | reset the robot |

`P` behaves better here than in Rescue. Rescue notes that releasing the drive
gains does not make the arm sag, because its root is teleported every tick and
so it can never build up any falling motion. The welded arm genuinely goes limp
and sags — and the dog feels it do so.

The gripper needed a stiffer drive than Rescue's to work at all here: Rescue's
200 N/m jaw drive gets pinned solid by the welded arm's motion, because the root
teleport it relies on was also acting as an accidental vibration damper. See the
gain comments in `flat_env_cfg.py`; `--selftest` now checks the jaws track, so it
cannot regress silently.

## The arm's mass, and where it lives

**The shipped `d1_arm/d1.urdf` describes a 0.72 kg arm.** Its inertials are a
SolidWorks export of the bare shells: no motors, no gearing, no wiring. Unitree
publishes **3152 g** for the D1-550. Unitree's own `d1_description` ships the
same 0.719 kg shells *and* effort limits of literally zero, so their URDF cannot
be used as-is — which is why Rescue's copy has hand-filled numbers.

How the URDF compares to [the published spec][spec]:

| Quantity | URDF said | Unitree publishes | |
| --- | --- | --- | --- |
| Joint ranges | ±134.6°, ±90°, ±90°, ±134.6°, ±90°, ±134.6° | J0 ±135°, J1 ±90°, J2 ±90°, J3 ±135°, J4 ±90°, J5 ±135° | matches |
| Reach | — | 550 mm excl. jaw | matches `ARM_MAX_REACH` |
| Claw stroke | 60 mm (2 × 30 mm fingers) | 0–65 mm | close enough |
| Joint effort | 3.33, 3.33, **3.33**, 1.67, 1.67, 1.67 Nm | 3.3, 3.3, **1.7**, 1.7, 1.7, 1.7 Nm | **Joint3 was ~2× too strong — fixed** |
| Total mass | 0.719 kg | 3.152 kg | **too light — see below** |

Mapping uses the protocol's offset (`d1_protocol.py`: "Protocol id 0 is URDF
Joint1"). All six ranges matching exactly is what confirms that offset. Whoever
filled in the zeros assumed "first three joints strong, last three weak"; the D1
is first *two* strong, last *four* weak, so `Joint3` carried double its real
torque. Now corrected in the URDF.

[spec]: https://support.unitree.com/home/en/developer/D1Arm_services

### Where the missing 2.4 kg goes matters more than you would guess

`--arm_mass` (default **3.152**, the published figure) brings the arm up to its
real weight. It scales mass and inertia only — **not** the effort limits, which
Unitree publishes independently. A heavier arm does not imply stronger motors,
and scaling the two together invents a robot that does not exist.

Spreading the missing mass evenly across the links is the obvious move and it is
wrong, visibly: it loads the wrist as heavily as the base, the shoulder then
needs ~6 Nm against its published 3.3 Nm, and the arm sags to its stops instead
of holding the pose the IK asks for (EE error 31.5 cm). That is not a D1, it is
an artefact of the smear.

The real D1's base is a heavy metal cylinder; the servos are small. So
`weld.py`'s mass model is:

- every link keeps its shell inertial,
- plus the bus servo sitting at its joint — 60 g for the 3.3 Nm joints, 45 g for
  the 1.7 Nm ones, sized from comparable parts ([Feetech STS3215][sts], 55 g at
  2.94 Nm; [Dynamixel XL430-W250][xl], 57 g at 1.5 Nm; [Feetech STS3032][s32],
  25 g at 0.44 Nm). Unitree does not publish per-servo masses, so these are
  inferred — but at 345 g of 3152 g they barely matter, which is the point.
- everything still missing (**2.09 kg**) goes on `base_link`.

That leaves **2.165 kg at the base and 0.987 kg of moving arm**, which a 3.3 Nm
shoulder holds comfortably. Because `base_link` is welded to the Go2, its share
is dead payload bolted to the dog's back at the mount — low and centred — rather
than swinging on the end of a lever. The full 3.152 kg still reaches the policy;
it just reaches it in the right place.

Consequence: the minimum expressible arm is shells + servos = 1.064 kg. Asking
for less is an error, not a silent clamp.

[sts]: https://www.robotshop.com/products/feetech-12v-30kgcm-magnetic-encoding-servo-sts3215
[xl]: https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/
[s32]: https://www.feetechrc.com/6v-45kg-magnetic-code-360-degree-serial-bus-steering-gear.html

## Results

10 s of walking at a commanded 1.0 m/s on flat ground:

| Configuration | Total mass | Distance | Speed | Max tilt | Arm EE error | Gripper | Verdict |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `--no_arm` (baseline) | 15.02 kg | 9.56 m | 96% | 5.9° | — | — | stayed up |
| **D1-550 as published** (default) | 18.17 kg | 9.51 m | 95% | 7.3° | 3.7 cm | 59.0 / 60 mm | **stayed up** |
| `--arm_mass 6.0` | 21.02 kg | 9.10 m | 91% | 9.3° | 3.5 cm | 59.4 / 60 mm | stayed up |

**The Go2 walks with a real D1 on its back, and barely notices it** — 95% of
commanded speed against 96% bare, 7.3° of tilt against 5.9°. The policy was
never trained on the payload and carries it anyway. Even at 6 kg it stays up,
because the weight sits at the mount rather than out on a lever.

That last row is worth reading carefully: it is *not* "the arm can be 6 kg". The
mass model puts everything above shells-and-servos on the base cylinder, so
`--arm_mass 6.0` is a heavier *base*, not a heavier reach. A payload in the
gripper would be a different experiment, and a much harsher one.

## Known behaviour

**The arm's IK used to settle ~13 cm short of its target, and it was not the
IK.** Rescue documents this shortfall and attributes it to the solver; both the
first version of this repo and its README repeated that. It is wrong.

The cause is that a P-only drive droops by `torque / stiffness`. At Rescue's
800 Nm/rad, ~1 Nm of gravity leaves several degrees of error on every joint, and
those compound down the chain. Measured, holding the same target:

| Arm drive stiffness | EE error |
| --- | --- |
| 100 | 30.3 cm |
| 400 | 19.6 cm |
| 800 (Rescue's) | 13.1 cm |
| **4000** (used here) | **3.3 cm** |

A real D1 servo closes its own position loop and holds the commanded angle
rather than sagging like a spring, so the gains are now 4000. This buys tracking
and never strength: PhysX still clamps every joint at its published effort limit,
so the arm's 3.3/1.7 Nm remain the physical statement of what it cannot lift.

**Contact sensing does not cover the arm.** `FlatSceneCfg.contact_forces` uses
`{ENV_REGEX_NS}/Robot/.*`, which matches the Go2's links but not the arm's —
those sit one level deeper, at `Robot/D1/...`, because the weld nests them. This
is reporting only: arm collisions still happen in physics, they just are not
readable from the sensor. The locomotion policy does not use them. Widen the
regex if you ever need to detect the gripper touching something.

**The inertia model is approximate.** Each link's inertia is scaled by its own
mass factor, keeping the shell's shape and raising its density. For `base_link`
that is not an approximation at all — a solid metal cylinder really is the same
shape as its shell, only denser. For the rest, each servo is treated as spread
through its link rather than as a point mass at the joint; at tens of grams the
error is small. Real per-link inertials would still be better.

## Files

| File | Role |
| --- | --- |
| `main.py` | argument parsing, launches Isaac Sim, hands off to `sim.py` |
| `sim.py` | teleop keybinds, the sim loop, and the `--selftest` harness |
| `weld.py` | URDF import + the go2/D1 weld; mass and effort rescaling |
| `ros2.py` | the L1 lidar, and the state publishers RViz needs |
| `description/` | the URDF + meshes + RViz layout for `run_rviz.sh` |
| `flat_env_cfg.py` | flat-plane scene, and the leg-scoped obs/action terms |
| `d1_ik_controller.py` | DLS Cartesian IK at the D1's real 10 Hz, ported from Rescue |
| `d1_direct.py` | buffers arm commands off the keyboard thread, writes to PhysX |
| `agent_cfg.py` | RSL-RL config for the walking checkpoint |
| `logs/` | the Go2 walking checkpoint, converted to the rsl_rl 2.x format |
| `position_only/`, `run_position_only.py` | Thesis B 18-action stance-and-reach task and its launcher |
| `motor_model.py`, `unitree_actuators.py` | Unitree's measured Go2 motor envelope (from unitree_rl_lab, Apache-2.0; see `third_party/`) |
| `results/`, `dashboard.py`, `evidence/` | the weekly experimental record and its localhost dashboard |

## Despite the name

There is no training here yet — this is policy playback, like Rescue. The pieces
a training run would need are present but deliberately de-tuned for a live
testbed, and all three must be reverted before `flat_env_cfg.py` could train
anything:

- `terminations.time_out` and `terminations.base_contact` are `None`, so the
  robot is never reset out from under the operator.
- `CommandsCfg.base_velocity` is frozen to zero ranges; the real velocity
  command is injected from the keyboard via `constant_commands`.
- The reward terms are carried over from Rescue but inert during playback.
