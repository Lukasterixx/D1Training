# D1Training

A flat testing area for a Unitree Go2 carrying a **hard-attached** D1 arm, in Isaac Sim.

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

The other simplification: no ROS 2, no RTX lidar, and no CycloneDDS. Rescue
drives the arm over the real D1's wire protocol so the same code path runs
against hardware; `d1_direct.py` replaces that with a direct PhysX write. The
`KinematicsBackend` seam in `d1_ik_controller.py` is kept, so the solver stays
portable if this ever needs to talk to a real arm again.

## Running

```bash
./run_sim.sh                      # teleop, arm at the URDF's own mass
./run_sim.sh --arm_mass 2.4       # realistic D1 mass, motors scaled to match
./run_sim.sh --no_arm             # bare-Go2 baseline
./run_sim.sh --headless --selftest 10 --arm_mass 2.4   # walk 10 s, print gait stats
```

Needs the `env_isaaclab` conda env from Rescue's setup (Isaac Sim 5.1, Isaac Lab
2.3.x, Python 3.11). `go2.usd` is fetched from NVIDIA's cloud assets on first
run; the D1 URDF is re-imported and the weld rebuilt into `generated/` on every
run, so that directory is disposable and gitignored.

## Keybinds

Identical to Rescue, minus `T` (there is no lidar here).

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

The gripper needed a stiffer drive than Rescue's to work at all here; see the
gain comments in `flat_env_cfg.py`. Rescue's 200 N/m jaw drive is pinned solid
by the welded arm's motion, because the root teleport it relies on was also
acting as an accidental vibration damper.

## The arm's mass is a trap

**The shipped `d1_arm/d1.urdf` describes a 0.72 kg arm.** Its inertials are a
SolidWorks export of the bare shells: no motors, no gearing, no wiring. At
0.72 kg the arm is ~5% of the Go2's mass and the gait does not notice it at all.

How the URDF compares to Unitree's published D1 spec — worth knowing which parts
of this file to trust:

| Quantity | URDF says | Unitree publishes | Verdict |
| --- | --- | --- | --- |
| Joint ranges | ±134.6°, ±90°, ±90°, ±134.6°, ±90°, ±134.6° | J1 ±135°, J2 ±90°, J3 ±90°, J4 ±135°, J5 ±90°, J6 ±135° | **matches** — trust it |
| Reach | — | 495–550 mm excl. jaw | matches `ARM_MAX_REACH = 0.55` |
| Total mass | 0.72 kg | not published for D1; the D1-T variant is listed at 2.37 kg | **too light** |
| Joint effort | 3.33 Nm (J1–J3), 1.67 Nm (wrist) | **not published** | **too weak — see below** |

The joint ranges matching the spec exactly is good evidence the geometry came
from Unitree. The effort limits did not: Rescue's notes say this URDF shipped
with effort and velocity limits of **zero** and had them filled in by hand.

You can show 3.33 Nm is wrong without any hardware. Unitree rates the D1 at a
500 g payload over a 550 mm span. Holding *just the payload* at full extension
needs `0.5 × 9.81 × 0.55 ≈ 2.7 Nm` at the shoulder — before the arm's own ~2.4 kg,
which adds roughly `2.37 × 9.81 × 0.20 ≈ 4.7 Nm`. So a real D1 shoulder needs on
the order of **7 Nm**, and 3.33 Nm could not hold the arm's own rated payload at
reach. It is a placeholder.

**Use `--arm_mass 2.4`** as the best available estimate (the D1-T's 2.37 kg is
the closest published figure; the Go2-mounted D1's own mass is not published).
That happens to scale the effort limits to ~11 Nm at the shoulder, comfortably
above the ~7 Nm the spec implies — so it is a physically plausible arm, even
though the absolute torque is still inferred rather than known.

Results:

| Configuration | Total mass | Distance in 10 s | Max tilt | Arm EE error | Verdict |
| --- | --- | --- | --- | --- | --- |
| `--no_arm` (baseline) | 15.02 kg | 9.56 m | 6.3° | — | stayed up |
| arm at URDF mass | 15.74 kg | 9.59 m | 6.2° | 12.4 cm | stayed up |
| **`--arm_mass 2.4`** (realistic) | 17.42 kg | 9.64 m | 8.0° | 4.7 cm | **stayed up** |
| `--arm_mass 3.6` | 18.62 kg | 9.82 m | 15.9° | 3.8 cm | stayed up |
| `--arm_mass 6.0` | 21.02 kg | 2.22 m | 106.2° | 0.7 cm | **FELL OVER** |

**The headline: at a realistic 2.4 kg the Go2 walks with the arm on its back** —
96% of commanded speed, 8° of tilt, versus 6.3° for the bare dog. The policy was
never trained on this payload and carries it anyway. It fails somewhere between
3.6 kg and 6 kg, which is well outside anything a real D1 weighs.

Run with the URDF's own mass and you will conclude "the arm doesn't affect
walking" — which is true, and meaningless, because that arm weighs nothing.
Use `--arm_mass` for any result you intend to believe.

Read the EE error column alongside the verdict — the two are coupled, and
ignoring that is how you get a false pass. A lighter arm tracks its IK target
worse (see *Known behaviour*), so it sits folded near the dog's back rather than
held out at the commanded 0.3 m forward. A payload tucked against the body
barely moves the centre of mass. The 6 kg row falls over precisely *because* its
arm holds the commanded pose: the same mass, actually extended, tips the dog.
An arm that droops is a lenient test.

`--arm_mass` scales the link masses, the inertia tensors **and the joint drive
force limits**, all by the same factor. The last part is not optional: the
URDF's effort limits (3.33 Nm on J1–J3, 1.67 Nm on the wrist) are sized for the
0.72 kg shell. Scale the mass alone and the motors cannot hold the arm up — it
sags to its limits, the IK sees an error it can never null, the joint targets
wind up, and the arm thrashes. That is not a heavy D1; it is a robot that does
not exist. Scaling the limits together models a D1 of that mass whose motors are
sized in proportion.

The inertia tensors are scaled by the same factor as the mass, which assumes the
added mass has the shell's spatial distribution. It does not — the motors sit at
the joints. It is a deliberate approximation: total mass and its rough placement
dominate the gait disturbance. **If you want a result you can quote, fix the
URDF's inertials properly rather than leaning on this knob.**

## Known behaviour

**The arm's IK falls short of its target, and how short depends on drive
authority.** At the URDF's own mass and effort limits it settles ~12 cm short;
scale the arm up (and its motors with it) and the error collapses — 3.8 cm at
3.6 kg, 0.7 cm at 6 kg.

This is *not* the DLS solver trading position error against orientation error.
The evidence against that: the arm's joints do not reach their commanded angles
at all — with the URDF's limits the IK asks Joint4 for −11.6° and gets −56.3°.
The solver is fine; the joint never arrives. What moves the number is the arm's
effort limit (3.33 Nm on J1–J3, 1.67 Nm on the wrist) together with its drive
stiffness, both of which `--arm_mass` scales.

So the shortfall is fixable, and the fix is to get the arm's *specification*
right rather than to touch the solver. If the real D1-550's motors are stronger
than the 3.33 Nm this URDF claims, correct that and the arm will track. Treat
the ~12 cm at the shipped URDF mass as a symptom of an under-specified arm.

The self-test reports this as `arm EE ... -> error N cm`. If it grows without
settling, the arm is diverging rather than merely short — check that the drive
force limits were scaled with the mass.

**Contact sensing does not cover the arm.** `FlatSceneCfg.contact_forces` uses
`{ENV_REGEX_NS}/Robot/.*`, which matches the Go2's links but not the arm's —
those sit one level deeper, at `Robot/D1/...`, because the weld nests them. This
is reporting only: arm collisions still happen in physics, they just are not
readable from the sensor. The locomotion policy does not use them. Widen the
regex if you ever need to detect the gripper touching something.

## Files

| File | Role |
| --- | --- |
| `main.py` | argument parsing, launches Isaac Sim, hands off to `sim.py` |
| `sim.py` | teleop keybinds, the sim loop, and the `--selftest` harness |
| `weld.py` | URDF import + the go2/D1 weld; mass and effort rescaling |
| `flat_env_cfg.py` | flat-plane scene, and the leg-scoped obs/action terms |
| `d1_ik_controller.py` | DLS Cartesian IK at the D1's real 10 Hz, ported from Rescue |
| `d1_direct.py` | buffers arm commands off the keyboard thread, writes to PhysX |
| `agent_cfg.py` | RSL-RL config for the walking checkpoint |
| `logs/` | the Go2 walking checkpoint, converted to the rsl_rl 2.x format |

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
