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
./run_sim.sh --arm_mass 3.6       # arm rescaled to 3.6 kg, motors scaled to match
./run_sim.sh --no_arm             # bare-Go2 baseline
./run_sim.sh --headless --selftest 10 --arm_mass 3.6   # walk 10 s, print gait stats
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

## The arm's mass is a trap

**The shipped `d1_arm/d1.urdf` describes a 0.72 kg arm.** Its inertials are a
SolidWorks export of the bare shells: no motors, no gearing, no wiring. A real
D1-550 is several kilos. At 0.72 kg the arm is ~5% of the Go2's mass and the
gait does not notice it at all:

| Configuration | Total mass | Distance in 10 s | Max tilt | Verdict |
| --- | --- | --- | --- | --- |
| `--no_arm` (baseline) | 15.02 kg | 9.56 m | 6.3° | stayed up |
| arm at URDF mass | 15.74 kg | 9.59 m | 6.6° | stayed up |
| `--arm_mass 3.6` | 18.62 kg | 9.51 m | 12.2° | stayed up |
| `--arm_mass 6.0` | 21.02 kg | 10.64 m | 16.5° | stayed up |

Run with the URDF's own mass and you will conclude "the arm doesn't affect
walking" — which is true, and meaningless, because that arm weighs nothing.
Use `--arm_mass` for any result you intend to believe.

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

- **The arm's IK settles ~13 cm short of its target.** This is inherited from
  Rescue, not introduced here — Rescue's own `D1_EE_REST_ROT` comment describes
  the same shortfall. `command_type="pose"` makes the DLS solver trade position
  error against orientation error, and it converges to a stable compromise. It
  is stable at every arm mass tested; it simply does not fully arrive.
- The self-test reports this as `arm EE ... -> error N cm`. Around 13 cm is
  expected. Tens of centimetres, or a number that grows, means the arm is
  diverging — check that the drive limits were scaled.

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
