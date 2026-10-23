# The combiner key: inserted by the arm, turned by the front-right foot

Lukas's lever key in the combiner box's cam lock, the door unlocked. The staged sequence is the combiner door demo's
(`demos/combiner_door`): the RealSense on the Go2's head finds the AprilTag and the MID-360 gives the door's heading.
Then the dog walks to the lock and the arm inserts the key without learning (inverse kinematics on the box's estimated
place). The dog walks in, and the key-trained leg policy turns the key with the front-right foot from 45° up to 45°
down. The door unlocks past 30° down. `--key_inserted` starts with the key already in the lock and skips the arm.

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH

./demos/combiner_key/run_combiner_key.py --leg_checkpoint <key stage-2 model.pt> --leg_variant position   # watch one
./demos/combiner_key/run_combiner_key.py --leg_checkpoint <...> --leg_variant force --attempts 8 --headless
./demos/combiner_key/run_combiner_key.py ... --key_inserted          # the key already in: look, walk, turn
./demos/combiner_key/run_combiner_key.py ... --key_inserted --handover settled --legs unifp   # the turn that works
python -m unittest demos.combiner_key.tests.test_combiner_key        # the geometry and the arm's reach, no simulator
```

Runs go to `logs/combiner_key/`; then `./dashboard.py record <dir> --title "..."`. Two options are diagnostics, not the
demo:

- `--given_box` drops the camera and the lidar and hands the look the true placement. It is for developing the rest on
  a GPU with no room for the renderer.
- `--handover direct` switches at once, for a leg policy trained from the walking policy's stances (`--spawn_states`).
- `--handover settled` puts the robot, at the turn's stance, in the stance the leg policy trained from, instead of easing
  it there from the walking policy's (below). It is a teleport, to see the turn apart from the handover.

## What happens in an attempt

| phase | what | legs | arm |
| --- | --- | --- | --- |
| `spawn` (4 s) | lands and stands, on the box's middle, its door 0.70 m ahead | walking policy | folded at rest |
| `look` (1 s) | the head camera finds the tag; the lidar's frames give the door's heading (`demos/combiner_door`) | standing | folded |
| `unfold` (3.5 s) | to the home point in joint space; the key appears in the hand | standing | joint space |
| `walk_to_insert` | 0.17 m ahead, 0.33 m left, to the adapter's face 0.55 m ahead and the lock 8 cm right of the base | walking | home point |
| `pre`, `align` | the jaw centre to the key 25 mm out of the keyway (its shaft 15 mm short), rolled to 45° up | standing | IK, 10 Hz |
| `insert`, `seat` | along the estimated keyway axis to 4 mm past seated, so the grip's spring presses the key on | standing | IK |
| `release`, `back` | the jaws open, the grip lets go; the hand backs 6 cm straight out along its approach | standing | IK |
| `fold` | back to rest in joint space | standing | joint space |
| `walk_to_turn` | 0.10 m in: the door 0.435 m ahead, the lock 8 cm right | walking | folded |
| `turn` (≤ 20 s) | the handover (below), then: lift the foot to the key, press it round to the end stop, hold, lift off, put it back down | **leg policy** | folded |
| `stand` | the walking policy again | walking | folded |

With `--key_inserted`: spawn, look, `walk_to_key` (straight to the turn's stance), `settle_turn`, `turn`, `stand`.

## The pieces

- **The key** (`assets/combinerbox_lever_v3.stl`, Lukas's; `geometry.py`). A 10 mm shaft with two wards 15 mm across, on
  a bar 15 x 20 mm and 175 mm long. Beside the bar is a 2.5 mm plate 35 mm wide, its top flush with the bar's: a flat
  platform 100 x 50 mm, 48-148 mm out from the shaft. **As drawn it points the wrong way.** With its shaft in the lock
  and the platform up, the lever points to the robot's left, out past the box's edge. The turn Lukas described
  (front-right foot, clockwise, downward) needs it pointing right, toward the box's middle. No rotation of the part
  gives that with the platform up, so the demo draws and simulates it mirrored left to right: the same part printed
  handed the other way.
- **The slot, adjusted.** The cam lock itself is drawn, not simulated, on the box. A printed **lock adapter** stands
  over it (`asset.build_adapter_usd`): 16 mm thick and 1 mm off the door. Its keyway is the key's outline (the wards'
  15 mm by the shaft's 10) plus 1.5 mm a side, behind a 45° lead-in 5 mm wide, and it is set at the insertion angle,
  45° up. A seated key's bar stands on the adapter's face, 17 mm off the door. The cylinder's own keyway behind it,
  which would register the wards' angle, is not modelled; the record keeps the key's roll at seating.
- **The grip.** The key is a free rigid body (53 g, its mass and inertia from the STL). Held, it hangs from the jaw
  centre on a 6-axis spring of 800 N/m and 0.8 N·m/rad, and the arm feels the reaction. This is a compliant grip like
  the insertion training's (`insertion.CompliantGrip`), not the fingers holding the bar: the fingers close round the bar
  but nothing of it collides. Let go, the spring is off and PhysX keeps the key in the keyway. The jaws hold the bar's
  top and bottom 25 mm out from the shaft, between it and the plate. The fingertips end 1 mm short of its back face, so
  they stay off the adapter as the key seats.
- **The robot** is the combiner door demo's: the MID-360 and its mount drawn but not simulated, and no wrist camera.
  The leg policy's training robot is not used, because its payload physics puts the wrist RealSense and bracket (99 g)
  on Link6. With the key there, the D1 reaching the lock needs ~100% of its shoulder's 3.33 N·m and sags ~16 mm. The
  CPU model has the chosen poses at 78-84% without the camera.
- **The insertion** is the decoupled baseline's arm (`demos/unifp/decoupled_arm.py`: IK on the jaw centre in the base's
  current frame, at the D1's 10 Hz through its motion planner and servo model). It needs the hand level, rolled 45°,
  with the arm's wrist roll at -1. From the turn's own stance that does not solve: the adapter is 0.42-0.49 m ahead and
  the hand would sit on the head camera. Insertion therefore happens 10 cm further back: on the CPU model it solves at
  0.50-0.60 m to the adapter's face, for base heights 0.27-0.31 m (a sweep on 2026-10-21; `test_combiner_key.Reach`
  checks the chosen stance). On entering `pre`, the IK is warm-started from the planner's seeded search, as
  `choose_grasp` does. From `align` on, an outer loop integrates the jaw centre's measured miss into the command (1.5/s,
  at most 30 mm), across the keyway's axis only once the key goes in. The measured miss is Link6's pose here; on the
  real arm it is the forward kinematics of the encoders.
- **The walks** drive each axis of the base's frame on its own at 0.15-0.22 m/s until within 12 mm. P2Dingo's clocked
  gait stands inside 0.1 m/s on each axis, and a diagonal command whose forward share fell under that walked sideways
  only.
- **The handover to the leg policy.** The legs' targets are eased over 1.5 s from the walking policy's last action
  (held) to UniFP's default targets, then held 0.5 s, and the policy starts with an empty history, as at a training
  reset. The foot friction is changed to the robot's own, which the leg policy trained on, and back to the walking
  policy's (0.6 / 0.2) afterwards.
- **The turn** is the leg task's own logic (`unifp_train/leg_env.py`, `--leg_object key`). `env.LegTurn` runs its
  approach, task layer, outer loop and goal hold, so they are the trained ones and not copies. The policy's 76-wide
  observation is built as in training, with the foot's goal in UniFP's goal channels and, for the force-aware variant,
  the layer's force command. The key turns as in training too, by the analytic plant (`handle.LeverHandle`, `pad=True`:
  the foot sphere against the key's bar and platform, a lock friction that sticks, a cam detent before the unlock). The
  plant sits at the key's true pose; the layer works from the look's estimate. PhysX draws the key where the plant has
  it. The lock is 1.5 N·m with a 0.5 N·m detent unless given, in the middle of the frozen set's.
- **The door** is latched (`demos/combiner/latch.py`) until the plant's key passes 30° down. The robot's own check is
  the turn as its foot measured it at the end of the hold (`HandleTaskLayer.theta_meas`).

## What it does and does not show

**Results (2026-10-21, seed 1's final leg policies; F-164, F-165; the Week 6 log has every run):**

| | outcome |
| --- | --- |
| the look (head camera + MID-360), 12 attempts | tags 12/12; the lock's place right to < 0.6 mm across, 0.1-1.1° in heading, but **3-11 mm off in depth** (the tag's range) |
| insertion, the box given | **seated** (0.16 mm out, 1.5 mm across, roll -0.8°), and stays seated after letting go |
| insertion, from the look | **0/12**: on the keyway's axis (1.1-1.4 mm across) when the depth was within ~3 mm, then stalled 2.7-7.8 mm short, tilted 2-4°; missed the lead-in at 10-11 mm |
| the turn, settled handover, UniFP's ideal-PD legs (`--legs unifp`), box given | force-aware **4/4** (peaks 76-88 N); position-only 0/4, all past 120 N |
| the same with the head camera and MID-360 look | force-aware **3/4** (the fourth past 120 N); position-only 0/4, all past 120 N |
| the turn, settled handover, the stack's DCMotor legs | force-aware 0/4 (never engaged); position-only 4/4 unlocked, 3 past 120 N |
| the turn, the eased handover from the walking policy | 0/4 engaged on either actuator: the worst support foot starts ~16 cm off, the door 0.55 m ahead |
| the turn, a direct handover, the leg policy fine-tuned from the walking policy's stances (F-167) | 0/8 (box given), 0/4 (camera): it starts from those stances at rest in training (97/108), not from the live handover |

So, for Lukas's questions:

- **The insertion without learning** works laterally (IK, the outer loop, the lead-in) and seats the key from a known box,
  but not yet from the look. The missing piece is along the keyway's axis: a push limited by force, not aimed by
  position, or a depth the robot measures by touch.
- **The turn** needs the leg actuator the policy trained on. With it, the force-aware policy turns the key in this
  scene as it does in the frozen evaluation (`unifp_train`, F-166: 190 against 43 of 216 over two seeds).
- **The handover** from the walking policy is what is missing for the whole sequence. The leg policy has only started
  from its own stance.

Limits of these results:

- **Not the trained conditions** unless `--legs unifp`. The decoupled stack's DCMotor legs (the walking policy's
  actuator: 23.5 N·m on every joint and a torque-speed curve) take the force-aware policy (seed 1) from 84 to 3 of 108
  frozen keys. That is their armature, 0 in the stock group against the trained 2e-4, not their torque limits: with
  2e-4 they give 84 (F-168, `unifp_train/leg_actuators.py`). Self-collision is off here (on in training). The payload is drawn, not simulated. The policy trained with the
  wrist RealSense's mass on the arm.
- **The insertion is PhysX on boxes.** The square shaft in a rectangular keyway stands in for the round shaft and the
  cylinder's keyway. The grip is a spring, not friction at the fingers, so a key cannot slip in the hand.
- **The turn is the training's plant**, not PhysX contact. The foot is checked against the door's face, the key's two
  boxes and nothing else of the box. The robot's own check (the foot's measure of the turn) reads false positives when
  the foot slides past the platform.
- Four box placements, four attempts per condition, one seed.

## The arm turning the key instead (2026-10-21, F-183)

Lukas: "I am curious if the arm itself can turn the lever" -- the foot's turn, 45 degrees up to 45 down, scripted, with
"the real lever takes about 5N to rotate in the lock". The key starts in the lock; the legs stay with the walking
policy; nothing is learnt.

    ./demos/combiner_key/run_arm_turn.sh                                         # watch one; frames/ and push.gif
    ./demos/combiner_key/run_arm_turn.sh --lock_n 10                             # a lock felt as 10 N at the push point
    ./demos/combiner_key/run_arm_turn.sh --attempts 8 --headless --no_overview   # eight placements, no pictures
    python demos/combiner_key/plot_arm_turn.py logs/combiner_key_arm/<run> --out <figure>.png
    python -m demos.combiner_key.arm_turn                                        # the CPU check: IK and push ceiling

| phase | what | arm |
| --- | --- | --- |
| `look` | the door demo's: the head camera's tag, the MID-360's face fit | folded |
| `unfold`, `walk_to_key`, `settle_arm` | toward the door 0.44 m ahead and the lock 8 cm left of the base, stopping the first time the estimate is close enough (door 0.42-0.45 m, lock 4-12 cm left, facing to 3°: `arm_turn.ACCEPT_*`), as the real door pull accepts a range | home point |
| `arm_out`, `arm_above` | the hand 8 cm out from the platform, then 3 cm above it, pitched 60° down onto it, jaws shut | IK, 10 Hz |
| `arm_touch` | the fingertips' edge 4 mm into the platform, 0.13 m out from the shaft | IK |
| `turn` (9 s) | the hand rotated rigidly about the shaft from -45° to +45°, the edge pressing | IK |
| `arm_hold`, `arm_lift`, `arm_back`, `fold` | resting on the key at its stop, off it, out, folded | IK, then joint space |

- `arm_turn.py` (numpy): the push's geometry, the stance, and the quasi-static scan. A level hand, as the isolator
  handle's push (`demos/combiner/press.py`) holds it, does not solve at the key: the lock is ~0.11 m above the base.
- `arm_env.py`: `ArmTurnEnv`, this demo's environment with the leg turn's slot (`_make_turn`) filled by the arm's: the
  same lock plant (`unifp_train.handle.LeverHandle(pad=True)`) against a 5 mm sphere at the fingertips' lower edge, its
  force on Link6; an overview camera for the pictures.
- The robot's own check (`verified_unlocked`): its fingertips' edge on the estimated key past 30° at the hold *and* the
  hand never more than 30 mm behind its command through the turn. The edge alone reads 45° on a stalled key once the
  fingers have slid off the platform.

| lock, N at 0.13 m | unlocked | notes |
| --- | --- | --- |
| 5 (Lukas's) | **27/27**, all to the stop | the hand 8-10 mm behind its command; 4.8 N along the arc; 16 of them stopping the walk in the window |
| 8 | 4/4 | 44.6-44.9° |
| 11 | 4/4 | 42-43°, short of the stop; joints at their limits 21-35% of the push |
| 14 | 1/4 | the key stalls near -44°, the fingers slide off |
| 17 | 0/4 | |

On the real dog: `robot/README.md`, "The key turned by the arm on the dog" (`d1dogkey`, dry unless `--live`).

The CPU's ceiling at the stance's spread is 1.31-1.50 N·m (10-11.5 N at the push point). Not shown: the real D1's
torque, the rest of the hand against the key, the cam's real feel, the wrist camera's mass on the simulated arm, where
the 5 N was felt. The contact force ripples 3-6 N about its mean: the D1's 10 Hz streaming as `motor_model` has it, not
the contact model.

## The key drawn out of the holster beside the arm's base (2026-10-23, F-189)

Lukas's holster for the key, beside the D1's base on the robot's right, re-angled 6° steeper after the first export
(the lever now 41° above level), so the arm can take the key by the bar's root (the dog's insertion grip) and slide it
out. The key in it is the mirrored v3, the simulation's.

    python -m demos.combiner_key.holster ~/Downloads/d1_with_holster.stl ~/Downloads/d1_with_lever.stl \
        ~/Downloads/d1_with_holster_and_lever.stl --write         # register a new export into assets/holster/
    python -m demos.combiner_key.holster --check                  # the draw on the CPU arm model
    ./demos/combiner_key/run_holster.py --headless                # one attempt in Isaac, frames/ and draw.gif
    ./demos/combiner_key/run_holster.py --attempts 8 --headless --no_overview --holster_error_mm 6   # misplaced holster

| phase | what | arm |
| --- | --- | --- |
| `spawn` (4 s) | stands; the key held in the holster | folded |
| `turn`, `reach` | Joint1 turned with the arm folded, then unfolded to the hand 45 mm out along the lever's axis, jaws open | joint space |
| `pre`, `on` | the key left to PhysX in the holster; the open jaws slid along the axis onto the bar's root | IK, 10 Hz |
| `close` | jaws shut on the bar; the spring grip on from where they closed | IK |
| `draw`, `hold` | 100 mm out along the holster's axis (its far end needs 75), held | IK |

- `holster.py` (numpy; trimesh to register): the CAD frame on `base_link`, the key's pose, the holster as nine boxes
  (0.06-0.34 mm off the bar), the draw's points and the hand (`hand_rotation(-n, u, 0, +1)`), the CPU check.
- `holster_env.py`: the key demo's environment with the holster carried kinematically on the base (its contacts with
  the robot filtered) and the key as a free body with its bar, platform and shaft as colliders.
- Timing (F-190): by default the unfold is twice the phase lengths above and the slide onto the bar and the draw are
  paced so the fastest joint turns at 0.6 rad/s (`--speed 2 --joint_speed 0.6`; the sequence 12.3 s). The D1 plans each
  10 Hz setpoint from rest, so slow moves go stop-start; even 2x lagged 37 mm. `--speed 1 --joint_speed 0` is the first
  timing (18.5 s).
- Results: 25/25 drawn at the first timing (as planned, up to 3 and up to 6 mm off); 8/8 at the default (up to 3 mm off). The key comes out turned 180° about its shaft from
  the insertion's grip (`geometry.GRASP_ROLL` -1 does not slide onto the bar). Not modelled: the spring grip's give, the
  holster's mounting, the wrist camera's mass; the real arm's reach to it is not checked.
