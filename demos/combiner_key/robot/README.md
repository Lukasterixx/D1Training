# The key-turning leg policy on the dog: shadow mode first

Before the force-aware leg policy drives the Go2's legs, it runs here in **shadow mode**. The sport controller keeps
the robot standing. The policy reads the robot's own state every 20 ms and records what it would do, and **sends
nothing**: no LowCmd, no sport request, no arm target. `tests/test_leg_shadow.py` checks the node has no publisher.

| file | where it runs | what it does |
| --- | --- | --- |
| `export_policy.py` | PC (torch) | a checkpoint to one `.npz`: the weights and every constant of the policy's interface, read from the training code; refuses to write unless numpy reproduces the checkpoint |
| `leg_policy.py` | dog (numpy only) | the policy's 76-wide observation from Unitree-ordered joints and the IMU, its 32-frame history, the network, the joint targets |
| `shadow.py` | dog (ROS 2, P2Dingo's image) | the shadow node: `/lowstate` and the arm link's feedback in, a record out |
| `run_shadow_on_dog.sh` | dog | starts the arm link with `--observe` (it reports, takes no target) and the node |
| `check_shadow.py` | PC (torch) | `--sim`: a recorded Isaac rollout through the dog's pipeline; `--dog`: a dog record against the training code |
| `plot_shadow.py` | PC (matplotlib) | a dog record as a figure: roll and pitch, sport mode's knee torques, the foot sensors, the would-be knee torques; `--mark` shades what was done at the robot |

## Running it

1. **On the PC**, export the policy (done for the walker-stance fine-tune, F-167):
   `python demos/combiner_key/robot/export_policy.py --checkpoint <model_2104.pt> --variant force --out checkpoints/leg_key_force_s2walker_model_2104.npz`.
   Commit it (it is 6 MB) or copy it to the dog's checkout.
2. **On the dog**: the robot standing in sport mode on level ground, the arm folded at rest, and P2Dingo's arm bridge
   not streaming to the arm (E-STOP in the website's arm panel). Otherwise the arm link refuses to start, and the arm
   is taken at its rest pose.
   `demos/combiner_key/robot/run_shadow_on_dog.sh --policy checkpoints/leg_key_force_s2walker_model_2104.npz --seconds 90`.
   While it runs:
   - stand still for ~20 s;
   - push the nose down gently, then a side down: the printed pitch should go **positive with the nose down** (UniFP's
     convention) and the roll should follow the side;
   - press down on the front-right shoulder: the knee torques sport mode reports (`tau_est`, printed) should rise most
     on the first leg printed (FR).
3. **Back on the PC** (`d1fetch` copies `logs/leg_shadow/`):
   `python demos/combiner_key/robot/check_shadow.py --policy checkpoints/<the .npz> --checkpoint <the .pt> --dog logs/leg_shadow/<stamp>`
   (without `--checkpoint` when the `.pt` is not on this PC: only the torch comparison is skipped), then
   `python demos/combiner_key/robot/plot_shadow.py logs/leg_shadow/<stamp> --mark <start> <end> "nose pushed down" ...`.
   There is no live view: the node publishes nothing, so RViz (`view_on_pc.sh`) shows no robot during a shadow run.

A second run with `--foot_goal 0.30 -0.17 0.12` (the foot asked ~10 cm up, ahead) puts the policy in its lifting
regime on the real robot's observation, for the same still-robot comparison. Do not read the targets as where the
foot would go. A standing leg's PD targets carry its support torque: on the PC's fake robot they sit ~8 cm behind and
1-1.4 cm below the held foot with either goal. The lift goal shows instead in the FR knee's target, 0.6 rad off, with
~15 N·m asked at the knee.

## What a good record looks like

| in the record (`summary.json`, `check_shadow --dog`) | expect | if not |
| --- | --- | --- |
| `step_period_s` median / p99 | 0.020 / under 0.025 | the dog's CPU is busy; the policy's 50 Hz is not held |
| `state_age_s_p99` | under 0.01 | `/lowstate` is not at full rate (try `--topic /lf/lowstate` and note it) |
| `dog_vs_torch_action_max_abs_err` | under 1e-4 | the dog's numpy differs from the PC's |
| `frame_vs_training_code_max_abs_err` | under 1e-6 | the dog's frame builder differs from the training code's |
| `stand_from_joints` | front-back ~0.41, left-right ~0.28 m (your tape), feet ~0.3 m below the base | joint signs or zeros are not what the URDF says |
| `rpy_deg_median` | within ~2° of level on level ground | the IMU's frame or the quaternion's order |
| `still_robot_prediction.rms_rad` | a few hundredths of a radian | the policy sees the real robot differently from a still model in the same pose (the IMU, the velocities, the arm) |
| `estimates_dog_median` against `estimates_still_robot_median` | close | the same, through the estimator |
| `arm_measured_share` | 1 (the arm link ran) | the arm was taken at its rest pose; say so |

The stand's base height from the joints is also the number F-170 lacked. The policy was evaluated from the classic stand
at 0.28-0.34 m and holds up to 0.32 m.

## What it does not show

- **The policy closing the loop.** The sport controller holds the robot, so the policy's targets never act. How it moves
  the real legs is the next step, after a fine-tune with the delay and the armature F-168 asks for.
- **The handover.** The switch out of sport mode into low-level control is not part of this.
- The task layer beyond waiting: no key, no pre-contact point, no force command.

Checked on the PC (2026-10-21, the Week 6 log):
- a recorded Isaac rollout (4 episodes, 3,596 steps) through `leg_policy`: frames to 1.0e-7, actions to 1.1e-6, joint
  targets to 1.2e-7 rad, the waiting command to 4.9e-7;
- the node under ROS 2 against a fake 500 Hz LowState (localhost, domain 77): 601 steps in 12 s, a 20.2 ms p99 period,
  0.96 ms p99 inference.

The joint order (`motor_state` FR, FL, RR, RL), the gyroscope as the body's angular velocity and the quaternion as
w, x, y, z are taken from rima's node (`~/rima_summerschool`), which runs Isaac Lab policies on this dog.

Checked on the dog (2026-10-21, F-173): 50 Hz held (period p99 20.7 ms, inference 3.1 ms on its CPU), frames as the
training code's to 1e-17, targets within 0.013-0.078 rad RMS of a still model's. Pushing the nose and the FR shoulder
confirmed all four conventions above. The stand from the encoders came out 38.7 x 31.7 cm under 0.32-0.33 m, not the
tape's 41 x 28; a second stand came out 38.4 x 27.8 under 0.337 m, so sport mode's stand differs between stands.
With `--foot_goal 0.30 -0.17 0.12` (F-174) the FR knee's target cycles 0.42 rad at 4.3 Hz from that stand, as a still
model's does: the policy's open-loop response, not the dog's sensors. Both runs paused once for ~0.18 s at 30 s and
once for ~0.4 s at 60 s: the node saving its whole record inside the loop every 30 s. It now writes each second's
new steps to `parts/` instead (2.9 ms at most on the dog) and saves the record once at the end (F-175).

# The foot-press policy driving the legs onto a hand (`press_live.py`, 2026-10-23, F-187)

The general force-aware foot-press policy (`checkpoints/leg_press_force_r4_model_5999.npz`, F-180), with its task layer
ported to numpy (`press_layer.py`), on the real legs: the front-right foot goes out to a pre-contact point, the operator
puts a hand there, and it presses at most 5 N. The first program here that commands the legs.

**Round 4's export is the policy to use (Lukas, 2026-10-23).** What the first live run showed (F-188, F-194):
- **It crouches by design.** In simulation, from every start, it sits back 17-25° nose up within 0.3 s while still
  waiting, the base dropping to ~0.22 m on ~21 N·m at the support knees. It presses from that posture: each episode's
  largest tilt is 30° (median), p95 36°, at most 42°. The dog's 0.24 s in F-188 matched it (0.212 m, 20°, 20-27 N·m).
- **So the default stop ends every run.** `--tilt_limit_deg 15` fired on that posture, not on a fall. Round 4 needs
  the stop above its own range, ~40°.
- **The settle gate refuses the dog's rest for it.** Its export has no `start_gate`, so the default 0.20 rad / 5°
  applies, and the dog's kp-25 rest (0.20 rad, 8.1° nose up) fails it. In simulation round 4 presses from that rest as
  from its trained stands (F-192: 197 of 288, none fell; the dog's own two hands 280 of 288, F-195).

**The press candidate for light legs (2026-10-24, F-200):** `checkpoints/leg_press_force_ft11a_model_11394.npz`.
- **In simulation it does not rattle and stands its 10 s** at 12-14° nose up (on light legs never past 12.7°).
- **It presses the palm in front (`--plate palm_facing`) well on light legs** (122 of 144) and the palm under the foot
  (`palm_up`) poorly (27 of 144). Its own estimate of the foot's force reads a downward press that is not there
  (F-199), so the first press to try is `palm_facing`.
- **Keep the tilt stop at 40°** for its presses; its `start_gate` needs the program's start flags, as ft6's does.

**For the rattle (2026-10-24, F-196, F-197):** `checkpoints/leg_press_force_ft6_model_6998.npz` is round 4's final
fine-tuned on rima's legs: armature 0, no delay, unitree_rl_lab's observation noise and penalties.
- **In simulation it stands without chattering** on light legs and on the trained ones.
- **Nothing falls in its 10 s stand.** Its waiting tilt reaches 21-22°: past the default 15° stop, under 30°.
- **It is for a stand-only test** (`--presses 0`), not for presses: it presses the dog's hands worse than round 4.
- **Its export's `start_gate`** (pitch -3.8 to +1.0°) refuses the dog's rest unless the program's start flags
  override it.

`checkpoints/leg_press_force_ft2a_model_6598.npz` (F-193) is a fine-tune from the dog's handover starts. It is not
better than round 4's, and it crouches too (past 15° in 238 of 288 10 s stands): not for the dog. The level-base
fine-tunes after it (F-195) stood level but lost the palm press, and are not exported.

    d1update                       # the program and the export on the dog
    d1dogpress                     # dry: nothing sent, the policy open loop beside sport mode
    d1dogpress --rehearse          # dry: a pretend foot walks the whole sequence (prompts, stall, press, return)
    d1dogpress --live --presses 0  # the legs driven, the policy only holding the stand (--stand_s, 10 s): first
    d1dogpress --live              # the legs driven (type `live` to confirm)
    d1dogpress --live --presses 3 --plate palm_facing --hold_s 2
    d1fetch                        # logs/leg_press/<stamp>/ back for ./dashboard.py record

Live, from sport mode's stand with the arm folded:

| step | what happens | gains |
| --- | --- | --- |
| preflight | CheckMode gives the sport mode's name; the stand is read from the encoders (the base above 0.25 m) | sport mode |
| `lie_down` | sport StandDown; the calves below -2.3 rad, or the run stops before anything is released | sport mode |
| `release` | ReleaseMode, then CheckMode until it reports none; `/lowcmd` listened to 0.4 s for anyone else | none |
| `stand_up` | from where it lies to the policy's default targets (2.5 s), held 1 s, gains blended to the policy's with the targets kept (1 s), held until no joint moves 0.01 rad over 15 ticks (up to 3 s); off the trained starts it lies down instead: by default more than 0.2 rad or 5° from the trained settled stance, or, for an export with a `start_gate` (the stances its training started from, `export_policy.start_gate`), past its joint bound or pitch range (roll still 5°) | kp 60/80/80, kd 5/4/4 -> 25 / 0.5 |
| `policy` | Enter lifts the foot to the pre-contact point; Enter again (the hand there) presses: in at 3 cm/s until the foot stalls, ramped to 5 N over 0.5 s, held 3 s, off, out, home | kp 25, kd 0.5 |
| `stand_end`, `lie_end` | back to the stand, then to where it lay, then damping 1.5 s | -> kp 60/80/80 -> kp 0, kd 3 |
| `handback` | on Enter: SelectMode with the name from preflight, then CheckMode until it reports it | sport mode |

LowCmd goes out at 500 Hz (the latest command repeated, unitree_rl_lab's rate), reliable QoS as unitree_ros2's examples.

**Stops.** Ctrl-C, `a` + Enter, the remote's **B** (not yet seen on this dog: the one rehearsal with the dog read no
key at all), a roll or pitch past 15°, a joint past its limit while the policy
drives, or a LowState older than 0.1 s: every motor kp 0 / kd 3 at once (the dog sinks down), then Enter gives the legs
back to sport mode (a second Ctrl-C leaves them damping). `q` + Enter ends gently (released, home, lie down). The
decoder's estimate of the foot's force past 25 N for 0.1 s releases the press. Lying, sport mode folds the calves past
the URDF's -2.72 rad (-2.78 to -2.85 on the dog), so the limits are not checked in the stand-up and lie-down.

**The plates** (`--plate`, both rows of the trained table): `palm_up`, the palm facing up under the foot at
(0.33, -0.20) m, its face 9 cm off the floor, pressed down onto; `palm_facing`, the palm facing the dog 0.42 m ahead and
20 cm up, pressed forward. A plate facing down (the press pushing up) is refused (F-182).

**The dog's rules, not the training's** (`press_layer.py`): the press waits at the pre-contact point for Enter, up to
15 s, then goes home; an approach that reaches the end of its travel without the foot stalling backs out unpressed (the
trained layer pressed anyway: no stall here means no hand); levels and the first ramp's start are capped at
`--max_force_n` (5 N; at most 35, the trained range). The frame moves with the base: no odometry, so the plate is held
relative to the base, where training held it in the world.

Checked (2026-10-23, the Week 6 log): the numpy layer against `foot_press.PressLayer` on closed-loop foot paths (rigid,
sprung, three segments, nothing there) to 1e-9 in float64; the CRC against Unitree's C (`motor_crc.cpp`) on words and on
a whole `LowCmd`; the program on a fake clock, live and dry, against a kinematic stand-in (`tests/test_press_live.py`);
over ROS 2 against `fake_go2.py` (12,520 LowCmd, none failing its CRC, none while sport mode held the legs); dry on
the dog, lying (`--rehearse`: the sequence walked, the arm link read, inference 3 ms).

**The first live run** (2026-10-23, F-188) stood up to sport mode's stance, started the policy as the gains reached
kp 25, and sat back within 0.24 s (rear knees folded to -2.43 rad, 20° nose up), the tilt stop putting it in damping:
that stance cannot be held at kp 25, and the policy took over a robot sinking through a crouch it never trained from.
The motors did what was asked (`tau_est` within ~5% of kp (target - q) - kd dq). The stand-up now settles into the
trained stance first, and Ctrl-C no longer kills the LowCmd writer (rclpy's SIGINT handler is off).

Not shown: anything the policy does with the real legs beyond those 0.24 s. It has never driven them; the stand-in has no dynamics; the
handover's ramps, the release's behaviour on this firmware and the remote's B bit are untried on the dog. For the first
live runs: the dog on the floor with room around it, someone at its side ready to catch it, Ctrl-C at hand, and a dry
`--rehearse` first to see the remote's B printed as `remote: ['B']`.

# The key turned by the arm on the dog (`node.py --task key`, 2026-10-21)

The simulated push (`../arm_turn.py`, F-183) as a task of the combiner door's robot program: the same look, walk in,
refit, arm link, aborts, keys, record and RViz view (`d1view`), with the key's phases from `key_plan.py`. Dry unless
`--live`; a live run shows its plan and waits for one Enter.

```bash
d1update                              # the code to the dog
d1view                                # RViz here: the box, the key at its angle, the push's arc, where to stand
d1dogkey                              # dry: look, unfold, walk in, reach, touch, turn, lift off, fold -- nothing moves
d1dogkey --stage reach --live         # live, one step at a time: the hand to 3 cm above the key and back, folded
d1dogkey --stage touch --live         #   ... onto the key and off it, no turn
d1dogkey --live --step                #   the whole turn, Enter before each step that moves something
d1dogkeydoor --live --yes             # the turn, then the door pulled to 60 deg from where it left the dog, no Enter
d1fetch                               # the record (logs/combiner_door_robot/<stamp>_key_<stage>_<dry|live>/)
```

| stage | phases after the look |
| --- | --- |
| `unfold` | unfold, pause, fold |
| `walk` | unfold, walk up (to the door 0.45 m ahead, the lock 7 cm left, facing it), settle and refit, home, fold |
| `reach` | ... `key_out` (joint move to the hand 8 cm out from the platform, pitched 60° down), `key_above` (3 cm above it), `key_clear` (straight up, clear of the head camera), home, fold |
| `touch` | ... `key_touch` (the fingertips' edge 4 mm into the platform, 0.13 m out from the shaft), `key_lift`, `key_clear`, ... |
| `turn` | ... `turn` (the hand rotated about the shaft, -45° to +45°, timed from the plan), `key_hold` (resting on the key at its stop), `key_lift`, `key_clear`, ... |

**Where to stand the dog.** Roughly in front of the box with the tag in view, 0.8-1.1 m from the door (since 2026-10-22 the
Go2's front camera looks, and closer in the tag leaves the top of its field: the box's root at least 0.95 m ahead,
`plan.FRONT_LOOK_MIN_ROOT_M`, `../../combiner_door/robot/README.md`, "The look's camera"): the robot walks
itself up (Lukas, 2026-10-21: "it needs a walk up stage. I don't want to position it myself"). The look puts the push's
stance -- facing the door, its face 0.44 m ahead at the lock, the lock 7 cm to the base's left (about the middle of
where the push works) -- and the `walk` stage walks there on the odometry: forward, sideways and turning at once (P2Dingo's
diagonal sidestep), each axis lifted out of sport mode's dead zone, the sideways floor raised if the dog does not move
sideways, stopped early by the gait's carry, and done once the odometry has it within 4 cm of where it was sent (a
green ring on RViz's floor). It stops early by the gait's carry (0.12 s at first: classic walk carries on 2-3 cm; learnt
from each stop) and takes a corrective step only when the gap is over 70% of the shortest step the gait makes (~6 cm):
smaller ones overshoot and settle back (the first live walks up stepped back and forth at the door). It is sent 13% short (`ODOM_WALK_SCALE`): on the dog the
refit after the walk put the door 42-63 mm nearer than the look in all four live walks, 12-15% of the walk, as sport
mode's odometry under-reporting it would. The refit (allowed to move the box 10 cm, the door pull's 6) then has to find
the box where the push works (yellow: door 0.42-0.48 m ahead, lock 2-12 cm left, square to 4°). The look refuses a stance more than 0.7 m ahead (or 0.2 m
back), 0.4 m across or 25° turned, and says how to move the dog. After the walk the refit's box is checked against the
push's area (door 0.42-0.48 m ahead, lock 2-12 cm left, door square to 4°); outside it, the attempt aborts before the
arm leaves home.

**What it adds to the door program** (`key_plan.KeyAttempt`, on `plan.Attempt`'s hooks):
- *The walk up* (above), in place of the door's straight walk in, the hand held back at `KEY_HOME_IN_BASE` (0.20 m ahead, 0.30 m up, in the base's own frame) through it: at the door pull's home (fingertips 0.42 m ahead) the D1 hit the door at the key's stance, and a script-frame height put the waiting hand inside the trunk proxy on the dog.
- *The hand's pitch planned per attempt* (50-70°, `PITCHES_DEG`): which one keeps the turn off the wrist's singularity depends on the base's height over the box (60° at 0.32 m, 50° or 70° at the dog's 0.353 m).
- *The turn planned before anything moves* (the first step, before the first command to the arm): every first push
  pose the seeded search finds, the turn traced from each every 2.5° (steps that jump subdivided, so passing close to
  the wrist's singularity is allowed and a flip is not), and the one whose Joint5 stays furthest from 0 kept. Through
  the push the 10 Hz IK is seeded from that plan at the key's commanded angle. Without it the dry run's wrist flipped
  mid-turn (J4 +84°, J5 +60°, J6 -94° in one step).
- *The turn retimed from the plan*: each step as long as the key at 10°/s or its fastest joint at 15°/s needs, eased:
  no joint over 22.5°/s, under the arm link's 30. Near 0° Joints 4 and 6 swing ~90° each way over 20° of key; at the
  simulation's pace that asked 66°/s and the arm fell 39° behind. The turn takes 12.7-15.5 s on the CPU model.
- *The turn waits for the arm*: while a joint is more than 8° behind its goal the turn's clock holds. It aborts as "not
  turning" only when the fingers' measure of the key has not advanced 2° in 3 s (at 5° and a 3 s budget of holds, two
  live turns stopped at +25° and +14° with the key plainly turning: Joint4 runs 5-7° behind near the singularity).
- *A stall abort*: the hand more than 30 mm behind its command for 0.6 s in the turn ("the key is not turning"), and
  the door program's lag abort (an arm joint 10° off for 0.6 s while on the key). `r` then lifts the hand off the key
  along the platform's normal, raises it clear of the head camera (`key_clear`), goes home and folds.
- *The way off the key past the head camera* (`key_clear`): the D435i and its bracket stand 4 cm past the trunk proxy's
  front (`head_camera.CLEARANCE_BOX_BASE_M`), outside the arm's clearance check. After the first turn to the stop the
  old way off, 8 cm back along the hand's approach and straight home, put the fingertips through the case: at +45° the
  approach points right and down. Now the hand rises straight up from above the key, its orientation held, to 0.20 m
  above the base, and only then goes home (dry: >= 45 mm of proxy margin from the case, 79 home). Each phase's smallest
  margin, from the measured joints, is in the record (`camera_gap` events).
- *The robot's own check* (`turn_check`, in the status, the record and RViz's text): the fingertips' edge on the
  estimated key past 30° at the hold, and the hand never more than 30 mm behind its command through the turn.

**On into the door pull** (`--then_door pull`, `d1dogkeydoor`; `walk`, `reach` or `grip` to stop it short): once the
robot's own check says the key turned (else the run ends there), the door pull takes over from where the key left the dog
(`key_plan.DoorAfterKey`), with no new look and no backing off to walk in straight. Its stance is the key's fix moved to
the door pull's (`door_fix`: square to the door, the box's root 0.69 m ahead), ~0.07 m back and ~0.18 m right of the
key's. The arm stays folded (the door pull's home, 0.40 m ahead, is in the door at the key's stance), the dog walks
over with the key's walk up (back, sideways and turning at once), settles, and the door pull's refit and range check run
there as after its own walk in; then its unfold, reach, grip, pull, release and fold, at `--door_deg` and
`--speed_scale`. The record has both: `key_attempt_events`, `key` (the turn's check), `door_attempt_events`. Dry on the
CPU model only so far.

**Before a live run**, as the door's (`../../combiner_door/robot/README.md`, "Before a live run"): the box's tag in view
and the dock not; the dog standing; nothing else commanding the legs, the joysticks or the arm (E-STOP in the website's
arm panel); clear space; the Unitree remote in hand. And for the key:
- the key in the lock at 45° up, pointing right (toward the box's middle), its platform up and facing the robot: what
  `../geometry.py` has (the simulation's key is the STL mirrored). Compare it with the key RViz draws in the dry run;
- a first live `reach` and `touch` before the turn, watching that the fingers land on the platform 0.13 m out.

**Checked on the CPU** (`../tests/test_key_robot.py`, 20 tests; this paragraph's list is the first 12): the look's fix and the key's range check from
synthetic tag poses and lidar returns; a dry attempt through every stage with the arm link's profile driving a model
servo and the gait's carry, ending folded; the turn to the stop with the robot's check verifying it (lag 7.7-8.6 mm);
a far box 3° off square still planned without a flip; every joint under the arm link's speed through the planned turn;
a stalled key aborting and the hand lifting off; RViz's key where the fingers put it. Since: the walk up, the waiting
hand clear of the trunk at 0.30-0.37 m, an arm running behind not stopped, the way off the key clear of the head camera
for every stage, the door pull after the turn and not after an unverified one. **On the dog** (F-184): the key turned to
its stop and verified in the third live turn; `key_clear` and `--then_door` not yet run there.

# The key inserted by the arm on the dog (`node.py --task insert`, 2026-10-21)

The simulation's insertion (`../env.py`: the hand level, rolled so the key goes in at 45° up, the jaws across the bar
25 mm out from the shaft) as a task of the same robot program, with the key turn's look, walk up, refit and head-camera
check. `insert_plan.py` (`InsertAttempt`, `stage_phases`) and `insert_viz.py`; dry unless `--live`. Since 2026-10-22 the
look is the Go2's front camera's (`--camera front`, the default: the arm stays folded for it) and the wrist RealSense is
used for the keyhole view alone (below), from its pose by the arm's encoders as before.

```bash
d1doginsert                           # dry: everything planned and drawn, nothing moves (the key is imagined)
d1doginsert --stage load --live       # live: unfold, 3 s to put the key in, it closes, 3 s, it lets go, fold
d1doginsert --stage lie --live        #   ... walk up with it, lie down, refit and plan, stand up, hand it back
d1doginsert --stage touch --live      #   ... the key in along its axis until it touches, back out
d1doginsert --live --yes              # the insertion, no Enter anywhere
```

The first two live runs (2026-10-21) aborted at the start of the walk, "odometry 1.4/1.8 s old", after the key was
handed in. The hand-in phase is named `grip`, like the door pull's, and the node read the door's angle from the lidar
through it, a thread back to back for the whole 43 s wait. The door reading is now the door pull's only
(`Attempt.reads_door`), the trace is flushed every step, and odometry over 0.5 s old is logged in any phase.

| stage | phases after the look |
| --- | --- |
| `load` | `unfold` (the jaws open in front of the dog, the hand as it inserts), `load` (a 3 s countdown in the log: put the key between the jaws and hold it), `grip` (they close; empty, they open and count down again, three tries), `unload`, `hand_back` (3 s: take hold of the key), `drop` (the jaws open), `fold` |
| `walk` | `walk_in` (the key turn's walk up, to the seated key 0.50 m ahead and the lock 0.10 m right) and `walk_settle` with the lidar refit FIRST, the arm folded (since 2026-10-22); then `load`'s `unfold`, `load`, `grip`, ... |
| `lie` | the walk, then `lie_down` (sport mode's StandDown, P2Dingo's "sit"; held until the body is down and still) and `lie_settle` (no lidar refit; the odometry plus the measured 4 cm sit-back carry the fix; the insertion planned there) with the arm still folded, `lock_look` (the folded wrist camera tilted up at the lock and panned by the arm's base; what it finds moves the box model, since 2026-10-23); then `load`'s `unfold`, `load`, `grip` lying; `stand_up`, `stand_settle` (StandUp, BalanceStand), `unload`, `drop`, `fold` |
| `reach` | ... `key_go` (joint move to the key 25 mm out on the keyway's axis), `key_pre`, `key_back`, `home`, ... |
| `touch` | ... `key_in` (in along the axis at 2 mm/s until the key stops following its command), `key_back`, ... |
| `insert` | ... `key_in`; `key_search` (a spiral across the keyway pressed 2 mm into the face, out to 4 mm); `key_seat`; `release`; `key_back`; `home`; `stand_up`, `stand_settle`; `fold` |

**Why lying down.** Standing, the base is 0.35-0.39 m over the box's root on the dog, and the lock only 3-6 cm above
it. The CPU arm model finds the level, rolled hand from no stance there (0.40-0.70 m from the door, 0.10 m left to
0.20 m right). The simulation's walker stood 0.27-0.31 m. Lying (StandDown, the base ~0.20 m lower), 29-46 stances
work with the shoulder at 34-67% of its torque. Unitree's `Sit` (1009) tips the chest up instead. The drop is
measured from sport mode's `body_height`.

**Walk first, hand in at the stance, no lidar lying** (2026-10-22). Carried out in front (0.28 m ahead, 0.25 m up, about
10 cm ahead of and above the MID-360), the key and hand shadow the door: the refit after a walk with the key in hand
turned the door 6.9 degrees and aborted the run. So the dog walks up with the arm folded, refits with a clear view, and
only then unfolds for the key; you hand it in standing beside the box. Lying, the key is in front again, so there is
no refit there (Lukas: "we cant trust the lidar after sitting down, as the tool obscures it"): the fix rides the
odometry, plus the part of the measured sit-back (4 cm straight back, `LIE_BACK_M`; the first lie-down's 4.5 degrees of
turn the odometry saw) it did not report. Whatever of the sit-back that misses is left to the keyhole view and the
touch search.

**Lie down folded, look for the lock, then the key** (2026-10-23). In Lukas's live run (2026-10-22 13:29 UTC) the wrist
camera at `key_view` saw the held key and the jaws where the model put the bezel, the bezel nowhere, and the key then
went into empty air ("no lock there?"). Lukas: "the search stage should happen right after the robot sits down while the
arm is folded - tilt the wrist up and pan left and right using the base. then continue with the extension and receiving
of the tool". So now: walk, refit, lie down, all folded; `lock_look`; then `unfold` and the key is handed in lying.
- `lock_look` (`insert_plan.aim_folded`, `_plan_look`, `_use_look`): from the fold only servo 5 (the wrist's pitch, up)
  and servo 1 (the arm's base) move: aimed at the model's bezel, then servo 1 by 0, +15, -15, +30, -30 degrees
  (`LOOK_PAN_DEG`). Places nearer the head camera than 2 cm, or whose move crosses the trunk proxy, are left out. Each
  place is held until 3 frames found nothing or 5 s after it settled (the dog's node searched ~1 frame a second in that
  run); the first with 3 finds ends the look. The image's own search looks 80 mm round the model's place
  (`LOOK_WINDOW_M`). What it finds moves the box model across the door onto the bezel (`check["look"]["moved_mm"]`),
  the insertion is planned again from there, the arm goes back to the fold, and `unfold` follows.
- In simulation (the test harness's renders, no head or Mid-360 in the way): aimed with the wrist 29 degrees up from the
  fold, the camera 0.48 m from the bezel, the bezel 25 px in radius, found from the first place; a bezel 20/-15 mm off
  the model moved it 20.2/-15.3 mm, and the key seated with `key_view` seeing nothing.
- From 0.48 m a degree of the wrist camera's mount error is 8 mm on the door. `key_view` close up refines it when it
  sees past the key; otherwise the key goes to where the first look put it, and the touch's 4 mm spiral and the kept
  calibration take up the rest. `key_view` itself is one place now, held until it has searched 3 frames.
- Not yet on the dog: whether the head camera, the heatshield or the Mid-360 cut the folded camera's view of the lock.

**After the first live run of `lock_look`** (20261022T135352, 2026-10-23):
- It found a bezel-sized disc (25 px radius, light all round) from both the aim and the +15 degree pan, the same disc
  250 px apart, 72-74 mm across the door from the model; from the aim the rim fit refused it as "not distinct" (83 against
  65 for a centre 4 mm off: 4 mm is a fifth of its radius at 0.48 m). A bezel the image found now skips that test (its own
  light-all-round test is the stronger one). The key then went 5 mm past seated touching nothing, as in the run before.
- **The key's grip**: Lukas hands the key with its shaft on the gripper's roll axis, not by the bar 25 mm out
  (the simulation's `geometry.KEY_GRASP_U_M`), so every insertion went 25 mm along the lever, "to the top right".
  `insert_plan.KEY_IN_JAWS_LINK6` puts the shaft on the axis for the robot (the simulation keeps its grip); the load
  prompt says so.
- **Depth**: both runs added 44 mm of sit-back the odometry did not see. A lock further than the model makes the key run
  out of travel untouched. Now the image's find is placed along its ray at the depth at which the 13.75 mm circle, on
  the model's door orientation, projects to the ellipse's size (`keyhole._image_find`, `out_mm`), used when it disagrees
  with the model's plane by more than 6% of the range (renders: the size's range good to 0.1-0.5% at 12-25 cm, 2-3% long
  at 0.48-0.6 m). `lock_look` and `key_view` move the box along the door's normal by it; the rim fit starts at that
  depth. And `key_in` goes on until it touches, up to 40 mm past the model's seated (`IN_SLACK_M`, `IN_S` 45 s); a touch
  more than half the keyway's depth past seated is taken as the lock's face.
- **Speed**: each search took 1.1-1.4 s inside the node on the dog (0.14 s alone); the image search now looks only at
  the window round the model's place (its corner on 32 px so the pyramid samples as uncropped).
- **The flicker**: through `lock_look` the front camera's frames and the wrist's took turns on RViz's one image topic;
  the wrist's is now shown throughout.
- Each keyhole search's frame, drawn, is kept in the run's `keyhole/` folder (at most one a second), and each search's
  phase and time are in the record.
- Simulation: with the dog sitting back 80 mm (model 40) the looks moved the box 39 mm out (true 40) and the key seated;
  without the camera, 30 mm further, the key touched past seated, took it as the face, searched and seated.

**The look's window widened to 150 mm** (20261022T142556, Lukas: "the search needs to be widened"). The bezel was in
the picture from the aim, ~140 px right of and ~65 px below the model's place (its saved frames), and the image's find
was refused as "more than 80 mm from the model's" in every frame. In the two frames that also held the tag, the bezel
measures 247-251 mm left of the tag's centre and 35-36 mm below it (the tag's 100 mm square as the door plane's scale);
the box model has 251 and 40. So the box is right and the offset, ~66-77 mm right and 35-39 mm down in both runs, is in
where the box was put against the wrist camera: the front camera's look, the walk and the sit-back, or the wrist camera's
mount and the arm's encoders. `key_view` now searches as wide when the first look found nothing. In simulation the first
look finds a bezel that far off (19/-74 mm along u/t) and moves the model by 19.1/-74.4; from the fold, that far to the
side, it leaves a few mm (its depth is the model's), which `key_view` takes up.

**Why touch and search.** The bare cam lock's keyway has under 0.5 mm of play (Lukas). Nothing the robot measures puts
the key that close: the look was 3-11 mm off in depth in simulation, and the D1 sags ~16 mm holding the key out. So the
depth is found by touch, as the place where the key stops following its command 3 mm more than it lagged in the first
second. Where along the axis it stops says where it is. Near the keyway's 8.7 mm out, the shaft is on the lock's face,
and an Archimedean spiral (0.4 mm pitch, 1.5 mm/s) searches across at that depth until the key drops 1.2 mm. Near
seated, it went straight in. Seated is by touch too: stopped again at least 6.7 mm past where it met the face (the keyway 8.7 mm deep, less 2). Every
depth and miss is the forward kinematics of the arm's encoders, with the key where it was handed in. A key that turns
or slides between the fingers is not seen.

**Dry on the CPU model** (`../tests/test_insert_robot.py`, 13 tests): the arm link's profile driving a model servo, the
gait's carry, the odometry's 13% short count, StandDown lowering the base 0.20 m, and a model keyway: rigid, no
friction, 0.4 mm of play, put 0.2-3.8 mm off where the box's model has it. Every stage short of the insertion hands the
key back and folds. The touch finds the face. The search found the keyway each time and the key seated, 13-67 s
searching, 52-109 s from the look. A keyway 6 mm off aborts ("no keyway within 4 mm"), the key comes back out, the dog
stands and the key is handed back. Jaws closing on nothing abort before the walk. A dog that does not lie down aborts.
The plan solves lying from a dog standing 0.33-0.39 m over the box, the shoulder under 0.9 of its torque. The arm stays
off the head camera throughout.

**Not run on the dog.** Nor checked anywhere: StandDown with the D1 on its back and the key out in the hand. The lock as
measured (Lukas, callipers): face 4.5 mm proud of the door, keyway 8.7 mm deep, the printed adapter's outline; the key's
10 mm shaft bottoms out, so it seats 5.8 mm off the door. Its place along the door is `../geometry.py`'s, which the
turn's fingers found good to the platform's ~1 cm. What the D1's position servos push with at 3 mm of lag. Whether a printed
key goes into a cam lock from a spiral at all, with friction, a round shaft, and the wards' 45° set by the hand to the
D1 wrist's accuracy.

## The keyhole, seen by the wrist camera (2026-10-21)

With the camera on the wrist, the insertion looks at the lock before putting the key in. `key_go` now takes the hand
to `key_view` (the key's back face 85 mm out, the bezel ~17 cm from the camera, ~150 px across), held 2.5 s. After the
first 0.8 s each frame is searched for the lock's black bezel (`../keyhole.py`):
- **the method**: the known circle (27.5 mm) on the known plane (the door, from the look and the lidar), from the
  camera's pose by the encoders. Candidate centres within 12 mm are projected as that circle and scored by the
  contrast across its rim, dark inside, light outside, on a coarse grid then finer ones.
- **acceptance**: at least 40% of the rim must be seen, and the best centre must stand 1.5 times above the best one
  4 mm away. A plain door, a hidden rim, or two candidates alike are refused.
- **what is used**: the median of at least 3 frames moves the key's target before `key_pre`.
- **depth**: the depth along the door's normal is not searched; seen 30 degrees off, depth and place trade off. When
  the key touches the face, the touch gives the face's depth. The camera's centre is carried along its ray to that
  depth, and the search is centred there.
- **self-calibration**: when the key seats, the difference between where it went in and where the camera put the
  keyway is kept in `logs/combiner_key_calibration/keyhole.json` on the dog. The mean of the runs, each capped at
  6 mm, is added to the camera's estimate in later runs. It holds the camera mount's error, and the arm's between
  `key_view` and the touch, but also how each hand-in put the key in the jaws, which varies.
- **fallback**: without the bezel the insertion carries on as before, by touch and the spiral search.

Dry, on the CPU model: rendered frames (`keyhole.render`, a ray-cast flat door, no lighting, no distortion) from where
the arm truly is, the dog sat back and turned. The keyway was put 3.5-9 mm off the model; the camera put the key in
with no search each time, and the residual kept was 0.01 mm. `../tests/test_keyhole.py`: within 0.2 mm with the key
across the rim; depth carried along the ray within 0.2 mm; a plain door and a hidden rim refused. **Not run on real
images**: the black rubber bezel's sheen, the chrome, the wrist's light, blur at 17 cm (the D435's colour lens is
fixed-focus), and the mount's real error are all untested.

### The bezel found in the image first (2026-10-23)

The search above only looks 12 mm round where the model puts the lock, and the model is only as good as the look and
the wrist camera's mount. **The bezel is now found in the image first** (`../bezel_image.py`, cv2), inside the projection
of 45 mm round the model's place and at the size the model's distance gives it. The ray through the ellipse's centre
meets the door's plane, and the rim search above is centred there. Nothing downstream changes: the same `Bezel`, plus
`how` ("image", or "model" when the image found nothing and the search fell back to the model's place) and `image` (the
ellipse and its tests). `node.py --keyhole_wide_mm` (default 45; 0 is the old search alone). The RViz frame draws the
image search's circle (dark grey) and its find (magenta). The gripper's jaw, always lower right in the wrist image, is
masked out (`bezel_image.gripper_mask`).

The finder: a dark disc with light door **all round** it (8 sectors each side of the rim, one of each allowed lost),
refined to an ellipse through its rim along 90 rays, accepted on the rim's support, its contrast, the light round it
(the box's "shoals" logo has an "o" the bezel's shape and size, with letters either side), how much of it is seen and the
ellipse's fit. Developed and scored on two hand-held wrist clips recorded through P2Dingo's website (1470 frames, daylight
and dim red room light, heavy motion blur in places), ground truth tracked through each clip and checked frame by frame
(`results/week_06/keyhole_clips/`; the clips themselves in `logs/keyhole_clips/` on the PC):

    python3 -m demos.combiner_key.robot.eval_keyhole_clips [--video]

- The image alone, no prior: right in 754/772 and 565/566 frames, wrong in 3 and 0 (a far blurred bezel ~10 px in
  radius), something found in 4 of 91 frames with the lock out of view. Searching every size, ~85 ms a frame on the PC
  and 0.6-0.9 s on the dog's CPU; `find_bezel` with the window 40 ms on the PC, 144 ms on the dog (71 ms without).
- `find_bezel` with the model's place pushed E mm off the truth (a made-up pose per frame, the clips have no arm pose):
  0-10 mm, the model's search alone and image first are both right in 146-147/152 and 102/114 frames; 20 mm, the model's
  alone 2/152 and 3/114, image first 146/152 and 102/114; 30-40 mm, 0 against 144/152 and 101-102/114.
