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
   `python demos/combiner_key/robot/check_shadow.py --policy checkpoints/<the .npz> --checkpoint <the .pt> --dog logs/leg_shadow/<stamp>`.

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
w, x, y, z are taken from rima's node (`~/rima_summerschool`), which runs Isaac Lab policies on this dog. Only a run on
the dog checks them here.
