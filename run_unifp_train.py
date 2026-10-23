#!/usr/bin/env python3
"""Train UniFP's position/force task natively in Isaac Lab.

`run_unifp_isaaclab.py` runs a checkpoint trained on the legacy Isaac Gym stack. This trains one
here instead, which is what F-088 argues for: the Isaac Gym policy holds a stance on the Isaac Lab
model but falls within half a second when told to walk, so moving trained weights between the two
simulators is not a reliable step.

    ./run_unifp_train.py smoke --num_envs 16 --headless            # build the env and step it
    ./run_unifp_train.py play  --num_envs 16 --headless            # ... driven by model_48800
    ./run_unifp_train.py train --num_envs 4096 --iterations 60000 --headless

    # the force-transmission task: pull a hooked handle / press a button (unifp_train/hook_env.py)
    ./run_unifp_train.py smoke --task hook --num_envs 16 --steps 400 --headless
    ./run_unifp_train.py hook_eval --task hook --checkpoint <model.pt> --headless   # staircase to 60 N
    ./run_unifp_train.py train --task hook --resume_from <unifp model.pt> --iterations 3000 --headless

    # the goal-commanded task: hold a handle, told only where it should go (unifp_train/mech_env.py)
    ./run_unifp_train.py smoke --task mechanism --num_envs 16 --steps 400 --headless
    ./run_unifp_train.py mech_eval --task mechanism --checkpoint <model.pt> --headless   # 4 kinds x 10-80 N
    ./run_unifp_train.py train --task mechanism --resume_from <model.pt> --iterations 3000 --headless
    ./run_unifp_train.py smoke --task insertion --num_envs 64 --steps 600 --headless        # a key into a slot
    ./run_unifp_train.py ins_eval --task insertion --checkpoint <model.pt> --headless [--ins_set detent]
    ./run_unifp_train.py smoke --task press --num_envs 64 --steps 600 --headless            # a spring-loaded button
    ./run_unifp_train.py press_eval --task press --checkpoint <model.pt> --headless [--press_set force]
    ./run_unifp_train.py smoke --task wipe --num_envs 64 --steps 900 --headless             # a pad along a path
    ./run_unifp_train.py wipe_eval --task wipe --checkpoint <model.pt> --headless
    ./run_unifp_train.py smoke --task rope --num_envs 64 --steps 900 --headless             # a taut rope that can let go
    ./run_unifp_train.py rope_eval --task rope --checkpoint <model.pt> --headless

`smoke` and `play` step the environment directly and report shapes and reward scale. `train`
builds the rsl-rl runner around it: `unifp_train.models.UniFPActor` for the adaptation-module
actor, `unifp_train.algorithm.UniFPPPO` for PPO plus the estimator loss, and
`unifp_train.agent.make_agent_cfg` for upstream's hyperparameters.

Every run writes a `run.json` next to its checkpoints, recording the stack versions, the welded
USD's hash, the git commit and the agent config, so `./dashboard.py record` has something to read.

**The GPU is shared.** Check `nvidia-smi` before starting a long run.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=("smoke", "play", "train", "manifest", "eval", "hook_eval", "mech_eval", "ins_eval",
                                         "press_eval", "wipe_eval", "rope_eval", "leg_eval", "leg_handover"),
                        help="'play' drives the new environment with an existing UniFP checkpoint, "
                             "which checks the whole loop -- observation, action mapping, reward -- "
                             "against a policy already known to work on this robot. 'train' runs "
                             "PPO with the adaptation module. 'manifest' freezes an "
                             "evaluation set; 'eval' scores a policy against one.")
    parser.add_argument("--task", choices=("unifp", "hook", "mechanism", "insertion", "press", "wipe", "rope", "leg"),
                        default="unifp",
                        help="'unifp' is UniFP's task as ported. 'hook' adds a fixture: a handle the "
                             "claw pulls or a button the pad presses, loaded with tens of newtons "
                             "(unifp_train/hook_env.py). 'mechanism' has the claw holding a drawer, "
                             "latch, door or button whose resistance is never observed, and commands "
                             "only where the handle should go (unifp_train/mech_env.py). 'insertion' pushes "
                             "a held key into a keyed slot (insert_env.py); 'press' presses a spring-loaded "
                             "button with the bare jaws or a probe (press_env.py); 'wipe' wipes a pad along a "
                             "path on a panel at a set force (wipe_env.py); 'rope' holds a steady pull on a "
                             "rope whose anchor may let go (rope_env.py). Same observation and action "
                             "widths, so a UniFP checkpoint resumes into any of them.")
    parser.add_argument("--rope_fraction", type=float, default=None,
                        help="'rope': fraction of episodes with a rope (rope_cfg default 0.8).")
    parser.add_argument("--release_prob", type=float, default=None,
                        help="'rope': of those, the fraction whose anchor lets go mid-pull (default 0.5).")
    parser.add_argument("--detect_never_prob", type=float, default=None,
                        help="'rope': of the releases, the fraction the task layer never notices (default 0.2).")
    parser.add_argument("--rope_stiffness_range", type=float, nargs=2, default=None, metavar=("LOW", "HIGH"),
                        help="'rope': rope stiffness drawn log-uniformly in training, N/m (default 500 6000).")
    parser.add_argument("--rope_length_range", type=float, nargs=2, default=None, metavar=("LOW", "HIGH"),
                        help="'rope': rope rest length drawn log-uniformly in training, m (default 0.3 2.0).")
    parser.add_argument("--rope_weight", nargs=2, action="append", default=None, metavar=("TERM", "WEIGHT"),
                        help="'rope': replace one of rope_cfg.WEIGHTS, repeatable (e.g. --rope_weight rope_ripple -2).")
    parser.add_argument("--curriculum_rule", choices=("window", "ema"), default=None,
                        help="'rope': how the force curriculum judges the frontier (default window; ema reproduces v1-v3).")
    parser.add_argument("--rope_stiffness", type=float, nargs="+", default=None,
                        help="'rope_eval': the stiffnesses evaluated, N/m (default rope_cfg.EVAL_STIFFNESS). A "
                             "sensitivity run, not the frozen set.")
    parser.add_argument("--rope_set", choices=("dev", "test"), default="dev",
                        help="'rope_eval': the development set (rope_cfg.EVAL_*, used to choose checkpoints) or the "
                             "held-out test set (rope_cfg.TEST_*, frozen 2026-10-07 for the final checkpoint only).")
    parser.add_argument("--freeze_legs", action="store_true",
                        help="'rope_eval': replace the policy's twelve leg actions with zeros, so the legs hold the default "
                             "stance under their PD while the arm acts: how much of the pull is the whole body's. "
                             "A diagnosis, never the frozen measurement.")
    parser.add_argument("--rope_ramp", type=float, default=None, metavar="N_PER_S",
                        help="'rope_eval': the command's ramp between levels, N/s (rope_cfg.LEVEL_RAMP_N_PER_S, 25). A "
                             "sensitivity condition, not the frozen set.")
    parser.add_argument("--rope_demo", action="store_true",
                        help="'play' with --task rope: the frozen evaluation's episodes at the 0.40 m grip and 4,000 N/m "
                             "(20 / 35 / 50 N, then the rope letting go three ways), one per environment, on a loop, "
                             "the viewer following environment 0 from the side: what results/week_04/figures/rope_pull.png "
                             "plots.")
    parser.add_argument("--real_time", action="store_true",
                        help="'play': hold each policy step to the policy's 20 ms, so a viewer shows real time.")
    parser.add_argument("--stochastic", action="store_true",
                        help="'rope_eval': sample actions with the checkpoint's own noise, as in training, instead of "
                             "the mean. A diagnosis, never the frozen measurement.")
    parser.add_argument("--ground_friction", type=float, default=None,
                        help="Static and dynamic friction of the ground and the default material (UniFP's is 1.0). "
                             "A sensitivity condition: say so when quoting a result made with it.")
    parser.add_argument("--button_fraction", type=float, default=None,
                        help="'press' only: fraction of episodes with a button (default 0.8).")
    parser.add_argument("--press_level", type=float, default=None,
                        help="'press' training: the curriculum's starting level, 0-1 (default 0.2).")
    parser.add_argument("--press_tools", nargs="+", choices=("bare", "probe"), default=None,
                        help="'press' training: which tools to draw (default both, equally often).")
    parser.add_argument("--press_hold_loop", choices=("hybrid", "full", "off"), default="hybrid",
                        help="--leg_object press: the layer's outer loop on the foot's miss while the plate is pressed "
                             "(unifp_train/foot_press_env.py): 'hybrid' across the normal and a commanded tangential force; "
                             "'full' across the normal, as rounds 1-4 of the foot-press task trained; 'off'.")
    parser.add_argument("--press_plate_face_only", action="store_true",
                        help="--leg_object press: the plate as its face only, as trained until 2026-10-22 10:55 (the default "
                             "is the slab with edges and a back).")
    parser.add_argument("--press_release_from_press_point", action="store_true",
                        help="--leg_object press: release from the press point wherever the foot is (as the layer did until "
                             "2026-10-22 10:45; the default backs straight out from the foot).")
    parser.add_argument("--press_up_cap_n", type=float, default=None,
                        help="--leg_object press, training: cap the commanded force's upward component (a plate facing "
                             "down) at this, N (F-182). Off by default.")
    parser.add_argument("--press_orientation_weight", type=float, default=None,
                        help="--leg_object press: the weight of UniFP's orientation term (default foot_press_cfg's -4).")
    parser.add_argument("--tilt_excess_weight", type=float, default=0.0,
                        help="--leg_object press: weight of the tilt past --tilt_excess_deg (rad, a hinge). 0: off.")
    parser.add_argument("--tilt_excess_deg", type=float, default=None,
                        help="--tilt_excess_weight's threshold, degrees from upright (default leg_cfg's 10).")
    parser.add_argument("--tilt_stages", choices=("all", "standing"), default="all",
                        help="--leg_object press: where --press_orientation_weight and --tilt_excess_weight apply: every "
                             "step, or only while a plate episode stands (waiting, or the foot back down), the -4 kept "
                             "everywhere.")
    parser.add_argument("--start_delay_steps", type=int, nargs=2, default=None, metavar=("LO", "HI"),
                        help="--task leg: policy steps a plate episode waits on its foot-down command before the "
                             "approach, drawn in [LO, HI) in training (15 50).")
    parser.add_argument("--eval_start_delay_steps", type=int, default=None,
                        help="--task leg: the same, fixed, in a frozen evaluation (25); e.g. 500 for 10 s of standing.")
    parser.add_argument("--press_hold_correction", action="store_true",
                        help="--leg_object press: the same as --press_hold_loop full (the name round 4's evaluations used).")
    parser.add_argument("--plate_set", choices=("v2", "slide", "dog"), default="v2",
                        help="leg_eval --leg_object press: the frozen plates (unifp_train/foot_press_eval.py): 'v2', 288 "
                             "presses; 'slide', the same 48 plates at 16 N with a tangential part or a 4 cm slide; 'dog', "
                             "the live program's two hands at 5 and 8 N, 288 presses.")
    parser.add_argument("--press_set", choices=("panel", "force", "recessed"), default="panel",
                        help="'press_eval': the frozen buttons to score (press_cfg.EVAL_*).")
    parser.add_argument("--wipe_fraction", type=float, default=None,
                        help="'wipe' only: fraction of episodes with a surface to wipe (default 0.8).")
    parser.add_argument("--wipe_level", type=float, default=None,
                        help="'wipe' training: the curriculum's starting level, 0-1 (default 0.2).")
    parser.add_argument("--insertion_fraction", type=float, default=None,
                        help="'insertion' only: fraction of episodes holding a key to a slot (default 0.8).")
    parser.add_argument("--ins_level", type=float, default=None,
                        help="'insertion' training: the curriculum's starting level, 0-1 (default 0.2).")
    parser.add_argument("--ins_debug", type=int, default=None, metavar="ENV",
                        help="'insertion' smoke/play: print one environment's task layer every 10 steps.")
    parser.add_argument("--ins_set", choices=("lateral", "detent"), default="lateral",
                        help="'ins_eval': the frozen slots to score (insert_cfg.EVAL_*).")
    parser.add_argument("--ins_tip_gain", type=float, default=None,
                        help="'insertion': the task layer's tip loop gain, 1/s (insertion.PoseCorrection; default "
                             "insert_cfg.TIP_CORRECTION_GAIN). 0 turns it off, as for the runs before v6.")
    parser.add_argument("--ins_tip_max", type=float, default=None,
                        help="'insertion': the tip loop's clamp, m (default insert_cfg.TIP_CORRECTION_MAX_M, 0.05).")
    parser.add_argument("--ins_elevation_max_deg", type=float, default=None,
                        help="'insertion': the elevation loop's clamp, degrees (default 20).")
    parser.add_argument("--ins_roll_max_deg", type=float, default=None,
                        help="'insertion': the roll loop's clamp, degrees (default 20).")
    parser.add_argument("--ins_promote_every", type=int, default=None,
                        help="'insertion' training: the fewest iterations between curriculum promotions "
                             "(default insert_cfg.CURRICULUM_MIN_ITERATIONS, 50).")
    parser.add_argument("--ins_engage_attitude_deg", type=float, default=None,
                        help="'insertion': the layer also waits for the approach's elevation and the roll to be "
                             "within this of their targets, degrees (default: the tip alone).")
    parser.add_argument("--ins_approach_weight", type=float, default=None,
                        help="'insertion': the approach term's weight per second (default task_cfg.APPROACH_WEIGHTS, 2.5).")
    parser.add_argument("--ins_true_targets", action="store_true",
                        help="'insertion': score the policy's reach and roll against the task's targets, not the "
                             "commands the outer loops shift (insert_env.reward_true_targets).")
    parser.add_argument("--ins_no_hold", action="store_true",
                        help="'insertion': the task layer keeps driving a seated key instead of holding it home "
                             "(the layer before 2026-09-27).")
    parser.add_argument("--ins_compliant_grip", action="store_true",
                        help="'insertion': hold the key in a compliant grip (insertion.CompliantGrip) rather than rigidly.")
    parser.add_argument("--ins_attitude_gain", type=float, default=None,
                        help="'insertion': the approach-elevation and roll loops' gain, 1/s (default insert_cfg's).")
    parser.add_argument("--mechanism_fraction", type=float, default=None,
                        help="'mechanism' only: fraction of episodes holding a mechanism (default 0.8).")
    parser.add_argument("--peak_ceiling", type=float, default=None,
                        help="'mechanism' only: starting ceiling on the mechanisms' peak resistance, N "
                             "(default 15).")
    parser.add_argument("--peak_ceiling_max", type=float, default=None,
                        help="'mechanism' only: where the curriculum stops raising it, N (default 80).")
    parser.add_argument("--mech_kinds", nargs="+", default=None,
                        choices=("drawer", "latch", "door", "button", "lid", "bolt",
                                 "lever", "lever_spindle_side", "lever_tip_side"),
                        help="'mech_eval': which fixed mechanisms (default the four the comparisons use; "
                             "'lid' and 'bolt' are held-out geometries; the three levers are a combiner box's "
                             "handle square and from either side, scored at mech_cfg.LEVER_EVAL_LEVELS_N unless "
                             "--mech_levels says otherwise).")
    parser.add_argument("--force_law", action="store_true",
                        help="'mechanism': the hierarchical variant -- hold the goal on the handle and "
                             "command a force from the handle's lag behind its reference "
                             "(mech_cfg.FORCE_LAW_*), for a policy trained to follow force commands. In "
                             "'mech_eval' it is the baseline; in 'train' and 'smoke' the law runs in the loop.")
    parser.add_argument("--law_variant", choices=("v1", "v2", "v3"), default="v1",
                        help="'mechanism' training with --force_law: 'v2' adds the over-force limit, a heavier lunge "
                             "price, the integral bleed and a cap drawn per episode (mech_cfg.LAW_V2_*); 'v3' "
                             "drops the outcome reward, a pure force-follower (mech_cfg.LAW_V3_WEIGHT_CHANGES).")
    parser.add_argument("--force_law_bleed", type=float, default=None, metavar="TAU_S",
                        help="'--force_law': let the law's integral decay with this time constant once the handle is "
                             "within mech_cfg.FORCE_LAW_ARRIVED_M of its reference (default: no bleed, as trained).")
    parser.add_argument("--lever_fraction", type=float, default=None,
                        help="'mechanism' training: fraction of mechanisms that are short levers turned down about a "
                             "spindle pointing at the robot (mech_cfg.LEVER_*; default 0).")
    parser.add_argument("--offpath_weight", type=float, default=None,
                        help="'mechanism' training: price on force across the mechanism's path (mech_rewards.mech_offpath, "
                             "per (20 N)^2; default 0). A claw lets go under it; the training's ball-joint grasp does not.")
    parser.add_argument("--lever_high", action="store_true",
                        help="'mechanism' training: draw lever handles up to mech_cfg.LEVER_HANDLE_PITCH_RANGE (~0.96 m), "
                             "above the robot's shoulder. 'mech_eval': score the high-handle lever placements "
                             "(mech_cfg.LEVER_HIGH_*, 0.6-0.9 m) instead of the development grid.")
    parser.add_argument("--opening_probs", type=float, nargs=4, default=None, metavar=("PULL", "PUSH", "SIDE", "UP"),
                        help="'mechanism' training: how often mechanisms open each way (default mech_cfg.OPENING_PROBS).")
    parser.add_argument("--force_law_gains", type=float, nargs=3, default=None, metavar=("KP", "KI", "MAX"),
                        help="'mech_eval --force_law': override the law's gains (N/m, N/(m*s), N).")
    parser.add_argument("--mech_test", action="store_true",
                        help="'mech_eval': the held-out test condition (mech_cfg.TEST_*: other placements, mass, "
                             "grasp stiffness, reference speed and damping), frozen before any policy was scored on it.")
    parser.add_argument("--mech_levels", type=float, nargs="+", default=None,
                        help="'mech_eval': peak resistances, N (default mech_cfg.EVAL_LEVELS_N, 10-80).")
    parser.add_argument("--fixture_fraction", type=float, default=None,
                        help="'hook' only: fraction of episodes with a fixture (hook_cfg default 0.75).")
    parser.add_argument("--press_fraction", type=float, default=None,
                        help="'hook' only: of those, the fraction that press rather than pull (default 0).")
    parser.add_argument("--force_ceiling", type=float, default=None,
                        help="'hook' only: starting ceiling of the commanded force, N (default 15).")
    parser.add_argument("--force_ceiling_max", type=float, default=None,
                        help="'hook' only: where the curriculum stops raising it, N (default 60).")
    parser.add_argument("--leg_variant", choices=("position", "force"), default="force",
                        help="--task leg: 'position' is the plan's P0 (no force command, the push a disturbance to "
                             "reject, no force in the critic or the estimator's target); 'force' is P1, UniFP's "
                             "formulation at the foot (unifp_train/leg_env.py).")
    parser.add_argument("--leg_object", choices=("handle", "key", "press"), default="handle",
                        help="--task leg: 'handle' is a lever door handle at body height (2026-10-19); 'key' is the "
                             "combiner box's lever key in its cam lock, turned 90 degrees down (2026-10-21), which also "
                             "starts each episode standing settled, holds the arm folded at rest and rewards the support "
                             "feet for their settled places (unifp_train/leg_cfg.py, 'the combiner box's key'); 'press' is "
                             "the key's stance with plates anywhere the foot reaches, facing any way it can push, pressed "
                             "with a commanded force profile (unifp_train/foot_press*.py; --handle_fraction is the share "
                             "of plate episodes).")
    parser.add_argument("--leg_actuator", choices=("unifp", "dcmotor", "knee23", "speedcurve", "knee23curve",
                                                        "dcmotor_armature", "unitree", "unitree_geared"),
                        default="unifp",
                        help="--task leg, a diagnostic of the legs' motor model (unifp_train/leg_actuators.py), the 25 / 0.5 "
                             "gains in all: 'unifp' is UniFP's ideal PD the leg policies trained on (knees 45.43 N·m); "
                             "'dcmotor' Isaac Lab's Go2 DCMotor (23.5 N·m on every joint, its torque-speed curve), as the "
                             "decoupled demos' robot; 'knee23' and 'speedcurve' change one of those differences each; "
                             "'unitree' is unitree_rl_lab's Go2 envelope; 'unitree_geared' that with the URDF's geared knee.")
    parser.add_argument("--key_turn_deg", type=float, nargs=3, default=None, metavar=("START", "STOP", "UNLOCK"),
                        help="--leg_object key: the turn in degrees below horizontal (positive down, clockwise as the "
                             "robot sees it): where the key starts, its end stop, where the door unlocks. The trained "
                             "turn is -45 45 30 (leg_cfg.KEY_THETA_*); e.g. 45 90 85 starts it 45 down and needs it "
                             "turned straight down. Applies to the frozen key set and to training.")
    parser.add_argument("--key_overdrive_deg", type=float, default=None,
                        help="--leg_object key: how far past the end stop the task layer's path ends (degrees; "
                             "leg_cfg.OVERDRIVE_RAD, 6 as trained); negative ends it short of the stop.")
    parser.add_argument("--key_hold_s", type=float, default=None,
                        help="--leg_object key: how long the task layer holds the foot on the key at the end of the turn "
                             "(leg_cfg.HOLD_S, 2 s as trained; a cam lock stays where it is turned).")
    parser.add_argument("--key_eval_lock_nm", type=float, nargs="+", default=None,
                        help="leg_eval --leg_object key: the frozen set's lock frictions, N·m (leg_cfg.KEY_EVAL_FRICTION_NM, "
                             "0.5 1.5 2.5); with --key_eval_detent_nm its detents (KEY_EVAL_DETENT_NM, 0 1). The set keeps "
                             "its 108 rows when both keep their counts (3 and 2).")
    parser.add_argument("--key_eval_detent_nm", type=float, nargs="+", default=None)
    parser.add_argument("--key_max_force_n", type=float, nargs=2, default=None, metavar=("LO", "HI"),
                        help="--leg_object key: the force-aware layer's command cap's limits, N (leg_cfg.KEY_MAX_N_LIMITS, "
                             "15 55; the cap is 1.8x the lock's rating within them).")
    parser.add_argument("--key_lock_range_nm", type=float, nargs=2, default=None, metavar=("LO", "HI"),
                        help="--leg_object key, training: the lock friction drawn per episode, N·m (leg_cfg.KEY_FRICTION_NM, "
                             "0.3 2.5); --key_detent_range_nm its cam detent (KEY_DETENT_NM, 0 1).")
    parser.add_argument("--key_detent_range_nm", type=float, nargs=2, default=None, metavar=("LO", "HI"))
    parser.add_argument("--key_damage_n", type=float, default=None,
                        help="--task leg: the contact force counted as damage, N (leg_cfg.DAMAGE_N, 120); a frozen "
                             "evaluation ends an episode there.")
    parser.add_argument("--key_press_limit_n", type=float, default=None,
                        help="--task leg: the force past which contact is penalised (with --key_press_weight, per cm of "
                             "the overlap it takes through the plant's stiffness; both variants).")
    parser.add_argument("--key_press_weight", type=float, default=0.0)
    parser.add_argument("--leg_delay_min_steps", type=int, default=0,
                        help="--task leg: the least command delay, physics steps of 5 ms (with --leg_delay_steps as the most).")
    parser.add_argument("--leg_armature_range", type=float, nargs=2, default=None, metavar=("LO", "HI"),
                        help="--task leg: draw each environment's leg armature once, uniform in this range (kg·m², hip and "
                             "thigh), the knees --leg_knee_armature_factor times as much.")
    parser.add_argument("--leg_knee_armature_factor", type=float, default=3.67)
    parser.add_argument("--leg_friction_range", type=float, nargs=2, default=None, metavar=("LO", "HI"),
                        help="--task leg: draw each environment's leg joint friction (PhysX static and dynamic effort, "
                             "N·m, hip and thigh) uniform in this range, the knees --leg_knee_friction_factor times as "
                             "much. LO = HI fixes it.")
    parser.add_argument("--leg_knee_friction_factor", type=float, default=1.917)
    parser.add_argument("--spawn_states_fraction", type=float, default=None,
                        help="--spawn_states, training: the share of episodes started from a recorded stance (0.5).")
    parser.add_argument("--leg_armature", type=float, default=None,
                        help="--task leg: the legs' armature, kg·m² (trained with 2e-4; unitree_rl_lab's GO2HV 0.01), in any "
                             "--leg_actuator; --leg_knee_armature sets the knees' apart.")
    parser.add_argument("--leg_knee_armature", type=float, default=None)
    parser.add_argument("--leg_delay_steps", type=int, default=0,
                        help="--task leg: delay the legs' commands by 0 to this many physics steps (5 ms), per environment "
                             "(the 'unifp', 'knee23', 'unitree' and 'unitree_geared' leg actuators).")
    parser.add_argument("--spawn_states", default="",
                        help="--task leg --leg_object key: a json of stances to start half the episodes from (the walking "
                             "policy's stops, demos/combiner_key/assets/walker_stances.json), for a handover from it.")
    parser.add_argument("--eval_spawn_states", action="store_true",
                        help="leg_eval with --spawn_states: start every frozen episode from one of the stances (a "
                             "diagnostic for the handover from the walking policy), the stance drawn per episode.")
    parser.add_argument("--handle_fraction", type=float, default=None,
                        help="--task leg: share of training episodes with a door handle ahead (default 0: free-space "
                             "foot goals with pushes only).")
    parser.add_argument("--force_ramp_iterations", type=int, default=None,
                        help="--task leg: iterations over which the foot pushes ramp up once the force curriculum opens.")
    parser.add_argument("--leg_frames", type=int, default=0,
                        help="leg_eval: save the wrist camera's and a side view's frames for the first N episodes "
                             "(needs --enable_cameras).")
    parser.add_argument("--leg_frame_every", type=int, default=5, help="leg_eval: policy steps between saved frames.")
    parser.add_argument("--leg_shadow_dump", type=int, default=0,
                        help="leg_eval: record the first N episodes' raw robot state, commands, frames, actions and joint "
                             "targets every step (shadow_dump.npz), to check the dog's numpy policy against "
                             "(demos/combiner_key/robot/check_shadow.py --sim).")
    parser.add_argument("--leg_rows", type=int, nargs="+", default=None,
                        help="leg_eval: score only these rows of the frozen handle set (a subset for frames).")
    parser.add_argument("--foot_push_n", type=float, default=None,
                        help="--task leg: peak per-axis force of the physical push on the foot, N (default 35; the "
                             "command channel keeps 35).")
    parser.add_argument("--leg_bar_radius", type=float, default=None,
                        help="leg_eval: give every frozen handle this bar radius, m (a diagnostic, not the frozen set: "
                             "how much of a failure is the foot's precision on a 2 cm round bar).")
    parser.add_argument("--small_physx", action="store_true",
                        help="Shrink PhysX's GPU contact and pair buffers (ample for a few hundred environments), so an "
                             "evaluation fits beside training runs on the shared GPU. Not for training at 4096.")
    parser.add_argument("--no_arm_jitter", action="store_true",
                        help="--task leg: hold the arm exactly at its look pose rather than jittering it per episode.")
    parser.add_argument("--no_curriculum", action="store_true",
                        help="'hook' and 'mechanism': hold the curriculum's ceiling where it starts.")
    parser.add_argument("--repeats", type=int, default=4,
                        help="'hook_eval': episodes per handle placement (15 placements).")
    parser.add_argument("--press", action="store_true",
                        help="'hook_eval': evaluate pressing a button instead of pulling a handle.")
    parser.add_argument("--eval_levels", type=float, nargs="+", default=None,
                        help="'hook_eval': the force staircase, N (default hook_cfg.EVAL_LEVELS_N, 10-60). "
                             "Longer staircases lengthen the episode to fit.")
    parser.add_argument("--fixture_kind", choices=("ring", "bar"), default="ring",
                        help="'hook_eval': pull on a ring (force transmission alone) or a horizontal bar "
                             "(the claw must also stay on it).")
    parser.add_argument("--checkpoint", default=os.path.expanduser(
        "~/thesis_b_legacy/UniFP/logs/go2d1_pos_force/Sep18_11-10-15_/model_48800.pt"),
        help="UniFP checkpoint for 'play'.")
    parser.add_argument("--num_envs", type=int, default=16)
    parser.add_argument("--steps", type=int, default=100,
                        help="Policy steps at 50 Hz, for 'smoke' and 'play'.")
    parser.add_argument("--iterations", type=int, default=60000,
                        help="Policy updates, for 'train'. Upstream's config asks for 60000.")
    parser.add_argument("--seed", type=int, default=1, help="UniFP's config seed is 1.")
    parser.add_argument("--robot_usd", help="Existing welded USD; otherwise it is rebuilt.")
    parser.add_argument("--payload", choices=("physical", "visual"), default="physical",
                        help="Bolted-on parts (payload.py): 'physical' (default) gives the MID-360, its mount, the wrist "
                             "RealSense and its bracket colliders and mass, as the real robot carries them (since "
                             "2026-10-19); 'visual' only draws the lidar, the robot every checkpoint before then was "
                             "trained on. Use 'visual' to re-score those checkpoints as they were.")
    parser.add_argument("--mount_pos", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                        help="Weld the D1 at this offset from the Go2's base link, m (default mount.ARM_MOUNT_POS_M, the "
                             "measured mount, F-117). For measuring how much a policy depends on where the arm is.")
    parser.add_argument("--output", default=str(ROOT / "logs/unifp_train"),
                        help="Where 'train' writes its run directory.")
    parser.add_argument("--run_name", default="", help="Appended to the run directory's name.")
    parser.add_argument("--roll_objective", action="store_true",
                        help="Command and reward the gripper's roll about its approach axis "
                             "(F-095, F-099). Adds one reward term and starts writing a command "
                             "channel the observation already carried, so no width changes and "
                             "every existing checkpoint still loads.")
    parser.add_argument("--tool_point", choices=("jaw_centre", "fingertip"), default="jaw_centre",
                        help="Which point the task controls. 'jaw_centre' is Link6 + the jaw "
                             "centre: invariant to jaw travel and on the roll axis (F-094, F-096). "
                             "'fingertip' is the Link7_1 point the released checkpoints were "
                             "trained against -- use it to reproduce their numbers.")
    parser.add_argument("--force_start_iteration", type=int, default=None,
                        help="Override the force curriculum. UniFP's default is 8000 iterations "
                             "of position-only training first; 0 applies forces immediately.")
    parser.add_argument("--resume_from", default=None,
                        help="Checkpoint to resume 'train' from, written by this script.")
    parser.add_argument("--manifest", default=None,
                        help="Frozen evaluation set, for 'eval'.")
    parser.add_argument("--manifest_out", default=None,
                        help="Where 'manifest' writes. Defaults under results/manifests/.")
    parser.add_argument("--role", default="validation", choices=("development", "validation", "test"),
                        help="Which frozen set 'manifest' is building.")
    parser.add_argument("--hold_command", nargs=3, type=float, metavar=("VX", "VY", "WZ"),
                        default=None,
                        help="For 'eval': hold one base velocity command for the whole episode "
                             "instead of sampling it. This deliberately overrides the frozen "
                             "schedule, so the digests will not match and the run is a "
                             "diagnostic rather than a manifest result.")
    parser.add_argument("--zero_actions", action="store_true",
                        help="Evaluate the do-nothing baseline instead of a checkpoint.")
    # Knobs for diagnosing the training instability of 2026-09-21. Each defaults to UniFP's own
    # value, so leaving them alone keeps the faithful port; naming one is a deliberate departure
    # and is recorded in the run's agent.json.
    parser.add_argument("--std_type", choices=("scalar", "log"), default=None,
                        help="Action-noise parameterisation. UniFP uses 'scalar'; 'log' makes the "
                             "updates multiplicative, so the noise cannot be driven to a value "
                             "where the KL divergence blows up.")
    parser.add_argument("--init_std", type=float, default=None,
                        help="Initial action noise. UniFP's is 1.0.")
    parser.add_argument("--estimator_learning_rate", type=float, default=None,
                        help="Rate for the adaptation module's own optimizer, which shares the "
                             "encoder with the actor. UniFP's is 1e-5 and fixed, so as PPO's "
                             "adaptive rate falls the estimator's share of the encoder rises. "
                             "0 disables it, which is a diagnosis rather than a configuration.")
    parser.add_argument("--diagnose_storage", action="store_true",
                        help="Before each update, check that the policy the update sees is the "
                             "one that produced the rollout, and print where it is not. The "
                             "cheap standing version of this check is `Loss/kl_first_minibatch`, "
                             "logged on every run; use this when that is not ~0.")
    parser.add_argument("--fixed_learning_rate", type=float, default=None,
                        help="Hold PPO's learning rate at this value, ignoring the adaptive "
                             "schedule. 0 freezes the policy entirely, which is what makes it a "
                             "null test: any KL it still reports is not produced by the "
                             "optimizer. A departure from UniFP; recorded as one.")
    parser.add_argument("--learning_rate_scale", type=float, default=None,
                        help="Multiply the optimizer's effective step by this. rsl-rl clamps the "
                             "adaptive rate to [1e-5, 1e-2] and the floor binds on this task, so "
                             "the schedule stops regulating; 0.25 moves the range down without "
                             "changing the schedule.")
    parser.add_argument("--resume_arm_std", type=float, default=None,
                        help="After --resume_from, set the arm's six action-noise std entries to this (action "
                             "units; x 0.25 rad). Checkpoints from the pull lineage carry 0.7-0.9 there -- ~0.2 rad "
                             "of random arm target every step -- which no millimetre task can succeed under in "
                             "training (the insertion, 2026-09-27). The legs keep theirs.")
    parser.add_argument("--resume_leg_std", type=float, default=None,
                        help="On a resume, set the twelve leg actions' noise std to this, as --resume_arm_std does for the "
                             "arm. The rope task's deterministic policy pulled 27%% over its command while the noisy one "
                             "tracked to 11-15%% (2026-10-07): noise the policy learns to compensate for.")
    parser.add_argument("--entropy_coef", type=float, default=None,
                        help="Entropy bonus. UniFP's is 0.01, and it is the term pushing the "
                             "action noise back up once the learning rate is floored.")
    return parser


def git_output(*args: str) -> str:
    try:
        return subprocess.run(("git", *args), cwd=ROOT, capture_output=True,
                              text=True, check=True).stdout.strip()
    except Exception:
        return ""


def stack_versions() -> dict:
    """What actually got imported, so a run record says which stack produced it."""
    from importlib.metadata import PackageNotFoundError, version

    out = {}
    for package in ("isaacsim", "isaaclab", "isaaclab-rl", "rsl-rl-lib", "torch"):
        try:
            out[package] = version(package)
        except PackageNotFoundError:
            out[package] = None
    return out


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def main() -> int:
    parser = build_parser()
    if len(sys.argv) == 1 or "-h" in sys.argv or "--help" in sys.argv:
        parser.parse_args()
        return 0

    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    sys.path.insert(0, str(ROOT))
    from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR  # noqa: E402
    from weld import build_welded_robot_usd  # noqa: E402
    from unifp_isaaclab import interface, robot as robot_mod  # noqa: E402
    from unifp_train import observations, task_cfg  # noqa: E402
    from unifp_train.env import Go2D1PosForceEnv  # noqa: E402
    from unifp_train.env_cfg import Go2D1PosForceEnvCfg  # noqa: E402
    from unifp_train import hook_cfg  # noqa: E402
    from unifp_train.hook_env import Go2D1HookEnv, Go2D1HookEnvCfg  # noqa: E402
    from unifp_train import mech_cfg  # noqa: E402
    from unifp_train.mech_env import Go2D1MechEnv, Go2D1MechEnvCfg  # noqa: E402
    from unifp_train import insert_cfg  # noqa: E402
    from unifp_train.insert_env import Go2D1InsertEnv, Go2D1InsertEnvCfg  # noqa: E402
    from unifp_train import press_cfg, wipe_cfg  # noqa: E402
    from unifp_train.press_env import Go2D1PressEnv, Go2D1PressEnvCfg  # noqa: E402
    from unifp_train.wipe_env import Go2D1WipeEnv, Go2D1WipeEnvCfg  # noqa: E402
    from unifp_train import rope_cfg  # noqa: E402
    from unifp_train.rope_env import Go2D1RopeEnv, Go2D1RopeEnvCfg  # noqa: E402
    from unifp_train import leg_actuators, leg_cfg  # noqa: E402
    from unifp_train.leg_env import Go2D1LegEnv, Go2D1LegEnvCfg  # noqa: E402
    from unifp_train import foot_press_cfg  # noqa: E402
    from unifp_train.foot_press_env import Go2D1FootPressEnv  # noqa: E402

    if args.mode == "hook_eval" and args.task != "hook":
        parser.error("hook_eval needs --task hook")
    if args.mode == "mech_eval" and args.task != "mechanism":
        parser.error("mech_eval needs --task mechanism")
    if args.mode == "ins_eval" and args.task != "insertion":
        parser.error("ins_eval needs --task insertion")
    if args.mode == "press_eval" and args.task != "press":
        parser.error("press_eval needs --task press")
    if args.mode == "wipe_eval" and args.task != "wipe":
        parser.error("wipe_eval needs --task wipe")
    if args.mode == "rope_eval" and args.task != "rope":
        parser.error("rope_eval needs --task rope")
    if args.mode == "leg_handover" and (args.task != "leg" or args.leg_object != "press"):
        parser.error("leg_handover needs --task leg --leg_object press")
    if args.mode == "leg_eval" and args.task != "leg":
        parser.error("leg_eval needs --task leg")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    suffix = f"_{args.run_name}" if args.run_name else ""
    run_dir = Path(args.output).resolve() / f"{stamp}_{args.mode}_seed{args.seed}{suffix}"

    payload_record = None
    if args.robot_usd:
        robot_usd = str(Path(args.robot_usd).resolve())
    else:
        from mount import ARM_MOUNT_POS_M
        mount_pos = tuple(args.mount_pos) if args.mount_pos is not None else ARM_MOUNT_POS_M
        # Its own file with the payload's physics, so a demo rebuilding `generated/go2_d1.usd` never swaps it.
        weld = build_welded_robot_usd(
            go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd",
            d1_urdf_path=str(ROOT / "d1_arm/d1.urdf"),
            out_usd_path=str(ROOT / ("generated/go2_d1_payload.usd" if args.payload == "physical"
                                     else "generated/go2_d1.usd")),
            mount_pos=mount_pos, arm_mass_kg=3.152, payload_physics=args.payload == "physical",
        )
        robot_usd, payload_record = weld.usd_path, weld.payload

    cfg = {"hook": Go2D1HookEnvCfg, "mechanism": Go2D1MechEnvCfg, "insertion": Go2D1InsertEnvCfg,
           "press": Go2D1PressEnvCfg, "wipe": Go2D1WipeEnvCfg, "rope": Go2D1RopeEnvCfg,
           "leg": Go2D1LegEnvCfg}.get(args.task, Go2D1PosForceEnvCfg)()
    if args.task == "leg":
        cfg.leg_variant = args.leg_variant
        if args.leg_object in ("key", "press"):
            cfg.leg_object, cfg.arm_pose, cfg.settled_spawn = args.leg_object, "rest", True
            if args.key_turn_deg is not None:
                # Read at run time by the plant, the task layer, the approach and the frozen plan (`c.KEY_THETA_*`).
                start, stop, unlock = args.key_turn_deg
                if not start < unlock <= stop:
                    parser.error("--key_turn_deg needs START < UNLOCK <= STOP")
                leg_cfg.KEY_THETA_START_RAD, leg_cfg.KEY_THETA_END_RAD = math.radians(start), math.radians(stop)
                leg_cfg.KEY_THETA_UNLOCK_RAD = math.radians(unlock)
            if args.key_overdrive_deg is not None:
                leg_cfg.OVERDRIVE_RAD = math.radians(args.key_overdrive_deg)
            if args.key_hold_s is not None:
                leg_cfg.HOLD_S = args.key_hold_s
            if args.key_eval_lock_nm is not None:
                leg_cfg.KEY_EVAL_FRICTION_NM = tuple(args.key_eval_lock_nm)
            if args.key_eval_detent_nm is not None:
                leg_cfg.KEY_EVAL_DETENT_NM = tuple(args.key_eval_detent_nm)
            if args.key_max_force_n is not None:
                leg_cfg.KEY_MAX_N_LIMITS = tuple(args.key_max_force_n)
            if args.key_lock_range_nm is not None:
                leg_cfg.KEY_FRICTION_NM = tuple(args.key_lock_range_nm)
            if args.key_detent_range_nm is not None:
                leg_cfg.KEY_DETENT_NM = tuple(args.key_detent_range_nm)
            cfg.stance_weight = leg_cfg.STANCE_WEIGHT
            cfg.press_hold_loop = "full" if args.press_hold_correction else args.press_hold_loop
            cfg.press_release_from_foot = not args.press_release_from_press_point
            cfg.press_plate_edges = not args.press_plate_face_only
            cfg.press_up_cap_n = args.press_up_cap_n
            cfg.press_orientation_weight = args.press_orientation_weight
            cfg.tilt_excess_weight = args.tilt_excess_weight
            cfg.tilt_stages = args.tilt_stages
            if args.tilt_excess_deg is not None:
                leg_cfg.TILT_EXCESS_RAD = math.radians(args.tilt_excess_deg)
            cfg.episode_length_s = (foot_press_cfg.EPISODE_LENGTH_S if args.leg_object == "press"
                                    else leg_cfg.KEY_EPISODE_LENGTH_S)
            if args.spawn_states:
                cfg.spawn_states = str(Path(args.spawn_states).resolve())
                if args.spawn_states_fraction is not None:
                    cfg.spawn_states_fraction = args.spawn_states_fraction
                if args.eval_spawn_states:
                    cfg.spawn_states_in_eval, cfg.spawn_states_fraction = True, 1.0
        if args.task == "leg":
            if args.key_damage_n is not None:
                leg_cfg.DAMAGE_N = args.key_damage_n
            if args.key_press_limit_n is not None:
                cfg.press_limit_n = args.key_press_limit_n
            cfg.press_excess_weight = args.key_press_weight
            if args.leg_armature_range is not None:
                cfg.leg_armature_range = tuple(args.leg_armature_range)
                cfg.knee_armature_factor = args.leg_knee_armature_factor
            if args.start_delay_steps is not None:
                cfg.start_delay_steps = tuple(args.start_delay_steps)
            if args.eval_start_delay_steps is not None:
                cfg.eval_start_delay_steps = args.eval_start_delay_steps
            if args.leg_friction_range is not None:
                cfg.leg_friction_range = tuple(args.leg_friction_range)
                cfg.knee_friction_factor = args.leg_knee_friction_factor
        if args.handle_fraction is not None:
            cfg.handle_fraction = args.handle_fraction
        if args.force_ramp_iterations is not None:
            cfg.force_ramp_iterations = args.force_ramp_iterations
        if args.no_arm_jitter:
            cfg.arm_jitter = False
        if args.foot_push_n is not None:
            cfg.foot_push_n = args.foot_push_n
    if args.task == "insertion":
        if args.insertion_fraction is not None:
            cfg.insertion_fraction = args.insertion_fraction
        if args.ins_level is not None:
            cfg.curriculum_level = args.ins_level
        if args.ins_tip_gain is not None:
            cfg.tip_correction_gain = args.ins_tip_gain
        if args.ins_attitude_gain is not None:
            cfg.elevation_correction_gain = cfg.roll_correction_gain = args.ins_attitude_gain
        if args.ins_compliant_grip:
            cfg.compliant_grip = True
        if args.ins_no_hold:
            cfg.hold_home = False
        if args.ins_true_targets:
            cfg.reward_true_targets = True
        if args.ins_engage_attitude_deg is not None:
            cfg.engage_attitude_rad = math.radians(args.ins_engage_attitude_deg)
        if args.ins_approach_weight is not None:
            cfg.approach_weight = args.ins_approach_weight
        if args.ins_promote_every is not None:
            cfg.curriculum_min_iterations = args.ins_promote_every
        if args.ins_tip_max is not None:
            cfg.tip_correction_max_m = args.ins_tip_max
        if args.ins_elevation_max_deg is not None:
            cfg.elevation_correction_max_rad = math.radians(args.ins_elevation_max_deg)
        if args.ins_roll_max_deg is not None:
            cfg.roll_correction_max_rad = math.radians(args.ins_roll_max_deg)
        if args.no_curriculum:
            cfg.curriculum = False
    if args.task == "press":
        if args.button_fraction is not None:
            cfg.button_fraction = args.button_fraction
        if args.press_level is not None:
            cfg.curriculum_level = args.press_level
        if args.press_tools:
            cfg.tools = tuple(args.press_tools)
        if args.no_curriculum:
            cfg.curriculum = False
    if args.task == "wipe":
        if args.wipe_fraction is not None:
            cfg.wipe_fraction = args.wipe_fraction
        if args.wipe_level is not None:
            cfg.curriculum_level = args.wipe_level
        if args.no_curriculum:
            cfg.curriculum = False
    # The frozen evaluations fix one episode per environment, so the plan sets the environment count.
    ins_rows = eval_plan = eval_rows = None
    if args.mode == "ins_eval":
        from unifp_train import insert_eval
        ins_plan, ins_rows = insert_eval.build_plan(args.ins_set, "cpu")
        eval_rows = ins_rows
        cfg.episode_length_s = insert_cfg.EVAL_EPISODE_S
    if args.mode == "press_eval":
        from unifp_train import press_eval
        eval_plan, eval_rows = press_eval.build_plan(args.press_set, "cpu")
        cfg.episode_length_s = press_cfg.EVAL_EPISODE_S
    if args.mode == "wipe_eval":
        from unifp_train import wipe_eval
        eval_plan, eval_rows = wipe_eval.build_plan("cpu")
        cfg.episode_length_s = wipe_cfg.EVAL_EPISODE_S
    # `play --task leg --leg_rows ...` loops chosen frozen handles in the viewer, one per environment.
    if args.mode == "leg_eval" or (args.mode == "play" and args.task == "leg" and args.leg_rows):
        from unifp_train import leg_eval
        if args.leg_object == "press":
            from unifp_train import foot_press_eval
            eval_plan, eval_rows = foot_press_eval.build_plan("cpu", args.plate_set)
        else:
            eval_plan, eval_rows = (leg_eval.build_key_plan if args.leg_object == "key" else leg_eval.build_plan)("cpu")
        # One episode per handle unless --repeats is given: the policy is deterministic, so repeats measure only PhysX.
        repeats = args.repeats if "--repeats" in sys.argv else 1
        if args.leg_bar_radius is not None:
            eval_plan["handles"]["radius"][:] = args.leg_bar_radius
            eval_rows = [dict(r, bar_radius=args.leg_bar_radius) for r in eval_rows]
        if args.leg_rows:
            keep = list(args.leg_rows)
            eval_rows = [dict(eval_rows[i], row=i) for i in keep]
            eval_plan = {"handles": {k: v[keep] for k, v in eval_plan["handles"].items()}}
        eval_rows = eval_rows * repeats
        eval_plan = {"handles": {k: v.repeat(repeats, *([1] * (v.dim() - 1))) for k, v in eval_plan["handles"].items()}}
        if args.leg_frames:
            cfg.wrist_camera = cfg.overview_camera = True
            cfg.unifp_markers = False
            if args.leg_object == "key":
                # The camera is on the head for the key: render that, not the folded arm's.
                cfg.head_camera_visual, cfg.head_camera, cfg.wrist_camera = True, True, False
        cfg.episode_length_s = {"key": leg_cfg.KEY_EVAL_EPISODE_S, "press": foot_press_cfg.EVAL_EPISODE_S}.get(
            args.leg_object, leg_cfg.EVAL_EPISODE_S)
    cfg.scene.num_envs = args.num_envs if eval_rows is None else len(eval_rows)
    if args.task == "mechanism":
        for name, value in (("mechanism_fraction", args.mechanism_fraction),
                            ("peak_ceiling_n", args.peak_ceiling),
                            ("peak_ceiling_max_n", args.peak_ceiling_max)):
            if value is not None:
                setattr(cfg, name, value)
        if args.no_curriculum:
            cfg.curriculum = False
        if args.opening_probs:
            total = sum(args.opening_probs)
            cfg.opening_probs = tuple(v / total for v in args.opening_probs)
        if args.lever_fraction is not None:
            cfg.lever_fraction = args.lever_fraction
        if args.offpath_weight is not None:
            cfg.offpath_weight = args.offpath_weight
        if args.lever_high and args.mode != "mech_eval":
            cfg.lever_handle_pitch_range = mech_cfg.LEVER_HANDLE_PITCH_RANGE
        if args.force_law and args.mode != "mech_eval":
            cfg.force_law = True
            if args.force_law_gains:
                cfg.force_law_gains = tuple(args.force_law_gains)
            cfg.force_law_bleed_s = args.force_law_bleed
            cfg.law_variant = args.law_variant
    if args.mode == "mech_eval":
        # One episode per environment: every kind at every level at every placement.
        mech_kinds = args.mech_kinds or list(mech_cfg.EVAL_DEFAULT_KINDS)
        levers_only = all(kind in mech_cfg.LEVER_EVAL_KINDS for kind in mech_kinds)
        mech_levels = args.mech_levels or list(mech_cfg.LEVER_EVAL_LEVELS_N if levers_only else mech_cfg.EVAL_LEVELS_N)
        mech_placements = (mech_cfg.test_handle_spheres() if args.mech_test
                           else mech_cfg.lever_high_handle_spheres() if args.lever_high else hook_cfg.eval_handle_spheres())
        cfg.scene.num_envs = len(mech_placements) * len(mech_levels) * len(mech_kinds)
        cfg.episode_length_s = mech_cfg.EVAL_EPISODE_S
    if args.task == "hook":
        for name, value in (("fixture_fraction", args.fixture_fraction),
                            ("press_fraction", args.press_fraction),
                            ("force_ceiling_n", args.force_ceiling),
                            ("force_ceiling_max_n", args.force_ceiling_max)):
            if value is not None:
                setattr(cfg, name, value)
        if args.no_curriculum:
            cfg.curriculum = False
    if args.task == "rope":
        for name, value in (("rope_fraction", args.rope_fraction), ("release_prob", args.release_prob),
                            ("detect_never_prob", args.detect_never_prob),
                            ("force_ceiling_n", args.force_ceiling), ("force_ceiling_max_n", args.force_ceiling_max)):
            if value is not None:
                setattr(cfg, name, value)
        if args.rope_stiffness_range is not None:
            cfg.stiffness_range = tuple(args.rope_stiffness_range)
        if args.rope_length_range is not None:
            cfg.length_range = tuple(args.rope_length_range)
        if args.rope_weight:
            cfg.weight_overrides = tuple((name, float(value)) for name, value in args.rope_weight)
        if args.curriculum_rule is not None:
            cfg.curriculum_rule = args.curriculum_rule
        if args.no_curriculum:
            cfg.curriculum = False
    rope_rows = None
    if args.mode == "play" and args.task == "rope" and args.rope_demo:
        # The figure's episode first (straight ahead, snap noticed), so the viewer, which follows environment 0, shows
        # it; then the other two releases at the same grip, then the same three at the grips 0.3 rad either side.
        grip = lambda r: round(rope_cfg.GOAL_CENTRE_HEIGHT_M + r["sphere"][0] * math.sin(r["sphere"][1]), 6)
        rope_rows = [r for r in rope_cfg.eval_plan() if grip(r) == 0.40 and r["stiffness"] == rope_cfg.EVAL_STIFFNESS[1]]
        rope_rows.sort(key=lambda r: (abs(r["sphere"][2]) > 1e-9, r["sphere"][2]))
        cfg.scene.num_envs = len(rope_rows)
        cfg.episode_length_s = 6.0 + rope_cfg.EVAL_HOLD_S * len(rope_cfg.EVAL_LEVELS_N) + rope_cfg.EVAL_RECOVERY_S + 1.0
    if args.mode == "rope_eval":
        rope_rows = rope_cfg.test_plan() if args.rope_set == "test" else rope_cfg.eval_plan()
        if args.rope_set == "test" and args.eval_levels is None:
            args.eval_levels = list(rope_cfg.TEST_LEVELS_N)
        if args.rope_stiffness:
            rope_rows = [dict(r, stiffness=k) for k in args.rope_stiffness
                         for r in rope_rows if r["stiffness"] == rope_cfg.EVAL_STIFFNESS[0]]
        rope_rows = rope_rows * args.repeats
        cfg.scene.num_envs = len(rope_rows)
        levels = args.eval_levels or list(rope_cfg.EVAL_LEVELS_N)
        # Room for the reach and engagement (~5 s), every level's hold, and the recovery.
        cfg.episode_length_s = 6.0 + rope_cfg.EVAL_HOLD_S * len(levels) + rope_cfg.EVAL_RECOVERY_S + 1.0
    if args.ground_friction is not None:
        import isaaclab.sim as sim_utils
        from isaaclab.assets import AssetBaseCfg

        material = dict(static_friction=args.ground_friction, dynamic_friction=args.ground_friction,
                        restitution=robot_mod.GROUND_RESTITUTION)
        cfg.sim.physics_material = sim_utils.RigidBodyMaterialCfg(**material)
        cfg.terrain = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg(
            physics_material=sim_utils.RigidBodyMaterialCfg(**material)))
    if args.mode == "hook_eval":
        # One episode per environment, long enough for the whole staircase; the placements fix
        # the environment count.
        cfg.scene.num_envs = len(hook_cfg.eval_handle_spheres()) * args.repeats
        levels = args.eval_levels or list(hook_cfg.EVAL_LEVELS_N)
        # Room for the reach and engagement (~5 s) and every level's hold.
        cfg.episode_length_s = max(hook_cfg.EVAL_EPISODE_S, 6.0 + hook_cfg.EVAL_HOLD_S * len(levels))
    if args.mode in ("manifest", "eval"):
        from unifp_train import eval as evaluation

        # The environment count is part of the frozen conditions: the schedule is drawn in
        # batches across environments, so the same seed with a different batch is a different
        # episode set. For 'eval' the manifest dictates it rather than the command line.
        if args.mode == "eval":
            manifest = evaluation.load(args.manifest)
            cfg.scene.num_envs = manifest["episode_count"]
            args.seed = manifest["seed"]
        # Evaluation runs with the force curriculum already open -- a policy that has never been
        # pushed is not what this task is for.
        cfg.force_start_step = 0
    cfg.seed = args.seed
    if args.small_physx:
        px = cfg.sim.physx
        px.gpu_max_rigid_contact_count, px.gpu_max_rigid_patch_count = 2**18, 2**16
        px.gpu_found_lost_pairs_capacity, px.gpu_found_lost_aggregate_pairs_capacity = 2**18, 2**20
        px.gpu_total_aggregate_pairs_capacity, px.gpu_collision_stack_size = 2**18, 2**25
        px.gpu_heap_capacity, px.gpu_temp_buffer_capacity = 2**25, 2**23
    # The hook and mechanism tasks have the roll objective on by default (the claw has to meet the
    # handle the right way up); UniFP's task has it off unless asked for.
    if args.task == "unifp":
        cfg.roll_objective = args.roll_objective
    if args.tool_point == "fingertip":
        cfg.tool_body, cfg.tool_offset_m = interface.TOOL_BODY, interface.TOOL_OFFSET_M
    if args.task in ("hook", "mechanism", "insertion", "press", "wipe", "rope", "leg"):
        # F-102: the arm's explicit PD chatters at UniFP's 2e-4 armature; see hook_cfg.ARM_ARMATURE_KG_M2.
        armature = {name: (hook_cfg.ARM_ARMATURE_KG_M2 if name in interface.ISAACLAB_NAMES[12:18]
                           else robot_mod.ARM_ARMATURE) for name in interface.ISAACLAB_NAMES}
        cfg.robot = robot_mod.make_robot_cfg(robot_usd, prim_path="/World/envs/env_.*/Robot", armature=armature)
        if args.task == "leg":
            leg_actuators.swap_legs(cfg.robot, args.leg_actuator, args.leg_delay_steps, args.leg_armature,
                                    args.leg_knee_armature, args.leg_delay_min_steps)
    else:
        cfg.robot = robot_mod.make_robot_cfg(robot_usd, prim_path="/World/envs/env_.*/Robot")
    cfg.robot_usd = robot_usd
    if args.force_start_iteration is not None:
        cfg.force_start_step = args.force_start_iteration * task_cfg.NUM_STEPS_PER_ENV
    if not args.headless:
        # Follow the robot rather than sit in the world frame: a walking quadruped leaves a fixed
        # camera within seconds. Offset from the base, slightly above and behind the right
        # shoulder, which keeps the arm and the front feet both in frame.
        cfg.viewer.origin_type = "asset_root"
        cfg.viewer.asset_name = "robot"
        cfg.viewer.env_index = 0
        cfg.viewer.eye = (1.6, -1.8, 0.9)
        cfg.viewer.lookat = (0.0, 0.0, 0.3)
        if args.task == "leg":
            # Fixed on environment 0 (the robot stands), from its front right onto the foot, the door and the handle; only
            # the handle's markers, UniFP's goal spheres hiding the foot.
            cfg.viewer.origin_type = "env"
            cfg.viewer.eye = (1.25, -1.45, 0.85)
            cfg.viewer.lookat = (0.35, -0.08, 0.28)
            cfg.unifp_markers = False
            if args.leg_object == "key":
                # The key is higher (0.42 m) and nearer; the head RealSense is drawn where the key task has it.
                cfg.viewer.eye = (1.15, -1.25, 0.95)
                cfg.viewer.lookat = (0.38, -0.10, 0.40)
                cfg.head_camera_visual = True
            if args.leg_object == "press":
                # From the front right, the foot's whole workspace in view.
                cfg.viewer.eye = (1.25, -1.35, 0.85)
                cfg.viewer.lookat = (0.35, -0.15, 0.25)
                cfg.head_camera_visual = True
        if rope_rows is not None:
            # From the side, back far enough to hold the robot, the rope and its post 1.5 m ahead: the lean shows.
            cfg.viewer.eye = (0.75, -2.9, 0.75)
            cfg.viewer.lookat = (0.75, 0.0, 0.3)

    env = {"hook": Go2D1HookEnv, "mechanism": Go2D1MechEnv, "insertion": Go2D1InsertEnv,
           "press": Go2D1PressEnv, "wipe": Go2D1WipeEnv, "rope": Go2D1RopeEnv,
           "leg": Go2D1FootPressEnv if args.leg_object == "press" else Go2D1LegEnv}.get(args.task, Go2D1PosForceEnv)(cfg)
    if args.mode == "ins_eval":
        env.set_evaluation(ins_plan)
    if args.mode in ("press_eval", "wipe_eval", "leg_eval") or (args.mode == "play" and args.task == "leg" and args.leg_rows):
        env.set_evaluation(eval_plan)
    if args.mode == "mech_eval":
        from unifp_train import mech_eval

        env.set_evaluation(*mech_eval.build_plan(mech_kinds, mech_levels, env.device, mech_placements),
                           conditions=mech_cfg.TEST_CONDITIONS if args.mech_test else mech_cfg.DEV_CONDITIONS)
        if args.force_law:
            env.set_force_law(*(args.force_law_gains or (mech_cfg.FORCE_LAW_KP_N_PER_M, mech_cfg.FORCE_LAW_KI_N_PER_M_S,
                                                         mech_cfg.FORCE_LAW_MAX_N)), bleed_s=args.force_law_bleed)
    if args.mode == "rope_eval" or (args.mode == "play" and rope_rows is not None):
        env.set_evaluation(rope_rows, levels=args.eval_levels or rope_cfg.EVAL_LEVELS_N,
                           hold_steps=round(rope_cfg.EVAL_HOLD_S / interface.POLICY_DT))
        if args.rope_ramp is not None:
            env._levels.ramp_per_step = args.rope_ramp * interface.POLICY_DT
    if args.mode == "hook_eval":
        from unifp_train import hook_eval

        env.set_evaluation(hook_eval.build_handles(args.repeats, env.device), press=args.press,
                           levels=args.eval_levels or hook_cfg.EVAL_LEVELS_N,
                           hold_steps=round(hook_cfg.EVAL_HOLD_S / interface.POLICY_DT),
                           kind=args.fixture_kind)
    obs, _ = env.reset()
    if not args.headless:
        # UniFP's overlay: the goal, the force-displaced goal the reward actually scores, the
        # tool tip, the sphere the goal is drawn on, the trajectory between start and goal, and
        # the applied and commanded force arrows. Viewer only -- headless training allocates none
        # of it.
        env.set_debug_vis(True)
    report = {
        "mode": args.mode,
        "seed": args.seed,
        # Recorded so a run says what produced it without the directory name having to be parsed,
        # which is the convention run_position_only.py already follows.
        "arguments": {k: v for k, v in vars(args).items() if not k.startswith("_")},
        "num_envs": env.num_envs,
        "actor_obs": list(obs["policy"].shape),
        "critic_obs": list(obs["critic"].shape),
        "estimate_obs": list(obs["estimates"].shape),
        "expected_actor": [env.num_envs, interface.NUM_OBS],
        "expected_critic": [env.num_envs, observations.NUM_CRITIC_OBS],
        "expected_estimates": [env.num_envs, observations.NUM_ESTIMATES],
        "action_space": env.cfg.action_space,
        "episode_length_steps": int(env.max_episode_length),
        "joint_order_ok": list(env._robot.joint_names)[env._order[0]] == interface.DOF_NAMES[0],
        "force_start_step": cfg.force_start_step,
        "force_start_iteration": cfg.force_start_step // task_cfg.NUM_STEPS_PER_ENV,
        "roll_objective": cfg.roll_objective,
        "task": args.task,
        "arm_armature_kg_m2": (hook_cfg.ARM_ARMATURE_KG_M2 if args.task in ("hook", "mechanism", "insertion", "press", "wipe", "rope", "leg")
                               else robot_mod.ARM_ARMATURE),
        "fixture": ({name: getattr(cfg, name) for name in (
            "fixture_fraction", "press_fraction", "force_ceiling_n", "force_ceiling_max_n", "curriculum")}
            if args.task == "hook" else None),
        "mechanism": ({name: getattr(cfg, name) for name in (
            "mechanism_fraction", "peak_ceiling_n", "peak_ceiling_max_n", "curriculum", "force_law", "force_law_gains",
            "force_law_bleed_s", "law_variant", "opening_probs", "lever_fraction", "lever_handle_pitch_range", "offpath_weight")}
            if args.task == "mechanism" else None),
        "insertion": ({"insertion_fraction": cfg.insertion_fraction, "curriculum": cfg.curriculum,
                       "curriculum_level": cfg.curriculum_level,
                       "outer_loops": {"tip_gain": cfg.tip_correction_gain, "elevation_gain": cfg.elevation_correction_gain,
                                       "roll_gain": cfg.roll_correction_gain, "tip_max_m": cfg.tip_correction_max_m,
                                       "elevation_max_rad": cfg.elevation_correction_max_rad,
                                       "roll_max_rad": cfg.roll_correction_max_rad},
                       "compliant_grip": cfg.compliant_grip, "hold_home": cfg.hold_home,
                       "curriculum_min_iterations": cfg.curriculum_min_iterations,
                       "reward_true_targets": cfg.reward_true_targets,
                       "engage_attitude_rad": cfg.engage_attitude_rad, "approach_weight": cfg.approach_weight}
                      if args.task == "insertion" else None),
        "press": ({"button_fraction": cfg.button_fraction, "tools": list(cfg.tools), "curriculum": cfg.curriculum,
                   "curriculum_level": cfg.curriculum_level, "curriculum_min_iterations": cfg.curriculum_min_iterations,
                   "outer_loops": {"tip_gain": cfg.tip_correction_gain, "tip_max_m": cfg.tip_correction_max_m,
                                   "elevation_gain": cfg.elevation_correction_gain,
                                   "elevation_max_rad": cfg.elevation_correction_max_rad}}
                  if args.task == "press" else None),
        "wipe": ({"wipe_fraction": cfg.wipe_fraction, "curriculum": cfg.curriculum,
                  "curriculum_level": cfg.curriculum_level, "curriculum_min_iterations": cfg.curriculum_min_iterations,
                  "pad": wipe_cfg.PAD,
                  "outer_loops": {"tip_gain": cfg.tip_correction_gain, "tip_max_m": cfg.tip_correction_max_m,
                                  "elevation_gain": cfg.elevation_correction_gain,
                                  "elevation_max_rad": cfg.elevation_correction_max_rad}}
                 if args.task == "wipe" else None),
        "rope": ({"rope_fraction": cfg.rope_fraction, "release_prob": cfg.release_prob,
                  "detect_never_prob": cfg.detect_never_prob, "stiffness_range": list(cfg.stiffness_range),
                  "length_range": list(cfg.length_range), "force_ceiling_n": cfg.force_ceiling_n,
                  "force_ceiling_max_n": cfg.force_ceiling_max_n, "curriculum": cfg.curriculum,
                  "weights": env._rope_weights,
                  "curriculum_rule": {"rule": cfg.curriculum_rule, "promote_rel_err": rope_cfg.CURRICULUM_PROMOTE_REL_ERR,
                                      "ema": rope_cfg.CURRICULUM_EMA, "step_n": rope_cfg.CURRICULUM_STEP_N,
                                      "min_iterations": rope_cfg.CURRICULUM_MIN_ITERATIONS},
                  "force_term": {"sigmas_n": [rope_cfg.FORCE_SIGMA_FINE_N, rope_cfg.FORCE_SIGMA_COARSE_N,
                                              rope_cfg.FORCE_SIGMA_BROAD_N], "shares": list(rope_cfg.FORCE_SHARES)}}
                 if args.task == "rope" else None),
        "leg": ({"variant": cfg.leg_variant, "object": cfg.leg_object, "arm_pose": cfg.arm_pose,
                 "leg_actuator": args.leg_actuator, "leg_delay_steps": args.leg_delay_steps,
                 "key_turn_deg": [round(math.degrees(v), 3) for v in (leg_cfg.KEY_THETA_START_RAD, leg_cfg.KEY_THETA_END_RAD,
                                                                      leg_cfg.KEY_THETA_UNLOCK_RAD)],
                 "key_overdrive_deg": round(math.degrees(leg_cfg.OVERDRIVE_RAD), 3), "key_hold_s": leg_cfg.HOLD_S,
                 "key_eval_lock_nm": list(leg_cfg.KEY_EVAL_FRICTION_NM), "key_eval_detent_nm": list(leg_cfg.KEY_EVAL_DETENT_NM),
                 "key_max_force_n": list(leg_cfg.KEY_MAX_N_LIMITS),
                 "leg_actuator_model": leg_actuators.describe(args.leg_actuator, args.leg_delay_steps, args.leg_armature,
                                                              args.leg_knee_armature, args.leg_delay_min_steps),
                 "leg_armature_range": (list(cfg.leg_armature_range) if cfg.leg_armature_range is not None else None),
                 "knee_armature_factor": cfg.knee_armature_factor,
                 "press_up_cap_n": cfg.press_up_cap_n,
                 "press_orientation_weight": cfg.press_orientation_weight, "tilt_excess_weight": cfg.tilt_excess_weight,
                 "tilt_stages": cfg.tilt_stages,
                 "tilt_excess_deg": round(math.degrees(leg_cfg.TILT_EXCESS_RAD), 3),
                 "start_delay_steps": list(cfg.start_delay_steps), "eval_start_delay_steps": cfg.eval_start_delay_steps,
                 "leg_friction_range": (list(cfg.leg_friction_range) if cfg.leg_friction_range is not None else None),
                 "knee_friction_factor": cfg.knee_friction_factor,
                 "press_limit_n": cfg.press_limit_n, "press_excess_weight": cfg.press_excess_weight,
                 "damage_n": leg_cfg.DAMAGE_N, "key_lock_range_nm": list(leg_cfg.KEY_FRICTION_NM),
                 "key_detent_range_nm": list(leg_cfg.KEY_DETENT_NM),
                 "settled_spawn": cfg.settled_spawn, "stance_weight": cfg.stance_weight,
                 "spawn_states": cfg.spawn_states, "spawn_states_fraction": cfg.spawn_states_fraction,
                 "handle_fraction": cfg.handle_fraction,
                 "force_ramp_iterations": cfg.force_ramp_iterations, "arm_jitter": cfg.arm_jitter,
                 "arm_look_pose": list(leg_cfg.ARM_LOOK_POSE), "foot_force_range_n": list(leg_cfg.FOOT_FORCE_RANGE_N),
                 "foot_push_n": cfg.foot_push_n, "foot_command_n": cfg.foot_command_n,
                 "foot_force_kp": leg_cfg.FOOT_FORCE_KP, "weights": dict(leg_cfg.REWARD_WEIGHTS),
                 "handle_weights": dict(leg_cfg.HANDLE_WEIGHTS),
                 "press": ({k: (list(v) if isinstance(v, tuple) else v) for k, v in vars(foot_press_cfg).items()
                            if k.isupper()} if args.leg_object == "press" else None)}
                if args.task == "leg" else None),
        "ground_friction": args.ground_friction if args.ground_friction is not None else robot_mod.GROUND_STATIC_FRICTION,
        "tool_body": cfg.tool_body,
        "tool_offset_m": list(cfg.tool_offset_m),
        "mount_pos_m": (list(args.mount_pos) if args.mount_pos is not None else None) if args.robot_usd else
                       list(args.mount_pos if args.mount_pos is not None else __import__("mount").ARM_MOUNT_POS_M),
        "robot_usd": robot_usd,
        "payload": None if args.robot_usd else {"mode": args.payload, "bodies": payload_record},
        "robot_usd_sha256": hashlib.sha256(Path(robot_usd).read_bytes()).hexdigest(),
        # The D1's Joint1-6 effort limits the actuator clips at: Joint3 was 1.7 N·m until 2026-10-09, 3.33 since.
        "arm_effort_limits_nm": list(interface.TORQUE_LIMITS[12:18]),
        # The welded USD only references the imported arm, so its hash does not change with the arm; this one does.
        "d1_urdf_sha256": hashlib.sha256((ROOT / "d1_arm/d1.urdf").read_bytes()).hexdigest(),
        "stack": stack_versions(),
        "git_commit": git_output("rev-parse", "HEAD"),
        "git_status": git_output("status", "--short"),
    }
    if args.task == "leg":
        # What the legs' joints loaded with, as PhysX holds them (UniFP's order): the leg actuator models
        # (`leg_actuators`) differ in these, and a group that sets no armature keeps the robot file's.
        legs, data = env._order[:12], env._robot.data
        report["leg_joints_loaded"] = {
            "groups": {name: type(group).__name__ for name, group in env._robot.actuators.items()},
            **{key: [round(float(v), 6) for v in getattr(data, attr)[0, legs].tolist()]
               for key, attr in (("armature", "joint_armature"), ("friction", "joint_friction_coeff"),
                                 ("effort_limit_sim", "joint_effort_limits"), ("velocity_limit_sim", "joint_vel_limits"))
               if getattr(data, attr, None) is not None}}
    report["shapes_ok"] = (report["actor_obs"] == report["expected_actor"]
                           and report["critic_obs"] == report["expected_critic"]
                           and report["estimate_obs"] == report["expected_estimates"])

    if args.mode == "train":
        status = train(args, env, report, run_dir)
    elif args.mode == "hook_eval":
        status = hook_evaluate(args, env, obs, report, run_dir)
        write_json(run_dir / "run.json", report)
        report["run_dir"] = str(run_dir)
    elif args.mode == "mech_eval":
        status = mech_evaluate(args, env, obs, report, run_dir, mech_kinds, mech_levels, mech_placements)
        write_json(run_dir / "run.json", report)
        report["run_dir"] = str(run_dir)
    elif args.mode == "ins_eval":
        status = ins_evaluate(args, env, obs, report, run_dir, ins_rows)
        write_json(run_dir / "run.json", report)
        report["run_dir"] = str(run_dir)
    elif args.mode == "rope_eval":
        status = rope_evaluate(args, env, obs, report, run_dir, rope_rows)
        write_json(run_dir / "run.json", report)
        report["run_dir"] = str(run_dir)
    elif args.mode == "leg_eval":
        status = leg_evaluate(args, env, obs, report, run_dir, eval_rows)
        write_json(run_dir / "run.json", report)
        report["run_dir"] = str(run_dir)
    elif args.mode == "leg_handover":
        from unifp_train import leg_handover
        status = leg_handover.run(env, args, report, run_dir)
        write_json(run_dir / "run.json", report)
        report.pop("handover", None)
        report["run_dir"] = str(run_dir)
    elif args.mode in ("press_eval", "wipe_eval"):
        status = contact_evaluate(args, env, obs, report, run_dir, eval_rows)
        write_json(run_dir / "run.json", report)
        report["run_dir"] = str(run_dir)
    elif args.mode in ("manifest", "eval"):
        status = evaluate(args, env, report, run_dir)
        write_json(run_dir / "run.json", report)
        report["run_dir"] = str(run_dir)
    else:
        # Written before the loop as well as after, so a run that is interrupted -- a viewer
        # session closed, a crash -- still leaves a record. Writing it only at the end means an
        # aborted run is invisible, which is exactly the run most worth having a note of.
        # The checkpoint goes in before the loop, not just after it. `rollout()` records the
        # iteration and source once it has loaded the weights, but a viewer session closed
        # halfway would otherwise leave a record that does not say which policy was on screen --
        # which is most of what such a record is for.
        if args.mode == "play":
            report["checkpoint"] = os.path.abspath(args.checkpoint)
        report["status"] = "running"
        write_json(run_dir / "run.json", report)
        report["run_dir"] = str(run_dir)
        status = rollout(args, env, obs, report, simulation_app)
        report["status"] = "finished"
        write_json(run_dir / "run.json", report)

    print(json.dumps(report, indent=2), flush=True)
    env.close()
    # Isaac Sim 5.1 hangs in teardown here as it does for run_unifp_isaaclab.py; everything this
    # prints is already flushed, so leave rather than wait.
    sys.stdout.flush()
    os._exit(status)


def rollout(args, env, obs, report: dict, simulation_app) -> int:
    """Step the environment open-loop, or under an existing checkpoint."""
    import torch

    from unifp_isaaclab import interface

    policy = None
    if args.mode == "play":
        # `eval.EvaluablePolicy` rather than `UniFPPolicy` because it loads *both* checkpoint
        # layouts -- UniFP's single module and rsl-rl 5.x's split actor -- behind the same `act`.
        # Without it `play` can only show a policy trained in the other stack, which stopped being
        # the interesting case the moment this one trained a policy of its own. The two loaders
        # agree to 0.0 on the same weights (`tests/test_unifp_train.py`), so this changes nothing
        # about what an Isaac Gym checkpoint does here.
        from unifp_train.eval import EvaluablePolicy

        policy = EvaluablePolicy(args.checkpoint, device=str(env.device))
        report["checkpoint"] = os.path.abspath(args.checkpoint)
        report["checkpoint_iteration"] = policy.iteration
        report["checkpoint_source"] = policy.source

    rewards_seen, resets, pushed, errors = [], 0, 0, []
    hook = hasattr(env, "_fixture")
    mech = hasattr(env, "_mech")
    ins = hasattr(env, "_slot")
    press = hasattr(env, "_button")
    wipe = hasattr(env, "_surface")
    roped = hasattr(env, "_rope")
    rope_held, rope_slack, rope_released, rope_fell, rope_peak, rope_peak_cmd = 0, 0, 0, 0, 0.0, 0.0
    ins_engaged, ins_entered, ins_seated, ins_peak, ins_deepest, ins_approach = 0, 0, 0, 0.0, -1.0, []
    contact_engaged, contact_touching, contact_done, contact_peak, contact_approach = 0, 0, 0, 0.0, []
    engaged_steps, lost, peak_force, peak_command = 0, 0, 0.0, 0.0
    held_steps, torn, peak_grasp, peak_speed, furthest = 0, 0, 0.0, 0.0, 0.0
    import time

    paced = time.monotonic()
    for _ in range(args.steps):
        if not simulation_app.is_running():
            break
        if args.real_time:
            paced += interface.POLICY_DT
            time.sleep(max(0.0, paced - time.monotonic()))
        if policy is None:
            actions = torch.zeros(env.num_envs, int(env.cfg.action_space), device=env.device)
        else:
            actions = policy.act(obs["policy"])
        obs, reward, terminated, truncated, _ = env.step(actions)
        rewards_seen.append(float(reward.mean()))
        resets += int((terminated | truncated).sum())
        pushed += int((env._ee_force_w.norm(dim=-1) > 1e-6).sum())
        # The tool-tip error is reported because the main objective, `tracking_ee_force_world`,
        # is `exp(-2 * error)`: anything past about 4 m is indistinguishable from zero in the
        # reward and from a broken frame convention in the log. A number here says which.
        errors.append((env._tip_pos() - env._goal_world()).norm(dim=-1))
        if hook:
            engaged_steps += int(env._fixture.engaged.sum())
            lost += int(env._term_lost.sum())
            peak_force = max(peak_force, float(env._fixture.applied_by_robot().norm(dim=-1).max()))
            peak_command = max(peak_command, float(env._fixture_command_w().norm(dim=-1).max()))
        if roped:
            rope_held += int(env._rope.engaged.sum())
            rope_slack += int(env._rope.slack.sum())
            rope_released += int(env._rope.released.sum())
            rope_fell += int((env._term_fallen & env._rope_env).sum())
            rope_peak = max(rope_peak, float(env._tension_sub.max()))
            rope_peak_cmd = max(rope_peak_cmd, float(env._cmd_level.max()))
        if mech:
            held_steps += int(env._mech.grasped.sum())
            torn += int(env._term_torn.sum())
            peak_grasp = max(peak_grasp, float(env._mech.force_on_tool.norm(dim=-1).max()))
            peak_speed = max(peak_speed, float(env._mech.v.abs().max()))
            travel = env._mech.travel.clamp(min=1e-6)
            furthest = max(furthest, float((env._mech.s / travel * env._mech.grasped).max()))
        if ins and args.ins_debug is not None and len(rewards_seen) % 10 == 0:
            ins_debug_print(env, args.ins_debug, len(rewards_seen))
        if ins:
            ins_engaged += int(env._engaged.sum())
            ins_entered += int(env._slot.entered.sum())
            ins_seated += int(env._slot.seated().sum())
            ins_peak = max(ins_peak, float(env._slot.normal_sum_n.max()))
            active = env._slot.active
            if bool(active.any()):
                ins_deepest = max(ins_deepest, float(env._slot.tip_depth[active].max()))
            if bool(env._ins_env.any()):
                approach = env._hand_frame_terms()["ee_approach_w"]
                cos = (approach * env._approach_des_w).sum(-1).clamp(-1.0, 1.0)[env._ins_env]
                ins_approach.append(torch.rad2deg(torch.acos(cos)))
        if press or wipe:
            active = env._press_env if press else env._wipe_env
            contact_engaged += int(env._engaged.sum())
            if press:
                contact_touching += int(env._button.on_cap.sum())
                contact_done += int(env._button.pressed().sum())
            else:
                contact_touching += int(env._surface.in_contact.sum())
                contact_done += int((env._layer.phase >= 2).sum())
            contact_peak = max(contact_peak, float(env._step_peak.max()))
            if bool(active.any()):
                approach = env._hand_frame_terms()["ee_approach_w"]
                cos = (approach * env._approach_des_w).sum(-1).clamp(-1.0, 1.0)[active]
                contact_approach.append(torch.rad2deg(torch.acos(cos)))

    report["steps"] = len(rewards_seen)
    report["mean_reward_per_step"] = round(sum(rewards_seen) / max(1, len(rewards_seen)), 5)
    report["driven_by"] = "zero actions" if policy is None else os.path.basename(args.checkpoint)
    report["resets"] = resets
    #: Environment-steps in which a non-zero external force was acting. Zero unless the force
    #: curriculum was opened with --force_start_iteration.
    report["env_steps_with_external_force"] = pushed
    if errors:
        stacked = torch.cat(errors)
        report["tip_goal_error_m"] = {
            "median": round(float(stacked.median()), 4),
            "p90": round(float(stacked.quantile(0.9)), 4),
            "max": round(float(stacked.max()), 4),
        }
        # Metres of error means the tool and its goal are being measured in different frames,
        # not that the policy is bad: the goal sphere is about half a metre across.
        report["frames_consistent"] = bool(stacked.median() < 1.0)
    # Per-term means, so a comparison against the Isaac Gym recording is 27 numbers rather than
    # one. A single total can agree while two terms are wrong in opposite directions.
    if hook:
        # A sanity check on the contact rather than a result: the spring is explicit, and a peak
        # force far above anything commanded means it rang.
        report["fixture"] = {"engaged_env_steps": engaged_steps, "lost": lost,
                             "peak_applied_n": round(peak_force, 2),
                             "peak_commanded_n": round(peak_command, 2),
                             "force_ceiling_n": env._ceiling}
    if roped:
        # Sanity checks on the rope rather than results: a peak tension far past any command means the spring rang.
        report["rope_rollout"] = {"held_env_steps": rope_held, "slack_env_steps": rope_slack,
                                  "released_env_steps": rope_released, "rope_episodes_fell": rope_fell,
                                  "peak_tension_n": round(rope_peak, 2), "peak_command_n": round(rope_peak_cmd, 2),
                                  "force_ceiling_n": env._ceiling}
    if mech:
        # Sanity checks on the plant rather than results: a grasp force far past any peak drawn, or
        # a handle moving metres per second, means the explicit spring rang.
        report["mechanism"] = {"held_env_steps": held_steps, "torn": torn,
                               "peak_grasp_force_n": round(peak_grasp, 2),
                               "peak_handle_speed_m_s": round(peak_speed, 3),
                               "furthest_fraction_of_travel": round(furthest, 3),
                               "peak_ceiling_n": env._ceiling}
    if ins:
        # Sanity checks on the contact, not results: a force far past `DAMAGE_N` or a key metres deep means it rang.
        stacked = torch.cat(ins_approach) if ins_approach else torch.zeros(1)
        report["insertion"] = {"engaged_env_steps": ins_engaged, "entered_env_steps": ins_entered,
                               "seated_env_steps": ins_seated, "peak_contact_n": round(ins_peak, 2),
                               "deepest_tip_m": round(ins_deepest, 4),
                               "approach_error_deg_median": round(float(stacked.median()), 2),
                               "curriculum_level": env._level}
    if press or wipe:
        # Sanity checks on the contact, not results: a peak far past the damage threshold means the penalty rang.
        stacked = torch.cat(contact_approach) if contact_approach else torch.zeros(1)
        report["press_rollout" if press else "wipe_rollout"] = {
            "engaged_env_steps": contact_engaged,
            ("on_cap_env_steps" if press else "in_contact_env_steps"): contact_touching,
            ("pressed_env_steps" if press else "finished_env_steps"): contact_done,
            "peak_contact_n": round(contact_peak, 2), "approach_error_deg_median": round(float(stacked.median()), 2),
            "curriculum_level": env._level}
        if wipe:
            report["wipe_rollout"]["coverage_of_engaged_median"] = (
                round(float(env.coverage()[env._engaged].median()), 3) if bool(env._engaged.any()) else None)
    report["reward_terms_per_step"] = {
        name: round(float(total.mean()) / max(1, len(rewards_seen)), 6)
        for name, total in env._episode_sums.items()}
    return 0 if report["shapes_ok"] and report.get("frames_consistent", True) else 1


def evaluate(args, env, report: dict, run_dir: Path) -> int:
    """Freeze an evaluation set, or score a policy against one."""
    from unifp_train import eval as evaluation

    run_dir.mkdir(parents=True, exist_ok=True)
    conditions = evaluation.conditions_of(env)
    report["conditions"] = conditions

    if args.mode == "manifest":
        # The schedule is policy-independent by construction -- every draw is triggered by the
        # episode clock, not by anything the robot does -- so the set is realised with zero
        # actions and the digests hold for any policy. `eval` re-checks that on every run.
        records = evaluation.run_episodes(env, evaluation.build(
            args.role, env.num_envs, args.seed, conditions), policy=None)
        manifest = evaluation.build(
            args.role, env.num_envs, args.seed, conditions,
            episodes=[{"index": r["index"], "schedule_sha256": r["schedule_sha256"]}
                      for r in records])
        out = Path(args.manifest_out or
                   ROOT / f"results/manifests/unifp_isaaclab_{args.role}.json")
        evaluation.save(manifest, str(out))
        report["manifest"] = str(out)
        report["manifest_sha256"] = manifest["content_sha256"]
        report["episodes"] = len(manifest["episodes"])
        report["status"] = "manifest written"
        print(f"[manifest] {out}  {manifest['content_sha256'][:16]}  "
              f"{len(manifest['episodes'])} episodes", flush=True)
        return 0

    manifest = evaluation.load(args.manifest)
    if args.hold_command:
        # Replacing the sampler covers both the five-second timer and the reset path, so every
        # environment holds this command for the whole episode.
        import torch as _torch

        fixed = _torch.tensor(args.hold_command, device=env.device)

        def hold(env_ids):
            env._commands[env_ids, :3] = fixed

        env._resample_velocity_commands = hold
        report["schedule_overridden"] = args.hold_command
        print(f"[eval] holding base velocity command {tuple(args.hold_command)} for every "
              f"episode; schedule digests will not match the manifest by design", flush=True)
    policy = None
    if not args.zero_actions:
        policy = evaluation.EvaluablePolicy(args.checkpoint, device=str(env.device))
        report["checkpoint"] = os.path.abspath(args.checkpoint)
        report["checkpoint_source"] = policy.source
        report["checkpoint_iteration"] = policy.iteration
    controller = "zero actions" if policy is None else (
        f"{os.path.basename(args.checkpoint)} ({policy.source})")

    records = evaluation.run_episodes(env, manifest, policy=policy)
    summary = evaluation.summarise(
        records, manifest, controller,
        conditions=evaluation.compare_conditions(manifest, conditions))
    report["summary"] = summary
    report["status"] = "evaluated"
    write_json(run_dir / "eval.json", summary)
    write_json(run_dir / "eval_episodes.json", {"records": records})

    quiet = summary["goal_tracking_quiet_m"]
    print(f"\n[eval] {controller}  on {manifest['role']} "
          f"({summary['episodes']} episodes)", flush=True)
    print(f"  goal tracking, unforced steps : "
          f"{quiet['median'] * 100:.1f} cm median, {quiet['p90'] * 100:.1f} cm p90"
          if quiet else "  goal tracking: no unforced steps", flush=True)
    for key, label, scale, unit in (
            ("unified_tracking_m", "unified tracking (the objective)", 100, "cm"),
            ("unified_tracking_pushed_m", "unified tracking, under force", 100, "cm"),
            ("force_realised_n", "force realised of commanded", 1, "N"),
            ("estimator_err_n", "estimator error", 1, "N"),
            ("base_vel_err_m_s", "base velocity error", 1, "m/s")):
        stat = summary[key]
        if stat:
            print(f"  {label:31s}: {stat['median'] * scale:.2f} {unit} median", flush=True)
    cmd = summary["force_cmd_n"]
    if cmd:
        print(f"  {'force commanded':31s}: {cmd['median']:.2f} N median", flush=True)
    print(f"  falls                          : {summary['falls']['count']} of "
          f"{summary['episodes']}", flush=True)
    if summary["schedule_mismatches"] and not args.hold_command:
        print(f"  !! {len(summary['schedule_mismatches'])} episodes did not reproduce their "
              f"frozen schedule; this run is not comparable", flush=True)
        return 1
    if summary["condition_mismatches"]:
        print(f"  !! conditions differ from the manifest: {summary['condition_mismatches']}",
              flush=True)
    return 0


def hook_evaluate(args, env, obs, report: dict, run_dir: Path) -> int:
    """Climb the force staircase on every placement once, and score it (`unifp_train.hook_eval`)."""
    import numpy as np
    import torch

    from unifp_isaaclab import interface
    from unifp_train import eval as evaluation, hook_cfg, hook_eval

    run_dir.mkdir(parents=True, exist_ok=True)
    policy = None
    if not args.zero_actions:
        policy = evaluation.EvaluablePolicy(args.checkpoint, device=str(env.device))
        report["checkpoint"] = os.path.abspath(args.checkpoint)
        report["checkpoint_source"] = policy.source
        report["checkpoint_iteration"] = policy.iteration
    controller = "zero actions" if policy is None else os.path.basename(args.checkpoint)
    report["controller"] = controller
    # Which bodies the arm's contact sensor actually found: a sensor that matched nothing reads 0 N
    # exactly like an arm that touched nothing.
    report["arm_contact_bodies"] = list(env._arm_contact.body_names)
    report["press"] = bool(args.press)
    report["fixture_kind"] = None if args.press else args.fixture_kind

    done = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    trace = {key: [] for key in hook_eval.FIELDS}
    with torch.inference_mode():
        for _ in range(int(env.max_episode_length) - 1):
            actions = (torch.zeros(env.num_envs, int(env.cfg.action_space), device=env.device)
                       if policy is None else policy.act(obs["policy"]))
            obs, _, terminated, truncated, _ = env.step(actions)
            estimate = None if policy is None else policy.estimate(obs["policy"])
            ended = (terminated | truncated) & ~done
            step = hook_eval.record_step(env, estimate, done, ended)
            for key in hook_eval.FIELDS:
                trace[key].append(step[key])
            done |= ended
            if bool(done.all()):
                break
    trace = {key: np.stack(values) for key, values in trace.items()}
    np.savez_compressed(run_dir / "hook_trace.npz", **trace)
    result = hook_eval.score(trace, levels=tuple(args.eval_levels or hook_cfg.EVAL_LEVELS_N))
    write_json(run_dir / "hook_eval.json", result["summary"])
    write_json(run_dir / "hook_eval_episodes.json", {"episodes": result["episodes"]})
    report["summary"] = result["summary"]
    report["status"] = "evaluated"

    summary = result["summary"]
    print(f"\n[hook_eval] {controller}  {'press' if args.press else 'pull on a ' + args.fixture_kind}  "
          f"{summary['episodes']} episodes: engaged {summary['engaged']}, lost {summary['lost']}, "
          f"fell {summary['fell']}", flush=True)
    print(f"  sustained force: median {summary['sustained_n']['median']:.0f} N, "
          f"p10 {summary['sustained_n']['p10']:.0f}, p90 {summary['sustained_n']['p90']:.0f}, "
          f"max {summary['sustained_n']['max']:.0f}", flush=True)
    for row in summary["by_level"]:
        print(f"  {row['level_n']:4.0f} N: held {row['held']:3d}/{row['of']}  applied "
              f"{row.get('applied_n', float('nan')):6.1f}  arm load {row.get('arm_load_max', float('nan')):.2f}  "
              f"pitch {row.get('pitch_deg', float('nan')):+5.1f} deg  estimate {row.get('estimate_n', float('nan')):6.1f}",
              flush=True)
    for row in summary["by_height"]:
        print(f"  handle {row['handle_height_m']:.2f} m: sustained median {row['sustained_n_median']:.0f} N",
              flush=True)
    return 0


def rope_evaluate(args, env, obs, report: dict, run_dir: Path, rows: list[dict]) -> int:
    """Climb the tension staircase on every placement, let the anchor go, and score it (`unifp_train.rope_eval`)."""
    import numpy as np
    import torch

    from unifp_isaaclab import interface
    from unifp_train import eval as evaluation, rope_cfg, rope_eval

    run_dir.mkdir(parents=True, exist_ok=True)
    policy = None
    if not args.zero_actions:
        policy = evaluation.EvaluablePolicy(args.checkpoint, device=str(env.device))
        report["checkpoint"] = os.path.abspath(args.checkpoint)
        report["checkpoint_source"] = policy.source
        report["checkpoint_iteration"] = policy.iteration
    controller = "zero actions" if policy is None else os.path.basename(args.checkpoint)
    if args.stochastic and policy is not None:
        controller += " (stochastic)"
    if args.freeze_legs and policy is not None:
        controller += " (legs frozen)"
    report["legs_frozen"] = bool(args.freeze_legs)
    report["ramp_n_per_s"] = args.rope_ramp if args.rope_ramp is not None else rope_cfg.LEVEL_RAMP_N_PER_S
    report["controller"] = controller
    report["stochastic_actions"] = bool(args.stochastic)
    report["eval_plan"] = {"rows": len(rows), "levels_n": list(args.eval_levels or rope_cfg.EVAL_LEVELS_N),
                           "hold_s": rope_cfg.EVAL_HOLD_S, "window_s": rope_cfg.EVAL_WINDOW_S,
                           "recovery_s": rope_cfg.EVAL_RECOVERY_S, "stiffness": sorted({r["stiffness"] for r in rows}),
                           "releases": list(dict.fromkeys(r["release"] for r in rows)),
                           "rope_set": args.rope_set,
                           "frozen_set": args.rope_stiffness is None
                           and (args.eval_levels is None or args.rope_set == "test")
                           and args.ground_friction is None and not args.stochastic and not args.freeze_legs
                           and args.rope_ramp is None}

    done = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    trace = {key: [] for key in rope_eval.FIELDS}
    with torch.inference_mode():
        for _ in range(int(env.max_episode_length) - 1):
            actions = (torch.zeros(env.num_envs, int(env.cfg.action_space), device=env.device) if policy is None
                       else policy.sample(obs["policy"]) if args.stochastic else policy.act(obs["policy"]))
            if args.freeze_legs:
                actions[:, :12] = 0.0
            obs, _, terminated, truncated, _ = env.step(actions)
            estimate = None if policy is None else policy.estimate(obs["policy"])
            ended = (terminated | truncated) & ~done
            step = rope_eval.record_step(env, estimate, done, ended)
            for key in rope_eval.FIELDS:
                trace[key].append(step[key])
            done |= ended
            if bool(done.all()):
                break
    trace = {key: np.stack(values) for key, values in trace.items()}
    np.savez_compressed(run_dir / "rope_trace.npz", **trace)
    result = rope_eval.score(trace, rows, levels=tuple(args.eval_levels or rope_cfg.EVAL_LEVELS_N))
    write_json(run_dir / "rope_eval.json", result["summary"])
    write_json(run_dir / "rope_eval_episodes.json", {"episodes": result["episodes"]})
    report["summary"] = result["summary"]
    report["status"] = "evaluated"

    summary = result["summary"]
    med = lambda entry, key: (entry.get(key) or {}).get("median", float("nan"))
    p90 = lambda entry, key: (entry.get(key) or {}).get("p90", float("nan"))
    print(f"\n[rope_eval] {controller}: {summary['episodes']} episodes, engaged {summary['engaged']}, "
          f"fell before release {summary['fell_before_release']}", flush=True)
    for row in summary["by_level"]:
        print(f"  {row['level_n']:4.0f} N: held {row['held']:3d}/{row['of']}  mean {med(row, 'mean_n'):5.1f}  "
              f"std {med(row, 'std_n'):4.2f} (p90 {p90(row, 'std_n'):4.2f})  shake {med(row, 'hf_rms_n'):4.2f}  "
              f"tugs over the hold in {row['with_hold_tugs']} ({row['hold_tugs_total']}), last 2 s in {row['with_tugs']}  "
              f"peak {med(row, 'hold_peak_n'):5.1f}  "
              f"slack {med(row, 'slack_fraction'):4.2f}  {med(row, 'dominant_hz'):4.1f} Hz  "
              f"pitch {med(row, 'pitch_deg'):+5.1f}  arm {med(row, 'arm_load_max'):4.2f}  align {med(row, 'align_deg'):4.1f} deg",
              flush=True)
    for row in summary["recovery"]:
        print(f"  release {row['release']:15s}: fell {row['fell']}/{row['released']}  excursion "
              f"{med(row, 'base_excursion_max_m'):4.2f} m (p90 {p90(row, 'base_excursion_max_m'):4.2f})  base speed "
              f"{med(row, 'base_speed_max_m_s'):4.2f}  pitch change {med(row, 'pitch_change_max_deg'):4.1f} deg  lunge "
              f"{med(row, 'tool_lunge_max_m'):4.2f} m  lift-offs {med(row, 'foot_liftoffs'):3.0f}  settle "
              f"{med(row, 'settle_s'):4.2f} s ({row['never_settled']} never)", flush=True)
    return 0


def ins_debug_print(env, i: int, count: int) -> None:
    """One insertion environment's task layer, loops and motion, and the holding environments' medians (`--ins_debug`)."""
    import torch

    key = env.key_state()
    goal_jaw = env._goal_world()[i]
    mouth_p, axis_p = env._planned_world(torch.arange(env.num_envs, device=env.device))
    planned_tip = mouth_p - env._standoff.unsqueeze(-1) * axis_p
    tip_miss = (key["tip"] - planned_tip).norm(dim=-1)
    appr = torch.rad2deg(torch.acos((env._hand_frame_terms()["ee_approach_w"] * env._approach_des_w).sum(-1).clamp(-1, 1)))
    speed = env._tool_vel_w().norm(dim=-1)
    print(f"[ins {count}] ins {bool(env._ins_env[i])} tip-miss {1000*float(tip_miss[i]):.1f} mm "
          f"approach {float(appr[i]):.1f} deg speed {float(speed[i]):.3f} | all: tip-miss median "
          f"{1000*float(tip_miss[env._ins_env].median()) if bool(env._ins_env.any()) else -1:.1f} mm, "
          f"<30mm {int(((tip_miss < 0.03) & env._ins_env).sum())}/{int(env._ins_env.sum())} "
          f"missed {int(env._missed.sum())} engaged {int(env._engaged.sum())}", flush=True)
    print(f"[ins {count}] engaged {bool(env._engaged[i])} s {1000*float(env._layer.s[i]):.1f} "
          f"ref {1000*float(env._layer.ref[i]):.1f} depth {1000*float(env._slot.tip_depth[i]):.1f} "
          f"F {float(env._layer.force_cmd[i].norm()):.1f} contact {float(env._slot.normal_sum_n[i]):.1f} "
          f"jaw-goal {1000*float((env._tip_pos()[i]-goal_jaw).norm()):.1f} mm "
          f"stall {float(env._layer.stall[i]):.2f} theta {float(env._layer.theta[i]):.2f} "
          f"v {1000*float(env._layer.v[i]):.1f} mm/s", flush=True)
    att = torch.rad2deg(env._attitude_miss()[i])
    c = env._correction
    print(f"[ins {count}] loops: miss elev {float(att[0]):+.1f} azim {float(att[1]):+.1f} "
          f"roll {float(att[2]):+.1f} deg | cmd elev {math.degrees(float(c.elevation[i])):+.1f} "
          f"roll {math.degrees(float(c.roll[i])):+.1f} deg, tip offset {[round(1000 * float(v), 1) for v in c.offset[i]]} mm "
          f"| entered {bool(env._slot.entered[i])} touched {bool(env._layer.touched[i])}"
          + (f" | grip tilt {[round(math.degrees(float(v)), 1) for v in env._grip.tilt[i]]} deg"
             if getattr(env, "_grip", None) is not None else ""), flush=True)
    rd = env._robot.data
    arm_qd = rd.joint_vel[i, env._robot.find_joints("Joint[1-6]")[0]]
    print(f"[ins {count}] motion: base {float(rd.root_lin_vel_w[i].norm()):.3f} m/s "
          f"{float(rd.root_ang_vel_w[i].norm()):.3f} rad/s, arm joints |qd| {float(arm_qd.norm()):.3f} rad/s "
          f"max {float(arm_qd.abs().max()):.3f}, action |a_arm| {float(env._actions[i, 12:18].norm()):.2f}", flush=True)
    holding = env._ins_env & env._goals.hold_mask & ~env._engaged
    if bool(holding.any()):
        att_all = torch.rad2deg(env._attitude_miss()[holding]).abs()
        print(f"[ins {count}] holding {int(holding.sum())}: median tool speed "
              f"{float(speed[holding].median()):.3f} m/s, tip-miss {1000*float(tip_miss[holding].median()):.1f} mm, "
              f"|elev| {float(att_all[:, 0].median()):.1f} |azim| {float(att_all[:, 1].median()):.1f} "
              f"|roll| {float(att_all[:, 2].median()):.1f} deg", flush=True)


def ins_evaluate(args, env, obs, report: dict, run_dir: Path, rows: list[dict]) -> int:
    """Insert a key into every planned slot once, and score it (`unifp_train.insert_eval`)."""
    import numpy as np
    import torch

    from unifp_isaaclab import interface
    from unifp_train import eval as evaluation, insert_eval

    run_dir.mkdir(parents=True, exist_ok=True)
    policy = None
    if not args.zero_actions:
        policy = evaluation.EvaluablePolicy(args.checkpoint, device=str(env.device))
        report["checkpoint"] = os.path.abspath(args.checkpoint)
        report["checkpoint_source"] = policy.source
        report["checkpoint_iteration"] = policy.iteration
    controller = "zero actions" if policy is None else os.path.basename(args.checkpoint)
    report["controller"] = controller
    report["ins_set"] = args.ins_set
    report["ins_rows"] = rows
    report["ins_conditions"] = env._evaluation["conditions"]
    done = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    trace = {key: [] for key in insert_eval.FIELDS}
    with torch.inference_mode():
        for _ in range(int(env.max_episode_length) - 1):
            actions = (torch.zeros(env.num_envs, int(env.cfg.action_space), device=env.device)
                       if policy is None else policy.act(obs["policy"]))
            obs, _, terminated, truncated, _ = env.step(actions)
            ended = (terminated | truncated) & ~done
            step = insert_eval.record_step(env, done, ended)
            if args.ins_debug is not None and len(trace["engaged"]) % 10 == 0:
                ins_debug_print(env, args.ins_debug, len(trace["engaged"]))
            for key in insert_eval.FIELDS:
                trace[key].append(step[key])
            done |= ended
            if bool(done.all()):
                break
    trace = {key: np.stack(values) for key, values in trace.items()}
    np.savez_compressed(run_dir / "ins_trace.npz", **trace)
    result = insert_eval.score(trace, rows)
    write_json(run_dir / "ins_eval.json", result["summary"])
    write_json(run_dir / "ins_eval_episodes.json", {"episodes": result["episodes"]})
    report["summary"] = result["summary"]
    report["status"] = "evaluated"
    s = result["summary"]
    print(f"\n[ins_eval {args.ins_set}] {controller}  seated {s['seated']}/{s['episodes']}  started {s['started']}  "
          f"entered {s['entered']}  ever seated {s['ever_seated']}  fell {s['fell']}  damaged {s['damaged']}  "
          f"at start {s['at_start_median_abs']}",
          flush=True)
    for key in ("by_lateral", "by_detent", "by_height", "by_roll"):
        for row in s[key]:
            print(f"  {key[3:]} {list(row.values())[0]}: seated {row['seated']}/{row['of']}  searched {row['searched']}  "
                  f"time {row['time_to_seat_s']}  contact {row['peak_contact_n']}  lateral {row['peak_lateral_n']}  "
                  f"approach {row['approach_err_start_deg']} deg", flush=True)
    return 0


def contact_evaluate(args, env, obs, report: dict, run_dir: Path, rows: list[dict]) -> int:
    """Press every planned button or wipe every planned path once, and score it (`press_eval`, `wipe_eval`)."""
    import numpy as np
    import torch

    from unifp_isaaclab import interface
    from unifp_train import eval as evaluation, press_eval, wipe_eval

    scorer, name = (press_eval, "press") if args.mode == "press_eval" else (wipe_eval, "wipe")
    run_dir.mkdir(parents=True, exist_ok=True)
    policy = None
    if not args.zero_actions:
        policy = evaluation.EvaluablePolicy(args.checkpoint, device=str(env.device))
        report["checkpoint"] = os.path.abspath(args.checkpoint)
        report["checkpoint_source"] = policy.source
        report["checkpoint_iteration"] = policy.iteration
    controller = "zero actions" if policy is None else os.path.basename(args.checkpoint)
    report["controller"] = controller
    report[f"{name}_set"] = args.press_set if name == "press" else "default"
    report[f"{name}_rows"] = rows
    report[f"{name}_conditions"] = env._evaluation["conditions"]
    done = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    trace = {key: [] for key in scorer.FIELDS}
    with torch.inference_mode():
        for _ in range(int(env.max_episode_length) - 1):
            actions = (torch.zeros(env.num_envs, int(env.cfg.action_space), device=env.device)
                       if policy is None else policy.act(obs["policy"]))
            obs, _, terminated, truncated, _ = env.step(actions)
            ended = (terminated | truncated) & ~done
            step = scorer.record_step(env, done, ended)
            for key in scorer.FIELDS:
                trace[key].append(step[key])
            done |= ended
            if bool(done.all()):
                break
    trace = {key: np.stack(values) for key, values in trace.items()}
    np.savez_compressed(run_dir / f"{name}_trace.npz", **trace)
    result = scorer.score(trace, rows)
    write_json(run_dir / f"{name}_eval.json", result["summary"])
    write_json(run_dir / f"{name}_eval_episodes.json", {"episodes": result["episodes"]})
    report["summary"] = result["summary"]
    report["status"] = "evaluated"
    s = result["summary"]
    print(f"\n[{name}_eval {report[f'{name}_set']}] {controller}  success {s['success']}/{s['episodes']}  "
          f"started {s['started']}  fell {s['fell']}  damaged {s['damaged']}  reasons {s['reasons']}", flush=True)
    groups = ("by_tool", "by_lateral", "by_force", "by_height") if name == "press" else (
        "by_kind", "by_offset", "by_force", "by_height")
    for key in groups:
        for row in s[key]:
            extra = (f"time {row['time_to_press_s']}" if name == "press"
                     else f"completion {row['completion']}  contact {row['contact_fraction']}")
            print(f"  {key[3:]} {list(row.values())[0]}: success {row['success']}/{row['of']}  {extra}  "
                  f"force rmse {row['force_rmse_n']}  peak {row['peak_contact_n']}  {row['reasons']}", flush=True)
    return 0


def leg_evaluate(args, env, obs, report: dict, run_dir: Path, rows: list[dict]) -> int:
    """Turn every planned door handle once with the front-right foot, and score it (`leg_eval`)."""
    import numpy as np
    import torch

    from unifp_isaaclab import interface
    from unifp_train import eval as evaluation, leg_eval

    if env.cfg.leg_object == "press":
        from unifp_train import foot_press_eval as leg_eval  # noqa: F811 -- the plates' fields, recording and scoring
    run_dir.mkdir(parents=True, exist_ok=True)
    policy = None
    if not args.zero_actions:
        policy = evaluation.EvaluablePolicy(args.checkpoint, device=str(env.device))
        report["checkpoint"] = os.path.abspath(args.checkpoint)
        report["checkpoint_source"] = policy.source
        report["checkpoint_iteration"] = policy.iteration
    controller = "zero actions" if policy is None else os.path.basename(args.checkpoint)
    report["controller"] = controller
    report["leg_rows"] = rows
    done = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    fields = leg_eval.FIELDS + leg_eval.TORQUE_FIELDS
    trace = {key: [] for key in fields}
    frames = min(args.leg_frames, env.num_envs) if args.leg_frames else 0
    if frames:
        import cv2

        env.set_debug_vis(True)
        (run_dir / "frames").mkdir(parents=True, exist_ok=True)
        report["frames"] = {"episodes": frames, "every_steps": args.leg_frame_every, "dir": str(run_dir / "frames")}
    dump_n = min(args.leg_shadow_dump, env.num_envs)
    dump = {k: [] for k in ("joint_pos", "joint_vel", "root_quat_w", "root_ang_vel_b", "commands", "frame", "action",
                            "joint_pos_target", "stage", "alive")} if dump_n else None
    with torch.inference_mode():
        for step_index in range(int(env.max_episode_length) - 1):
            if frames and step_index % args.leg_frame_every == 0:
                cams = [(name, getattr(env, attr)) for name, attr in (("head", "_head_cam"), ("wrist", "_wrist_cam"),
                                                                     ("side", "_overview_cam")) if hasattr(env, attr)]
                for name, cam in cams:
                    rgb = cam.data.output["rgb"][:frames, ..., :3].cpu().numpy()
                    for e in range(frames):
                        cv2.imwrite(str(run_dir / "frames" / f"ep{e:02d}_{name}_{step_index:04d}.png"),
                                    cv2.cvtColor(rgb[e], cv2.COLOR_RGB2BGR))
            actions = (torch.zeros(env.num_envs, int(env.cfg.action_space), device=env.device)
                       if policy is None else (policy.sample(obs["policy"]) if args.stochastic else policy.act(obs["policy"])))
            if dump is not None:
                # The state the frame below was built from (nothing moves between the observation and the action).
                data = env._robot.data
                for key, value in (("joint_pos", data.joint_pos), ("joint_vel", data.joint_vel),
                                   ("root_quat_w", data.root_quat_w), ("root_ang_vel_b", data.root_ang_vel_b),
                                   ("commands", env._commands), ("frame", obs["policy"][:, -interface.NUM_SINGLE_OBS:]),
                                   ("action", actions), ("stage", env._stage.float()), ("alive", (~done).float())):
                    dump[key].append(value[:dump_n].detach().float().cpu().numpy())
            obs, _, terminated, truncated, _ = env.step(actions)
            if dump is not None:
                # The joint targets the actuators were given for that action (Isaac's order).
                dump["joint_pos_target"].append(env._robot.data.joint_pos_target[:dump_n].detach().float().cpu().numpy())
            ended = (terminated | truncated) & ~done
            step = leg_eval.record_step(env, done, ended)
            for key in fields:
                trace[key].append(step[key])
            done |= ended
            if bool(done.all()):
                break
    trace = {key: np.stack(values) for key, values in trace.items()}
    np.savez_compressed(run_dir / "leg_trace.npz", **trace)
    if dump is not None:
        np.savez_compressed(run_dir / "shadow_dump.npz", joint_names=np.array(list(env._robot.joint_names)),
                            **{key: np.stack(values, axis=1) for key, values in dump.items()})
        report["shadow_dump"] = {"episodes": dump_n, "file": str(run_dir / "shadow_dump.npz")}
    result = leg_eval.score(trace, rows)
    write_json(run_dir / "leg_eval.json", result["summary"])
    write_json(run_dir / "leg_eval_episodes.json", {"episodes": result["episodes"]})
    report["summary"] = result["summary"]
    report["status"] = "evaluated"
    s = result["summary"]
    if env.cfg.leg_object == "press":
        print(f"\n[press_eval {env.cfg.leg_variant}] {controller}  success {s['success']}/{s['episodes']}  within {s['within']}  "
              f"engaged {s['engaged']}  released {s['released']}  stalled {s['stalled']}  touched {s['touched']}  "
              f"slipped {s['slipped']}  fell {s['fell']}  damaged {s['damaged']}  foot down {s['foot_down']}  {s['reasons']}\n"
              f"  force bias median {s['force_bias_n_median']} N (relative {s['force_rel_bias_median']}), mean abs error "
              f"median {s['force_mae_n_median']} N (p90 {s['force_mae_n_p90']}); peak median {s['peak_n_median']} N, p95 "
              f"{s['peak_n_p95']}, max {s['peak_n_max']}, past 60 N {s['over_60n']}; pressed {s['plane_err_mm_median']} mm from "
              f"the true contact; at the pre-contact point {s['engage_miss_mm_median']} mm off (p90 {s['engage_miss_mm_p90']}); stance worst-foot median {s['stance_max_cm_median']} cm; tilt {s['max_tilt_deg_median']} deg",
              flush=True)
        if "tangent" in s:
            print(f"  tangential: commanded {s['tangent']['cmd_n_median']} N, held {s['tangent']['mean_n_median']} N "
                  f"(error {s['tangent']['mae_n_median']} N); slide: commanded {s['slide']['cmd_m_median']} m, the foot "
                  f"moved {s['slide']['foot_m_median']} m (p10 {s['slide']['foot_m_p10']})", flush=True)
        for key in ("by_kind", "by_stiffness", "by_level_n", "by_mode"):
            for row in s[key]:
                print(f"  {key[3:]} {list(row.values())[0]}: success {row['success']}/{row['of']}  bias "
                      f"{row['force_bias_n_median']}  mae {row['force_mae_n_median']}  rel {row['force_rel_bias_median']}  "
                      f"press peak {row['press_peak_n_median']}  peak max {row['peak_n_max']}  {row['reasons']}", flush=True)
        return 0
    print(f"\n[leg_eval {env.cfg.leg_variant}] {controller}  success {s['success']}/{s['episodes']}  engaged {s['engaged']}  "
          f"opened {s['opened']}  fell {s['fell']}  damaged {s['damaged']}  slipped {s['slipped']}  reasons {s['reasons']}\n"
          f"  peak median {s['peak_n_median']} N (p95 {s['peak_n_p95']})  at stop {s['peak_at_stop_n_median']} N  "
          f"held {s['hold_force_n_median']} N  time to open {s['time_to_open_s_median']} s  "
          f"force rmse {s['force_rmse_n_median']} N  door {s['door_peak_n_median']} N  tilt {s['max_tilt_deg_median']} deg",
          flush=True)
    if "unlocked" in s:
        print(f"  key: unlocked {s['unlocked']}  foot back down {s['foot_down']}  both {s['success_and_foot_down']}  "
              f"time to unlock {s['time_to_unlock_s_median']} s  FR footfall {s['fr_footfall_cm_median']} cm off  "
              f"support stance median {s['stance_median_cm']} cm, worst-foot median {s['stance_max_cm_median']} cm "
              f"(p90 {s['stance_max_cm_p90']})", flush=True)
    if "torques" in s:
        t = s["torques"]
        print(f"  torques ({args.leg_actuator}, delay 0-{args.leg_delay_steps}): FR knee peak median {t['fr_calf_peak_nm']['median']} "
              f"(p90 {t['fr_calf_peak_nm']['p90']}, max {t['fr_calf_peak_nm']['max']}) N·m, demanded {t['fr_calf_demand_peak_nm']['median']}, "
              f"while turning {t['fr_calf_turn_peak_nm']['median']}; FR knee past 23.5 N·m in {t['episodes_fr_calf_over_stock']} episodes "
              f"for a median {t['fr_calf_over_stock_s']['median']} s; FR thigh {t['fr_thigh_peak_nm']['median']}, hip {t['fr_hip_peak_nm']['median']}; "
              f"support knees {t['sup_calf_peak_nm']['median']} (past 23.5 in {t['episodes_sup_calf_over_stock']}); FR knee speed "
              f"{t['fr_calf_speed_peak']['median']} rad/s (past 7.04 for {t['fr_calf_fast_s']['median']} s); clipped: FR "
              f"{t['fr_saturated_s']['median']} s, any leg {t['legs_saturated_s']['median']} s", flush=True)
    for key in ("by_full_nm", "by_friction_nm", "by_detent_nm", "by_axis_side", "by_contact_side", "by_left",
                "by_door_ahead", "by_height"):
        for row in s.get(key, []):
            print(f"  {key[3:]} {list(row.values())[0]}: success {row['success']}/{row['of']}  peak {row['peak_n_median']}  "
                  f"at stop {row['peak_at_stop_n_median']}  held {row['hold_force_n_median']}  "
                  f"time {row['time_to_open_s_median']}  {row['reasons']}", flush=True)
    return 0


def mech_evaluate(args, env, obs, report: dict, run_dir: Path, kinds: list[str], levels: list[float],
                  placements: list) -> int:
    """Open every planned mechanism once, and score it (`unifp_train.mech_eval`)."""
    import numpy as np
    import torch

    from unifp_isaaclab import interface
    from unifp_train import eval as evaluation, mech_eval

    run_dir.mkdir(parents=True, exist_ok=True)
    policy = None
    if not args.zero_actions:
        policy = evaluation.EvaluablePolicy(args.checkpoint, device=str(env.device))
        report["checkpoint"] = os.path.abspath(args.checkpoint)
        report["checkpoint_source"] = policy.source
        report["checkpoint_iteration"] = policy.iteration
    controller = "zero actions" if policy is None else os.path.basename(args.checkpoint)
    report["controller"] = controller
    report["arm_contact_bodies"] = list(env._arm_contact.body_names)
    report["mech_kinds"], report["mech_levels_n"] = kinds, levels
    report["mech_set"] = "test" if args.mech_test else "lever_high" if args.lever_high else "development"
    report["mech_conditions"] = env._evaluation["conditions"]
    report["mech_placements"] = [list(p) for p in placements]
    report["force_law"] = env._force_law

    done = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    trace = {key: [] for key in mech_eval.FIELDS}
    with torch.inference_mode():
        for _ in range(int(env.max_episode_length) - 1):
            actions = (torch.zeros(env.num_envs, int(env.cfg.action_space), device=env.device)
                       if policy is None else policy.act(obs["policy"]))
            obs, _, terminated, truncated, _ = env.step(actions)
            estimate = None if policy is None else policy.estimate(obs["policy"])
            ended = (terminated | truncated) & ~done
            step = mech_eval.record_step(env, estimate, done, ended)
            for key in mech_eval.FIELDS:
                trace[key].append(step[key])
            done |= ended
            if bool(done.all()):
                break
    trace = {key: np.stack(values) for key, values in trace.items()}
    np.savez_compressed(run_dir / "mech_trace.npz", **trace)
    result = mech_eval.score(trace, kinds, levels, placements=placements)
    write_json(run_dir / "mech_eval.json", result["summary"])
    write_json(run_dir / "mech_eval_episodes.json", {"episodes": result["episodes"]})
    report["summary"] = result["summary"]
    report["status"] = "evaluated"

    print(f"\n[mech_eval] {controller}  {result['summary']['episodes']} episodes", flush=True)
    for kind, entry in result["summary"]["kinds"].items():
        print(f"  {kind}: capacity {entry['capacity_n']:.0f} N, opened {entry['opened']}/{entry['of']}, "
              f"fell {entry['fell']}, torn {entry['torn']}", flush=True)
        for row in entry["by_level"]:
            fmt = lambda value, spec: format(value, spec) if value is not None else "  -  "
            print(f"    {row['level_n']:4.0f} N: opened {row['opened']:2d}/{row['of']}  "
                  f"time {fmt(row['open_time_s'], '5.2f')} s  peak drive {fmt(row['peak_drive_n'], '5.1f')} N  "
                  f"arm p95 {fmt(row['arm_load_p95'], '.2f')}  pitch {fmt(row['pitch_at_peak_deg'], '+5.1f')}  "
                  f"base moved {fmt(row['base_moved_at_peak_m'], '.3f')} m  "
                  f"peak speed {fmt(row['peak_speed_m_s'], '.2f')} m/s  estimate {fmt(row['estimate_at_peak_n'], '5.1f')}",
                  flush=True)
    return 0


def resumed_iteration(runner, checkpoint: str) -> int:
    """How many iterations a resumed run has already done.

    Prefers what the runner restored from the checkpoint, and falls back to the iteration in the
    filename, because the whole point of this number is to survive a checkpoint whose bookkeeping
    is not what this version of rsl-rl expects.
    """
    stored = getattr(runner, "current_learning_iteration", None)
    if isinstance(stored, int) and stored > 0:
        return stored
    match = re.fullmatch(r"model_(\d+)\.pt", os.path.basename(checkpoint))
    return int(match.group(1)) if match else 0


def train(args, env, report: dict, run_dir: Path) -> int:
    """PPO with the adaptation module, through rsl-rl's `OnPolicyRunner`."""
    from copy import deepcopy

    from isaaclab.utils.io import dump_yaml
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
    from rsl_rl.runners import OnPolicyRunner

    from unifp_train import task_cfg
    from unifp_train.agent import make_agent_cfg

    if not report["shapes_ok"] or not report["joint_order_ok"]:
        report["status"] = "refused: observation shapes or joint order wrong"
        write_json(run_dir / "run.json", report)
        return 1

    agent_cfg = make_agent_cfg(seed=args.seed, device=str(env.device), iterations=args.iterations,
                               run_name=args.run_name)
    departures = {}
    if args.std_type is not None:
        departures["std_type"] = args.std_type
        agent_cfg["actor"]["distribution_cfg"]["std_type"] = args.std_type
    if args.init_std is not None:
        departures["init_std"] = args.init_std
        agent_cfg["actor"]["distribution_cfg"]["init_std"] = args.init_std
    if args.entropy_coef is not None:
        departures["entropy_coef"] = args.entropy_coef
        agent_cfg["algorithm"]["entropy_coef"] = args.entropy_coef
    if args.learning_rate_scale is not None:
        departures["learning_rate_scale"] = args.learning_rate_scale
        agent_cfg["algorithm"]["learning_rate_scale"] = args.learning_rate_scale
    if args.estimator_learning_rate is not None:
        departures["estimator_learning_rate"] = args.estimator_learning_rate
        agent_cfg["algorithm"]["estimator_learning_rate"] = args.estimator_learning_rate
    if args.fixed_learning_rate is not None:
        departures["fixed_learning_rate"] = args.fixed_learning_rate
        agent_cfg["algorithm"]["fixed_learning_rate"] = args.fixed_learning_rate
    report["departures_from_unifp"] = departures

    run_dir.mkdir(parents=True, exist_ok=True)
    # The watcher checks liveness by this file rather than by `pgrep -f run_unifp_train.py`,
    # which matches any shell that merely mentions the script -- including the one that ran the
    # grep, so it reports a finished run as still going.
    (run_dir / "train.pid").write_text(f"{os.getpid()}\n")
    report["run_dir"] = str(run_dir)
    report["pid"] = os.getpid()
    report["iterations_requested"] = args.iterations
    report["agent"] = deepcopy(agent_cfg)
    report["status"] = "running"
    write_json(run_dir / "run.json", report)
    write_json(run_dir / "agent.json", agent_cfg)
    dump_yaml(str(run_dir / "env.yaml"), env.cfg)

    # clip_actions matches UniFP, which clips actions at +/-100 before its own scaling.
    from unifp_isaaclab import interface

    wrapped = RslRlVecEnvWrapper(env, clip_actions=interface.CLIP_ACTIONS)
    runner = OnPolicyRunner(wrapped, deepcopy(agent_cfg), log_dir=str(run_dir),
                            device=str(env.device))
    if args.resume_from:
        runner.load(args.resume_from)
        # F-082: the force curriculum lives on the environment, not in the checkpoint, and
        # `common_step_counter` restarts at zero on every launch. Without this a resume past the
        # gate silently trains position-only again for another `force_start_iteration` iterations,
        # with nothing in the log saying so. `launch_training.py` does this for the Isaac Gym side;
        # this path never did.
        done = resumed_iteration(runner, args.resume_from)
        env.common_step_counter = done * task_cfg.NUM_STEPS_PER_ENV
        report["resumed_from"] = os.path.abspath(args.resume_from)
        report["resumed_iteration"] = done
        if args.resume_arm_std is not None:
            import torch

            std = runner.alg.actor.distribution.std_param
            report["departures_from_unifp"]["resume_arm_std"] = {
                "from": [round(float(v), 4) for v in std.data[12:18]], "to": args.resume_arm_std}
            with torch.no_grad():
                std.data[12:18] = args.resume_arm_std
            print(f"[unifp] arm action-noise std set to {args.resume_arm_std} (was "
                  f"{report['departures_from_unifp']['resume_arm_std']['from']})", flush=True)
            write_json(run_dir / "run.json", report)
        if args.resume_leg_std is not None:
            import torch

            std = runner.alg.actor.distribution.std_param
            report["departures_from_unifp"]["resume_leg_std"] = {
                "from": [round(float(v), 4) for v in std.data[0:12]], "to": args.resume_leg_std}
            with torch.no_grad():
                std.data[0:12] = args.resume_leg_std
            print(f"[unifp] leg action-noise std set to {args.resume_leg_std}", flush=True)
            write_json(run_dir / "run.json", report)
        report["forces_active_at_resume"] = bool(
            env.common_step_counter > env.cfg.force_start_step)
        # Also worth saying out loud: on a resume `--iterations` means "this many *more*", because
        # `OnPolicyRunner.learn` adds it to the iteration it restored (F-082).
        print(f"[unifp] resumed at iteration {done}; force curriculum "
              f"{'ACTIVE' if report['forces_active_at_resume'] else 'not yet open'}; "
              f"training {args.iterations} more iterations, to {done + args.iterations}",
              flush=True)
    if args.diagnose_storage:
        runner.alg.diagnose_storage_requested = True

    # Isaac Sim installs its own signal handling, so a plain SIGINT kills the process without
    # Python ever raising KeyboardInterrupt and the shutdown below never runs -- which is how the
    # run of 2026-09-20 was left with a `run.json` still saying "running". Installing handlers
    # here puts the interpreter back in charge of both the interrupt and a polite terminate.
    import signal

    def stop(signum, _frame):
        raise KeyboardInterrupt(f"signal {signal.Signals(signum).name}")

    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, stop)

    started = datetime.now(timezone.utc)
    status = 0
    try:
        runner.learn(num_learning_iterations=args.iterations, init_at_random_ep_len=True)
        report["status"] = "complete"
    except KeyboardInterrupt:
        report["status"] = "interrupted"
    except Exception as error:  # noqa: BLE001 - recorded, then re-raised through the status code
        report["status"] = f"failed: {type(error).__name__}: {error}"
        status = 1
    finally:
        report["iterations_completed"] = runner.current_learning_iteration
        report["wall_clock_s"] = round(
            (datetime.now(timezone.utc) - started).total_seconds(), 1)
        checkpoint = run_dir / f"model_{runner.current_learning_iteration}.pt"
        if report["status"] != "complete" or not checkpoint.exists():
            runner.save(str(checkpoint))
        report["final_checkpoint"] = str(checkpoint)
        write_json(run_dir / "run.json", report)
        (run_dir / "train.pid").unlink(missing_ok=True)
    return status


if __name__ == "__main__":
    raise SystemExit(main())
