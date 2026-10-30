#!/usr/bin/env python3
"""Run the cup pick, the combiner-box opening or the door pull under UniFP's trained whole-body policy.

    source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
    unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH

    # watch one attempt
    ./demos/unifp/run_demo.py --task cup --num_envs 1 --attempts 1

    # measure: sixteen placements at once, headless
    ./demos/unifp/run_demo.py --task cup --attempts 16 --headless
    ./demos/unifp/run_demo.py --task combiner --attempts 16 --headless

    # the same, with the two wrist joints servoed instead of policy-driven
    ./demos/unifp/run_demo.py --task cup --attempts 16 --headless --wrist

    # the door pull: the box unlocked, the lever never turned; grip the handle and walk the door 90 degrees
    # open, the base carried round the hinge with the door by velocity commands
    ./demos/unifp/run_demo.py --task door --wrist --num_envs 1 --attempts 1
    ./demos/unifp/run_demo.py --task door --wrist --attempts 16 --headless

    # the combiner under the mechanism-trained policy (F-108): claw, force law, roll command, and the
    # script turning the lever to its stop with the claw 95 mm out; watch one, or measure a stiff door
    ./demos/unifp/run_demo.py --task combiner --mech --num_envs 1 --attempts 1 --door_torque_nm 8
    ./demos/unifp/run_demo.py --task combiner --mech --attempts 16 --headless --door_torque_nm 16
    # the old controller given the same claw, goal correction and script, for the comparison
    ./demos/unifp/run_demo.py --task combiner --wrist --claw --goal_correction --turn_deg 60 \
        --lever_grasp_m 0.095 --attempts 16 --headless --door_torque_nm 8

Then, as for every run in this repository:

    ./dashboard.py record <out dir> --title "..."

**What `--wrist` changes, and why it is reported separately.** UniFP commands a tool-tip position
and no orientation, so the hand's roll and pitch are free and the policy has no term that holds
them; measured on the deliverable checkpoint, the jaw axis swings a median of 20 degrees at a goal
that is not moving. `--wrist` replaces the policy's last two actions -- `Joint5` and `Joint6` --
with the servo in `wrist.py`, leaving it the legs and the arm's first four joints. That is a
different controller. A run made with it is evidence about "UniFP plus a wrist servo" and must
never be quoted as UniFP.

**What `--mech` changes.** It runs a policy trained on the goal-commanded mechanism task with the task
layer's force law in the loop (`unifp_train/mech_env.py --force_law`, F-108) the way it was trained:
the jaw-centre tool point, the arm's 0.01 kg·m² armature (F-102), a roll command in place of the
wrist servo, a claw on the lever bar, and, while the claw holds, the goal on the handle and a force
command from the PI law on how far the handle lags the script (`mech_env.py`). The claw is by default
Lukas's L-lip design as geometry -- a lip turned inward beyond each fingertip, the two overbiting so the
jaws still close fully, real colliders (`claw.py`) -- or, with `--claw_model spring`, a spring from the
jaw centre to the lever. The script is the same one, with the lever turned to its stop and hooked
further in (`MECH_*`). `--claw` gives any other controller the same claw, which is what makes a
comparison about the policy rather than the tool.

**What a result here does and does not show.** The demo is given the cup's or the box's pose. The
scripted demos find it first, with a wrist camera and a detector, and aiming that camera needs the
wrist control this policy does not have -- so the comparison is of the manipulation phase, from
first motion to release, and says nothing about perception. The object models, the latch rule, the
spring and the jaw geometry are the scripted demos' own, so what differs between the two is the
controller and the posture: these run standing, with the object on a table or a post, where the
scripted demos run lying down with it on the floor.
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import random
import subprocess
import time
import sys

ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT))

# Safe before `AppLauncher`: geometry, the phase machine and the placements are plain Python and
# numpy. Everything that touches Isaac Lab -- the environment, the robot, the policy -- is imported
# inside `main`, after the app exists.
from demos.combiner.geometry import DEFAULT_HANDLE_TORQUE_NM, GEOMETRY  # noqa: E402
from demos.unifp import props, script as demo_script  # noqa: E402

#: The reaching policy for the robot as modelled since 2026-10-09: the measured base (F-125, F-126), Unitree's CAD
#: lengths and a 3.33 N·m shoulder and elbow (F-139, F-140). `..._measured_model_57499.pt` (F-127) fine-tuned 900
#: iterations on it. `checkpoints/unifp_go2d1_isaaclab_measured_model_57499.pt` is the policy before that, and
#: `..._model_56000.pt` the same lineage on Unitree's base at the old mount.
DEFAULT_CHECKPOINT = "checkpoints/unifp_go2d1_isaaclab_cad333_model_58400.pt"
#: The mechanism-trained policy of F-108, law-trained, final checkpoint (`mech_law_v1` `model_17499`),
#: copied out of the gitignored `logs/` so the demo runs on another machine (`checkpoints/README.md`).
MECH_CHECKPOINT = "checkpoints/unifp_go2d1_mech_law_model_17499.pt"
#: What `--mech` asks of the script unless told otherwise (week 2 log, 2026-09-25): turn the lever to its
#: stop rather than 52 degrees, and hook it near its end rather than 75 mm out. Pushing the lever down is
#: this policy's weakest move (6-10 N whatever it is commanded); at 52 degrees and 75 mm the lever
#: stalled at ~40-44 degrees, short of the latch's 45, in up to 12 of 16 attempts.
MECH_TURN_DEG = GEOMETRY.handle_limit_deg
MECH_LEVER_GRASP_M = 0.095
#: With the lip claw, further in: its fingers are 26 mm wide, so at 95 mm on the 105 mm lever half the claw
#: hangs past the lever's end, one lip has no bar under it, and the lever pivoted out of the claw in 4 of 4
#: attempts (2026-09-25). At 80 mm the whole claw is on the bar with 12 mm to spare.
MECH_LEVER_GRASP_LIPS_M = 0.080


def git_output(*args: str, default: str = "") -> str:
    result = subprocess.run(["git", "-C", str(ROOT), *args], capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else default


def stack_versions() -> dict:
    versions = {}
    for name in ("isaacsim", "isaaclab", "isaaclab-rl", "rsl-rl-lib", "torch"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    return versions


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--task", choices=("cup", "combiner", "door"), default="cup",
                        help="'door': the combiner box unlocked, its handle never turned; the hand grips it and the "
                             "robot walks round the hinge with the door until it is --door_deg open.")
    parser.add_argument("--door_deg", type=float, default=props.DOOR_PULL_DEG,
                        help=f"Door only: how far the script swings the door, degrees (default {props.DOOR_PULL_DEG:g}; "
                             f"success is {props.DOOR_PULL_SUCCESS_DEG:g} or more at the end of the hold).")
    parser.add_argument("--close", action="store_true",
                        help="Door with --perception head (the combiner door demo): after the pull, close the door "
                             "again as the robot program's first close did (`demos/combiner_door/env.close_phases`; "
                             "the robot's close now looks at the open door's tag): the lidar reads the door, the base walks round the hinge to it, "
                             "the hand grabs the handle again and the base walks the door back to 5 deg.")
    parser.add_argument("--show_holster", action="store_true",
                        help="The combiner door demo: draw Lukas's holster beside the D1's base and the key in it, "
                             "carried with the robot, collisions off (on with --close).")
    parser.add_argument("--grip_s", type=float, default=None,
                        help="Door only: how long the jaws take to close on the handle, s (default DoorTiming's 0.8, "
                             "times --speed_scale).")
    parser.add_argument("--pull_s", type=float, default=None,
                        help="Door only: the swing's duration, s (default DoorTiming's 7, times --speed_scale). Much "
                             "slower and the walk command falls inside UniFP's dead zones, where it is told to stand.")
    parser.add_argument("--controller", choices=("unifp", "decoupled"), default="unifp",
                        help="Door only: 'decoupled' is the non-UniFP baseline -- a walking policy on the legs "
                             "(--walker) and inverse kinematics on the arm through the measured D1 model "
                             "(`decoupled.py`), on the same script, scene and placements.")
    parser.add_argument("--walker", choices=("rescue_2024", "p2dingo_gait"), default=None,
                        help="With --controller decoupled: the legs' walking policy (demos/unifp/decoupled.py WALKERS). "
                             "Default p2dingo_gait (since 2026-10-20); rescue_2024 reproduces F-151 and F-152.")
    parser.add_argument("--seeded_ik_loop", action="store_true",
                        help="With --controller decoupled: the arm IK's first version, which re-ran the seeded search "
                             "inside the 10 Hz loop (up to ~0.5 s a command). A diagnostic for comparing results.")
    parser.add_argument("--park_arm", action="store_true",
                        help="With --controller decoupled: never command the arm, which stays at its reset pose while the "
                             "legs walk the script's path. A diagnostic of the walking policy alone.")
    parser.add_argument("--perception", choices=("given", "head"), default="given",
                        help="Door with --controller decoupled: 'head' finds the box with a RealSense on the dog's head "
                             "and verifies the door with the MID-360 (demos/combiner_door). Default: the box's pose "
                             "is given.")
    parser.add_argument("--look_back_m", type=float, default=None,
                        help="With --perception head: how far behind the door pull's stance the robot starts and "
                             "looks from (default demos/combiner_door/env.py's LOOK_BACK_M).")
    parser.add_argument("--checkpoint", default=None,
                        help=f"Default {DEFAULT_CHECKPOINT}, or {MECH_CHECKPOINT} with --mech.")
    parser.add_argument("--attempts", type=int, default=16,
                        help="Placements to try. One environment each unless --num_envs is smaller.")
    parser.add_argument("--num_envs", type=int, default=None,
                        help="Attempts run at once. Defaults to 1 with the viewer (the rest follow one after another) "
                             "and to --attempts headless. The count changes results (F-098): state it with any rate.")
    parser.add_argument("--markers", action="store_true",
                        help="With the viewer: draw UniFP's goal overlay (`unifp_train/debug_vis.py`): yellow goal, "
                             "magenta scored goal, blue tool point, cyan sphere centre. Off by default so the hand can "
                             "be seen.")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--wrist", action="store_true",
                        help="Servo Joint5/Joint6 instead of letting the policy drive them. A "
                             "different controller; see the module docstring.")
    parser.add_argument("--wrist_joints", choices=("6", "56"), default="6",
                        help="Which wrist joints --wrist takes from the policy. '6' is the roll "
                             "alone, which levels the jaws and barely moves the jaw centre; '56' "
                             "adds the pitch, which also aims the approach and costs the reach.")
    parser.add_argument("--wrist_gain", type=float, default=None)
    parser.add_argument("--mech", action="store_true",
                        help="Combiner only: run the mechanism-trained policy as it was trained -- jaw-centre "
                             "tool point, 0.01 kg*m^2 arm armature, roll command, claw, force law. See the "
                             "module docstring.")
    parser.add_argument("--claw", action="store_true",
                        help="Combiner only: a claw on the lever bar (implied by --mech). See --claw_model.")
    parser.add_argument("--claw_model", choices=("lips", "spring"), default="lips",
                        help="'lips': the L-shaped fingers -- a 20 mm lip turned inward beyond each finger tip, the "
                             "two overbiting, real colliders built into the robot (demos/unifp/claw.py), the jaws "
                             "approaching fully open and closing fully onto the bar. 'spring': the first model, a "
                             "spring-damper from the jaw centre to the lever that holds in every direction.")
    parser.add_argument("--no_force_law", action="store_true",
                        help="With --mech: keep the claw and the roll, but follow the script's arc with the "
                             "goal alone, no force command. Measures what the task layer's law is worth.")
    parser.add_argument("--force_law_gains", type=float, nargs=3, default=None, metavar=("KP", "KI", "MAX"),
                        help="The law's gains (N/m, N/(m*s), N). Default mech_env.MechDemoEnvCfg's 400 1000 80.")
    parser.add_argument("--force_law_bleed", type=float, default=1.0,
                        help="The law's integral bleed time constant once the handle arrives, s (0 for none).")
    parser.add_argument("--no_goal_correction", action="store_true",
                        help="With --mech: no integral correction of the goal by the jaw centre's measured "
                             "miss during the reach's holds (mech_env.MechDemoEnvCfg.goal_correction).")
    parser.add_argument("--goal_correction", action="store_true",
                        help="Give any controller the reach's goal correction (implied by --mech).")
    parser.add_argument("--turn_deg", type=float, default=None,
                        help=f"Combiner only: the lever angle the script turns to (default "
                             f"props.LEVER_TURN_DEG, {props.LEVER_TURN_DEG:g}); the lever's stop is "
                             f"{GEOMETRY.handle_limit_deg:g} and the latch releases at {GEOMETRY.handle_release_deg:g}.")
    parser.add_argument("--ease_deg", type=float, default=None,
                        help="Combiner only: the lever angle the script lets the lever back to before the door is "
                             "pulled (CombinerTiming's 5). With --mech and the lip claw it defaults to the turn "
                             "angle: the lever is kept against its stop, where a hard pull cannot turn it out of the claw.")
    # The legs' model (2026-10-26, `unifp_train.leg_actuators`), as `run_unifp_train.py` takes it: the dog's leg corner the
    # retrained policies trained on is `--leg_actuator unitree_geared --leg_armature 0.0002 --leg_knee_armature 0.0007
    # --leg_delay_min_steps 3 --leg_delay_steps 4` (F-196's light legs). Not with the decoupled stack, which has its own.
    parser.add_argument("--payload", choices=("none", "physical"), default="none",
                        help="'physical': the robot with the lidar and wrist camera's masses (weld.py's payload, as "
                             "run_unifp_train.py builds by default since 2026-10-19), for policies trained on it. "
                             "'none', the demos' robot until 2026-10-26. Not with the claw.")
    parser.add_argument("--small_physx", action="store_true",
                        help="Shrink PhysX's GPU buffers (unifp_train.env_cfg.shrink_physx), so the demo fits beside "
                             "training runs on the shared GPU.")
    parser.add_argument("--leg_actuator", default="unifp",
                        choices=("unifp", "dcmotor", "knee23", "speedcurve", "knee23curve", "dcmotor_armature", "unitree",
                                 "unitree_geared"),
                        help="The legs' motor model (unifp_train/leg_actuators.py); 'unifp', UniFP's ideal PD, as trained.")
    parser.add_argument("--leg_armature", type=float, default=None, help="The legs' armature, kg·m² (trained 2e-4).")
    parser.add_argument("--leg_knee_armature", type=float, default=None)
    parser.add_argument("--leg_delay_min_steps", type=int, default=0)
    parser.add_argument("--leg_delay_steps", type=int, default=0,
                        help="Delay the legs' commands by --leg_delay_min_steps to this many physics steps (5 ms).")
    parser.add_argument("--jaw_effort_n", type=float, default=None,
                        help="Combiner only: the finger drives' force limit, N (the model's is the URDF's 15). A "
                             "bar pressing sideways on a finger pushes it open past this, which is how a lip claw "
                             "lets go; the real D1 gripper's value is unmeasured.")
    parser.add_argument("--turn_s", type=float, default=None,
                        help="Combiner only: the lever turn's duration, s (default CombinerTiming's 2.5, "
                             "times --speed_scale).")
    parser.add_argument("--lever_grasp_m", type=float, default=None,
                        help=f"Combiner only: how far out the lever the jaws close, from the spindle (default "
                             f"props.LEVER_GRASP_OFFSET_M, {props.LEVER_GRASP_OFFSET_M:g}; the lever is "
                             f"{GEOMETRY.handle_length:g} long). Further out needs less force for the same torque.")
    parser.add_argument("--door_torque_nm", type=float, default=0.0,
                        help="Combiner only: a door closer, the torque it needs at 45 degrees. 0 is a free door.")
    parser.add_argument("--post_top_m", type=float, default=None,
                        help=f"Combiner only: the post's height, m (default props.POST_TOP_M, {props.POST_TOP_M:g}). The "
                             f"lever's spindle is {GEOMETRY.spindle[2]:g} m above it: raise it to put the "
                             "handle above the robot, where turning it down is a pull. Pair with --box_distance_m.")
    parser.add_argument("--box_distance_m", type=float, nargs=2, default=None, metavar=("NEAR", "FAR"),
                        help="Combiner only: the range the box's root is placed at from the robot, m (default "
                             f"props.BOX_DISTANCE_RANGE_M, {props.BOX_DISTANCE_RANGE_M[0]:g}-{props.BOX_DISTANCE_RANGE_M[1]:g}). "
                             "A higher handle has to be nearer to stay inside the trained goal sphere.")
    parser.add_argument("--correct_entry", action="store_true",
                        help="With the goal correction: keep correcting through the entry, across the approach only "
                             "(mech_env goal_correction_enter), for a policy whose miss drifts while the hand moves in.")
    parser.add_argument("--stance_deg", type=float, default=0.0,
                        help="Combiner only: stand this far off the door's normal, degrees -- the box turned about "
                             "the grasp point (props.side_stance), so the reach is unchanged. Positive is the latch "
                             "side, where turning the lever down also draws it toward the robot; negative the hinge "
                             "side, where the opening door comes toward it.")
    parser.add_argument("--no_object", action="store_true",
                        help="Run the same script with the cup or the box moved aside. The control "
                             "that separates how well the arm follows the demo path from what "
                             "happens when it meets the object.")
    parser.add_argument("--free_space", action="store_true",
                        help="Move the furniture aside too, so the demo's path runs in clear air. "
                             "With --no_object, separates a path the arm cannot follow from an "
                             "arm that is leaning on the table.")
    parser.add_argument("--command_tip", action="store_true",
                        help="Diagnostic: command the tool point at the target rather than "
                             "converting so the jaw centre lands there. Cannot grasp anything; it "
                             "measures how much of the demo's position error is the conversion "
                             "riding on a hand whose orientation nothing holds.")
    parser.add_argument("--jaw_hold", type=float, default=None, metavar="TRAVEL_M",
                        help="Diagnostic: hold the jaws at this travel per finger for the whole "
                             "run, ignoring the script. 0 is the URDF's stop (17.2 mm gap, the "
                             "pose the policy trained in), 0.03 is fully open (77.2 mm).")
    parser.add_argument("--no_jaw_compensation", action="store_true",
                        help="Do not correct the goal for the jaws' travel. Measures what the "
                             "policy's blindness to its own jaws costs a grasp.")
    parser.add_argument("--zero_actions", action="store_true",
                        help="Discard the policy's actions: the baseline that says how much of "
                             "any result is the demo script rather than the controller.")
    parser.add_argument("--handle_torque_nm", type=float, default=None,
                        help="Combiner only: torque the lever's return spring needs at 45 degrees.")
    parser.add_argument("--speed_scale", type=float, default=1.0,
                        help="Multiply every phase duration. Below 1 the demo runs faster; how "
                             "far it can be pushed before the success rate goes is the only "
                             "honest speed number, since the script is clock-driven.")
    parser.add_argument("--hold_scale", type=float, default=1.0,
                        help="Multiply both of the script's holds -- at the standoff and at the "
                             "grasp point. How much of the position error is simply not having "
                             "waited long enough.")
    parser.add_argument("--settle_s", type=float, default=1.0,
                        help="Standing time before the script starts, so the spawn drop is over.")
    parser.add_argument("--camera_body", action=argparse.BooleanOptionalAction, default=True,
                        help="Draw the wrist RealSense in its printed bracket on the gripper, as fitted to the real arm, at "
                             "the saved mount (demos/cup/pick_demo). Visual only: no mass, no collider, no rendering.")
    parser.add_argument("--robot_usd", default=None)
    parser.add_argument("--out", default=None, help="Defaults to logs/unifp_demos/<stamp>_<task>.")
    parser.add_argument("--run_name", default="")
    return parser


def main() -> int:
    parser = build_parser()
    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    args.num_envs = args.num_envs or (args.attempts if args.headless else 1)
    if args.perception == "head":
        # The head camera renders; and the RealSense is on the head, so the wrist one is not drawn.
        args.enable_cameras = True
        args.camera_body = False
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    sys.path.insert(0, str(ROOT))
    import torch  # noqa: E402

    from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR  # noqa: E402
    from weld import build_welded_robot_usd  # noqa: E402
    from unifp_isaaclab import interface, robot as robot_mod  # noqa: E402
    from unifp_train import task_cfg  # noqa: E402
    from unifp_train.eval import EvaluablePolicy  # noqa: E402

    from demos.combiner.asset import build_combiner_usd  # noqa: E402
    from demos.cup.pick_demo.cup_asset import build_cup_usd  # noqa: E402
    from demos.unifp import env as demo_env, mech_env, wrist  # noqa: E402

    torque = args.handle_torque_nm if args.handle_torque_nm is not None else DEFAULT_HANDLE_TORQUE_NM
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    suffix = f"_{args.run_name}" if args.run_name else ""
    if args.mech and args.task != "combiner":
        parser.error("--mech is for the combiner task")
    if args.task == "door" and args.stance_deg:
        parser.error("--stance_deg is for the combiner task; the door pull starts square to the door")
    if args.task == "door" and not 10.0 <= args.door_deg <= GEOMETRY.open_limit_deg - 5.0:
        parser.error(f"--door_deg must be between 10 and {GEOMETRY.open_limit_deg - 5.0:g}")
    if args.mech and args.wrist:
        parser.error("--mech commands the roll itself; do not add --wrist")
    decoupled_run = args.controller == "decoupled"
    head_perception = args.perception == "head"
    if head_perception and not (decoupled_run and args.task == "door"):
        parser.error("--perception head is the combiner door demo: --task door --controller decoupled")
    if args.look_back_m is not None and not head_perception:
        parser.error("--look_back_m is for --perception head")
    if decoupled_run and (args.task != "door" or args.wrist or args.mech or args.zero_actions or args.checkpoint):
        parser.error("--controller decoupled is the door pull's baseline: --task door, without --wrist, --mech, "
                     "--zero_actions or --checkpoint")
    if args.checkpoint is None:
        args.checkpoint = MECH_CHECKPOINT if args.mech else DEFAULT_CHECKPOINT
    if args.mech:
        args.turn_deg = MECH_TURN_DEG if args.turn_deg is None else args.turn_deg
        if args.lever_grasp_m is None:
            args.lever_grasp_m = MECH_LEVER_GRASP_LIPS_M if args.claw_model == "lips" else MECH_LEVER_GRASP_M
    claw = args.claw or args.mech
    lips = claw and args.claw_model == "lips"
    force_law = args.mech and not args.no_force_law
    controller = ("mech_law" if force_law else "mech_goal") if args.mech else (
        "unifp_wrist" if args.wrist else ("zero_actions" if args.zero_actions else "unifp"))
    if decoupled_run:
        controller = "combiner_door" if head_perception else "decoupled_walk_ik"
    if claw and not args.mech:
        controller += "_claw"
    if claw and not lips:
        controller += "_spring"
    if args.mech and args.no_goal_correction:
        controller += "_nocorr"
    if args.goal_correction and not args.mech:
        controller += "_corr"
    if args.no_object:
        controller += "_no_object"
    if args.free_space:
        controller += "_free"
    if args.command_tip:
        controller += "_tipgoal"
    if args.jaw_hold is not None:
        controller += f"_jaw{int(round(args.jaw_hold * 1000))}mm"
    if args.no_jaw_compensation:
        controller += "_nojawcomp"
    out_dir = Path(args.out).resolve() if args.out else (
        ROOT / "logs/unifp_demos" / f"{stamp}_{args.task}_{controller}_seed{args.seed}{suffix}")
    out_dir.mkdir(parents=True, exist_ok=True)

    # Written before anything that can fail, not after the environment exists. A run that dies in
    # scene construction is exactly the run most worth having a note of, and three of this
    # session's did: they left a directory with no `run.json` and so nothing `./dashboard.py
    # record` could take.
    early = {
        "mode": "unifp_demo", "task": args.task, "controller": controller, "status": "running",
        "seed": args.seed, "arguments": {k: v for k, v in vars(args).items()
                                         if not k.startswith("_")},
        "command": " ".join(sys.argv), "stack": stack_versions(),
        "git_commit": git_output("rev-parse", "HEAD"), "git_status": git_output("status", "--short"),
    }
    (out_dir / "run.json").write_text(json.dumps(early, indent=2))

    if lips and args.payload != "none":
        parser.error("--payload physical builds the plain robot; the claw robot has no payload")
    if lips:
        # The L-lip fingers are geometry, so they are a different robot: a copy of the URDF with a lip on each
        # finger, welded into its own directory -- the weld writes `d1.usd` beside its output, and sharing
        # `generated/` would swap the plain robot's arm for this one.
        from demos.unifp import claw as lip_geometry

        claw_urdf = lip_geometry.write_claw_urdf(ROOT / "d1_arm/d1.urdf", ROOT / "generated/claw/d1_claw.urdf")
        robot_usd = build_welded_robot_usd(
            go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd",
            d1_urdf_path=str(claw_urdf), out_usd_path=str(ROOT / "generated/claw/go2_d1_claw.usd"),
            arm_mass_kg=3.152).usd_path
    else:
        # Its own file with the payload's physics, as run_unifp_train.py's, so the plain robot's is never swapped.
        robot_usd = args.robot_usd or build_welded_robot_usd(
            go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd",
            d1_urdf_path=str(ROOT / "d1_arm/d1.urdf"),
            out_usd_path=str(ROOT / ("generated/go2_d1_payload.usd" if args.payload == "physical" else "generated/go2_d1.usd")),
            arm_mass_kg=3.152, payload_physics=args.payload == "physical").usd_path

    if args.post_top_m is not None:
        # Before the scene is configured: the post, the box's spawn and every placement read the module constants.
        props.POST_TOP_M = args.post_top_m
    if args.box_distance_m is not None:
        props.BOX_DISTANCE_RANGE_M = tuple(args.box_distance_m)
    if args.lever_grasp_m is not None:
        # Before the phases are built: `props.BoxSite.grasp_point_m` reads the module constant, and so
        # does everything downstream of it (the script, the recorder, the law's Jacobian).
        props.LEVER_GRASP_OFFSET_M = args.lever_grasp_m
    asset_info: dict = {}
    holds = args.hold_scale * args.speed_scale
    if args.task == "cup":
        base = demo_script.CupTiming()
        timing = demo_script.CupTiming(
            **{f.name: getattr(base, f.name) * args.speed_scale
               for f in dataclasses.fields(base) if f.name.endswith("_s")},
            standoff_m=base.standoff_m)
        timing = dataclasses.replace(timing, settle_s=base.settle_s * holds,
                                     arrive_s=base.arrive_s * holds)
        phases = demo_script.cup_phases(timing)
    elif args.task == "door":
        base = demo_script.DoorTiming()
        timing = demo_script.DoorTiming(
            **{f.name: getattr(base, f.name) * args.speed_scale
               for f in dataclasses.fields(base) if f.name.endswith("_s")},
            standoff_m=base.standoff_m, door_deg=args.door_deg)
        timing = dataclasses.replace(timing, settle_s=base.settle_s * holds, arrive_s=base.arrive_s * holds)
        if args.pull_s is not None:
            timing = dataclasses.replace(timing, pull_s=args.pull_s)
        if args.grip_s is not None:
            timing = dataclasses.replace(timing, grip_s=args.grip_s)
        phases = demo_script.door_phases(timing)
        if lips:
            jaws = {demo_script.JAW_LEVER_OPEN_M: demo_script.JAW_WIDE_M}
            phases = [dataclasses.replace(p, jaw_m=jaws.get(p.jaw_m, p.jaw_m)) for p in phases]
    else:
        base = demo_script.CombinerTiming()
        timing = demo_script.CombinerTiming(
            **{f.name: getattr(base, f.name) * args.speed_scale
               for f in dataclasses.fields(base) if f.name.endswith("_s")},
            **{f.name: getattr(base, f.name) for f in dataclasses.fields(base)
               if not f.name.endswith("_s")})
        timing = dataclasses.replace(timing, settle_s=base.settle_s * holds,
                                     arrive_s=base.arrive_s * holds)
        if args.turn_deg is not None:
            timing = dataclasses.replace(timing, turn_deg=args.turn_deg)
        if args.turn_s is not None:
            timing = dataclasses.replace(timing, turn_s=args.turn_s)
        if args.ease_deg is None and args.mech and lips:
            args.ease_deg = timing.turn_deg
        if args.ease_deg is not None:
            timing = dataclasses.replace(timing, ease_deg=args.ease_deg)
        phases = demo_script.combiner_phases(timing)
        if lips:
            # The lips narrow the way in: at the friction grip's 41.2 mm opening the lip tips are 1.2 mm apart
            # and no bar gets between them, so the jaws come in fully open (37.2 mm between the tips). They close
            # as the friction grip does, onto the bar: beyond the fingertips the lips overbite and stop nothing.
            jaws = {demo_script.JAW_LEVER_OPEN_M: demo_script.JAW_WIDE_M}
            phases = [dataclasses.replace(p, jaw_m=jaws.get(p.jaw_m, p.jaw_m)) for p in phases]
    if args.close and not (head_perception and args.task == "door"):
        parser.error("--close is the combiner door demo's: --task door --controller decoupled --perception head")
    if head_perception:
        from demos.combiner_door import env as door_env  # noqa: E402

        look_back_m = door_env.LOOK_BACK_M if args.look_back_m is None else args.look_back_m
        phases = door_env.pre_phases(look_back_m) + list(phases)
        if args.close:
            phases += door_env.close_phases(timing)
    total_s = sum(phase.duration_s for phase in phases)

    goal_correction = (args.mech and not args.no_goal_correction) or args.goal_correction
    use_mech_env = args.mech or claw or goal_correction
    cfg = mech_env.MechDemoEnvCfg() if use_mech_env else demo_env.UniFPDemoEnvCfg()
    # The door pull is the combiner's scene: the same box on the same plinth.
    cfg.task = "cup" if args.task == "cup" else "combiner"
    cfg.seed = args.seed
    cfg.scene.num_envs = args.num_envs
    cfg.scene.env_spacing = 3.0
    if args.mech:
        # The mechanism policy's training conditions (F-108): the arm's rotor inertia of F-102, the
        # jaw-centre tool point (not the fingertip `UniFPDemoEnvCfg` pins for model_56000), the roll
        # objective, and its claw and law.
        from unifp_train import hook_cfg

        armature = {name: (hook_cfg.ARM_ARMATURE_KG_M2 if name in interface.ISAACLAB_NAMES[12:18]
                           else robot_mod.ARM_ARMATURE) for name in interface.ISAACLAB_NAMES}
        cfg.robot = robot_mod.make_robot_cfg(robot_usd, prim_path="/World/envs/env_.*/Robot", armature=armature)
        cfg.tool_body, cfg.tool_offset_m = task_cfg.TOOL_BODY, task_cfg.TOOL_OFFSET_M
        cfg.roll_objective = True
        cfg.roll_command = True
    elif decoupled_run:
        from demos.unifp import decoupled  # noqa: E402

        cfg.robot = decoupled.make_robot_cfg(robot_usd)
        cfg.seeded_ik_loop = args.seeded_ik_loop
        cfg.walker = args.walker or "p2dingo_gait"
    else:
        cfg.robot = robot_mod.make_robot_cfg(robot_usd, prim_path="/World/envs/env_.*/Robot")
    leg_model = None
    if not decoupled_run:
        from unifp_train import leg_actuators  # noqa: E402

        # 'unifp' with no delay and no armature leaves the robot as it was.
        leg_actuators.swap_legs(cfg.robot, args.leg_actuator, args.leg_delay_steps, args.leg_armature,
                                args.leg_knee_armature, args.leg_delay_min_steps)
        leg_model = leg_actuators.describe(args.leg_actuator, args.leg_delay_steps, args.leg_armature,
                                           args.leg_knee_armature, args.leg_delay_min_steps)
    elif args.leg_actuator != "unifp" or args.leg_delay_steps or args.leg_armature is not None:
        parser.error("--leg_* set UniFP's robot; the decoupled stack has its own legs")
    if args.jaw_effort_n is not None and decoupled_run:
        parser.error("--jaw_effort_n sets UniFP's actuator group; the decoupled robot has its own")
    if args.jaw_effort_n is not None:
        actuator = cfg.robot.actuators["unifp"]
        for name in ("Joint7_1", "Joint7_2"):
            actuator.effort_limit[name] = args.jaw_effort_n
            actuator.effort_limit_sim[name] = args.jaw_effort_n
    if use_mech_env:
        cfg.claw = claw and not lips
        cfg.claw_lips = lips
        cfg.force_law = force_law
        cfg.goal_correction = goal_correction
        cfg.goal_correction_enter = goal_correction and args.correct_entry
        if args.force_law_gains:
            cfg.force_law_gains = tuple(args.force_law_gains)
        cfg.force_law_bleed_s = args.force_law_bleed or None
    cfg.robot_usd = robot_usd
    cfg.track_leg_chatter = True      # leg_chatter.json beside the summary (unifp_train/leg_chatter.py)
    if args.small_physx:
        from unifp_train.env_cfg import shrink_physx  # noqa: E402

        shrink_physx(cfg.sim)
    camera_body = None
    if args.camera_body:
        from demos.cup.pick_demo.camera import resolve_mount
        from demos.cup.pick_demo.camera_asset import build_bracket_usd, build_camera_usd

        mount, mount_source, mount_file = resolve_mount()
        body, bracket = build_camera_usd(ROOT / "generated/pick_demo"), build_bracket_usd(ROOT / "generated/pick_demo")
        camera_body = {"mount_source": mount_source, "mount_file": mount_file,
                       "pos_link6_m": [float(v) for v in mount.pos_link6],
                       "quat_wxyz": [float(v) for v in mount.quat_wxyz()],
                       "body_usd": body["usd_path"], "bracket_usd": bracket and bracket["usd_path"],
                       "physics": "visual only: no mass, no collider"}
        cfg.camera_body_usd = body["usd_path"]
        cfg.camera_body_pose = (tuple(camera_body["pos_link6_m"]), tuple(camera_body["quat_wxyz"]))
        cfg.bracket_usd = camera_body["bracket_usd"] or ""
    cfg.handle_torque_nm = torque
    cfg.door_torque_nm = args.door_torque_nm
    # Long enough that the time limit never lands inside an attempt: the runner ends attempts, and
    # a reset partway through one would be scored as a failure that never happened.
    cfg.episode_length_s = args.settle_s + total_s + 5.0
    if args.task == "cup":
        cup = build_cup_usd(ROOT / "demos/cup/pick_demo/assets/High-Resolution_3D_Cup_Model_FBX.usdz",
                            ROOT / "generated/unifp_demos",
                            diameter_m=props.CUP_DIAMETER_M, height_m=props.CUP_HEIGHT_M,
                            mass_kg=props.CUP_MASS_KG)
        cfg.cup_usd = cup["usd_path"]
        asset_info = {"cup": cup}
        cfg.table = demo_env.table_cfg()
        cfg.cup = demo_env.cup_cfg(cfg.cup_usd)
    else:
        # Cached by what builds it: the torque, the geometry's every value and the builder's code. Keyed by the torque
        # alone it outlived the geometry it was built from: the lever's measured shape and gap (2026-10-20) went into
        # the scripts but not into the box simulated, until this key.
        import demos.combiner.asset as box_asset  # noqa: E402

        built_from = json.dumps(dataclasses.asdict(GEOMETRY), sort_keys=True).encode() + \
            Path(box_asset.__file__).read_bytes()
        box_key = hashlib.sha256(built_from).hexdigest()[:12]
        box_usd = ROOT / "generated/unifp_demos" / f"combiner_{torque:.3f}_{box_key}.usda"
        if not box_usd.exists():
            build_combiner_usd(box_usd, handle_torque_nm=torque)
        cfg.box_usd = str(box_usd)
        asset_info = {"combiner_usd": str(box_usd)}
        cfg.post = demo_env.post_cfg()
        cfg.combiner = demo_env.combiner_cfg(cfg.box_usd, torque, args.door_torque_nm)

    if not args.headless:
        # Follow the robot rather than sit in the world frame, and look at the work rather than at
        # the dog: both demos now happen at about 0.68 m, on a table or a post, so a camera aimed
        # at the old 0.45 m puts the grasp at the top of the frame.
        cfg.viewer.origin_type = "asset_root"
        cfg.viewer.asset_name = "robot"
        cfg.viewer.env_index = 0
        if args.task == "combiner":
            # Off the robot's right shoulder, high enough to see the lever turn and the door swing.
            cfg.viewer.eye = (1.15, -1.25, 1.05)
            cfg.viewer.lookat = (0.50, 0.0, 0.62)
        elif args.task == "door":
            # Behind and to the right, high: the robot walks right round the hinge and turns to face it.
            cfg.viewer.eye = (-0.9, -2.0, 1.7)
            cfg.viewer.lookat = (0.35, -0.45, 0.45)
        else:
            cfg.viewer.eye = (1.0, -1.15, 1.0)
            cfg.viewer.lookat = (0.45, 0.0, 0.64)

    if head_perception:
        cfg.look_back_m, cfg.door_deg = look_back_m, args.door_deg
        # The holster and the key in it, drawn on the robot (`door_env.CombinerDoorEnv._spawn_holster`).
        cfg.show_holster = args.close or args.show_holster
    env_class = door_env.CombinerDoorEnv if head_perception else decoupled.DecoupledDoorEnv if decoupled_run else (
        mech_env.MechDemoEnv if use_mech_env else demo_env.UniFPDemoEnv)
    env = env_class(cfg)
    if args.park_arm:
        if not decoupled_run:
            parser.error("--park_arm is for --controller decoupled")
        env.park_arm = True
    if args.task != "cup":
        # The grip squeezes the bar past the URDF's stop (F-063), with or without the lips.
        env.open_gripper_stop(demo_script.JAW_BAR_SHUT_M)
        # Unlocked for the door pull: the door is free, and the lever's spring still holds it up.
        env.attach_latches(locked=args.task == "combiner")
    env.hide_object = args.no_object
    env.hide_furniture = args.free_space
    env.command_tip = args.command_tip
    env.jaw_override = args.jaw_hold
    # The jaw centre does not move with the jaws' travel; only the fingertip needs the correction.
    env.jaw_compensation = not args.no_jaw_compensation and not args.mech
    env.scripts = [demo_script.DemoScript(phases) for _ in range(env.num_envs)]
    if not args.headless:
        env.set_debug_vis(args.markers)

    walker = decoupled.WalkingPolicy(decoupled.walker_checkpoint(env.walker), env.device) if decoupled_run else None
    policy = None if (args.zero_actions or decoupled_run) else EvaluablePolicy(args.checkpoint, device=str(env.device))

    report = {
        **early,
        "mode": "unifp_demo",
        "wrist_joints": args.wrist_joints if args.wrist else None,
        "task": args.task,
        "controller": controller,
        "status": "running",
        "seed": args.seed,
        "arguments": {k: v for k, v in vars(args).items() if not k.startswith("_")},
        "checkpoint": os.path.abspath(args.checkpoint) if policy else None,
        "checkpoint_source": policy.source if policy else None,
        "checkpoint_iteration": policy.iteration if policy else None,
        "decoupled": ({"legs": {"walker": env.walker, "policy": str(walker.path), "sha256": walker.sha256,
                                "trained": "a bare Go2's velocity tracking, 12 leg actions (decoupled.WALKERS)",
                                "actuator": "Isaac Lab Go2 DCMotor (25/0.5, 23.5 N·m)",
                                "observation": "flat_env_cfg's 235 (velocities, gravity, command, legs, last action, "
                                               "height scan; no noise), plus the gait clock for p2dingo_gait"},
                       "arm": {"control": "IK on Link6 + jaw centre in the base's current frame (decoupled_arm.py), "
                                          "grasp pitch/roll from the scripted grip's order, held per attempt",
                               "command_hz": 10.0,
                               "model": "D1 firmware planner (F-045) into the stiff servo drive, published torque "
                                        "and measured speed limits"},
                       "jaws": "UniFP's drive (800 N/m, 15 N)"} if decoupled_run else None),
        "park_arm": bool(args.park_arm),
        "perception": ({"box": "AprilTag in the Go2's front camera (demos/combiner_door/front_camera.py: the URDF's "
                                "mount, a NOMINAL 1280x720 fisheye drawn by remapping a wide pinhole render), "
                                "averaged over the look",
                        "look_back_m": look_back_m,
                        "camera_model": env.camera_model.source,
                        "door": "MID-360 ray caster, door angle fitted round the tag (demos/combiner_door/mid360.py); "
                                "verification only",
                        "odometry": "the simulator's base pose"}
                       if head_perception else {"box": "given (the simulator's placement)"}),
        "seeded_ik_loop": bool(args.seeded_ik_loop),
        "num_envs": env.num_envs,
        "attempts": args.attempts,
        "script_phases": [{"name": p.name, "duration_s": p.duration_s} for p in phases],
        "script_total_s": total_s,
        "speed_scale": args.speed_scale,
        "hold_scale": args.hold_scale,
        "settle_s": args.settle_s,
        "handle_torque_nm": torque if args.task != "cup" else None,
        "door_torque_nm": args.door_torque_nm if args.task != "cup" else None,
        "stance_deg": args.stance_deg if args.task == "combiner" else None,
        "post_top_m": props.POST_TOP_M if args.task != "cup" else None,
        "box_distance_m": list(props.BOX_DISTANCE_RANGE_M) if args.task != "cup" else None,
        "door_pull": ({"door_deg": args.door_deg, "success_deg": props.DOOR_PULL_SUCCESS_DEG, "latch": "unlocked",
                       "bearing_deg": list(props.DOOR_PULL_BEARING_RANGE_DEG),
                       "yaw_jitter_deg": list(props.DOOR_PULL_YAW_JITTER_DEG),
                       "base_gains_per_s": [demo_script.BASE_GAIN_XY, demo_script.BASE_GAIN_YAW],
                       "base_command_limits": list(demo_script.BASE_COMMAND_LIMITS)}
                      if args.task == "door" else None),
        "turn_deg": (args.turn_deg if args.turn_deg is not None else props.LEVER_TURN_DEG)
        if args.task == "combiner" else None,
        "lever_grasp_m": props.LEVER_GRASP_OFFSET_M if args.task == "combiner" else None,
        "ease_deg": args.ease_deg if args.task == "combiner" else None,
        "jaw_effort_n": args.jaw_effort_n,
        "tool_point": {"body": cfg.tool_body, "offset_m": list(cfg.tool_offset_m)},
        "arm_armature_kg_m2": 0.01 if args.mech else robot_mod.ARM_ARMATURE,
        "roll_objective": bool(cfg.roll_objective),
        "claw": (_claw_record(cfg, lips) if claw else None),
        "force_law": ({"gains": list(cfg.force_law_gains), "bleed_s": cfg.force_law_bleed_s}
                      if force_law else None),
        "goal_correction": ({"gain_per_s": cfg.goal_correction_gain, "max_m": cfg.goal_correction_max_m,
                             "through_entry": cfg.goal_correction_enter}
                            if goal_correction else None),
        "jaw_compensation": not args.no_jaw_compensation,
        "forces": "scheduled pushes off; contact only",
        "base_command": ("scripted walk round the hinge with the door (BoxSite.base_pose_m), velocity commands from "
                         "script.base_velocity_command: path rate fed forward, pose error fed back"
                         if args.task == "door" else "zero (standing)"),
        "perception": "none -- the object pose is given",
        "camera_body": camera_body,
        "assets": asset_info,
        "robot_usd": robot_usd,
        "leg_model": leg_model,
        "payload": args.payload,
        "robot_usd_sha256": hashlib.sha256(Path(robot_usd).read_bytes()).hexdigest(),
        "stack": stack_versions(),
        "git_commit": git_output("rev-parse", "HEAD"),
        "git_status": git_output("status", "--short"),
        "command": " ".join(sys.argv),
    }
    (out_dir / "run.json").write_text(json.dumps(report, indent=2))

    rows: list[dict] = []
    attempts: list[dict] = []
    rng = random.Random(args.seed)
    if args.task == "cup":
        sampler = props.sample_cup_site
    elif args.task == "door":
        sampler = props.sample_door_site
    else:
        # The same draws at every stance: a stance sweep compares the same placements, turned.
        sampler = lambda generator: props.side_stance(props.sample_box_site(generator), args.stance_deg)
    remaining, batch_index = args.attempts, 0
    status = "finished"

    wrist_indices = ([wrist.JOINT6_ACTION_INDEX] if args.wrist_joints == "6"
                     else [wrist.JOINT5_ACTION_INDEX, wrist.JOINT6_ACTION_INDEX])
    limits = torch.as_tensor(task_cfg.JOINT_POS_LIMITS, device=env.device)[wrist_indices]
    gain = args.wrist_gain if args.wrist_gain is not None else wrist.DEFAULT_GAIN

    while remaining > 0 and simulation_app.is_running():
        count = min(remaining, env.num_envs)
        sites = [sampler(rng) for _ in range(env.num_envs)]
        env.place(sites)
        # The script starts only after the spawn drop has settled: UniFP spawns the robot above
        # its standing height, so the first moments of an episode are a fall rather than a reach.
        # Set before the reset, so the reset seeds each script from the hand's measured position
        # and the first goal the policy sees is the one it is already at.
        # The combiner door demo spawns behind its stance (`demos/combiner_door`), so its home point is behind too.
        spawn_home = door_env.home_behind(look_back_m) if head_perception else demo_script.HOME_POINT
        for demo in env.scripts:
            demo.phases = [demo_script.Phase("spawn", args.settle_s,
                                             target=lambda st: spawn_home,
                                             jaw_m=phases[0].jaw_m)] + list(phases)
        obs, _ = env.reset()

        steps = int(round((args.settle_s + total_s) / interface.POLICY_DT)) + 2
        record = _AttemptRecord(env, args.task, count, batch_index, sites)
        with torch.no_grad():
            for step in range(steps):
                if not simulation_app.is_running():
                    status = "aborted"
                    break
                started = time.perf_counter()
                if decoupled_run:
                    action = env.action_from_legs(walker.act(env.walking_observation()))
                elif policy is None:
                    action = torch.zeros(env.num_envs, interface.NUM_ACTIONS, device=env.device)
                else:
                    action = policy.act(obs["policy"]).clone()
                if args.wrist:
                    action = _servo_wrist(env, action, wrist_indices, limits, gain)
                obs, _, terminated, truncated, _ = env.step(action)
                # Wall time of the step: the controller and the simulator together, every environment at once. Not a
                # deployment cost (the robot runs no simulator), but where a stall in the controller shows up.
                record.step_wall_ms = 1000.0 * (time.perf_counter() - started)
                record.update(step, terminated | truncated, rows)
        attempts.extend(record.finish())
        remaining -= count
        batch_index += 1

    summary = _summarise(attempts, args.task)
    report.update({"status": status, "attempt_count": len(attempts), "summary": summary})
    (out_dir / "run.json").write_text(json.dumps(report, indent=2))
    (out_dir / "summary.json").write_text(json.dumps(
        {"controller": controller, "task": args.task, "summary": summary, "attempts": attempts},
        indent=2))
    chatter = env.chatter_summary()
    if chatter is not None:
        (out_dir / "leg_chatter.json").write_text(json.dumps(chatter, indent=2))
    if rows:
        with (out_dir / "trace.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

    print(json.dumps({"out_dir": str(out_dir), "controller": controller, "task": args.task,
                      "summary": summary}, indent=2), flush=True)
    env.close()
    sys.stdout.flush()
    os._exit(0)


def _claw_record(cfg, lips: bool) -> dict:
    """What the run's claw was, for `run.json`."""
    if lips:
        from demos.unifp import claw as lip_geometry

        return {"model": "lips", "placement": "beyond the fingertips (overbite)",
                "lip_length_m": lip_geometry.LIP_LENGTH_M,
                "lip_thickness_m": lip_geometry.LIP_THICKNESS_M, "lip_clearance_m": lip_geometry.LIP_CLEARANCE_M,
                "lip_face_z_m": lip_geometry.LIP_FACE_Z_M,
                "entry_gap_open_m": lip_geometry.entry_gap_m(lip_geometry.MAX_TRAVEL_M),
                "lost_after_steps": cfg.claw_lost_steps}
    return {"model": "spring", "capture_m": cfg.claw_capture_m, "stiffness_n_m": cfg.claw_stiffness,
            "damping_n_s_m": cfg.claw_damping, "grip_n": cfg.claw_grip_n}


def _servo_wrist(env, action, indices, limits, gain):
    """Replace the policy's actions for the servoed wrist joints with the servo's.

    Only where the script asks for it. While the arm is still crossing to the object the policy
    keeps its whole wrist; the servo starts at the standoff hold, so the hand turns and the policy
    re-converges before the fine approach rather than during it.
    """
    import torch

    from unifp_isaaclab import interface
    from demos.unifp import wrist

    approach_cmd = torch.zeros(env.num_envs, 3, device=env.device)
    approach_cmd[:, 0] = 1.0
    object_axis = torch.zeros(env.num_envs, 3, device=env.device)
    object_axis[:, 2] = 1.0
    for index, command in enumerate(env.commands):
        if command is None:
            continue
        approach_cmd[index] = torch.as_tensor(command.approach, device=env.device)
        object_axis[index] = torch.as_tensor(command.object_axis, device=env.device)
    # The script works in the frame the robot spawned in; the servo compares world directions.
    approach_cmd = wrist.unit(interface.quat_apply(env._spawn_yaw_quat, approach_cmd))
    object_axis = wrist.unit(interface.quat_apply(env._spawn_yaw_quat, object_axis))

    _, approach_cur, jaw_cur = env._hand_frame()
    roll_only = indices == [wrist.JOINT6_ACTION_INDEX]
    # With the roll alone the approach is the policy's and cannot be argued with, so the jaw axis
    # is built across the approach the hand actually has.
    jaw_des = wrist.desired_jaw_axis(approach_cur if roll_only else approach_cmd, object_axis)
    error = wrist.orientation_error(approach_cur, jaw_cur, jaw_des,
                                    approach_des=None if roll_only else approach_cmd)
    steps = wrist.joint_correction(error, env.wrist_axes(indices), gain=gain)
    current = env._dof()[0][:, indices]
    servoed = wrist.joint_actions(current, steps, limits, interface.ACTION_SCALE)

    wanted = torch.as_tensor([bool(c is not None and c.servo) for c in env.commands],
                             device=env.device).unsqueeze(-1)
    action[:, indices] = torch.where(wanted, servoed, action[:, indices])
    return action


class _AttemptRecord:
    """Per-step truth for one batch of attempts, and the per-attempt rows it reduces to.

    Everything here is read from the simulator, not from the sequence: the script is not told
    where the cup ends up or how far the door opened, and the difference between what it commanded
    and what happened is the measurement.
    """

    #: Phases whose tracking and hand pose are worth a per-attempt number: the moments a grasp is
    #: actually decided. A transit phase is not a tracking measurement -- the goal is moving and
    #: the tip is meant to be behind it.
    SCORED_PHASES = ("settle", "arrive", "close", "grip", "hold", "pull")

    def __init__(self, env, task, count, batch, sites):
        import torch

        self.torch = torch
        self.env, self.task, self.count, self.batch, self.sites = env, task, count, batch, sites
        self.best = [{} for _ in range(env.num_envs)]
        self.fell = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
        self.phase_start_s: list[dict] = [{} for _ in range(env.num_envs)]
        self.scored: list[dict] = [{} for _ in range(env.num_envs)]
        self.last: list[dict] = [{} for _ in range(env.num_envs)]
        #: When the cup first went over *on the table*, and in which phase. A cup knocked flat
        #: reads as a 27.5 mm "lift" -- its own radius -- so a knock-over has to be named rather
        #: than inferred from height, and it has to be told apart from a cup that is up in the
        #: jaws and merely tilted there.
        self.topple: list[dict | None] = [None] * env.num_envs
        self.final_row: list[dict] = [{} for _ in range(env.num_envs)]
        #: The last row of each phase of the close after the pull (`--close`), by attempt.
        self.close_rows: list[list[dict]] = [[] for _ in range(env.num_envs)]
        self.arm_speed_peak = torch.zeros(env.num_envs, device=env.device)
        #: The combiner door demo's lidar frames, each (phase, reading deg or nan, the simulator's door deg), and the
        #: last frame's stamp seen, so each frame is counted once although every step's row repeats it.
        self.lidar_frames: list[list] = [[] for _ in range(env.num_envs)]
        self._lidar_stamp = [None] * env.num_envs

    def update(self, step, done, rows):
        import math

        import torch

        from unifp_isaaclab import interface

        env = self.env
        self.fell |= done
        commands = list(env.commands)
        if any(command is None for command in commands[:self.count]):
            return

        tip, goal = env._tip_pos(), env._goal_world()
        centre, approach, jaw = env._hand_frame()
        tip_error = torch.norm(tip - goal, dim=1)
        # Where the jaw centre was asked to be, carried from the script's frame into the world in
        # one batched call rather than one per environment.
        wanted = torch.as_tensor([command.point for command in commands],
                                 dtype=torch.float32, device=env.device)
        wanted_w = (env.scene.env_origins + env._spawn_pos
                    + interface.quat_apply(env._spawn_yaw_quat, wanted))
        delta = centre - wanted_w
        jaw_error = torch.norm(delta, dim=1)
        # A grasp does not care where the error is. Along the approach it is a hand that stopped
        # short or went deep, which a parallel jaw forgives; across it, it is a finger through the
        # object. The jaws clear this cup by 11 mm a side and the lever's bar by 12 mm.
        approach_cmd = torch.as_tensor([command.approach for command in commands],
                                       dtype=torch.float32, device=env.device)
        approach_cmd = interface.quat_apply(env._spawn_yaw_quat, approach_cmd)
        approach_cmd = approach_cmd / approach_cmd.norm(dim=-1, keepdim=True).clamp(min=1e-9)
        along = (delta * approach_cmd).sum(-1)
        lateral = torch.norm(delta - along.unsqueeze(-1) * approach_cmd, dim=1)
        elevation = torch.rad2deg(torch.asin(approach[:, 2].clamp(-1.0, 1.0)))
        tilt = torch.rad2deg(torch.asin(jaw[:, 2].abs().clamp(0.0, 1.0)))
        base = env._robot.data.root_pos_w - env.scene.env_origins
        base_z = base[:, 2]
        # Where the base has got to, against where it was when the attempt started. UniFP's goal
        # sphere is centred on the robot's own x,y, so a base that walks carries the commanded
        # goal with it -- and a demo's target is a point in the room, not on the robot.
        base_shift = torch.norm(base[:, :2] - env._spawn_pos[:, :2], dim=1)
        # The same, signed in the robot's spawn frame (forward, left), and the trunk's roll and pitch:
        # which way the body leans into the work.
        shift = torch.zeros_like(base)
        shift[:, :2] = base[:, :2] - env._spawn_pos[:, :2]
        shift = interface.quat_rotate_inverse(env._spawn_yaw_quat, shift)
        body_roll, body_pitch = _roll_pitch_deg(env._robot.data.root_quat_w)
        # Fastest arm joint this step. The D1's measured single-command ceiling is 1.21-1.29 rad/s
        # (F-033) and a 10 Hz stream through its firmware planner manages about 0.8 (F-046);
        # UniFP's port drives the arm with its own PD and neither model, so this is the number
        # that says whether a time here could be a time on the bench.
        arm_speed = env._dof()[1][:, 12:interface.NUM_ACTIONS].abs().max(dim=1).values
        arm_q_deg = torch.rad2deg(env._dof()[0][:, 12:interface.NUM_ACTIONS])
        base_yaw =torch.rad2deg(interface.yaw_from_quat(env._robot.data.root_quat_w)
                                 - interface.yaw_from_quat(env._spawn_yaw_quat))
        truth = env.cup_state() if self.task == "cup" else env.box_state()
        contact = env.contact_state()
        mech = env.mech_state() if hasattr(env, "mech_state") else None
        time_s = round(step * interface.POLICY_DT, 3)

        self.arm_speed_peak = torch.maximum(self.arm_speed_peak, arm_speed)
        for index in range(self.count):
            command = commands[index]
            best = self.best[index]
            row = {
                "attempt": self.batch * env.num_envs + index, "step": step, "time_s": time_s,
                "phase": command.phase,
                "jaw_centre_error_m": round(float(jaw_error[index]), 5),
                "lateral_error_m": round(float(lateral[index]), 5),
                "along_error_m": round(float(along[index]), 5),
                "tip_goal_error_m": round(float(tip_error[index]), 5),
                "jaw_command_m": round(command.jaw_m, 5),
                "approach_elevation_deg": round(float(elevation[index]), 2),
                "jaw_tilt_deg": round(float(tilt[index]), 2),
                "base_height_m": round(float(base_z[index]), 4),
                "base_shift_m": round(float(base_shift[index]), 4),
                "base_yaw_deg": round(float(base_yaw[index]), 3),
                "base_forward_m": round(float(shift[index, 0]), 4),
                "base_left_m": round(float(shift[index, 1]), 4),
                "base_roll_deg": round(float(body_roll[index]), 2),
                "base_pitch_deg": round(float(body_pitch[index]), 2),
                "arm_speed_rad_s": round(float(arm_speed[index]), 3),
                "finger_contact_n": round(float(contact["finger_n"][index]), 3),
                "forearm_contact_n": round(float(contact["forearm_n"][index]), 3),
                "fell": int(bool(self.fell[index])),
                "step_wall_ms": round(getattr(self, "step_wall_ms", float("nan")), 2),
            }
            if self.task == "cup":
                row["cup_lift_m"] = round(float(truth["lift_m"][index]), 4)
                row["cup_tilt_deg"] = round(float(truth["tilt_deg"][index]), 2)
            else:
                row["door_deg"] = round(float(truth["door_deg"][index]), 2)
                row["handle_deg"] = round(float(truth["handle_deg"][index]), 2)
            if self.task == "door":
                # The walk: what the script commanded the base, and how far the base is from the pose it asked for.
                vx, vy, wz = env._base_cmd[index].tolist()
                row["base_cmd_vx_m_s"], row["base_cmd_vy_m_s"], row["base_cmd_wz_rad_s"] = (
                    round(vx, 4), round(vy, 4), round(wz, 4))
                pose_error, yaw_error = env.base_pose_error[index].tolist()
                row["base_pose_error_m"] = None if math.isnan(pose_error) else round(pose_error, 4)
                row["base_yaw_error_deg"] = None if math.isnan(yaw_error) else round(yaw_error, 2)
                # The goal as the policy is given it (UniFP's sphere: trained radius 0.30-0.58 m, pitch -45..60 deg,
                # yaw +/-72 deg) and the arm's six joints: what shape the arm takes for it while the dog walks.
                radius, pitch, yaw = env._goal_sphere[index].tolist()
                row["goal_radius_m"] = round(radius, 4)
                row["goal_pitch_deg"], row["goal_yaw_deg"] = round(math.degrees(pitch), 2), round(math.degrees(yaw), 2)
                for joint, angle in enumerate(arm_q_deg[index].tolist()):
                    row[f"arm_q{joint}_deg"] = round(angle, 2)
                if hasattr(env, "ik_status"):
                    # The decoupled baseline's arm: how its latest 10 Hz command was solved (`decoupled_arm.IKStep`).
                    row["ik_status"] = env.ik_status[index]
                if hasattr(env, "lidar_readings"):
                    # The combiner door demo: the MID-360's latest reading of the door, beside the script's command
                    # (the simulator's angle is `door_deg`). Repeated every step until the next 10 Hz frame.
                    reading = env.lidar_readings[index]
                    row["commanded_door_deg"] = round(env.commanded_door_deg(index), 2)
                    row["lidar_door_deg"] = round(reading.angle_deg, 2) if reading.ok else None
                    row["lidar_points"] = reading.points
                    row["lidar_rms_mm"] = round(1000 * reading.rms_m, 1) if reading.ok else None
                    row["lidar_offset_mm"] = round(1000 * reading.offset_m, 1) if reading.ok else None
                    stamp = env.lidar_times_s[index]
                    if math.isfinite(stamp) and stamp != self._lidar_stamp[index]:
                        self._lidar_stamp[index] = stamp
                        self.lidar_frames[index].append((command.phase, reading.angle_deg, env.lidar_truth_deg[index]))
                if row["base_pose_error_m"] is not None:
                    best["max_base_pose_error_m"] = max(best.get("max_base_pose_error_m", 0.0), row["base_pose_error_m"])
                    best["max_base_yaw_error_deg"] = max(best.get("max_base_yaw_error_deg", 0.0),
                                                         abs(row["base_yaw_error_deg"]))
            if mech is not None:
                row["claw_engaged"] = int(bool(mech["claw_engaged"][index] > 0.5))
                row["claw_force_n"] = round(float(mech["claw_force_n"][index]), 3)
                row["force_cmd_n"] = round(float(mech["force_cmd_n"][index]), 3)
                for key in ("bar_along_m", "bar_depth_m", "bar_across_m", "bar_tilt_deg"):
                    if key in mech:
                        row[key] = round(float(mech[key][index]), 4)
            rows.append(row)

            for key in ("claw_force_n", "force_cmd_n"):
                if key in row:
                    best[f"peak_{key}"] = max(best.get(f"peak_{key}", 0.0), row[key])
            for key in ("cup_lift_m", "door_deg", "handle_deg"):
                if key in row:
                    best[f"max_{key}"] = max(best.get(f"max_{key}", -1e9), row[key])
                    best[f"final_{key}"] = row[key]
            best["final_phase"] = command.phase
            self.phase_start_s[index].setdefault(command.phase, time_s)
            self.final_row[index] = row
            if "door_deg" in row and command.phase in ("push_hold", "re_back"):
                rows_ = self.close_rows[index]
                if rows_ and rows_[-1]["phase"] == command.phase:
                    rows_[-1] = row
                else:
                    rows_.append(row)
            if (self.task == "cup" and self.topple[index] is None
                    and row["cup_tilt_deg"] > 60.0 and row["cup_lift_m"] < 0.02):
                self.topple[index] = {"phase": command.phase, "time_s": time_s}
            if command.phase in self.SCORED_PHASES:
                bucket = self.scored[index].setdefault(command.phase, [])
                bucket.append((row["jaw_centre_error_m"], row["jaw_tilt_deg"],
                               row["approach_elevation_deg"], row["lateral_error_m"],
                               row["forearm_contact_n"], row["finger_contact_n"],
                               row["base_shift_m"]))
                self.last[index][command.phase] = row

    def _perception_record(self, index: int) -> dict:
        """The combiner door demo's look and its lidar's reading of the door, for one attempt."""
        import math

        frames = [f for f in self.lidar_frames[index] if f[0] in ("grip", "pull", "hold")]
        read = [f for f in frames if math.isfinite(f[1])]
        errors = sorted(abs(f[1] - f[2]) for f in read)
        pick = lambda q: round(errors[min(len(errors) - 1, int(q * len(errors)))], 2) if errors else None
        end = self.last[index].get("hold", {}).get("lidar_door_deg")
        return {"look": dict(self.env.look_records[index]),
                "lidar": {"frames_grip_pull_hold": len(frames), "read": len(read),
                          "error_deg_median": pick(0.5), "error_deg_p90": pick(0.9),
                          "error_deg_max": round(errors[-1], 2) if errors else None},
                "lidar_door_at_end_of_hold_deg": end,
                "verified_open": end is not None and end >= props.DOOR_PULL_SUCCESS_DEG}

    def finish(self) -> list[dict]:
        out = []
        for index in range(self.count):
            record = {
                "attempt": self.batch * self.env.num_envs + index,
                "fell": bool(self.fell[index]),
                "phase_start_s": self.phase_start_s[index],
            }
            record.update(self.best[index])
            record["arm_speed_peak_rad_s"] = round(float(self.arm_speed_peak[index]), 3)
            for phase, samples in self.scored[index].items():
                mean = lambda column: round(sum(s[column] for s in samples) / len(samples), 4)
                record[f"jaw_centre_error_{phase}_m"] = mean(0)
                record[f"lateral_error_{phase}_m"] = mean(3)
                record[f"jaw_tilt_{phase}_deg"] = mean(1)
                record[f"approach_elevation_{phase}_deg"] = mean(2)
                record[f"forearm_contact_{phase}_n"] = mean(4)
                record[f"finger_contact_{phase}_n"] = mean(5)
                record[f"base_shift_{phase}_m"] = mean(6)
                # The value at the moment the phase ends, which for a hold is the state the next
                # phase starts from -- and for `arrive` is the hand the jaws close from.
                final = self.last[index][phase]
                record[f"final_jaw_centre_error_{phase}_m"] = final["jaw_centre_error_m"]
                record[f"final_lateral_error_{phase}_m"] = final["lateral_error_m"]
            site = self.sites[index]
            final = self.final_row[index]
            if self.task == "cup":
                record["site"] = {"cup_xy_m": [round(v, 4) for v in site.cup_xy_m]}
                record["knocked_over"] = self.topple[index]
                record["final_cup_tilt_deg"] = final.get("cup_tilt_deg")
                record["final_finger_contact_n"] = final.get("finger_contact_n")
                # Picked up: off the table by the success height at the end of the hold, and still
                # between the fingers. Height alone is not enough -- a cup knocked flat sits one
                # radius up, and a cup swept off the table reads as a large negative.
                record["picked"] = bool(
                    record.get("final_cup_lift_m", 0.0) >= props.CUP_SUCCESS_LIFT_M
                    and (final.get("finger_contact_n") or 0.0) > 0.5)
                # Carried, but at what angle. A cup gripped by its wall rotates in the jaws; past
                # 45 degrees it would have emptied, which is not a pick anyone wants.
                record["spilled"] = bool(record["picked"]
                                         and (record["final_cup_tilt_deg"] or 0.0) > 45.0)
                record["success"] = bool(record["picked"] and not record["spilled"]
                                         and not record["fell"])
            elif self.task == "door":
                record["site"] = {"distance_m": round(site.distance_m, 4),
                                  "bearing_deg": round(site.bearing_deg, 2),
                                  "yaw_deg": round(site.yaw_deg, 2)}
                record["door_at_end_of_hold_deg"] = self.last[index].get("hold", {}).get("door_deg")
                record["base_yaw_final_deg"] = final.get("base_yaw_deg")
                record["base_forward_final_m"] = final.get("base_forward_m")
                record["base_left_final_m"] = final.get("base_left_m")
                if hasattr(self.env, "ik_counts"):
                    record["ik_counts"] = dict(self.env.ik_counts[index])
                    record["grasp"] = self.env.arms[index].grasp_note
                if hasattr(self.env, "look_records"):
                    record.update(self._perception_record(index))
                if getattr(self.env, "closes", None) and self.env.closes[index].get("looking"):
                    # The close after the pull (`--close`): the angle it worked from against the simulator's, and the
                    # door when it let go and at the end.
                    close = {k: v for k, v in self.env.closes[index].items() if k not in ("readings", "looking")}
                    close["readings"] = len(self.env.closes[index].get("readings", []))
                    door_at = {r["phase"]: r["door_deg"] for r in self.close_rows[index]}
                    close["door_at_end_of_push_hold_deg"] = door_at.get("push_hold")
                    close["door_final_deg"] = final.get("door_deg")
                    record["close"] = close
                # Swung open: the door at the success angle when the hold ends, the jaws still on it, and the robot
                # still up. Its peak alone would count a door flung past and swinging back.
                record["success"] = bool((record["door_at_end_of_hold_deg"] or 0.0) >= props.DOOR_PULL_SUCCESS_DEG
                                         and not record["fell"])
            else:
                record["site"] = {"distance_m": round(site.distance_m, 4),
                                  "bearing_deg": round(site.bearing_deg, 2),
                                  "yaw_deg": round(site.yaw_deg, 2)}
                record["latch_released"] = bool(
                    record.get("max_handle_deg", 0.0) >= GEOMETRY.handle_release_deg)
                if hasattr(self.env, "claw"):
                    capture = float(self.env.claw_capture[index])
                    record["claw_capture_m"] = None if math.isnan(capture) else round(capture, 4)
                    record["claw_hooked"] = bool(self.env.claw_hooked[index])
                    record["claw_torn"] = bool(self.env.claw.torn[index])
                    record["claw_lost"] = bool(self.env.claw_lost[index])
                record["success"] = bool(record.get("max_door_deg", 0.0) >= props.DOOR_SUCCESS_DEG
                                         and not record["fell"])
            out.append(record)
        return out


def _roll_pitch_deg(quat: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """Roll (positive: left side up) and pitch (positive: nose down) of (w, x, y, z) quaternions, degrees."""
    import torch

    w, x, y, z = quat.unbind(-1)
    roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = torch.asin((2 * (w * y - z * x)).clamp(-1.0, 1.0))
    return torch.rad2deg(roll), torch.rad2deg(pitch)


def _tally(values) -> dict:
    counts: dict[str, int] = {}
    for value in values:
        counts[value] = counts.get(value, 0) + 1
    return dict(sorted(counts.items(), key=lambda item: -item[1]))


def _summarise(attempts: list[dict], task: str) -> dict:
    if not attempts:
        return {}

    def median(values):
        ordered = sorted(v for v in values if v is not None)
        return round(ordered[len(ordered) // 2], 5) if ordered else None

    summary = {
        "attempts": len(attempts),
        "successes": sum(1 for a in attempts if a["success"]),
        "falls": sum(1 for a in attempts if a["fell"]),
        "arm_speed_peak_rad_s_median": median([a.get("arm_speed_peak_rad_s") for a in attempts]),
    }
    if task == "cup":
        summary.update({
            "picked": sum(1 for a in attempts if a.get("picked")),
            "spilled": sum(1 for a in attempts if a.get("spilled")),
            "knocked_over": sum(1 for a in attempts if a.get("knocked_over")),
            "knocked_over_in": _tally(a["knocked_over"]["phase"]
                                      for a in attempts if a.get("knocked_over")),
            "final_cup_tilt_deg_median": median([a.get("final_cup_tilt_deg") for a in attempts]),
            "final_lift_m_median": median([a.get("final_cup_lift_m") for a in attempts]),
            "max_lift_m_median": median([a.get("max_cup_lift_m") for a in attempts]),
            "jaw_centre_error_arrive_m_median": median(
                [a.get("final_jaw_centre_error_arrive_m") for a in attempts]),
            "lateral_error_arrive_m_median": median(
                [a.get("final_lateral_error_arrive_m") for a in attempts]),
            "jaw_tilt_arrive_deg_median": median([a.get("jaw_tilt_arrive_deg") for a in attempts]),
            "approach_elevation_arrive_deg_median": median(
                [a.get("approach_elevation_arrive_deg") for a in attempts]),
            "forearm_contact_arrive_n_median": median(
                [a.get("forearm_contact_arrive_n") for a in attempts]),
            "base_shift_arrive_m_median": median([a.get("base_shift_arrive_m") for a in attempts]),
        })
    elif task == "door":
        summary.update({
            "jaw_centre_error_arrive_m_median": median(
                [a.get("final_jaw_centre_error_arrive_m") for a in attempts]),
            "lateral_error_arrive_m_median": median(
                [a.get("final_lateral_error_arrive_m") for a in attempts]),
            "finger_contact_grip_n_median": median([a.get("finger_contact_grip_n") for a in attempts]),
            "door_at_end_of_hold_deg_median": median([a.get("door_at_end_of_hold_deg") for a in attempts]),
            "door_at_end_of_hold_deg": sorted(a.get("door_at_end_of_hold_deg") or 0.0 for a in attempts),
            "max_door_deg_median": median([a.get("max_door_deg") for a in attempts]),
            "jaw_centre_error_pull_m_median": median([a.get("jaw_centre_error_pull_m") for a in attempts]),
            "finger_contact_pull_n_median": median([a.get("finger_contact_pull_n") for a in attempts]),
            "max_base_pose_error_m_median": median([a.get("max_base_pose_error_m") for a in attempts]),
            "max_base_yaw_error_deg_median": median([a.get("max_base_yaw_error_deg") for a in attempts]),
            "base_yaw_final_deg_median": median([a.get("base_yaw_final_deg") for a in attempts]),
        })
        if any("look" in a for a in attempts):
            looks = [a["look"] for a in attempts]
            lidar = [a["lidar"] for a in attempts]
            summary.update({
                "tag_found": sum(bool(look.get("found")) for look in looks),
                "look_site_position_mm_median": median([look.get("site_position_mm") for look in looks]),
                "look_site_position_mm_max": max((look.get("site_position_mm") or 0.0 for look in looks), default=None),
                "look_site_yaw_deg_median": median([look.get("site_yaw_deg") for look in looks]),
                "lidar_read_fraction_median": median([l["read"] / l["frames_grip_pull_hold"]
                                                      for l in lidar if l["frames_grip_pull_hold"]]),
                "lidar_error_deg_median": median([l["error_deg_median"] for l in lidar]),
                "lidar_error_deg_p90_max": max((l["error_deg_p90"] or 0.0 for l in lidar), default=None),
                "verified_open": sum(bool(a.get("verified_open")) for a in attempts),
                "verified_agrees_with_success": sum(bool(a.get("verified_open")) == bool(a.get("success"))
                                                    for a in attempts),
            })
    else:
        summary.update({
            "jaw_centre_error_arrive_m_median": median(
                [a.get("final_jaw_centre_error_arrive_m") for a in attempts]),
            "lateral_error_arrive_m_median": median(
                [a.get("final_lateral_error_arrive_m") for a in attempts]),
            "jaw_tilt_arrive_deg_median": median([a.get("jaw_tilt_arrive_deg") for a in attempts]),
            "forearm_contact_arrive_n_median": median(
                [a.get("forearm_contact_arrive_n") for a in attempts]),
            "finger_contact_grip_n_median": median(
                [a.get("finger_contact_grip_n") for a in attempts]),
            "max_door_deg_median": median([a.get("max_door_deg") for a in attempts]),
            "max_handle_deg_median": median([a.get("max_handle_deg") for a in attempts]),
            "latch_released": sum(1 for a in attempts if a.get("latch_released")),
            "jaw_centre_error_grip_m_median": median(
                [a.get("jaw_centre_error_grip_m") for a in attempts]),
            "jaw_tilt_grip_deg_median": median([a.get("jaw_tilt_grip_deg") for a in attempts]),
        })
        if any("claw_hooked" in a for a in attempts):
            summary.update({
                "claw_hooked": sum(1 for a in attempts if a.get("claw_hooked")),
                "claw_torn": sum(1 for a in attempts if a.get("claw_torn")),
                "claw_lost": sum(1 for a in attempts if a.get("claw_lost")),
                "claw_capture_m_median": median([a.get("claw_capture_m") for a in attempts]),
                "peak_claw_force_n_median": median([a.get("peak_claw_force_n") for a in attempts]),
                "peak_force_cmd_n_median": median([a.get("peak_force_cmd_n") for a in attempts]),
            })
    return summary


if __name__ == "__main__":
    raise SystemExit(main())
