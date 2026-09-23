#!/usr/bin/env python3
"""Run UniFP's trained Go2+D1 policy on this repository's Isaac Lab model.

The policy in `unifp_go2d1/` is trained on the legacy stack (Isaac Gym Preview 4, Python 3.8).
This runs the same checkpoint on the Isaac Lab welded Go2+D1 instead. Nothing is retrained and
nothing is fine-tuned: the weights are loaded as they are, handed the observation they were
trained on, and their actions are applied through the torque law they were trained through
(`unifp_isaaclab/robot.py` argues for that setup, and lists what it does *not* reproduce).

    # no simulator: check the interface and that the checkpoint loads against it
    ./run_unifp_isaaclab.py check --checkpoint ~/thesis_b_legacy/UniFP/logs/go2d1_pos_force/Sep18_11-10-15_/model_48800.pt

    # watch it
    ./run_unifp_isaaclab.py play --checkpoint <model_48800.pt> --command 0.5 0 0

    # headless, recorded
    ./run_unifp_isaaclab.py play --checkpoint <model_48800.pt> --headless --steps 1500 \
        --out logs/unifp_isaaclab/$(date -u +%Y%m%dT%H%M%S)_play48800

Then, as for every run in this repository:

    ./dashboard.py record <out dir> --title "..."

What a result here does and does not show: agreement means the policy's behaviour survives the
change of simulator, which is a precondition for it meaning anything about hardware. Disagreement
is not by itself evidence about the policy -- the two robot models are built from different files
and the terrain differs -- so read `unifp_isaaclab/robot.py` before concluding anything from a gap.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent

DEFAULT_CHECKPOINT = os.path.expanduser(
    "~/thesis_b_legacy/UniFP/logs/go2d1_pos_force/Sep18_11-10-15_/model_48800.pt")


def git_output(*args, default=""):
    result = subprocess.run(["git", "-C", str(ROOT), *args], capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else default


def stack_versions() -> dict:
    """The simulator stack a run actually executed on.

    Recorded per run because it is not reconstructible afterwards and it changes results: the
    same checkpoint, robot and configuration gave a different tool-tip error on isaaclab 0.47.2
    than on 0.54.3, and 0.47.2 is no longer installed anywhere on this machine. A `git_commit`
    pins this repository, not the simulator under it.
    """
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
    parser.add_argument("mode", choices=("check", "play"),
                        help="'check' verifies the interface and the checkpoint with no simulator.")
    parser.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT,
                        help="UniFP model_<iteration>.pt to run.")
    parser.add_argument("--num_envs", type=int, default=1)
    parser.add_argument("--steps", type=int, default=1500, help="Policy steps at 50 Hz.")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--command", nargs=3, type=float, default=(0.0, 0.0, 0.0),
                        metavar=("VX", "VY", "WZ"),
                        help="Held base velocity command: forward and lateral m/s, yaw rad/s. "
                             "Inside the dead zone (|vx|<0.1, |vy|<0.1, |wz|<0.2) the policy is "
                             "being told to stand, and the gait phase is pinned to 0.")
    parser.add_argument("--ee_force_cmd", nargs=3, type=float, default=(0.0, 0.0, 0.0),
                        metavar=("FX", "FY", "FZ"),
                        help="Force the policy is ASKED to produce at the tool tip, N. Enters the "
                             "observation; nothing external is applied. Trained range is +/-8 N.")
    parser.add_argument("--ee_force_ext", nargs=3, type=float, default=(0.0, 0.0, 0.0),
                        metavar=("FX", "FY", "FZ"),
                        help="Force actually APPLIED to the tool tip as an external world-frame "
                             "wrench, N -- a disturbance to reject, not a command. The policy does "
                             "not observe it directly; it can only infer it.")
    parser.add_argument("--spawn_height", type=float, default=None,
                        help="Base height at reset. Defaults to UniFP's 0.35 m.")
    parser.add_argument("--robot_usd", help="Existing welded USD; otherwise it is rebuilt.")
    parser.add_argument("--zero_actions", action="store_true",
                        help="Discard the policy's actions and hold the default pose: the baseline "
                             "that says whether the robot stands at all under UniFP's gains.")
    parser.add_argument("--report_every", type=int, default=50, help="0 to print nothing per step.")
    parser.add_argument("--out", metavar="DIR",
                        help="Write run.json, trace.csv and summary.json here, for ./dashboard.py record.")
    return parser


def run_check(args) -> int:
    """Everything that can be verified without starting a simulator."""
    sys.path.insert(0, str(ROOT))
    import torch  # noqa: F401  (imported for the version report only)
    from unifp_isaaclab import interface
    from unifp_isaaclab.policy import UniFPPolicy

    report = {
        "checkpoint": os.path.abspath(args.checkpoint),
        "checkpoint_exists": os.path.isfile(args.checkpoint),
        "torch": torch.__version__,
        "cuda_available": torch.cuda.is_available(),
        "interface": {
            "num_single_obs": interface.NUM_SINGLE_OBS,
            "frame_stack": interface.FRAME_STACK,
            "num_obs": interface.NUM_OBS,
            "num_actions": interface.NUM_ACTIONS,
            "policy_rate_hz": round(1.0 / interface.POLICY_DT, 3),
            "physics_rate_hz": round(1.0 / interface.SIM_DT, 3),
        },
    }
    if report["checkpoint_exists"]:
        policy = UniFPPolicy(args.checkpoint)
        report["checkpoint_iteration"] = policy.iteration
        report["checkpoint_sha256"] = hashlib.sha256(Path(args.checkpoint).read_bytes()).hexdigest()
        report["network"] = {
            "num_obs": policy.num_obs,
            "num_single_obs": policy.num_single_obs,
            "num_latent": policy.num_latent,
            "num_actions": policy.num_actions,
        }
        # A shape check is not a behaviour check, but it is the one that fails loudly.
        obs = torch.zeros(2, interface.NUM_OBS)
        report["forward_pass_ok"] = tuple(policy.act(obs).shape) == (2, interface.NUM_ACTIONS)
    report["ready"] = bool(report["checkpoint_exists"] and report.get("forward_pass_ok"))
    print(json.dumps(report, indent=2), flush=True)
    return 0 if report["ready"] else 1


def write_run_json(args, out_dir: Path, robot_usd: str, summary: dict | None,
                   status: str, error: str | None) -> None:
    """The same fields the other stacks write, so the dashboard lines the runs up."""
    meta = {
        "mode": "playback",
        "seed": args.seed,
        "status": status,
        "error": error,
        "arguments": {"num_envs": args.num_envs, "iterations": None, "steps": args.steps},
        "git_commit": git_output("rev-parse", "HEAD"),
        "git_status": git_output("status", "--porcelain"),
        "stack": "isaaclab/unifp_port",
        "stack_versions": stack_versions(),
        "unifp_commit": "68847a070f88d731058c3d8476929bc3b205f5bd",
        "command": " ".join(sys.argv),
        "notes": f"UniFP {os.path.basename(args.checkpoint)} run on the Isaac Lab welded Go2+D1 "
                 f"(sim-to-sim); base command {tuple(args.command)}, "
                 f"ee force cmd {tuple(args.ee_force_cmd)} N, ext {tuple(args.ee_force_ext)} N",
        "checkpoint": os.path.abspath(args.checkpoint),
        "checkpoint_sha256": hashlib.sha256(Path(args.checkpoint).read_bytes()).hexdigest(),
        "robot_usd": robot_usd,
        "robot_usd_sha256": hashlib.sha256(Path(robot_usd).read_bytes()).hexdigest(),
        "trace": "trace.csv",
    }
    if summary:
        meta["articulation_mass_kg"] = summary["model"]["total_mass_kg"]
        meta["summary"] = {k: v for k, v in summary.items() if k != "model"}
    (out_dir / "run.json").write_text(json.dumps(meta, indent=2) + "\n")


def main() -> int:
    parser = build_parser()
    if "--help" in sys.argv or "-h" in sys.argv or len(sys.argv) == 1:
        parser.parse_args()
        return 0
    known, _ = parser.parse_known_args()
    if known.mode == "check":
        return run_check(parser.parse_args())

    # Isaac Sim has to be up before anything under isaaclab.* can be imported, so the app
    # launcher's arguments are merged into this parser and the real imports happen below it.
    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.spawn_height is None:
        args.spawn_height = 0.35

    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    sys.path.insert(0, str(ROOT))
    from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR  # noqa: E402
    from weld import build_welded_robot_usd  # noqa: E402
    from unifp_isaaclab import rollout  # noqa: E402

    out_dir = Path(args.out) if args.out else None
    if out_dir:
        out_dir.mkdir(parents=True, exist_ok=True)

    if args.robot_usd:
        robot_usd = str(Path(args.robot_usd).resolve())
    else:
        # Built once into `generated/`, not into each run directory. The URDF importer writes a
        # layered USD whose sublayers live in a `configuration/` folder beside it, and importing
        # into a fresh directory takes minutes where re-importing over an existing one takes
        # seconds. Provenance does not need the copy: `run.json` records the sha256 of the USD
        # actually loaded, which is what makes a run reproducible.
        destination = ROOT / "generated/go2_d1.usd"
        robot_usd = build_welded_robot_usd(
            go2_usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd",
            d1_urdf_path=str(ROOT / "d1_arm/d1.urdf"),
            out_usd_path=str(destination),
            # Same mount and same published D1-550 mass as every other task in this repository,
            # so the Isaac Lab robot is the one the rest of the evidence is about.
            mount_pos=(0.0, 0.0, 0.08), arm_mass_kg=3.152,
        ).usd_path
    args.robot_usd = robot_usd

    summary, status, error = None, "finished", None
    try:
        summary = rollout.run(args, simulation_app)
        print(json.dumps(summary, indent=2), flush=True)
    except Exception as exception:  # recorded rather than swallowed: a failed run is still evidence
        status, error = "failed", f"{type(exception).__name__}: {exception}"
        raise
    finally:
        if out_dir:
            write_run_json(args, out_dir, robot_usd, summary, status, error)
            print(f"[unifp] wrote {out_dir}", flush=True)
        sys.stdout.flush()
        sys.stderr.flush()
        # Isaac Sim 5.1 reliably hangs in `SimulationApp.close()` on this machine for a headless
        # run -- the rollout finishes, the summary prints, and the process then sits at 100% of one
        # core indefinitely, holding ~1 GB of GPU memory. Everything this script produces is on
        # disk by now, so leave rather than wait: the kernel reclaims the context. Without this a
        # scripted sequence of runs deadlocks on the first one, and the GPU fills up with finished
        # jobs. Drop the `os._exit` if a later Isaac Sim version closes cleanly.
        os._exit(0 if status == "finished" else 1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
