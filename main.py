"""Entry point: launch Isaac Sim, then hand off to sim.run().

Isaac Sim has to be up before anything under `isaaclab.*`, `omni.*`, `carb` or
`pxr` can be imported, which is why this file does nothing but argument parsing
and why the `import sim` sits below the launcher rather than at the top.
"""
from __future__ import annotations

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(
    description="Drive a Go2 with a welded-on D1 arm around a flat testing area."
)
parser.add_argument(
    "--task", type=str, default="Isaac-Velocity-Rough-Unitree-Go2-v0",
    help="Gym id to instantiate. Only the env class is used -- the config comes from flat_env_cfg.",
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of robots to simulate.")
parser.add_argument(
    "--arm_mass", type=float, default=None,
    help="Rescale the arm to this total mass in kg. The shipped URDF's inertials are a "
         "SolidWorks export of the shells alone and total only ~0.72 kg, which is far too "
         "light to perturb the gait. Omit to use the URDF's own values.",
)
parser.add_argument(
    "--no_arm", action="store_true",
    help="Baseline: run the bare Go2 on the same flat course, for comparison.",
)
parser.add_argument(
    "--selftest", type=float, default=0.0, metavar="SECONDS",
    help="Skip teleop: walk forward for this long, print gait stats and exit. "
         "Pair with --headless to compare arm masses without a viewport.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import sim  # noqa: E402  -- must follow AppLauncher

sim.run(args_cli, simulation_app)
simulation_app.close()
