"""Constants of the goal-commanded task: the robot holds a handle and is told where it should go.

The force-transmission task (`hook_cfg`) tells the policy what force to apply. On a real box nobody
knows that number -- how stiff the latch is, how much the door's closer resists -- and the task
layer knows only the geometry (from tags) and where the handle should end up. So here the command
is the goal alone: a reference point sliding along the mechanism's path. The resistance is drawn
per episode and never observed; the policy finds the force, and when the arm cannot make it alone,
the body has to.

There is no hardware to measure, so every mechanism number here is invented and drawn from a wide
range on purpose: the policy should meet stiff and soft, sticky and free, latched and not.

Reward weights are the config's numbers; `scaled_weights()` multiplies by the policy step, as
`task_cfg` does for UniFP's own terms.
"""
from __future__ import annotations

import math

from unifp_isaaclab import interface

from . import hook_cfg

# --- which environments do what ----------------------------------------------------------------

#: Fraction of episodes that grasp a mechanism. The rest reach through free space as UniFP's task
#: does, but with no pushes (see `mech_env`): in this task an external force is something to
#: overcome, never something to give way to.
MECHANISM_FRACTION = 0.8

# --- where the handle is, and when the claw takes it ------------------------------------------
#
# The same placements and settling rule as the pull task (`hook_cfg.HANDLE_*`, `ENGAGE_*`): the goal
# slides to a handle placement, and once the tool has settled there the grasp is made wherever the
# tool actually is. "The robot is already holding it" is the premise; the reach is how it gets there.

# --- the path ------------------------------------------------------------------------------------

#: Of the mechanisms, the fraction on a hinge (door, lid); the rest slide (drawer, latch, bolt, button).
HINGE_PROB = 0.4
#: Of the hinges, the fraction with a vertical axis (a door); the rest horizontal (a lid, a flap).
HINGE_VERTICAL_PROB = 0.6
#: Hinge radius at the handle, m, and how far it swings open, rad (20-80 degrees).
HINGE_RADIUS_M = (0.15, 0.45)
HINGE_ANGLE_RAD = (math.radians(20.0), math.radians(80.0))
#: Which way the handle first moves as the mechanism opens, relative to the robot: back toward it (a
#: drawer, a door that opens outward), away (a button, a push door), sideways (a bolt), up (a lifting
#: latch, a lid). In that order; they sum to one.
OPENING_PROBS = (0.45, 0.25, 0.15, 0.15)
OPENING_NAMES = ("pull", "push", "side", "up")
#: A slide's travel by opening direction, m. A button is short; a drawer long.
SLIDE_TRAVEL_M = {"pull": (0.03, 0.20), "push": (0.01, 0.05), "side": (0.02, 0.12), "up": (0.02, 0.10)}
#: Pull and push directions get the pull task's noise: turned up to this much either side, tilted in
#: this range, and a push goes down onto a top face with this probability.
YAW_NOISE_RAD = hook_cfg.PULL_YAW_NOISE_RAD
ELEVATION_RAD = hook_cfg.PULL_ELEVATION_RAD
PUSH_DOWN_PROB = 0.3

#: The whole path has to stay where the robot can work, in the goal sphere's frame (about a centre
#: 0.49 m above the ground under the base): this far from the centre, clear of the body (this far
#: ahead of it or this far to the side, a little inside the handle placements' own margins so a pull
#: can come toward the robot), and above the ground. A path that leaves the region is shortened by
#: `PATH_SHRINK` until it fits, down to `PATH_MIN_TRAVEL_M`.
PATH_RADIUS_M = (0.26, 0.64)
PATH_CLEAR_AHEAD_M = 0.22
PATH_CLEAR_SIDE_M = 0.18
PATH_MIN_HEIGHT_M = 0.08
PATH_SHRINK = 0.7
PATH_SHRINK_TRIES = 6
PATH_MIN_TRAVEL_M = 0.01
#: Points along the path checked against that region, as fractions of travel.
PATH_CHECK_FRACTIONS = (0.25, 0.5, 0.75, 1.0)

# --- the resistance ------------------------------------------------------------------------------
#
# Each mechanism is drawn by the largest force opening it needs from rest (its *peak*, N), then that
# peak is split at random between a spring, stiction and a latch (`mechanism.resistance_profile`).
# The peak is drawn under the curriculum's ceiling.

#: Fraction of mechanisms with a latch: held shut until the force passes it, then it lets go at once.
LATCH_PROB = 0.5
#: The spring's force at closed as a fraction of its force fully open (a closer's preload).
PRELOAD_FRACTION = (0.0, 0.8)
#: Kinetic friction as a fraction of stiction.
KINETIC_FRACTION = (0.5, 1.0)
#: Viscous damping, N*s/m, on top of the peak: a few newtons at the reference speeds.
DAMPING_N_S_M = (0.0, 20.0)
#: Moving mass at the handle, kg, log-uniform: a light flap to a door's effective mass at its edge.
MASS_KG = (0.3, 3.0)
#: The smallest peak drawn, N. Half the draws land in the top 30% of the ceiling, where the curriculum
#: is testing the policy.
PEAK_MIN_N = 2.0
FRONTIER_PROB = 0.5
FRONTIER_FRACTION = 0.7

# --- the grasp -----------------------------------------------------------------------------------

#: N/m, the claw on the handle, every direction. Drawn log-uniformly in training; 2,000 in
#: evaluation. The pull task's fixture ran up to 3,000 at the 5 ms physics step (smoke runs).
GRASP_STIFFNESS_RANGE = (1000.0, 3000.0)
GRASP_STIFFNESS = 2000.0
GRASP_DAMPING = 10.0
#: The grasp's spring force above which the handle is torn out of the claw and the episode ends.
#: Above the curriculum's 80 N ceiling, so any mechanism the curriculum draws can be opened without
#: tearing; below the 200 N a ringing explicit spring would show.
GRIP_N = 150.0

# --- the goal ------------------------------------------------------------------------------------

#: How fast the reference moves along the path, m/s, drawn per episode.
GOAL_SPEED_M_S = (0.05, 0.20)
#: How long each target is held once the reference reaches it, s.
GOAL_HOLD_S = (1.0, 3.0)
#: The first target opens the mechanism this fraction of its travel; later targets are anywhere.
GOAL_FIRST_FRACTION = (0.6, 1.0)

#: The virtual stiffness in UniFP's end-effector target, N/m. UniFP asks for `goal + force / 200`:
#: under an external force the target gives way, which is the right behaviour for its pushes and the
#: opposite of what opening a stiff latch needs. Here it is made rigid, so the target is the goal.
STIFF_KP = 1.0e6

# --- the curriculum ------------------------------------------------------------------------------

#: The ceiling on the peak force drawn. Raised by `CURRICULUM_STEP_N` whenever the frontier episodes
#: -- those whose peak is at least `FRONTIER_FRACTION` of the ceiling -- open often enough, and at most
#: once per `CURRICULUM_MIN_ITERATIONS`. It stops at 80 N, beyond the pull task's ~70 N plateau (F-103),
#: so where it stalls is itself a measurement.
CURRICULUM_START_N = 15.0
CURRICULUM_STEP_N = 5.0
CURRICULUM_MAX_N = 80.0
#: An episode *opened* if the handle reached this fraction of its first target.
OPENED_FRACTION = 0.8
CURRICULUM_PROMOTE_OPENED = 0.7
CURRICULUM_MIN_ITERATIONS = 50
CURRICULUM_EMA = 0.05

# --- the reward ----------------------------------------------------------------------------------

WEIGHTS = {
    # The objective: the handle at its reference, along the path.
    "mech_progress": 4.0,
    # Force toward the reference while behind it (v2; `mech_rewards.mech_push`). v1 had only the term
    # above, and its drive levelled off at ~30 N at every resistance, the arm's motors at their limits
    # and the estimator reading the force correctly: nothing paid for trying harder until a mechanism
    # gave, so it never found the posture the pull policy uses for 60 N.
    "mech_push": 2.0,
    # The arm's motors above `hook_cfg.ARM_TORQUE_MARGIN` of their limits while holding the handle:
    # the term that says "if it takes force, get it from the body". Same weight as the pull task,
    # so the two are comparable.
    "arm_torque_margin": -4.0,
    # Tearing the handle out of the claw ends the episode; paid once.
    "mech_torn": -100.0,
    # The handle moving faster than the reference plus a margin, squared: the lunge when a latch lets
    # go under a built-up force. 1 m/s over costs 5 per second.
    "mech_overspeed": -5.0,
    # The grasp force, squared, per 100 N: a small price on force that does nothing, so the policy
    # does not lean on a mechanism already at its stop. 60 N costs 0.36 per second against up to 4
    # for being where it was asked.
    "mech_effort": -1.0,
    # Only the hierarchical v2 variant weights this (`LAW_V2_WEIGHT_CHANGES`): drive past the force command.
    "mech_overforce": 0.0,
}
PROGRESS_SIGMA_FINE_M = 0.02
#: v3: for a short mechanism both widths are capped at these fractions of its travel
#: (`mech_rewards._progress`): a 1.5 cm button gets 4.5 mm and 1.5 cm, a drawer keeps 2 and 8 cm.
PROGRESS_SCALES_WITH_TRAVEL = True
PROGRESS_FINE_PER_TRAVEL = 0.3
PROGRESS_COARSE_PER_TRAVEL = 1.0
#: `mech_push` pays per this many newtons, and counts none past the cap.
PUSH_SCALE_N = 50.0
#: `mech_overforce` charges drive beyond the command by more than this.
OVERFORCE_MARGIN_N = 10.0
PUSH_CAP_N = 100.0
PROGRESS_SIGMA_COARSE_M = 0.08
OVERSPEED_MARGIN_M_S = 0.10
EFFORT_SCALE_N = 100.0

#: The critic's privileged block, the same 22-wide `mass_params` slot the pull task uses.
PRIVILEGED_WIDTH = hook_cfg.PRIVILEGED_WIDTH


#: The hierarchical variant (`--force_law`, v3) changes these. The force command now carries the push,
#: so the pull task's force-tracking term comes back at the pull task's weight, and `mech_push` -- the
#: end-to-end policy's substitute for a force command -- goes.
LAW_WEIGHT_CHANGES = {
    "fixture_force_tracking": 4.0,
    "mech_push": 0.0,
}
#: Law v2 (`--law_variant v2`), after F-108: the command becomes a limit the policy keeps to (`mech_overforce`,
#: with the cap drawn per episode from `LAW_V2_MAX_RANGE_N` so the policy meets many limits), and the lunge
#: after a release costs four times as much. Trained with the integral bleed (`LAW_V2_BLEED_S`).
LAW_V2_WEIGHT_CHANGES = {
    **LAW_WEIGHT_CHANGES,
    "mech_overforce": -1.0,
    "mech_overspeed": -20.0,
}
#: Law v3 (`--law_variant v3`), after v2's cap was still not a limit (at a 40 N command it opened 60-70 N
#: mechanisms): the outcome reward goes, so the policy is a pure force-follower and the task layer owns both the
#: escalation and the limit; the over-force price is four times v2's.
LAW_V3_WEIGHT_CHANGES = {
    **LAW_V2_WEIGHT_CHANGES,
    "mech_progress": 0.0,
    "mech_overforce": -4.0,
}
LAW_V2_MAX_RANGE_N = (30.0, 100.0)
LAW_V2_BLEED_S = 1.0


def scaled_weights(weights: dict[str, float] | None = None) -> dict[str, float]:
    """`WEIGHTS` (or `weights`) multiplied by the policy timestep, as the environment applies them."""
    return {name: weight * interface.POLICY_DT for name, weight in (WEIGHTS if weights is None else weights).items()}


# --- evaluation ------------------------------------------------------------------------------------
#
# Four fixed mechanisms, each at the pull task's fifteen handle placements (`hook_cfg`) and at every
# level in `EVAL_LEVELS_N`: one episode each. The target is fully open, approached at
# `EVAL_SPEED_M_S`. Nothing is drawn.

EVAL_LEVELS_N = (10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0)
EVAL_SPEED_M_S = 0.10
EVAL_EPISODE_S = 14.0
#: An episode *opened* when the handle reached this fraction of the travel, with the robot upright and
#: the handle still in the claw at that moment.
EVAL_OPENED_FRACTION = 0.9
EVAL_MASS_KG = 1.0
#: The force-commanded baseline's law (`mech_env.Go2D1MechEnv.set_force_law`): 10 N at 2.5 cm of lag,
#: plus 30 N/s more for every 3 cm it stays behind, up to 80 N. The pull policy was trained on commands
#: ramped at 25 N/s (`hook_cfg.LEVEL_RAMP_N_PER_S`), so the integral ramps at about that rate.
FORCE_LAW_KP_N_PER_M = 400.0
FORCE_LAW_KI_N_PER_M_S = 1000.0
FORCE_LAW_MAX_N = 80.0
#: The law's optional integral bleed (`--force_law_bleed`): within this of the reference, the integral decays.
FORCE_LAW_ARRIVED_M = 0.005
EVAL_DAMPING_N_S_M = 5.0

#: Each kind: a slide or a hinge, which way it opens, its travel (a hinge's as radius and angle), and
#: how its peak splits between spring, stiction and latch (`mechanism.resistance_profile`). A hinge's
#: axis is vertical (a door) or level (a lid), and its hinge on the handle's outer side ("outward", away
#: from the robot's centreline) or beyond the handle ("far"). A sideways slide opens outward.
EVAL_KINDS = {
    # A drawer: 12 cm straight back, sticky, a light return spring.
    "drawer": {"hinge": False, "opening": "pull", "travel_m": 0.12,
               "weights": (0.3, 0.7, 0.0), "preload": 0.5, "kinetic": 0.7},
    # A latch like a combiner box's: 3 cm back, held shut until it snaps.
    "latch": {"hinge": False, "opening": "pull", "travel_m": 0.03,
              "weights": (0.2, 0.2, 0.6), "preload": 0.5, "kinetic": 0.7},
    # A door with a closer: a vertical hinge 25 cm from the handle, 40 degrees, swinging toward the robot.
    "door": {"hinge": True, "opening": "pull", "radius_m": 0.25, "angle_rad": math.radians(40.0),
             "axis": "vertical", "hinge_side": "outward",
             "weights": (0.6, 0.4, 0.0), "preload": 0.5, "kinetic": 0.7},
    # A button like an emergency stop: 1.5 cm in, away from the robot, held until it snaps through.
    "button": {"hinge": False, "opening": "push", "travel_m": 0.015,
               "weights": (0.4, 0.0, 0.6), "preload": 0.5, "kinetic": 0.7},
    # Held out, opt-in (`--mech_kinds lid bolt`): directions training draws, geometries it does not.
    # A lid: a level hinge 20 cm beyond the handle, lifted 60 degrees, mostly its own weight (a spring with
    # a high preload stands in for gravity).
    "lid": {"hinge": True, "opening": "up", "radius_m": 0.20, "angle_rad": math.radians(60.0),
            "axis": "level", "hinge_side": "far",
            "weights": (0.7, 0.3, 0.0), "preload": 0.9, "kinetic": 0.7},
    # A bolt: 6 cm sideways, outward, nearly all stiction.
    "bolt": {"hinge": False, "opening": "side", "travel_m": 0.06,
             "weights": (0.1, 0.9, 0.0), "preload": 0.5, "kinetic": 0.6},
}
#: What `mech_eval` runs unless told otherwise: the four the comparisons are made on.
EVAL_DEFAULT_KINDS = ("drawer", "latch", "door", "button")

#: A held-out test condition (`mech_eval --mech_test`), frozen on 2026-09-25 before any policy was scored on it,
#: after the development evaluation above had been used to choose reward changes and checkpoints. Every value
#: differs from the development one: handle placements between and beyond the development grid (4 heights x 4
#: bearings at 0.42 m instead of 5 x 3 at 0.45 m), twice the moving mass, a softer grasp, a faster reference,
#: more damping. Kinds and levels are the development ones, so the numbers compare.
TEST_HEIGHTS_M = (0.25, 0.35, 0.45, 0.55)
TEST_BEARINGS_RAD = (-0.55, -0.2, 0.2, 0.55)
TEST_REACH_M = 0.42
TEST_CONDITIONS = {"mass_kg": 2.0, "grasp_k": 1400.0, "speed_m_s": 0.15, "damping_n_s_m": 12.0}
DEV_CONDITIONS = {"mass_kg": EVAL_MASS_KG, "grasp_k": GRASP_STIFFNESS, "speed_m_s": EVAL_SPEED_M_S,
                  "damping_n_s_m": EVAL_DAMPING_N_S_M}


def test_handle_spheres() -> list[tuple[float, float, float]]:
    """(radius, pitch, yaw) about the goal-sphere centre for each held-out test placement."""
    out = []
    for height in TEST_HEIGHTS_M:
        for bearing in TEST_BEARINGS_RAD:
            dz = height - hook_cfg.GOAL_CENTRE_HEIGHT_M
            out.append((math.hypot(TEST_REACH_M, dz), math.atan2(dz, TEST_REACH_M), bearing))
    return out
