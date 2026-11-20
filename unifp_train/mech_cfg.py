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

#: Short levers (2026-09-25, after F-110..F-113): a handle on a spindle, turned *down* about a level axis that
#: points roughly at the robot, like a combiner box's door handle. The draw above never makes one: its hinges are
#: 15-45 cm doors and lids, and a push goes down onto a top face in only ~7.5% of mechanisms. Off unless
#: `--lever_fraction` asks for it, so earlier runs reproduce. A lever's radius and turn, how far it may already
#: rest into its turn (negative: resting raised), and how far its spindle is turned from pointing at the robot
#: either way -- where the robot stands against the box (`mechanism.lever_axes`).
LEVER_FRACTION = 0.0
LEVER_RADIUS_M = (0.05, 0.12)
LEVER_ANGLE_RAD = (math.radians(40.0), math.radians(75.0))
LEVER_REST_RAD = (math.radians(-20.0), math.radians(20.0))
LEVER_STANCE_RAD = math.radians(45.0)
#: Where a lever's handle may be, as a pitch about the goal sphere's centre (0.49 m up), rad. The pull task's placements
#: stop at 0.45 rad (a handle at most ~0.74 m up, most below 0.6 m); a lever above the robot's shoulder is turned down
#: by *pulling* -- the arm in tension, the body's weight hanging from it -- not by leaning onto it, so levers reach up
#: to 0.95 rad (~0.96 m at the sphere's rim), inside UniFP's trained 60 degrees (Lukas, 2026-09-25). None before
#: `mech_law_lever_high`: the first lever run drew the pull task's placements.
LEVER_HANDLE_PITCH_RANGE = (-0.75, 0.95)
#: A lever's peak force at the handle, N, drawn here rather than under the curriculum's ceiling, and never counted
#: at its frontier: the D1's arm alone holds a median 9 N downward and at most ~33 N in its best posture
#: (`results/week_02/figures/direction_capacity.json`), so a lever drawn at the 80 N frontier is one nothing turns.
#: Half the draws land in the top 30%, as for the other mechanisms.
LEVER_PEAK_N = (3.0, 35.0)

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
    # Force across the path, squared, per (`OFFPATH_SCALE_N`)^2 (`mech_rewards.mech_offpath`). Off unless
    # `--offpath_weight` sets it, so earlier runs reproduce.
    "mech_offpath": 0.0,
}
#: `mech_offpath`'s scale: at weight -2, 20 N across the path costs 2 per second, 40 N costs 8 (being on the reference
#: pays up to 4).
OFFPATH_SCALE_N = 20.0
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
    # Opt-in (`--mech_kinds lever ...`), frozen 2026-09-25 before the lever training was scored: a combiner box's
    # door handle. Grasped 8 cm out on a spindle that points at the robot, turned down 60 degrees against a return
    # spring preloaded to a fifth of its force at the stop (the combiner's quarter-torque preload at 0 degrees of
    # 60), a little stiction. Square to the box, then with the robot 30 degrees to the spindle's side (the late
    # turn a pull) and to the tip's side (a push). Scored at `LEVER_EVAL_LEVELS_N`, not the 10-80 N levels.
    "lever": {"hinge": True, "opening": "down", "radius_m": 0.08, "angle_rad": math.radians(60.0),
              "axis": "lever", "stance_deg": 0.0, "weights": (0.85, 0.15, 0.0), "preload": 0.2, "kinetic": 0.7},
    "lever_spindle_side": {"hinge": True, "opening": "down", "radius_m": 0.08, "angle_rad": math.radians(60.0),
                           "axis": "lever", "stance_deg": 30.0, "weights": (0.85, 0.15, 0.0), "preload": 0.2,
                           "kinetic": 0.7},
    "lever_tip_side": {"hinge": True, "opening": "down", "radius_m": 0.08, "angle_rad": math.radians(60.0),
                       "axis": "lever", "stance_deg": -30.0, "weights": (0.85, 0.15, 0.0), "preload": 0.2,
                       "kinetic": 0.7},
}
LEVER_EVAL_KINDS = ("lever", "lever_spindle_side", "lever_tip_side")
#: The high-handle lever test (`mech_eval --lever_high`), frozen 2026-09-25 before any policy was scored on it: the same
#: levers with the handle at and above the combiner demo's 0.68 m, 0.35 m ahead of the goal sphere's centre (the reach
#: that keeps 0.9 m inside its 0.58 m radius).
LEVER_HIGH_HEIGHTS_M = (0.60, 0.70, 0.80, 0.90)
LEVER_HIGH_BEARINGS_RAD = (-0.4, 0.0, 0.4)
LEVER_HIGH_REACH_M = 0.35
LEVER_EVAL_LEVELS_N = (5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0)
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


def lever_high_handle_spheres() -> list[tuple[float, float, float]]:
    """(radius, pitch, yaw) about the goal-sphere centre for each high-handle lever placement."""
    out = []
    for height in LEVER_HIGH_HEIGHTS_M:
        for bearing in LEVER_HIGH_BEARINGS_RAD:
            dz = height - hook_cfg.GOAL_CENTRE_HEIGHT_M
            out.append((math.hypot(LEVER_HIGH_REACH_M, dz), math.atan2(dz, LEVER_HIGH_REACH_M), bearing))
    return out


def test_handle_spheres() -> list[tuple[float, float, float]]:
    """(radius, pitch, yaw) about the goal-sphere centre for each held-out test placement."""
    out = []
    for height in TEST_HEIGHTS_M:
        for bearing in TEST_BEARINGS_RAD:
            dz = height - hook_cfg.GOAL_CENTRE_HEIGHT_M
            out.append((math.hypot(TEST_REACH_M, dz), math.atan2(dz, TEST_REACH_M), bearing))
    return out


#: S4 (`docs/tool_paper_plan.md`, 2026-11-18): T1 on the deployable models -- the D1 link, `66600`'s legs -- is scored
#: on placements nobody has scored before, drawn on new streams across the development and test grids' span (height
#: 0.20-0.60 m, bearing +-0.55 rad, reach 0.42-0.45 m from the goal sphere's centre). Validation runs at the development
#: conditions, test at the held-out ones (`TEST_CONDITIONS`); kinds and levels are the comparisons' four and 10-80 N.
#: Frozen in `results/manifests/mech_t1_v2.json` (`s4_manifest`, written once); the evaluator reads the file.
S4_MANIFEST = "results/manifests/mech_t1_v2.json"
S4_SEEDS = {"validation": 20261127, "test": 20261128}
S4_PLACEMENTS = {"validation": 12, "test": 16}
S4_HEIGHT_RANGE_M = (0.20, 0.60)
S4_BEARING_RANGE_RAD = (-0.55, 0.55)
S4_REACH_RANGE_M = (0.42, 0.45)
#: The task's force budget (the law's cap, `FORCE_LAW_MAX_N`), and a violation of it: the drive on the handle above the
#: budget on `BUDGET_HOLD_STEPS` consecutive policy steps (0.1 s) during the push, so a single contact spike does not
#: count. Stated 2026-11-18 before any S4 policy was scored.
BUDGET_N = 80.0
BUDGET_HOLD_STEPS = 5


def s4_draw_placements(seed: int, n: int) -> list[list[float]]:
    """`n` (radius, pitch, yaw) placements about the goal sphere's centre on their own stream."""
    import torch

    g = torch.Generator().manual_seed(seed)
    u = lambda lo_hi: lo_hi[0] + float(torch.rand(1, generator=g)) * (lo_hi[1] - lo_hi[0])
    out = []
    for _ in range(n):
        height, bearing, reach = u(S4_HEIGHT_RANGE_M), u(S4_BEARING_RANGE_RAD), u(S4_REACH_RANGE_M)
        dz = height - hook_cfg.GOAL_CENTRE_HEIGHT_M
        out.append([round(math.hypot(reach, dz), 6), round(math.atan2(dz, reach), 6), round(bearing, 6)])
    return out


def s4_manifest() -> dict:
    return {"name": "mech_t1_v2", "version": 2, "written": "2026-11-18",
            "plan": "docs/tool_paper_plan.md S4 (H1, E1); code unifp_train/mech_cfg.py (s4_manifest)",
            "supersedes": "the development grid (hook_cfg.eval_handle_spheres) and the 2026-09-25 test set "
                          "(mech_cfg.TEST_*), both spent by F-105 to F-110",
            "placement_units": "(radius m, pitch rad, yaw rad) about the goal sphere's centre",
            "kinds": list(EVAL_DEFAULT_KINDS), "levels_n": list(EVAL_LEVELS_N),
            "budget_n": BUDGET_N, "budget_hold_steps": BUDGET_HOLD_STEPS,
            "sets": {name: {"seed": S4_SEEDS[name], "conditions": dict(DEV_CONDITIONS if name == "validation"
                                                                       else TEST_CONDITIONS),
                            "placements": s4_draw_placements(S4_SEEDS[name], S4_PLACEMENTS[name])}
                     for name in ("validation", "test")}}


def s4_set(name: str, manifest: str | None = None) -> tuple[list[tuple[float, float, float]], dict]:
    """A frozen S4 set's placements and conditions, from the manifest file (`S4_MANIFEST`, or `S4B_MANIFEST`)."""
    import json
    from pathlib import Path

    man = json.loads((Path(__file__).resolve().parents[1] / (manifest or S4_MANIFEST)).read_text())
    spec = man["sets"][name]
    return [tuple(p) for p in spec["placements"]], dict(spec["conditions"])


#: S4b (2026-11-19, the week 10 log): a T1 policy for the dog, trained with the bleed in the loop and from the dog's stance,
#: needs sets that F-294 and F-295 did not read. Drawn as S4's on new streams; the same counts, conditions, kinds, levels
#: and budget. Frozen in `results/manifests/mech_t1_v3.json` (`s4b_manifest`, written once).
S4B_MANIFEST = "results/manifests/mech_t1_v3.json"
S4B_SEEDS = {"validation": 20261201, "test": 20261202}


def s4b_manifest() -> dict:
    out = s4_manifest()
    out.update({"name": "mech_t1_v3", "version": 3, "written": "2026-11-19",
                "plan": "S4b, the week 10 log 2026-11-19; code unifp_train/mech_cfg.py (s4b_manifest)",
                "supersedes": "mech_t1_v2 for anything after F-295 (its test set read by F-294 and F-295)"})
    out["sets"] = {name: {"seed": S4B_SEEDS[name], "conditions": dict(DEV_CONDITIONS if name == "validation"
                                                                      else TEST_CONDITIONS),
                          "placements": s4_draw_placements(S4B_SEEDS[name], S4_PLACEMENTS[name])}
                   for name in ("validation", "test")}
    return out
