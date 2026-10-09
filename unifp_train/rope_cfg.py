"""Constants of the rope-pull task: hold a steady pull on a rope, and survive it letting go.

The task (`rope_env.py`) is the hook task's pull (`hook_cfg`, F-103) with three things changed, each because the
question asked of it (Lukas, 2026-10-07) is different:

  * **a rope, not a ring.** The claw's interlocking teeth hold a rope, and the rope pulls the tool only along its own
    line (`rope.RopeTether`). The ring held the tool at a point and carried the arm's weight; a rope makes the arm
    hold its own weight and keep its own line, so "the arm drawn straight along the pull" (F-103) has to be the
    policy's doing.
  * **a steady force, not just a mean one.** The hook staircase scored the mean over a window; here the tension is
    read at the physics rate and its ripple, slack and tugs are priced in training and measured in evaluation
    (`rope_rewards.rope_ripple`, `rope_slack`; `rope_eval`).
  * **the anchor can give way.** In half the rope episodes the anchor lets go mid-pull -- snaps, slips to a residual
    drag, or fades -- and the task layer notices only after a delay, or not at all. The robot was leaning on that
    force; it must not fall, lunge or run.

Reward weights are the config's numbers; `scaled_weights()` multiplies by the policy step as `hook_cfg` does.
"""
from __future__ import annotations

import math

from unifp_isaaclab import interface

from . import hook_cfg

# --- which environments do what ----------------------------------------------------------------

#: Fraction of episodes that hold a rope. The rest are UniFP's free-space task, pushes and all, as in the hook task.
ROPE_FRACTION = 0.8

# --- the rope ----------------------------------------------------------------------------------

#: Rest length, m, drawn log-uniformly per episode. Short ropes hold the tool's line more stiffly (T / L across it);
#: long ones leave the arm almost free across the pull.
ROPE_LENGTH_RANGE_M = (0.3, 2.0)
#: N/m, log-uniform per episode in training. A rope's own stretch plus whatever it is tied to. Real rope is far stiffer
#: than this (a 1 m, 6 mm polyester line is ~10^5 N/m); the top is set by what the explicit 5 ms spring integrates
#: without ringing (smoke runs, 2026-10-07: see the Week 4 log), and a stiffer tether is where a steady force is
#: hardest, because a millimetre of body sway is k newtons per millimetre.
ROPE_STIFFNESS_RANGE = (500.0, 6000.0)
#: N*s/m along the rope.
ROPE_DAMPING = 10.0
#: The pull's direction: straight back toward the robot from where the claw holds the rope, turned and tilted by the
#: hook task's noise (`hook_cfg.PULL_*`).
PULL_YAW_NOISE_RAD = hook_cfg.PULL_YAW_NOISE_RAD
PULL_ELEVATION_RAD = hook_cfg.PULL_ELEVATION_RAD

# --- the force command ---------------------------------------------------------------------------

#: Levels held longer than the hook task's (1.5-4 s): steadiness over a hold is the point.
LEVEL_HOLD_S = (2.0, 5.0)
LEVEL_RAMP_N_PER_S = 25.0
LEVEL_ZERO_PROB = 0.1
LEVEL_FRONTIER_PROB = 0.5
LEVEL_FRONTIER_FRACTION = 0.7
#: The curriculum on the largest level drawn, as the hook task's. It starts at 30 N because the starting policy (hook
#: v4 `model_13000`) already holds 60 N on a ring; it rises by 5 N whenever the frontier's relative error averages below
#: `CURRICULUM_PROMOTE_REL_ERR`.
CURRICULUM_START_N = 30.0
CURRICULUM_STEP_N = 5.0
CURRICULUM_MAX_N = 60.0
#: v2 (2026-10-07): 0.25 and an average ten times slower than v1's 0.30 / 0.05. The average is updated on every reset
#: batch -- about 24 per iteration -- so at 0.05 it spanned about one iteration, and v1 promoted on noise to 50 N while
#: what it could pull fell to ~20 N; the frontier then sat 30 N away, where the force term has almost no slope.
#: From v4 the average is replaced by the error summed over the whole window since the last promotion, judged once
#: `CURRICULUM_MIN_ITERATIONS` have passed (`rope_env._update_curriculum`): restarted from one batch and moved at 0.005,
#: the average was that batch, and v3 went 30 -> 55 N in 239 iterations. `CURRICULUM_EMA` is v1-v3's only.
CURRICULUM_PROMOTE_REL_ERR = 0.25
CURRICULUM_MIN_ITERATIONS = 50
CURRICULUM_EMA = 0.005
CURRICULUM_MIN_ENGAGED_STEPS = 50

# --- the anchor giving way -------------------------------------------------------------------------

#: Of the rope episodes, the fraction whose anchor lets go.
RELEASE_PROB = 0.5
#: When, s after the rope was engaged, drawn uniformly; the release waits for a pull of at least
#: `RELEASE_MIN_COMMAND_N` (a rope letting go at 2 N is not the event this is about).
RELEASE_AFTER_S = (2.0, 11.0)
RELEASE_MIN_COMMAND_N = 10.0
#: How it lets go, as probabilities of (snap, slip, fade):
#:   * snap: the rope parts or is cut, capacity to 0 at once;
#:   * slip: the load breaks free and drags, capacity to `SLIP_RESIDUAL_FRACTION` of the tension within `SLIP_RAMP_S`;
#:   * fade: the anchor yields over `FADE_RAMP_S`, to 0.
RELEASE_KIND_PROBS = (0.5, 0.3, 0.2)
SLIP_RESIDUAL_FRACTION = (0.1, 0.5)
SLIP_RAMP_S = (0.0, 0.2)
FADE_RAMP_S = (0.1, 0.5)

#: The task layer notices the release this long after it happened (from the tool's sudden move, or the force
#: estimate collapsing), drawn uniformly, and then ramps its command to zero at `DETECT_RAMP_N_PER_S` and stops holding
#: the goal on the world: the goal stays where it then is relative to the body. `DETECT_NEVER_PROB` of releases are
#: never noticed, so the policy cannot count on the layer to stop pulling for it.
DETECT_DELAY_S = (0.1, 0.6)
DETECT_NEVER_PROB = 0.2
DETECT_RAMP_N_PER_S = 200.0
#: How long after a release the recovery terms are charged, s.
RECOVERY_WINDOW_S = 3.0

# --- terminations ----------------------------------------------------------------------------------

#: A rope episode also ends when the trunk touches the ground (contact force on `base` above this, N). UniFP's own
#: termination is on tilt alone (pitch past 1 rad, roll past 0.8), which a robot sitting down on its rear passes.
TRUNK_CONTACT_N = 1.0

# --- the reward ------------------------------------------------------------------------------------

WEIGHTS = {
    # The hook task's force objective, along the commanded direction, with a quarter of whatever goes across it, plus a
    # third, broad exponential (`FORCE_SIGMA_BROAD_N`, v2) so it still slopes 30 N short of the command. Charged while the
    # rope is held and the anchor holds: once it has let go the commanded force cannot be met.
    "fixture_force_tracking": 4.0,
    # The hook task's: load on the arm's motors above 70% of each joint's limit, while the rope is held.
    "arm_torque_margin": -4.0,
    # The hook task's v9 term: the tool's speed, squared, while the rope is held (released or not).
    "fixture_tool_speed": -5.0,
    # The tension's total variation at the physics rate against the command's, per policy step, in units of
    # `RIPPLE_SCALE_N`, squared and capped at `RIPPLE_CAP`. While the anchor holds. -1.0 in v1, where it was the largest
    # penalty in training (-1.3 to -1.4 per episode-second): mostly the exploration noise, which an arm drawn straight
    # along the rope turns into tension and a bent one does not, so it paid for bending the arm off the rope and pulling
    # less. Kept at -0.2 against high-frequency jitter; tugs are the force term's to price.
    "rope_ripple": -0.2,
    # The fraction of the step's physics substeps with the rope slack while at least `SLACK_COMMAND_N` is commanded:
    # the rope going slack and snapping taut again is what a tug is. While the anchor holds.
    "rope_slack": -2.0,
    # After a release, for `RECOVERY_WINDOW_S`: the base's horizontal speed squared plus a quarter of its roll and pitch
    # rates squared. UniFP's own velocity tracking and `ang_vel_xy` already ask for this mildly.
    "rope_upset": -2.0,
    # Once, on the step a rope episode ends by falling (tilt or trunk on the ground).
    "rope_fallen": -100.0,
    # The arm drawn taut along the rope: exp(-angle / `TAUT_ARM_SIGMA_RAD`) between the shoulder-to-jaw line and the
    # rope's, while the anchor holds and at least `TAUT_ARM_MIN_COMMAND_N` is commanded. Added 2026-10-07 after v1 pulled
    # with its arm 35 degrees off the rope (v1 ran without it); 1.0 from v2: the posture Lukas asked for, and the one
    # that carries force (F-101).
    "rope_taut_arm": 1.0,
    # Wobble: the force error's squared deviation from its own running mean (`WOBBLE_TAU_S`), averaged over the step's
    # physics substeps, in units of `WOBBLE_SCALE_N`, capped at `RIPPLE_CAP`. What the evaluation's std and tugs see, and
    # what `rope_ripple` (variation within 20 ms) and the force term (3 N fine width) barely price: v2 at 400 iterations
    # held 35-50 N with a 3.8 Hz, +-4 N spike train from Joint1 cycling between 52% and 100% of its limit. A steady
    # offset or a tracked ramp costs nothing. Off (0) in v1-v2.
    "rope_wobble": 0.0,
}

FORCE_SIGMA_FINE_N = 3.0
FORCE_SIGMA_COARSE_N = 12.0
FORCE_SIGMA_BROAD_N = 40.0
#: Shares of the three exponentials in the force term (the hook task's, and v1's, is 0.5 / 0.5 / 0).
FORCE_SHARES = (0.4, 0.4, 0.2)
RIPPLE_SCALE_N = 2.0
WOBBLE_TAU_S = 0.3
WOBBLE_SCALE_N = 2.0
TAUT_ARM_SIGMA_RAD = 0.25
TAUT_ARM_MIN_COMMAND_N = 5.0
RIPPLE_CAP = 4.0
SLACK_COMMAND_N = 3.0

#: The critic's `mass_params` block carries the rope (`hook_cfg.PRIVILEGED_WIDTH`).
PRIVILEGED_WIDTH = hook_cfg.PRIVILEGED_WIDTH


def scaled_weights() -> dict[str, float]:
    """`WEIGHTS` multiplied by the policy timestep, as the environment applies them."""
    return {name: weight * interface.POLICY_DT for name, weight in WEIGHTS.items()}


# --- evaluation ------------------------------------------------------------------------------------
#
# Frozen 2026-10-07, before any policy was trained on the task. Every episode: the goal to one of
# `eval_grip_spheres()`, the rope engaged there running `EVAL_ROPE_LENGTH_M` straight ahead (a horizontal pull straight
# back), the command climbing `EVAL_LEVELS_N` (each ramped at `LEVEL_RAMP_N_PER_S` and held `EVAL_HOLD_S`, read over the
# last `EVAL_WINDOW_S`), then the anchor letting go at the end of the last hold in one of `EVAL_RELEASES`, and
# `EVAL_RECOVERY_S` to recover. Steadiness is scored on the holds, recovery on what follows the release.

EVAL_LEVELS_N = (20.0, 35.0, 50.0)
EVAL_HOLD_S = 4.0
EVAL_WINDOW_S = 2.0
EVAL_RECOVERY_S = 4.0
EVAL_HEIGHTS_M = (0.25, 0.40, 0.55)
EVAL_BEARINGS_RAD = (-0.3, 0.0, 0.3)
EVAL_REACH_M = 0.45
EVAL_ROPE_LENGTH_M = 1.0
EVAL_STIFFNESS = (1000.0, 4000.0)
#: (name, kind, residual fraction, ramp s, detection delay s or None for never noticed).
EVAL_RELEASES = (
    ("snap_noticed", "snap", 0.0, 0.0, 0.3),
    ("snap_unnoticed", "snap", 0.0, 0.0, None),
    ("slip_noticed", "slip", 0.2, 0.05, 0.3),
)
EVAL_ENGAGE_DELAY_STEPS = 15
#: A level is *held* when the mean tension over the window is within `EVAL_HELD_FRACTION` of it.
EVAL_HELD_FRACTION = 0.8
#: A *tug* is a peak of the tension standing at least `TUG_PROMINENCE_FRACTION` of the command, and at least
#: `TUG_MIN_PROMINENCE_N`, above the troughs either side (scipy's prominence): a surge and a release, whichever way the
#: tension then goes. Counted only while at least `TUG_MIN_COMMAND_N` is commanded, since under it the arm resting on the
#: rope (2-5 N) is all the tension there is. (First defined 2026-10-07 as a drop below 75% of the command after being
#: above 90%; replaced the same day, before any trained policy was scored, because the baseline's tugs at tie-on are
#: surges *above* the command -- 10-35 N swings on a 20 N command -- which that definition counted in 8 of 108 holds.)
TUG_PROMINENCE_FRACTION = 0.25
TUG_MIN_PROMINENCE_N = 3.0
TUG_MIN_COMMAND_N = 10.0
#: A strict fall, for scoring recovery: tilt past 0.6 rad (34 degrees), the base below 0.15 m, or the trunk touching.
#: Stricter than the training termination on purpose: a robot that sat down survived UniFP's, not this one.
EVAL_FALL_TILT_RAD = 0.6
EVAL_FALL_HEIGHT_M = 0.15
#: Settled: base horizontal speed and roll/pitch rate under these for `EVAL_SETTLE_HOLD_S` (the base only; see
#: `rope_eval.settle_time`).
EVAL_SETTLE_BASE_M_S = 0.10
EVAL_SETTLE_ANG_RAD_S = 0.30
EVAL_SETTLE_HOLD_S = 0.5
GOAL_CENTRE_HEIGHT_M = interface.EE_GOAL_CENTER_OFFSET[2]


def eval_grip_spheres() -> list[tuple[float, float, float]]:
    """(radius, pitch, yaw) about the goal-sphere centre for each evaluation grip placement."""
    out = []
    for height in EVAL_HEIGHTS_M:
        for bearing in EVAL_BEARINGS_RAD:
            dz = height - GOAL_CENTRE_HEIGHT_M
            out.append((math.hypot(EVAL_REACH_M, dz), math.atan2(dz, EVAL_REACH_M), bearing))
    return out


def eval_plan() -> list[dict]:
    """Every evaluation episode once: placements x stiffnesses x releases."""
    rows = []
    for placement in eval_grip_spheres():
        for stiffness in EVAL_STIFFNESS:
            for name, kind, residual, ramp, detect in EVAL_RELEASES:
                rows.append({"sphere": placement, "stiffness": stiffness, "release": name, "kind": kind,
                             "residual": residual, "ramp_s": ramp, "detect_s": detect})
    return rows


# --- the held-out test set ------------------------------------------------------------------------
#
# Frozen 2026-10-07, 20:10 local, before any policy was run on it. The development set above has been used to diagnose
# v1-v3 and to choose checkpoints, so a number quoted on it is optimistic; this one is for the final checkpoint only,
# run once. Everything differs from the development set: grip placements (0.30 / 0.48 m high, +-0.2 rad, 0.50 m out),
# rope length (0.5 and 1.5 m), stiffness (2,000 and 6,000 N/m), the pull tilted 0.15 rad up or down (down where the
# bearing is negative), the levels (15 / 30 / 45 N, released at the end of the 45 N hold) and the releases.

TEST_LEVELS_N = (15.0, 30.0, 45.0)
TEST_HEIGHTS_M = (0.30, 0.48)
TEST_BEARINGS_RAD = (-0.2, 0.2)
TEST_REACH_M = 0.50
TEST_ROPE_LENGTHS_M = (0.5, 1.5)
TEST_STIFFNESS = (2000.0, 6000.0)
TEST_ELEVATION_RAD = 0.15
TEST_RELEASES = (
    ("snap_quick_notice", "snap", 0.0, 0.0, 0.15),
    ("fade_noticed", "fade", 0.0, 0.3, 0.45),
    ("slip_unnoticed", "slip", 0.35, 0.1, None),
)


def test_plan() -> list[dict]:
    """Every held-out test episode once: placements x lengths x stiffnesses x releases (48)."""
    rows = []
    for height in TEST_HEIGHTS_M:
        for bearing in TEST_BEARINGS_RAD:
            dz = height - GOAL_CENTRE_HEIGHT_M
            sphere = (math.hypot(TEST_REACH_M, dz), math.atan2(dz, TEST_REACH_M), bearing)
            elevation = TEST_ELEVATION_RAD if bearing > 0 else -TEST_ELEVATION_RAD
            for length in TEST_ROPE_LENGTHS_M:
                for stiffness in TEST_STIFFNESS:
                    for name, kind, residual, ramp, detect in TEST_RELEASES:
                        rows.append({"sphere": sphere, "stiffness": stiffness, "length": length, "elevation": elevation,
                                     "release": name, "kind": kind, "residual": residual, "ramp_s": ramp,
                                     "detect_s": detect})
    return rows
