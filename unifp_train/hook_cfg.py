"""Constants of the force-transmission task: pulling on a hooked handle, pressing a button.

The task is UniFP's with a fixture added (`hook_env.py`). What it asks is different: not "track a
few newtons wherever the goal is" but "put the tool on this handle and load it with tens of
newtons", which the D1 cannot do by its motors alone. The static study
(`results/week_02/figures/force_transmission_study.py`, F-101) sets the numbers here:

  * a typical bent reach holds 9.4 N horizontally (median), 23 N along its best direction;
  * the best posture for a horizontal pull holds ~20 N with the base level and 40-59 N with the base
    pitched nose-down 15 degrees, each at 2 degrees of joint error; exact singular postures reach
    100-200 N but lose most of it to that error;
  * pulling (tension) is stable at any force; pushing (compression) buckles at a force proportional
    to the servo stiffness -- 43 N at a tenth of the simulator's gains, 430 N at them.

So the ceiling runs to 60 N: past what any posture holds robustly, which is the point -- a policy
that meets 60 N has found a posture the study did not, or is leaning on something the static
model does not have. The curriculum starts at 15 N, just above what a bent arm holds.

Reward weights are the config's numbers; `scaled_weights()` multiplies by the policy step, as
`task_cfg` does for UniFP's own terms.
"""
from __future__ import annotations

import math

from unifp_isaaclab import interface

# --- which environments do what ----------------------------------------------------------------

#: Fraction of episodes that engage a fixture. The rest are UniFP's own free-space task, pushes and
#: all, so what the policy already does is kept in training rather than overwritten.
FIXTURE_FRACTION = 0.75
#: Of the fixture episodes, the fraction that press a button rather than pull a handle.
PRESS_FRACTION = 0.0

# --- where the fixture is ----------------------------------------------------------------------
#
# In UniFP's goal-sphere coordinates: (radius, pitch, yaw) about a centre 0.49 m above the ground
# under the base, in the yaw-only base frame. Pitch -0.75..0.45 rad over radii 0.38-0.58 m spans
# handle heights of 0.10-0.74 m, which covers the static study's range (UniFP's own goals reach as
# low). A handle must be clear of the robot's own body -- ahead of its head, or beside it -- and is
# redrawn otherwise.

HANDLE_RADIUS_RANGE = (0.38, 0.58)
HANDLE_PITCH_RANGE = (-0.75, 0.45)
HANDLE_YAW_RANGE = (-0.6, 0.6)
#: A handle counts as clear if it is this far ahead of the goal-sphere centre, or this far to the side.
HANDLE_CLEAR_AHEAD_M = 0.32
HANDLE_CLEAR_SIDE_M = 0.20

#: Policy steps after the goal reaches the handle before the tool engages it, drawn per episode.
#: The goal holds for at least 25 steps (`interface.EE_GOAL_HOLD_TIME_S`), so this is always inside
#: the hold and never after the goal has moved on.
ENGAGE_DELAY_STEPS = (10, 20)
#: ... and then only once the tool has slowed below this, or `ENGAGE_MAX_WAIT_STEPS` later regardless
#: (v4). Engaging a tool still converging on its goal put the anchor where it was passing, and its
#: overshoot then read as lifting or sliding off.
ENGAGE_SETTLE_SPEED_M_S = 0.10
ENGAGE_MAX_WAIT_STEPS = 50

# --- the force directions ------------------------------------------------------------------------

#: A pull runs from the handle back toward the robot, turned by up to this much either side.
PULL_YAW_NOISE_RAD = 0.5
#: ... and tilted by an elevation in this range (positive pulls upward).
PULL_ELEVATION_RAD = (-0.35, 0.35)
#: A press runs away from the robot into a panel with the same noise, or down onto a top face.
PRESS_DOWN_PROB = 0.3

# --- the contact ---------------------------------------------------------------------------------

#: N/m. Drawn log-uniformly per engagement in training, so the policy meets both a sprung latch and
#: a stiff one; 2,000 N/m (60 N at 3 cm of stretch) in evaluation. A single stiff value in v1-v2 made
#: a millimetre of body sway two newtons, and the policy never learned to hold a small force.
#: 3,000 N/m is inside what the explicit 5 ms spring integrates (smoke runs, not assumed).
FIXTURE_STIFFNESS_RANGE = (500.0, 3000.0)
FIXTURE_STIFFNESS = 2000.0
#: N*s/m, every direction.
FIXTURE_DAMPING = 10.0
#: A claw over a bar: the bar sits inside the hook, so a sideways load is carried by geometry --
#: modelled as a high friction coefficient plus an allowance -- and the claw can slide 5 cm along
#: the bar before it comes off the end. Easing off moves the claw toward the door behind the bar,
#: which it meets 3 cm on; it does not unhook that way. (v1 of the task, `hook_pull_v1`, lost the
#: contact there instead, which made easing off a terminal risk and over-pulling the safe choice.)
HOOK_MU = 2.0
HOOK_ALLOWANCE_N = 5.0
HOOK_SLIP_RADIUS_M = 0.05
HOOK_BACKSTOP_M = 0.03
#: Lifting the claw this far off its bar unhooks it: a 3 cm hook (2 cm in v3, which lost most
#: episodes this way to ordinary tracking error). The bar carries whatever the claw rests on it
#: (v3; v1-v2 used the pad's isotropic friction for the claw too, so the arm's own ~10 N weight slid
#: it off the bar whenever the pull was light, and that was most of v2's losses).
HOOK_LIFT_RELEASE_M = 0.03
#: A cabinet pull's bar is vertical; a door handle's horizontal.
HOOK_VERTICAL_BAR_PROB = 0.3
#: Of the pulls, the fraction on a *ring*: the claw through a loop (a D-ring pull, a rope loop),
#: captured across d in every direction, so it cannot lift or slide off. It isolates the question the
#: task is about -- how much force the body can put through the arm -- from keeping a claw on a bar,
#: which v1-v3 spent most of their fixture time failing at. Modelled as the pad's contact with
#: friction too large to bind, and the claw's backstop.
RING_PROB = 0.4
RING_MU = 1.0e3
RING_ALLOWANCE_N = 1.0e3
#: A push tool on a button: a flat rubber pad, 4 cm in radius, on a plastic mushroom head. The contact
#: is lost when the button leaves the pad (the contact point slides 4 cm) or the pad lifts 3 cm off.
#: v5 used a bare 2 cm button: with no push yet, friction holds nothing sideways, so the tool had to
#: hover within 2 cm while its tracking error is 1.5-6 cm, and every press was lost within a second.
#: The pad is the tool doing what a tool is for -- widening what the controller must hit -- which is
#: the tool-conditioning half of the thesis, not a softening of the task.
PRESS_MU = 0.6
PRESS_ALLOWANCE_N = 1.0
PRESS_SLIP_RADIUS_M = 0.04
PRESS_RELEASE_M = 0.03
#: A curriculum on that 4 cm, in training only (v7). With it from the start, 59 of 60 presses slid off
#: a median 0.1 s after engaging (v6, `model_14000`): the policy's arm is never still -- it drives
#: Joint1-3 into saturation even in free space, a habit UniFP's task rewards and a ring hides -- so the
#: tool crossed 4 cm sideways before it could push. The allowance starts wide and closes by
#: `PRESS_SLIP_STEP_M` whenever fewer than `PRESS_LOST_TARGET` of the press episodes lose the pad, so
#: the policy gets to push while it learns to hold the tool still. Evaluation always uses 4 cm.
PRESS_SLIP_START_M = 0.15
PRESS_SLIP_STEP_M = 0.01
PRESS_LOST_TARGET = 0.30
#: Press episodes, counted over resets, between two tightenings. Only episodes that engaged count.
PRESS_CURRICULUM_MIN_EPISODES = 400
#: In training a pad that leaves its button re-engages when pressed back onto it, instead of ending the
#: episode (v8). A real button can be pressed again; and in v5-v7 every press episode ended within a
#: second or so of engaging -- first by sliding, then, once sliding was allowed, by the pull policy
#: backing off -- which left the policy almost no pushing to learn from. Evaluation keeps it strict:
#: a pad that leaves the button is a failed press. The press curriculum counts an episode as lost if
#: its pad ever came off.
PRESS_REATTACH = True
#: The pad's resting friction follows the same curriculum (v9): `PRESS_ALLOWANCE_START_N` at the widest
#: slide allowance, down to `PRESS_ALLOWANCE_N` at 4 cm. v8's pads came off *down* in 25 of 60 evaluation
#: presses (median 1.7 cm by the time they went): the pull policy lets a fixture carry the arm's weight,
#: and a pad holds 1 N at rest. Starting at 15 N lets the pad carry it while the policy learns to hold
#: the arm up or press hard enough (0.6 x 15 N) to keep it by friction.
PRESS_ALLOWANCE_START_N = 15.0

# --- the arm's rotor inertia ---------------------------------------------------------------------

#: kg*m^2 on each of the six arm joints, in place of the 2e-4 `unifp_isaaclab.robot.ARM_ARMATURE`
#: puts on every joint. ESTIMATED, not measured: a small servo rotor (~1e-6 kg*m^2) through a
#: ~100:1 gearbox reflects about this much, and the D1's gearing is unpublished.
#:
#: The reason it is not left at 2e-4 is F-102. UniFP's explicit PD (kd 0.8 N*m*s/rad, every 5 ms)
#: on the D1's gram-weight wrist links puts kd*dt/I near 19, far past the ~2 an explicit damper can
#: integrate. With zero actions and a standing robot, Joint4-6 then flip torque sign on 98-99.8% of
#: physics steps at 75-90% of their limit and spin at 1.6-1.7 rad/s RMS, and Joint3 sits pinned at
#: its limit -- a numerical limit cycle, not a load. At 0.01 the wrist is still and Joint2/3 carry
#: 41% / 69% of their limits holding the default pose, against the static model's 32% / 63%. A
#: task whose whole point is how much torque the arm has to spare cannot run on the first.
#: The legs, 2e-4 as before, do not chatter (at most 1.6% sign flips).
ARM_ARMATURE_KG_M2 = 0.01

# --- the force command ---------------------------------------------------------------------------

LEVEL_HOLD_S = (1.5, 4.0)
LEVEL_RAMP_N_PER_S = 25.0
LEVEL_ZERO_PROB = 0.15
#: Half the draws land in the top 30% of the ceiling, where the curriculum is testing the policy.
LEVEL_FRONTIER_PROB = 0.5
LEVEL_FRONTIER_FRACTION = 0.7

#: The curriculum on the largest level drawn. It is raised by `CURRICULUM_STEP_N` whenever the
#: engaged episodes' force error *along the axis*, relative to the force commanded, averages below
#: `CURRICULUM_PROMOTE_REL_ERR` -- and at most once per `CURRICULUM_MIN_ITERATIONS`. Along the axis
#: because that is the force the task is for; in v1 the vector error was used, and ~8 N of sideways
#: load (the arm resting on the handle) held it above the threshold at any pull. And only over steps
#: at the frontier (level at least `LEVEL_FRONTIER_FRACTION` of the ceiling, v3), because the question
#: the curriculum asks is whether the policy can make the forces it is about to be asked for more of;
#: holding 2 N to the newton is a different skill, and in v2 it held the average down at every ceiling.
CURRICULUM_START_N = 15.0
CURRICULUM_STEP_N = 5.0
CURRICULUM_MAX_N = 60.0
CURRICULUM_PROMOTE_REL_ERR = 0.40
CURRICULUM_MIN_ITERATIONS = 50
#: Exponential average over reset batches, so one lucky batch does not promote.
CURRICULUM_EMA = 0.05
#: An episode counts toward the average only once it has been engaged this long.
CURRICULUM_MIN_ENGAGED_STEPS = 50

# --- the reward ----------------------------------------------------------------------------------

WEIGHTS = {
    # The objective: the force the robot applies along the fixture's axis against the one
    # commanded, with a quarter weight on whatever it puts across the axis (see
    # `FORCE_LATERAL_WEIGHT`).
    "fixture_force_tracking": 4.0,
    # Load carried by the arm's motors, above `ARM_TORQUE_MARGIN` of each joint's limit. This is the
    # term that says "route it through the structure": a policy can meet a force with a saturated
    # elbow in simulation, and a real servo holding its limit trips or overheats. Kept below the
    # force term: three saturated joints cost 1.1 against 4.0 for the force, so easing off the
    # pull is never the cheaper way out -- the warm-start policy saturates Joint1-3 even in free
    # space (F-102 run), and a heavier margin would first teach it to stop pulling.
    "arm_torque_margin": -4.0,
    # Losing the contact ends the episode; this is paid once, on the step it happens.
    "fixture_lost": -100.0,
    # How close the contact is to being lost, squared: 0 seated, 1 at the edge (v4). A dense warning
    # before the terminal event above, which on its own is a sparse signal.
    "fixture_seat": -2.0,
    # The tool's speed while on a fixture, squared, m^2/s^2 (v9). The policy drives Joint1-3 into
    # saturation even in free space and the tool is never still: a ring hides it, a pad on a button
    # slides off (v8: 2.8 cm across, median, in 0.16 s). A real D1 on a 10 Hz firmware loop cannot
    # move like that either. 0.4 m/s costs 0.16 x 5 = 0.8 against 4.0 for the force.
    "fixture_tool_speed": -5.0,
}

#: Fraction of each arm joint's effort limit above which `arm_torque_margin` starts to charge.
ARM_TORQUE_MARGIN = 0.7
#: Widths of the two exponentials `fixture_force_tracking` sums, newtons: a sharp one that pays
#: for the last few newtons and a broad one that still has a gradient 30 N out.
FORCE_SIGMA_FINE_N = 3.0
FORCE_SIGMA_COARSE_N = 12.0
#: How much a newton across the axis counts against a newton of error along it. Not zero: a load
#: across a hook is a load on its friction, and wasted. Not one: the arm resting its weight on the
#: handle is harmless to the task, and at full weight it swamped the along-axis signal in v1.
FORCE_LATERAL_WEIGHT = 0.25

#: Width of the privileged block the fixture state is written into. It is the critic's
#: `mass_params` block: zero throughout this task because mass randomisation is off, and 22 wide
#: in every checkpoint, so the critic learns to read it without any width changing.
PRIVILEGED_WIDTH = 22


def scaled_weights() -> dict[str, float]:
    """`WEIGHTS` multiplied by the policy timestep, as the environment applies them."""
    return {name: weight * interface.POLICY_DT for name, weight in WEIGHTS.items()}


# --- evaluation ------------------------------------------------------------------------------------

#: The staircase every evaluation episode climbs once engaged: each level ramped to at
#: `LEVEL_RAMP_N_PER_S` and held for `EVAL_HOLD_S`, the force read over the hold's last
#: `EVAL_WINDOW_S`.
EVAL_LEVELS_N = (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)
EVAL_HOLD_S = 2.5
EVAL_WINDOW_S = 1.0
#: A level is *held* when the mean applied force along d over the window is within this fraction of
#: it and the contact survives the whole hold.
EVAL_HELD_FRACTION = 0.8
#: Handle placements: heights above the ground and bearings from the robot's heading, at a fixed
#: horizontal reach. Every combination is one episode per repeat.
EVAL_HEIGHTS_M = (0.20, 0.30, 0.40, 0.50, 0.60)
EVAL_BEARINGS_RAD = (-0.4, 0.0, 0.4)
EVAL_REACH_M = 0.45
EVAL_EPISODE_S = 25.0
#: What the staircase pulls on: "ring" measures force transmission alone, "bar" also asks the claw to
#: stay on a horizontal bar.
EVAL_FIXTURE_KINDS = ("ring", "bar")
GOAL_CENTRE_HEIGHT_M = interface.EE_GOAL_CENTER_OFFSET[2]


def eval_handle_spheres() -> list[tuple[float, float, float]]:
    """(radius, pitch, yaw) about the goal-sphere centre for each evaluation placement."""
    out = []
    for height in EVAL_HEIGHTS_M:
        for bearing in EVAL_BEARINGS_RAD:
            dz = height - GOAL_CENTRE_HEIGHT_M
            out.append((math.hypot(EVAL_REACH_M, dz), math.atan2(dz, EVAL_REACH_M), bearing))
    return out
