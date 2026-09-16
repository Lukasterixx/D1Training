"""Where the task puts the robot and its targets, with no dependencies so CPU tools can read it.

`workspace.py` (CPU forward kinematics) and the task itself must agree on these, so they live here
rather than being copied into both. Every number below is measured in the simulator, not modelled:
the CPU forward kinematics put the resting pincer tip about 1.6 cm from where it actually rests,
because the model has neither the arm's residual sag (F-010) nor the base's 1 deg resting tilt.
"""

# Base height at reset. `flat_env_cfg.ROBOT_START_POS` drops the robot from 0.42 m, which suits
# playback but made this task start every episode with a fall: 18.8 deg peak tilt and the base
# passing 0.197 m, only 4.7 cm above the `low_base` termination. Spawning at the standing height
# gives 5.1 deg and 0.260 m. It does not remove the backward slide, which is the zero-action
# posture settling rather than the drop (Week 1, 2026-09-16).
SPAWN_HEIGHT_M = 0.30

# Measured under zero actions at SPAWN_HEIGHT_M, relative to the environment origin, as the mean
# over the last 2 s of a 12 s settle (16 envs; Week 1 log, 2026-09-16). The base still slides
# 5.6 cm back from where it spawns and is settled by 3.6 s.
ZERO_ACTION_BASE_OFFSET_M = (-0.0555, 0.0, 0.2737)

# Where the pincer tip rests when the episode starts and nothing moves. The target box is placed
# against this point, so it is measured rather than taken from forward kinematics.
ZERO_ACTION_TIP_M = (0.3601, -0.0141, 0.7604)

# The reaching target box, relative to the environment origin (uniform, one fixed target per
# episode). Placed so that no target is within G1a's 5 cm of ZERO_ACTION_TIP_M: zero actions score
# 0% by construction, and every target is a reach the arm has to make. Its nearest face is 14 cm
# below the resting tip, which is far larger than the 1.4 cm the resting tip varies by.
TARGET_RANGES = ((0.36, 0.48), (-0.08, 0.08), (0.50, 0.62))
