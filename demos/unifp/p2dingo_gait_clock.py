# The gait clock: one phase per robot that says which feet should be down and where the legs
# should be, and that the policy SEES.
#
# WHY A CLOCK. The Spot-derived task (rough_env_cfg.py) leaves the rhythm to emerge from
# timing rewards (`GaitReward`'s sync/async, `air_time`'s mode time) and never tells the policy
# where in the stride it is. What emerges is a trot in phase terms whose SHAPE nobody asked for:
# a flat-topped swing that three reward-shaping attempts could not round (training/README.md,
# "Still open"), a stride that changes with the command in whatever way the optimiser found
# cheapest. The UniFP Go2+D1 policy (D1Training, `unifp_train/gait.py`), whose walking was the
# look wanted here, does the opposite: a phase advances at a set rate, a CONTACT SCHEDULE
# derived from it is rewarded foot by foot, the legs are rewarded for tracking a swing
# reference drawn from it, and sin/cos of it is in the observation. The rhythm is specified,
# not discovered. That is the "reference trajectory" the README said the swing needed.
#
# WHAT IS UniFP's AND WHAT IS NOT:
#   * the stance mask and the swing reference are UniFP's formulas (FL+RR against FR+RL, the
#     shifted-sine double-support overlap, a reference that only ever pushes a leg through its
#     SWING and holds the stance leg at the nominal pose), with the overlap and the amplitude
#     as parameters;
#   * UniFP's period is a constant 0.64 s for commands up to 0.6 m/s. This robot is commanded
#     up to 1.4 m/s by nav2's sim overlay, and at 0.64 s a stance at 1.4 m/s would carry each
#     foot 0.6 m under the body. So the stride FREQUENCY rises with the commanded speed
#     (`frequency`), and the swing is 40 % of the stride rather than a third;
#   * the stand rule is UniFP's (the phase is pinned to 0, where every foot is in stance,
#     whenever the whole command is inside its dead zone), with a yaw dead zone of 0.1 rad/s
#     rather than 0.2 so that slow turns are still commandable.
#
# NO ISAAC LAB IMPORTS, ON PURPOSE. The simulator (`go2_omniverse/custom_rl_env.py`) has to
# run this exact clock for the policy's observation to mean what it meant in training, and it
# loads this file by path rather than importing the task package (which registers gym ids).
# Change a number here and every policy trained against the old one sees a different clock:
# the policy table in `go2_omniverse/agent_cfg.py` records which clock a checkpoint needs.

from __future__ import annotations

import math
from dataclasses import dataclass

import torch

#: Foot order of every per-foot tensor here: FL, FR, RL, RR. FL+RR swing together.
FEET = ("FL_foot", "FR_foot", "RL_foot", "RR_foot")
#: The twelve leg joints in the same leg order, (hip, thigh, calf) per leg -- the order of
#: `reference_joint_pos`'s columns. Resolve them with `preserve_order=True`.
LEG_JOINTS = tuple(f"{leg}_{j}_joint" for leg in ("FL", "FR", "RL", "RR") for j in ("hip", "thigh", "calf"))


@dataclass
class GaitClockCfg:
    """Every number that defines the clock. A checkpoint is only valid against the values it trained on."""

    #: Stride frequency at a standstill-to-walk transition, Hz (UniFP's is a constant 1/0.64 = 1.56).
    base_frequency: float = 1.6
    #: Added per m/s of commanded planar speed, Hz.
    frequency_per_speed: float = 0.8
    #: Added per rad/s of commanded yaw rate, Hz: a hard spin steps a little quicker.
    frequency_per_yaw: float = 0.2
    #: Ceiling, Hz.
    max_frequency: float = 3.0
    #: The double-support overlap: a foot is in stance while its diagonal's shifted sine is past
    #: -`overlap`. UniFP's 0.5 swings a third of the stride; 0.3 swings asin -> 40.3 %.
    overlap: float = 0.3
    #: Peak swing deflection of the thigh from its stance pose, rad (the calf gets -2x). UniFP 0.17.
    swing_thigh: float = 0.2
    #: The whole command is "stand" when |vx| and |vy| are under this, m/s ...
    lin_dead_zone: float = 0.1
    #: ... and |wz| under this, rad/s.
    yaw_dead_zone: float = 0.1
    #: Policy step, s (Isaac Lab's Go2: sim dt 0.005 x decimation 4).
    dt: float = 0.02

    def swing_fraction(self) -> float:
        """Fraction of the stride each foot spends in the air."""
        return (math.pi - 2.0 * math.asin(self.overlap)) / (2.0 * math.pi)


def walking(cfg: GaitClockCfg, command: torch.Tensor) -> torch.Tensor:
    """(N,) bool: some component of the (N, 3) [vx, vy, wz] command is outside its dead zone."""
    return ((command[:, 0].abs() > cfg.lin_dead_zone) | (command[:, 1].abs() > cfg.lin_dead_zone)
            | (command[:, 2].abs() > cfg.yaw_dead_zone))


def apply_dead_zone(cfg: GaitClockCfg, command: torch.Tensor) -> torch.Tensor:
    """The command with rows inside the whole dead zone set to exactly zero -- what the policy is shown.

    A row with any component outside keeps all three, so a slow creep while turning survives. Training
    zeroes at sampling time and the simulator per step; both call this.
    """
    return torch.where(walking(cfg, command).unsqueeze(1), command, torch.zeros_like(command))


def frequency(cfg: GaitClockCfg, command: torch.Tensor) -> torch.Tensor:
    """(N,) stride frequency, Hz, for an (N, 3) command."""
    speed = torch.linalg.norm(command[:, :2], dim=1)
    f = cfg.base_frequency + cfg.frequency_per_speed * speed + cfg.frequency_per_yaw * command[:, 2].abs()
    return f.clamp(max=cfg.max_frequency)


class GaitClock:
    """The phase of every robot's stride, advanced once per policy step."""

    def __init__(self, cfg: GaitClockCfg, num_envs: int, device):
        self.cfg = cfg
        self.phase = torch.zeros(num_envs, device=device)

    def reset(self, env_ids=None) -> None:
        if env_ids is None:
            self.phase.zero_()
        else:
            self.phase[env_ids] = 0.0

    def advance(self, command: torch.Tensor, dt: float | None = None) -> None:
        """One policy step. A standing robot is pinned at phase 0, where all four feet are in stance.

        The pin is how the policy is told to stop stepping (UniFP's `gait_step`): a phase left running
        at a zero command makes the robot march on the spot.
        """
        step = (self.cfg.dt if dt is None else dt) * frequency(self.cfg, command)
        advanced = torch.remainder(self.phase + step, 1.0)
        self.phase = torch.where(walking(self.cfg, command), advanced, torch.zeros_like(advanced))

    def observation(self) -> torch.Tensor:
        """(N, 2): sin and cos of the phase, the two numbers appended to the policy's observation."""
        angle = 2.0 * math.pi * self.phase
        return torch.stack((torch.sin(angle), torch.cos(angle)), dim=1)

    def stance_mask(self) -> torch.Tensor:
        """(N, 4) of 1.0 where the schedule wants the foot down, FL FR RL RR. UniFP's `stance_mask`."""
        s = torch.sin(2.0 * math.pi * self.phase)
        a = (s + self.cfg.overlap >= 0).float()     # FL + RR
        b = (s - self.cfg.overlap < 0).float()      # FR + RL
        return torch.stack((a, b, b, a), dim=1)

    def swing_offsets(self) -> torch.Tensor:
        """(N, 12) deflection of the legs from their stance pose, in `LEG_JOINTS` order. UniFP's `reference_leg_pos`.

        Swing only -- a stance leg's deflection is exactly zero, so the reference never pushes against the
        ground -- and the hip never moves. The thigh flexes forward by up to `swing_thigh` and the calf folds by
        twice that, on a sine that is zero at lift-off and at touchdown.
        """
        s = torch.sin(2.0 * math.pi * self.phase)
        scale = self.cfg.swing_thigh / (1.0 - self.cfg.overlap)
        a = torch.clamp(s + self.cfg.overlap, max=0.0)    # <= 0 while FL + RR swing
        b = torch.clamp(s - self.cfg.overlap, min=0.0)    # >= 0 while FR + RL swing
        swing_a = -a * scale                              # >= 0, peaks at swing_thigh
        swing_b = b * scale
        zero = torch.zeros_like(s)
        per_leg = []
        for swing in (swing_a, swing_b, swing_b, swing_a):    # FL, FR, RL, RR
            per_leg += [zero, swing, -2.0 * swing]
        return torch.stack(per_leg, dim=1)
