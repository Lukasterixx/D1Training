"""The thing the tool pulls on or presses against, and the force it is told to apply to it.

This is the contact half of the force-transmission task (`hook_env.py`). UniFP's own force channel
pushes the tool with a wrench from nowhere; here the force comes from a fixture the tool is
engaged with, so it exists only while the robot is actually loading it, and in only one direction:

  * a **hook** (a claw over a handle's bar) transmits tension and nothing else. Push the claw back
    toward the handle and it goes slack; push it far enough and it comes off.
  * a **press** (a pad on a button) transmits compression and nothing else. Pull away and the pad
    lifts off.

Both are the same model along the fixture's force axis `d`, the direction the robot must push the
fixture in (toward the robot for a hook, into the panel for a button): a unilateral spring. Across
`d` they differ as their geometry does -- a pad holds by friction in every direction and slides off
the button's edge; a claw rests on its bar, slides along it, and unhooks by lifting off it or
running off its end. `ContactFixture` has the details and `hook_cfg` the numbers.

Why a virtual spring rather than a rigid body in the scene: the force it produces is exact and
known, which is what a force-*tracking* reward needs, and the contact model can be read in one
screen. What it is not: a door that moves, a latch that gives, or a claw whose own geometry can
jam. The handle is anchored; the task is to load it, not yet to open it.

The spring is stepped at the physics rate, not the policy rate -- at 2,000 N/m a force updated
every 20 ms would ring. `step()` takes the tool point's state at each physics substep.

No Isaac imports, so it can be checked without a simulator (`tests/test_force_transmission.py`).
"""
from __future__ import annotations

import math

import torch

from unifp_isaaclab import interface

from . import hook_cfg


class ContactFixture:
    """One unilateral contact per environment, engaged on demand.

    Along the force axis d the contact is the same for both kinds: a spring that resists the tool
    moving *into* the fixture and never pulls it back. Across d they differ, because the geometry
    does:

      * **a pad on a button** holds sideways by friction alone, the same in every direction: `mu`
        times the normal load plus `allowance_n`, past which the contact point slides with the
        tool. Sliding further than `slip_radius_m` from where it landed takes it off the button.
      * **a claw over a bar** is different in the two directions across d. Along the bar (`bar`)
        it slides the way a pad does, and sliding `slip_radius_m` takes it off the bar's end. Across
        the bar (`support`, the side the bar is under) the bar carries whatever the claw rests on it
        -- the arm's own weight, typically -- without slipping at all; and lifting the claw
        `lift_release_m` off the bar unhooks it. Easing off along -d meets the door `backstop_m`
        behind the bar rather than unhooking.

    A contact with `bar` all zeros is a pad.
    """

    #: Stands in for "never" in the per-environment distances below.
    NEVER_M = 1.0e6
    LOST_BACKED_OFF, LOST_SLID_OFF, LOST_LIFTED_OFF = 1, 2, 3

    def __init__(self, num_envs: int, device, *, damping: float) -> None:
        self.num_envs = num_envs
        self.device = torch.device(device)
        self.damping = damping
        zeros = lambda: torch.zeros(num_envs, device=self.device)
        zeros3 = lambda: torch.zeros(num_envs, 3, device=self.device)
        never = lambda: torch.full((num_envs,), self.NEVER_M, device=self.device)
        #: N/m, per environment: drawn at each engagement in training (see `hook_cfg`).
        self.stiffness = zeros()
        #: How far the tool may back off along -d before the contact is lost: a pad leaving the
        #: button. A claw does not come off this way -- it meets the door -- so it has `NEVER_M`.
        self.release_m = never()
        #: How far the tool may back off along -d before something stops it: the claw's back meeting
        #: the door behind the bar. `NEVER_M` for a pad, which just lifts away.
        self.backstop_m = never()
        #: How far a claw may lift off its bar before it is unhooked. `NEVER_M` for a pad.
        self.lift_release_m = never()
        self.engaged = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        #: Set on the physics step the contact was lost, cleared on reset. Latches: a lost contact
        #: does not re-engage by itself.
        self.lost = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        #: How it was lost: 0 not lost, then `LOST_BACKED_OFF`, `LOST_SLID_OFF`, `LOST_LIFTED_OFF`.
        self.lost_how = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        #: Contacts that come back: a pad that leaves its button is *detached* rather than lost, and
        #: re-engages when the tool is pressed onto the button again (inside `slip_radius_m` of where
        #: it first landed). Training only; see `hook_cfg.PRESS_REATTACH`.
        self.reattach = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.detached = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        #: True once a reattaching contact has detached at least once this episode.
        self.ever_detached = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.origin = zeros3()      # where it engaged, env frame
        self.anchor = zeros3()      # the contact point now: moves only by sliding across d
        self.axis = zeros3()        # unit d, the direction the robot must push the fixture
        self.bar = zeros3()         # unit, along the bar; zero for a pad
        self.support = zeros3()     # unit, the side of the bar the claw rests on; zero for a pad
        self.mu = zeros()
        self.allowance_n = zeros()
        self.slip_radius_m = zeros()
        #: Force the fixture applies to the tool, world frame, as of the last `step()`.
        self.force_on_tool = zeros3()
        #: How far the tool has moved into the fixture along d, as of the last `step()`.
        self.penetration_m = zeros()
        #: How near the contact is to being lost, 0 to 1: the largest of the lift, the slide and the
        #: back-off, each as a fraction of the distance that loses it. As of the last `step()`.
        self.risk = zeros()

    @property
    def hooked(self) -> torch.Tensor:
        """True where the contact is a claw over a bar rather than a pad."""
        return self.bar.norm(dim=-1) > 0.5

    def engage(self, env_ids: torch.Tensor, point: torch.Tensor, axis: torch.Tensor, *,
               stiffness: torch.Tensor, mu: torch.Tensor, allowance_n: torch.Tensor,
               slip_radius_m: torch.Tensor, release_m: torch.Tensor, backstop_m: torch.Tensor,
               bar: torch.Tensor, lift_release_m: torch.Tensor, reattach: torch.Tensor | None = None) -> None:
        """Hook onto / land on the fixture at `point` (env frame), which becomes its anchor.

        `bar` must be perpendicular to `axis` (zeros for a pad); the support direction is the one
        perpendicular to both that points upward, so the bar is under the claw.
        """
        axis = axis / axis.norm(dim=-1, keepdim=True)
        support = torch.cross(bar, axis, dim=-1)
        support = torch.where((support[:, 2:3] < 0.0), -support, support)
        self.stiffness[env_ids] = stiffness
        self.reattach[env_ids] = False if reattach is None else reattach
        self.detached[env_ids] = False
        self.release_m[env_ids] = release_m
        self.backstop_m[env_ids] = backstop_m
        self.lift_release_m[env_ids] = lift_release_m
        self.engaged[env_ids] = True
        self.lost[env_ids] = False
        self.origin[env_ids] = point
        self.anchor[env_ids] = point
        self.axis[env_ids] = axis
        self.bar[env_ids] = bar
        self.support[env_ids] = support
        self.mu[env_ids] = mu
        self.allowance_n[env_ids] = allowance_n
        self.slip_radius_m[env_ids] = slip_radius_m

    def reset(self, env_ids: torch.Tensor) -> None:
        self.engaged[env_ids] = False
        self.lost[env_ids] = False
        self.lost_how[env_ids] = 0
        self.reattach[env_ids] = False
        self.detached[env_ids] = False
        self.ever_detached[env_ids] = False
        self.risk[env_ids] = 0.0
        self.force_on_tool[env_ids] = 0.0
        self.penetration_m[env_ids] = 0.0

    def _slide(self, spring: torch.Tensor, offset: torch.Tensor, cap: torch.Tensor):
        """Clip a sideways spring force to `cap`; return it and how far the contact point moves.

        `spring` and `offset` are (N, 3) with the same direction (the spring pulls the tool back
        over `offset`). Past the cap the contact point follows the tool until the spring alone sits
        at the cap, and the returned shift is how far it moved.
        """
        magnitude = spring.norm(dim=-1)
        slipping = magnitude > cap
        scale = torch.where(slipping, cap / magnitude.clamp(min=1e-9), torch.ones_like(magnitude))
        length = offset.norm(dim=-1)
        keep = torch.where(slipping & (length > 1e-9),
                           (cap / (self.stiffness * length).clamp(min=1e-9)).clamp(max=1.0),
                           torch.ones_like(length))
        return spring * scale.unsqueeze(-1), offset * (1.0 - keep).unsqueeze(-1)

    def step(self, tool_pos: torch.Tensor, tool_vel: torch.Tensor) -> torch.Tensor:
        """Advance one physics step and return the force on the tool (N, 3), world frame.

        `tool_pos` is in the env frame (the anchor's), `tool_vel` in the world frame; both are the
        controlled point's, not a link origin's.
        """
        k, c = self.stiffness, self.damping
        dot = lambda a, b: (a * b).sum(-1)
        # A detached pad re-engages when the tool is back on the button face, pressing into it.
        back = tool_pos - self.origin
        into = dot(back, self.axis)
        across_origin = (back - into.unsqueeze(-1) * self.axis).norm(dim=-1)
        retouch = self.detached & (into > 0.0) & (across_origin < self.slip_radius_m)
        self.anchor = torch.where(retouch.unsqueeze(-1), tool_pos - into.unsqueeze(-1) * self.axis, self.anchor)
        self.engaged |= retouch
        self.detached &= ~retouch
        delta = tool_pos - self.anchor
        depth, speed = dot(delta, self.axis), dot(tool_vel, self.axis)
        # Unilateral: resists motion *into* the fixture along d and never pulls the tool in. The
        # damping may reduce the load while the tool backs off, never turn it into adhesion.
        loaded = torch.where(depth > 0.0, (k * depth + c * speed).clamp(min=0.0), torch.zeros_like(depth))
        # Past the backstop the door pushes the tool back along +d, the same way.
        beyond = -self.backstop_m - depth
        stopped = torch.where(beyond > 0.0, (k * beyond - c * speed).clamp(min=0.0), torch.zeros_like(depth))
        normal = loaded + stopped
        cap = self.mu * normal + self.allowance_n

        lateral = delta - depth.unsqueeze(-1) * self.axis
        lateral_vel = tool_vel - speed.unsqueeze(-1) * self.axis

        # A pad: friction, the same in every direction across d.
        pad_force, pad_shift = self._slide(-k.unsqueeze(-1) * lateral - c * lateral_vel, lateral, cap)

        # A claw: one-sided support across the bar, and friction along it from both loads.
        height, rising = dot(lateral, self.support), dot(lateral_vel, self.support)
        carried = torch.where(height < 0.0, (-k * height - c * rising).clamp(min=0.0), torch.zeros_like(height))
        along = dot(lateral, self.bar).unsqueeze(-1) * self.bar
        along_vel = dot(lateral_vel, self.bar).unsqueeze(-1) * self.bar
        bar_force, bar_shift = self._slide(-k.unsqueeze(-1) * along - c * along_vel, along,
                                           self.mu * (normal + carried) + self.allowance_n)
        claw_force = bar_force + carried.unsqueeze(-1) * self.support

        hooked = self.hooked.unsqueeze(-1)
        across = torch.where(hooked, claw_force, pad_force)
        shift = torch.where(hooked, bar_shift, pad_shift)
        self.anchor = torch.where(self.engaged.unsqueeze(-1), self.anchor + shift, self.anchor)

        force = (stopped - loaded).unsqueeze(-1) * self.axis + across
        slid = (self.anchor - self.origin).norm(dim=-1)
        backed, off_end, lifted = depth < -self.release_m, slid > self.slip_radius_m, height > self.lift_release_m
        newly_lost = self.engaged & (backed | off_end | lifted)
        how = torch.where(lifted, self.LOST_LIFTED_OFF, torch.where(off_end, self.LOST_SLID_OFF, self.LOST_BACKED_OFF))
        self.lost_how = torch.where(newly_lost, how, self.lost_how)
        self.detached |= newly_lost & self.reattach
        self.ever_detached |= newly_lost & self.reattach
        self.lost |= newly_lost & ~self.reattach
        self.engaged &= ~newly_lost
        self.force_on_tool = torch.where(self.engaged.unsqueeze(-1), force, torch.zeros_like(force))
        self.penetration_m = torch.where(self.engaged, depth, torch.zeros_like(depth))
        risk = torch.stack(((height / self.lift_release_m).clamp(0.0, 1.0),
                            (slid / self.slip_radius_m).clamp(0.0, 1.0),
                            (-depth / self.release_m).clamp(0.0, 1.0)), dim=-1).max(dim=-1).values
        # A detached pad is as far from seated as it gets.
        self.risk = torch.where(self.engaged, risk, torch.where(self.detached, torch.ones_like(risk),
                                                                torch.zeros_like(risk)))
        return self.force_on_tool

    def applied_by_robot(self) -> torch.Tensor:
        """The force the robot applies to the fixture: the reaction to `force_on_tool`."""
        return -self.force_on_tool


def bar_directions(axis: torch.Tensor, vertical: torch.Tensor,
                   generator: torch.Generator | None = None) -> torch.Tensor:
    """A bar perpendicular to each force axis: horizontal, or as near vertical as it can be.

    A door handle's bar is horizontal and a cabinet pull's vertical; `vertical` (N,) picks which.
    """
    up = torch.zeros_like(axis)
    up[:, 2] = 1.0
    horizontal = torch.cross(axis, up, dim=-1)
    degenerate = horizontal.norm(dim=-1, keepdim=True) < 1e-6
    horizontal = torch.where(degenerate, torch.tensor([0.0, 1.0, 0.0], device=axis.device).expand_as(axis),
                             horizontal)
    horizontal = horizontal / horizontal.norm(dim=-1, keepdim=True)
    upright = torch.cross(horizontal, axis, dim=-1)
    upright = upright / upright.norm(dim=-1, keepdim=True).clamp(min=1e-9)
    return torch.where(vertical.unsqueeze(-1), upright, horizontal)


class ForceLevelSchedule:
    """The commanded force magnitude: hold a level, then ramp to the next one.

    Levels are drawn uniformly in `[0, ceiling]` with a `zero_prob` chance of zero, held for a
    drawn time, and approached at no more than `ramp_n_per_s`. A step change would be a demand the
    body cannot meet in one policy step; a ramp keeps the tracking error about the controller, not
    about the command.

    The ceiling is passed in at each draw rather than fixed, so a curriculum can raise it while
    the run is going.
    """

    def __init__(self, num_envs: int, device, *, hold_s: tuple[float, float], ramp_n_per_s: float,
                 zero_prob: float, dt: float, frontier_prob: float = 0.0,
                 frontier_fraction: float = 0.7, generator: torch.Generator | None = None) -> None:
        self.device = torch.device(device)
        #: The chance a draw lands in the top `1 - frontier_fraction` of the ceiling rather than
        #: anywhere under it, so a curriculum spends time where it is being tested.
        self.frontier_prob = frontier_prob
        self.frontier_fraction = frontier_fraction
        self.hold_steps = (math.ceil(hold_s[0] / dt), math.ceil(hold_s[1] / dt))
        self.ramp_per_step = ramp_n_per_s * dt
        self.zero_prob = zero_prob
        self.generator = generator
        self.level = torch.zeros(num_envs, device=self.device)
        self.current = torch.zeros(num_envs, device=self.device)
        self.timer = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        #: Evaluation only: a fixed sequence of levels, each held `staircase_hold` steps, replacing
        #: the random draw. `stage` is how many of them each environment has started.
        self.staircase: torch.Tensor | None = None
        self.staircase_hold = 0
        self.stage = torch.zeros(num_envs, dtype=torch.long, device=self.device)

    def _rand(self, count: int) -> torch.Tensor:
        return torch.rand(count, device=self.device, generator=self.generator)

    def use_staircase(self, levels, hold_steps: int) -> None:
        """Replace the random draw by `levels` in order, each held `hold_steps`; the last is kept."""
        self.staircase = torch.as_tensor(levels, dtype=torch.float32, device=self.device)
        self.staircase_hold = int(hold_steps)

    def draw(self, env_ids: torch.Tensor, ceiling: float) -> None:
        if self.staircase is not None:
            stage = self.stage[env_ids].clamp(max=len(self.staircase) - 1)
            self.level[env_ids] = self.staircase[stage]
            self.stage[env_ids] += 1
            self.timer[env_ids] = self.staircase_hold
            return
        count = len(env_ids)
        uniform = self._rand(count)
        frontier = self.frontier_fraction + (1.0 - self.frontier_fraction) * self._rand(count)
        level = torch.where(self._rand(count) < self.frontier_prob, frontier, uniform) * ceiling
        self.level[env_ids] = torch.where(self._rand(count) < self.zero_prob,
                                          torch.zeros_like(level), level)
        low, high = self.hold_steps
        self.timer[env_ids] = low + (self._rand(count) * (high - low)).long()

    def set_levels(self, env_ids: torch.Tensor, level: torch.Tensor, hold_steps: int) -> None:
        """Command an exact level, for evaluation."""
        self.level[env_ids] = level
        self.timer[env_ids] = hold_steps

    def reset(self, env_ids: torch.Tensor) -> None:
        self.level[env_ids] = 0.0
        self.current[env_ids] = 0.0
        self.timer[env_ids] = 0
        self.stage[env_ids] = 0

    def step(self, active: torch.Tensor, ceiling: float) -> torch.Tensor:
        """Advance one policy step for the `active` environments; others stay at zero."""
        due = active & (self.timer <= 0)
        if bool(due.any()):
            self.draw(due.nonzero(as_tuple=False).flatten(), ceiling)
        change = (self.level - self.current).clamp(-self.ramp_per_step, self.ramp_per_step)
        self.current = torch.where(active, self.current + change, torch.zeros_like(self.current))
        self.timer = torch.where(active, self.timer - 1, self.timer)
        return self.current


def pull_axes(anchor_w: torch.Tensor, base_pos_w: torch.Tensor, *, yaw_noise_rad: float,
              elevation_rad: tuple[float, float], generator: torch.Generator | None = None) -> torch.Tensor:
    """Unit force directions for a pull: horizontally from the handle back toward the robot.

    That is the direction a handle on a door facing the robot is pulled in. It is turned by up to
    `yaw_noise_rad` either side, so the robot is not always square to the door, and tilted by an
    elevation drawn from `elevation_rad` (positive pulls up).
    """
    count = anchor_w.shape[0]
    device = anchor_w.device
    back = base_pos_w[:, :2] - anchor_w[:, :2]
    heading = torch.atan2(back[:, 1], back[:, 0])
    rand = lambda: torch.rand(count, device=device, generator=generator)
    heading = heading + (2.0 * rand() - 1.0) * yaw_noise_rad
    elevation = elevation_rad[0] + rand() * (elevation_rad[1] - elevation_rad[0])
    return torch.stack((torch.cos(elevation) * torch.cos(heading),
                        torch.cos(elevation) * torch.sin(heading),
                        torch.sin(elevation)), dim=-1)


def press_axes(anchor_w: torch.Tensor, base_pos_w: torch.Tensor, *, yaw_noise_rad: float,
               elevation_rad: tuple[float, float], down_prob: float,
               generator: torch.Generator | None = None) -> torch.Tensor:
    """Unit force directions for a press: away from the robot into a panel, or down onto a top face.

    An emergency stop is either on a panel facing the robot (pushed away from it) or on top of an
    enclosure (pushed down). `down_prob` of the draws are the second kind, tilted by the same
    noise so they are not exactly vertical.
    """
    away = -pull_axes(anchor_w, base_pos_w, yaw_noise_rad=yaw_noise_rad,
                      elevation_rad=(-elevation_rad[1], -elevation_rad[0]), generator=generator)
    count = anchor_w.shape[0]
    rand = lambda: torch.rand(count, device=anchor_w.device, generator=generator)
    tilt = rand() * yaw_noise_rad * 0.5
    azimuth = rand() * 2.0 * math.pi
    down = torch.stack((torch.sin(tilt) * torch.cos(azimuth), torch.sin(tilt) * torch.sin(azimuth),
                        -torch.cos(tilt)), dim=-1)
    return torch.where((rand() < down_prob).unsqueeze(-1), down, away)


def sample_handles(count: int, device, generator: torch.Generator | None = None) -> torch.Tensor:
    """Handle placements in goal-sphere coordinates, clear of the robot's body (`hook_cfg`)."""
    rand = lambda n: torch.rand(n, device=device, generator=generator)
    span = lambda r, n: r[0] + rand(n) * (r[1] - r[0])
    out = torch.zeros(count, 3, device=device)
    pending = torch.arange(count, device=device)
    for _ in range(20):
        n = len(pending)
        draw = torch.stack((span(hook_cfg.HANDLE_RADIUS_RANGE, n), span(hook_cfg.HANDLE_PITCH_RANGE, n),
                            span(hook_cfg.HANDLE_YAW_RANGE, n)), dim=-1)
        out[pending] = draw
        local = interface.sphere2cart(draw)
        clear = (local[:, 0] >= hook_cfg.HANDLE_CLEAR_AHEAD_M) | (local[:, 1].abs() >= hook_cfg.HANDLE_CLEAR_SIDE_M)
        pending = pending[~clear]
        if len(pending) == 0:
            break
    # Anything still inside after twenty draws goes straight ahead at shoulder height.
    out[pending] = torch.tensor((0.5, 0.0, 0.0), device=device)
    return out
