"""A taut tether the claw holds, and an anchor that can give way.

This is the plant of the rope-pull task (`rope_env.py`). The hook task's fixtures (`fixture.py`) hold the tool at a
point: a ring or a bar carries the arm's weight and resists the tool moving sideways. A rope does neither. It pulls
the tool along one line only, from the tool toward where the rope is tied, and only while it is stretched:

  * **tension only.** `T = k * stretch + c * stretch_rate` while the rope is longer than its rest length, never
    negative (the damper cannot pull the rope slack-tight), and zero once it is slack. Push the claw back toward the
    anchor and the rope goes slack; pull it away and the rope tightens. Nothing across the line, so the arm holds its
    own weight and its own line: a rope of length `L` under tension `T` resists a sideways move with only `T / L`
    newtons per metre, which for a 1 m rope at 40 N is 40 N/m.
  * **the grip does not slip.** The claw's teeth interlock on the rope, so the rope is tied to the controlled point
    (the jaw centre) for the whole episode. There is no "lost" state; what can fail is the anchor.
  * **the anchor can give way** (`release`). Until then it holds whatever the robot puts into it. A release lowers
    what it holds -- its *capacity* -- from the tension of that moment to a residual, linearly over a ramp. While the
    rope's spring would carry more than the capacity, the anchor pays out rope (moves toward the tool along the line)
    so that the tension sits at the capacity: a massless anchor under Coulomb friction, which is a load that broke
    free and drags, or a knot that slips. Residual 0 with no ramp is a rope that snapped or was cut.

The spring is stepped at the physics rate (`step()` per physics substep), as `fixture.ContactFixture` is: at a few
kN/m a force updated every 20 ms would ring.

No Isaac imports, so it is checked without a simulator (`tests/test_rope.py`).
"""
from __future__ import annotations

import torch

#: Release kinds, as stored in `RopeTether.release_kind`. 0 is "not released".
SNAP, SLIP, FADE = 1, 2, 3
RELEASE_NAMES = {SNAP: "snap", SLIP: "slip", FADE: "fade"}

#: Below this, a rope counts as slack (N). The spring is exact, so a slack rope reads 0.0 exactly; the margin is for
#: the damper's share while the rope is just taut and the tool is moving back.
SLACK_N = 0.5


class RopeTether:
    """One rope per environment, from the controlled point to an anchor, engaged on demand.

    All positions are in the environment frame (the scene's origin taken off), velocities in the world frame, as the
    hook task's fixture takes them. `direction` is the unit vector from the anchor to the tool: the way the robot
    pulls the rope, and the opposite of the force the rope puts on the tool.
    """

    def __init__(self, num_envs: int, device) -> None:
        self.num_envs = num_envs
        self.device = torch.device(device)
        zeros = lambda: torch.zeros(num_envs, device=self.device)
        zeros3 = lambda: torch.zeros(num_envs, 3, device=self.device)
        self.anchor = zeros3()
        self.length = zeros()
        self.stiffness = zeros()
        self.damping = zeros()
        self.engaged = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        #: The anchor's holding capacity, N: infinite until a release, then ramping from `release_from` to `residual`.
        self.capacity = torch.full((num_envs,), float("inf"), device=self.device)
        self.released = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.release_kind = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        self.release_from = zeros()
        self.residual = zeros()
        #: Physics steps the capacity takes to fall from `release_from` to `residual`; 0 is instantaneous.
        self.ramp_steps = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        #: Physics steps since the release, counting the step it happened on as 0.
        self.release_age = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        #: How far the anchor has paid out since the release, m.
        self.paid_out = zeros()
        # As of the last `step()`:
        self.tension = zeros()
        self.stretch = zeros()
        self.direction = zeros3()
        self.force_on_tool = zeros3()

    @property
    def slack(self) -> torch.Tensor:
        return self.engaged & (self.tension < SLACK_N)

    def engage(self, env_ids: torch.Tensor, point: torch.Tensor, pull_direction: torch.Tensor, *,
               length: torch.Tensor, stiffness: torch.Tensor, damping: torch.Tensor) -> None:
        """Tie the rope to the tool at `point`, running `length` back from it against `pull_direction`.

        The rope starts exactly taut: the anchor is at `point - length * pull_direction`, so the tool pulling along
        `pull_direction` stretches it at once and moving the other way slackens it.
        """
        pull_direction = pull_direction / pull_direction.norm(dim=-1, keepdim=True)
        self.anchor[env_ids] = point - length.unsqueeze(-1) * pull_direction
        self.length[env_ids] = length
        self.stiffness[env_ids] = stiffness
        self.damping[env_ids] = damping
        self.engaged[env_ids] = True
        self.capacity[env_ids] = float("inf")
        self.released[env_ids] = False
        self.release_kind[env_ids] = 0
        self.release_from[env_ids] = 0.0
        self.residual[env_ids] = 0.0
        self.ramp_steps[env_ids] = 0
        self.release_age[env_ids] = 0
        self.paid_out[env_ids] = 0.0
        self.direction[env_ids] = pull_direction

    def release(self, env_ids: torch.Tensor, kind: torch.Tensor, residual_fraction: torch.Tensor,
                ramp_steps: torch.Tensor) -> None:
        """Let the anchor give way: its capacity falls from the present tension to `residual_fraction` of it."""
        self.released[env_ids] = True
        self.release_kind[env_ids] = kind
        self.release_from[env_ids] = self.tension[env_ids]
        self.residual[env_ids] = self.tension[env_ids] * residual_fraction
        self.ramp_steps[env_ids] = ramp_steps.long()
        self.release_age[env_ids] = 0
        self.paid_out[env_ids] = 0.0
        self._update_capacity()

    def reset(self, env_ids: torch.Tensor) -> None:
        self.engaged[env_ids] = False
        self.released[env_ids] = False
        self.release_kind[env_ids] = 0
        self.capacity[env_ids] = float("inf")
        self.release_age[env_ids] = 0
        self.paid_out[env_ids] = 0.0
        self.tension[env_ids] = 0.0
        self.stretch[env_ids] = 0.0
        self.force_on_tool[env_ids] = 0.0

    def _update_capacity(self) -> None:
        ramp = self.ramp_steps.clamp(min=1).float()
        progress = torch.where(self.ramp_steps > 0, (self.release_age.float() / ramp).clamp(max=1.0),
                               torch.ones_like(ramp))
        falling = self.release_from + (self.residual - self.release_from) * progress
        self.capacity = torch.where(self.released, falling, torch.full_like(falling, float("inf")))

    def step(self, tool_pos: torch.Tensor, tool_vel: torch.Tensor) -> torch.Tensor:
        """Advance one physics step and return the force on the tool (N, 3), world frame."""
        self._update_capacity()
        offset = tool_pos - self.anchor
        distance = offset.norm(dim=-1).clamp(min=1e-6)
        direction = offset / distance.unsqueeze(-1)
        stretch = distance - self.length
        rate = (direction * tool_vel).sum(-1)
        spring = (self.stiffness * stretch + self.damping * rate).clamp(min=0.0)
        tension = torch.where(stretch > 0.0, spring, torch.zeros_like(spring))
        # The anchor pays out where the rope would carry more than it holds: the stretch drops to what carries the
        # capacity (the damper's share included), and the anchor moves the difference along the rope toward the tool.
        over = self.engaged & (tension > self.capacity)
        capacity = torch.where(torch.isfinite(self.capacity), self.capacity, torch.zeros_like(self.capacity))
        kept = ((capacity - self.damping * rate) / self.stiffness.clamp(min=1e-6)).clamp(min=0.0)
        paid = torch.where(over, (stretch - kept).clamp(min=0.0), torch.zeros_like(stretch))
        self.anchor = self.anchor + paid.unsqueeze(-1) * direction
        self.paid_out = self.paid_out + paid
        tension = torch.where(over, capacity, tension)
        stretch = torch.where(over, kept, stretch)
        tension = torch.where(self.engaged, tension, torch.zeros_like(tension))
        self.tension = tension
        self.stretch = torch.where(self.engaged, stretch, torch.zeros_like(stretch))
        self.direction = torch.where(self.engaged.unsqueeze(-1), direction, self.direction)
        self.force_on_tool = -tension.unsqueeze(-1) * direction
        self.release_age = torch.where(self.released, self.release_age + 1, self.release_age)
        return self.force_on_tool

    def applied_by_robot(self) -> torch.Tensor:
        """The force the robot puts into the rope: the reaction to `force_on_tool`."""
        return -self.force_on_tool
