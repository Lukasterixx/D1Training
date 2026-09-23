"""UniFP's viewer overlay, rebuilt on Isaac Lab's markers.

Isaac Gym draws this with `gymutil.draw_lines`, which pushes wireframe geometry at the viewer
every frame. Isaac Lab has no equivalent; it has `VisualizationMarkers`, a `UsdGeom.PointInstancer`
holding a fixed set of prototypes that you re-place each frame. So the same picture is assembled
differently: one instancer, seven prototypes, and every marker in the scene placed by one
`visualize()` call per frame.

What it draws, from UniFP's `_draw_ee_goal_curr`, `_draw_ee_goal_traj`, `_draw_ee_force` and
`_draw_base_force`:

  * **yellow** -- the current end-effector goal
  * **magenta** -- the goal displaced by the force term, which is what the reward actually scores
    against. Comes from `rewards.ee_target` rather than a second copy of the expression, so it
    cannot drift away from the reward it claims to show. With no force commanded and none applied
    the magenta sphere sits inside the yellow one; when they separate, the separation *is* the
    force command.
  * **blue** -- the controlled point, the CAD pincer tip. Tracking error is the blue-to-magenta gap
    and nothing else; blue-to-yellow is not the error being rewarded.
  * **cyan** -- the centre of the sphere the goal is drawn on, which rides with the base. The goal
    is generated in spherical coordinates about this point, so it is the frame the arm works in.
  * **small red** -- ten points interpolating the current trajectory from its start to its goal, in
    spherical coordinates, the same lerp the goal itself follows.
  * **arrows from the tip and from above the base** -- blue for the force actually applied,
    green for the force commanded, each a hundredth of a metre per newton, as upstream scales them.

Two deliberate departures, both cosmetic. Upstream's trajectory spheres have a 0.005 m radius,
which reads as a wireframe dot in Isaac Gym and as almost nothing as a solid sphere here, so they
are 0.01. And upstream draws a green sphere at the world origin of environment 0, which is a
legged_gym debugging aid rather than part of the task; it is not reproduced.

Nothing here is evidence. It is an aid for watching a policy, and it runs only when a viewer is
open -- `set_debug_vis(False)`, the default, allocates none of it.
"""
from __future__ import annotations

import torch

#: Markers per environment: goal, force-displaced goal, tip, sphere centre, ten trajectory
#: samples, and four arrows (tip measured/commanded, base measured/commanded).
TRAJECTORY_SAMPLES = 10
MARKERS_PER_ENV = 4 + TRAJECTORY_SAMPLES + 4

#: Upstream's `forces / 100`: one centimetre of arrow per newton.
NEWTONS_PER_METRE = 100.0

#: Upstream draws the base arrows half a metre above the base so they clear the body.
BASE_ARROW_LIFT_M = 0.5

#: The arrow prototype's scale. X is 1.0 so `update` can multiply it by the force length; Y and Z
#: are the cross-section, slimmer than the library's stock 0.1 because these are drawn at the tool
#: tip and over the base, where a 10 cm-thick arrow swallows the robot. Named rather than inlined
#: so the contract can be checked without importing `isaaclab.sim`, which needs a running app.
ARROW_SCALE = (1.0, 0.02, 0.02)


def marker_cfg(prim_path: str):
    """The seven prototypes, in the index order `TaskMarkers` uses."""
    import isaaclab.sim as sim_utils
    from isaaclab.markers import VisualizationMarkersCfg
    from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

    def sphere(radius: float, colour: tuple[float, float, float]):
        return sim_utils.SphereCfg(
            radius=radius, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=colour))

    def arrow(colour: tuple[float, float, float]):
        return sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
            scale=ARROW_SCALE,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=colour))

    return VisualizationMarkersCfg(prim_path=prim_path, markers={
        "goal": sphere(0.05, (1.0, 1.0, 0.0)),
        "goal_forced": sphere(0.05, (1.0, 0.0, 1.0)),
        "tip": sphere(0.05, (0.0, 0.0, 1.0)),
        "centre": sphere(0.05, (0.0, 1.0, 1.0)),
        "trajectory": sphere(0.01, (1.0, 0.0, 0.0)),
        "force_measured": arrow((0.0, 0.0, 1.0)),
        "force_commanded": arrow((0.0, 1.0, 0.0)),
    })


GOAL, GOAL_FORCED, TIP, CENTRE, TRAJECTORY, FORCE_MEASURED, FORCE_COMMANDED = range(7)


def arrow_pose(vectors: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """Orientations and scales placing an +X arrow prototype along each vector.

    Returns the quaternion rotating +X onto the vector and the length in metres. The length is a
    *multiplier* on the prototype's own X scale, never the whole scale: `visualize()` takes an
    absolute scale, so passing a bare `(length, 1, 1)` silently discards the prototype's lateral
    scale and draws an arrow the full width of the robot. Isaac Lab's own velocity arrows start
    from `cfg.markers[...].scale` for the same reason.
    """
    length = torch.norm(vectors, dim=-1, keepdim=True)
    direction = vectors / length.clamp(min=1e-8)
    x = torch.zeros_like(direction)
    x[:, 0] = 1.0
    axis = torch.cross(x, direction, dim=-1)
    sin = torch.norm(axis, dim=-1, keepdim=True)
    cos = (direction[:, :1]).clamp(-1.0, 1.0)
    # Antiparallel is the one case the cross product cannot orient: any axis perpendicular to x
    # does, so pick one rather than dividing by zero.
    fallback = torch.zeros_like(axis)
    fallback[:, 2] = 1.0
    axis = torch.where(sin > 1e-6, axis / sin.clamp(min=1e-8), fallback)
    angle = torch.atan2(sin, cos)
    half = angle * 0.5
    quat = torch.cat((torch.cos(half), axis * torch.sin(half)), dim=-1)
    return quat, length.clamp(min=1e-4).squeeze(-1)


class TaskMarkers:
    """The overlay for one environment set. Create it only when a viewer is open."""

    def __init__(self, prim_path: str, num_envs: int, device: str) -> None:
        from isaaclab.markers import VisualizationMarkers

        self.markers = VisualizationMarkers(marker_cfg(prim_path))
        self.num_envs = num_envs
        self.device = device
        # The prototype each marker slot uses never changes, only where the slots are, so the
        # index vector is built once. Order within an environment must match `update()`.
        per_env = ([GOAL, GOAL_FORCED, TIP, CENTRE]
                   + [TRAJECTORY] * TRAJECTORY_SAMPLES
                   + [FORCE_MEASURED, FORCE_COMMANDED, FORCE_MEASURED, FORCE_COMMANDED])
        self.indices = torch.tensor(per_env * num_envs, device=device, dtype=torch.long)
        self._identity = torch.zeros(num_envs * MARKERS_PER_ENV, 4, device=device)
        self._identity[:, 0] = 1.0
        # Each prototype's own scale, so every marker starts at the size its config asks for.
        # `visualize()` wants an absolute scale and overwrites the prototype's, so anything not
        # read back from the config here is silently thrown away.
        defaults = torch.tensor(
            [tuple(getattr(cfg, "scale", None) or (1.0, 1.0, 1.0))
             for cfg in self.markers.cfg.markers.values()], device=device, dtype=torch.float32)
        self._base_scale = defaults[self.indices]

    def update(self, *, goal_w: torch.Tensor, target_w: torch.Tensor, tip_w: torch.Tensor,
               centre_w: torch.Tensor, trajectory_w: torch.Tensor,
               tip_force_measured_w: torch.Tensor, tip_force_commanded_w: torch.Tensor,
               base_pos_w: torch.Tensor, base_force_measured_w: torch.Tensor,
               base_force_commanded_w: torch.Tensor) -> None:
        """Place every marker. All positions are world-frame; all forces are newtons in world."""
        n, s = self.num_envs, TRAJECTORY_SAMPLES
        translations = torch.zeros(n * MARKERS_PER_ENV, 3, device=self.device)
        orientations = self._identity.clone()
        scales = self._base_scale.clone()

        view = translations.view(n, MARKERS_PER_ENV, 3)
        view[:, 0] = goal_w
        view[:, 1] = target_w
        view[:, 2] = tip_w
        view[:, 3] = centre_w
        view[:, 4:4 + s] = trajectory_w

        arrow_origin = base_pos_w.clone()
        arrow_origin[:, 2] += BASE_ARROW_LIFT_M
        for slot, (start, force) in enumerate((
                (tip_w, tip_force_measured_w), (tip_w, tip_force_commanded_w),
                (arrow_origin, base_force_measured_w), (arrow_origin, base_force_commanded_w))):
            quat, length = arrow_pose(force / NEWTONS_PER_METRE)
            view[:, 4 + s + slot] = start
            orientations.view(n, MARKERS_PER_ENV, 4)[:, 4 + s + slot] = quat
            scales.view(n, MARKERS_PER_ENV, 3)[:, 4 + s + slot, 0] *= length

        self.markers.visualize(translations=translations, orientations=orientations,
                               scales=scales, marker_indices=self.indices)

    def set_visibility(self, visible: bool) -> None:
        self.markers.set_visibility(visible)
