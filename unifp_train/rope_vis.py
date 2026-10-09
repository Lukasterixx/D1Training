"""The rope task's viewer overlay: the rope, the post it is tied to, and what happens to both when it lets go.

UniFP's overlay (`debug_vis.py`) draws the goal, the force-displaced target, the tool and the force arrows -- blue the
force applied, green the force commanded, a centimetre per newton -- but not the rope, which is all there is to see of
the task. This adds, per environment:

  * **the rope**, a thin cylinder from the jaw centre to the anchor: **orange** while it is taut (at least `TAUT_N`),
    **grey** while slack. Once the anchor has let go it disappears if it carries nothing (a snap, or a fade that has
    run out), and turns **red** while a slipping load still drags on it, following the load.
  * **the post** it was tied to, a vertical cylinder from the ground to where the rope was tied: brown, **red** once
    the anchor has let go. It stays where it was; a slipping load slides away from it along the rope.
  * **the load**, a small sphere where the rope ends now: dark while it holds, red once released.

Nothing is drawn for an environment without a rope. Viewer only: nothing is allocated until a viewer asks for it.
"""
from __future__ import annotations

import torch

from .debug_vis import arrow_pose

ROPE_RADIUS_M = 0.006
POST_RADIUS_M = 0.02
LOAD_RADIUS_M = 0.025
#: Tension below which the rope is drawn slack, N (`rope.SLACK_N`).
TAUT_N = 0.5
#: A released anchor holding less than this draws no rope: it has snapped, or faded to nothing.
CUT_N = 0.5

ROPE_TAUT, ROPE_SLACK, ROPE_SLIPPING, LOAD, LOAD_RELEASED, POST, POST_RELEASED = range(7)
MARKERS_PER_ENV = 3   # rope, load, post


def marker_cfg(prim_path: str):
    """The seven prototypes, in the index order above."""
    import isaaclab.sim as sim_utils
    from isaaclab.markers import VisualizationMarkersCfg

    def material(colour):
        return sim_utils.PreviewSurfaceCfg(diffuse_color=colour)

    def rope(colour):
        # Along +X and a metre long, so `arrow_pose` orients it and its length scales X.
        return sim_utils.CylinderCfg(radius=ROPE_RADIUS_M, height=1.0, axis="X", visual_material=material(colour))

    def post(colour):
        return sim_utils.CylinderCfg(radius=POST_RADIUS_M, height=1.0, axis="Z", visual_material=material(colour))

    def load(colour):
        return sim_utils.SphereCfg(radius=LOAD_RADIUS_M, visual_material=material(colour))

    return VisualizationMarkersCfg(prim_path=prim_path, markers={
        "rope_taut": rope((1.0, 0.45, 0.0)),
        "rope_slack": rope((0.7, 0.7, 0.7)),
        "rope_slipping": rope((0.9, 0.1, 0.1)),
        "load": load((0.15, 0.15, 0.15)),
        "load_released": load((0.9, 0.1, 0.1)),
        "post": post((0.45, 0.3, 0.15)),
        "post_released": post((0.9, 0.1, 0.1)),
    })


class RopeMarkers:
    """The overlay for one environment set. Create it only when a viewer is open."""

    def __init__(self, prim_path: str, num_envs: int, device: str) -> None:
        from isaaclab.markers import VisualizationMarkers

        self.markers = VisualizationMarkers(marker_cfg(prim_path))
        self.num_envs = num_envs
        self.device = device

    def update(self, *, tip_w: torch.Tensor, load_w: torch.Tensor, tied_w: torch.Tensor, ground_z: torch.Tensor,
               held: torch.Tensor, released: torch.Tensor, tension: torch.Tensor, capacity: torch.Tensor) -> None:
        """Place every marker. Positions world-frame; `tied_w` where the rope was tied, `load_w` where it ends now."""
        n = self.num_envs
        translations = torch.zeros(n, MARKERS_PER_ENV, 3, device=self.device)
        orientations = torch.zeros(n, MARKERS_PER_ENV, 4, device=self.device)
        orientations[..., 0] = 1.0
        scales = torch.ones(n, MARKERS_PER_ENV, 3, device=self.device)
        indices = torch.zeros(n, MARKERS_PER_ENV, dtype=torch.long, device=self.device)

        code = lambda value: torch.full((n,), value, dtype=torch.long, device=self.device)
        quat, length = arrow_pose(load_w - tip_w)
        translations[:, 0] = 0.5 * (tip_w + load_w)
        orientations[:, 0] = quat
        scales[:, 0, 0] = length
        cut = released & (capacity < CUT_N)
        indices[:, 0] = torch.where(released, code(ROPE_SLIPPING),
                                    torch.where(tension >= TAUT_N, code(ROPE_TAUT), code(ROPE_SLACK)))

        translations[:, 1] = load_w
        indices[:, 1] = torch.where(released, code(LOAD_RELEASED), code(LOAD))

        height = (tied_w[:, 2] - ground_z).clamp(min=0.01)
        translations[:, 2] = tied_w
        translations[:, 2, 2] = ground_z + 0.5 * height
        scales[:, 2, 2] = height
        indices[:, 2] = torch.where(released, code(POST_RELEASED), code(POST))

        # Hide what is not there by shrinking it to nothing: the whole set without a rope, the rope and load once cut.
        tiny = 1.0e-6
        scales[~held] = tiny
        scales[cut, 0] = tiny
        scales[cut, 1] = tiny
        self.markers.visualize(translations=translations.view(-1, 3), orientations=orientations.view(-1, 4),
                               scales=scales.view(-1, 3), marker_indices=indices.view(-1))

    def set_visibility(self, visible: bool) -> None:
        self.markers.set_visibility(visible)
