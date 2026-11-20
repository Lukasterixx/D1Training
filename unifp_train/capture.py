"""`play --capture_dir DIR`: clean frames from the viewer camera (no Isaac Sim window round them) every N policy steps,
each with environment 0's task state in `frames.jsonl`, so screenshots are picked by what was happening, not by eye.

The camera is `cfg.viewer`'s (origin, eye, lookat) at `cfg.viewer.resolution`. Isaac Lab's `env.render()` in
'rgb_array' mode reads it through a replicator render product; headless, that needs `--enable_cameras`. Rendering steps
no physics and changes no state, so the episode is the one the policy plays without the capture. It is still one
episode, one draw of the contact: a lock or a mechanism need not end as the same row did in a many-environment
evaluation (week 10 log, 2026-11-20, "One environment does not repeat a lock's evaluated episode"). The state logged
with each frame is what that frame shows.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import torch

from unifp_isaaclab import interface

PHASES = ("approach", "search", "seat", "hold", "done")


def _f(value: torch.Tensor, digits: int = 4) -> float:
    return round(float(value), digits)


def frame_state(env) -> dict:
    """Environment 0's state as a frame shows it: the base's tilt, and the task's own (mechanism, insertion, key hold)."""
    out = {}
    roll_pitch = interface.body_roll_pitch(env._robot.data.root_quat_w)[0]
    out["roll_deg"], out["pitch_deg"] = (_f(math.degrees(float(v)), 2) for v in roll_pitch)
    if hasattr(env, "_mech"):
        from . import mech_cfg

        m = env._mech
        plan = getattr(env, "_evaluation", None)
        if plan:
            out["kind"] = list(mech_cfg.EVAL_KINDS)[int(plan["kinds"][0])]
            out["level_n"] = _f(plan["levels"][0], 1)
        travel = float(m.travel[0])
        out.update({"grasped": bool(m.grasped[0]), "s_m": _f(m.s[0]), "travel_m": _f(travel),
                    "open_fraction": _f(m.s[0] / travel, 3) if travel > 0 else None,
                    "opened": bool(m.grasped[0]) and travel > 0 and float(m.s[0]) >= mech_cfg.EVAL_OPENED_FRACTION * travel,
                    "drive_n": _f(m.drive_n[0], 2), "force_on_tool_n": _f(m.force_on_tool[0].norm(), 2),
                    "torn": bool(m.torn[0]), "hinge": bool(m.hinge[0])})
    if hasattr(env, "_layer_ins"):
        lk, layer = env._lock, env._layer_ins
        out.update({"phase": PHASES[int(layer.phase[0])], "started": bool(env._layer_started[0]),
                    "seated": bool(lk.seated[0]), "damaged": bool(env._damaged[0]), "lost": bool(lk.lost[0]),
                    "lock_force_n": _f(lk.force_w[0].norm(), 2)})
    elif hasattr(env, "_handle"):
        h = env._handle
        out.update({"key_deg": _f(math.degrees(float(h.theta[0])), 2), "latch_open": bool(h.latch_open()[0]),
                    "active": bool(h.active[0])})
    return out


class FrameCapture:
    def __init__(self, env, out_dir: str | Path, every: int) -> None:
        self.dir = Path(out_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.every = max(1, int(every))
        self.step = 0
        self.saved = 0
        self.episode = 0
        self._log = open(self.dir / "frames.jsonl", "w")
        env.render_mode = "rgb_array"

    def after_step(self, env, ended: bool) -> None:
        """Called after each policy step; `ended`: environment 0's episode ended on it (the frame shows its reset)."""
        self.step += 1
        if self.step % self.every == 0:
            from PIL import Image

            rgb = None
            for _ in range(2):      # the markers move on the app update the first render runs; the second shows them
                rgb = env.render()
            name = f"frame_{self.step:05d}.png"
            Image.fromarray(np.ascontiguousarray(rgb)).save(self.dir / name)
            record = {"file": name, "step": self.step, "t_s": round(self.step * interface.POLICY_DT, 3),
                      "episode": self.episode, "reset_frame": bool(ended), **frame_state(env)}
            self._log.write(json.dumps(record) + "\n")
            self._log.flush()
            self.saved += 1
        if ended:
            self.episode += 1

    def close(self) -> None:
        self._log.close()
