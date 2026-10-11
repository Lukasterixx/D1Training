"""Render the insertion demo headless from four fixed views, every SHOT_EVERY policy steps: the figures
insertion_door_*.png (Week 4 log, 2026-10-09, "The insertion demo dressed as a door").

Wraps demos/insertion/run_demo.py unchanged: AppLauncher is patched so that, once the app is up, the demo's env class
is replaced by a subclass that adds four cameras (placed from the estimated mouth) and writes their RGB to SHOT_DIR.
Arguments go to run_demo.py; it needs --enable_cameras. The figures came from

    SHOT_DIR=<dir> SHOT_EVERY=25 python results/week_04/figures/insertion_scene_renders.py \
        --checkpoint checkpoints/unifp_go2d1_insertion_seed3_measured_model_21400.pt --compliant_key --num_envs 1 \
        --attempts 2 --lateral_mm 2 --hold_s 1.5 --headless --enable_cameras --scenery --camera_body --run_name render_check3

(insertion_door_room.png is a02_s0375_side, _key_seated a02_s0375_close, _wrist_camera a00_s0300_hand; "a02" is the
second attempt, because the episode's own reset at its end counts as one.)
"""
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demos/insertion"))
OUT = Path(os.environ["SHOT_DIR"])
OUT.mkdir(parents=True, exist_ok=True)
EVERY = int(os.environ.get("SHOT_EVERY", "50"))

import isaaclab.app as app_mod  # noqa: E402

_Orig = app_mod.AppLauncher


def install():
    import cv2
    import torch

    import isaaclab.sim as sim_utils
    from isaaclab.sensors import Camera, CameraCfg

    import demos.insertion.env as E

    VIEWS = {  # eye, target offsets from the estimated mouth (env frame)
        "side": ((-0.95, -1.25, 0.35), (-0.25, 0.0, -0.15)),
        "close": ((-0.26, -0.20, 0.10), (0.0, 0.0, 0.0)),
        "behind": ((-1.45, 0.55, 0.75), (-0.1, 0.0, -0.1)),
        "hand": ((-0.42, 0.22, 0.20), (-0.17, 0.0, 0.02)),
    }

    class Shot(E.InsertionDemoEnv):
        def _setup_scene(self):
            super()._setup_scene()
            self._shot_cams = {}
            for name in VIEWS:
                cfg = CameraCfg(prim_path=f"/World/envs/env_.*/Shot_{name}", update_period=0.0, width=1024,
                                height=768, data_types=["rgb"],
                                spawn=sim_utils.PinholeCameraCfg(focal_length=18.0 if name != "close" else 24.0,
                                                                 clipping_range=(0.01, 30.0)))
                self._shot_cams[name] = Camera(cfg)
                self.scene.sensors[f"shot_{name}"] = self._shot_cams[name]
            self._shot_count = 0
            self._shot_attempt = -1

        def _reset_idx(self, env_ids):
            super()._reset_idx(env_ids)
            self._shot_attempt += 1
            self._shot_count = 0

        def step(self, action):
            mouth = self._est_mouth + self.scene.env_origins
            for name, (eye, target) in VIEWS.items():
                e = mouth + torch.tensor(eye, device=self.device)
                t = mouth + torch.tensor(target, device=self.device)
                self._shot_cams[name].set_world_poses_from_view(e, t)
            out = super().step(action)
            if self._shot_count % EVERY == 0:
                for name, cam in self._shot_cams.items():
                    rgb = cam.data.output["rgb"][0, ..., :3].cpu().numpy()
                    path = OUT / f"a{self._shot_attempt:02d}_s{self._shot_count:04d}_{name}.png"
                    cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            self._shot_count += 1
            return out

    E.InsertionDemoEnv = Shot


class Patched(_Orig):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        install()


app_mod.AppLauncher = Patched

import run_demo  # noqa: E402

sys.argv = ["run_demo.py", *sys.argv[1:]]
raise SystemExit(run_demo.main())
