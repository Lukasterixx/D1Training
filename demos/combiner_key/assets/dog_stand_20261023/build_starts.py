"""Build the foot-press fine-tune's start library (`ft_starts.json`) from the dog's handover, 2026-10-23.

    python3 demos/combiner_key/assets/dog_stand_20261023/build_starts.py

Three families, in the spawn library's format (`leg_env.Go2D1LegEnvCfg.spawn_states`: legs in UniFP's order, base_z,
roll, pitch; joints jittered again at spawn by `spawn_state_jitter`):

- `sim_handover_sink.json` (`unifp_train/leg_handover.py`, 64 robots with leg friction 0-2 N·m, 11 times each from
  the end of the kp-80 hold to 4 s after it, with their joint and base velocities): the handover's sink in simulation,
  from the hold that matches the dog's (0.306 m and 4.9 degrees nose up there, 0.303 and 4.6 on the dog), dropping at
  up to 0.18 m/s, to a crouch lower than the dog's (0.21 m, 13 degrees).
- the dog's kp-25 rest (`kp25_rest.json`, F-191) and the path to it from the simulated hold, interpolated in joint
  space, each joint off it by up to `NOISE_RAD`; base_z by the export's kinematics so every foot clears the ground by
  2 mm, roll and pitch interpolated with the joints.
- the sport stands and walker stops round 4 trained from (`../sport_stands_20261021.json`).

Shares by count: `SHARES`.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
sys.path.insert(0, str(REPO))

from demos.combiner_key.robot import leg_policy, press_layer as pl, press_live  # noqa: E402

UNITREE_FROM_UNIFP = np.array([3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8])
SHARES = {"sim": 0.45, "dog": 0.35, "round4": 0.20}
NOISE_RAD, SEED = 0.06, 20261023


def quat(roll: float, pitch: float) -> np.ndarray:
    """As `leg_env._reset_idx` writes a spawn state's root (w, x, y, z)."""
    cr, sr, cp, sp = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return np.array([cr * cp, sr * cp, cr * sp, -sr * sp])


def base_z(legs_unifp: np.ndarray, roll: float, pitch: float, meta: dict) -> float:
    out, height = press_live.feet(legs_unifp[UNITREE_FROM_UNIFP], quat(roll, pitch), meta)
    return height + (pl.FOOT_RADIUS_M - min(v[2] for v in out.values())) + 0.002


def main() -> None:
    meta = leg_policy.LegPolicy(REPO / "checkpoints" / "leg_press_force_r4_model_5999.npz").meta
    sim = json.loads((HERE / "sim_handover_sink.json").read_text())
    dog = json.loads((HERE / "kp25_rest.json").read_text())[0]
    old = json.loads((HERE.parent / "sport_stands_20261021.json").read_text())
    hold = [s for s in sim if s["source"].endswith(", 0.0 s after the hold")]
    hold_legs = np.mean([s["legs"] for s in hold], axis=0)
    hold_rp = np.mean([[s["roll"], s["pitch"]] for s in hold], axis=0)
    dog_legs, dog_rp = np.asarray(dog["legs"]), np.array([dog["roll"], dog["pitch"]])

    rng = np.random.default_rng(SEED)
    total = round(len(sim) / SHARES["sim"])
    count = round(SHARES["dog"] * total)
    family = []
    for k in range(count):
        # A third at the dog's rest itself, the rest along the path to it from the simulated hold.
        a = 1.0 if k < count // 3 else float(rng.uniform(0.0, 1.0))
        legs = hold_legs + a * (dog_legs - hold_legs) + rng.uniform(-NOISE_RAD, NOISE_RAD, 12)
        roll, pitch = hold_rp + a * (dog_rp - hold_rp)
        family.append({"legs": [round(float(v), 5) for v in legs], "base_z": round(base_z(legs, roll, pitch, meta), 4),
                       "roll": round(float(roll), 5), "pitch": round(float(pitch), 5),
                       "source": f"dog kp-25 rest path, a={a:.2f} from the simulated hold, +-{NOISE_RAD} rad"})
    repeat = max(1, round(SHARES["round4"] * total / len(old)))
    library = sim + family + [dict(s, source="sport_stands_20261021") for s in old] * repeat
    (HERE / "ft_starts.json").write_text(json.dumps(library, indent=1))
    counts = {"sim": len(sim), "dog": len(family), "round4": len(old) * repeat}
    z = np.array([s["base_z"] for s in library])
    pitch = np.degrees([s["pitch"] for s in library])
    print(json.dumps({"entries": len(library), "counts": counts,
                      "shares": {k: round(v / len(library), 3) for k, v in counts.items()},
                      "base_z": [round(float(z.min()), 3), round(float(np.median(z)), 3), round(float(z.max()), 3)],
                      "pitch_deg": [round(float(pitch.min()), 1), round(float(np.median(pitch)), 1),
                                    round(float(pitch.max()), 1)]}, indent=1))


if __name__ == "__main__":
    main()
