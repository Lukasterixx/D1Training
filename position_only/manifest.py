"""Frozen evaluation manifests: the fixed episode sets G1a is measured on.

A manifest names every case a policy is evaluated against, so P0-P4 and every seed see matched
episodes. It is written once, hashed, and not regenerated: rebuilding it with the same arguments
must give the same file, and a changed `content_sha256` means the comparison changed.

Three roles, kept separate so checkpoint selection cannot leak into the reported result
(`docs/thesis_b_plan.md`, "Controlled training and testing"):

- `development` - free to look at while building and tuning.
- `validation`  - checkpoint selection only.
- `test`        - untouched until the final result. Do not use it to choose anything.

The manifest fixes the targets, the initial state and the conditions the episodes run under. It
does not fix the policy: that is the thing being measured.

    python run_position_only.py manifest --role test --episodes 100

CPU only, no Isaac imports, so the manifests can be built and inspected without a simulator.
"""
from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from .task_space import SPAWN_HEIGHT_M, TARGET_RANGES
from .tool_point import TOOL_BODY, TOOL_OFFSET_M

FORMAT = "position_only_eval_manifest_v1"
SUCCESS_RADIUS_M = 0.05  # G1a
DWELL_S = 1.0  # G1a: continuous time inside the success radius
EPISODE_LENGTH_S = 10.0  # G1a
ROLES = ("development", "validation", "test")
# Distinct RNG streams per role, so the three sets are drawn independently rather than being
# prefixes of one sequence. Changing these changes every manifest, hence the recorded hash.
ROLE_SEEDS = {"development": 20260916, "validation": 20260917, "test": 20260918}


def _digest(payload):
    """Hash of the episode content and the conditions, not of the surrounding metadata."""
    material = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(material).hexdigest()


def build(role, episodes=100, ranges=TARGET_RANGES, spawn_height=SPAWN_HEIGHT_M,
          robustness="none", latency="estimated", leg_actuator="unitree", arm_actuator="d1_servo",
          self_collisions=True, seed=None):
    """A manifest as a plain dict. Deterministic in its arguments."""
    if role not in ROLES:
        raise ValueError(f"Unknown manifest role: {role!r}; expected one of {ROLES}")
    if episodes < 1:
        raise ValueError(f"A manifest needs at least one episode, got {episodes}")
    seed = ROLE_SEEDS[role] if seed is None else int(seed)
    rng = np.random.default_rng(seed)
    ranges = tuple(tuple(float(v) for v in axis) for axis in ranges)
    # One draw per axis in a fixed order, so the stream does not depend on numpy's broadcasting.
    targets = np.stack([rng.uniform(low, high, size=episodes) for low, high in ranges], axis=-1)

    conditions = {
        # Everything that must match across policies for the comparison to mean anything.
        "spawn_height_m": float(spawn_height),
        "target_box_env_frame_m": [list(axis) for axis in ranges],
        "robustness": robustness,
        "latency": latency,
        "leg_actuator": leg_actuator,
        "arm_actuator": arm_actuator,
        "self_collisions": bool(self_collisions),
        "tool_body": TOOL_BODY,
        "tool_offset_m": list(TOOL_OFFSET_M),
        "episode_length_s": EPISODE_LENGTH_S,
        "success_radius_m": SUCCESS_RADIUS_M,
        "dwell_s": DWELL_S,
    }
    content = {
        "conditions": conditions,
        "episodes": [{"index": i, "target_env_frame_m": targets[i].round(6).tolist()} for i in range(episodes)],
    }
    return {
        "format": FORMAT,
        "role": role,
        "episode_count": episodes,
        "seed": seed,
        "created_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "content_sha256": _digest(content),
        "note": "Frozen evaluation cases. Rebuilding with the same arguments reproduces this file; "
                "a different content_sha256 means the comparison changed.",
        **content,
    }


def write(manifest, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, indent=2) + "\n")
    return path


def load(path):
    """Load a manifest and re-check its hash, so a hand-edited file cannot pass unnoticed."""
    manifest = json.loads(Path(path).read_text())
    if manifest.get("format") != FORMAT:
        raise ValueError(f"{path}: not a {FORMAT} manifest (got {manifest.get('format')!r})")
    content = {"conditions": manifest["conditions"], "episodes": manifest["episodes"]}
    actual = _digest(content)
    if actual != manifest["content_sha256"]:
        raise ValueError(f"{path}: content_sha256 does not match the episodes and conditions "
                         f"(recorded {manifest['content_sha256']}, actual {actual}). The manifest was edited.")
    return manifest


def targets(manifest):
    return np.asarray([episode["target_env_frame_m"] for episode in manifest["episodes"]], dtype=np.float64)


def mismatches(manifest, conditions):
    """Which of the manifest's conditions the given run does not meet, as human-readable lines."""
    out = []
    for key, expected in manifest["conditions"].items():
        if key not in conditions:
            continue
        actual = conditions[key]
        if isinstance(expected, float) or isinstance(actual, float):
            same = abs(float(expected) - float(actual)) < 1e-9
        else:
            same = json.loads(json.dumps(expected)) == json.loads(json.dumps(actual))
        if not same:
            out.append(f"{key}: manifest {expected!r}, run {actual!r}")
    return out
