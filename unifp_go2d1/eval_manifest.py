"""Frozen evaluation manifests for the Go2+D1 UniFP task.

Same idea as `position_only/manifest.py` on the Isaac Lab side, and deliberately the same shape,
so the two records read alike: a role, a fixed episode set, the conditions the episodes must be
run under, and a content hash that changes if any of it changes.

**How an episode is frozen here is different, and the difference matters.** The position-only task
has one target per episode, so the manifest can simply state it. This task generates a whole
schedule as the episode runs -- base velocity commands on a 5 s timer, an end-effector goal
trajectory, and gripper force pushes on their own intervals -- through a few dozen random draws
inside the environment. Restating all of that in the manifest would mean reimplementing the
environment's own timing, which is exactly the sort of second source of truth that goes stale.

So the episode SET is frozen by one seed and one environment count, and each episode is verified
by its own **schedule digest**. Every draw the environment makes comes from the global torch RNG,
batched across environments, and is triggered by `episode_length_buf` rather than by anything the
policy does -- so a given (seed, episode_count) yields the same schedules for every policy,
including the zero-action baseline. The environment count is part of the frozen conditions
precisely because the draws are batched: the same seed with a different batch size is a different
episode set. The manifest records a digest of each realised schedule (goal in arm-frame spherical
coordinates, velocity command, and force command, at every step of the full episode), and the
evaluator recomputes it on every run: a policy that changed the schedule, or an environment that
has drifted, shows up as a digest mismatch rather than as a quietly different experiment.

That verification is the point. The seed alone would be a claim; the digest is a check.
"""

import hashlib
import json
import os
from datetime import datetime, timezone

FORMAT = "unifp_go2d1_eval_manifest_v1"


def _canonical(manifest):
    """The manifest without its own hash, serialised deterministically."""
    body = {k: v for k, v in manifest.items() if k != "content_sha256"}
    return json.dumps(body, sort_keys=True, separators=(",", ":")).encode()


def content_hash(manifest):
    return hashlib.sha256(_canonical(manifest)).hexdigest()


def build(role, episode_count, seed, conditions, episodes=None):
    manifest = {
        "format": FORMAT,
        "role": role,
        "episode_count": episode_count,
        "seed": seed,
        "created_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "note": ("Frozen evaluation cases for the Go2+D1 UniFP position/force task. The set is "
                 "one seeded batch of episode_count environments; each episode's schedule digest "
                 "is what proves the episode that ran is the episode that was frozen. The "
                 "environment count is part of the conditions because the schedule is drawn in "
                 "batches. A different content_sha256 means the set changed and results before "
                 "and after are not comparable."),
        "conditions": conditions,
        # One episode per environment in one seeded batch; the digest is filled in by `build`.
        "episodes": episodes if episodes is not None else [
            {"index": i} for i in range(episode_count)
        ],
    }
    manifest["content_sha256"] = content_hash(manifest)
    return manifest


def save(manifest, path):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as handle:
        json.dump(manifest, handle, indent=2)
        handle.write("\n")
    return path


def load(path):
    manifest = json.load(open(path))
    if manifest.get("format") != FORMAT:
        raise ValueError(f"{path}: not a {FORMAT} manifest (got {manifest.get('format')!r})")
    recomputed = content_hash(manifest)
    if recomputed != manifest["content_sha256"]:
        raise ValueError(f"{path}: content_sha256 does not match its contents "
                         f"({recomputed} != {manifest['content_sha256']}). The file has been "
                         f"edited by hand; rebuild it instead.")
    return manifest


def compare_conditions(manifest, actual):
    """[(key, expected, got)] for every declared condition the run did not match."""
    out = []
    for key, expected in manifest["conditions"].items():
        if key not in actual:
            out.append((key, expected, "<not reported>"))
            continue
        got = actual[key]
        if isinstance(expected, float) and isinstance(got, (int, float)):
            if abs(float(got) - expected) > 1e-9:
                out.append((key, expected, got))
        elif expected != got:
            out.append((key, expected, got))
    return out
