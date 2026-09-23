#!/usr/bin/env python3
"""Screen a training run for the drift that precedes a policy collapse.

    ./unifp_train/kl_drift.py                       # every run under logs/unifp_train
    ./unifp_train/kl_drift.py logs/unifp_train/2026*_p0   # named runs
    ./unifp_train/kl_drift.py --window 300          # iterations to judge on

The long run of 2026-09-20 collapsed three times, and each collapse was visible beforehand only
as the KL divergence creeping upward across thousands of iterations. Measured across nine
configurations, the *drift* in KL over the first 300 iterations predicts the loss of end-effector
tracking with r = -0.74, while the mean KL predicts almost nothing: every configuration starts
between 0.0138 and 0.0149 regardless of how it ends up.

So this reads the first `window` iterations of a run and reports whether the KL is holding or
climbing. A run whose KL has risen by more than about half over that window has, on the evidence
so far, already decided to degrade, and there is no point spending the other forty hours on it.

That is a correlation over nine single runs, not a law. It is useful because it is cheap --
thirteen minutes against eight hours -- and it should be treated as a screen, not a verdict.
"""
from __future__ import annotations

import argparse
import json
import re
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LOGS = ROOT / "logs/unifp_train"

ANSI = re.compile(r"\x1b\[[0-9;]*m")
BOLD, DIM, RESET = "\x1b[1m", "\x1b[2m", "\x1b[0m"
GREEN, YELLOW, RED = "\x1b[32m", "\x1b[33m", "\x1b[31m"

#: A run whose KL grows by more than this over the window is expected to degrade.
DRIFT_WARN = 0.50
DRIFT_BAD = 1.00


def find_log(run_dir: Path) -> Path | None:
    """The captured stdout for a run directory.

    The log file's name is whatever the shell redirected to and has no fixed relationship to the
    run directory, so the match is made on the `Run name:` line rsl-rl's logger prints in every
    iteration block, against the `run_name` the launcher recorded in `run.json`.
    """
    try:
        meta = json.loads((run_dir / "run.json").read_text())
    except (OSError, ValueError):
        return None
    # Runs from 2026-09-21 onward record their arguments; older ones only encode the name in the
    # directory, which the launcher builds as "<stamp>_train_seed<seed>_<run_name>".
    run_name = (meta.get("arguments") or {}).get("run_name")
    if not run_name:
        marker = f"_train_seed{meta.get('seed')}_"
        run_name = run_dir.name.split(marker, 1)[-1] if marker in run_dir.name else None
    if not run_name:
        return None
    marker = f"Run name: {run_name}"
    for candidate in sorted(LOGS.glob("*.log"), key=lambda p: -p.stat().st_mtime):
        try:
            if marker in ANSI.sub("", candidate.read_text(errors="ignore")[:400000]):
                return candidate
        except OSError:
            continue
    return None


def read(log_path: Path, window: int):
    text = ANSI.sub("", log_path.read_text(errors="ignore"))
    kl = [float(x) for x in re.findall(r"Mean kl loss: (-?[\d.]+)", text)][:window]
    ee = [float(x) for x in
          re.findall(r"Episode_Reward/tracking_ee_force_world: (-?[\d.]+)", text)][:window]
    return kl, ee


def assess(kl: list[float], ee: list[float]) -> dict | None:
    """Compare the first and last fifty logged iterations of the window."""
    if len(kl) < 120:
        return None
    span = max(25, len(kl) // 6)
    first, last = statistics.mean(kl[:span]), statistics.mean(kl[-span:])
    out = {"iterations": len(kl), "kl_first": first, "kl_last": last,
           "drift": (last - first) / first if first else float("nan")}
    if len(ee) >= len(kl):
        out["ee_first"] = statistics.mean(ee[span:2 * span])
        out["ee_last"] = statistics.mean(ee[-span:])
        out["ee_change"] = ((out["ee_last"] - out["ee_first"]) / out["ee_first"]
                            if out["ee_first"] else float("nan"))
    return out


def describe(run_dir: Path) -> str:
    try:
        meta = json.loads((run_dir / "run.json").read_text())
    except (OSError, ValueError):
        return "?"
    parts = [f"{meta.get('num_envs', '?')} envs", f"seed {meta.get('seed', '?')}"]
    departures = meta.get("departures_from_unifp") or {}
    parts.append(", ".join(f"{k}={v}" for k, v in departures.items()) if departures else "unchanged")
    return "  ".join(parts)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="*", help="run directories; default is all of them")
    parser.add_argument("--window", type=int, default=300,
                        help="iterations to judge on (default 300)")
    args = parser.parse_args()

    runs = [Path(r) for r in args.runs] or sorted(p for p in LOGS.glob("*_train_*") if p.is_dir())
    rows = []
    for run_dir in runs:
        log_path = find_log(run_dir)
        if log_path is None:
            continue
        verdict = assess(*read(log_path, args.window))
        if verdict:
            rows.append((run_dir, verdict))

    if not rows:
        print("No run has enough instrumented iterations yet. The KL is only logged by runs "
              "started after 2026-09-21; earlier ones cannot be screened.")
        return 1

    print(f"\n{BOLD}KL drift over the first {args.window} iterations{RESET}"
          f"   {DIM}rising KL precedes the collapse (r = -0.74 over nine runs){RESET}")
    print("─" * 96)
    print(f"{'run':34s} {'configuration':30s} {'KL':>14} {'drift':>8} {'ee':>9}")
    for run_dir, v in sorted(rows, key=lambda r: r[1]["drift"]):
        colour = GREEN if v["drift"] < DRIFT_WARN else (YELLOW if v["drift"] < DRIFT_BAD else RED)
        ee = f"{v['ee_change']:+8.1%}" if "ee_change" in v else "        -"
        print(f"{run_dir.name.split('_train_')[-1][:34]:34s} {describe(run_dir)[:30]:30s} "
              f"{v['kl_first']:6.4f}→{v['kl_last']:6.4f} {colour}{v['drift']:+7.0%}{RESET} {ee}")
    print()
    print(f"  {GREEN}under +50%{RESET}: holding    {YELLOW}+50 to +100%{RESET}: watch    "
          f"{RED}over +100%{RESET}: expect the tracking term to fall")
    print(f"  {DIM}A screen, not a verdict: nine single runs, one per configuration.{RESET}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
