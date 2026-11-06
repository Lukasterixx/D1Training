#!/usr/bin/env python3
"""Where in the D1's report cycle each command landed, and whether the next report came (F-238). Reads records only;
it commands nothing.

    python demos/combiner_door/robot/arm_gap_phase.py logs/arm_command_rate/<stamp> logs/holster_dog/<stamp>_live ...

Takes a rate test's directory (`rate_test.json`, one row a block) or a live run's (`arm_link.jsonl`, one row a run), each
with `feedback_timing/samples.jsonl` beside it. Only commands the arm executes count: those with the gripper's angle
(F-202). A report's time is its source stamp, put on the dog's clock by a line fitted to the arrival times, so the arm's
drift (~5 ms in 140 s) drops out and the network's mean latency stays in.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

#: A report later than this skipped at least one (they come every 111 ms).
SKIP_S = 0.16
#: Commands this long after a report stalled the arm on the moving runs' timestamps (d1_hardware.STALL_WINDOW_S).
WINDOW_S = (0.088, 0.110)


def reports_on_dog_clock(samples_path: Path) -> np.ndarray:
    rows = [json.loads(line) for line in samples_path.read_text().splitlines()]
    rows = [r for r in rows if r["topic"] == "current_servo_angle" and r.get("src") is not None]
    src = np.array([r["src"] for r in rows])
    wall = np.array([r["wall"] for r in rows])
    fit = np.polyfit(wall - wall[0], wall - src, 1)
    return src + np.polyval(fit, wall - wall[0])


def cycle_rows(reports: np.ndarray, commands: np.ndarray) -> list[dict]:
    """One row a report-to-report cycle: the commands after it (ms, up to 125), and whether the next report skipped."""
    rows = []
    for this, nxt in zip(reports, reports[1:]):
        after = commands[(commands > this) & (commands <= this + 0.125)]
        rows.append({"t": float(this), "interval_s": float(nxt - this), "skipped": bool(nxt - this > SKIP_S),
                     "commands_ms": ((after - this) * 1e3).tolist()})
    return rows


def summarise(rows: list[dict]) -> dict:
    lo, hi = WINDOW_S[0] * 1e3, WINDOW_S[1] * 1e3
    hit = [any(lo <= u <= hi for u in r["commands_ms"]) for r in rows]
    skipped = [r for r in rows if r["skipped"]]
    span = sum(r["interval_s"] for r in rows)
    lost = sum(r["interval_s"] for r in skipped)
    return {"cycles": len(rows), "skips": len(skipped), "seconds": round(span, 1), "lost_s": round(lost, 2),
            "lost_pct": round(100 * lost / span, 1) if span else 0.0, "longest_s": round(max(
                (r["interval_s"] for r in rows), default=0.0), 3),
            "in_window": sum(hit), "in_window_skipped": sum(h and r["skipped"] for h, r in zip(hit, rows)),
            "outside": len(rows) - sum(hit), "outside_skipped": sum(not h and r["skipped"] for h, r in zip(hit, rows))}


def rate_test_blocks(run: Path) -> list[tuple[str, np.ndarray, np.ndarray]]:
    reports = reports_on_dog_clock(run / "feedback_timing" / "samples.jsonl")
    out = []
    for block in json.loads((run / "rate_test.json").read_text())["blocks"]:
        every = block.get("gripper_every", 1 if block.get("gripper", True) else 0)
        if not block["hz"]:
            continue
        commands = np.array(block["commands"])[::every] if every else np.array([])
        mask = (reports >= block["wall_start"]) & (reports <= block["wall_end"])
        out.append((block["name"], reports[mask], commands))
    return out


def live_run(run: Path) -> tuple[str, np.ndarray, np.ndarray]:
    reports = reports_on_dog_clock(run / "feedback_timing" / "samples.jsonl")
    rows = [json.loads(line) for line in (run / "arm_link.jsonl").read_text().splitlines()]
    commands = np.array([r["t"] for r in rows if r.get("kind") == "command" and '"angle6"' in r.get("sent", "")])
    return run.name, reports[(reports >= commands[0]) & (reports <= commands[-1])], commands


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("runs", nargs="+", type=Path)
    p.add_argument("--out", type=Path, help="write every cycle here as JSON")
    args = p.parse_args(argv)
    every = {}
    for run in args.runs:
        parts = rate_test_blocks(run) if (run / "rate_test.json").exists() else [live_run(run)]
        for name, reports, commands in parts:
            rows = cycle_rows(reports, commands)
            every[f"{run.name}/{name}"] = rows
            print(f"{run.name}/{name}", json.dumps(summarise(rows)))
    if args.out:
        args.out.write_text(json.dumps(every))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
