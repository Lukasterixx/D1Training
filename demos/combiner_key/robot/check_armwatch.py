#!/usr/bin/env python3
"""Item 1b (2026-11-19): would the dog runner's `reach_arm.ArmWatch` stop a learned insertion in contact? From
`--insert_trace_body` runs (`insert_trace.npz`, with `arm_goal` and `arm_reported`). CPU only.

    python demos/combiner_key/robot/check_armwatch.py logs/unifp_train/<run> [<run> ...] [--contact_n 1]

Each episode's arm goals (the layer's, as `insert_live` sends them) go through `ArmWatch.check` against the simulated
link's report, one policy step apart, the grace from the first goal as `insert_live` begins it. The report is the
simulator's `arm_link.ArmLink.reported` (its delay and quantisation): always fresh, so the report-gap stop is not
tested. Per run: how many episodes it would stop, by which rule and in which phase, and the lead (goal minus report,
the worst joint) in contact (the key's normal force over `--contact_n`) against the 25 deg / 0.5 s and 60 deg limits.

What it does not show: the dog's arm. The lead in contact is the simulated D1's (its drive and stiffness), and how far the
real arm lags a goal pressing a key on a lock is not measured.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

from demos.combiner_key.robot import reach_arm  # noqa: E402

DT = 0.02                    # interface.POLICY_DT
PHASES = {0: "approach", 1: "search", 2: "seat", 3: "hold", 4: "done"}     # keylock.APPROACH ... DONE


class Reports:
    """`reach_arm.ArmReports`' face for `ArmWatch.check`: the simulated report, stamped now."""

    def __init__(self):
        self.q, self.stamp = None, None

    def age(self, now_s: float) -> float:
        return 0.0


def episode(t: dict, i: int, contact_n: float) -> dict:
    alive = t["alive"][:, i] > 0.5
    steps = int(alive.sum()) - 1         # the last alive step is the reset's new robot (check_odometry.py)
    goal, rep = t["arm_goal"][:steps, i].astype(float), t["arm_reported"][:steps, i].astype(float)
    lead = np.degrees(np.abs(goal - rep)).max(axis=1)
    joint = np.degrees(np.abs(goal - rep)).argmax(axis=1) + 1
    step = np.r_[0.0, np.degrees(np.abs(np.diff(goal, axis=0))).max(axis=1)]
    contact = t["normal"][:steps, i] > contact_n
    watch, reports = reach_arm.ArmWatch(), Reports()
    watch.begin(0.0)
    stop = None
    for k in range(steps):
        reports.q, reports.stamp = rep[k], k * DT
        why = watch.check(k * DT, reports, goal[k])
        if why:
            stop = {"step": k, "s": round(k * DT, 2), "why": why, "phase": PHASES.get(int(t["phase"][k, i]), str(int(t["phase"][k, i]))),
                    "contact": bool(contact[k])}
            break
    after = np.arange(steps) * DT >= reach_arm.GRACE_S
    c = contact & after
    return {"steps": steps, "stop": stop, "lead_max": float(lead[after].max()) if after.any() else 0.0,
            "lead_contact_max": float(lead[c].max()) if c.any() else None,
            "lead_contact_p95": float(np.percentile(lead[c], 95)) if c.any() else None,
            "lead_contact_joint": int(np.bincount(joint[c]).argmax()) if c.any() else None,
            "step_max": float(step[1:steps][after[1:steps]].max()) if after[1:].any() else 0.0,
            "contact_steps": int(c.sum())}


def run(path: Path, contact_n: float) -> dict:
    t = dict(np.load(path / "insert_trace.npz"))
    if "arm_goal" not in t:
        raise SystemExit(f"{path}: no arm goal in the trace (an insertion run with --insert_trace_body, after 2026-11-19)")
    eps = json.loads((path / "insert_eval_episodes.json").read_text())["episodes"]
    rows = [dict(episode(t, i, contact_n), success=bool(e["success"] and e["max_tilt_deg"] <= 15.0)) for i, e in enumerate(eps)]
    stops = [r for r in rows if r["stop"]]
    by = {}
    for r in stops:
        why = r["stop"]["why"]
        rule = "step" if "stepped" in why else "lead held" if " for " in why else "hard lead" if "ahead" in why else why
        key = f"{rule}, {r['stop']['phase']}{', in contact' if r['stop']['contact'] else ''}"
        by[key] = by.get(key, 0) + 1
    lc = np.array([r["lead_contact_max"] for r in rows if r["lead_contact_max"] is not None])
    return {"run": path.name, "episodes": len(rows), "would_stop": len(stops),
            "would_stop_successes": sum(r["success"] for r in stops), "stops_by": by,
            "lead_contact_max_deg": {"median": round(float(np.median(lc)), 2), "p90": round(float(np.percentile(lc, 90)), 2),
                                     "max": round(float(lc.max()), 2)} if len(lc) else None,
            "lead_contact_joint": int(np.bincount([r["lead_contact_joint"] for r in rows if r["lead_contact_joint"]]).argmax()) if len(lc) else None,
            "lead_after_grace_max_deg": round(max(r["lead_max"] for r in rows), 2),
            "step_after_grace_max_deg": round(max(r["step_max"] for r in rows), 2),
            "first_stops": [dict(episode=i, **r["stop"]) for i, r in enumerate(rows) if r["stop"]][:5]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("runs", nargs="+")
    parser.add_argument("--contact_n", type=float, default=1.0)
    args = parser.parse_args()
    for p in args.runs:
        print(json.dumps(run(Path(p), args.contact_n)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
