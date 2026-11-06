#!/usr/bin/env python3
"""The arm link against the real arm: the commands the arm was sent, replayed, against what it reported.

Five 20 s blocks of Joint1 on a locked stream, 2026-11-06 (`tests/data/d1_locked_swing.json`). Replaying them, not a triangle
written here, is the point: their 10 commands a period are locked to the triangle, so the speeds read off the reports depend
on where those land, and on a best-fit shift the same arm reads 98 ms against held commands and 150 against the sends as
points. Read both sides with the same functions and neither can catch you out. An earlier fit did not, and came out 60 ms slow.

    python -m unifp_train.arm_link_check

`replay` is the arm alone, at the sends' own times. `stream` runs the same commands through the locked link, on the dog's
sender's 5 ms steps and on training's 20 ms. Torch only, on the CPU: the first-order lag stands for the joint.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import torch

import motor_model
from unifp_train.arm_link import ArmLink

DATA = Path(__file__).resolve().parent.parent / "tests/data/d1_locked_swing.json"
#: All six joints with the key in the jaws: nothing in the profile was fitted to this one.
DRAW = Path(__file__).resolve().parent.parent / "tests/data/d1_locked_draw.json"
#: Holds the one skipped report, which no arm model should fit: left out of the RMS, as a skip is not the arm's dynamics.
SKIPPED = "20261106T021303_swing_lock_20ms_again"
#: `motor_model`'s locked profile, as ArmLink takes it.
LOCKED = {"plan": motor_model.arm_trajectory("locked"), "servo_lag_s": motor_model.D1_LOCKED_SERVO_LAG_S}


def blocks() -> dict:
    return json.loads(DATA.read_text())["blocks"]


def best_shift_ms(sends: np.ndarray, reports: np.ndarray, skip_s: float = 3.0) -> float:
    """The time shift that best lays the commanded angle, *held* from each send, over the reports, ms. Not the shift against
    the sends as points, which is ~55 ms more."""
    grid = np.arange(reports[0, 0] + skip_s, reports[-1, 0] - 1.0, 0.001)
    rep = np.interp(grid, reports[:, 0], reports[:, 1])
    rep = rep - rep.mean()
    best, best_err = 0.0, np.inf
    for shift in np.arange(0.0, 0.4, 0.001):
        cmd = sends[np.clip(np.searchsorted(sends[:, 0], grid - shift, side="right") - 1, 0, len(sends) - 1), 1]
        cmd = cmd - cmd.mean()
        scale = float(rep @ cmd) / float(cmd @ cmd)                # the amplitudes differ: compare shapes
        err = float(np.square(rep - scale * cmd).mean())
        if err < best_err:
            best, best_err = shift, err
    return 1000 * float(best)


def read(sends: np.ndarray, reports: np.ndarray) -> dict:
    """(t s, deg) arrays to the measures, past the first 3 s."""
    steady = reports[reports[:, 0] > 3.0]
    speed = np.abs(np.diff(steady[:, 1]) / np.diff(steady[:, 0]))
    return {"shift_ms": best_shift_ms(sends, reports), "half_range_deg": float(np.ptp(steady[:, 1]) / 2),
            "speed_p50_deg_s": float(np.median(speed)), "speed_peak_deg_s": float(speed.max())}


def measured(block: dict) -> tuple[np.ndarray, np.ndarray]:
    sends = np.c_[np.array(block["t_cmd_ms"]) / 1000, block["cmd_deg"]]
    reports = np.c_[np.array(block["t_rep_ms"]) / 1000, block["rep_deg"]]
    return sends, reports


def replay(block: dict, dt: float = 0.002, report_age_s: float = motor_model.D1_LOCKED_REPORT_AGE_S, **link_args) -> np.ndarray:
    """The joint at each report's time, deg: the commands at their own times through the planner and lag."""
    sends, reports = measured(block)
    link = ArmLink(1, "cpu", dt, planner=True, seed=3, **(link_args or LOCKED))
    start = torch.zeros(1, 6)
    start[0, 0] = math.radians(reports[0, 1])
    link.reset(None, start)
    steps = int((reports[-1, 0] + 0.5) / dt)
    trace = np.empty(steps)
    every, sent = torch.ones(1, dtype=torch.bool), 0
    target = start.clone()
    for k in range(steps):
        while sent < len(sends) and sends[sent, 0] <= k * dt:
            target[0, 0] = math.radians(sends[sent, 1])
            link.tracker.command(every, target)
            sent += 1
        trace[k] = math.degrees(float(link.drive(dt)[0][0, 0]))
    return np.interp(reports[:, 0] - report_age_s, np.arange(steps) * dt, trace)


def replay_rms(**link_args) -> float:
    """RMS deg against the reports over the blocks without a skip, each block's standing offset removed."""
    errors = []
    for name, block in blocks().items():
        if name != SKIPPED:
            e = replay(block, **link_args) - np.array(block["rep_deg"])
            errors.append(e - e.mean())
    return float(np.sqrt(np.mean(np.square(np.concatenate(errors)))))


def replay_draw(dt: float = 0.002, **link_args) -> dict:
    """The held-out draw, all six joints at once: RMS deg per joint overall and over reports moving past 5 deg/s, with each
    joint's median bias removed and the skip left out."""
    draw = json.loads(DRAW.read_text())["draw"]
    tc, tr = np.array(draw["tc_ms"]) / 1000, np.array(draw["tr_ms"]) / 1000
    commands = np.radians([draw[f"J{j}"]["cmd_deg"] for j in range(1, 7)]).T
    reports = np.array([draw[f"J{j}"]["rep_deg"] for j in range(1, 7)]).T
    link = ArmLink(1, "cpu", dt, planner=True, seed=3, **(link_args or LOCKED))
    start = torch.as_tensor(np.radians(reports[0]), dtype=torch.float32).unsqueeze(0)
    link.reset(None, start)
    steps = int((tr[-1] + 0.3) / dt)
    trace = np.empty((steps, 6))
    every, sent = torch.ones(1, dtype=torch.bool), 0
    target = start.clone()
    for k in range(steps):
        while sent < len(tc) and tc[sent] <= k * dt:
            target[0] = torch.as_tensor(commands[sent], dtype=torch.float32)
            link.tracker.command(every, target)
            sent += 1
        trace[k] = np.degrees(link.drive(dt)[0][0].numpy())
    keep = (tr >= 0.2) & ~((tr > 37.5) & (tr < 37.9))
    every_rms, moving_rms = [], []
    for j in range(6):
        e = (np.interp(tr, np.arange(steps) * dt, trace[:, j]) - reports[:, j])[keep]
        e = e - np.median(e)
        speed = np.abs(np.gradient(reports[:, j], tr))[keep]
        every_rms.append(float(np.sqrt(np.mean(e ** 2))))
        moving_rms.append(float(np.sqrt(np.mean(e[speed > 5] ** 2))))
    return {"all": every_rms, "moving": moving_rms}


def stream(block: dict, dt: float, **link_args) -> tuple[np.ndarray, np.ndarray]:
    """The block's commands, in order, through the locked link at step `dt`: (sends, reports), (t s, deg)."""
    wanted = np.radians(block["cmd_deg"])
    link = ArmLink(1, "cpu", dt, feedback_hz=motor_model.D1_FEEDBACK_HZ, quantum_deg=motor_model.D1_FEEDBACK_QUANTUM_DEG,
                   locked=True, planner=True, seed=3, report_age_s=motor_model.D1_LOCKED_REPORT_AGE_S, **(link_args or LOCKED))
    start = torch.zeros(1, 6)
    start[0, 0] = math.radians(block["rep_deg"][0])
    link.reset(None, start)
    physics = min(dt, 0.005)
    sends, reports, last = [], [], None
    index = 0
    target = start.clone()
    k = 0
    while index < len(wanted):
        target[0, 0] = float(wanted[index])
        due = float(link.clock[0]) >= float(link.next_command[0])
        now = float(link.clock[0])
        link.command(target)
        if due:
            sends.append((now, math.degrees(float(wanted[index]))))
            index += 1
        for _ in range(max(1, round(dt / physics))):
            q, _ = link.drive(physics)
        seen, _ = link.observe(q.clone(), torch.zeros(1, 6))
        stamp = float(link.last_report[0])
        if stamp != last:
            reports.append((stamp, math.degrees(float(seen[0, 0]))))
            last = stamp
        k += 1
    return np.array(sends), np.array(reports)


def first_motion_ms(dt: float = 0.005, step_deg: float = 4.0, **link_args) -> float:
    """From rest, one command after a report: ms to the first report showing more than 0.3 deg (the arm: 88-90)."""
    link = ArmLink(1, "cpu", dt, feedback_hz=motor_model.D1_FEEDBACK_HZ, quantum_deg=motor_model.D1_FEEDBACK_QUANTUM_DEG,
                   locked=True, lock_offset_s=0.021, planner=True, seed=3, report_age_s=motor_model.D1_LOCKED_REPORT_AGE_S,
                   **(link_args or LOCKED))
    link.reset(None, torch.zeros(1, 6))
    target, sent = torch.zeros(1, 6), None
    physics = min(dt, 0.005)
    for k in range(int(1.5 / dt)):
        if k * dt > 0.4:
            target[:, 0] = math.radians(step_deg)
        before, now = float(link.target[0, 0]), float(link.clock[0])
        link.command(target)
        if sent is None and float(link.target[0, 0]) != before:
            sent = now
        for _ in range(max(1, round(dt / physics))):
            q, _ = link.drive(physics)
        seen, _ = link.observe(q.clone(), torch.zeros(1, 6))
        if sent is not None and abs(math.degrees(float(seen[0, 0]))) > 0.3:
            return 1000 * (float(link.last_report[0]) - sent)
    return float("nan")


def mean_read(rows: list[dict]) -> dict:
    return {key: float(np.mean([row[key] for row in rows])) for key in rows[0]}


def main() -> None:
    data = blocks()
    arm = mean_read([read(*measured(b)) for b in data.values()])
    arm_replay = mean_read([read(measured(b)[0], np.c_[measured(b)[1][:, 0], replay(b)]) for b in data.values()])
    print(f"{'':28s} {'shift ms':>8s} {'half deg':>8s} {'speed p50':>9s} {'peak':>6s}")
    show = lambda name, r: print(f"{name:28s} {r['shift_ms']:8.0f} {r['half_range_deg']:8.2f} {r['speed_p50_deg_s']:9.1f} {r['speed_peak_deg_s']:6.1f}")
    show("the arm", arm)
    show("model, commands replayed", arm_replay)
    for dt, name in ((0.005, "model, locked link, 5 ms"), (0.02, "model, locked link, 20 ms")):
        show(name, mean_read([read(*stream(b, dt)) for b in data.values()]))
    print(f"RMS against the reports, blocks without a skip: {replay_rms():.3f} deg")
    print(f"first report of motion: {first_motion_ms():.0f} ms on 5 ms steps, {first_motion_ms(0.02):.0f} on 20 ms (the arm: 88-90)")
    held_out = replay_draw()
    print("held-out draw, J1-6, deg RMS: all", [round(v, 2) for v in held_out["all"]], "moving", [round(v, 2) for v in held_out["moving"]])


if __name__ == "__main__":
    main()
