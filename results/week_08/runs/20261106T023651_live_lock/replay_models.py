#!/usr/bin/env python3
"""Replay the locked commands through simple arm models and compare with every report (F-244, F-245).

`lag` caps the lag's own output rate (F-244's structure, superseded). `series` slews at the cap, then lags (F-245).

Reads only `results/`: the three swing runs' `swing_series.json` and this run's `draw_series.json`.
Parameters come from the J1 swing; the all-joint draw is held out. Run from the repo root.
"""
import json
from pathlib import Path

import numpy as np

W = Path("results/week_08/runs")
DT = 0.001


def lag(tc, u, tr, tau, delay, vmax, x0):
    """First-order lag on the held command, `delay` after the send, speed capped at `vmax`."""
    t = np.arange(tr[0], tr[-1] + DT, DT)
    i = np.searchsorted(tc + delay, t, side="right") - 1
    target = np.where(i >= 0, u[np.maximum(i, 0)], x0)
    x, c = np.empty_like(t), x0
    for n in range(len(t)):
        c += max(-vmax * DT, min(vmax * DT, (target[n] - c) * DT / tau))
        x[n] = c
    return np.interp(tr, t, x)


def series(tc, u, tr, tau, delay, vmax, x0):
    """A slew towards the held command at `vmax`, then a first-order lag on the slew: two stages in series."""
    t = np.arange(tr[0], tr[-1] + DT, DT)
    k = np.searchsorted(tc + delay, t, side="right") - 1
    a = 1.0 - np.exp(-DT / tau)
    x, s, y, goal = np.empty_like(t), x0, x0, x0
    for n in range(len(t)):
        if k[n] >= 0:
            goal = u[k[n]]
        s += min(vmax * DT, max(-vmax * DT, goal - s))
        y += a * (s - y)
        x[n] = y
    return np.interp(tr, t, x)


def line(tc, u, tr, duration, delay, x0):
    """Each command: a straight line from where the arm is to the target over `duration`, no cap."""
    t = np.arange(tr[0], tr[-1] + DT, DT)
    k = np.searchsorted(tc + delay, t, side="right") - 1
    x, c, last, rate, end = np.empty_like(t), x0, -1, 0.0, -1e9
    for n in range(len(t)):
        if k[n] != last and k[n] >= 0:
            last, rate, end = k[n], (u[k[n]] - c) / duration, t[n] + duration
        if t[n] < end:
            c += rate * DT
        x[n] = c
    return np.interp(tr, t, x)


def rms(e):
    return float(np.sqrt((e ** 2).mean()))


def main():
    blocks = []
    for run, only in (("20261106T021112", None), ("20261106T021303", "swing_lock_20ms"), ("20261106T021615", None)):
        for b in json.loads((W / run / "swing_series.json").read_text())["blocks"]:
            if b["lock_ms"] and only in (None, b["name"]):
                c, p = np.array(b["cmd_ms_centideg"], float), np.array(b["report_ms_decideg"], float)
                blocks.append((c[:, 0] / 1e3, c[:, 1] / 100, p[:, 0] / 1e3, p[:, 1] / 10))
    d = json.loads((W / "20261106T023651_live_lock" / "draw_series.json").read_text())
    c, p = np.array(d["cmd_ms_centideg"], float), np.array(d["report_ms_decideg"], float)
    tc, U, tr, Q = c[:, 0] / 1e3, c[:, 1:] / 100, p[:, 0] / 1e3, p[:, 1:] / 10
    keep = np.ones(len(tr), bool)  # the one skip event out, as the swing's blocks with a skip are
    for g in tr[:-1][np.diff(tr) > 0.17]:
        keep &= ~((tr > g - 0.2) & (tr < g + 1.0))
    speed = np.abs(np.gradient(Q, tr, axis=0))
    print(f"draw: {len(tc)} commands, {len(tr)} reports; peak command speed {np.abs(np.diff(U, axis=0) / np.diff(tc)[:, None]).max(0).round(1)}, "
          f"peak report speed {np.abs(np.diff(Q, axis=0) / np.diff(tr)[:, None]).max(0).round(1)} deg/s")
    print("model                  swing J1 | draw J1-6, median bias removed | same, moving over 5 deg/s")
    models = [(f"lag {t * 1e3:.0f} d{dl * 1e3:.0f} v{v}", lambda a, b, r, x, t=t, dl=dl, v=v: lag(a, b, r, t, dl, v, x))
              for t, dl, v in ((.077, .005, 72), (.077, .005, 36), (.063, .02, 36), (.05, .04, 42), (.063, .02, 45), (.05, .04, 50))]
    models += [("line 130 d20", lambda a, b, r, x: line(a, b, r, .13, .02, x))]
    models += [("series 77 d5 v71.6", lambda a, b, r, x: series(a, b, r, .077, .005, 71.6, x))]
    for name, f in models:
        s = rms(np.concatenate([(f(a, b, r, q[0]) - q)[5:] for a, b, r, q in blocks]))
        allj, mov = [], []
        for j in range(6):
            e = f(tc, U[:, j], tr, Q[0, j]) - Q[:, j]
            e -= np.median(e[keep])
            allj.append(rms(e[keep]))
            mov.append(rms(e[keep & (speed[:, j] > 5)]))
        print(f"{name:22s} {s:.3f}   | {' '.join(f'{v:.2f}' for v in allj)} | {' '.join(f'{v:.2f}' for v in mov)}")
    per_joint(tc, U, tr, Q, keep, speed)


def per_joint(tc, U, tr, Q, keep, speed):
    """Series model, delay 5 ms, each joint's URDF cap: the best lag while moving, whole run and each half."""
    caps = np.degrees([1.25, 1.29, 1.23, 1.21, 1.25, 1.25])
    lags = (.03, .04, .05, .055, .06, .065, .07, .075, .08, .09, .1)
    half = np.arange(len(tr)) < len(tr) // 2
    print("series, delay 5 ms: joint, best lag ms, rms there, rms at 75 ms, best lag in each half, moving reports")
    for j in range(6):
        errors = {}
        for lag_s in lags:
            e = series(tc, U[:, j], tr, lag_s, .005, caps[j], Q[0, j]) - Q[:, j]
            errors[lag_s] = e - np.median(e[keep])
        moving = keep & (speed[:, j] > 5)
        best = min(lags, key=lambda v: rms(errors[v][moving]))
        halves = [min(lags, key=lambda v: rms(errors[v][moving & h])) for h in (half, ~half)]
        print(f"J{j + 1} {best * 1e3:.0f} {rms(errors[best][moving]):.2f} {rms(errors[.075][moving]):.2f} "
              f"{[round(v * 1e3) for v in halves]} {int(moving.sum())}")


if __name__ == "__main__":
    main()
