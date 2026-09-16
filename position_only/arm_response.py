"""Fit the D1's commanded-motion response from recorded hardware, with the simulator's own tracker.

The simulator's firmware model (`core.TrapezoidTracker`) needs a dead time, an acceleration, a
deceleration and a replanning rule. None of them can be read off the hardware summaries, because every
recorded timing is quantised by the arm's 111 ms feedback cycle (F-020): F-021's "~127 ms latency" and
F-035's "two cycles to reach cruise" are mostly waiting for the next sample. So this fits the raw
samples at their actual timestamps instead.

- **Steps** (F-033 sweeps, `results/week_01/runs/20260916T0800_d1_hold_sweeps`): six joints, each
  stepped 30 deg out and back on funcode 2 mode 0 -- 12 legs. Dead time is profiled on a grid, with
  acceleration and deceleration refitted at each value, because on a single step the two trade off.
- **Streaming** (F-035 cap sweep): a waypoint every feedback cycle at a per-cycle step cap. This is what
  separates a planner that keeps its speed on a new setpoint from one that restarts from rest.

Both are simulated with `TrapezoidTracker`, so the fitted values are values *for that implementation*.

    python -m position_only.arm_response            # writes results/week_01/figures/d1_arm_response.*

Command times are reconstructed from each outbound leg's recorded latency, whose baseline is the
measured start angle. Return legs are placed one hold after it, not from their own recorded latency:
there the baseline is the *commanded* target, and a joint that settled 0.3 deg away and dithers by
0.1 deg crosses the 0.3 deg detection threshold with no motion at all (J1 and J2).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import torch
from scipy.optimize import least_squares

from .core import TrapezoidTracker

ROOT = Path(__file__).resolve().parent.parent
SWEEP_RUN = ROOT / "results/week_01/runs/20260916T0800_d1_hold_sweeps"
SWEEP_FILES = ("sweep_j0_fc2m0", "sweep_fast_j1", "sweep_fast_j2", "sweep_wrist_j3", "sweep_wrist_j4",
               "sweep_wrist_j5")
DETECT_DEG = 0.3            # the recording's own motion threshold (d1_hardware.analyse_sweep)
DUPLICATE_S = 0.05          # both angle topics deliver each 111 ms sample within a few ms
FEEDBACK_S = 0.111          # F-020
CAP_SWEEP_DEG_PER_CYCLE = {5: 2.5, 8: 4.4, 12: 4.9, 16: 5.3}  # F-035, measured
CRUISE_DEG_S = 70.0         # F-035's cruise on the streamed joint
FIT_DT = 0.0005


def _command_time(samples, joint, baseline, latency):
    """The outbound command time the recording's own latency implies."""
    for t, angles in samples:
        if abs(angles[joint] - baseline) <= DETECT_DEG:
            continue
        candidate = t - latency
        earlier = any(candidate <= s < t and abs(a[joint] - baseline) > DETECT_DEG for s, a in samples)
        if not earlier and candidate > 0.5:
            return candidate
    raise ValueError("no command time consistent with the recorded latency")


def load_steps(run_dir=SWEEP_RUN, files=SWEEP_FILES):
    """Each recorded leg as displacement from its held angle, against time since its command."""
    from motor_model import D1_VELOCITY_LIMIT_RAD_S

    limits = list(D1_VELOCITY_LIMIT_RAD_S.values())
    legs = []
    for name in files:
        record = json.loads((Path(run_dir) / f"{name}.json").read_text())
        joint, samples = record["joint"], record["samples"]
        t_out = _command_time(samples, joint, record["start_deg"], record["out"]["latency_s"])
        hold = round((samples[-1][0] - 1.0) / 2.0)       # 1 s baseline, then two equal holds
        t_back = t_out + hold + 0.003
        kept = []
        for t, angles in samples:
            if not kept or t - kept[-1][0] > DUPLICATE_S:
                kept.append((t, angles[joint]))
        kept = np.asarray(kept)
        for leg, t_cmd, t_end in (("out", t_out, t_back), ("back", t_back, math.inf)):
            before = kept[(kept[:, 0] < t_cmd) & (kept[:, 0] > t_cmd - 0.6), 1]
            window = kept[(kept[:, 0] >= t_cmd - 0.25) & (kept[:, 0] < min(t_end, t_cmd + 1.9))]
            settled = kept[(kept[:, 0] > t_cmd + 1.0) & (kept[:, 0] < min(t_end, t_cmd + 1.95)), 1]
            start, end = float(before.mean()), float(settled.mean())
            legs.append({"file": name, "joint": joint, "leg": leg, "t_cmd_s": float(t_cmd),
                         "distance_rad": math.radians(end - start), "vmax_rad_s": limits[joint],
                         "tau_s": window[:, 0] - t_cmd, "disp_deg": window[:, 1] - start,
                         "held_band_deg": float(np.ptp(before))})
    return legs


def simulate_steps(legs, dead_time, accel, decel, retention=0.0):
    """Every leg as one joint of one tracker, commanded at tau = 0; displacement at each sample, deg."""
    horizon = max(float(leg["tau_s"].max()) for leg in legs) + FIT_DT
    tracker = TrapezoidTracker(1, len(legs), accel, decel, [leg["vmax_rad_s"] for leg in legs],
                               dead_time=dead_time, retention=retention)
    tracker.reset(None, torch.zeros(1, len(legs)))
    tracker.command(torch.tensor([True]), torch.tensor([[leg["distance_rad"] for leg in legs]]))
    steps = int(math.ceil(horizon / FIT_DT))
    trace = torch.empty(steps + 1, len(legs))
    trace[0] = 0.0
    for i in range(steps):
        trace[i + 1] = tracker.step(FIT_DT)[0]
    trace = np.degrees(trace.numpy())
    grid = np.arange(steps + 1) * FIT_DT
    return [np.where(leg["tau_s"] < 0, 0.0, np.interp(leg["tau_s"], grid, trace[:, k]))
            for k, leg in enumerate(legs)]


def step_residuals(legs, dead_time, accel, decel):
    predicted = simulate_steps(legs, dead_time, accel, decel)
    return np.concatenate([p - leg["disp_deg"] for p, leg in zip(predicted, legs)])


def fit_steps(legs, dead_times=(0.0, 0.010, 0.020, 0.030, 0.040, 0.060, 0.090, 0.127)):
    """Profile the dead time: acceleration and deceleration refitted at each fixed value."""
    profile = []
    for dead_time in dead_times:
        result = least_squares(lambda q: step_residuals(legs, dead_time, q[0], q[1]), [13.0, 17.0],
                               bounds=([1.0, 1.0], [400.0, 400.0]), diff_step=1e-3)
        residual = step_residuals(legs, dead_time, *result.x)
        profile.append({"dead_time_s": dead_time, "accel_rad_s2": float(result.x[0]),
                        "decel_rad_s2": float(result.x[1]),
                        "rms_deg": float(np.sqrt(np.mean(residual ** 2)))})
    return profile


def streaming_per_cycle(cap_deg, dead_time, accel, decel, retention, cycles=40):
    """Degrees covered per feedback cycle when each cycle commands the measured angle plus `cap`."""
    vmax = math.radians(CRUISE_DEG_S)
    tracker = TrapezoidTracker(1, 1, accel, decel, vmax, dead_time=dead_time, retention=retention)
    tracker.reset(None, torch.zeros(1, 1))
    per_cycle = int(round(FEEDBACK_S / FIT_DT))
    positions = []
    for _ in range(cycles):
        here = tracker.position.clone()
        positions.append(float(here))
        tracker.command(torch.tensor([True]), here + math.radians(cap_deg))
        for _ in range(per_cycle):
            tracker.step(FIT_DT)
    return float(np.degrees(np.diff(positions)[10:]).mean())


def fit_streaming(dead_time, accel, decel, retentions=(0.0, 0.25, 0.5, 0.75, 1.0)):
    rows = []
    for retention in retentions:
        sim = {cap: streaming_per_cycle(cap, dead_time, accel, decel, retention)
               for cap in CAP_SWEEP_DEG_PER_CYCLE}
        rms = math.sqrt(np.mean([(sim[c] - CAP_SWEEP_DEG_PER_CYCLE[c]) ** 2 for c in sim]))
        rows.append({"retention": retention, "deg_per_cycle": sim, "rms_deg_per_cycle": rms})
    return rows


STEP_TIE_DEG = 0.01


def choose(profile, streaming):
    """The model to simulate, by a rule fixed before looking at which value it picks.

    On a single step, dead time and acceleration trade off: every dead time from 0 to 40 ms fits the
    12 legs within 0.02 deg RMS of the best. Streaming does not trade off the same way. So: among the
    dead times whose step fit is within `STEP_TIE_DEG` of the best, take the one whose best replanning
    rule best reproduces the cap sweep.
    """
    floor = min(row["rms_deg"] for row in profile)
    tied = [row for row in profile if row["rms_deg"] <= floor + STEP_TIE_DEG
            and f"{row['dead_time_s']*1000:.0f}ms" in streaming]
    def streaming_rms(row):
        return min(r["rms_deg_per_cycle"] for r in streaming[f"{row['dead_time_s']*1000:.0f}ms"])
    chosen = dict(min(tied, key=streaming_rms))
    rows = streaming[f"{chosen['dead_time_s']*1000:.0f}ms"]
    replan = min(rows, key=lambda r: r["rms_deg_per_cycle"])
    chosen.update({"retention": replan["retention"], "streaming_rms_deg_per_cycle": replan["rms_deg_per_cycle"],
                   "rule": f"best streaming fit among dead times within {STEP_TIE_DEG} deg of the best step fit",
                   "step_rms_floor_deg": floor, "tied_dead_times_s": [row["dead_time_s"] for row in tied]})
    return chosen


def main(out_dir=ROOT / "results/week_01/figures"):
    legs = load_steps()
    profile = fit_steps(legs)
    streaming = {f"{row['dead_time_s']*1000:.0f}ms": fit_streaming(row["dead_time_s"], row["accel_rad_s2"],
                                                                  row["decel_rad_s2"])
                 for row in profile if row["dead_time_s"] <= 0.040}
    best = choose(profile, streaming)
    residual = simulate_steps(legs, best["dead_time_s"], best["accel_rad_s2"], best["decel_rad_s2"])
    per_leg = [{"joint": leg["joint"], "leg": leg["leg"], "t_cmd_s": round(leg["t_cmd_s"], 4),
                "distance_deg": round(math.degrees(leg["distance_rad"]), 2),
                "rms_deg": float(np.sqrt(np.mean((p - leg["disp_deg"]) ** 2)))}
               for p, leg in zip(residual, legs)]
    result = {"source": str(SWEEP_RUN.relative_to(ROOT)), "legs": len(legs),
              "samples": int(sum(len(leg["tau_s"]) for leg in legs)),
              "dead_time_profile": profile, "chosen": best, "per_leg": per_leg,
              "cap_sweep_measured_deg_per_cycle": CAP_SWEEP_DEG_PER_CYCLE, "streaming": streaming}
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "d1_arm_response.json").write_text(json.dumps(result, indent=2) + "\n")
    _plot(legs, best, profile, streaming, out_dir / "d1_arm_response.png")
    return result


def _plot(legs, best, profile, streaming, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(16, 9))
    grid = fig.add_gridspec(3, 6)
    dense = [dict(leg, tau_s=np.linspace(-0.1, float(leg["tau_s"].max()), 400)) for leg in legs]
    smooth = simulate_steps(dense, best["dead_time_s"], best["accel_rad_s2"], best["decel_rad_s2"])
    for k, leg in enumerate(legs):
        ax = fig.add_subplot(grid[k // 6, k % 6])
        sign = math.copysign(1.0, leg["distance_rad"])
        ax.plot(dense[k]["tau_s"], sign * smooth[k], color="tab:blue", lw=1.2, label="tracker model")
        ax.plot(leg["tau_s"], sign * leg["disp_deg"], "o", ms=3, color="black", label="D1, 9 Hz feedback")
        ax.axvline(0, color="grey", lw=0.6, ls=":")
        ax.set_title(f"J{leg['joint']} {leg['leg']}", fontsize=9)
        ax.set_xlim(-0.15, 0.9)
        if k == 0:
            ax.legend(fontsize=7, loc="lower right")
        if k % 6 == 0:
            ax.set_ylabel("deg toward goal")
        if k >= 6:
            ax.set_xlabel("s since command")
    ax = fig.add_subplot(grid[2, 0:3])
    ax.plot([r["dead_time_s"] * 1000 for r in profile], [r["rms_deg"] for r in profile], "o-")
    ax.axvline(best["dead_time_s"] * 1000, color="tab:green", ls="--", lw=1, label="chosen")
    ax.set_xlabel("dead time (ms), accel/decel refitted at each"); ax.set_ylabel("step RMS residual (deg)")
    ax.set_title("Single steps: 127 ms is ruled out; 0-40 ms fit alike", fontsize=9)
    ax.legend(fontsize=7)
    ax = fig.add_subplot(grid[2, 3:6])
    caps = list(CAP_SWEEP_DEG_PER_CYCLE)
    x = np.arange(len(caps))
    rows = streaming[f"{best['dead_time_s']*1000:.0f}ms"]
    ax.bar(x - 0.3, [CAP_SWEEP_DEG_PER_CYCLE[c] for c in caps], 0.2, color="black", label="D1 measured")
    for off, row, colour in ((-0.1, rows[0], "tab:blue"), (0.1, rows[-1], "tab:orange")):
        ax.bar(x + off, [row["deg_per_cycle"][c] for c in caps], 0.2, color=colour,
               label=f"tracker, retention {row['retention']:g}")
    ax.set_xticks(x, [f"cap {c} deg" for c in caps]); ax.set_ylabel("deg per 111 ms cycle")
    ax.set_title("Streaming a waypoint per cycle: a new setpoint restarts the plan from rest", fontsize=9)
    ax.legend(fontsize=7)
    fig.suptitle(f"D1 commanded-motion model: dead time {best['dead_time_s']*1000:.0f} ms, "
                 f"accel {best['accel_rad_s2']:.1f}, decel {best['decel_rad_s2']:.1f} rad/s^2, "
                 f"replan retention {best['retention']:g}  |  step RMS {best['rms_deg']:.2f} deg over "
                 f"{len(legs)} legs, streaming RMS {best['streaming_rms_deg_per_cycle']:.2f} deg/cycle",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


SIM_RUNS = {  # recorded copies, results/week_01/runs
    "no planner": "20260916T234613_766543Z_arm_steps_seed42",
    "planner, no feedforward": "20260916T234602_388647Z_arm_steps_seed42",
    "planner + feedforward": "20260916T234806_566293Z_arm_steps_seed42",
}


def compare_sim(out_dir=ROOT / "results/week_01/figures", runs=SIM_RUNS):
    """The simulated arm under the task's 10 Hz setpoint stream, beside the D1 in both of its regimes."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fit = json.loads((Path(out_dir) / "d1_arm_response.json").read_text())["chosen"]
    records = {name: json.loads((ROOT / "results/week_01/runs" / run / "arm_steps.json").read_text())
               for name, run in runs.items()}
    summary = {}
    for name, rec in records.items():
        dt = rec["step_dt_s"]
        per_cycle, tracking, lag = [], [], []
        for leg in rec["legs"]:
            c0, c1 = leg["command_steps"]
            q = np.asarray(leg["joint_rad"])
            per_cycle.append(max(abs(q[c0 + i + 5] - q[c0 + i]) for i in range(0, 40, 5)))
            if leg["planned_rad"] is not None:
                p = np.asarray(leg["planned_rad"])[c0:c1]
                seg = q[c0:c1]
                tracking.append(float(np.sqrt(np.mean((seg - p) ** 2))))
                lag.append(min(range(6), key=lambda k: np.mean((seg[k:] - p[:len(p) - k]) ** 2)) * dt)
        summary[name] = {"rad_s_per_command_cycle": float(np.mean(per_cycle)) / 0.1,
                         "drive_vs_plan_rms_deg": None if not tracking else math.degrees(float(np.mean(tracking))),
                         "drive_lag_s": None if not lag else float(np.mean(lag))}
    hardware = {"D1, one message per step": math.radians(np.mean([7.8, 8.3])) / FEEDBACK_S,
                "D1, a waypoint every cycle (cap 16)": math.radians(CAP_SWEEP_DEG_PER_CYCLE[16]) / FEEDBACK_S}

    fig, axes = plt.subplots(1, 3, figsize=(16, 4.8))
    ax = axes[0]
    joint = 1  # Joint2, the shoulder pitch that carries the arm's weight
    single = simulate_steps([{"tau_s": np.linspace(0, 1.2, 600), "distance_rad": math.radians(30),
                              "vmax_rad_s": 1.29}],   # Joint2's measured ceiling
                            fit["dead_time_s"], fit["accel_rad_s2"], fit["decel_rad_s2"])[0]
    ax.plot(np.linspace(0, 1.2, 600), single, color="black", lw=2, label="D1 fit, one message")
    colours = {"no planner": "tab:orange", "planner, no feedforward": "tab:red", "planner + feedforward": "tab:blue"}
    for name, rec in records.items():
        leg, dt = rec["legs"][joint], rec["step_dt_s"]
        c0 = leg["command_steps"][0]
        q = np.degrees(np.asarray(leg["joint_rad"][c0 - 5:c0 + 60]))
        ax.plot((np.arange(len(q)) - 5) * dt, q, color=colours[name], label=f"sim, {name}")
        if name == "planner + feedforward":
            p = np.degrees(np.asarray(leg["planned_rad"][c0 - 5:c0 + 60]))
            ax.plot((np.arange(len(p)) - 5) * dt, p, color=colours[name], ls=":", lw=1, label="planned target")
    ax.set_xlabel("s since the setpoint changed"); ax.set_ylabel("Joint2 (deg)")
    ax.set_title("30 deg step; the sim streams the setpoint at 10 Hz", fontsize=9)
    ax.set_xlim(-0.1, 1.2); ax.legend(fontsize=7, loc="lower right")

    ax = axes[1]
    labels = list(hardware) + [f"sim, {n}" for n in summary]
    values = list(hardware.values()) + [s["rad_s_per_command_cycle"] for s in summary.values()]
    ax.barh(range(len(values)), values, color=["black", "dimgrey", "tab:orange", "tab:red", "tab:blue"])
    ax.set_yticks(range(len(values)), labels, fontsize=8); ax.invert_yaxis()
    for i, v in enumerate(values):
        ax.text(v + 0.02, i, f"{v:.2f}", va="center", fontsize=8)
    ax.set_xlabel("fastest joint speed per command cycle (rad/s)")
    ax.set_title("A streamed D1 moves at ~0.8 rad/s; the old model gave 1.27", fontsize=9)

    ax = axes[2]
    for name in ("planner, no feedforward", "planner + feedforward"):
        leg, dt = records[name]["legs"][joint], records[name]["step_dt_s"]
        c0, c1 = leg["command_steps"]
        e = np.degrees(np.asarray(leg["joint_rad"][c0:c1]) - np.asarray(leg["planned_rad"][c0:c1]))
        ax.plot(np.arange(len(e)) * dt, e, color=colours[name],
                label=f"{name}: RMS {summary[name]['drive_vs_plan_rms_deg']:.2f} deg, lag "
                      f"{summary[name]['drive_lag_s']*1000:.0f} ms")
    ax.axhline(0, color="grey", lw=0.6)
    ax.set_xlabel("s since the setpoint changed"); ax.set_ylabel("simulated joint minus plan (deg)")
    ax.set_title("Without feedforward the drive's damping trails the plan", fontsize=9)
    ax.set_xlim(0, 1.2); ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(Path(out_dir) / "d1_arm_sim_vs_hardware.png", dpi=110)
    plt.close(fig)
    result = {"runs": runs, "sim": summary, "hardware_rad_s_per_cycle": hardware}
    (Path(out_dir) / "d1_arm_sim_vs_hardware.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


if __name__ == "__main__":
    import sys

    if sys.argv[1:] == ["--compare-sim"]:
        print(json.dumps(compare_sim(), indent=2))
        raise SystemExit
    out = main()
    best = out["chosen"]
    print(f"chosen: dead time {best['dead_time_s']*1000:.0f} ms, accel {best['accel_rad_s2']:.2f}, "
          f"decel {best['decel_rad_s2']:.2f}, retention {best['retention']:g}; {best['rule']}")
    print(f"legs {out['legs']}, samples {out['samples']}")
    for row in out["dead_time_profile"]:
        print(f"  dead time {row['dead_time_s']*1000:5.0f} ms  accel {row['accel_rad_s2']:6.1f}  "
              f"decel {row['decel_rad_s2']:6.1f}  RMS {row['rms_deg']:.3f} deg")
    for key, rows in out["streaming"].items():
        print(f"  streaming at dead time {key}: " + "  ".join(
            f"r={r['retention']:g}:{r['rms_deg_per_cycle']:.2f}" for r in rows))
