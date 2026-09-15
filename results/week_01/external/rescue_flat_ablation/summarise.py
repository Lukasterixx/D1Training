"""Rebuild the Rescue flat-ablation table and figure from the benchmark JSON copied here.

    python results/week_01/external/rescue_flat_ablation/summarise.py

Writes summary.csv beside this file and ../../figures/rescue_flat_ablation_gait.png.
Needs matplotlib for the figure (the Isaac environment has it); the CSV needs nothing.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
BENCH = HERE / "bench"
FIGURE = HERE.parent.parent / "figures" / "rescue_flat_ablation_gait.png"

# label, arm on the TRAINING robot, 0.5 m/s file, 1.0 m/s file, turn-rate prefix (0.2 / 0.5 rad/s runs)
CONFIGS = [
    ("P2Dingo flat", False, "2026-09-14_flat/p2dflat_v0.5.json", "2026-09-14_flat/p2dflat_v1.0.json", "p2dflat"),
    ("all removed", False, "2026-09-15_flat_ablation/abl_FlatP2D_v0.5.json", "2026-09-15_flat_ablation/abl_FlatP2D_v1.0.json", "allOff"),
    ("no arm", False, "2026-09-15_flat_ablation/abl_FlatNoArm_v0.5.json", "2026-09-15_flat_ablation/abl_FlatNoArm_v1.0.json", "noArm"),
    ("Rescue, seed 42", True, "2026-09-14_flat/flat_arm2000_v0.5.json", "2026-09-14_flat/flat_arm2000_v1.0.json", None),
    ("Rescue, seed 2", True, "2026-09-15_flat_ablation/abl_Rescue_s2_v0.5.json", "2026-09-15_flat_ablation/abl_Rescue_s2_v1.0.json", "rescue_s2"),
    ("no MaiRo DR", True, "2026-09-15_flat_ablation/abl_FlatNoDR_v0.5.json", "2026-09-15_flat_ablation/abl_FlatNoDR_v1.0.json", None),
    ("stock motor", True, "2026-09-15_flat_ablation/abl_FlatNoAct_v0.5.json", "2026-09-15_flat_ablation/abl_FlatNoAct_v1.0.json", None),
    ("8 cm clearance", True, "2026-09-15_flat_ablation/abl_FlatClear08_v0.5.json", "2026-09-15_flat_ablation/abl_FlatClear08_v1.0.json", None),
]


def load(rel: str | None) -> dict | None:
    path = BENCH / rel if rel else None
    return json.loads(path.read_text()) if path and path.exists() else None


def rows() -> list[dict]:
    out = []
    for label, arm, slow, fast, turns in CONFIGS:
        a, b = load(slow), load(fast)
        t02 = load(f"2026-09-15_turn_rates/{turns}_w0.2.json") if turns else None
        t05 = load(f"2026-09-15_turn_rates/{turns}_w0.5.json") if turns else None
        out.append({
            "config": label, "arm_in_training": arm,
            "front_td_minus_neutral_cm_0.5": a["td_flat_front"]["td_minus_neutral_cm"],
            "rear_td_minus_neutral_cm_0.5": a["td_flat_rear"]["td_minus_neutral_cm"],
            "same_side_spacing_cm_1.0": b["td_flat_front"]["gap_cm"],
            "stride_cm_0.5": a["td_flat_front"]["stride_cm"],
            "stance_ms_0.5": a["td_flat_front"]["stance_ms"],
            "forward_tracking_pct_0.5": a["response"]["steady_pct"],
            "forward_tracking_pct_1.0": b["response"]["steady_pct"],
            "turn_tracking_pct_0.2": t02["turn"]["pct"] if t02 else None,
            "turn_tracking_pct_0.5": t05["turn"]["pct"] if t05 else None,
            "turn_tracking_pct_1.0": a["turn"]["pct"],
            "turn_drift_m_s_1.0": a["turn"]["drift"],
            "backward_after_request_pct_1.0": b["response"]["backward_pct"],
            "pitch_wobble_deg_1.0": b["stab_flat"]["wobble_deg"],
        })
    return out


def write_csv(data: list[dict]) -> Path:
    path = HERE / "summary.csv"
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(data[0]))
        writer.writeheader()
        for row in data:
            writer.writerow({k: (f"{v:.4g}" if isinstance(v, float) and not math.isnan(v) else v) for k, v in row.items()})
    return path


def draw(data: list[dict]) -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    surface, ink, ink2, muted, grid, axis = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
    without_arm, with_arm = "#2a78d6", "#eb6834"  # validated categorical slots 1 and 2
    plt.rcParams.update({"font.family": "sans-serif", "font.size": 9, "text.color": ink, "axes.labelcolor": ink2,
                         "xtick.color": muted, "ytick.color": ink2})
    panels = [
        ("rear_td_minus_neutral_cm_0.5", "Rear foot at touchdown vs neutral point, 0.5 m/s",
         "cm (0 = on Raibert's neutral point; + = ahead of it)"),
        ("same_side_spacing_cm_1.0", "Same-side front–rear foot spacing, 1.0 m/s",
         "cm at front touchdown (hips are 38.7 cm apart)"),
    ]
    labels = [row["config"] for row in data]
    fig, axes = plt.subplots(1, 2, figsize=(9.2, 3.6), dpi=200, facecolor=surface, sharey=True)
    for ax, (key, title, xlabel) in zip(axes, panels):
        ax.set_facecolor(surface)
        values = [row[key] for row in data]
        colors = [with_arm if row["arm_in_training"] else without_arm for row in data]
        y = range(len(data))
        ax.barh(y, values, height=0.55, color=colors, zorder=3)
        ax.axvline(0, color=axis, linewidth=1, zorder=4)
        span = max(abs(v) for v in values)
        for yi, v in zip(y, values):
            ax.text(v + (0.02 * span if v >= 0 else -0.02 * span), yi, f"{v:+.0f}" if "neutral" in key else f"{v:.0f}",
                    va="center", ha="left" if v >= 0 else "right", color=ink2, fontsize=8)
        ax.set_title(title, loc="left", fontsize=9.5, fontweight="semibold", color=ink, pad=8)
        ax.set_xlabel(xlabel, fontsize=8, color=muted)
        ax.grid(axis="x", color=grid, linewidth=0.8, zorder=0)
        low = min(values)
        ax.set_xlim(low - 0.18 * span if low < 0 else 0, max(values) + 0.14 * span)
        for side in ("top", "right", "left"):
            ax.spines[side].set_visible(False)
        ax.spines["bottom"].set_color(axis)
        ax.tick_params(axis="y", length=0)
        ax.tick_params(axis="x", colors=muted, labelsize=8)
    axes[0].set_yticks(list(range(len(labels))), labels)
    axes[0].invert_yaxis()
    from matplotlib.patches import Patch
    fig.legend(handles=[Patch(color=without_arm, label="arm absent from the training robot"),
                        Patch(color=with_arm, label="arm welded on during training")],
               loc="lower left", bbox_to_anchor=(0.01, -0.01), ncol=2, frameon=False, fontsize=8.5)
    fig.suptitle("Rescue flat ablation: every policy benchmarked on the welded Go2+D1 (measured motor)",
                 x=0.01, ha="left", fontsize=10.5, fontweight="semibold", color=ink)
    fig.text(0.01, 0.915, "One training seed per configuration except Rescue (two). 2000 iterations from scratch, "
             "4096 environments. Source: Rescue measure_bench.py, 2026-09-14/15.", fontsize=8, color=muted)
    fig.tight_layout(rect=(0, 0.06, 1, 0.93))
    FIGURE.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGURE, facecolor=surface)
    return FIGURE


if __name__ == "__main__":
    data = rows()
    print("wrote", write_csv(data))
    try:
        print("wrote", draw(data))
    except ImportError:
        print("matplotlib not available; skipped the figure")
