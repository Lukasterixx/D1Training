"""Why UniFP's policy reaches the cup and cannot pick it up, and what the roll servo changes.

    python results/week_02/figures/unifp_demos.py

Three panels, each from recorded runs in `results/week_02/runs/`:

  * **The hand's orientation is not commanded.** Every goal in the orientation probe, plotted as
    the jaw axis's angle from horizontal against the goal's height. A grasp needs this near 0 for
    a cup standing on a table, or near 90 for a lever's horizontal bar, and the policy delivers
    whatever the goal happens to produce.
  * **The controlled point rides on a finger.** Steady-state tracking error at one goal against
    how far the jaws are opened, with and without the correction that puts the jaws' own travel
    back into the commanded goal.
  * **What each controller does to the two demos**, 16 placements each.

Numbers are read from the recorded `run.json`/`summary.json`, so the figure cannot drift from the
runs it draws.
"""
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[3]
RUNS = ROOT / "results/week_02/runs"

PURE = "#2a78d6"
SERVO = "#eb6834"
GREY = "#94a3b8"

#: Recorded runs this figure is built from.
PROBE = "20260923T102440_probe_seed1_orientation_probe"
JAW_SWEEP = {17.2: "20260923T111858_cup_unifp_no_object_free_jaw0mm_seed1",
             47.2: "20260923T111919_cup_unifp_no_object_free_jaw15mm_seed1",
             77.2: "20260923T111940_cup_unifp_no_object_free_jaw30mm_seed1"}
JAW_CORRECTED = "20260923T112102_cup_unifp_no_object_free_jaw30mm_seed1"
#: 16 placements per cell. The environment count is a condition, not a detail (F-098): the same
#: placements and seed give different outcomes batched and one at a time, so both are drawn.
OUTCOMES = {("cup", "pure", 16): "20260923T112548_cup_unifp_seed1",
            ("cup", "servo", 16): "20260923T112611_cup_unifp_wrist_seed1",
            ("combiner", "pure", 16): "20260923T112735_combiner_unifp_seed1",
            ("combiner", "servo", 16): "20260923T112832_combiner_unifp_wrist_seed1",
            ("cup", "pure", 1): "20260923T131701_cup_unifp_seed1",
            ("cup", "servo", 1): "20260923T132040_cup_unifp_wrist_seed1",
            ("combiner", "pure", 1): "20260923T131105_combiner_unifp_seed1",
            ("combiner", "servo", 1): "20260923T125405_combiner_unifp_wrist_seed1"}


def load(run, name):
    return json.loads((RUNS / run / name).read_text())


def main(dest):
    figure, axes = plt.subplots(1, 3, figsize=(13.5, 4.1))

    # --- the hand's orientation, over the whole workspace -------------------------------------
    probe = load(PROBE, "orientation_probe.json")["records"]
    height = [record["goal_base_m"][2] + 0.49 for record in probe]
    tilt = [record["jaw_tilt_deg"] for record in probe]
    wobble = [record["jaw_wobble_deg"] for record in probe]
    points = axes[0].scatter(height, tilt, c=wobble, cmap="magma_r", s=22, vmin=0, vmax=60,
                             edgecolor="white", linewidth=0.3)
    axes[0].axhspan(-5, 15, color=PURE, alpha=0.10)
    axes[0].axhspan(75, 95, color=SERVO, alpha=0.10)
    axes[0].text(0.285, 5, "jaws level — a cup on a table", ha="left", va="center",
                 fontsize=8, color=PURE)
    axes[0].text(0.285, 91, "jaws upright — a lever's bar", ha="left", va="center",
                 fontsize=8, color=SERVO)
    axes[0].set_xlabel("goal height above the ground (m)")
    axes[0].set_ylabel("jaw axis, degrees from horizontal")
    axes[0].set_title("UniFP commands no orientation:\n105 held goals, hand as it happens to land",
                      fontsize=10)
    axes[0].set_ylim(-8, 98)
    bar = figure.colorbar(points, ax=axes[0])
    bar.set_label("swing while the goal is still (deg)", fontsize=8)

    # --- the controlled point rides on a finger ------------------------------------------------
    gaps = sorted(JAW_SWEEP)
    errors = [load(JAW_SWEEP[gap], "run.json")["summary"]["jaw_centre_error_arrive_m_median"] * 100
              for gap in gaps]
    corrected = load(JAW_CORRECTED, "run.json")["summary"]["jaw_centre_error_arrive_m_median"] * 100
    axes[1].plot(gaps, errors, "o-", color=PURE, label="as the policy is commanded")
    axes[1].plot([gaps[-1]], [corrected], "D", color=SERVO, markersize=9,
                 label="jaw travel put back into the goal")
    axes[1].annotate(f"{corrected:.2f} cm", (gaps[-1], corrected), textcoords="offset points",
                     xytext=(-12, 12), fontsize=9, color=SERVO, ha="right")
    axes[1].annotate(f"{errors[-1]:.2f} cm", (gaps[-1], errors[-1]), textcoords="offset points",
                     xytext=(-8, 4), fontsize=9, color=PURE, ha="right")
    axes[1].axhline(1.1, color=GREY, linestyle=":", linewidth=1)
    axes[1].text(20, 1.25, "the clearance a 55 mm cup leaves in the jaws", fontsize=8, color=GREY)
    axes[1].set_xlabel("gap between the pads (mm)")
    axes[1].set_ylabel("tracking error at the grasp point (cm)")
    axes[1].set_title("The controlled point is one fingertip,\nand the jaws are not in the "
                      "observation", fontsize=10)
    axes[1].set_ylim(0, 4)
    axes[1].legend(fontsize=8, loc="upper left")

    # --- what it does to the demos -------------------------------------------------------------
    labels = ["cup\npicked up", "combiner\ndoor opened", "combiner\nlatch released"]

    def outcomes(controller, envs):
        cup = load(OUTCOMES[("cup", controller, envs)], "run.json")["summary"]
        box = load(OUTCOMES[("combiner", controller, envs)], "run.json")["summary"]
        return [cup["successes"], box["successes"], box["latch_released"]]

    series = [("UniFP alone, 16 envs", outcomes("pure", 16), PURE, None),
              ("UniFP alone, 1 env", outcomes("pure", 1), PURE, "///"),
              ("+ roll servo, 16 envs", outcomes("servo", 16), SERVO, None),
              ("+ roll servo, 1 env", outcomes("servo", 1), SERVO, "///")]
    width = 0.2
    positions = range(len(labels))
    for index, (label, values, colour, hatch) in enumerate(series):
        offset = (index - 1.5) * width
        axes[2].bar([p + offset for p in positions], values, width, label=label,
                    color=colour, alpha=1.0 if hatch is None else 0.45,
                    hatch=hatch, edgecolor="white", linewidth=0.6)
        for position, value in zip(positions, values):
            axes[2].text(position + offset, value + 0.3, f"{value}", ha="center", fontsize=7,
                         color=colour)
    axes[2].axhline(16, color=GREY, linestyle=":", linewidth=1)
    axes[2].text(2.45, 15.0, "16 placements", fontsize=8, color=GREY, ha="right", va="top")
    axes[2].set_xticks(list(positions))
    axes[2].set_xticklabels(labels, fontsize=9)
    axes[2].set_ylabel("attempts out of 16")
    axes[2].set_ylim(0, 19)
    axes[2].set_title("Standing, object given, no falls in any run.\nThe same placements, batched "
                      "and one at a time (F-098)", fontsize=10)
    axes[2].legend(fontsize=7, loc="upper left", ncol=2)

    for axis in axes:
        axis.spines[["top", "right"]].set_visible(False)
    figure.tight_layout()
    figure.savefig(dest, dpi=150)
    print(f"wrote {dest}")


if __name__ == "__main__":
    main(ROOT / "results/week_02/figures/unifp_demos.png")
