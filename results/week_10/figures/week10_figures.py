"""week10_figures.py: the week 10 result figures, rebuilt from the recorded runs under results/ only.

Run from the repo root:  python3 results/week_10/figures/week10_figures.py
(system python3 with matplotlib; no simulator, no GPU). Writes the PNGs beside this script and prints every number it
plots, with the run ids it read.

Figures (simulation only; nothing here was run on the real robot):
  fig1_f303_insertion_retrain.png    F-303: ki7's test, ki6_force_8200 against the three selected ki7 seeds
  fig2_f294_f295_mechanisms.png      F-294 / F-295: S4's arms on T1's held mechanisms, and the bleed rerun of (i)
  fig3_f306_weak_legs.png            F-306: ki7_s62_9200 on legs that answer 0.17 of each target
  fig4_f302_f305_lock_estimate.png   F-302 / F-305: seats against the size of a deliberate lock-estimate error

The scoring copies the analysis scripts that made the findings (logs/unifp_train/s1tools/, gitignored):
  - insertion (ki7_compare.py, lockerr_compare.py): a seat counts only if success and max_tilt_deg <= 15; damaged and
    false seats are the episode flags; each run name's latest run is read;
  - mechanisms (s4_compare.py): opened within the budget = opened and not over_budget; over budget = over_budget;
    the selected checkpoints are s4_select.py's, as the week 10 log records them.
Every plotted number is checked against the finding it illustrates. On a mismatch the figure is not written and the
script exits 1 after the others, naming both numbers.
"""
import glob
import math
import json
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import patheffects  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import FancyBboxPatch, Patch, Rectangle  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "../../.."))
RUNS = os.path.join(ROOT, "results", "week_*", "runs")
TILT = 15.0

# ---------------------------------------------------------------- the palette (dataviz reference instance, light mode)
SURFACE = "#fcfcfb"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID, AXIS, TRACK = "#e1e0d9", "#c3c2b7", "#f0efec"
REST = "#e4e3dd"                                       # the unfilled remainder of a whole (fig 2's "not opened")
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"   # categorical slots 1-3, in that order
GRAY = MUTED                                           # the de-emphasised series (fig 4's ki6)

DPI = 100
FIG_W, FIG_H = 16.0, 9.0                               # 1600 x 900 px
PX = 72.0 / DPI                                        # one pixel in points

plt.rcParams.update({
    "font.family": ["Lato", "Liberation Sans", "DejaVu Sans"],
    "font.size": 14,
    "text.color": INK,
    "axes.edgecolor": AXIS,
    "axes.labelcolor": INK2,
    "axes.linewidth": PX,
    "xtick.color": INK2,
    "ytick.color": INK2,
    "xtick.labelsize": 14,
    "ytick.labelsize": 14,
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
})

PROBLEMS = []          # mismatches against the findings, reported at the end
USED = {}              # figure -> run ids read


# ---------------------------------------------------------------- reading the recorded runs
def run_dir(name, mode, need):
    """The latest recorded run of this name (the analysis scripts' `sorted(glob(...))[-1]`), across results/week_*/runs."""
    ds = [d for d in glob.glob(os.path.join(RUNS, f"*_{mode}_seed*_{name}")) if os.path.exists(os.path.join(d, need))]
    if not ds:
        raise SystemExit(f"no recorded run named {name} with {need} under results/")
    d = sorted(ds, key=os.path.basename)[-1]
    rid = os.path.basename(d)
    assert rid.split("_seed", 1)[1].split("_", 1)[1] == name, (rid, name)
    return d, rid


def insert_eps(name, fig):
    d, rid = run_dir(name, "leg_eval", "insert_eval_episodes.json")
    USED.setdefault(fig, []).append(rid)
    j = json.load(open(os.path.join(d, "insert_eval_episodes.json")))
    return j["episodes"] if isinstance(j, dict) else j


def mech_eps(name, fig):
    d, rid = run_dir(name, "mech_eval", "mech_eval_episodes.json")
    USED.setdefault(fig, []).append(rid)
    return json.load(open(os.path.join(d, "mech_eval_episodes.json")))["episodes"]


def seat(e):
    """ki7_compare.ok / lockerr_compare.ok: a seat inside the 15 deg tilt stop."""
    return bool(e["success"] and e["max_tilt_deg"] <= TILT)


def in_budget(e):
    """s4_compare.ok: opened within the 80 N budget."""
    return bool(e["opened"] and not e["over_budget"])


def mcnemar(b, c):
    """The analysis scripts' exact McNemar, two-sided, to 3 significant figures."""
    n = b + c
    return 1.0 if n == 0 else float(f"{min(1.0, 2 * sum(math.comb(n, i) for i in range(min(b, c) + 1)) / 2 ** n):.3g}")


def insert_summary(ee):
    return {"seated": sum(map(seat, ee)), "damaged": sum(bool(e["damaged"]) for e in ee),
            "false_seat": sum(bool(e["false_seat"]) for e in ee), "fell": sum(bool(e["fell"]) for e in ee),
            "lost": sum(bool(e["lost"]) for e in ee), "of": len(ee)}


def mech_summary(ee):
    return {"in_budget": sum(map(in_budget, ee)), "over_budget": sum(bool(e["over_budget"]) for e in ee),
            "opened_over": sum(bool(e["opened"] and e["over_budget"]) for e in ee),
            "opened": sum(bool(e["opened"]) for e in ee), "tilt": sum(e["tilt_step"] is not None for e in ee),
            "fell": sum(e["fell_step"] is not None for e in ee), "of": len(ee)}


def expect(fig, what, got, want):
    if got != want:
        PROBLEMS.append(f"{fig}: {what}: computed {got}, the finding says {want}")


def as_quoted(p, quoted):
    """A p value rounded to the significant figures the finding quotes it with (0.004 -> 1, 0.018 -> 2)."""
    digits = len(f"{quoted:e}".split("e")[0].replace(".", "").rstrip("0"))
    return float(f"{p:.{digits}g}")


# ---------------------------------------------------------------- drawing helpers
def fmt(n):
    return f"{n:,}"


def pfmt(p):
    """A p value as the findings write it: two significant figures, or one with an exponent below 0.001."""
    if p >= 0.001:
        return f"{p:.2g}"
    m, e = f"{p:.0e}".split("e")
    return f"{m}e{int(e)}"


def footnote(fig, lines, y=0.022):
    for i, t in enumerate(reversed(lines)):
        fig.text(0.03, y + i * 0.03, t, fontsize=12.5, color=INK2, va="bottom", ha="left")


def px_scale(ax):
    """Data units per pixel along x and y for an axes placed with fig.add_axes and fixed limits."""
    pos = ax.get_position()
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    return abs(x1 - x0) / (pos.width * FIG_W * DPI), abs(y1 - y0) / (pos.height * FIG_H * DPI)


def hbar(ax, y, left, width, height_px, color, round_end=True, zorder=3):
    """A horizontal bar: square at its baseline, a 4 px rounded data end."""
    if width <= 0:
        return
    sx, sy = px_scale(ax)
    h = height_px * sy
    r = 4 * sx
    if not round_end or width <= 2 * r:
        ax.add_patch(Rectangle((left, y - h / 2), width, h, fc=color, ec="none", lw=0, zorder=zorder))
        return
    ax.add_patch(FancyBboxPatch((left, y - h / 2), width, h, boxstyle=f"round,pad=0,rounding_size={r}",
                                mutation_aspect=sy / sx, fc=color, ec="none", lw=0, zorder=zorder))
    ax.add_patch(Rectangle((left, y - h / 2), width - r, h, fc=color, ec="none", lw=0, zorder=zorder))


def style_value_axis(ax, ticks, label):
    ax.set_xticks(ticks)
    ax.set_xticklabels([fmt(t) for t in ticks])
    ax.tick_params(axis="x", length=0, pad=8)
    ax.tick_params(axis="y", length=0)
    for t in ticks:
        ax.axvline(t, color=GRID, lw=PX, zorder=0)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    ax.spines["bottom"].set_visible(False)
    ax.axvline(0, color=AXIS, lw=PX, zorder=4)
    ax.set_xlabel(label, fontsize=14, color=INK2, labelpad=10)


def header(fig, lines, sub, top=0.965):
    """Title lines (the finding, in plain words) and a secondary subtitle, left-aligned."""
    y = top
    for i, t in enumerate(lines):
        fig.text(0.03, y, t, fontsize=22 if i == 0 else 18, weight="bold" if i == 0 else "normal",
                 color=INK, va="top", ha="left")
        y -= 0.052 if i == 0 else 0.044
    for t in sub:
        fig.text(0.03, y - 0.004, t, fontsize=13.5, color=INK2, va="top", ha="left")
        y -= 0.034
    return y


def legend_row(fig, items, y, x=0.03):
    handles = [Patch(fc=c, ec="none", label=l) if kind == "patch" else
               Line2D([0], [0], color=c, lw=2 * PX, marker="o", ms=10 * PX * 1.0 + 4, mec=SURFACE, mew=2 * PX, label=l)
               for kind, c, l in items]
    fig.legend(handles=handles, loc="upper left", bbox_to_anchor=(x, y), ncol=len(items), frameon=False,
               fontsize=14, handlelength=1.2, handleheight=1.0, columnspacing=2.0, borderaxespad=0)


def save(fig, name):
    path = os.path.join(HERE, name)
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    print(f"  wrote {os.path.relpath(path, ROOT)}")


# ================================================================ figure 1: F-303
KI7_STEMS = ["ki6_force_8200", "ki7_s61_9400", "ki7_s62_9200", "ki7_s63_9699"]   # ki7_s6X_selected.json (week 10 log)
F303 = {  # findings.md F-303's table: (success of 192, damaged, false seats)
    "strict_legs": {"ki6_force_8200": (114, 26, 52), "ki7_s61_9400": (110, 25, 41), "ki7_s62_9200": (140, 4, 48),
                    "ki7_s63_9699": (130, 18, 44)},
    "default_true": {"ki6_force_8200": (136, 12, 43), "ki7_s61_9400": (136, 1, 55), "ki7_s62_9200": (115, 1, 77),
                     "ki7_s63_9699": (132, 18, 42)},
}


def fig1():
    F = "fig1"
    E = {c: {s: {f: insert_eps(f"ki7t_{s}_{c}_f{f}", F) for f in (1, 2)} for s in KI7_STEMS} for c in F303}
    R, P = {}, {}
    for c in F303:
        for s in KI7_STEMS:
            R[c, s] = insert_summary(E[c][s][1] + E[c][s][2])
            R[c, s]["f1"], R[c, s]["f2"] = (sum(map(seat, E[c][s][f])) for f in (1, 2))
            expect(F, f"{s} {c} (seated, damaged, false seats)",
                   (R[c, s]["seated"], R[c, s]["damaged"], R[c, s]["false_seat"]), F303[c][s])
            if s != "ki6_force_8200":
                b = cc = 0
                for f in (1, 2):
                    A, B = E[c][s][f], E[c]["ki6_force_8200"][f]
                    assert len(A) == len(B) and all(x["lock"] == y["lock"] for x, y in zip(A, B))
                    b += sum(seat(x) and not seat(y) for x, y in zip(A, B))
                    cc += sum(seat(y) and not seat(x) for x, y in zip(A, B))
                P[c, s] = (b, cc, mcnemar(b, cc))
    k7 = KI7_STEMS[1:]
    pooled = (sum(R["strict_legs", s]["seated"] for s in k7), sum(R["strict_legs", s]["damaged"] for s in k7))
    ki6x3 = (3 * R["strict_legs", "ki6_force_8200"]["seated"], 3 * R["strict_legs", "ki6_force_8200"]["damaged"])
    expect(F, "three ki7 seeds pooled, strict_legs (seated, damaged)", pooled, (380, 47))
    expect(F, "3 x ki6, strict_legs (seated, damaged)", ki6x3, (342, 78))
    b, c, p = P["strict_legs", "ki7_s62_9200"]
    expect(F, "ki7_s62_9200 vs ki6 paired, strict_legs (only ki7, only ki6, p)", (b, c, as_quoted(p, 0.004)),
           (51, 25, 0.004))

    print("\nFigure 1 (F-303): seated inside the tilt stop / damaged / false seats, of 192 (f1 + f2)")
    for c in F303:
        for s in KI7_STEMS:
            r = R[c, s]
            pr = f"  vs ki6 {P[c, s][0]} / {P[c, s][1]}, p {pfmt(P[c, s][2])}" if (c, s) in P else ""
            print(f"  {c:12s} {s:15s} seated {r['seated']:3d} (f1 {r['f1']}, f2 {r['f2']})  damaged {r['damaged']:2d}  "
                  f"false seats {r['false_seat']:2d}  of {r['of']}{pr}")
    print(f"  pooled ki7 on strict_legs: seated {pooled[0]}, damaged {pooled[1]}; 3 x ki6: {ki6x3[0]}, {ki6x3[1]}")
    if any(p.startswith(F) for p in PROBLEMS):
        print("  NOT WRITTEN: a number does not match F-303")
        return

    fig = plt.figure(figsize=(FIG_W, FIG_H), dpi=DPI)
    y = header(fig, [
        "Retrained for the dog's conditions, the key insertion breaks fewer keys and seats more (three seeds pooled)",
        "The dog's pick, ki7_s62_9200, seats 140 of 192 with 4 keys damaged; ki6_force_8200 seats 114 with 26",
    ], [
        "Simulation only. ki7's 96 test locks at each of 1 and 2 N·m leg friction, pooled: 192 episodes per bar. "
        "A seat counts only inside the 15° tilt stop.",
        f"Rule stated before training: the three ki7 seeds pooled seat {pooled[0]} and damage {pooled[1]} against "
        f"3 × ki6's {ki6x3[0]} and {ki6x3[1]}, so ki7 replaces ki6.",
    ])
    legend_row(fig, [("patch", BLUE, "Seated inside the tilt stop"),
                     ("patch", ORANGE, "False seat (layer takes a jammed key for seated)"),
                     ("patch", AQUA, "Damaged key")], y - 0.012)
    labels = {"ki6_force_8200": ("ki6_force_8200", "before the retrain"),
              "ki7_s61_9400": ("ki7_s61_9400", "seed 61"),
              "ki7_s62_9200": ("ki7_s62_9200", "seed 62, the dog's pick"),
              "ki7_s63_9699": ("ki7_s63_9699", "seed 63")}
    panels = [("strict_legs", "The dog's conditions: strict keyway, leg-odometry base"),
              ("default_true", "Default contact, true base (where ki6 was trained)")]
    bottom, height = 0.175, 0.50
    for i, (c, ptitle) in enumerate(panels):
        ax = fig.add_axes([0.19 + i * 0.41, bottom, 0.36, height])
        ax.set_xlim(0, 192 * 1.12)
        ax.set_ylim(len(KI7_STEMS) - 0.5, -0.5)
        style_value_axis(ax, [0, 48, 96, 144, 192], "episodes, of 192")
        ax.set_yticks([])
        ax.set_title(ptitle, loc="left", fontsize=15.5, weight="bold", color=INK, pad=14)
        sx, sy = px_scale(ax)
        step = 26 * sy
        for k, s in enumerate(KI7_STEMS):
            r = R[c, s]
            for j, (key, col) in enumerate((("seated", BLUE), ("false_seat", ORANGE), ("damaged", AQUA))):
                yy = k + (j - 1) * step
                hbar(ax, yy, 0, r[key], 22, col)
                ax.text(r[key] + 6 * sx, yy, fmt(r[key]), va="center", ha="left", fontsize=14,
                        color=INK, weight="bold" if key == "seated" else "normal")
            if i == 0:
                name, note = labels[s]
                bold = s in ("ki7_s62_9200", "ki6_force_8200")
                ax.text(-0.04, k - 0.1, name, transform=ax.get_yaxis_transform(), ha="right", va="center",
                        fontsize=15, weight="bold" if bold else "normal", color=INK)
                ax.text(-0.04, k + 0.17, note, transform=ax.get_yaxis_transform(), ha="right", va="center",
                        fontsize=13, color=INK2)
            if k:
                ax.axhline(k - 0.5, color=GRID, lw=PX, zorder=0)
    p = {s: P["strict_legs", s] for s in k7}
    q = P["default_true", "ki7_s62_9200"]
    d = R["default_true", "ki6_force_8200"]["seated"] - R["default_true", "ki7_s62_9200"]["seated"]
    footnote(fig, [
        "Seats paired with ki6 on the same locks (only ki7 / only ki6, exact McNemar), the dog's conditions: "
        + " · ".join(f"seed {s[5:7]} {p[s][0]} / {p[s][1]}, p {pfmt(p[s][2])}" for s in k7) + ".",
        f"On the default contact with the true base the pick seats {d} fewer than ki6 ({q[0]} / {q[1]}, p {pfmt(q[2])}). "
        "Seed 61 gains nothing on the dog's conditions; the false seats stay.",
    ])
    save(fig, "fig1_f303_insertion_retrain.png")


# ================================================================ figure 2: F-294 / F-295
S4_SEEDS = (81, 82, 83)
S4_LAW = {81: 68400, 82: 68400, 83: 68000}   # s4_law_s8X_selected.json, the week 10 log "S4 seed 8X, arm (i)"
S4_OUT = {81: 68200, 82: 68599, 83: 68200}   # s4_out_s8X_selected.json, the week 10 log "S4 seed 8X, arm (iii)"


def s4_runs(arm, seed):
    """s4_compare.runs_for's names, plus F-295's bleed rerun of (i)."""
    names = {"i": f"s4_law_s{seed}_{S4_LAW.get(seed)}", "i+bleed": f"s4_law_s{seed}_{S4_LAW.get(seed)}_bleed1",
             "iii": f"s4_out_s{seed}_{S4_OUT.get(seed)}", "iii+law": f"s4_out_s{seed}_{S4_OUT.get(seed)}_law",
             "ii": "s4_66600_law", "iv": "s4_66600_goal"}
    return [f"{names[arm]}_test_f{f}" for f in (1, 2)]


def fig2():
    F = "fig2"
    arms = [("i", "(i) Layer in training", S4_SEEDS),
            ("i+bleed", "(i) + integral bleed τ = 1 s at evaluation, no retraining (F-295)", S4_SEEDS),
            ("iii+law", "(iii) + layer zero-shot: the outcome-trained policy given the layer", S4_SEEDS),
            ("iii", "(iii) Goal only: outcome-trained, no layer", S4_SEEDS),
            ("ii", "(ii) 66600 + layer zero-shot (untrained on mechanisms)", (None,)),
            ("iv", "(iv) Position script: 66600 given the sliding reference as its goal", (None,))]
    R, pooled = {}, {}
    for a, _, seeds in arms:
        allp = []
        for s in seeds:
            ee = [e for n in s4_runs(a, s) for e in mech_eps(n, F)]
            R[a, s] = mech_summary(ee)
            allp += ee
        pooled[a] = mech_summary(allp)
    # F-294's table and the week 10 log's per-seed rows; F-295's bleed numbers.
    expect(F, "(i) within budget per seed", tuple(R["i", s]["in_budget"] for s in S4_SEEDS), (870, 783, 615))
    expect(F, "(i) pooled within budget / of", (pooled["i"]["in_budget"], pooled["i"]["of"]), (2268, 3072))
    expect(F, "(i) pooled over budget", pooled["i"]["over_budget"], 176)
    expect(F, "(iii)+layer pooled within budget", pooled["iii+law"]["in_budget"], 1049)
    expect(F, "(iii) pooled within budget", pooled["iii"]["in_budget"], 563)
    expect(F, "(ii) within budget / of", (pooled["ii"]["in_budget"], pooled["ii"]["of"]), (193, 1024))
    expect(F, "(iv) within budget / of", (pooled["iv"]["in_budget"], pooled["iv"]["of"]), (19, 1024))
    expect(F, "over budget, the other arms", tuple(pooled[a]["over_budget"] for a in ("iii+law", "iii", "ii", "iv")),
           (0, 0, 0, 0))
    expect(F, "(i)+bleed pooled over budget", pooled["i+bleed"]["over_budget"], 23)
    expect(F, "(i)+bleed pooled within budget", pooled["i+bleed"]["in_budget"], 2412)
    expect(F, "opened, (i)+bleed against (i)", (pooled["i+bleed"]["opened"], pooled["i"]["opened"]), (2435, 2444))
    expect(F, "tilt stops, (i) then (i)+bleed", (pooled["i"]["tilt"], pooled["i+bleed"]["tilt"]), (12, 15))
    expect(F, "falls, (i) then (i)+bleed", (pooled["i"]["fell"], pooled["i+bleed"]["fell"]), (0, 0))
    for a, _, seeds in arms:   # every over-budget episode opened, so the stacked segments add up
        for s in seeds:
            expect(F, f"{a} seed {s}: over budget without opening", R[a, s]["over_budget"] - R[a, s]["opened_over"], 0)

    print("\nFigure 2 (F-294 / F-295): opened within the 80 N budget / opened but over budget / tilt stops, of 1,024 a seed")
    for a, _, seeds in arms:
        for s in seeds:
            r = R[a, s]
            print(f"  ({a:7s}) seed {s or '-':>3}  within {r['in_budget']:4d}  over {r['over_budget']:3d}  "
                  f"opened {r['opened']:4d}  tilt {r['tilt']:3d}  fell {r['fell']}  of {r['of']}")
        q = pooled[a]
        print(f"  ({a:7s}) pooled    within {q['in_budget']:4d}  over {q['over_budget']:3d}  opened {q['opened']:4d}  "
              f"tilt {q['tilt']:3d}  of {q['of']}")
    if any(p.startswith(F) for p in PROBLEMS):
        print("  NOT WRITTEN: a number does not match F-294 / F-295")
        return

    fig = plt.figure(figsize=(FIG_W, FIG_H), dpi=DPI)
    q_i, q_b = pooled["i"], pooled["i+bleed"]
    y = header(fig, [
        "With the task layer's force law in training, the policy opens the most of T1's held mechanisms within 80 N",
        f"{fmt(q_i['in_budget'])} of {fmt(q_i['of'])} test episodes over three seeds; the next best arm "
        f"{fmt(pooled['iii+law']['in_budget'])}. A bleed added at evaluation cuts its {q_i['over_budget']} violations "
        f"to {q_b['over_budget']}",
    ], [
        "Simulation only, through the D1 link on 66600's legs at 1 and 2 N·m. Held-out test set: 1,024 episodes per seed "
        "and arm (16 placements × 4 kinds × 8 levels, 10–80 N, × 2 frictions).",
        "T1: four invented mechanisms (button, door, drawer, latch). The layer: F-106's PI law v1, capped at 80 N. "
        "Over budget: the drive on the handle above 80 N for 0.1 s.",
    ])
    legend_row(fig, [("patch", BLUE, "Opened within the 80 N budget"),
                     ("patch", ORANGE, "Opened, over budget"),
                     ("patch", REST, "Not opened")], y - 0.012)
    # rows: a header line per arm (with its pooled totals), then a bar per seed
    rows, yy = [], 0.0
    for a, title, seeds in arms:
        rows.append(("h", a, title, yy))
        yy += 1.05
        for s in seeds:
            rows.append(("b", a, s, yy))
            yy += 1.0
        yy += 0.35
    ax = fig.add_axes([0.105, 0.125, 0.70, 0.60])
    ax.set_xlim(0, 1024)
    ax.set_ylim(yy - 0.75, -0.6)
    style_value_axis(ax, [0, 256, 512, 768, 1024], "test episodes, of 1,024 per seed")
    ax.set_yticks([])
    sx, sy = px_scale(ax)
    gap = 2 * sx
    cw, co = 1.085, 1.175          # the right-hand columns (axes fraction): within budget, over budget
    for x, t in ((cw, "within\nbudget"), (co, "over\nbudget")):
        ax.text(x, -0.25, t, transform=ax.get_yaxis_transform(), ha="right", va="bottom", fontsize=13,
                color=INK2, linespacing=1.0)
    mask = dict(fc=SURFACE, ec="none", pad=2.0)
    for kind, a, x, y0 in rows:
        if kind == "h":
            q = pooled[a]
            tot = (f"  —  {fmt(q['in_budget'])} of {fmt(q['of'])} within budget"
                   + (f", {q['over_budget']} over" if q["over_budget"] else ""))
            ax.text(-0.075, y0, x + tot, transform=ax.get_yaxis_transform(), ha="left", va="center",
                    fontsize=14.5, color=INK, weight="bold", zorder=6, bbox=mask)
            continue
        r = R[a, x]
        hbar(ax, y0, 0, 1024, 18, REST)
        w1, w2 = r["in_budget"], r["opened_over"]
        hbar(ax, y0, 0, w1, 18, BLUE, round_end=(w2 == 0))
        if w2:
            hbar(ax, y0, w1 + gap, w2 - gap, 18, ORANGE)
        ax.text(-0.012, y0, f"seed {x}" if x else "one run", transform=ax.get_yaxis_transform(), ha="right",
                va="center", fontsize=13.5, color=INK2)
        ax.text(cw, y0, fmt(r["in_budget"]), transform=ax.get_yaxis_transform(), ha="right", va="center",
                fontsize=14, color=INK, weight="bold")
        ax.text(co, y0, fmt(r["over_budget"]), transform=ax.get_yaxis_transform(), ha="right", va="center",
                fontsize=14, color=INK if r["over_budget"] else MUTED)
    footnote(fig, [
        f"Selected on validation before the test set was read: (i) {', '.join(str(S4_LAW[s]) for s in S4_SEEDS)}; "
        f"(iii) {', '.join(str(S4_OUT[s]) for s in S4_SEEDS)} (seeds 81, 82, 83). With the bleed: opened "
        f"{fmt(q_i['opened'])} → {fmt(q_b['opened'])}, tilt stops {q_i['tilt']} → {q_b['tilt']}, no falls.",
    ])
    save(fig, "fig2_f294_f295_mechanisms.png")


# ================================================================ figure 3: F-306
def fig3():
    F = "fig3"
    names = {"base": "lockerr_ki7_s62_9200_base", "weak": "ins017_ki7_s62_9200"}
    E = {k: {f: insert_eps(f"{n}_f{f}", F) for f in (1, 2)} for k, n in names.items()}
    R, P = {}, {}
    for f in (1, 2):
        A, B = E["base"][f], E["weak"][f]   # paired by lock as lockerr_compare pairs: key row and direction
        assert len(A) == len(B) and all(x["key_row"] == y["key_row"] and x["direction_deg"] == y["direction_deg"]
                                        for x, y in zip(A, B))
        for k in names:
            R[k, f] = insert_summary(E[k][f])
        b = sum(seat(x) and not seat(y) for x, y in zip(A, B))
        c = sum(seat(y) and not seat(x) for x, y in zip(A, B))
        P[f] = (b, c, mcnemar(b, c))
    P["both"] = (P[1][0] + P[2][0], P[1][1] + P[2][1], mcnemar(P[1][0] + P[2][0], P[1][1] + P[2][1]))
    tot = {k: R[k, 1]["seated"] + R[k, 2]["seated"] for k in names}
    expect(F, "seated pooled, legs as trained against 0.17", (tot["base"], tot["weak"]), (147, 124))
    expect(F, "seated at 1 N·m, as trained -> 0.17", (R["base", 1]["seated"], R["weak", 1]["seated"]), (68, 76))
    expect(F, "seated at 2 N·m, as trained -> 0.17", (R["base", 2]["seated"], R["weak", 2]["seated"]), (79, 48))
    expect(F, "false seats at 2 N·m, as trained -> 0.17", (R["base", 2]["false_seat"], R["weak", 2]["false_seat"]),
           (14, 46))
    expect(F, "false seats at 1 N·m, as trained -> 0.17", (R["base", 1]["false_seat"], R["weak", 1]["false_seat"]),
           (25, 16))
    expect(F, "damaged, as trained and 0.17", (R["base", 1]["damaged"] + R["base", 2]["damaged"],
                                                R["weak", 1]["damaged"] + R["weak", 2]["damaged"]), (6, 6))
    expect(F, "falls and lost keys, both legs and frictions", sum(R[k, f]["fell"] + R[k, f]["lost"] for k in names
                                                              for f in (1, 2)), 0)
    expect(F, "paired pooled (only as trained, only 0.17, p)", P["both"][:2] + (as_quoted(P["both"][2], 0.018),),
           (55, 32, 0.018))

    print("\nFigure 3 (F-306): ki7_s62_9200, legs as trained against legs answering 0.17, of 96 per friction")
    for f in (1, 2):
        for k in names:
            r = R[k, f]
            print(f"  {f} N·m  {k:4s}  seated {r['seated']:2d}  false seats {r['false_seat']:2d}  damaged {r['damaged']}  of {r['of']}")
        print(f"  {f} N·m  paired: only as trained {P[f][0]}, only 0.17 {P[f][1]}, p {pfmt(P[f][2])}")
    print(f"  pooled: seated {tot['base']} against {tot['weak']}; paired {P['both'][0]} / {P['both'][1]}, "
          f"p {pfmt(P['both'][2])}")
    if any(p.startswith(F) for p in PROBLEMS):
        print("  NOT WRITTEN: a number does not match F-306")
        return

    fig = plt.figure(figsize=(FIG_W, FIG_H), dpi=DPI)
    y = header(fig, [
        "On legs that answer 0.17 of each target, as the dog's do, the dog's insertion seats fewer keys",
        "ki7_s62_9200 seats 124 of 192 against 147 on the legs it trained on. The loss is all at 2 N·m, where "
        "false seats rise from 14 to 46",
    ], [
        "Simulation only. The dog's conditions (strict keyway, leg-odometry base), the frozen 96 locks at each leg "
        "friction. The same locks; the legs' answer is the only change.",
        "A seat counts only inside the 15° tilt stop. 0.17 is F-256's measurement on the dog; the dog's leg friction "
        "is not measured.",
    ])
    legend_row(fig, [("patch", BLUE, "Seated inside the tilt stop"),
                     ("patch", ORANGE, "False seat (layer takes a jammed key for seated)"),
                     ("patch", AQUA, "Damaged key")], y - 0.012)
    rows = [("base", "Legs as trained", "answer as commanded"), ("weak", "Legs answer 0.17", "each target's offset × 0.17")]
    bottom, height = 0.185, 0.50
    for i, f in enumerate((1, 2)):
        ax = fig.add_axes([0.19 + i * 0.41, bottom, 0.36, height])
        ax.set_xlim(0, 96 * 1.12)
        ax.set_ylim(len(rows) - 0.5, -0.5)
        style_value_axis(ax, [0, 24, 48, 72, 96], "episodes, of 96 locks")
        ax.set_yticks([])
        b, c, p = P[f]
        ax.set_title(f"Leg friction {f} N·m: seats {R['base', f]['seated']} → {R['weak', f]['seated']}  "
                     f"(paired {b} / {c}, p {pfmt(p)})", loc="left", fontsize=15.5, weight="bold", color=INK, pad=14)
        sx, sy = px_scale(ax)
        step = 30 * sy
        for k, (key, name, note) in enumerate(rows):
            r = R[key, f]
            for j, (m, col) in enumerate((("seated", BLUE), ("false_seat", ORANGE), ("damaged", AQUA))):
                yy = k + (j - 1) * step
                hbar(ax, yy, 0, r[m], 24, col)
                ax.text(r[m] + 6 * sx, yy, fmt(r[m]), va="center", ha="left", fontsize=14.5, color=INK,
                        weight="bold" if m == "seated" else "normal")
            if i == 0:
                ax.text(-0.04, k - 0.08, name, transform=ax.get_yaxis_transform(), ha="right", va="center",
                        fontsize=15, weight="bold", color=INK)
                ax.text(-0.04, k + 0.12, note, transform=ax.get_yaxis_transform(), ha="right", va="center",
                        fontsize=13, color=INK2)
            if k:
                ax.axhline(k - 0.5, color=GRID, lw=PX, zorder=0)
    dmg = {k: R[k, 1]["damaged"] + R[k, 2]["damaged"] for k in names}
    footnote(fig, [
        f"Paired by lock: only with the legs as trained / only with legs answering 0.17, exact McNemar. Both frictions "
        f"pooled: seats {tot['base']} → {tot['weak']} ({P['both'][0]} / {P['both'][1]}, p {pfmt(P['both'][2])}).",
        f"Damaged keys {dmg['base']} and {dmg['weak']}; no falls or lost keys. ki6_force_8200 was not run on this screen.",
    ])
    save(fig, "fig3_f306_weak_legs.png")


# ================================================================ figure 4: F-302 / F-305
SHIFTS = ("lat150", "lat200", "lat300", "dep150", "dep200", "roll150", "roll200", "roll000", "roll050")  # lockerr_compare
GRID_MAX = {"lat": 3.5, "dep": 6.0, "roll": 3.0}   # the frozen grid's largest error at scale 1 (lockerr_compare.GRID_MAX)
TRAIN_MAX = {"lat": 3.5, "dep": 8.0, "roll": 4.0}  # training's largest drawn error (keylock.ERR_*, the week 10 log)
F302 = {  # seats inside the tilt stop of 192, by shift ("base" = scale 1), and the accuracy needed by the rule
    "ki6_force_8200": ({"base": 120, "lat150": 115, "lat200": 114, "lat300": 92, "dep150": 124, "dep200": 103,
                        "roll000": 118, "roll050": 136, "roll150": 63, "roll200": 28}, {"lat": 7.0, "dep": 9.0, "roll": 3.0}),
    "ki7_s62_9200": ({"base": 147, "lat150": 135, "lat200": 125, "lat300": 104, "dep150": 149, "dep200": 142,
                      "roll000": 122, "roll050": 165, "roll150": 94, "roll200": 61}, {"lat": 5.25, "dep": 12.0, "roll": 3.0}),
}


def fig4():
    F = "fig4"
    S, need = {}, {}
    for stem in F302:
        B = insert_eps(f"lockerr_{stem}_base_f1", F) + insert_eps(f"lockerr_{stem}_base_f2", F)
        base = sum(map(seat, B))
        S[stem] = {"base": (base, 1.0)}
        matters = {}
        for s in SHIFTS:
            X = insert_eps(f"lockerr_{stem}_{s}_f1", F) + insert_eps(f"lockerr_{stem}_{s}_f2", F)
            assert len(X) == len(B) and all(x["key_row"] == y["key_row"] and x["direction_deg"] == y["direction_deg"]
                                            for x, y in zip(X, B))
            b = sum(seat(y) and not seat(x) for x, y in zip(X, B))
            c = sum(seat(x) and not seat(y) for x, y in zip(X, B))
            n = sum(map(seat, X))
            S[stem][s] = (n, mcnemar(b, c))
            matters[s] = bool(mcnemar(b, c) < 0.05 and abs(n - base) >= 0.1 * base)
        need[stem] = {}
        for axis, mx in GRID_MAX.items():   # lockerr_compare's "accuracy needed"
            scales = sorted(v for v in (int(s[len(axis):]) / 100 for s in SHIFTS if s.startswith(axis)) if v > 1.0)
            last = 1.0
            for v in scales:
                if matters[f"{axis}{int(round(v * 100))}"]:
                    break
                last = v
            need[stem][axis] = mx * last
        want, want_need = F302[stem]
        expect(F, f"{stem} seats by shift", {k: v[0] for k, v in S[stem].items()}, want)
        expect(F, f"{stem} accuracy needed (lat mm, dep mm, roll deg)", need[stem], want_need)

    print("\nFigure 4 (F-302 / F-305): seats inside the tilt stop of 192, by the grid's largest lock-estimate error")
    for stem in F302:
        for axis in GRID_MAX:
            pts = sorted(((GRID_MAX[axis] * (1.0 if k == "base" else int(k[len(axis):]) / 100), v[0], v[1])
                          for k, v in S[stem].items() if k == "base" or k.startswith(axis)))
            print(f"  {stem:15s} {axis:4s} " + ", ".join(f"{x:g}: {n} (p {pfmt(p)})" for x, n, p in pts)
                  + f"   needed {need[stem][axis]:g}")
    if any(p.startswith(F) for p in PROBLEMS):
        print("  NOT WRITTEN: a number does not match F-302 / F-305")
        return

    fig = plt.figure(figsize=(FIG_W, FIG_H), dpi=DPI)
    y = header(fig, [
        "How far off the lock estimate may be: within 5.25 mm across the keyway, 12 mm along it and 3° in roll",
        "for the dog's insertion ki7_s62_9200, which seats more than ki6_force_8200 at every error size; in roll it "
        "seats 165 at 1.5° but 122 at 0°",
    ], [
        "Simulation only. The dog's conditions, the frozen 96 locks at 1 and 2 N·m: 192 episodes per point. Each "
        "lock's estimate error is scaled on one axis at a time; x is the test grid's largest error.",
        "A seat counts only inside the 15° tilt stop. The dog's tag accuracy is not measured.",
    ])
    legend_row(fig, [("line", BLUE, "ki7_s62_9200, the dog's insertion now (F-305)"),
                     ("line", GRAY, "ki6_force_8200, before it (F-302)")], y - 0.012)
    panels = [("lat", "Across the keyway (lateral)", "mm"), ("dep", "Along the keyway (depth)", "mm"),
              ("roll", "Roll about the keyway", "°")]
    left, width, gapw = 0.07, 0.275, 0.045
    for i, (axis, ptitle, unit) in enumerate(panels):
        ax = fig.add_axes([left + i * (width + gapw), 0.20, width, 0.44])
        xs = sorted({GRID_MAX[axis] * (1.0 if k == "base" else int(k[len(axis):]) / 100)
                     for k in S["ki7_s62_9200"] if k == "base" or k.startswith(axis)})
        span = xs[-1] - xs[0]
        lo, hi = xs[0] - 0.08 * span, xs[-1] + 0.08 * span
        ax.set_xlim(lo, hi)
        ax.set_ylim(0, 200)
        ax.axvspan(TRAIN_MAX[axis], hi, color=TRACK, zorder=0, lw=0)
        ax.text(TRAIN_MAX[axis] + 0.025 * span, 196, "outside training", fontsize=12.5, color=INK2,
                ha="left", va="top")
        for t in (0, 48, 96, 144, 192):
            ax.axhline(t, color=GRID, lw=PX, zorder=1)
        ax.axhline(0, color=AXIS, lw=PX, zorder=2)
        ax.set_yticks([0, 48, 96, 144, 192])
        ax.set_yticklabels(["0", "48", "96", "144", "192"] if i == 0 else [])
        ax.set_xticks(xs)
        ax.set_xticklabels([f"{x:g}" + ("\nas frozen" if x == GRID_MAX[axis] else "") for x in xs])
        ax.tick_params(length=0, pad=8)
        for s in ("top", "right", "left", "bottom"):
            ax.spines[s].set_visible(False)
        ax.set_xlabel(f"largest estimate error on the grid ({unit})", fontsize=14, color=INK2, labelpad=8)
        if i == 0:
            ax.set_ylabel("seats inside the tilt stop, of 192", fontsize=14, color=INK2, labelpad=10)
        ax.set_title(ptitle, loc="left", fontsize=16, weight="bold", color=INK, pad=34)
        u = unit if unit == "°" else f" {unit}"
        ax.text(0, 1.035, f"needed: ki7 within {need['ki7_s62_9200'][axis]:g}{u}, "
                          f"ki6 within {need['ki6_force_8200'][axis]:g}{u}",
                transform=ax.transAxes, fontsize=13, color=INK2, ha="left", va="bottom")
        for stem, col, z, dy, va in (("ki6_force_8200", GRAY, 4, -11, "top"), ("ki7_s62_9200", BLUE, 5, 11, "bottom")):
            pts = sorted((GRID_MAX[axis] * (1.0 if k == "base" else int(k[len(axis):]) / 100), v[0])
                         for k, v in S[stem].items() if k == "base" or k.startswith(axis))
            px_, py_ = zip(*pts)
            ax.plot(px_, py_, color=col, lw=2 * PX, solid_capstyle="round", solid_joinstyle="round", zorder=z)
            ax.plot(px_, py_, ls="none", marker="o", ms=10 * PX + 3, mfc=col, mec=SURFACE, mew=2 * PX, zorder=z + 0.5)
            for x, n in pts:
                ax.annotate(f"{n}", (x, n), xytext=(0, dy), textcoords="offset points", ha="center", va=va,
                            fontsize=14 if stem == "ki7_s62_9200" else 12.5,
                            weight="bold" if stem == "ki7_s62_9200" else "normal",
                            color=INK if stem == "ki7_s62_9200" else INK2, zorder=6,
                            path_effects=[patheffects.withStroke(linewidth=4, foreground=SURFACE)])
    footnote(fig, [
        "Needed, by the rule stated before scoring: the largest error before the first scale past ×1 that moves seats "
        "by ≥ 10% of the policy's own baseline at p < 0.05",
        "(exact McNemar, paired by lock). Each policy against its own baseline. Training drew errors up to 3.5 mm "
        "lateral, ±8 mm depth and ±4° roll.",
    ])
    save(fig, "fig4_f302_f305_lock_estimate.png")


def main():
    print(f"reading recorded runs under {os.path.relpath(RUNS, ROOT)}")
    for f in (fig1, fig2, fig3, fig4):
        f()
    print("\nRun ids read:")
    for fig, ids in USED.items():
        print(f"  {fig} ({len(set(ids))} runs):")
        for r in sorted(set(ids)):
            print(f"    {r}")
    if PROBLEMS:
        print("\nMISMATCHES against the findings (those figures were not written):")
        for p in PROBLEMS:
            print("  " + p)
        sys.exit(1)
    print("\nAll plotted numbers match F-294, F-295, F-302, F-303, F-305 and F-306.")


if __name__ == "__main__":
    main()
