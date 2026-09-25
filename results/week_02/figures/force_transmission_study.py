"""How much force the D1 can hold at its jaw centre, and how much of that depends on routing the force line
through its joints rather than around them. CPU model only: published joint limits, the weld's mass model,
gravity + J^T F, the arm's own kinematics. No simulator, no servo loop, no structure. Week 2 log, 2026-09-24;
F-101.

    PYTHONPATH=. python results/week_02/figures/force_transmission_study.py      # ~2 min, any Python with numpy

Four questions, one section each:

  1. Typical postures. Over reachable, body-clear configurations, the most the arm holds along the direction
     that matters (pull toward the mount, horizontal pull) and along its best and worst directions.
  2. Best alignment. For a horizontal pull on a handle at a given height, the largest force any arm posture
     holds, with the base level or pitched and at three heights -- the part the legs can choose. Reported
     both nominally and robustly: the 10th percentile when every joint is off by 2 degrees (s.d.), because a
     policy will not hold a singular posture to a tenth of a degree.
  3. Tolerance. At the robust-best posture, how capacity falls with joint error and with pull-direction error.
  4. Push versus pull. The same posture with the tip pinned (a hooked handle, a button under friction):
     the force at which the servo stiffness can no longer hold the arm's null-space motions, for tension
     and for compression. Torque capacity is the same number either way; stability is not.

Numbers go to force_transmission_study.json; the figure to force_transmission_study.png.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

import d1_ik
from position_only.workspace import (DISTAL_LINKS, EFFORT_LIMIT_NM, MOUNT_B, clear_of_body, forward, load_urdf,
                                     sample_configs, simulated_masses)

HERE = Path(__file__).resolve().parent
JAW_CENTRE_LINK6 = np.array([0.0, 0.0, 0.1051])     # unifp_train.task_cfg.TOOL_OFFSET_M, the controlled point
GRAVITY_W = np.array([0.0, 0.0, -9.81])
#: Sim arm stiffness, N·m/rad: unifp_isaaclab.interface.P_GAINS for Joint1-6. The real servo's is unmeasured.
SIM_KP = np.array([60.0, 60.0, 40.0, 40.0, 40.0, 40.0])
#: Whole robot, kg: the Isaac Gym asset (unifp_isaaclab.robot.report_model's reference, 18.172 kg).
ROBOT_MASS_KG = 18.172
#: Go2 hip spacing, m: feet sit roughly under the hips in stance (0.1934 m either side of the base origin).
HIP_X_M = 0.1934
BASE_HEIGHTS_M = (0.24, 0.30, 0.34)
BASE_PITCH_DEG = (-15.0, 0.0, 15.0)                  # positive = nose down
HANDLE_HEIGHTS_M = np.round(np.arange(0.10, 0.71, 0.05), 3)
ROBUST_SIGMA_DEG = 2.0

JOINTS, LINKS = load_urdf()
MASSES = simulated_masses(LINKS)


def rot_y(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


def kinematics(q: np.ndarray, gravity_b: np.ndarray):
    """Jaw-centre position (N,3), linear Jacobian (N,3,6) and holding torque (N,6), all in the base frame.

    Holding torque is what the motors must supply against gravity alone; `gravity_b` is gravity expressed in
    the base frame, so a pitched base is gravity rotated rather than a different arm.
    """
    frames, axes, origins = forward(JOINTS, q)
    rot6, pos6 = frames["Link6"]
    tip = pos6 + np.einsum("nij,j->ni", rot6, JAW_CENTRE_LINK6)
    jac = d1_ik.jacobian(axes, origins, tip)[:, :3]
    hold = np.zeros(q.shape)
    for name, (rot, pos) in frames.items():
        if name == "base_link":
            continue
        com = pos + np.einsum("nij,j->ni", rot, LINKS[name]["com"])
        force = MASSES[name] * gravity_b
        for j in range(6):
            if name in DISTAL_LINKS[j]:
                hold[:, j] -= np.einsum("ni,ni->n", axes[:, j], np.cross(com - origins[:, j], force))
    return tip, jac, hold


def capacity(jac: np.ndarray, hold: np.ndarray, u: np.ndarray, limits=EFFORT_LIMIT_NM):
    """Largest force (N,) the arm applies along unit `u` (N,3 or 3) with every joint inside its limit.

    Motor torque needed is `hold + F * J^T u`; each joint's root is where that reaches +/- its limit. A
    posture that cannot even hold itself scores 0. Also returns the index of the binding joint.
    """
    u = np.broadcast_to(u, (jac.shape[0], 3))
    per_newton = np.einsum("nij,ni->nj", jac, u)
    with np.errstate(divide="ignore", invalid="ignore"):
        up = (limits - hold) / per_newton
        down = (-limits - hold) / per_newton
    roots = np.where(per_newton > 1e-12, up, np.where(per_newton < -1e-12, down, np.inf))
    force = roots.min(axis=1)
    joint = roots.argmin(axis=1)
    held = (np.abs(hold) <= limits).all(axis=1)
    return np.where(held, np.clip(force, 0.0, None), 0.0), joint


def fibonacci_sphere(count: int) -> np.ndarray:
    i = np.arange(count) + 0.5
    polar = np.arccos(1 - 2 * i / count)
    azimuth = math.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(azimuth) * np.sin(polar), np.sin(azimuth) * np.sin(polar), np.cos(polar)], -1)


def pct(values, qs=(10, 50, 90)):
    values = np.asarray(values)
    return {f"p{q}": round(float(np.percentile(values, q)), 2) for q in qs}


# --- 1. typical postures -------------------------------------------------------------------------------------

def typical_postures(rng, count=300_000):
    q = sample_configs(JOINTS, count, rng)
    q = q[clear_of_body(JOINTS, q, 0.30)]
    tip, jac, hold = kinematics(q, GRAVITY_W)
    # UniFP's goal shell: 0.30-0.58 m from a centre 0.49 m above the ground under the base, i.e. 0.19 m above
    # the base origin at the standing height. Postures whose jaw centre lands in it are the ones the reaching
    # task actually uses.
    centre = np.array([0.0, 0.0, 0.49 - 0.30])
    radius = np.linalg.norm(tip - centre, axis=1)
    keep = (radius > 0.30) & (radius < 0.58) & (tip[:, 0] > 0.0)
    q, tip, jac, hold = q[keep], tip[keep], jac[keep], hold[keep]
    to_mount = MOUNT_B - tip
    to_mount /= np.linalg.norm(to_mount, axis=1, keepdims=True)
    toward, _ = capacity(jac, hold, to_mount)
    horizontal, _ = capacity(jac, hold, np.array([-1.0, 0.0, 0.0]))
    sub = rng.choice(len(q), size=min(20_000, len(q)), replace=False)
    dirs = fibonacci_sphere(162)
    per_dir = np.stack([capacity(jac[sub], hold[sub], d)[0] for d in dirs], 1)
    return {
        "postures": int(len(q)),
        "pull_toward_mount_n": pct(toward),
        "horizontal_pull_n": pct(horizontal),
        "best_direction_n": pct(per_dir.max(1)),
        "worst_direction_n": pct(per_dir.min(1)),
        "_toward": toward, "_horizontal": horizontal,
    }


# --- 2. best alignment for a horizontal pull -----------------------------------------------------------------

def robust_capacity(q0, gravity_b, u_b, rng, sigma_deg=ROBUST_SIGMA_DEG, draws=64, quantile=10):
    """The `quantile`th percentile of capacity when every joint of q0 (6,) is perturbed by N(0, sigma)."""
    q = q0[None] + rng.normal(0.0, math.radians(sigma_deg), size=(draws, 6))
    _, jac, hold = kinematics(q, gravity_b)
    return float(np.percentile(capacity(jac, hold, u_b)[0], quantile))


def refine(q0, score, rng, steps=250, sigma0=0.15):
    """Stochastic hill climb on `score(q)`, shrinking the step as it stalls. Joint limits are respected."""
    lows = np.array([JOINTS[f"Joint{i}"]["limits"][0] for i in range(1, 7)])
    highs = np.array([JOINTS[f"Joint{i}"]["limits"][1] for i in range(1, 7)])
    best_q, best = q0.copy(), score(q0)
    sigma = sigma0
    for _ in range(steps):
        cand = np.clip(best_q + rng.normal(0.0, sigma, 6), lows, highs)
        s = score(cand)
        if s > best:
            best_q, best = cand, s
        else:
            sigma = max(sigma * 0.97, 0.005)
    return best_q, best


def best_alignment(rng, count=200_000, tolerance_m=0.025):
    """For each handle height, base height and base pitch: the best nominal and robust horizontal pull."""
    q_all = sample_configs(JOINTS, count, rng)
    u_w = np.array([-1.0, 0.0, 0.0])        # the force on the handle: toward the robot, which faces +x
    table = []
    for base_h in BASE_HEIGHTS_M:
        clear = clear_of_body(JOINTS, q_all, base_h)
        q_clear = q_all[clear]
        for pitch in BASE_PITCH_DEG:
            r = rot_y(pitch)                     # base -> world, about the base origin
            g_b, u_b = r.T @ GRAVITY_W, r.T @ u_w
            tip, jac, hold = kinematics(q_clear, g_b)
            tip_w = tip @ r.T
            height = base_h + tip_w[:, 2]
            nominal, _ = capacity(jac, hold, u_b)
            # a handle in front of the robot and beyond its head; the handle is what is at that height
            front = tip_w[:, 0] > 0.30
            for handle_h in HANDLE_HEIGHTS_M:
                band = front & (np.abs(height - handle_h) < tolerance_m)
                if not band.any():
                    table.append({"base_h": base_h, "pitch_deg": pitch, "handle_h": float(handle_h),
                                  "nominal_n": 0.0, "robust_n": 0.0, "q": None})
                    continue
                idx = np.flatnonzero(band)
                seeds = idx[np.argsort(-nominal[idx])[:4]]

                def score(q, _h=handle_h, _g=g_b, _u=u_b, _r=r, _bh=base_h):
                    q = q[None]
                    if not clear_of_body(JOINTS, q, _bh)[0]:
                        return -1.0
                    t, j, h = kinematics(q, _g)
                    tw = t @ _r.T
                    if tw[0, 0] <= 0.30:
                        return -1.0
                    miss = abs(_bh + tw[0, 2] - _h)
                    return robust_capacity(q[0], _g, _u, rng) - 400.0 * max(0.0, miss - 0.005)

                best_q, best_score = None, -1.0
                for s in seeds:
                    cand, sc = refine(q_clear[s], score, rng)
                    if sc > best_score:
                        best_q, best_score = cand, sc
                t, j, h = kinematics(best_q[None], g_b)
                nom, joint = capacity(j, h, u_b)
                table.append({
                    "base_h": base_h, "pitch_deg": pitch, "handle_h": float(handle_h),
                    "nominal_n": round(float(nom[0]), 1), "binding_joint": f"Joint{int(joint[0]) + 1}",
                    "robust_n": round(robust_capacity(best_q, g_b, u_b, np.random.default_rng(7), draws=256), 1),
                    "q": [round(float(v), 4) for v in best_q],
                    "tip_w": [round(float(v), 3) for v in (t @ r.T)[0] + np.array([0.0, 0.0, base_h])],
                })
    return table


# --- 3. tolerance --------------------------------------------------------------------------------------------

def tolerance(q0, gravity_b, u_b, rng):
    out = {"joint_sigma_deg": {}, "direction_error_deg": {}}
    for sigma in (0.0, 0.5, 1.0, 2.0, 5.0, 10.0):
        q = q0[None] + rng.normal(0.0, math.radians(sigma), size=(512, 6))
        _, jac, hold = kinematics(q, gravity_b)
        out["joint_sigma_deg"][str(sigma)] = pct(capacity(jac, hold, u_b)[0])
    _, jac, hold = kinematics(q0[None], gravity_b)
    perp = np.cross(u_b, [0.0, 0.0, 1.0])
    perp = perp / np.linalg.norm(perp) if np.linalg.norm(perp) > 1e-6 else np.array([0.0, 1.0, 0.0])
    up = np.cross(perp, u_b)
    for err in (0.0, 2.0, 5.0, 10.0, 20.0):
        a = math.radians(err)
        caps = []
        for phi in np.linspace(0, 2 * math.pi, 16, endpoint=False):
            d = math.cos(a) * u_b + math.sin(a) * (math.cos(phi) * perp + math.sin(phi) * up)
            caps.append(capacity(jac, hold, d)[0][0])
        out["direction_error_deg"][str(err)] = {"min": round(float(min(caps)), 1),
                                                "median": round(float(np.median(caps)), 1)}
    return out


# --- 4. push versus pull -------------------------------------------------------------------------------------

def generalised_force_jacobian(q0, f_b, gravity_b, eps=1e-5):
    """d(J^T f)/dq and d(hold)/dq at q0, by central differences (6,6)."""
    djtf, dhold = np.zeros((6, 6)), np.zeros((6, 6))
    for k in range(6):
        dq = np.zeros(6)
        dq[k] = eps
        _, jp, hp = kinematics((q0 + dq)[None], gravity_b)
        _, jm, hm = kinematics((q0 - dq)[None], gravity_b)
        djtf[:, k] = (jp[0].T @ f_b - jm[0].T @ f_b) / (2 * eps)
        dhold[:, k] = (hp[0] - hm[0]) / (2 * eps)
    return djtf, dhold


def critical_force(q0, u_b, gravity_b, stiffness):
    """Force along the pull (u_b, applied by the arm) at which the pinned-tip arm loses stability.

    With the tip held by the handle, the arm keeps three degrees of freedom (the null space N of the 3x6
    Jacobian: the elbow swinging about the tip-shoulder line, and so on). The equilibrium is stable while
    N^T (K + dhold/dq - d(J^T f)/dq) N is positive definite, f being the handle's force *on* the arm
    (-F u_b). That matrix is A + F B, so the critical F is the smallest positive root of det(A + F B) = 0.
    Positive F is the arm pulling along u_b; negative F is pushing along it. Returns both.
    """
    _, jac, _ = kinematics(q0[None], gravity_b)
    _, _, vt = np.linalg.svd(jac[0])
    null = vt[3:].T                                           # (6,3)
    djtf, dhold = generalised_force_jacobian(q0, -u_b, gravity_b)
    a = null.T @ (np.diag(stiffness) + dhold) @ null
    b = null.T @ (-djtf) @ null
    a, b = 0.5 * (a + a.T), 0.5 * (b + b.T)
    mu = np.linalg.eigvals(np.linalg.solve(a, b))
    mu = mu[np.abs(mu.imag) < 1e-9].real
    # |mu| below 1e-9 per newton is a root beyond 1e9 N: numerically zero, and physically no root at all.
    pull = [-1.0 / m for m in mu if m < -1e-9]                # F > 0 roots
    push = [1.0 / m for m in mu if m > 1e-9]                  # F < 0 roots, as magnitudes
    return (round(min(pull), 1) if pull else None), (round(min(push), 1) if push else None)


def body_limits():
    """Static whole-body ceilings on a horizontal pull at handle height h: tipping over the front feet, and
    sliding. The robot is a rigid body on four point feet; CoM at the base origin unless leaned."""
    mg = ROBOT_MASS_KG * 9.81
    out = {}
    for h in (0.2, 0.4, 0.6):
        out[str(h)] = {
            "tip_level_n": round(mg * HIP_X_M / h, 1),
            "tip_lean_back_8cm_n": round(mg * (HIP_X_M + 0.08) / h, 1),
            "slide_mu_0.5_n": round(0.5 * mg, 1),
            "slide_mu_1.0_n": round(1.0 * mg, 1),
        }
    return out


def main():
    rng = np.random.default_rng(20260924)
    result = {}
    typical = typical_postures(rng)
    result["typical"] = {k: v for k, v in typical.items() if not k.startswith("_")}
    print("1. typical postures", json.dumps(result["typical"], indent=1))

    table = best_alignment(rng)
    result["best_alignment"] = table
    print("2. best alignment, horizontal pull (robust = p10 at 2 deg joint s.d.)")
    for row in table:
        if row["q"] is None:
            continue
        print(f"   base {row['base_h']:.2f} m pitch {row['pitch_deg']:+5.1f}  handle {row['handle_h']:.2f} m  "
              f"nominal {row['nominal_n']:7.1f} N  robust {row['robust_n']:6.1f} N  ({row.get('binding_joint')})")

    # Sections 3 and 4 at two postures: the best robust one at the standing height (whatever pitch it took),
    # and the best with the base level and the handle at 0.35 m, a posture the legs did not help choose.
    u_w = np.array([-1.0, 0.0, 0.0])
    standing = [r for r in table if r["base_h"] == 0.30 and r["q"] is not None]
    cases = {
        "best_standing": max(standing, key=lambda r: r["robust_n"]),
        "level_0.35": min((r for r in standing if r["pitch_deg"] == 0.0), key=lambda r: abs(r["handle_h"] - 0.35)),
    }
    result["cases"] = {}
    for name, row in cases.items():
        q0, r = np.array(row["q"]), rot_y(row["pitch_deg"])
        g_b, u_b = r.T @ GRAVITY_W, r.T @ u_w
        stability = {}
        for scale in (0.1, 0.25, 0.5, 1.0):
            pull, push = critical_force(q0, u_b, g_b, SIM_KP * scale)
            stability[str(scale)] = {"pull_n": pull, "push_n": push}
        result["cases"][name] = {"posture": row, "tolerance": tolerance(q0, g_b, u_b, rng),
                                 "stability_vs_stiffness_scale": stability}
        print(f"3-4. {name}", json.dumps(result["cases"][name], indent=1))

    result["body_limits"] = body_limits()
    print("   body limits", json.dumps(result["body_limits"], indent=1))

    (HERE / "force_transmission_study.json").write_text(json.dumps(result, indent=1) + "\n")
    np.savez(HERE / "force_transmission_study_samples.npz", toward=typical["_toward"],
             horizontal=typical["_horizontal"])


def draw(result, path):
    """Three panels: what posture buys, how fragile it is, and why pushing differs from pulling."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ink, muted, grid = "#0b0b0b", "#52514e", "#e4e3df"
    series = {15.0: ("#2a78d6", "nose down 15°"), 0.0: ("#eb6834", "level"), -15.0: ("#1baf7a", "nose up 15°")}
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": muted, "axes.labelcolor": ink, "xtick.color": muted,
                         "ytick.color": muted, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(1, 3, figsize=(13.0, 4.0), constrained_layout=True)

    ax = axes[0]
    typical = result["typical"]["horizontal_pull_n"]["p50"]
    for pitch, (colour, label) in series.items():
        rows = [r for r in result["best_alignment"] if r["pitch_deg"] == pitch and r["q"] is not None]
        heights = sorted({r["handle_h"] for r in rows})
        best = [max(r["robust_n"] for r in rows if r["handle_h"] == h) for h in heights]
        ax.plot(heights, best, color=colour, lw=2, marker="o", ms=4)
        ax.annotate(label, (heights[-1], best[-1]), xytext=(4, 0), textcoords="offset points", color=ink,
                    va="center", fontsize=8)
    ax.axhline(typical, color=muted, lw=1, ls="--")
    ax.annotate(f"typical bent reach, median {typical:.1f} N", (0.45, typical), xytext=(0, -11),
                textcoords="offset points", color=muted, fontsize=8)
    ax.set_xlim(0.08, 0.80)
    ax.set_ylim(0, None)
    ax.set_xlabel("handle height above ground (m)")
    ax.set_ylabel("horizontal pull the arm holds (N)")
    ax.set_title("Best posture per height, base pitch chosen\n(p10 with 2° joint error; best of 3 base heights)",
                 fontsize=9, color=ink, loc="left")
    ax.grid(axis="y", color=grid, lw=0.8)

    ax = axes[1]
    for name, colour, label in (("best_standing", "#2a78d6", "best standing posture"),
                                ("level_0.35", "#eb6834", "level base, handle 0.35 m")):
        tol = result["cases"][name]["tolerance"]["joint_sigma_deg"]
        sig = [float(k) for k in tol]
        ax.plot(sig, [tol[k]["p50"] for k in tol], color=colour, lw=2, marker="o", ms=4)
        ax.fill_between(sig, [tol[k]["p10"] for k in tol], [tol[k]["p90"] for k in tol], color=colour,
                        alpha=0.15, lw=0)
        ax.annotate(label, (sig[-1], tol[str(sig[-1])]["p50"]), xytext=(-4, 8), textcoords="offset points",
                    ha="right", color=ink, fontsize=8)
    ax.set_xlabel("joint error, s.d. per joint (deg)")
    ax.set_ylabel("pull the arm holds (N)")
    ax.set_ylim(0, None)
    ax.set_title("How alignment error costs force\n(median, band p10-p90 over 512 draws)", fontsize=9,
                 color=ink, loc="left")
    ax.grid(axis="y", color=grid, lw=0.8)

    ax = axes[2]
    case = result["cases"]["best_standing"]
    stab = case["stability_vs_stiffness_scale"]
    scales = [float(k) for k in stab]
    push = [stab[k]["push_n"] for k in stab]
    ax.plot(scales, push, color="#e34948", lw=2, marker="o", ms=4)
    ax.annotate("push: arm buckles", (scales[-1], push[-1]), xytext=(-4, -12), textcoords="offset points",
                ha="right", color=ink, fontsize=8)
    cap = case["posture"]["robust_n"]
    ax.axhline(cap, color=muted, lw=1, ls="--")
    ax.annotate(f"torque ceiling, {cap:.0f} N (push or pull)", (0.1, cap), xytext=(0, 4),
                textcoords="offset points", color=muted, fontsize=8)
    pulls = [stab[k]["pull_n"] for k in stab]
    ax.annotate("pull: no instability at any force" if all(p is None for p in pulls) else "pull: see JSON",
                (0.1, max(push) * 0.85), color=ink, fontsize=8)
    ax.set_xscale("log")
    ax.set_xticks(scales, [f"{s:g}×" for s in scales])
    ax.set_xlabel("servo stiffness, multiple of the sim's kp (60/40 N·m/rad)")
    ax.set_ylabel("critical force, tip pinned (N)")
    ax.set_ylim(0, None)
    ax.set_title("Tension is stable, compression is not\n(best standing posture, arm only)", fontsize=9,
                 color=ink, loc="left")
    ax.grid(axis="y", color=grid, lw=0.8)

    fig.savefig(path, dpi=160)


if __name__ == "__main__":
    import sys
    if "--draw" in sys.argv:
        draw(json.loads((HERE / "force_transmission_study.json").read_text()), HERE / "force_transmission_study.png")
    else:
        main()
        draw(json.loads((HERE / "force_transmission_study.json").read_text()), HERE / "force_transmission_study.png")
