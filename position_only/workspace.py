"""Kinematic workspace of the welded D1's controlled point against the reaching target box (CPU only).

    python -m position_only.workspace --out results/week_01/figures

Forward kinematics come from `d1_arm/d1.urdf`, mounted as `weld.py` mounts it (identity rotation,
0.08 m above the Go2 base origin). Link masses follow `weld.py`'s D1 mass model: shell inertials,
plus a servo per moving link, with the rest on the welded base, so static gravity torques match
what PhysX is given. The collision test is a proxy (joint-to-joint segments against a box around
the Go2 body and the ground), not mesh collision. Nothing here runs the simulator, so torque
limits are checked statically: no base motion, no arm acceleration.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np

from .tool_point import TOOL_BODY, TOOL_OFFSET_M

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "d1_arm/d1.urdf"
ARM_JOINTS = [f"Joint{i}" for i in range(1, 7)]
MOUNT_B = np.array([0.0, 0.0, 0.08])  # weld.py mount_pos
ARM_MASS_KG = 3.152  # run_position_only.py arm_mass_kg
SERVO_MASS_BY_LINK = {"Link1": 0.060, "Link2": 0.060, "Link3": 0.045, "Link4": 0.045, "Link5": 0.045, "Link6": 0.090}
EFFORT_LIMIT_NM = np.array([3.3, 3.3, 1.7, 1.7, 1.7, 1.7])  # motor_model.D1_EFFORT_LIMIT_NM
SOFT_LIMIT_FACTOR = 0.9  # UNITREE_GO2_CFG soft_joint_pos_limit_factor; actions are clamped to it
GRAVITY = np.array([0.0, 0.0, -9.81])
# Target box relative to the environment origin (position_only/mdp.py WorldPositionCommandCfg.ranges).
TARGET_RANGES = ((0.24, 0.36), (-0.08, 0.08), (0.66, 0.78))
# Collision proxy in the Go2 base frame: the trunk and hips, and the ground below the base.
BODY_BOX_B = ((-0.30, 0.30), (-0.13, 0.13), (-0.20, 0.06))
LINK_RADIUS_M = 0.03
SUCCESS_RADIUS_M = 0.05  # G1a reach criterion
ZERO_ACTION_BASE_HEIGHT_M = 0.266  # Week 1 smoke runs, settled mean
# Where the base actually settles under zero actions: every reset drops it from 0.42 m and it slides back
# 7.4 cm (Week 1 smoke 20260915T104159_671108Z, 8 envs: x -0.065 to -0.094 m, y 0.000 m, yaw 0.0 deg).
ZERO_ACTION_BASE_OFFSET_M = (-0.0745, 0.0, 0.2666)


def _vec(text):
    return np.array([float(v) for v in text.split()])


def rpy_matrix(roll, pitch, yaw):
    """URDF fixed-axis roll-pitch-yaw: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, angles):
    """Rotation matrices (..., 3, 3) about a unit axis (Rodrigues)."""
    axis = axis / np.linalg.norm(axis)
    k = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    s, c = np.sin(angles)[..., None, None], np.cos(angles)[..., None, None]
    return np.eye(3) + s * k + (1.0 - c) * (k @ k)


def load_urdf(path=URDF):
    root = ET.parse(path).getroot()
    joints = {}
    for joint in root.findall("joint"):
        origin, limit, axis = joint.find("origin"), joint.find("limit"), joint.find("axis")
        joints[joint.get("name")] = {
            "parent": joint.find("parent").get("link"), "child": joint.find("child").get("link"),
            "xyz": _vec(origin.get("xyz")), "rpy": rpy_matrix(*_vec(origin.get("rpy"))),
            "axis": _vec(axis.get("xyz")), "limits": (float(limit.get("lower")), float(limit.get("upper"))),
        }
    links = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        links[link.get("name")] = {"mass": float(inertial.find("mass").get("value")),
                                   "com": _vec(inertial.find("origin").get("xyz"))}
    return joints, links


def simulated_masses(links):
    """weld.py's D1 mass model: shell + servo per moving link, the remainder on base_link."""
    masses = {name: link["mass"] + SERVO_MASS_BY_LINK.get(name, 0.0) for name, link in links.items()}
    masses["base_link"] += ARM_MASS_KG - sum(masses.values())
    return masses


def forward(joints, q, gripper=None):
    """Link frames in the Go2 base frame for arm angles q (N, 6) and jaw positions (N, 2), closed if None.

    Returns {link: (R (N,3,3), p (N,3))} plus each arm joint's axis and origin in the base frame.
    """
    n = q.shape[0]
    frames = {"base_link": (np.broadcast_to(np.eye(3), (n, 3, 3)).copy(), np.broadcast_to(MOUNT_B, (n, 3)).copy())}
    axes, origins = [], []
    for name in ARM_JOINTS + ["Joint7_1", "Joint7_2"]:
        joint = joints[name]
        rot_parent, pos_parent = frames[joint["parent"]]
        pos = pos_parent + np.einsum("nij,j->ni", rot_parent, joint["xyz"])
        rot = rot_parent @ joint["rpy"]
        if name in ARM_JOINTS:
            axes.append(np.einsum("nij,j->ni", rot, joint["axis"] / np.linalg.norm(joint["axis"])))
            origins.append(pos)
            rot = rot @ axis_angle(joint["axis"], q[:, ARM_JOINTS.index(name)])
        elif gripper is not None:  # prismatic fingers slide along their axis
            axis = joint["axis"] / np.linalg.norm(joint["axis"])
            pos = pos + np.einsum("nij,j->ni", rot, axis) * gripper[:, ["Joint7_1", "Joint7_2"].index(name), None]
        frames[joint["child"]] = (rot, pos)
    return frames, np.stack(axes, 1), np.stack(origins, 1)


def gravity_torques(joints, links, q):
    """Static joint torque (N, 6) needed to hold q against gravity, base level."""
    frames, axes, origins = forward(joints, q)
    masses = simulated_masses(links)
    torque = np.zeros(q.shape)
    for name, (rot, pos) in frames.items():
        if name == "base_link":
            continue
        com = pos + np.einsum("nij,j->ni", rot, links[name]["com"])
        force = masses[name] * GRAVITY
        for j in range(6):
            if name in DISTAL_LINKS[j]:
                torque[:, j] -= np.einsum("ni,ni->n", axes[:, j], np.cross(com - origins[:, j], force))
    return torque


# Joint{j} drives Link{j}, so it carries Link{j} and every link after it.
DISTAL_LINKS = [set([f"Link{k}" for k in range(j, 7)] + ["Link7_1", "Link7_2"]) for j in range(1, 7)]


def read_stl(path):
    """Vertices (M, 3) of a binary STL."""
    data = Path(path).read_bytes()
    count = struct.unpack("<I", data[80:84])[0]
    triangles = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                              count=count, offset=84)
    return np.unique(triangles["v"].reshape(-1, 3), axis=0).astype(float)


def gripper_points(joints):
    """Finger mesh vertices in the Link6 frame, gripper closed (q7 = 0)."""
    points = []
    for name in ("Joint7_1", "Joint7_2"):
        joint = joints[name]
        vertices = read_stl(ROOT / "d1_arm/meshes" / f"{joint['child']}.STL")
        points.append(vertices @ joint["rpy"].T + joint["xyz"])
    return points


def tip_offsets(joints):
    """Fingertip centre and finger-pad centre in the Link6 frame, from the CAD meshes."""
    fingers = gripper_points(joints)
    both = np.vstack(fingers)
    centroid = both.mean(axis=0)
    approach = centroid / np.linalg.norm(centroid)  # Link6 origin towards the fingers
    reach = both @ approach
    tip = reach.max()
    near_tip = both[reach > tip - 0.005]
    pads = [f[(f @ approach) > (f @ approach).max() - 0.02] for f in fingers]
    return {
        "approach_axis_link6": approach.round(4).tolist(),
        "fingertip_centre_m": near_tip.mean(axis=0).round(4).tolist(),
        "finger_pad_centre_m": np.mean([p.mean(axis=0) for p in pads], axis=0).round(4).tolist(),
        "finger_length_along_approach_m": float(round(tip - reach.min(), 4)),
    }


def pincer_tip(finger="Link7_1"):
    """Centre of a pincer's end face in its own link frame, from the CAD mesh.

    The end face is every vertex within 2 mm of the finger's far end along Link6's z axis
    (the direction the fingers point).
    """
    joints, _ = load_urdf()
    joint = next(j for j in joints.values() if j["child"] == finger)
    vertices = read_stl(ROOT / "d1_arm/meshes" / f"{finger}.STL")
    approach = joint["rpy"].T @ np.array([0.0, 0.0, 1.0])
    reach = vertices @ approach
    return vertices[reach > reach.max() - 0.002].mean(axis=0)


def tool_position(frames, body=TOOL_BODY, offset=TOOL_OFFSET_M):
    """The controlled point in the base frame, for frames from `forward`."""
    rot, pos = frames[body]
    return pos + np.einsum("nij,j->ni", rot, np.asarray(offset, dtype=float))


def sample_configs(joints, count, rng):
    lows, highs = [], []
    for name in ARM_JOINTS:
        low, high = joints[name]["limits"]
        mid, half = (low + high) / 2.0, (high - low) / 2.0 * SOFT_LIMIT_FACTOR
        lows.append(mid - half)
        highs.append(mid + half)
    return rng.uniform(lows, highs, size=(count, 6))


def clear_of_body(joints, q, base_height):
    """Collision proxy: joint-to-joint segments (and the gripper) keep LINK_RADIUS_M from the body box and ground."""
    frames, _, origins = forward(joints, q)
    tip = frames["Link6"][1] + np.einsum("nij,j->ni", frames["Link6"][0], np.array([0.0, 0.0, 0.126]))
    chain = np.concatenate([origins[:, 1:], frames["Link6"][1][:, None], tip[:, None]], axis=1)  # from Joint2 on
    clear = np.ones(q.shape[0], dtype=bool)
    for a, b in zip(range(chain.shape[1] - 1), range(1, chain.shape[1])):
        for t in np.linspace(0.0, 1.0, 5):
            point = chain[:, a] * (1 - t) + chain[:, b] * t
            inside = np.ones(q.shape[0], dtype=bool)
            for axis, (low, high) in enumerate(BODY_BOX_B):
                inside &= (point[:, axis] > low - LINK_RADIUS_M) & (point[:, axis] < high + LINK_RADIUS_M)
            clear &= ~inside & (point[:, 2] > -base_height + LINK_RADIUS_M)
    return clear


def box_points(spacing=0.01):
    axes = [np.arange(low, high + 1e-9, spacing) for low, high in TARGET_RANGES]
    return np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)


def analyse(count=400_000, seed=0, base_heights=(0.22, 0.24, ZERO_ACTION_BASE_HEIGHT_M, 0.28, 0.30, 0.32, 0.34, 0.36, 0.38),
            radius=0.015):
    """Box coverage and start distances for the pincer tip (the controlled point) and, for comparison, Link6."""
    from scipy.spatial import cKDTree

    joints, links = load_urdf()
    rng = np.random.default_rng(seed)
    q = sample_configs(joints, count, rng)
    frames, _, _ = forward(joints, q)
    points = {"pincer_tip": tool_position(frames), "link6": frames["Link6"][1]}
    torque = np.abs(gravity_torques(joints, links, q))
    holdable = (torque <= EFFORT_LIMIT_NM).all(axis=1)
    # The arm starts every episode at its zero pose: upper arm up, forearm forward (not folded).
    zero_frames, _, _ = forward(joints, np.zeros((1, 6)))
    zero = {"pincer_tip": tool_position(zero_frames)[0], "link6": zero_frames["Link6"][1][0]}
    zero_torque = gravity_torques(joints, links, np.zeros((1, 6)))[0]
    targets_world = box_points()

    by_height = []
    for height in base_heights:
        targets_b = targets_world - np.array([0.0, 0.0, height])
        clear = clear_of_body(joints, q, height)
        row = {"base_height_m": height}
        for name, positions in points.items():
            entry = {}
            for label, mask in (("kinematic", np.ones(count, bool)), ("clear", clear), ("clear_and_holdable", clear & holdable)):
                hits = cKDTree(positions[mask]).query_ball_point(targets_b, r=radius, return_length=True)
                entry[f"box_fraction_{label}"] = float((hits > 0).mean())
            distance = np.linalg.norm(targets_b - zero[name], axis=1)
            entry["zero_pose_to_box_m"] = {"min": float(distance.min()), "mean": float(distance.mean()),
                                           "max": float(distance.max())}
            # G1a's success radius: targets the arm already meets without moving.
            entry["box_fraction_within_5cm_of_zero_pose"] = float((distance <= SUCCESS_RADIUS_M).mean())
            row[name] = entry
        by_height.append(row)

    # The measured zero-action stance: base slid back and settled, arm at its zero pose.
    targets_b = targets_world - np.array(ZERO_ACTION_BASE_OFFSET_M)
    settled = {"base_offset_from_env_origin_m": ZERO_ACTION_BASE_OFFSET_M}
    settled_distances = {}
    for name in points:
        distance = np.linalg.norm(targets_b - zero[name], axis=1)
        settled_distances[name] = distance
        settled[name] = {"zero_pose_to_box_m": {"min": float(distance.min()), "mean": float(distance.mean()),
                                                "max": float(distance.max())},
                         "box_fraction_within_5cm_of_zero_pose": float((distance <= SUCCESS_RADIUS_M).mean()),
                         "zero_pose_in_env_frame_m": (zero[name] + np.array(ZERO_ACTION_BASE_OFFSET_M)).round(4).tolist()}

    masses = simulated_masses(links)
    return {
        "method": "URDF forward kinematics (CPU); uniform joint samples inside the 0.9 soft limits; a box point "
                  f"counts as reachable if a sampled controlled point lies within {radius * 100:.1f} cm",
        "controlled_point": {"body": TOOL_BODY, "offset_m": list(TOOL_OFFSET_M),
                             "mesh_end_face_centre_m": pincer_tip(TOOL_BODY).round(4).tolist()},
        "samples": count, "seed": seed, "target_box_relative_to_env_origin_m": TARGET_RANGES,
        "target_grid_points": len(targets_world),
        "collision_proxy": {"body_box_base_frame_m": BODY_BOX_B, "link_radius_m": LINK_RADIUS_M,
                            "note": "segments between joint origins and the 12.6 cm fingertip, not meshes"},
        "moving_arm_mass_kg": float(sum(m for name, m in masses.items() if name != "base_link")),
        "fraction_of_samples_holdable": float(holdable.mean()),
        "zero_pose": {"pincer_tip_in_base_frame_m": zero["pincer_tip"].round(4).tolist(),
                      "link6_in_base_frame_m": zero["link6"].round(4).tolist(),
                      "gravity_torque_nm": zero_torque.round(3).tolist()},
        "tip_offsets_link6_frame": tip_offsets(joints),
        "by_base_height": by_height,
        "measured_zero_action_stance": settled,
        "_samples": (points["pincer_tip"], holdable, clear_of_body(joints, q, ZERO_ACTION_BASE_HEIGHT_M)),
        "_settled_distances": settled_distances,
    }


def draw(result, path, baseline=None):
    """`baseline`: zero-action final errors (m) from a smoke run, overlaid on the model's distribution."""
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    import matplotlib.pyplot as plt

    tip, holdable, clear = result["_samples"]
    height = ZERO_ACTION_BASE_HEIGHT_M
    fig, (side, curve) = plt.subplots(1, 2, figsize=(11.5, 5.2), gridspec_kw={"width_ratios": [1.1, 1]})
    band = np.abs(tip[:, 1]) < 0.02
    ok = clear & holdable
    side.scatter(tip[band & ~ok, 0], tip[band & ~ok, 2], s=1.2, color="#c9ccd1", rasterized=True)
    side.scatter(tip[band & ok, 0], tip[band & ok, 2], s=1.2, color="#2a78d6", rasterized=True)
    (xl, xh), _, (zl, zh) = TARGET_RANGES
    base_x, _, base_z = ZERO_ACTION_BASE_OFFSET_M
    side.add_patch(plt.Rectangle((xl - base_x, zl - base_z), xh - xl, zh - zl, fill=False, lw=2, ec="#eb6834"))
    zero_tip, zero_link6 = result["zero_pose"]["pincer_tip_in_base_frame_m"], result["zero_pose"]["link6_in_base_frame_m"]
    side.plot(zero_tip[0], zero_tip[2], marker="*", ms=13, color="#1a1a1a", ls="none")
    side.plot(zero_link6[0], zero_link6[2], marker="o", ms=7, mfc="none", mec="#1a1a1a", ls="none")
    side.add_patch(plt.Rectangle((BODY_BOX_B[0][0], BODY_BOX_B[2][0]), BODY_BOX_B[0][1] - BODY_BOX_B[0][0],
                                 BODY_BOX_B[2][1] - BODY_BOX_B[2][0], color="#8a8f98", alpha=0.3))
    side.axhline(-height, color="#8a8f98", lw=1)
    side.text(0.62, -height + 0.01, "ground", fontsize=8, color="#5f6570", ha="right")
    side.set(xlabel="x forward in base frame (m)", ylabel="z up in base frame (m)", aspect="equal",
             title="Pincer tip positions, side slice |y| < 2 cm")
    side.legend(handles=[
        Line2D([], [], marker="o", ls="none", color="#2a78d6", label="clear of body proxy and holdable"),
        Line2D([], [], marker="o", ls="none", color="#c9ccd1", label="collides with body proxy or ground"),
        Patch(fill=False, ec="#eb6834", lw=2, label="target box, from the measured zero-action stance"),
        Line2D([], [], marker="*", ms=11, ls="none", color="#1a1a1a", label="pincer tip at the zero pose (episode start)"),
        Line2D([], [], marker="o", ms=7, mfc="none", mec="#1a1a1a", ls="none", label="Link6 origin at the zero pose"),
        Patch(color="#8a8f98", alpha=0.3, label="Go2 body proxy"),
    ], loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=2, fontsize=8, frameon=False)

    distances = result["_settled_distances"]
    bins = np.arange(0.0, 22.0, 1.0)
    for name, color, style, label in (("pincer_tip", "#2a78d6", "-", "pincer tip, model"),
                                      ("link6", "#8a8f98", "--", "Link6 origin, model (before)")):
        d = 100 * distances[name]
        share = 100 * float((d <= 100 * SUCCESS_RADIUS_M).mean())
        curve.hist(d, bins=bins, density=True, histtype="step", lw=2, color=color, ls=style,
                   label=f"{label}: {share:.1f}% within 5 cm")
    if baseline is not None:
        d = 100 * np.asarray(baseline)
        share = 100 * float((d <= 100 * SUCCESS_RADIUS_M).mean())
        curve.hist(d, bins=bins, density=True, color="#eb6834", alpha=0.35,
                   label=f"pincer tip, simulator, {len(d)} envs: {share:.1f}% within 5 cm")
    curve.axvline(100 * SUCCESS_RADIUS_M, color="#eb6834", ls="--", lw=1.5, label="5 cm success radius (G1a)")
    curve.set(xlabel="distance from the zero-action start to the target (cm)", ylabel="share of targets (per cm)",
              title="Zero actions: how far the controlled point starts from its target")
    curve.legend(loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=1, fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, required=True, help="Folder for workspace.json and d1_workspace.png")
    parser.add_argument("--samples", type=int, default=400_000)
    parser.add_argument("--baseline_smoke", type=Path,
                        help="smoke.json of a zero-action run with the same controlled point, to overlay its final errors")
    args = parser.parse_args()
    result = analyse(args.samples)
    args.out.mkdir(parents=True, exist_ok=True)
    baseline = None
    if args.baseline_smoke:
        baseline = json.loads(args.baseline_smoke.read_text())["final_position_error_m"]
        result["simulator_zero_action_baseline"] = {
            "source": str(args.baseline_smoke), "envs": len(baseline),
            "mean_m": float(np.mean(baseline)), "min_m": float(np.min(baseline)), "max_m": float(np.max(baseline)),
            "fraction_within_5cm": float((np.asarray(baseline) <= SUCCESS_RADIUS_M).mean()),
        }
    draw(result, args.out / "d1_workspace.png", baseline)
    result.pop("_samples")
    result.pop("_settled_distances")
    (args.out / "d1_workspace.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "by_base_height"}, indent=2))
    for row in result["by_base_height"]:
        print(row)


if __name__ == "__main__":
    main()
