"""Static ceiling for turning the combiner lever by pushing straight down from above with the arm locked,
the standing Go2's legs supplying the stroke (crouch). CPU model only (press_capacity: gravity + J^T F
against the published joint limits, base level). Week 2 log, 2026-09-21; F-077.

    PYTHONPATH=. python results/week_02/figures/combiner_crouch_push_study.py    # in env_isaaclab, ~10 min
"""
import math, random
import numpy as np
from dataclasses import replace
from demos.combiner.tests.test_turn import JOINTS, LINKS, box_in_base, BASE_W as LYING_W
from demos.combiner.geometry import GEOMETRY, Placement, sample_placement
from demos.combiner.pull import Handle, PullParams
from demos.combiner.press import _solve, _seeds, press_capacity
from demos.cup.pick_demo.grasp import JAW_CENTRE_LINK6, proxy_margin
from demos.cup.pick_demo.camera import invert

g = GEOMETRY
STAND_H = 0.2737                       # ZERO_ACTION_BASE_OFFSET_M: the standing base height
D = 0.065                              # push this far out from the spindle: 52 deg before running off the 105 mm lever
STROKE = D * math.tan(math.radians(52))
FOOT = ((-0.40, 0.40), (-0.17, 0.17))  # the dog's trunk, head and legs from above (m, base frame), generous

def base_w(h):
    w = np.eye(4); w[2, 3] = h; return w

def box_corners_b(box_b):
    # the enclosure footprint plus the lever out in front of the door
    xs = (-g.depth / 2, g.door_x + g.handle_projection + 0.02)
    ys = (-g.width / 2, g.width / 2)
    pts = np.array([[x, y, 0.0, 1.0] for x in xs for y in ys])
    return (box_b @ pts.T).T[:, :2]

def clear_of_dog(box_b):
    # separating-axis test between the dog's footprint rectangle and the box footprint (a rotated rectangle)
    c = box_corners_b(box_b)
    dog = np.array([[x, y] for x in FOOT[0] for y in FOOT[1]])
    axes = [np.array([1.0, 0]), np.array([0, 1.0])]
    e = box_b[:2, 0], box_b[:2, 1]
    axes += [v / np.linalg.norm(v) for v in e]
    for a in axes:
        p, q = c @ a, dog @ a
        if p.max() < q.min() or q.max() < p.min():
            return True
    return False

def ceiling(box_b, roll, push=np.array([0.0, 0.0, -1.0])):
    h = Handle(box_b, replace(PullParams(), pitch_deg=90.0, roll=roll, radius_m=D))
    pos, rot = h.grasp(0.0, 0.0)
    q = _solve(JOINTS, LINKS, pos, rot, _seeds(pos, []), np.asarray(JAW_CENTRE_LINK6))
    if q is None or proxy_margin(JOINTS, q) < 0.0:
        return None
    c = press_capacity(JOINTS, LINKS, q, pos, push)
    return c["force_n"] * D, c["limiting_joint"], pos

rng = random.Random(4)
rows = []
for _ in range(4000):
    # box anywhere within 0.9 m of the base, any yaw facing roughly toward the dog
    r, bearing = rng.uniform(0.2, 0.9), rng.uniform(-math.pi, math.pi)
    x, y = r * math.cos(bearing), r * math.sin(bearing)
    yaw = bearing + math.pi + rng.uniform(-0.6, 0.6)          # door (+X) roughly back toward the dog
    box_w = np.eye(4); box_w[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    box_w[:3, 3] = (x, y, 0.0)
    box_b = invert(base_w(STAND_H)) @ box_w
    if not clear_of_dog(box_b):
        continue
    best = None
    for roll in (1, -1):
        c = ceiling(box_b, roll)
        if c and (best is None or c[0] > best[0]):
            best = c
    if best:
        rows.append((best[0], best[1], best[2]))
print(f"standing (base {STAND_H} m), push down from above {100*D:.1f} cm out, stroke {100*STROKE:.1f} cm for 52 deg")
print(f"  placements clear of the dog where it solves: {len(rows)}")
t = np.array([r[0] for r in rows])
for q in (50, 75, 90, 100):
    print(f"  ceiling p{q}: {np.percentile(t, q):.2f} N·m")
from collections import Counter
print("  limiting joint:", Counter(r[1] for r in rows).most_common())
top = sorted(rows, key=lambda r: -r[0])[:5]
for tq, j, pos in top:
    print(f"   best {tq:.2f} N·m ({j}) lever point in base frame {np.round(pos, 3)}  horizontal {np.linalg.norm(pos[:2]):.2f} m from mount")

# The same push from the lying pose for comparison
lying = []
for _ in range(300):
    box_b = box_in_base(sample_placement(rng))
    for roll in (1, -1):
        c = ceiling(box_b, roll)
        if c: lying.append(c[0])
print(f"lying, same top-down push: solves at {len(lying)} of 600 grasp tries")
