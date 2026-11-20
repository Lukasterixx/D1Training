"""S1's key family and its frozen sets (`docs/tool_paper_plan.md`, H4: tool conditioning on the key insertion).

A key is its tip's reach beyond the jaw centre along the jaws' axis, its shaft's axis off the jaws' (Link6 x, y), its
give in the jaws (m/N, every axis) and its mass (`keylock.set_tool`, `keyinsert_env._set_tools`). The shaft, wards and
tip are v4's, so the keyway fits every key, and only the tool varies. Only v4's reach is measured (and F-224's 2 mm
offset on the dog); the other values are a designed family, not keys that exist.

**The sets** live in a versioned manifest (`results/manifests/keyinsert_tools_v2.json`, written once by
`python -m unifp_train.keytools --write`), every lock's draws written out, so a change to this code cannot move them:

- *train*: three keys spanning the range (`TRAIN`), one of them v4; the *specialist* trains on v4 alone.
- *validation*: the train keys on 48 new locks (`VALIDATION_SEED`). Checkpoints are selected here.
- *test*: six held-out keys (`HELD_OUT`) -- two in the gaps of the training range, two beyond its ends, two that mix a
  train key's geometry with another's dynamics -- on 48 other new locks (`TEST_SEED`). Scored once.
- *test_train*: the train keys on the test locks (the in-distribution reference on the same locks).
- *wrong*: a train key (T-long) whose believed key -- the layer's tip and P4's descriptor -- is off by a stated amount.

A lock is drawn from training's own distributions (`keyinsert_env._draw_insert`: the estimate off to 3.5 mm across,
8 mm along, 4 deg in roll, 0.8 deg in tilt; the lock's play, edge breaks, friction and the grip's slip), the
force-aware press at `keylock.EVAL_PRESS_N`, the box placed within the key hold's frozen grid (`leg_cfg.KEYHOLD_EVAL_*`)
on the same new stream. None of `keyinsert_eval`'s 96 locks is reused.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import torch

from . import keylock, leg_cfg

MANIFEST = Path(__file__).resolve().parents[1] / "results" / "manifests" / "keyinsert_tools_v2.json"
#: v1 (superseded 2026-11-13 14:53, its masses past the D1; its test set never scored).
MANIFEST_V1 = MANIFEST.with_name("keyinsert_tools_v1.json")
VALIDATION_SEED = 20261116
TEST_SEED = 20261117
LOCKS_PER_SET = 48

#: Keys: reach mm, shaft offset x and y mm (Link6: x along the bar, y = -t), compliance mm/N, mass g.
#: v2 (2026-11-13 14:53): v1's masses (60-250 g) were past what the D1 holds at the lock -- Joint2 already carries 2.66 of
#: its 3.33 N·m there, T-long's 150 g added 0.74 and H-high's 250 g 1.23, so the parent's arm swung 12-19 cm at the
#: spawn and the layer felt the face 29-89 mm short of it -- and its compliance (to 0.25 mm/N, 3 mm of give at 15 N) was
#: a very soft grip. v2 keeps v1's reaches, offsets and locks; masses are 5-60 g (keys, not tools: Joint2 at most ~90%
#: of its limit standing still), compliance 0-0.12 mm/N.
KEYS = {
    "v4": (keylock.TIP_FROM_JAW_M * 1000.0, 0.0, 0.0, 0.0, keylock.NOMINAL_MASS_KG * 1000.0),
    "T-short": (24.0, 1.5, 0.0, 0.04, 25.0),
    "T-long": (45.0, 0.0, -1.5, 0.08, 40.0),
    "H-gap-a": (28.0, 0.75, -0.75, 0.02, 15.0),
    "H-gap-b": (38.0, -0.75, -0.75, 0.06, 32.0),
    "H-low": (19.0, 2.5, 0.0, 0.0, 5.0),
    "H-high": (55.0, 0.0, -2.5, 0.12, 60.0),
    "H-mix-a": (45.0, 1.5, 0.0, 0.0, 10.0),
    "H-mix-b": (24.0, 0.0, -1.5, 0.08, 40.0),
}
TRAIN = ("T-short", "v4", "T-long")
SPECIALIST = ("v4",)
HELD_OUT = ("H-gap-a", "H-gap-b", "H-low", "H-high", "H-mix-a", "H-mix-b")
#: The wrong descriptors, on T-long: (name, true key, believed key as KEYS' tuple).
WRONG = (
    ("W-geom", "T-long", (42.0, 0.0, 0.0, 0.08, 40.0)),       # reach believed 3 mm short, the 1.5 mm offset missed
    ("W-dyn", "T-long", (45.0, 0.0, -1.5, 0.0, 10.0)),        # compliance and mass believed v4's
    ("W-all", "T-long", KEYS["v4"]),                          # believed the v4 key
)


#: `key_si`'s fields as the plan's columns (`tool_<short>` true, `bel_<short>` believed; `keyinsert_env._set_tools`).
_COLUMNS = {"reach_m": "reach", "ox_m": "ox", "oy_m": "oy", "compliance_m_per_n": "compliance", "mass_kg": "mass"}


def key_si(values) -> dict:
    """KEYS' tuple (mm, mm, mm, mm/N, g) in SI."""
    reach, ox, oy, comp, mass = values
    return {"reach_m": reach / 1000.0, "ox_m": ox / 1000.0, "oy_m": oy / 1000.0,
            "compliance_m_per_n": comp / 1000.0, "mass_kg": mass / 1000.0}


def tool_tuple(name: str) -> tuple:
    """A key as `keyinsert_env`'s `cfg.tools` entry: (reach m, offset x m, offset y m, compliance m/N, mass kg)."""
    k = key_si(KEYS[name])
    return (k["reach_m"], k["ox_m"], k["oy_m"], k["compliance_m_per_n"], k["mass_kg"])


def draw_locks(seed: int, n: int = LOCKS_PER_SET) -> list[dict]:
    """`n` locks from training's distributions on a new stream (`keyinsert_env._draw_insert`, `ERR_*`)."""
    ke = keylock
    g = torch.Generator().manual_seed(seed)
    u = lambda: float(torch.rand(1, generator=g))
    span = lambda r: r[0] + u() * (r[1] - r[0])
    sym = lambda a: (2 * u() - 1) * a
    c = leg_cfg
    locks = []
    for i in range(n):
        locks.append({
            "lock": i,
            "door_ahead": span((min(c.KEYHOLD_EVAL_DOOR_AHEAD_M), max(c.KEYHOLD_EVAL_DOOR_AHEAD_M))),
            "axis_side": span((min(c.KEYHOLD_EVAL_AXIS_SIDE_M), max(c.KEYHOLD_EVAL_AXIS_SIDE_M))),
            "height": span((min(c.KEYHOLD_EVAL_AXIS_HEIGHT_M), max(c.KEYHOLD_EVAL_AXIS_HEIGHT_M))),
            "door_yaw": sym(c.KEYHOLD_EVAL_DOOR_YAW_RAD),
            "lat_r": ke.ERR_LATERAL_M * math.sqrt(u()), "lat_dir": 2 * math.pi * u(),
            "depth": sym(ke.ERR_DEPTH_M), "roll": sym(ke.ERR_ROLL_RAD),
            "tilt_u": sym(ke.ERR_TILT_RAD), "tilt_t": sym(ke.ERR_TILT_RAD),
            "play": span(keylock.PLAY_M), "lock_chamfer": span(keylock.LOCK_CHAMFER_M),
            "key_chamfer": span(keylock.KEY_CHAMFER_M), "mu": span(keylock.MU), "grip_slip": span(keylock.GRIP_SLIP_N),
            "start_off": span(ke.START_OFF_M), "press": keylock.EVAL_PRESS_N,
        })
    return [{k: (round(v, 7) if isinstance(v, float) else v) for k, v in lock.items()} for lock in locks]


def manifest() -> dict:
    keys = {name: {"mm_mm_mm_mm_per_n_g": list(v), **key_si(v)} for name, v in KEYS.items()}
    entry = lambda name: {"name": name, "true": name, "believed": name}
    return {
        "name": "keyinsert_tools_v2",
        "version": 2,
        "supersedes": "keyinsert_tools_v1 (masses 60-250 g past the D1's Joint2; compliance to 0.25 mm/N); the same locks",
        "written": "2026-11-13",
        "plan": "docs/tool_paper_plan.md S1 (H4); code unifp_train/keytools.py",
        "key_fields": "reach (jaw centre to tip along Link6 z), shaft offset x, y (Link6), compliance (every axis), mass",
        "measured": "v4's reach only (geometry.KEY_ORIGIN_LINK6 + the 10 mm shaft); the rest is a designed family",
        "keys": keys,
        "train": list(TRAIN),
        "specialist": list(SPECIALIST),
        "descriptor": {"offset_m": keylock.DESC_OFFSET_M, "reach_m": keylock.DESC_REACH_M,
                       "compliance_m_per_n": keylock.DESC_COMPLIANCE_M_PER_N, "mass_kg": keylock.DESC_MASS_KG,
                       "nominal_mass_kg": keylock.NOMINAL_MASS_KG, "width": keylock.NUM_DESCRIPTOR},
        "lock_units": "m, rad, N; press the force-aware layer's (N)",
        "locks": {"validation": {"seed": VALIDATION_SEED, "rows": draw_locks(VALIDATION_SEED)},
                  "test": {"seed": TEST_SEED, "rows": draw_locks(TEST_SEED)}},
        "sets": {
            "validation": {"locks": "validation", "keys": [entry(k) for k in TRAIN]},
            "test": {"locks": "test", "keys": [entry(k) for k in HELD_OUT]},
            "test_train": {"locks": "test", "keys": [entry(k) for k in TRAIN]},
            "wrong": {"locks": "test", "keys": [{"name": n, "true": t, "believed": list(b)} for n, t, b in WRONG]},
        },
    }


#: S2 (H3, twins from a common parent): the key the twins train on -- v4, rigid and massless, as every insertion run before
#: S1 -- and two geometry-only reach shifts; new locks on their own streams (48 to select on, 96 to score once).
TWINS_MANIFEST = MANIFEST.with_name("keyinsert_twins_v1.json")
TWINS_KEYS = {"v4-legacy": (keylock.TIP_FROM_JAW_M * 1000.0, 0.0, 0.0, 0.0, 0.0),
              "reach-45": (45.0, 0.0, 0.0, 0.0, 0.0), "reach-24": (24.0, 0.0, 0.0, 0.0, 0.0)}
TWINS_VALIDATION_SEED = 20261120
TWINS_TEST_SEED = 20261121


def twins_manifest() -> dict:
    entry = lambda name: {"name": name, "true": name, "believed": name}
    return {
        "name": "keyinsert_twins_v1", "version": 1, "written": "2026-11-13",
        "plan": "docs/tool_paper_plan.md S2 (H3); code unifp_train/keytools.py (twins_manifest)",
        "key_fields": "reach (jaw centre to tip along Link6 z), shaft offset x, y (Link6), compliance (every axis), mass",
        "measured": "v4's reach only; the reach shifts are geometry alone (rigid, massless, on the shaft's axis)",
        "keys": {name: {"mm_mm_mm_mm_per_n_g": list(v), **key_si(v)} for name, v in TWINS_KEYS.items()},
        "lock_units": "m, rad, N; press the force-aware layer's (N)",
        "locks": {"validation": {"seed": TWINS_VALIDATION_SEED, "rows": draw_locks(TWINS_VALIDATION_SEED, 48)},
                  "test": {"seed": TWINS_TEST_SEED, "rows": draw_locks(TWINS_TEST_SEED, 96)}},
        "sets": {"validation": {"locks": "validation", "keys": [entry("v4-legacy")]},
                 "test": {"locks": "test", "keys": [entry("v4-legacy")]},
                 "test_reach45": {"locks": "test", "keys": [entry("reach-45")]},
                 "test_reach24": {"locks": "test", "keys": [entry("reach-24")]}},
    }


#: S1b (F-287's redesign): training keys drawn from a continuous region -- reach 22-47 mm less a held-out band 33-40,
#: the shaft's offset within 1.8 mm, compliance 0-0.09 mm/N, mass 8-45 g -- so the descriptor is a scale, not three
#: labels; held-out keys in the band and past the region's ends; new locks. SI, `keyinsert_env.cfg.tool_ranges`' order.
S1B_MANIFEST = MANIFEST.with_name("keyinsert_tools_v3.json")
S1B_RANGES = (0.022, 0.047, 0.033, 0.040, 0.0018, 0.09e-3, 0.008, 0.045)
S1B_VALIDATION_SEED = 20261124
S1B_TEST_SEED = 20261125
S1B_KEY_SEED = 20261126
S1B_HELD_OUT = {
    "G-35": (35.5, 0.9, -0.6, 0.05, 25.0),       # in the held-out reach band
    "G-38": (38.0, -0.6, 0.9, 0.03, 35.0),       # in the band
    "X-low": (19.0, 1.2, 0.0, 0.02, 12.0),       # reach below the region
    "X-high": (52.0, 0.0, -1.2, 0.06, 30.0),     # reach above it, the rest inside
    "X-dyn": (30.0, 0.0, 0.0, 0.12, 58.0),       # compliance and mass above it, the geometry inside
    "X-all": (55.0, 0.0, -2.5, 0.12, 60.0),      # past every end (v2's H-high)
}


def s1b_draw_keys(n: int, seed: int) -> dict:
    """`n` keys drawn from S1B_RANGES' region (as training draws them) on their own stream, KEYS' units."""
    g = torch.Generator().manual_seed(seed)
    u = lambda: float(torch.rand(1, generator=g))
    r_lo, r_hi, gap_lo, gap_hi, off_max, comp_hi, m_lo, m_hi = S1B_RANGES
    keys = {}
    for i in range(n):
        x = r_lo + u() * ((r_hi - r_lo) - (gap_hi - gap_lo))
        reach = x + (gap_hi - gap_lo) if x >= gap_lo else x
        ang, rad = 2 * math.pi * u(), off_max * math.sqrt(u())
        keys[f"R-{i}"] = (round(reach * 1000, 3), round(rad * math.cos(ang) * 1000, 3), round(rad * math.sin(ang) * 1000, 3),
                          round(comp_hi * u() * 1000, 4), round((m_lo + (m_hi - m_lo) * u()) * 1000, 2))
    return keys


def s1b_manifest() -> dict:
    entry = lambda name: {"name": name, "true": name, "believed": name}
    drawn = s1b_draw_keys(9, S1B_KEY_SEED)
    val_keys, train_ref = list(drawn)[:6], list(drawn)[6:]
    keys = {"v4": KEYS["v4"], "T-long": KEYS["T-long"], **drawn, **S1B_HELD_OUT}
    return {
        "name": "keyinsert_tools_v3", "version": 3, "written": "2026-11-13",
        "plan": "docs/tool_paper_plan.md S1, redesigned after the pilot (F-287); code unifp_train/keytools.py (s1b_manifest)",
        "key_fields": "reach (jaw centre to tip along Link6 z), shaft offset x, y (Link6), compliance (every axis), mass",
        "measured": "v4's reach only; the rest is a designed family",
        "train_ranges": {"order": "reach lo, hi, held-out band lo, hi, offset radius max, compliance max, mass lo, hi (SI)",
                         "values": list(S1B_RANGES)},
        "specialist": ["v4"],
        "keys": {name: {"mm_mm_mm_mm_per_n_g": list(v), **key_si(v)} for name, v in keys.items()},
        "descriptor": {"offset_m": keylock.DESC_OFFSET_M, "reach_m": keylock.DESC_REACH_M,
                       "compliance_m_per_n": keylock.DESC_COMPLIANCE_M_PER_N, "mass_kg": keylock.DESC_MASS_KG,
                       "nominal_mass_kg": keylock.NOMINAL_MASS_KG, "width": keylock.NUM_DESCRIPTOR},
        "lock_units": "m, rad, N; press the force-aware layer's (N)",
        "locks": {"validation": {"seed": S1B_VALIDATION_SEED, "rows": draw_locks(S1B_VALIDATION_SEED, 24)},
                  "test": {"seed": S1B_TEST_SEED, "rows": draw_locks(S1B_TEST_SEED, 48)}},
        "sets": {
            "validation": {"locks": "validation", "keys": [entry(k) for k in val_keys]},
            "test": {"locks": "test", "keys": [entry(k) for k in S1B_HELD_OUT]},
            "test_train": {"locks": "test", "keys": [entry(k) for k in train_ref]},
            "wrong": {"locks": "test", "keys": [{"name": n, "true": t, "believed": list(b)} for n, t, b in WRONG]},
        },
    }


#: S1c (2026-11-18): the layer's seated test (`keylock.InsertLayer.seat_test`) compared on locks nobody has scored -- v3's
#: test locks were spent by S1b with the old test. The same six held-out keys, and v4 (the dog's key) as its own set, on
#: 48 locks drawn on a new stream. Scored once, old against located, after the located test was set on v3's validation.
S1C_MANIFEST = MANIFEST.with_name("keyinsert_tools_v4.json")
S1C_TEST_SEED = 20261129


def s1c_manifest() -> dict:
    entry = lambda name: {"name": name, "true": name, "believed": name}
    v3 = s1b_manifest()
    keys = {"v4": KEYS["v4"], **S1B_HELD_OUT}
    return {
        "name": "keyinsert_tools_v4", "version": 4, "written": "2026-11-18",
        "plan": "docs/tool_paper_plan.md S1c (the layer's seated test); code unifp_train/keytools.py (s1c_manifest)",
        "key_fields": v3["key_fields"], "measured": "v4's reach only; the rest is a designed family",
        "train_ranges": v3["train_ranges"], "specialist": ["v4"],
        "keys": {name: {"mm_mm_mm_mm_per_n_g": list(v), **key_si(v)} for name, v in keys.items()},
        "descriptor": v3["descriptor"], "lock_units": v3["lock_units"],
        "locks": {"test": {"seed": S1C_TEST_SEED, "rows": draw_locks(S1C_TEST_SEED, 48)}},
        "sets": {"test": {"locks": "test", "keys": [entry(k) for k in S1B_HELD_OUT]},
                 "test_v4": {"locks": "test", "keys": [entry("v4")]}},
    }


#: Item 1's insertion retrain (2026-11-19, the week 10 log): the v4 key alone, as the twins' sets have it, on locks no run
#: has scored (the twins' validation was S2's selection set and its test S2's read; the frozen 96 are spent for selection).
KI7_MANIFEST = MANIFEST.with_name("keyinsert_ki7_v1.json")
KI7_VALIDATION_SEED = 20261203
KI7_TEST_SEED = 20261204


def ki7_manifest() -> dict:
    out = twins_manifest()
    out.update({"name": "keyinsert_ki7_v1", "written": "2026-11-19",
                "plan": "the week 10 log, item 1 (the insertion retrain); code unifp_train/keytools.py (ki7_manifest)",
                "keys": {"v4-legacy": out["keys"]["v4-legacy"]},
                "locks": {"validation": {"seed": KI7_VALIDATION_SEED, "rows": draw_locks(KI7_VALIDATION_SEED, 48)},
                          "test": {"seed": KI7_TEST_SEED, "rows": draw_locks(KI7_TEST_SEED, 96)}}})
    out["sets"] = {"validation": out["sets"]["validation"], "test": out["sets"]["test"]}
    return out


#: Item 3's follow-up (2026-11-19, the week 10 log "the hold from the insertion's handovers"): the v4 key on new locks.
#: `spawn` is scored by the insertion only to make the hold's training stances (never a selection or a result); the chain
#: (insertion, handover, hold and turn) selects on `validation` and is read once on `test`.
CHAIN_MANIFEST = MANIFEST.with_name("keyinsert_chain_v1.json")
CHAIN_SPAWN_SEED = 20261205
CHAIN_VALIDATION_SEED = 20261206
CHAIN_TEST_SEED = 20261207


def chain_manifest() -> dict:
    out = ki7_manifest()
    out.update({"name": "keyinsert_chain_v1", "written": "2026-11-19",
                "plan": "the week 10 log, item 3's follow-up (the hold from the insertion's handovers); code "
                        "unifp_train/keytools.py (chain_manifest)",
                "locks": {"spawn": {"seed": CHAIN_SPAWN_SEED, "rows": draw_locks(CHAIN_SPAWN_SEED, 96)},
                          "validation": {"seed": CHAIN_VALIDATION_SEED, "rows": draw_locks(CHAIN_VALIDATION_SEED, 48)},
                          "test": {"seed": CHAIN_TEST_SEED, "rows": draw_locks(CHAIN_TEST_SEED, 96)}}})
    out["sets"] = {**out["sets"], "spawn": {"locks": "spawn", "keys": out["sets"]["validation"]["keys"]}}
    return out


def load(path: str | Path = MANIFEST) -> dict:
    return json.loads(Path(path).read_text())


def build_plan(device, set_name: str, path: str | Path = MANIFEST,
               overrides: dict | None = None) -> tuple[dict, list[dict]]:
    """A frozen tool set as `keyinsert_eval.build_plan`'s plan and rows: every key of the set on every lock (key-major),
    the box from the lock's draws, the true key in `tool_*` and the believed one in `bel_*`. `overrides`
    (`keyinsert_eval.OVERRIDES`' names; S2's lock shifts) replace a column for every lock or scale the estimate's errors,
    on top of the frozen draws; the rows record them."""
    from . import keyinsert_eval, leg_eval

    overrides = overrides or {}
    for name in overrides:
        if name not in keyinsert_eval.OVERRIDES:
            raise ValueError(f"--insert_eval_set: {name!r} is not one of {sorted(keyinsert_eval.OVERRIDES)}")
    man = load(path)
    if set_name not in man["sets"]:
        raise ValueError(f"tool set {set_name!r} is not one of {sorted(man['sets'])} in {path}")
    spec = man["sets"][set_name]
    locks = man["locks"][spec["locks"]]["rows"]
    si = lambda key: key_si(man["keys"][key]["mm_mm_mm_mm_per_n_g"] if isinstance(key, str) else key)
    rows, cols = [], {}
    for index, entry in enumerate(spec["keys"]):
        true, bel = si(entry["true"]), si(entry["believed"])
        for lock in locks:
            row = {"tool": entry["name"], "tool_true": entry["true"],
                   "tool_believed": entry["believed"] if isinstance(entry["believed"], str) else "stated",
                   "tool_index": index, "lock": lock["lock"], "set": set_name,
                   "lateral_mm": round(lock["lat_r"] * 1000.0, 3), "direction_deg": round(math.degrees(lock["lat_dir"]), 2),
                   "depth_mm": round(lock["depth"] * 1000.0, 3), "roll_deg": round(math.degrees(lock["roll"]), 3)}
            rows.append(row)
            draw = dict(lock)
            for field, short in _COLUMNS.items():
                draw["tool_" + short], draw["bel_" + short] = true[field], bel[field]
            draw["tool_index"] = float(index)
            for k, v in draw.items():
                cols.setdefault(k, []).append(float(v))
    n = len(rows)
    insert_keys = ("lat_r", "lat_dir", "depth", "roll", "tilt_u", "tilt_t", "play", "lock_chamfer", "key_chamfer", "mu",
                   "grip_slip", "start_off", "press", "tool_reach", "tool_ox", "tool_oy", "tool_compliance", "tool_mass",
                   "bel_reach", "bel_ox", "bel_oy", "bel_compliance", "bel_mass", "tool_index")
    insert = {k: torch.tensor(cols[k], dtype=torch.float32) for k in insert_keys}
    for name, value in overrides.items():
        if name in keyinsert_eval._PLAN_KEY:
            insert[keyinsert_eval._PLAN_KEY[name]] = torch.full((n,), float(value) * keyinsert_eval.OVERRIDES[name])
        else:
            column = {"lateral_scale": "lat_r", "depth_scale": "depth", "roll_scale": "roll"}[name]
            insert[column] = insert[column] * float(value)
            unit = {"lateral_scale": "lateral_mm", "depth_scale": "depth_mm", "roll_scale": "roll_deg"}[name]
            for row in rows:
                row[unit] = round(row[unit] * float(value), 4)
    for row in rows:
        row["overrides"] = dict(overrides)
    # The box: the key plan's columns (its turning ones at the frozen set's first row: the foot never turns here), the
    # placement and the door's yaw the lock's, the key hold's estimate errors zero (the insertion draws its own).
    key_plan, _ = leg_eval.build_key_plan("cpu")
    handles = {k: v[:1].repeat(n) for k, v in key_plan["handles"].items()}
    for k in ("door_ahead", "axis_side", "height", "door_yaw"):
        handles[k] = torch.tensor(cols[k], dtype=torch.float32)
    for k in ("err_side", "err_up", "err_depth", "err_rating"):
        handles[k] = torch.zeros(n)
    for i, row in enumerate(rows):
        row.update({"door_ahead": round(cols["door_ahead"][i], 4), "axis_side": round(cols["axis_side"][i], 4),
                    "height": round(cols["height"][i], 4), "door_yaw": round(cols["door_yaw"][i], 4)})
    return {"handles": {k: v.to(device) for k, v in handles.items()},
            "insert": {k: v.to(device) for k, v in insert.items()}}, rows


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--write", action="store_true", help=f"write {MANIFEST} (refuses to overwrite)")
    parser.add_argument("--check", action="store_true", help="rebuild the manifest and compare it with the file")
    parser.add_argument("--twins", action="store_true", help=f"--write / --check S2's {TWINS_MANIFEST.name} instead")
    parser.add_argument("--s1b", action="store_true", help=f"--write / --check S1b's {S1B_MANIFEST.name} instead")
    parser.add_argument("--s1c", action="store_true", help=f"--write / --check S1c's {S1C_MANIFEST.name} instead")
    parser.add_argument("--ki7", action="store_true", help=f"--write / --check the insertion retrain's {KI7_MANIFEST.name} instead")
    parser.add_argument("--chain", action="store_true", help=f"--write / --check item 3's {CHAIN_MANIFEST.name} instead")
    args = parser.parse_args()
    path, build = ((TWINS_MANIFEST, twins_manifest) if args.twins else (S1B_MANIFEST, s1b_manifest) if args.s1b
                   else (S1C_MANIFEST, s1c_manifest) if args.s1c else (KI7_MANIFEST, ki7_manifest) if args.ki7
                   else (CHAIN_MANIFEST, chain_manifest) if args.chain else (MANIFEST, manifest))
    if args.write:
        if path.exists():
            raise SystemExit(f"{path} exists: a frozen set is never rewritten; bump the version instead")
        path.write_text(json.dumps(build(), indent=1) + "\n")
        print(f"wrote {path}")
    if args.check:
        same = load(path) == json.loads(json.dumps(build()))
        print(f"{path.name} matches the code" if same else f"{path.name} DIFFERS from the code")
        raise SystemExit(0 if same else 1)
