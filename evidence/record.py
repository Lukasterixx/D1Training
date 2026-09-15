"""Copy a run's small artefacts into the week folder so the evidence is versioned.

`logs/` is gitignored and large, so the record keeps what a thesis claim needs:
run metadata and configuration, smoke/evaluation JSON, every scalar as a CSV,
and the final checkpoint's path and SHA-256 (the weights stay in `logs/`).
"""
from __future__ import annotations

import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shutil

from . import store, tfevents

COPY_SUFFIXES = {".json", ".yaml", ".yml", ".csv", ".txt", ".md"}
MAX_COPY_BYTES = 5 * 1024 * 1024


def run_started(run_dir: Path) -> datetime:
    """UTC stamp at the start of `run_position_only.py` directory names, else mtime."""
    match = re.match(r"(\d{8}T\d{6})", run_dir.name)
    if match:
        return datetime.strptime(match.group(1), "%Y%m%dT%H%M%S").replace(tzinfo=timezone.utc)
    match = re.match(r"(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})", run_dir.name)  # Isaac Lab's local-time names
    if match:
        return datetime.strptime(match.group(1), "%Y-%m-%d_%H-%M-%S").astimezone()
    return datetime.fromtimestamp(run_dir.stat().st_mtime, timezone.utc)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def latest_checkpoint(run_dir: Path) -> Path | None:
    checkpoints = [(int(m.group(1)), p) for p in run_dir.glob("model_*.pt")
                   if (m := re.fullmatch(r"model_(\d+)\.pt", p.name))]
    return max(checkpoints)[1] if checkpoints else None


def write_scalars_csv(scalars: dict[str, list[tuple[int, float, float]]], path: Path) -> list[str]:
    """Wide CSV: step, wall_time, then one column per tag. `*/time` tags are
    indexed by seconds rather than iteration, so they are left out."""
    tags = sorted(tag for tag in scalars if not tag.endswith("/time"))
    rows: dict[int, dict] = {}
    for tag in tags:
        for step, wall_time, value in scalars[tag]:
            row = rows.setdefault(step, {"wall_time": wall_time})
            row[tag] = value
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["step", "wall_time", *tags])
        for step in sorted(rows):
            row = rows[step]
            writer.writerow([step, f"{row['wall_time']:.3f}",
                             *(format(row[tag], ".7g") if tag in row else "" for tag in tags)])
    return tags


def summarise_series(points: list[tuple[int, float, float]]) -> dict:
    values = [value for _, _, value in points]
    tail = values[-10:]
    return {
        "last": values[-1], "last_step": points[-1][0], "mean_last_10": sum(tail) / len(tail),
        "min": min(values), "max": max(values), "points": len(values),
    }


def summarise_json(data: dict) -> dict:
    """Scalars pass through; numeric lists become count/mean/min/max; one level of named
    scalars or statistics ({name: value} or {name: {min, mean, max}}) is flattened to `key.name`."""
    summary = {}
    for key, value in data.items():
        if isinstance(value, (int, float, str, bool)) or value is None:
            summary[key] = value
        elif isinstance(value, list) and value and all(isinstance(v, (int, float)) for v in value):
            ordered = sorted(value)
            summary[key] = {"count": len(value), "mean": sum(value) / len(value), "min": ordered[0],
                            "median": ordered[len(ordered) // 2], "max": ordered[-1]}
        elif isinstance(value, dict):
            for name, item in value.items():
                if isinstance(item, (int, float)) and not isinstance(item, bool):
                    summary[f"{key}.{name}"] = item
                elif isinstance(item, dict) and item and all(isinstance(v, (int, float)) for v in item.values()):
                    summary[f"{key}.{name}"] = item
    return summary


def summarise_verify(data: dict) -> dict:
    checks = data.get("checks") or []
    return {"all_passed": data.get("all_passed"), "passed": sum(bool(c.get("passed")) for c in checks),
            "total": len(checks), "failed": data.get("failed") or [], "interpretation": data.get("interpretation")}


def find_existing(root: Path, run_id: str) -> Path | None:
    for folder in store.results_dir(root).glob(f"week_*/runs/{run_id}"):
        return folder
    return None


def record_run(run_dir: str | Path, week: int | None = None, title: str | None = None,
               notes: str | None = None, root: Path = store.ROOT) -> Path:
    source = Path(run_dir).resolve()
    if not source.is_dir():
        raise SystemExit(f"Not a directory: {source}")
    events = tfevents.event_files(source)
    meta = store.load_json(source / "run.json")
    if meta is None and not events:
        raise SystemExit(f"{source} has neither run.json nor TensorBoard event files.")
    cfg = store.load_config(root)

    started = run_started(source)
    if week is None:
        week = store.week_of(cfg, started.astimezone().date())
        if week is None:
            raise SystemExit(f"Run started {started:%Y-%m-%d}, outside the {cfg['weeks']}-week term; pass --week.")
    existing = find_existing(root, source.name)
    dest = store.week_dir(root, week) / "runs" / source.name
    if existing and existing.resolve() != dest.resolve():
        raise SystemExit(f"Already recorded at {existing.relative_to(root)}; pass --week "
                         f"{existing.parent.parent.name.split('_')[1].lstrip('0')} to refresh it.")
    dest.mkdir(parents=True, exist_ok=True)

    copied = []
    for path in sorted(source.iterdir()):
        if path.is_file() and path.suffix in COPY_SUFFIXES and path.stat().st_size <= MAX_COPY_BYTES:
            shutil.copy2(path, dest / path.name)
            copied.append(path.name)
    params = source / "params"  # Isaac Lab's own train scripts write their configs here.
    if params.is_dir():
        for path in sorted(params.iterdir()):
            if path.is_file() and path.suffix in COPY_SUFFIXES and path.stat().st_size <= MAX_COPY_BYTES:
                shutil.copy2(path, dest / f"params_{path.name}")
                copied.append(f"params_{path.name}")

    scalars = tfevents.read_scalars(source) if events else {}
    summary = {}
    if scalars:
        write_scalars_csv(scalars, dest / "scalars.csv")
        for tag in cfg.get("key_scalars", []):
            if scalars.get(tag):
                summary[tag] = summarise_series(scalars[tag])

    previous = store.load_json(dest / "record.json") or {}
    arguments = (meta or {}).get("arguments", {})
    checkpoint = latest_checkpoint(source)
    try:
        source_label = source.relative_to(root.resolve()).as_posix()
    except ValueError:
        source_label = str(source)
    record = {
        "id": source.name,
        "title": title or previous.get("title") or source.name,
        "notes": notes if notes is not None else previous.get("notes", ""),
        "week": week,
        "recorded_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "started_utc": started.astimezone(timezone.utc).isoformat(timespec="seconds"),
        "source_dir": source_label,
        "mode": (meta or {}).get("mode"),
        "seed": (meta or {}).get("seed"),
        "status": (meta or {}).get("status"),
        "error": (meta or {}).get("error"),
        "num_envs": arguments.get("num_envs"),
        "iterations": arguments.get("iterations"),
        "steps": arguments.get("steps"),
        "git_commit": (meta or {}).get("git_commit"),
        "git_dirty": bool((meta or {}).get("git_status")),
        "articulation_mass_kg": (meta or {}).get("articulation_mass_kg"),
        "last_iteration": max((points[-1][0] for tag, points in scalars.items() if not tag.endswith("/time")),
                              default=None),
        "final_checkpoint": {"file": checkpoint.name, "sha256": sha256(checkpoint)} if checkpoint else None,
        "scalar_summary": summary,
        "copied_files": copied,
    }
    # Playback self-tests carry the same kind of diagnostic summary as smoke runs.
    smoke = store.load_json(source / "smoke.json") or store.load_json(source / "selftest.json")
    if smoke:
        record["smoke_summary"] = summarise_json(smoke)
    verify = store.load_json(source / "verify.json")
    if verify:
        record["verify_summary"] = summarise_verify(verify)
    (dest / "record.json").write_text(json.dumps(record, indent=2) + "\n")
    return dest
