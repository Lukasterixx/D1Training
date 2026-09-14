"""Read the results tree into the plain dictionaries the dashboard serves.

Layout (see results/README.md):
    results/config.json            term start, week count, log roots, key scalars
    results/findings.md            numbered findings with status and evidence
    results/gates.md               G0-G3 decision gates
    results/week_NN/notes.md       the weekly log, with **Status:** and **Focus:** lines
    results/week_NN/runs/<id>/     record.json, copied run artefacts, scalars.csv
    results/week_NN/figures/       generated plots
    results/week_NN/screenshots/   screenshots added by hand (loose images in week_NN/ count too)
"""
from __future__ import annotations

import csv
from datetime import date, datetime, timedelta
import json
from pathlib import Path
import re
import time
import urllib.parse

from . import mdrender, tfevents

ROOT = Path(__file__).resolve().parent.parent
IMAGE_EXT = {".png", ".jpg", ".jpeg", ".gif", ".webp", ".svg"}
VIDEO_EXT = {".mp4", ".webm"}


def results_dir(root: Path = ROOT) -> Path:
    return root / "results"


def load_config(root: Path = ROOT) -> dict:
    return json.loads((results_dir(root) / "config.json").read_text())


def week_dir(root: Path, n: int) -> Path:
    return results_dir(root) / f"week_{n:02d}"


def week_range(cfg: dict, n: int) -> tuple[date, date]:
    start = date.fromisoformat(cfg["term_start"]) + timedelta(weeks=n - 1)
    return start, start + timedelta(days=6)


def week_of(cfg: dict, day: date) -> int | None:
    offset = (day - date.fromisoformat(cfg["term_start"])).days
    n = offset // 7 + 1
    return n if 1 <= n <= cfg["weeks"] else None


def format_range(start: date, end: date) -> str:
    if start.month == end.month:
        return f"{start.day}–{end.day} {end:%b %Y}"
    return f"{start.day} {start:%b} – {end.day} {end:%b %Y}"


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return ""


def field(text: str, name: str) -> str | None:
    """Value of a `**Name:** value` line, as used in notes and findings."""
    match = re.search(rf"^\s*(?:[-*]\s+)?\*\*{name}:\*\*\s*(.+?)\s*$", text, re.M)
    return match.group(1) if match else None


def resolver(root: Path, md_path: Path) -> mdrender.Resolver:
    """Map relative Markdown links onto dashboard routes and /files/ URLs."""
    base = md_path.parent

    def resolve(url: str, kind: str) -> str:
        if url.startswith("#") or re.match(r"^[a-zA-Z][\w+.-]*:", url):
            return url
        path, _, fragment = url.partition("#")
        target = (base / urllib.parse.unquote(path)).resolve()
        try:
            rel = target.relative_to(root.resolve()).as_posix()
        except ValueError:
            return "#"
        if kind == "link" and rel.endswith(".md"):
            week = re.fullmatch(r"results/week_(\d+)/notes\.md", rel)
            if week:
                return f"#/week/{int(week.group(1))}"
            if rel == "results/findings.md":
                return "#/findings"
            return f"#/doc/{rel}"
        return "/files/" + urllib.parse.quote(rel) + (f"#{fragment}" if fragment else "")

    return resolve


def render_file(root: Path, path: Path, header_fields: tuple[str, ...] | None = None) -> str:
    """Render a Markdown file. With `header_fields`, the page header shows the title
    and those fields, so they are dropped from the body."""
    text = read_text(path)
    if header_fields is not None:
        text = mdrender.strip_header(text, header_fields)
    return mdrender.render(text, resolver(root, path))


def caption(name: str) -> str:
    stem = Path(name).stem
    stem = re.sub(r"^\d+[_\-\s]+", "", stem)
    return re.sub(r"[_\-]+", " ", stem).strip() or name


def media(root: Path, folders: list[Path]) -> list[dict]:
    items = []
    for folder in folders:
        if not folder.is_dir():
            continue
        for path in sorted(folder.iterdir(), key=lambda p: p.name.lower()):
            suffix = path.suffix.lower()
            if path.is_file() and (suffix in IMAGE_EXT or suffix in VIDEO_EXT):
                rel = path.relative_to(root).as_posix()
                items.append({
                    "name": path.name, "path": rel, "url": "/files/" + urllib.parse.quote(rel),
                    "kind": "video" if suffix in VIDEO_EXT else "image", "caption": caption(path.name),
                    "modified": path.stat().st_mtime,
                })
    return items


def load_json(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def scalars_from_csv(path: Path) -> dict[str, list[list[float]]]:
    tags: dict[str, list[list[float]]] = {}
    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader, [])
        names = header[2:]
        for row in reader:
            step = int(float(row[0]))
            for name, value in zip(names, row[2:]):
                if value != "":
                    tags.setdefault(name, []).append([step, float(value)])
    return tags


def scalars_from_events(run_dir: Path) -> dict[str, list[list[float]]]:
    return {
        tag: [[step, value] for step, _, value in points]
        for tag, points in tfevents.read_scalars(run_dir).items() if not tag.endswith("/time")
    }


def run_entry(root: Path, folder: Path) -> dict:
    record = load_json(folder / "record.json") or {}
    files = sorted(p.name for p in folder.iterdir() if p.is_file())
    return {
        **record,
        "id": record.get("id", folder.name),
        "folder": folder.relative_to(root).as_posix(),
        "files": [{"name": name, "url": "/files/" + urllib.parse.quote(f"{folder.relative_to(root).as_posix()}/{name}")}
                  for name in files],
        "has_scalars": (folder / "scalars.csv").is_file(),
        "smoke": load_json(folder / "smoke.json"),
    }


def week_runs(root: Path, n: int) -> list[dict]:
    runs_dir = week_dir(root, n) / "runs"
    if not runs_dir.is_dir():
        return []
    folders = sorted(p for p in runs_dir.iterdir() if p.is_dir())
    return [run_entry(root, folder) for folder in folders]


def week_summary(root: Path, cfg: dict, n: int, today: date) -> dict:
    folder = week_dir(root, n)
    notes = read_text(folder / "notes.md")
    start, end = week_range(cfg, n)
    runs_dir = folder / "runs"
    return {
        "n": n, "dates": format_range(start, end), "start": start.isoformat(), "end": end.isoformat(),
        "status": (field(notes, "Status") or "not started").lower(),
        "focus": field(notes, "Focus") or "",
        "current": start <= today <= end,
        "runs": sum(1 for p in runs_dir.iterdir() if p.is_dir()) if runs_dir.is_dir() else 0,
        "screenshots": len(media(root, [folder / "screenshots", folder])),
        "figures": len(media(root, [folder / "figures"])),
    }


def week_detail(root: Path, cfg: dict, n: int, today: date) -> dict:
    folder = week_dir(root, n)
    return {
        **week_summary(root, cfg, n, today),
        "notes_html": render_file(root, folder / "notes.md", ("Status", "Focus")),
        "notes_path": (folder / "notes.md").relative_to(root).as_posix(),
        "folder": folder.relative_to(root).as_posix(),
        "run_list": week_runs(root, n),
        "figure_list": media(root, [folder / "figures"]),
        "screenshot_list": media(root, [folder / "screenshots", folder]),
    }


def parse_findings(text: str) -> list[dict]:
    """Entries are `### F-001 — Title` headings followed by `- **Status:**` etc."""
    entries = []
    parts = re.split(r"^###\s+", text, flags=re.M)[1:]
    for part in parts:
        heading, _, body = part.partition("\n")
        match = re.match(r"(F-\d+)\s*[—–-]+\s*(.+)", heading.strip())
        if not match:
            continue
        entries.append({
            "id": match.group(1), "title": re.sub(r"[`*_]", "", match.group(2)),
            "status": (field(body, "Status") or "").lower(),
            "week": field(body, "Week") or "", "date": field(body, "Date") or "",
            "anchor": mdrender.slugify(heading.strip()),
        })
    return entries


def parse_gates(text: str) -> list[dict]:
    """Rows of the gates table: `| G0 | Criterion | Status | Evidence |`."""
    gates = []
    for line in text.splitlines():
        if re.match(r"^\s*\|\s*\**G\d", line):
            cells = mdrender.split_row(line)
            clean = [re.sub(r"[`*_]", "", cell).strip() for cell in cells]
            gates.append({"id": clean[0], "name": clean[1] if len(clean) > 1 else "",
                          "status": clean[2].lower() if len(clean) > 2 else ""})
    return gates


def docs(root: Path) -> list[dict]:
    paths = sorted((root / "docs").glob("*.md")) + [root / "README.md", results_dir(root) / "README.md"]
    return [{"path": p.relative_to(root).as_posix(), "title": mdrender.title_of(read_text(p), p.stem)}
            for p in paths if p.is_file()]


def recorded_ids(root: Path) -> dict[str, int]:
    found = {}
    for runs in results_dir(root).glob("week_*/runs"):
        n = int(runs.parent.name.split("_")[1])
        for folder in runs.iterdir():
            if folder.is_dir():
                found[folder.name] = n
    return found


def live_runs(root: Path, cfg: dict) -> list[dict]:
    recorded = recorded_ids(root)
    runs = []
    for log_root in cfg.get("log_roots", []):
        base = root / log_root
        if not base.is_dir():
            continue
        for folder in base.iterdir():
            events = tfevents.event_files(folder) if folder.is_dir() else []
            meta = load_json(folder / "run.json") if folder.is_dir() else None
            if meta is None and not events:
                continue
            updated = max([p.stat().st_mtime for p in events + [folder / "run.json"] if p.exists()])
            arguments = (meta or {}).get("arguments", {})
            runs.append({
                "id": folder.name, "path": folder.relative_to(root).as_posix(),
                "mode": (meta or {}).get("mode"), "seed": (meta or {}).get("seed"),
                "status": (meta or {}).get("status", "unknown"), "error": (meta or {}).get("error"),
                "num_envs": arguments.get("num_envs"), "iterations": arguments.get("iterations"),
                "updated": updated, "has_scalars": bool(events), "recorded_week": recorded.get(folder.name),
            })
    return sorted(runs, key=lambda r: r["updated"], reverse=True)


def overview(root: Path, cfg: dict, today: date) -> dict:
    weeks = [week_summary(root, cfg, n, today) for n in range(1, cfg["weeks"] + 1)]
    return {
        "title": cfg.get("title", "Experimental record"), "term": cfg.get("term", ""),
        "today": today.isoformat(), "current_week": week_of(cfg, today), "weeks": weeks,
        "gates_html": render_file(root, results_dir(root) / "gates.md", ()),
        "gates": parse_gates(read_text(results_dir(root) / "gates.md")),
        "findings": parse_findings(read_text(results_dir(root) / "findings.md")),
        "docs": docs(root), "key_scalars": cfg.get("key_scalars", []),
        "scalar_labels": cfg.get("scalar_labels", {}), "log_roots": cfg.get("log_roots", []),
    }


def stamp(root: Path, cfg: dict) -> str:
    """Changes whenever anything the dashboard shows changes on disk."""
    paths = [root / "README.md"]
    for base in (results_dir(root), root / "docs"):
        paths += base.rglob("*")
    for log_root in cfg.get("log_roots", []):
        base = root / log_root
        paths += base.glob("*/run.json")
        paths += base.glob("*/events.out.tfevents.*")
    latest, sizes = 0.0, 0
    for path in paths:
        try:
            info = path.stat()
        except OSError:  # Deleted between listing and stat.
            continue
        latest, sizes = max(latest, info.st_mtime), sizes + info.st_size
    return f"{latest:.3f}:{len(paths)}:{sizes}"


def today_local() -> date:
    return datetime.fromtimestamp(time.time()).date()
