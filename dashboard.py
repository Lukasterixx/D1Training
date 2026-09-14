#!/usr/bin/env python3
"""Thesis B experimental record: serve the results dashboard, or record a run into it.

    ./dashboard.py                          # serve on http://localhost:8765 and open a browser
    ./dashboard.py serve --port 9000 --no-browser
    ./dashboard.py record logs/position_only/<run> --title "64-env PPO pilot, seed 42"

Standard library only: runs from the system Python or the Isaac environment.
See results/README.md for how the record is organised.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from evidence import record, server  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command")
    serve = commands.add_parser("serve", help="Serve the dashboard (default).")
    serve.add_argument("--port", type=int, default=8765)
    serve.add_argument("--no-browser", action="store_true")
    rec = commands.add_parser("record", help="Copy a run's artefacts and scalars into its week folder.")
    rec.add_argument("run_dir")
    rec.add_argument("--week", type=int, help="Defaults to the week the run started in.")
    rec.add_argument("--title", help="Short human description; kept on re-record if omitted.")
    rec.add_argument("--notes", help="One-line context; kept on re-record if omitted.")
    args = parser.parse_args()

    if args.command == "record":
        dest = record.record_run(args.run_dir, week=args.week, title=args.title, notes=args.notes, root=ROOT)
        data = json.loads((dest / "record.json").read_text())
        print(f"Recorded {data['id']} -> {dest.relative_to(ROOT)}")
        print(f"  dashboard: #/week/{data['week']}/run/{data['id']}")
        print(f"  status={data['status']} mode={data['mode']} seed={data['seed']} "
              f"last_iteration={data['last_iteration']} files={len(data['copied_files'])}")
        for tag, stats in data["scalar_summary"].items():
            print(f"  {tag}: last={stats['last']:.4g} mean_last_10={stats['mean_last_10']:.4g}")
        return 0
    server.serve(ROOT, "127.0.0.1", getattr(args, "port", 8765), not getattr(args, "no_browser", False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
