#!/usr/bin/env python3
"""Live status for the Isaac Lab UniFP training run.

    ./unifp_train/progress.py              # one report
    ./unifp_train/progress.py --watch      # refresh every 30 s until Ctrl-C
    ./unifp_train/progress.py --watch 10   # ... every 10 s

The sibling of `unifp_go2d1/progress.py`, which watches the Isaac Gym run. Reads the run
directory and the captured stdout; touches nothing, so it is safe against a live job.

It reports the **three objective terms separately**, which is not decoration. The frame bug of
F-078 left `tracking_ee_force_world` at exactly zero for every environment while every other
term moved and the total stayed plausible; a run where one objective is dead should be visible
from across the room rather than found by reading a log afterwards.
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LOGS = Path(os.environ.get("UNIFP_TRAIN_LOGS", ROOT / "logs/unifp_train"))
TARGET = int(os.environ.get("TARGET", 60000))
FORCE_START = int(os.environ.get("FORCE_START", 8000))

ANSI = re.compile(r"\x1b\[[0-9;]*m")
BOLD, DIM, RESET = "\x1b[1m", "\x1b[2m", "\x1b[0m"
GREEN, YELLOW, RED, CYAN = "\x1b[32m", "\x1b[33m", "\x1b[31m", "\x1b[36m"

#: The three terms the task is actually optimising, and `alive`, which pays for merely not
#: falling over and so says whether the robot is upright without needing the episode length.
OBJECTIVES = ("tracking_ee_force_world", "tracking_lin_vel_force_world", "tracking_ang_vel")


def newest(pattern: str, where: Path):
    hits = sorted(where.glob(pattern), key=lambda p: p.stat().st_mtime) if where.is_dir() else []
    return hits[-1] if hits else None


def tail(path: Path, lines: int = 4000) -> str:
    try:
        out = subprocess.run(["tail", "-n", str(lines), str(path)],
                             capture_output=True, text=True)
        return ANSI.sub("", out.stdout)
    except Exception:
        return ""


def last(pattern: str, text: str, cast=float):
    hits = re.findall(pattern, text)
    return cast(hits[-1]) if hits else None


def series(pattern: str, text: str, count: int = 30):
    return [float(h) for h in re.findall(pattern, text)][-count:]


def gpu():
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.used,memory.total,utilization.gpu,temperature.gpu",
             "--format=csv,noheader,nounits"], capture_output=True, text=True).stdout.strip()
        return tuple(int(x) for x in out.split(", "))
    except Exception:
        return None


def running_pid(run_dir: Path | None) -> int | None:
    """The training process, from the pid file the launcher writes and removes.

    Not `pgrep -f run_unifp_train.py`: that pattern matches any shell whose command line mentions
    the script, the grep's own shell included, so it reports a finished run as still going.
    """
    if run_dir is None:
        return None
    pid_file = run_dir / "train.pid"
    if not pid_file.is_file():
        return None
    try:
        pid = int(pid_file.read_text().strip())
    except ValueError:
        return None
    return pid if Path(f"/proc/{pid}").exists() else None


def hms(seconds) -> str:
    if seconds is None or seconds < 0:
        return "?"
    seconds = int(seconds)
    d, r = divmod(seconds, 86400)
    h, r = divmod(r, 3600)
    m, s = divmod(r, 60)
    return f"{d}d {h:02d}h {m:02d}m" if d else f"{h:02d}h {m:02d}m {s:02d}s"


def bar(fraction: float, width: int = 44) -> str:
    filled = int(round(fraction * width))
    return "█" * filled + "·" * (width - filled)


def report(log_path: Path | None, run_dir: Path | None) -> None:
    text = tail(log_path) if log_path and log_path.exists() else ""
    it = last(r"Learning iteration (\d+)/", text, int)
    iter_s = last(r"Iteration time: ([\d.]+)s", text)
    fps = last(r"Steps per second: (\d+)", text, int)
    reward = last(r"Mean reward: (-?[\d.]+)", text)
    ep_len = last(r"Mean episode length: ([\d.]+)", text)
    estimator = last(r"Mean estimator loss: (-?[\d.]+)", text)
    rewards = series(r"Mean reward: (-?[\d.]+)", text)

    checkpoint = newest("model_*.pt", run_dir) if run_dir else None
    ckpt_iter = int(re.search(r"model_(\d+)", checkpoint.name).group(1)) if checkpoint else -1

    done = it if it is not None else max(ckpt_iter, 0)
    frac = min(done / TARGET, 1.0) if TARGET else 0.0
    eta = max(TARGET - done, 0) * iter_s if iter_s else None

    pid = running_pid(run_dir)
    fresh = time.time() - log_path.stat().st_mtime if log_path and log_path.exists() else None

    print(f"\n{BOLD}Go2+D1 · UniFP position/force · Isaac Lab{RESET}"
          f"   {DIM}{time.strftime('%Y-%m-%d %H:%M:%S')}{RESET}")
    print("─" * 72)

    if pid:
        health = f"{GREEN}running{RESET} (pid {pid})"
        if fresh and fresh > 600:
            health = (f"{YELLOW}running but quiet{RESET} (pid {pid}, "
                      f"no log line for {hms(fresh)})")
    elif done >= TARGET:
        health = f"{GREEN}complete{RESET}"
    else:
        health = f"{RED}stopped{RESET} — the run is not going"
    print(f"  training    {health}")
    if run_dir:
        print(f"  run         {DIM}{run_dir.name}{RESET}")

    print()
    print(f"  {bar(frac)}  {100 * frac:5.1f}%")
    print(f"  iteration {BOLD}{done:,}{RESET} of {TARGET:,}"
          + (f"   ·   {iter_s:.2f}s/iter" if iter_s else "")
          + (f"   ·   {fps:,} steps/s" if fps else ""))
    if eta:
        finish = time.localtime(time.time() + eta)
        print(f"  {DIM}ETA {hms(eta)}  ·  finishes about "
              f"{time.strftime('%a %d %b %H:%M', finish)}{RESET}")

    if done < FORCE_START:
        left = (FORCE_START - done) * iter_s if iter_s else None
        print(f"  force curriculum: {YELLOW}not started{RESET} — opens at iteration "
              f"{FORCE_START:,}" + (f", in {hms(left)}" if left else ""))
    else:
        print(f"  force curriculum: {GREEN}active{RESET} since iteration {FORCE_START:,}")

    print()
    if reward is not None:
        trend = ""
        if len(rewards) >= 10:
            delta = rewards[-1] - rewards[0]
            arrow = "↑" if delta > 0 else ("↓" if delta < 0 else "→")
            trend = f"   {DIM}{arrow} {delta:+.1f} over the last {len(rewards)} logged{RESET}"
        print(f"  mean return            {BOLD}{reward:>9.2f}{RESET}{trend}")
    if ep_len is not None:
        share = ep_len / 1000
        note = f"{DIM}({100 * share:.0f}% of the 20 s episode){RESET}"
        print(f"  mean episode length    {ep_len:>9.0f} steps   {note}")
    if estimator is not None:
        print(f"  estimator loss         {estimator:>9.4f}   "
              f"{DIM}(the adaptation module's supervised error){RESET}")

    print()
    print(f"  {DIM}objectives — a term pinned at 0.0000 while the others move is a bug, "
          f"not a policy{RESET}")
    for name in OBJECTIVES:
        value = last(rf"Episode_Reward/{name}: (-?[\d.]+)", text)
        if value is None:
            continue
        flag = f"   {RED}<-- dead{RESET}" if abs(value) < 1e-9 else ""
        print(f"    {name:<32s} {CYAN}{value:>8.4f}{RESET}{flag}")
    alive_term = last(r"Episode_Reward/alive: (-?[\d.]+)", text)
    if alive_term is not None:
        print(f"    {'alive':<32s} {alive_term:>8.4f}   "
              f"{DIM}(1.50 = upright for the whole 20 s){RESET}")

    print()
    if checkpoint:
        age = time.time() - checkpoint.stat().st_mtime
        print(f"  last checkpoint        {checkpoint.name}   {DIM}{hms(age)} ago{RESET}")
    g = gpu()
    if g:
        used, total, util, temp = g
        warn = f"   {RED}<-- near the limit{RESET}" if used > 0.92 * total else ""
        print(f"  gpu                    {used:,} / {total:,} MiB   {util}% util   "
              f"{temp}°C{warn}")
    stat = os.statvfs(ROOT)
    free_gb = stat.f_bavail * stat.f_frsize / 1e9
    warn = f"   {RED}<-- low{RESET}" if free_gb < 20 else ""
    print(f"  disk free              {free_gb:.0f} GB{warn}")
    print()


def resolve(args) -> tuple[Path | None, Path | None]:
    log_path = Path(args.log) if args.log else newest("*.log", LOGS)
    if args.run_dir:
        run_dir = Path(args.run_dir)
    else:
        candidates = [p for p in LOGS.glob("*_train_*") if p.is_dir()] if LOGS.is_dir() else []
        run_dir = max(candidates, key=lambda p: p.stat().st_mtime) if candidates else None
    return log_path, run_dir


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--watch", nargs="?", type=int, const=30, default=None,
                        help="refresh every N seconds (default 30)")
    parser.add_argument("--log", help="captured stdout; default is the newest under logs/unifp_train")
    parser.add_argument("--run_dir", help="run directory; default is the newest")
    args = parser.parse_args()

    if args.watch is None:
        report(*resolve(args))
        return
    try:
        while True:
            print("\x1b[2J\x1b[H", end="")
            report(*resolve(args))
            print(f"  {DIM}refreshing every {args.watch}s — Ctrl-C to stop{RESET}")
            time.sleep(args.watch)
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    sys.exit(main())
