"""Live status for the Go2+D1 UniFP training run.

    ./unifp_go2d1/progress.py              # one report
    ./unifp_go2d1/progress.py --watch      # refresh every 30 s until Ctrl-C
    ./unifp_go2d1/progress.py --watch 10   # ... every 10 s

Reads the training log, the checkpoints and the supervisor log. Touches nothing, so it is safe to
run against a live job as often as you like. Progress is reported against the checkpoints rather
than the log, so it stays right across a resume (the log file changes, the checkpoint numbering
does not).
"""

import argparse
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
UNIFP = os.environ.get("UNIFP_ROOT", os.path.expanduser("~/thesis_b_legacy/UniFP"))
EXP = os.environ.get("EXP", "go2d1_pos_force")
TARGET = int(os.environ.get("TARGET", 60000))
FORCE_START = int(os.environ.get("FORCE_START", 8000))

ANSI = re.compile(r"\x1b\[[0-9;]*m")
BOLD, DIM, RESET = "\x1b[1m", "\x1b[2m", "\x1b[0m"
GREEN, YELLOW, RED = "\x1b[32m", "\x1b[33m", "\x1b[31m"


def newest_log():
    logs = [f for f in os.listdir(f"{UNIFP}/logs")
            if f.startswith(("train_", "train_resumed_")) and f.endswith(".log")]
    if not logs:
        return None
    return max((os.path.join(f"{UNIFP}/logs", f) for f in logs), key=os.path.getmtime)


def tail(path, lines=2000):
    try:
        out = subprocess.run(["tail", "-n", str(lines), path], capture_output=True, text=True)
        return ANSI.sub("", out.stdout)
    except Exception:
        return ""


def last(pattern, text, cast=float):
    hits = re.findall(pattern, text)
    return cast(hits[-1]) if hits else None


def series(pattern, text, count=30):
    return [float(h) for h in re.findall(pattern, text)][-count:]


def gpu():
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.used,memory.total,utilization.gpu,temperature.gpu",
             "--format=csv,noheader,nounits"], capture_output=True, text=True).stdout.strip()
        used, total, util, temp = (int(x) for x in out.split(", "))
        return used, total, util, temp
    except Exception:
        return None


def alive(pattern):
    out = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True)
    return [int(p) for p in out.stdout.split()]


def hms(seconds):
    if seconds is None or seconds < 0:
        return "?"
    seconds = int(seconds)
    d, r = divmod(seconds, 86400)
    h, r = divmod(r, 3600)
    m, s = divmod(r, 60)
    return f"{d}d {h:02d}h {m:02d}m" if d else f"{h:02d}h {m:02d}m {s:02d}s"


def bar(fraction, width=44):
    filled = int(round(fraction * width))
    return "█" * filled + "·" * (width - filled)


def report():
    sys.path.insert(0, HERE)
    from find_checkpoint import checkpoints

    found = checkpoints(f"{UNIFP}/logs/{EXP}")
    ckpt_iter, ckpt_run = (found[0] if found else (-1, None))

    log_path = newest_log()
    text = tail(log_path) if log_path else ""
    it = last(r"Learning iteration (\d+)/", text, int)
    iter_s = last(r"Iteration time: ([\d.]+)s", text)
    fps = last(r"Computation: (\d+) steps/s", text)
    reward = last(r"Mean reward: (-?[\d.]+)", text)
    ep_len = last(r"Mean episode length: ([\d.]+)", text)
    rewards = series(r"Mean reward: (-?[\d.]+)", text)

    done = it if it is not None else max(ckpt_iter, 0)
    frac = min(done / TARGET, 1.0)
    remaining = max(TARGET - done, 0)
    eta = remaining * iter_s if iter_s else None

    train_pids = alive("launch_training.py.*--task=go2d1_pos_force")
    sup_pids = alive("supervise_training.sh")
    sup_log = f"{UNIFP}/logs/supervisor.log"
    sup_text = tail(sup_log, 200) if os.path.exists(sup_log) else ""
    restarts = len(re.findall(r"Resuming from", sup_text))
    fatal = "FATAL" in sup_text
    stopped = os.path.exists(f"{UNIFP}/logs/STOP")

    fresh = None
    if log_path:
        fresh = time.time() - os.path.getmtime(log_path)

    print(f"\n{BOLD}Go2+D1 · UniFP position/force{RESET}   {DIM}{time.strftime('%Y-%m-%d %H:%M:%S')}{RESET}")
    print("─" * 66)

    if train_pids:
        health = f"{GREEN}running{RESET} (pid {train_pids[0]})"
        if fresh and fresh > 300:
            health = f"{YELLOW}running but quiet{RESET} (pid {train_pids[0]}, no log line for {hms(fresh)})"
    elif stopped:
        health = f"{DIM}stopped on purpose (STOP file present){RESET}"
    elif done >= TARGET:
        health = f"{GREEN}complete{RESET}"
    else:
        health = f"{RED}NOT RUNNING{RESET}"
    print(f"  training    {health}")

    if fatal:
        sup = f"{RED}gave up — see supervisor.log{RESET}"
    elif sup_pids:
        sup = f"{GREEN}watching{RESET} (pid {sup_pids[0]})"
    else:
        sup = f"{YELLOW}not running{RESET}"
    print(f"  supervisor  {sup}" + (f"   {DIM}{restarts} resume(s) so far{RESET}" if restarts else ""))

    print()
    print(f"  {bar(frac)}  {100 * frac:5.1f}%")
    print(f"  iteration {BOLD}{done:,}{RESET} of {TARGET:,}"
          + (f"   ·   {iter_s:.2f}s/iter" if iter_s else "")
          + (f"   ·   {fps:,.0f} steps/s" if fps else ""))
    if eta:
        print(f"  {DIM}ETA {hms(eta)}  ·  finishes about {time.strftime('%a %H:%M', time.localtime(time.time() + eta))}{RESET}")

    if done < FORCE_START:
        left = (FORCE_START - done) * iter_s if iter_s else None
        print(f"  force curriculum: {YELLOW}not started{RESET} — begins at iteration "
              f"{FORCE_START:,}" + (f", in {hms(left)}" if left else ""))
    else:
        print(f"  force curriculum: {GREEN}active{RESET} since iteration {FORCE_START:,}")

    print()
    if reward is not None:
        trend = ""
        if len(rewards) >= 10:
            delta = rewards[-1] - rewards[0]
            arrow = "↑" if delta > 0 else ("↓" if delta < 0 else "→")
            trend = f"  {DIM}{arrow} {delta:+.1f} over the last {len(rewards)} logged{RESET}"
        print(f"  mean return          {BOLD}{reward:>8.1f}{RESET}{trend}")
    if ep_len is not None:
        print(f"  mean episode length  {ep_len:>8.0f} steps   {DIM}(1000 = ran the full 20 s){RESET}")

    if ckpt_iter >= 0:
        age = time.time() - os.path.getmtime(f"{UNIFP}/logs/{EXP}/{ckpt_run}/model_{ckpt_iter}.pt")
        print(f"  last checkpoint      model_{ckpt_iter}.pt   {DIM}{hms(age)} ago, in {ckpt_run}{RESET}")

    g = gpu()
    if g:
        used, total, util, temp = g
        print(f"  gpu                  {used:,} / {total:,} MiB   {util}% util   {temp}°C")

    free_gb = os.statvfs(UNIFP).f_bavail * os.statvfs(UNIFP).f_frsize / 1e9
    warn = f"   {RED}<-- low{RESET}" if free_gb < 20 else ""
    print(f"  disk free            {free_gb:.0f} GB{warn}")
    print()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--watch", nargs="?", type=int, const=30, default=None,
                   help="refresh every N seconds (default 30)")
    p.add_argument("--once", action="store_true", help="print one report and exit (the default)")
    a = p.parse_args()
    if a.watch is None:
        report()
        return
    try:
        while True:
            print("\x1b[2J\x1b[H", end="")
            report()
            print(f"  {DIM}refreshing every {a.watch}s — Ctrl-C to stop{RESET}")
            time.sleep(a.watch)
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
