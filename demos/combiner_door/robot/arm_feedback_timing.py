#!/usr/bin/env python3
"""Read-only: when the D1's reports arrive on the dog, and when the arm stamped them. Readers only -- no writer on
`rt/arm_Command`, so this cannot command the arm.

    docker run --rm --network host -v ~/D1Training:/root/D1Training --entrypoint python3 p2dingo-arm:latest \\
        /root/D1Training/demos/combiner_door/robot/arm_feedback_timing.py --seconds 60 --out <dir>

Why (2026-10-23): on the dog the arm link saw the arm's reports go 0.3-0.8 s stale in 10-15% of its ticks, as often
with the arm still as moving, and the arm stopped acting on commands through them; F-020 had measured a steady 111 ms
on the bench. Each sample's arrival time (this process's clock) and its DDS source timestamp (the arm's) tell the two
apart: gaps in both, the arm stopped publishing; gaps on arrival only, the network or this computer held them.

Writes `samples.jsonl` (topic, funcode, arrival time, source time) and `summary.json` (per stream: intervals and gaps
on either clock; the arm NIC's counters before and after; the load average each second), and prints the summary.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

GAP_S = 0.25


def nic_counters(iface: str) -> dict:
    base = Path("/sys/class/net") / iface / "statistics"
    out = {}
    for name in ("rx_packets", "tx_packets", "rx_errors", "rx_dropped", "tx_errors", "tx_dropped", "rx_bytes"):
        try:
            out[name] = int((base / name).read_text())
        except OSError:
            out[name] = None
    return out


def loadavg() -> list[float]:
    try:
        return [float(v) for v in Path("/proc/loadavg").read_text().split()[:3]]
    except OSError:
        return []


def stats(times: list[float]) -> dict:
    if len(times) < 2:
        return {"n": len(times)}
    dt = [b - a for a, b in zip(times, times[1:])]
    s = sorted(dt)
    pick = lambda q: round(s[min(len(s) - 1, int(q * len(s)))], 4)
    return {"n": len(times), "rate_hz": round((len(times) - 1) / (times[-1] - times[0]), 3), "p50_s": pick(0.5),
            "p90_s": pick(0.9), "p99_s": pick(0.99), "max_s": round(s[-1], 4),
            "gaps_over_0.25s": sum(1 for v in dt if v > GAP_S),
            "time_in_gaps_s": round(sum(v for v in dt if v > GAP_S), 3)}


def main(argv=None) -> int:
    import d1_hardware

    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--seconds", type=float, default=60.0, help="at most; SIGTERM or SIGINT end it early, summary written")
    p.add_argument("--iface", default=d1_hardware.ARM_IFACE)
    p.add_argument("--out", default=str(ROOT / "logs/arm_feedback_timing" / time.strftime("%Y%m%dT%H%M%S", time.gmtime())))
    args = p.parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    os.environ["CYCLONEDDS_URI"] = ("<CycloneDDS><Domain><General><Interfaces>"
                                    f"<NetworkInterface name='{args.iface}'/>"
                                    "</Interfaces></General></Domain></CycloneDDS>")
    from cyclonedds.core import InstanceState, ReadCondition, SampleState, ViewState, WaitSet
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from cyclonedds.util import duration

    ArmString, PubServoInfo = d1_hardware._message_types()
    dp = DomainParticipant(0)
    servo = DataReader(dp, Topic(dp, "current_servo_angle", PubServoInfo))
    fb = DataReader(dp, Topic(dp, "rt/arm_Feedback", ArmString))
    ws = WaitSet(dp)
    any_state = SampleState.Any | ViewState.Any | InstanceState.Any
    ws.attach(ReadCondition(servo, any_state))
    ws.attach(ReadCondition(fb, any_state))

    def source_s(sample):
        info = getattr(sample, "sample_info", None)
        stamp = getattr(info, "source_timestamp", None) if info is not None else None
        return None if stamp is None else stamp / 1e9

    streams = {"current_servo_angle": [], "arm_Feedback/1": [], "arm_Feedback/3": []}
    sources = {k: [] for k in streams}
    loads = []
    stop = {"now": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    nic_before = nic_counters(args.iface)
    start = time.monotonic()
    next_load = start
    with (out / "samples.jsonl").open("w") as f:
        while time.monotonic() - start < args.seconds and not stop["now"]:
            try:
                ws.wait(duration(milliseconds=50))
            except Exception:
                pass
            now = time.monotonic()
            if now >= next_load:
                loads.append([round(now - start, 1), *loadavg()])
                next_load += 1.0
            for s in servo.take(N=50):
                if not hasattr(s, "servo0_data_"):
                    continue
                streams["current_servo_angle"].append(now)
                src = source_s(s)
                if src is not None:
                    sources["current_servo_angle"].append(src)
                f.write(json.dumps({"topic": "current_servo_angle", "t": round(now - start, 4), "src": src,
                                    "wall": round(time.time(), 4)}) + "\n")
            for s in fb.take(N=50):
                if not hasattr(s, "data_"):
                    continue
                try:
                    funcode = json.loads(s.data_).get("funcode")
                except ValueError:
                    continue
                key = f"arm_Feedback/{funcode}"
                if key not in streams:
                    continue
                streams[key].append(now)
                src = source_s(s)
                if src is not None:
                    sources[key].append(src)
                f.write(json.dumps({"topic": key, "t": round(now - start, 4), "src": src, "wall": round(time.time(), 4)})
                        + "\n")
    summary = {"seconds": round(time.monotonic() - start, 1), "iface": args.iface,
               "arrival": {k: stats(v) for k, v in streams.items()},
               "source": {k: stats(v) for k, v in sources.items()},
               "nic_before": nic_before, "nic_after": nic_counters(args.iface), "loadavg_each_s": loads}
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps({k: summary[k] for k in ("arrival", "source", "nic_before", "nic_after")}, indent=1))
    if loads:
        print("load average (1 min) from", loads[0][1], "to", loads[-1][1], "max", max(l[1] for l in loads))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
