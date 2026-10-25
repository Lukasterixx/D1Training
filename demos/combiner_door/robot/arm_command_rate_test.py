#!/usr/bin/env python3
"""Does commanding the D1 make it pause its joint reports (F-200), and what in the commands does it? The arm held still
where it is, commanded its own pose in blocks: silent, 10 Hz, 5 Hz, 2 Hz, 10 Hz without the gripper's angle, 10 Hz
again. Run on the dog with `run_command_rate_test_on_dog.sh`, which records the arm's report timing beside it
(`arm_feedback_timing.py`), then `--analyse <dir>` on either side.

Lukas, 2026-10-23: "you can set the tests up and you have permission to move the arm. the robot is sat stationary and
nothing is nearby it".

What it will and will not do:
- **Refuses** unless the arm reports, P2Dingo's arm bridge is not streaming to it, and the arm is folded (Joint1-3
  within `FOLDED_DEG` of the commandable fold).
- **Commands only the pose it read at the start** (funcode 2, mode 0, `d1_hardware.set_all_joint_angles`, which clamps
  to the hard limits and paces motion commands to at most 10 Hz): the arm should not move. A block where the arm's
  report moves more than `MOVED_DEG` from that pose stops the test.
- **Without the gripper's angle** a command is funcode 2 with the six arm angles and no `angle6` (every command in a
  block with `gripper_every` 0, all but every n-th with n). **The arm does not execute such commands** (F-202: a live
  run that left it out moved the arm only on the commands that had it), and stops its status stream (funcode 3) while it
  gets them. A held pose cannot tell an ignored command from an executed one: in these blocks the rate of commands the
  arm executed is `hz / gripper_every`.
- **Never releases the arm.** It ends by stopping the stream: in mode 0 the arm then holds where it is (F-022).

Writes `rate_test.json`: each block's wall-clock start and end, its commands' wall times, and the reports this client
saw (arrival times); `--analyse` adds the capture's source-timestamp gaps per block.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

#: (name, commands a second (0: silent), the gripper's angle in every how many commands (0: none)).
BLOCK_SETS = {
    # The first test (2026-10-23): the pauses come with the gripper's angle in each command.
    "rate": (("silent", 0.0, 1), ("10hz", 10.0, 1), ("5hz", 5.0, 1), ("2hz", 2.0, 1),
             ("10hz_no_gripper", 10.0, 0), ("10hz_again", 10.0, 1)),
    # The second: without it the arm stopped its status stream; the gripper's angle at 2 Hz in a 10 Hz stream?
    "gripper": (("silent", 0.0, 1), ("10hz_gripper_every_5th", 10.0, 5), ("10hz", 10.0, 1),
                ("10hz_gripper_every_10th", 10.0, 10), ("10hz_no_gripper", 10.0, 0), ("silent_after", 0.0, 1),
                ("10hz_gripper_every_5th_again", 10.0, 5)),
}
BLOCKS = BLOCK_SETS["rate"]
BLOCK_S = 20.0
BETWEEN_S = 3.0
FOLDED_DEG = 15.0
MOVED_DEG = 3.0
GAP_S = 0.25


def log(msg: str) -> None:
    print(f"[rate_test {time.strftime('%H:%M:%S')}] {msg}", flush=True)


def run(out: Path, block_s: float, blocks=BLOCKS) -> int:
    import d1_hardware
    from demos.combiner_door.robot import arm_link

    status = arm_link.bridge_status()
    if status and status.get("connected") and status.get("mode") in ("MANUAL", "BUSY"):
        log(f"refusing: P2Dingo's arm bridge is streaming to the arm ({status})")
        return 3
    client = d1_hardware.D1Client()
    q0 = client.wait_for_feedback(timeout_s=5.0)
    pose, gripper = [float(v) for v in q0[:6]], float(q0[6])
    fold = [float(np.clip(v, -90.0, 90.0)) for v in d1_hardware.REST_SERVO_DEG]
    off = max(abs(a - b) for a, b in zip(pose[:3], fold[:3]))
    if off > FOLDED_DEG:
        log(f"refusing: the arm is not folded ({[round(v, 1) for v in pose]}; Joint1-3 {off:.0f} deg off the fold)")
        return 2
    log(f"holding {[round(v, 1) for v in pose]}, gripper {gripper:.1f}")
    record = {"pose": pose, "gripper": gripper, "blocks": []}
    seen = {"t": 0.0}

    def poll_until(t_end: float, block: dict) -> None:
        while time.monotonic() < t_end:
            client.poll(timeout_s=0.01)
            rx = client._last_rx
            if rx != seen["t"]:
                seen["t"] = rx
                block["reports"].append(round(time.time() - (time.monotonic() - rx), 4))
                q = client.get_joint_angles()
                moved = max(abs(a - b) for a, b in zip(q, pose))
                block["moved_max_deg"] = max(block.get("moved_max_deg", 0.0), round(moved, 2))
                if moved > MOVED_DEG:
                    raise RuntimeError(f"the arm moved {moved:.1f} deg in {block['name']}: {[round(v, 1) for v in q]}")

    try:
        for name, hz, gripper_every in blocks:
            gap = {"name": "between", "reports": [], "commands": []}
            poll_until(time.monotonic() + BETWEEN_S, gap)
            block = {"name": name, "hz": hz, "gripper_every": gripper_every, "wall_start": round(time.time(), 4),
                     "reports": [], "commands": []}
            log(f"{name}: {block_s:.0f} s" + (f" at {hz:g} Hz" if hz else " (no commands)"))
            start = time.monotonic()
            next_cmd = start
            while time.monotonic() - start < block_s:
                if hz and time.monotonic() >= next_cmd:
                    if gripper_every and len(block["commands"]) % gripper_every == 0:
                        client.set_all_joint_angles(pose, mode=0, gripper=gripper)
                    else:
                        data = {"mode": 0}
                        data.update({f"angle{i}": round(a, 3) for i, a in enumerate(pose)})
                        client._pace()
                        client._send(2, data)
                    block["commands"].append(round(time.time(), 4))
                    next_cmd += 1.0 / hz
                poll_until(min(next_cmd if hz else start + block_s, start + block_s), block)
            block["wall_end"] = round(time.time(), 4)
            record["blocks"].append(block)
            gaps = np.diff(block["reports"]) if len(block["reports"]) > 1 else np.zeros(1)
            log(f"  {len(block['commands'])} commands, {len(block['reports'])} reports, "
                f"{int((gaps > GAP_S).sum())} gaps over {GAP_S} s (max {gaps.max():.2f} s), moved {block.get('moved_max_deg', 0)} deg")
    except (RuntimeError, KeyboardInterrupt) as exc:
        record["stopped"] = str(exc) or "Ctrl-C"
        log(f"stopped: {record['stopped']}; the stream stops and the arm holds where it is")
    record["final"] = client.get_joint_angles() + [client.get_gripper_units()]
    (out / "rate_test.json").write_text(json.dumps(record, indent=1))
    log(f"done; the arm at {[round(v, 1) for v in record['final']]}; record {out}")
    return 0 if "stopped" not in record else 1


def analyse(out: Path) -> dict:
    """Per block: this client's report gaps, and the capture's (`feedback_timing/samples.jsonl`) on the arm's clock."""
    record = json.loads((out / "rate_test.json").read_text())
    samples = []
    path = out / "feedback_timing" / "samples.jsonl"
    if path.exists():
        samples = [json.loads(line) for line in path.read_text().splitlines()]
    result = {}
    for block in record["blocks"]:
        w0, w1 = block["wall_start"], block["wall_end"]
        row = {"hz": block["hz"], "gripper_every": block.get("gripper_every", 1 if block.get("gripper", True) else 0),
               "commands": len(block["commands"]),
               "moved_max_deg": block.get("moved_max_deg")}
        for topic in ("current_servo_angle", "arm_Feedback/1", "arm_Feedback/3"):
            s = [x for x in samples if x["topic"] == topic and w0 <= x["wall"] <= w1 and x.get("src") is not None]
            src = np.array([x["src"] for x in s])
            if len(src) <= 1:
                row[topic] = {"n": len(src), "stopped": True}
            else:
                d = np.diff(src)
                row[topic] = {"n": len(src), "rate_hz": round((len(src) - 1) / (src[-1] - src[0]), 2),
                              "gaps": int((d > GAP_S).sum()), "in_gaps_s": round(float(d[d > GAP_S].sum()), 2),
                              "max_s": round(float(d.max()), 3)}
        result[block["name"]] = row
    (out / "analysis.json").write_text(json.dumps(result, indent=1))
    return result


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--out", default=None)
    p.add_argument("--block_s", type=float, default=BLOCK_S)
    p.add_argument("--blocks", choices=tuple(BLOCK_SETS), default="rate")
    p.add_argument("--analyse", default=None, help="a finished test's directory: summarise it, command nothing")
    args = p.parse_args(argv)
    if args.analyse:
        for name, row in analyse(Path(args.analyse)).items():
            print(name, json.dumps(row))
        return 0
    out = Path(args.out or ROOT / "logs/arm_command_rate" / time.strftime("%Y%m%dT%H%M%S", time.gmtime()))
    out.mkdir(parents=True, exist_ok=True)
    return run(out, args.block_s, BLOCK_SETS[args.blocks])


if __name__ == "__main__":
    raise SystemExit(main())
