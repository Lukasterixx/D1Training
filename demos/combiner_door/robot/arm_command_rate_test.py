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
  report moves more than `MOVED_DEG` from that pose stops the test. A swing block's Joint1 alone gets `SWING_DEG` more.
- **Without the gripper's angle** a command is funcode 2 with the six arm angles and no `angle6` (every command in a
  block with `gripper_every` 0, all but every n-th with n). **The arm does not execute such commands** (F-202: a live
  run that left it out moved the arm only on the commands that had it), and stops its status stream (funcode 3) while it
  gets them. A held pose cannot tell an ignored command from an executed one: in these blocks the rate of commands the
  arm executed is `hz / gripper_every`.
- **Never releases the arm.** It ends by stopping the stream: in mode 0 the arm then holds where it is (F-022).

**Locked blocks** (`--blocks phase`, `--blocks swing`; F-238) send one command a report, a fixed time after it
(`d1_hardware.ReportClock`). `phase` holds the pose: the offsets the old runs say are safe, and one aimed at the stall
window on purpose, which should stall. A held pose cannot show that a command was executed (F-201), so `swing` moves
Joint1 by `SWING_DEG` either way at `SWING_DEG_S`, unlocked at 10 Hz and locked. Swing blocks, here and in `--blocks gate`,
are the only ones that move the arm: run them only with someone watching. **This test turns `D1Client`'s gate off**, because the stall is what it measures; `--blocks
gate` turns it on per block, a plain 10 Hz clock through the gate, as a sender with its own timing gets it.

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

#: (name, commands a second (0: silent), the gripper's angle in every how many commands (0: none), then optionally
#: ms after each report to send (None: by the clock at `hz`), Joint1's swing in degrees either way, and `D1Client`'s
#: gate on or off (default off).
BLOCK_SETS = {
    # The first test (2026-10-23): the pauses come with the gripper's angle in each command.
    "rate": (("silent", 0.0, 1), ("10hz", 10.0, 1), ("5hz", 5.0, 1), ("2hz", 2.0, 1),
             ("10hz_no_gripper", 10.0, 0), ("10hz_again", 10.0, 1)),
    # The second: without it the arm stopped its status stream; the gripper's angle at 2 Hz in a 10 Hz stream?
    "gripper": (("silent", 0.0, 1), ("10hz_gripper_every_5th", 10.0, 5), ("10hz", 10.0, 1),
                ("10hz_gripper_every_10th", 10.0, 10), ("10hz_no_gripper", 10.0, 0), ("silent_after", 0.0, 1),
                ("10hz_gripper_every_5th_again", 10.0, 5)),
    # F-238: commands locked to the reports. 103 ms is the stall window, the control: it should stall, the others not.
    "phase": (("silent", 0.0, 1), ("10hz", 10.0, 1), ("lock_20ms", 9.0, 1, 20.0), ("lock_50ms", 9.0, 1, 50.0),
              ("lock_80ms", 9.0, 1, 80.0), ("lock_103ms_control", 9.0, 1, 103.0), ("lock_20ms_again", 9.0, 1, 20.0)),
    # The same on a moving arm, the same step a command: does the motion stop with the reports, and does locking end it?
    # A sender on its own 10 Hz clock with only the client's gate to protect it (the console, the pick demo, the CLI).
    "gate": (("10hz", 10.0, 1), ("10hz_gated", 10.0, 1, None, 0.0, True), ("10hz_gated_again", 10.0, 1, None, 0.0, True),
             ("swing_10hz_gated", 10.0, 1, None, 10.0, True), ("swing_10hz_gated_again", 10.0, 1, None, 10.0, True)),
    "swing": (("swing_10hz", 10.0, 1, None, 10.0), ("swing_lock_20ms", 9.0, 1, 20.0, 10.0),
              ("swing_10hz_again", 10.0, 1, None, 10.0), ("swing_lock_20ms_again", 9.0, 1, 20.0, 10.0)),
}
BLOCKS = BLOCK_SETS["rate"]
BLOCK_S = 20.0
BETWEEN_S = 3.0
FOLDED_DEG = 15.0
MOVED_DEG = 3.0
GAP_S = 0.25
#: A swing's reach and pace: 36 deg/s is 4 deg a command locked (F-199: over ~2.5 keeps the firmware moving between them).
SWING_DEG = 10.0
SWING_DEG_S = 36.0


def swing_offset(t_s: float, swing_deg: float, rate_deg_s: float = SWING_DEG_S) -> float:
    """Joint1's offset from its start `t_s` into a swing: a triangle, from 0 out to +swing, back through 0 to -swing."""
    if not swing_deg:
        return 0.0
    quarter = swing_deg / rate_deg_s
    phase = (t_s / quarter) % 4.0
    return swing_deg * (phase if phase < 1 else 2 - phase if phase < 3 else phase - 4)


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
    client.report_gate = False
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

    def poll_until(t_end: float, block: dict, swing_deg: float = 0.0) -> None:
        while time.monotonic() < t_end:
            client.poll(timeout_s=min(0.01, max(0.001, t_end - time.monotonic())))
            rx = client._last_rx
            if rx != seen["t"]:
                seen["t"] = rx
                block["reports"].append(round(time.time() - (time.monotonic() - rx), 4))
                q = client.get_joint_angles()
                moved = max(max(0.0, abs(a - b) - (swing_deg if i == 0 else 0.0))
                            for i, (a, b) in enumerate(zip(q, pose)))
                if swing_deg:
                    block.setdefault("joint1", []).append([block["reports"][-1], round(q[0], 2)])
                block["moved_max_deg"] = max(block.get("moved_max_deg", 0.0), round(moved, 2))
                if moved > MOVED_DEG:
                    raise RuntimeError(f"the arm moved {moved:.1f} deg in {block['name']}: {[round(v, 1) for v in q]}")

    try:
        last_swing = 0.0
        for name, hz, gripper_every, *rest in blocks:
            lock_ms = rest[0] if rest else None
            swing_deg = rest[1] if len(rest) > 1 else 0.0
            client.report_gate = rest[2] if len(rest) > 2 else False
            clock = None
            if lock_ms is not None:
                clock = d1_hardware.ReportClock(offset_s=lock_ms / 1000.0, slack_s=0.004, allow_window=lock_ms > 80.0)
            gap = {"name": "between", "reports": [], "commands": []}
            poll_until(time.monotonic() + BETWEEN_S, gap, last_swing)  # the arm may still be on its way back
            last_swing = swing_deg
            block = {"name": name, "hz": hz, "gripper_every": gripper_every, "lock_ms": lock_ms, "swing_deg": swing_deg,
                     "gated": client.report_gate,
                     "wall_start": round(time.time(), 4), "reports": [], "commands": []}
            if swing_deg:
                block["sent"] = []
            log(f"{name}: {block_s:.0f} s" + (f" at {hz:g} Hz" if hz else " (no commands)")
                + (f", {lock_ms:g} ms after each report" if clock else ""))
            start = time.monotonic()
            next_cmd = start

            def command() -> None:
                target = list(pose)
                target[0] += swing_offset(time.monotonic() - start, swing_deg)
                if gripper_every and len(block["commands"]) % gripper_every == 0:
                    client.set_all_joint_angles(target, mode=0, gripper=gripper)
                else:
                    data = {"mode": 0}
                    data.update({f"angle{i}": round(a, 3) for i, a in enumerate(target)})
                    client._pace()
                    client._send(2, data)
                block["commands"].append(round(time.time(), 4))
                if swing_deg:
                    block["sent"].append(round(target[0], 2))

            while time.monotonic() - start < block_s:
                if clock is not None:
                    now = time.monotonic()
                    if clock.due(now, client.servo_report_s):
                        command()
                        clock.sent(now)
                    poll_until(min(time.monotonic() + 0.002, start + block_s), block, swing_deg)
                    continue
                if hz and time.monotonic() >= next_cmd:
                    command()
                    next_cmd += 1.0 / hz
                poll_until(min(next_cmd if hz else start + block_s, start + block_s), block, swing_deg)
            block["wall_end"] = round(time.time(), 4)
            record["blocks"].append(block)
            if swing_deg:
                client.set_all_joint_angles(pose, mode=0, gripper=gripper)  # back to the middle, not counted
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
