#!/usr/bin/env python3
"""Log the Go2's leg torques from LowState, to settle what the knee delivers. READ-ONLY: it sends nothing to the robot.

The leg policies' simulators disagree about the knee (2026-10-21, `unifp_train/leg_actuators.py`): Isaac Lab's stock Go2
model (rima's, P2Dingo's) and unitree_rl_lab cap it at the hip motor's ~23.5 N·m, while Unitree's URDF gives it 45.43 N·m
at half the hip's speed, a geared knee. In the key task the support legs' knees work at 18-23 N·m standing on three legs
and reach ~33 N·m in brief peaks, so which is right decides whether the leg policy transfers.

Run it on the dog (`run_on_dog.sh` starts it in P2Dingo's ROS image) while the robot is in sport mode, driven from the
Unitree remote as usual:

1. lying down, then stand up (the stand-up loads the knees most);
2. standing still for ~10 s (the unit check below);
3. standing, someone presses down on the back with both hands, steadily, then harder; or a known load on the back;
4. sit down.

What it reports, every second and at the end (`summary.json` beside `lowstate.csv` in the run directory):

- each joint's largest |tau_est| and its 99th percentile, and how many samples the knees spent past 23.5 N·m. A knee
  clearly past 23.5 (30+) under load means it delivers more than Isaac Lab's cap; the geared model is then the closer.
- **a unit check while standing still**: the knee torque the robot's weight needs, from the measured joint angles, the
  body's pitch and an equal share of `--mass_kg` per foot (sagittal plane, leg masses and dynamics ignored), against
  the knees' tau_est. A ratio near 1 says tau_est is the joint's torque; near 0.52 (1 / 1.917), the motor's before the
  knee's reduction. Read the peaks only after this check: a motor-side tau_est would hide a geared knee.

`--mass_kg` is the dog as it stands, arm, mounts, sensors and battery included: weigh it (it also checks the simulator's
mass). Unitree's `tau_est` is the firmware's estimate, not a torque sensor; the check above is what makes it usable.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path

import numpy as np

#: LowState.motor_state order (fixed by the firmware).
JOINTS = [f"{leg}_{part}" for leg in ("FR", "FL", "RR", "RL") for part in ("hip", "thigh", "calf")]
KNEES = [i for i, n in enumerate(JOINTS) if n.endswith("calf")]
#: Go2 thigh and calf lengths, m (`description/go2_d1.urdf`: the calf joint 0.213 below the thigh's, the foot 0.213 below).
THIGH_M, CALF_M = 0.213, 0.213
STOCK_KNEE_NM = 23.5
KNEE_RATIO = 45.43 / 23.7
#: Standing still: every joint slower than this (rad/s) for the unit check.
STILL_RAD_S = 0.05


def static_knee_torque(thigh: float, calf: float, pitch: float, foot_n: float) -> float:
    """|knee torque| holding `foot_n` of vertical ground force at the foot, in the leg's sagittal plane.

    Go2 joint convention: thigh 0 points the leg straight down, positive swings the foot back; calf negative folds it.
    The body pitched by `pitch` (nose down positive, as LowState's rpy) tilts both by it relative to gravity.
    """
    a = thigh + calf + pitch                      # the calf's angle from the vertical
    return abs(CALF_M * math.sin(a) * foot_n)     # the foot's horizontal offset from the knee times the force


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--mass_kg", type=float, required=True, help="the dog as it stands, weighed (kg)")
    parser.add_argument("--topic", default="/lowstate", help="LowState topic: /lowstate (full rate) or /lf/lowstate")
    parser.add_argument("--run_dir", default="logs/go2_knee")
    parser.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0: Ctrl-C)")
    args = parser.parse_args()

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from unitree_go.msg import LowState

    run_dir = Path(args.run_dir) / f"{time.strftime('%Y%m%dT%H%M%S', time.gmtime())}_go2_knee"   # UTC, as every run
    run_dir.mkdir(parents=True, exist_ok=True)
    out = (run_dir / "lowstate.csv").open("w", newline="")
    writer = csv.writer(out)
    writer.writerow(["t"] + [f"{n}_{k}" for k in ("q", "dq", "tau") for n in JOINTS]
                    + ["roll", "pitch"] + [f"foot_force_{leg}" for leg in ("FR", "FL", "RR", "RL")])
    peak = np.zeros(12)
    taus: list[np.ndarray] = []
    over = np.zeros(12, dtype=int)
    ratios: list[float] = []
    start = time.monotonic()
    last_print = [start]

    def on_state(msg) -> None:
        now = time.monotonic() - start
        q = np.array([msg.motor_state[i].q for i in range(12)])
        dq = np.array([msg.motor_state[i].dq for i in range(12)])
        tau = np.array([msg.motor_state[i].tau_est for i in range(12)])
        roll, pitch = float(msg.imu_state.rpy[0]), float(msg.imu_state.rpy[1])
        writer.writerow([f"{now:.4f}"] + [f"{v:.4f}" for v in np.concatenate((q, dq, tau))]
                        + [f"{roll:.4f}", f"{pitch:.4f}"] + [int(v) for v in msg.foot_force])
        np.maximum(peak, np.abs(tau), out=peak)
        taus.append(np.abs(tau))
        over[:] += np.abs(tau) > STOCK_KNEE_NM
        if np.all(np.abs(dq) < STILL_RAD_S) and abs(roll) < 0.1:
            need = sum(static_knee_torque(q[k - 1], q[k], pitch, args.mass_kg * 9.81 / 4) for k in KNEES)
            if need > 1.0:
                ratios.append(float(np.abs(tau[KNEES]).sum() / need))
        if now - last_print[0] >= 1.0:
            last_print[0] = now
            knees = "  ".join(f"{JOINTS[k][:2]} {abs(tau[k]):5.1f}/{peak[k]:5.1f}" for k in KNEES)
            check = f"unit check {np.median(ratios):.2f} ({len(ratios)} still samples)" if ratios else "unit check: stand still"
            print(f"[{now:6.1f} s] knee |tau| now/max N·m: {knees}   past 23.5: {int(over[KNEES].sum())}   {check}",
                  flush=True)

    rclpy.init()
    node = rclpy.create_node("go2_knee_log")
    node.create_subscription(LowState, args.topic, on_state, qos_profile_sensor_data)
    print(f"logging {args.topic} to {run_dir} (read-only); Ctrl-C to stop", flush=True)
    try:
        while rclpy.ok() and (not args.seconds or time.monotonic() - start < args.seconds):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        out.close()
        samples = np.array(taus) if taus else np.zeros((0, 12))
        summary = {
            "topic": args.topic, "mass_kg": args.mass_kg, "samples": len(taus),
            "seconds": round(time.monotonic() - start, 1),
            "peak_abs_tau_nm": dict(zip(JOINTS, np.round(peak, 2).tolist())),
            "p99_abs_tau_nm": (dict(zip(JOINTS, np.round(np.quantile(samples, 0.99, axis=0), 2).tolist()))
                               if len(samples) else None),
            "knee_samples_past_23_5_nm": {JOINTS[k]: int(over[k]) for k in KNEES},
            "unit_check": {"median_ratio": round(float(np.median(ratios)), 3) if ratios else None,
                           "still_samples": len(ratios),
                           "reads": "~1: tau_est is the joint's torque; ~0.52: the motor's, before the knee's reduction"},
        }
        (run_dir / "summary.json").write_text(json.dumps(summary, indent=2))
        (run_dir / "run.json").write_text(json.dumps({"mode": "go2_knee_log", "status": "recorded", "published": "nothing",
                                                      "arguments": vars(args), "summary": summary}, indent=2))
        print(json.dumps(summary, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
