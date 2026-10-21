#!/usr/bin/env python3
"""The key-turning leg policy in shadow mode on the dog. READ-ONLY: it publishes nothing and commands nothing.

Start it with `run_shadow_on_dog.sh` while the robot stands in sport mode (the classic stand), driven from the Unitree
remote as usual. Every 20 ms it builds the policy's observation from the robot's own state (`/lowstate`: the legs, the
IMU) and the D1's (the arm link's feedback, read only; the rest pose if there is none), runs the exported policy
(`leg_policy.py`, numpy) as at the start of a frozen key episode -- an empty history, the foot told to stay on its
footfall, no force -- and records what it would have done: its 12 actions, the joint targets they mean, the torque
kp 25 / kd 0.5 would ask for at the measured joints, and its estimator's view of the base's speed, the foot and the
forces on it.

The sport controller keeps the robot standing; the policy never moves it, so this is open loop: what it checks is the
observation (joint order, signs and offsets as the firmware reports them, the IMU's frame, the rates) and what the
policy makes of the real robot standing still. `check_shadow.py --dog <record>` reads the record on the PC: the frames
against the training code's own, the actions against the checkpoint's in torch, and against what the policy gives a
still robot in the same pose (which is what it should give here, if the observation is right).

    run_shadow_on_dog.sh --policy checkpoints/leg_key_force_s2walker_model_2104.npz
    run_shadow_on_dog.sh --policy ... --seconds 60 --foot_goal 0.30 -0.17 0.12   # the lifting regime (open loop)

The record is `logs/leg_shadow/<stamp>/` (`shadow.npz`, `summary.json`); `d1fetch` copies it back.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import threading
import time
from pathlib import Path

# One BLAS thread, set before numpy loads: the network is a few small matrix products, and a threaded BLAS spinning up
# beside the 500 Hz LowState callbacks took single steps past the 20 ms period (p99 30 ms on the PC, 0.7 ms median).
for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import numpy as np  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

from demos.combiner_key.robot import leg_policy  # noqa: E402

#: A LowState older than this when the policy steps marks the step stale; this long stale restarts the policy's history.
STALE_S, RESTART_S = 0.1, 0.5
#: Arm feedback older than this is not used (the rest pose is).
ARM_STALE_S = 1.0
ARM_LINK_PORT = 8095


def log(msg: str) -> None:
    print(f"[shadow] {msg}", flush=True)


class ArmFeedback:
    """The D1's joint angles from the arm link (`demos/combiner_door/robot/arm_link.py`, started with `--observe`).

    Listens only: this client never writes to the socket, so the link gets no target, keepalive or hold from it."""

    def __init__(self, port: int):
        self.q_rad, self.rx, self.hello, self.lock = None, 0.0, None, threading.Lock()
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=2.0)
        self.sock.settimeout(None)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        import d1_ik

        buffer = b""
        while True:
            try:
                chunk = self.sock.recv(65536)
            except OSError:
                return
            if not chunk:
                return
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                try:
                    msg = json.loads(line)
                except ValueError:
                    continue
                if msg.get("type") == "feedback" and msg.get("q") is not None:
                    q = np.asarray(d1_ik.from_servo_deg(np.asarray(msg["q"], dtype=float))[0], dtype=float)
                    with self.lock:
                        self.q_rad, self.rx = q, time.monotonic() - float(msg.get("age_s", 0.0))
                elif msg.get("type") == "hello":
                    self.hello = msg

    def latest(self):
        with self.lock:
            return (None if self.q_rad is None else self.q_rad.copy()), time.monotonic() - self.rx


class Shadow:
    def __init__(self, args):
        self.args = args
        self.policy = leg_policy.LegPolicy(args.policy)
        self.meta = self.policy.meta
        self.obs = leg_policy.LegObservation(self.meta)
        self.waiting = leg_policy.waiting_command(self.meta, args.foot_goal)
        self.kp, self.kd = float(self.meta["leg_kp"]), float(self.meta["leg_kd"])
        self.limits = np.asarray(self.meta["leg_torque_limits"], dtype=float)[self.obs.to_unitree]
        self.default_u = self.obs.default[:12][self.obs.to_unitree]
        self.rest = np.asarray(self.meta["arm_rest_pose"], dtype=float)
        self.state, self.state_rx, self.lock = None, 0.0, threading.Lock()
        self.arm = None
        if args.arm_link_port:
            try:
                self.arm = ArmFeedback(args.arm_link_port)
                log(f"arm link on 127.0.0.1:{args.arm_link_port} (listening only)")
            except OSError as error:
                log(f"no arm link ({error}): the arm is taken at its rest pose {np.round(self.rest, 3).tolist()} rad")
        self.rows: dict[str, list] = {k: [] for k in (
            "t", "dt", "state_age", "reset", "stale", "q", "dq", "tau_est", "quat", "gyro", "rpy", "foot_force",
            "arm_q", "arm_measured", "commands", "frame", "action", "target", "torque", "estimate", "infer_ms")}
        self.fresh, self.stale_since, self.start, self.last_tick = True, None, None, None
        # A UTC stamp first, as every run directory here (the dashboard dates a record by it).
        self.run_dir = Path(args.run_dir) / f"{time.strftime('%Y%m%dT%H%M%S', time.gmtime())}_leg_shadow"
        self.run_dir.mkdir(parents=True, exist_ok=True)

    # --- input ---------------------------------------------------------------------------------------------------

    def on_state(self, msg) -> None:
        state = {"q": np.array([msg.motor_state[i].q for i in range(12)]),
                 "dq": np.array([msg.motor_state[i].dq for i in range(12)]),
                 "tau_est": np.array([msg.motor_state[i].tau_est for i in range(12)]),
                 "quat": np.array(msg.imu_state.quaternion, dtype=float),        # w, x, y, z
                 "gyro": np.array(msg.imu_state.gyroscope, dtype=float),         # body frame, rad/s
                 "rpy": np.array(msg.imu_state.rpy, dtype=float),
                 "foot_force": np.array(msg.foot_force, dtype=float)}
        with self.lock:
            self.state, self.state_rx = state, time.monotonic()

    # --- the policy step -------------------------------------------------------------------------------------------

    def tick(self) -> None:
        now = time.monotonic()
        with self.lock:
            state, age = self.state, now - self.state_rx
        if state is None:
            return
        if self.start is None:
            self.start = now
            log("first LowState: the policy starts from an empty history, as at a training reset")
        stale = age > STALE_S
        if stale:
            self.stale_since = self.stale_since or now
            if now - self.stale_since > RESTART_S and not self.fresh:
                log(f"LowState {age:.2f} s old: the history will restart when it returns")
                self.fresh = True
            return
        self.stale_since = None
        reset = self.fresh
        if reset:
            self.obs.reset()
            self.fresh = False
        arm_q, arm_age = (self.arm.latest() if self.arm is not None else (None, math.inf))
        measured = arm_q is not None and arm_age < ARM_STALE_S
        arm_q = arm_q if measured else self.rest
        # The reset's own frame has all-zero commands, as the environment builds it (leg_policy.waiting_command).
        commands = np.zeros_like(self.waiting) if reset else self.waiting
        frame = self.obs.frame(state["quat"], state["gyro"], state["q"], state["dq"], arm_q, np.zeros(6), commands)
        stack = self.obs.step(frame)
        t0 = time.perf_counter()
        action = self.policy.act(stack)[0]
        estimate = self.policy.estimate(stack)[0]
        infer_ms = 1000.0 * (time.perf_counter() - t0)
        legs = self.obs.record_action(action)
        target = self.obs.leg_targets_unitree(legs)
        torque = np.clip(self.kp * (target - state["q"]) - self.kd * state["dq"], -self.limits, self.limits)
        row = {"t": now - self.start, "dt": (now - self.last_tick) if self.last_tick else 0.0, "state_age": age,
               "reset": float(reset), "stale": float(stale), "arm_q": arm_q, "arm_measured": float(measured),
               "commands": commands, "frame": frame, "action": legs, "target": target, "torque": torque,
               "estimate": estimate, "infer_ms": infer_ms, **state}
        for key, value in row.items():
            self.rows[key].append(np.asarray(value, dtype=float))
        self.last_tick = now
        if len(self.rows["t"]) % max(1, round(self.args.print_s / self.meta["policy_dt"])) == 0:
            self.print_status()

    # --- reporting -------------------------------------------------------------------------------------------------

    def window(self, key: str, seconds: float = 1.0) -> np.ndarray:
        n = max(1, round(seconds / self.meta["policy_dt"]))
        return np.array(self.rows[key][-n:])

    def print_status(self) -> None:
        q, target, torque, tau = self.window("q"), self.window("target"), self.window("torque"), self.window("tau_est")
        gap = np.abs(target - q).max(axis=0)
        worst = int(np.argmax(gap))
        est = leg_policy.decode_estimates(self.meta, self.window("estimate").mean(axis=0))
        # Roll and pitch as the policy sees them (its frame's first two values: UniFP's euler_from_quat, pitch positive
        # nose down), not the IMU's own rpy, which the record keeps too.
        rp = np.degrees(self.window("frame")[:, 0:2].mean(axis=0))
        log(f"t {self.rows['t'][-1]:6.1f} s  roll {rp[0]:+5.1f} pitch {rp[1]:+5.1f} deg  | largest target-q "
            f"{gap[worst]:.3f} rad at {leg_policy.UNITREE_LEGS[worst]}  | would-be knee torques "
            f"{np.round(np.abs(torque[:, 2::3]).max(axis=0), 1).tolist()} N·m (sport mode's tau_est "
            f"{np.round(np.abs(tau[:, 2::3]).mean(axis=0), 1).tolist()})  | est. base v "
            f"{np.round(est['base_lin_vel_b'], 2).tolist()} m/s, foot force {np.round(est['foot_force'], 1).tolist()} N  | "
            f"{np.mean(self.window('infer_ms')):.1f} ms, arm {'measured' if self.rows['arm_measured'][-1] else 'rest pose'}")

    def save(self) -> Path:
        arrays = {key: np.array(values) for key, values in self.rows.items() if values}
        if not arrays:
            log("nothing recorded (no LowState arrived)")
            return self.run_dir
        np.savez_compressed(self.run_dir / "shadow.npz", **arrays)
        summary = self.summary(arrays)
        (self.run_dir / "summary.json").write_text(json.dumps(summary, indent=2))
        # The dashboard's run record (`./dashboard.py record <this directory>`).
        (self.run_dir / "run.json").write_text(json.dumps(
            {"mode": "leg_shadow", "status": "recorded", "published": "nothing", "arguments": vars(self.args),
             "policy_meta": {k: self.meta[k] for k in ("variant", "checkpoint", "checkpoint_sha256", "checkpoint_iteration")},
             "summary": summary}, indent=2, default=str))
        return self.run_dir

    def summary(self, a: dict) -> dict:
        names = leg_policy.UNITREE_LEGS
        per = lambda x, f: dict(zip(names, np.round(f(x), 4).tolist()))
        gap = a["target"] - a["q"]
        est = [leg_policy.decode_estimates(self.meta, e) for e in a["estimate"]]
        fk = np.array([leg_policy.foot_local("FR", q[0:3], self.meta) for q in a["q"]])
        return {
            "policy": self.args.policy, "policy_variant": self.meta["variant"],
            "policy_checkpoint": self.meta["checkpoint"], "topic": self.args.topic,
            "foot_goal_local": self.args.foot_goal, "published": "nothing",
            "steps": len(a["t"]), "seconds": round(float(a["t"][-1]), 2), "resets": int(a["reset"].sum()),
            "step_period_s": {"median": round(float(np.median(a["dt"][1:])), 4) if len(a["dt"]) > 1 else None,
                              "p99": round(float(np.quantile(a["dt"][1:], 0.99)), 4) if len(a["dt"]) > 1 else None},
            "state_age_s_p99": round(float(np.quantile(a["state_age"], 0.99)), 4),
            "infer_ms": {"median": round(float(np.median(a["infer_ms"])), 2),
                         "p99": round(float(np.quantile(a["infer_ms"], 0.99)), 2)},
            "arm_measured_share": round(float(a["arm_measured"].mean()), 3),
            "rpy_deg_median": np.round(np.degrees(np.median(a["rpy"], axis=0)), 2).tolist(),
            "q_minus_default_median_rad": per(a["q"] - self.default_u, lambda x: np.median(x, axis=0)),
            "target_minus_q_median_rad": per(gap, lambda x: np.median(x, axis=0)),
            "target_minus_q_max_abs_rad": per(gap, lambda x: np.abs(x).max(axis=0)),
            "would_be_torque_median_nm": per(a["torque"], lambda x: np.median(x, axis=0)),
            "sport_mode_tau_est_median_nm": per(a["tau_est"], lambda x: np.median(x, axis=0)),
            "estimates_median": {
                "base_lin_vel_b_m_s": np.round(np.median([e["base_lin_vel_b"] for e in est], axis=0), 3).tolist(),
                "fr_foot_local_m": np.round(np.median([e["foot_local"] for e in est], axis=0), 3).tolist(),
                "fr_foot_force_n": np.round(np.median([e["foot_force"] for e in est], axis=0), 2).tolist(),
                "base_force_n": np.round(np.median([e["base_force"] for e in est], axis=0), 2).tolist()},
            "fr_foot_from_joints_base_frame_m": np.round(np.median(fk, axis=0), 3).tolist(),
            "read_with": "demos/combiner_key/robot/check_shadow.py --dog <this directory>",
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--policy", required=True, help="an exported policy (.npz, export_policy.py)")
    parser.add_argument("--topic", default="/lowstate", help="LowState topic: /lowstate (full rate) or /lf/lowstate")
    parser.add_argument("--arm_link_port", type=int, default=ARM_LINK_PORT, help="0: no arm link, the rest pose")
    parser.add_argument("--foot_goal", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                        help="the foot's goal in the yaw frame from the ground under the base (m), instead of its "
                             "footfall: the policy's lifting regime on the real observation, for check_shadow's "
                             "still-robot comparison (open loop; the targets are not where the foot would go)")
    parser.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0: Ctrl-C)")
    parser.add_argument("--print_s", type=float, default=1.0)
    parser.add_argument("--run_dir", default="logs/leg_shadow")
    args = parser.parse_args()

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from unitree_go.msg import LowState

    shadow = Shadow(args)
    log(f"SHADOW MODE: nothing is sent to the robot. {shadow.meta['variant']} policy "
        f"{Path(shadow.meta['checkpoint']).parent.name}/{Path(shadow.meta['checkpoint']).name}, the foot's goal "
        f"{'its footfall' if args.foot_goal is None else args.foot_goal}; recording to {shadow.run_dir}")
    rclpy.init()
    node = rclpy.create_node("leg_shadow")
    node.create_subscription(LowState, args.topic, shadow.on_state, qos_profile_sensor_data)
    # The policy's 50 Hz is kept here, not by a ROS timer, so the 500 Hz LowState callbacks cannot delay it: spin for
    # whatever is ready until the next step is due, then step (on a fixed grid, so the period does not drift).
    dt = float(shadow.meta["policy_dt"])
    next_tick = last_save = time.monotonic()
    try:
        while rclpy.ok() and (not args.seconds or shadow.start is None or time.monotonic() - shadow.start < args.seconds):
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.002, next_tick - time.monotonic())))
            now = time.monotonic()
            if now >= next_tick:
                shadow.tick()
                next_tick += dt
                if now - next_tick > 5 * dt:      # far behind (a stall): start the grid again rather than burst
                    next_tick = now + dt
            if now - last_save > 30.0:
                shadow.save()
                last_save = now
    except KeyboardInterrupt:
        pass
    finally:
        out = shadow.save()
        log(f"record: {out}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
