#!/usr/bin/env python3
"""A kinematic stand-in for the Go2 on ROS 2, to run `press_live.py --live` on the PC: never on the dog.

It publishes `/lowstate` at 500 Hz and answers what the press program sends. Sport mode holds a stand until StandDown
lays it down. The motion switcher releases and selects modes, answering CheckMode with the mode's name. `/lowcmd`'s
leg targets are followed at a rate set by kp (no dynamics, no gravity), and under damping the joints sink to lying.
Every LowCmd's CRC is checked, and any LowCmd sent while sport mode holds the legs is counted. With `--hand` the FR
foot stops at the palm_up plate (`press_layer.PLATES`), so the press layer's stall can be seen.

    ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=77 python3 demos/combiner_key/robot/fake_go2.py --hand --out <summary.json>

What it checks is the plumbing: message fields, QoS matching, CRCs, the order of requests. It says nothing about what
the policy does to a real robot.
"""
from __future__ import annotations

import argparse
import json
import signal
import sys
import threading
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

from demos.combiner_key.robot import leg_policy, lowlevel as ll, press_layer as pl, press_live  # noqa: E402

#: The settled stance (`leg_cfg.SETTLED_LEG_POS`) in Unitree's order, the thighs 0.05 rad further bent: sport mode's.
STAND_Q = np.array([-0.1108, 0.7934, -1.8007, 0.1112, 0.7931, -1.8014, -0.1037, 1.0405, -1.8324, 0.1045, 1.0405, -1.8329])
#: The dog lying in sport mode (its LowState, 2026-10-23): the calves past the URDF's -2.7227 rad.
LYING_Q = np.array([-0.046, 1.253, -2.79, 0.081, 1.247, -2.78, -0.346, 1.271, -2.805, 0.302, 1.295, -2.847])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--hand", action="store_true", help="the FR foot stops at the palm_up plate")
    parser.add_argument("--policy", default=str(REPO / "checkpoints" / "leg_press_force_r4_model_5999.npz"),
                        help="for its kinematics only")
    parser.add_argument("--out", default=None, help="write the summary here at exit")
    args = parser.parse_args()

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from unitree_api.msg import Request, Response
    from unitree_go.msg import LowCmd, LowState

    meta = leg_policy.LegPolicy(args.policy).meta
    # Gravity as the policy's PD meets it: at kp 25 the default targets settle into the trained settled stance; at other
    # gains the joints sag by that torque over kp.
    obs = leg_policy.LegObservation(meta)
    load = 25.0 * (np.asarray(meta["settled_leg_pos"])[obs.to_unitree] - obs.default[:12][obs.to_unitree])
    plate = pl.PLATES["palm_up"] if args.hand else None
    rclpy.init()
    node = rclpy.create_node("fake_go2")
    lock = threading.Lock()
    s = {"q": STAND_Q.copy(), "mode": "normal", "lying": False, "target": None, "kp": np.zeros(12), "kd": np.zeros(12),
         "lowcmd": 0, "bad_crc": 0, "while_sport": 0, "requests": [], "stopped_at_hand": 0}
    state_pub = node.create_publisher(LowState, "/lowstate", qos_profile_sensor_data)
    resp_pub = node.create_publisher(Response, ll.MOTION_SWITCHER_RESPONSE, 10)

    def on_lowcmd(msg) -> None:
        with lock:
            s["lowcmd"] += 1
            if ll.lowcmd_crc(msg) != msg.crc:
                s["bad_crc"] += 1
                return
            if s["mode"]:
                s["while_sport"] += 1
                return
            s["target"] = np.array([msg.motor_cmd[i].q for i in range(12)])
            s["kp"] = np.array([msg.motor_cmd[i].kp for i in range(12)])
            s["kd"] = np.array([msg.motor_cmd[i].kd for i in range(12)])

    def on_switch(msg) -> None:
        api = int(msg.header.identity.api_id)
        with lock:
            if api == ll.RELEASE_MODE:
                s["mode"] = ""
            elif api == ll.SELECT_MODE:
                s["mode"] = json.loads(msg.parameter)["name"]
                s["lying"] = True            # sport mode takes the legs where they are
            s["requests"].append(("switcher", api))
            name = s["mode"]
        out = Response()
        out.header.identity.id, out.header.identity.api_id = msg.header.identity.id, api
        out.header.status.code = 0
        out.data = json.dumps({"name": name, "form": "0"})
        resp_pub.publish(out)

    def on_sport(msg) -> None:
        api = int(msg.header.identity.api_id)
        with lock:
            s["requests"].append(("sport", api))
            if api == ll.SPORT_STAND_DOWN and s["mode"]:
                s["lying"] = True

    node.create_subscription(LowCmd, "/lowcmd", on_lowcmd, 10)
    node.create_subscription(Request, ll.MOTION_SWITCHER_REQUEST, on_switch, 10)
    node.create_subscription(Request, ll.SPORT_REQUEST, on_sport, 10)

    dt = 0.002
    last_q = s["q"].copy()

    def step() -> None:
        nonlocal last_q
        with lock:
            q = s["q"]
            before = q.copy()
            if s["mode"]:
                if s["lying"]:
                    q += (LYING_Q - q) * 0.01
            elif s["target"] is not None and s["kp"].max() > 0:
                q += (s["target"] + load / np.maximum(s["kp"], 1e-6) - q) * np.clip(s["kp"] / 25.0 * 0.03, 0.0, 0.5)
            else:
                q += (LYING_Q - q) * 0.002
            if plate is not None:
                # A palm 12 x 10 cm: it stops a foot coming down onto it from above, nothing else.
                now_f = press_live.feet(q, (1.0, 0.0, 0.0, 0.0), meta)[0]["FR"]
                was_f = press_live.feet(before, (1.0, 0.0, 0.0, 0.0), meta)[0]["FR"]
                over = abs(now_f[0] - plate.contact[0]) < 0.06 and abs(now_f[1] - plate.contact[1]) < 0.05
                if over and (was_f - plate.contact) @ plate.n >= 0.0 > (now_f - plate.contact) @ plate.n:
                    q[0:3] = before[0:3]
                    s["stopped_at_hand"] += 1
            dq = (q - last_q) / dt
            last_q = q.copy()
        msg = LowState()
        for i in range(12):
            msg.motor_state[i].q, msg.motor_state[i].dq = float(q[i]), float(dq[i])
        msg.imu_state.quaternion = [1.0, 0.0, 0.0, 0.0]
        msg.foot_force = [20, 20, 20, 20]
        state_pub.publish(msg)

    node.create_timer(dt, step)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        with lock:
            summary = {k: v for k, v in s.items() if k not in ("q", "target", "kp", "kd")}
            summary["final_q"] = np.round(s["q"], 3).tolist()
        print(json.dumps(summary), flush=True)
        if args.out:
            Path(args.out).write_text(json.dumps(summary, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
