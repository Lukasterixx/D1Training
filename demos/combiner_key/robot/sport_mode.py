#!/usr/bin/env python3
"""The Go2's sport mode through the motion switcher: report it, or select one (`--select mcf`).

What `press_live.py`'s handback does, on its own: for a run that ended with sport mode released (CheckMode reporting
no name), when the remote stops answering. Selecting a mode gives it the legs; it may stand the dog up. Run in
P2Dingo's ROS image on the dog (`d1dogsport` in ~/.bashrc).

    python3 demos/combiner_key/robot/sport_mode.py                 # CheckMode only
    python3 demos/combiner_key/robot/sport_mode.py --select mcf    # SelectMode, then CheckMode
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from demos.combiner_key.robot import lowlevel as ll  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--select", default=None, help="the mode to select (the dog's is 'mcf')")
    args = parser.parse_args()

    import rclpy
    from unitree_api.msg import Request, Response

    rclpy.init()
    node = rclpy.create_node("sport_mode")
    got: dict[int, tuple[int, str]] = {}
    node.create_subscription(Response, ll.MOTION_SWITCHER_RESPONSE,
                             lambda m: got.__setitem__(int(m.header.identity.id), (int(m.header.status.code), m.data)), 10)
    pub = node.create_publisher(Request, ll.MOTION_SWITCHER_REQUEST, 10)
    ident = int(time.time() * 1000) % 2 ** 40

    def call(api_id: int, parameter: dict | None = None, wait: float = 5.0):
        nonlocal ident
        ident += 1
        req = Request()
        ll.fill_request(req, api_id, ident, parameter)
        t0, resend = time.time(), 0.0
        while time.time() - t0 < wait and ident not in got:
            if time.time() - t0 >= resend:
                pub.publish(req)
                resend += 1.5
            rclpy.spin_once(node, timeout_sec=0.05)
        return got.get(ident)

    t0 = time.time()
    while time.time() - t0 < 1.0:                 # discovery before the first request
        rclpy.spin_once(node, timeout_sec=0.05)
    before = call(ll.CHECK_MODE)
    print(f"CheckMode: {before and repr(ll.mode_name(before[1]))} {'(released)' if before and not ll.mode_name(before[1]) else ''}")
    ok = before is not None
    if args.select:
        result = call(ll.SELECT_MODE, {"name": args.select}, wait=6.0)
        print(f"SelectMode {args.select!r}: {'answered, code %d' % result[0] if result else 'no answer'}")
        time.sleep(1.0)
        after = call(ll.CHECK_MODE)
        name = ll.mode_name(after[1]) if after else None
        print(f"CheckMode: {name!r}")
        ok = name == args.select
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
