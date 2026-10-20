#!/usr/bin/env python3
"""The combiner door demo: find the box with the head RealSense, walk in, grip the handle above the AprilTag and walk
the door open with the walking policy and arm IK, while the MID-360 checks the door's angle. See README.md.

    ./demos/combiner_door/run_combiner_door.py                          # watch one attempt (one scene)
    ./demos/combiner_door/run_combiner_door.py --attempts 16 --headless  # measure: the door pull's 16 placements

It is the door pull's runner (`demos/unifp/run_demo.py`) with `--task door --controller decoupled --perception head`
set, so its placements, script, trace and summary are the door pull's own; any of that runner's options can be added.
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

DEMO_ARGS = ["--task", "door", "--controller", "decoupled", "--perception", "head"]
#: The spawn's settle, seconds: long enough for the walking policy to have landed and stopped turning (P2Dingo's
#: turns ~15 deg over its first 3-4 s), after which the attempt is anchored where it stands (`env._anchor`).
SETTLE_S = 4.0


def main() -> int:
    from demos.unifp import run_demo

    given = sys.argv[1:]
    named = any(a == "--run_name" or a.startswith("--run_name=") for a in given)
    settled = any(a == "--settle_s" or a.startswith("--settle_s=") for a in given)
    sys.argv = ([sys.argv[0]] + DEMO_ARGS + ([] if named else ["--run_name", "combiner_door"])
                + ([] if settled else ["--settle_s", str(SETTLE_S)]) + given)
    return run_demo.main()


if __name__ == "__main__":
    raise SystemExit(main())
