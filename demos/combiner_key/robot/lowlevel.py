"""The Go2's low-level interface for a program that drives the legs: `LowCmd` and its CRC, the remote's buttons, the
motion switcher, and the ramps in and out of the policy's gains. Plain Python and numpy: the ROS message objects are
passed in, never imported here, so `tests/test_press_live.py` runs it without ROS.

Sources:
  * `LowCmd`'s CRC is `unitree_ros2`'s `example/src/src/common/motor_crc.cpp` (BSD-3-Clause, Unitree Robotics): a
    CRC-32 (polynomial 0x04C11DB7, initial 0xFFFFFFFF, no reflection, no final xor) over the C struct's 32-bit words but
    the last. Its word loop xors the polynomial in twice for a set data bit where the textbook loop does it once; the
    two agree, so it is CRC-32/MPEG-2 over each little-endian word's bytes in big-endian order (`crc32_core_unitree`,
    the line-for-line port, is kept for the test).
  * The motion switcher's API ids (1001 check, 1002 select, 1003 release) and its request and response topics are
    `unitree_ros2`'s `g1_motion_switch_client.hpp`; the Go2's sport API ids are those P2Dingo and the door program use.
  * The remote's 40 bytes in `LowState.wireless_remote` are Unitree's `xRockerBtnDataStruct` (2 header bytes, a
    16-bit key field, then the sticks); the key bits are `unitree_legged_sdk`'s `joystick.h`. Not yet read on this dog:
    `press_live.py`'s dry run prints the buttons it decodes, to check them before a live run.
  * The stand-up and damping gains are unitree_rl_lab's Go2 deploy config (`deploy/robots/go2/config/config.yaml`,
    Apache-2.0): FixStand kp 60/80/80, kd 5/4/4; Passive kd 3.
"""
from __future__ import annotations

import json
import struct

import numpy as np

# --- LowCmd ----------------------------------------------------------------------------------------------------------

#: `LowCmd`'s fixed fields for the Go2 (`motor_crc.h`, unitree_ros2's go2 low-level example).
HEAD = (0xFE, 0xEF)
LEVEL_FLAG = 0xFF
#: A motor in servo (FOC) mode; an unused one is sent `POS_STOP` / `VEL_STOP` with no gains, as the examples do.
MODE_SERVO = 0x01
POS_STOP, VEL_STOP = 2.146e9, 16000.0
NUM_MOTORS = 20

# The C struct, little-endian with its natural alignment (812 bytes): head[2] u8, levelFlag u8, frameReserve u8,
# SN[2] u32, version[2] u32, bandWidth u16 + 2 pad, motorCmd[20] {mode u8 + 3 pad, q dq tau Kp Kd f32, reserve[3] u32},
# bms {off u8, reserve[3] u8}, wirelessRemote[40] u8, led[12] u8, fan[2] u8, gpio u8 + 1 pad, reserve u32, crc u32.
_HEADER = struct.Struct("<2BBB2I2IH2x")
_MOTOR = struct.Struct("<B3x5f3I")
_TAIL = struct.Struct("<B3B40B12B2BBxII")
LOWCMD_BYTES = _HEADER.size + NUM_MOTORS * _MOTOR.size + _TAIL.size
assert LOWCMD_BYTES == 812, LOWCMD_BYTES


def _crc_table() -> list[int]:
    table = []
    for byte in range(256):
        crc = byte << 24
        for _ in range(8):
            crc = ((crc << 1) ^ 0x04C11DB7) if crc & 0x80000000 else (crc << 1)
        table.append(crc & 0xFFFFFFFF)
    return table


_TABLE = _crc_table()


def crc32_words(words) -> int:
    """`motor_crc.cpp`'s `crc32_core` over 32-bit words, table-driven (see the module docstring)."""
    crc = 0xFFFFFFFF
    for word in words:
        for shift in (24, 16, 8, 0):
            crc = ((crc << 8) & 0xFFFFFFFF) ^ _TABLE[((crc >> 24) ^ (word >> shift)) & 0xFF]
    return crc


def crc32_core_unitree(words) -> int:
    """`motor_crc.cpp`'s `crc32_core`, line for line (slow; the test's reference)."""
    crc = 0xFFFFFFFF
    for data in words:
        xbit = 1 << 31
        for _ in range(32):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
            if data & xbit:
                crc ^= 0x04C11DB7
            xbit >>= 1
    return crc


def pack_lowcmd(msg) -> bytes:
    """A `unitree_go/msg/LowCmd` as the C struct `get_crc` builds (its `crc` field as it is)."""
    out = [_HEADER.pack(*msg.head, msg.level_flag, msg.frame_reserve, *msg.sn, *msg.version, msg.bandwidth)]
    for m in msg.motor_cmd:
        out.append(_MOTOR.pack(m.mode, m.q, m.dq, m.tau, m.kp, m.kd, *m.reserve))
    out.append(_TAIL.pack(msg.bms_cmd.off, *msg.bms_cmd.reserve, *msg.wireless_remote, *msg.led, *msg.fan, msg.gpio,
                          msg.reserve, msg.crc))
    return b"".join(out)


def lowcmd_crc(msg) -> int:
    raw = pack_lowcmd(msg)
    words = struct.unpack(f"<{len(raw) // 4}I", raw)
    return crc32_words(words[:-1])


def init_lowcmd(msg) -> None:
    """The fixed fields, every motor in servo mode at its stop values with no gains (`init_cmd` in the examples)."""
    msg.head = list(HEAD)
    msg.level_flag = LEVEL_FLAG
    msg.gpio = 0
    for m in msg.motor_cmd:
        m.mode, m.q, m.dq, m.tau, m.kp, m.kd = MODE_SERVO, POS_STOP, VEL_STOP, 0.0, 0.0, 0.0


def fill_legs(msg, q, kp, kd, dq=None, tau=None) -> None:
    """The 12 leg motors (Unitree's order) given targets and gains (scalars or 12 each), then the CRC."""
    q = np.asarray(q, dtype=float)
    kp, kd = np.broadcast_to(np.asarray(kp, dtype=float), (12,)), np.broadcast_to(np.asarray(kd, dtype=float), (12,))
    dq = np.zeros(12) if dq is None else np.asarray(dq, dtype=float)
    tau = np.zeros(12) if tau is None else np.asarray(tau, dtype=float)
    for i in range(12):
        m = msg.motor_cmd[i]
        m.mode, m.q, m.dq, m.tau, m.kp, m.kd = MODE_SERVO, float(q[i]), float(dq[i]), float(tau[i]), float(kp[i]), float(kd[i])
    msg.crc = lowcmd_crc(msg)


# --- the remote -----------------------------------------------------------------------------------------------------

#: `joystick.h`'s key bits, the 16-bit field at bytes 2-3 of `wireless_remote`.
KEYS = ("R1", "L1", "start", "select", "R2", "L2", "F1", "F2", "A", "B", "X", "Y", "up", "right", "down", "left")


def remote_keys(wireless_remote) -> set[str]:
    raw = bytes(int(b) & 0xFF for b in wireless_remote)
    if len(raw) < 4:
        return set()
    bits = raw[2] | (raw[3] << 8)
    return {name for k, name in enumerate(KEYS) if bits & (1 << k)}


# --- the motion switcher and the sport API ---------------------------------------------------------------------------

MOTION_SWITCHER_REQUEST, MOTION_SWITCHER_RESPONSE = "/api/motion_switcher/request", "/api/motion_switcher/response"
CHECK_MODE, SELECT_MODE, RELEASE_MODE = 1001, 1002, 1003
SPORT_REQUEST = "/api/sport/request"
#: The Go2 sport API ids used here (unitree_sdk2's `sport_api.hpp`).
SPORT_STAND_DOWN, SPORT_STAND_UP, SPORT_STOP_MOVE = 1005, 1004, 1003


def fill_request(req, api_id: int, identity: int, parameter: dict | None = None) -> None:
    req.header.identity.id = int(identity)
    req.header.identity.api_id = int(api_id)
    req.parameter = json.dumps(parameter) if parameter is not None else ""


def mode_name(response_data: str) -> str:
    """The sport mode `CheckMode` reports ("" once released)."""
    try:
        return str(json.loads(response_data or "{}").get("name", ""))
    except ValueError:
        return ""


# --- gains and ramps ------------------------------------------------------------------------------------------------

#: unitree_rl_lab's Go2 FixStand gains (hip, thigh, calf per leg) and Passive damping.
STAND_KP = np.array([60.0, 80.0, 80.0] * 4)
STAND_KD = np.array([5.0, 4.0, 4.0] * 4)
DAMP_KD = 3.0
#: unitree_rl_lab's lying pose (hip, thigh, calf), which its FixStand starts from.
LIE_Q = np.array([0.0, 1.36, -2.65] * 4)


def ramp(q_from, q_to, t: float) -> np.ndarray:
    """A smoothstep blend of joint targets, t in [0, 1]."""
    s = min(max(float(t), 0.0), 1.0)
    s = s * s * (3.0 - 2.0 * s)
    return np.asarray(q_from, dtype=float) + (np.asarray(q_to, dtype=float) - np.asarray(q_from, dtype=float)) * s


def blend(a, b, t: float) -> np.ndarray:
    s = min(max(float(t), 0.0), 1.0)
    return np.asarray(a, dtype=float) * (1.0 - s) + np.asarray(b, dtype=float) * s
