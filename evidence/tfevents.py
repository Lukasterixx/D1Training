"""Read scalar summaries from TensorBoard event files without TensorFlow.

An event file is a sequence of TFRecords: u64 payload length, u32 CRC, payload,
u32 CRC. Each payload is an `Event` protobuf. Scalars arrive either as
`Summary.Value.simple_value` (torch's SummaryWriter, which RSL-RL uses) or as a
one-element float/double `tensor`. Reading is incremental, so polling a file
that training is still writing is cheap, and a half-written trailing record is
left for the next read instead of raising.
"""
from __future__ import annotations

from pathlib import Path
import struct

DT_FLOAT, DT_DOUBLE = 1, 2


def _varint(buf: bytes, pos: int) -> tuple[int, int]:
    result = shift = 0
    while True:
        byte = buf[pos]
        pos += 1
        result |= (byte & 0x7F) << shift
        if byte < 0x80:
            return result, pos
        shift += 7


def _fields(buf: bytes):
    """Yield (field number, wire type, raw value) for one protobuf message."""
    pos = 0
    while pos < len(buf):
        key, pos = _varint(buf, pos)
        number, wire = key >> 3, key & 7
        if wire == 0:
            value, pos = _varint(buf, pos)
        elif wire == 1:
            value, pos = buf[pos:pos + 8], pos + 8
        elif wire == 2:
            length, pos = _varint(buf, pos)
            value, pos = buf[pos:pos + length], pos + length
        elif wire == 5:
            value, pos = buf[pos:pos + 4], pos + 4
        else:
            raise ValueError(f"Unsupported protobuf wire type {wire}.")
        yield number, wire, value


def _tensor_scalar(buf: bytes) -> float | None:
    dtype, floats, doubles, content = None, [], [], b""
    for number, wire, value in _fields(buf):
        if number == 1:
            dtype = value
        elif number == 4:
            content = value
        elif number == 5:
            floats += struct.unpack(f"<{len(value) // 4}f", value)
        elif number == 6:
            doubles += struct.unpack(f"<{len(value) // 8}d", value)
    if dtype == DT_FLOAT:
        values = floats or list(struct.unpack(f"<{len(content) // 4}f", content))
    elif dtype == DT_DOUBLE:
        values = doubles or list(struct.unpack(f"<{len(content) // 8}d", content))
    else:
        return None
    return float(values[0]) if len(values) == 1 else None


def parse_event(payload: bytes) -> list[tuple[str, int, float, float]]:
    """Return (tag, step, wall_time, value) for every scalar in one Event."""
    wall_time, step, summary = 0.0, 0, None
    for number, _, value in _fields(payload):
        if number == 1:
            wall_time = struct.unpack("<d", value)[0]
        elif number == 2:
            step = value - (1 << 64) if value >= 1 << 63 else value
        elif number == 5:
            summary = value
    scalars = []
    for number, _, value_msg in _fields(summary or b""):
        if number != 1:
            continue
        tag, scalar = None, None
        for field, _, value in _fields(value_msg):
            if field == 1:
                tag = value.decode("utf-8", "replace")
            elif field == 2:
                scalar = struct.unpack("<f", value)[0]
            elif field == 8 and scalar is None:
                scalar = _tensor_scalar(value)
        if tag is not None and scalar is not None:
            scalars.append((tag, step, wall_time, scalar))
    return scalars


class EventFileReader:
    """Accumulates scalars from one event file across repeated `read()` calls."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.offset = 0
        self.scalars: dict[str, list[tuple[int, float, float]]] = {}

    def read(self) -> dict[str, list[tuple[int, float, float]]]:
        size = self.path.stat().st_size
        if size < self.offset:  # Replaced or truncated: start again.
            self.offset, self.scalars = 0, {}
        if size == self.offset:
            return self.scalars
        with self.path.open("rb") as handle:
            handle.seek(self.offset)
            data = handle.read()
        pos = 0
        while pos + 12 <= len(data):
            (length,) = struct.unpack_from("<Q", data, pos)
            end = pos + 12 + length + 4
            if end > len(data):
                break
            for tag, step, wall_time, value in parse_event(data[pos + 12:pos + 12 + length]):
                self.scalars.setdefault(tag, []).append((step, wall_time, value))
            pos = end
        self.offset += pos
        return self.scalars


_readers: dict[Path, EventFileReader] = {}


def event_files(run_dir: str | Path) -> list[Path]:
    return sorted(Path(run_dir).glob("events.out.tfevents.*"))


def read_scalars(run_dir: str | Path) -> dict[str, list[tuple[int, float, float]]]:
    """Merge every event file in a run directory, keyed by tag and sorted by step.

    A resumed run writes a second file; where both hold a step, the later file wins.
    """
    merged: dict[str, dict[int, tuple[int, float, float]]] = {}
    for path in event_files(run_dir):
        reader = _readers.setdefault(path.resolve(), EventFileReader(path))
        for tag, points in reader.read().items():
            by_step = merged.setdefault(tag, {})
            for point in points:
                by_step[point[0]] = point
    return {tag: [by_step[s] for s in sorted(by_step)] for tag, by_step in merged.items()}
