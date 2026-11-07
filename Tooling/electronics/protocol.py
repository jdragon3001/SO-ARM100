"""Reference tool bus codec/endpoint, not flashed hardware firmware.

Transport proposal: dedicated half-duplex RS-485 at 115200 8N1. Native arm TTL
servos remain on their existing bus. A module MCU translates tool commands to
its actuator driver. Electrical PCB and MCU port remain bench work.
"""

import json, math, struct, zlib

MAGIC = b"ATC2"
MAX_PAYLOAD = 256


def encode(sequence, message):
    if not isinstance(sequence, int) or not 0 <= sequence < 2**32:
        raise ValueError("sequence")
    payload = json.dumps(
        message, separators=(",", ":"), sort_keys=True, allow_nan=False
    ).encode()
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("oversize")
    body = struct.pack("<IH", sequence, len(payload)) + payload
    return MAGIC + body + struct.pack("<I", zlib.crc32(body))


def decode(frame):
    if len(frame) < 14 or frame[:4] != MAGIC:
        raise ValueError("header")
    sequence, length = struct.unpack("<IH", frame[4:10])
    if length > MAX_PAYLOAD or len(frame) != 14 + length:
        raise ValueError("length")
    if zlib.crc32(frame[4:-4]) != struct.unpack("<I", frame[-4:])[0]:
        raise ValueError("CRC")
    data = json.loads(frame[10:-4])
    if not isinstance(data, dict):
        raise ValueError("message must be an object")
    return sequence, data


class Endpoint:
    """Controller reference behavior: boot disabled, explicit arm, bounded
    command, monotonic sequence, 250ms watchdog, no auto-resume after fault.
    Hardware driver must implement equivalent behavior at a faster local loop.
    """

    def __init__(self, tool):
        if tool not in ("gripper", "soft_gripper", "spatula", "stirrer"):
            raise ValueError("tool")
        self.tool = tool
        self.enabled = False
        self.output = 0.0
        self.last_sequence = -1
        self.last_valid = None
        self.fault = None

    def stop(self, fault=None):
        self.enabled = False
        self.output = 0.0
        self.fault = fault

    def tick(self, now):
        if not math.isfinite(now):
            self.stop("clock")
            return
        if self.enabled and (
            self.last_valid is None or not 0 <= now - self.last_valid <= 0.25
        ):
            self.stop("watchdog")

    def receive(self, frame, now):
        self.tick(now)
        try:
            sequence, message = decode(frame)
            if not math.isfinite(now) or sequence <= self.last_sequence:
                raise ValueError("stale sequence/clock")
            op = message.get("op")
            if message.get("tool") != self.tool:
                raise ValueError("wrong tool")
            if op == "stop":
                self.stop()
            elif op == "arm":
                if (
                    message.get("locked") is not True
                    or message.get("power_good") is not True
                ):
                    raise ValueError("interlock")
                self.fault = None
                self.enabled = True
                self.output = 0.0
            elif op == "set":
                value = message.get("value")
                if type(value) not in (int, float) or not math.isfinite(value):
                    raise ValueError("value")
                limit = (
                    60.0
                    if self.tool == "stirrer"
                    else 0.0
                    if self.tool == "spatula"
                    else 1.0
                )
                if not self.enabled or self.fault or not 0 <= value <= limit:
                    raise ValueError("disabled/range")
                self.output = float(value)
            elif op == "heartbeat":
                if not self.enabled:
                    raise ValueError("disabled")
            elif op != "status":
                raise ValueError("operation")
            self.last_sequence = sequence
            if op in ("arm", "set", "heartbeat"):
                self.last_valid = now
            return {
                "ok": True,
                "tool": self.tool,
                "enabled": self.enabled,
                "output": self.output,
                "fault": self.fault,
            }
        except (
            ValueError,
            TypeError,
            UnicodeError,
            json.JSONDecodeError,
            struct.error,
        ):
            self.stop("invalid command")
            return {
                "ok": False,
                "tool": self.tool,
                "enabled": False,
                "output": 0.0,
                "fault": self.fault,
            }
