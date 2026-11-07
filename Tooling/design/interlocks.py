"""Release and power permissions; simulation supplies synthetic sensor inputs."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Sensors:
    dock_supported: bool = False
    seated: bool = False
    locked: bool = False
    unlocked: bool = False
    tool_present: bool = False
    tool_id: str | None = None
    rail_v: float = 0.0
    tool_stopped: bool = True
    power_good: bool = False
    estop: bool = False


def may_unlock(s):
    return (
        not s.estop
        and s.dock_supported
        and s.seated
        and s.tool_stopped
        and math.isfinite(s.rail_v)
        and 0 <= s.rail_v < 0.5
    )


def may_depart(s, expected):
    return (
        not s.estop
        and s.locked
        and not s.unlocked
        and s.tool_present
        and s.tool_id == expected
    )


def may_energize(s, expected):
    return may_depart(s, expected) and s.tool_stopped


def identify_resistor(ohms):
    if not math.isfinite(ohms):
        return None
    choices = {"gripper": 1000, "soft_gripper": 2200, "spatula": 4700, "stirrer": 10000}
    hits = [k for k, v in choices.items() if abs(ohms - v) <= v * 0.1]
    return hits[0] if len(hits) == 1 else None
