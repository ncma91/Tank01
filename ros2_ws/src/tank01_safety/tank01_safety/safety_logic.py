"""Safety supervisor decision logic (software safety functions SF3-SF7, SF9).

Pure Python and side-effect free: the ROS node collects inputs into a :class:`Inputs`
snapshot, calls :func:`evaluate` and applies the resulting :class:`Decision`.
See docs/09_sicherheitskonzept.md.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum

from tank01_safety.geofence import Geofence


class Level(IntEnum):
    OK = 0
    SLOW = 1
    PAUSE = 2
    STOP = 3
    ESTOP = 4


@dataclass
class Limits:
    max_speed: float = 1.4             # m/s, absolute cap in any mode
    max_speed_transit: float = 2.2
    max_yaw_rate: float = 1.0          # rad/s
    slow_speed: float = 0.5            # m/s when a person is near / poor localisation
    max_roll_deg: float = 25.0
    max_pitch_deg: float = 30.0
    warn_roll_deg: float = 18.0
    warn_pitch_deg: float = 22.0
    person_slow_m: float = 10.0
    person_blade_off_m: float = 5.0
    person_stop_m: float = 3.0
    max_pos_std_mowing: float = 0.10   # m (RTK fix required for mowing)
    max_pos_std_driving: float = 1.0   # m
    max_pos_std_slow: float = 0.4      # m, above → SLOW
    geofence_margin: float = 0.5       # m
    geofence_lookahead: float = 1.5    # s
    stale_timeout: float = 0.5         # s


@dataclass
class Inputs:
    now: float
    estop_hw: bool = False
    last_seen: dict[str, float] = field(default_factory=dict)
    required: tuple[str, ...] = ('imu', 'odom', 'base', 'localization')
    roll: float = 0.0                  # rad
    pitch: float = 0.0                 # rad
    x: float | None = None             # map frame position
    y: float | None = None
    yaw: float = 0.0
    pos_std: float = math.inf          # 1-sigma horizontal position uncertainty [m]
    nearest_person: float = math.inf   # m
    mowing_requested: bool = False
    transit: bool = False
    paused_by_operator: bool = False


@dataclass
class Decision:
    level: Level
    reasons: list[str]
    max_speed: float
    max_yaw_rate: float
    blade_allowed: bool
    block_forward: bool = False        # geofence: forbid motion that leaves the fence

    def summary(self) -> str:
        return f'{self.level.name}: ' + ('; '.join(self.reasons) if self.reasons else 'all ok')


def evaluate(inp: Inputs, limits: Limits, fence: Geofence | None = None) -> Decision:
    level = Level.OK
    reasons: list[str] = []
    speed = limits.max_speed_transit if (inp.transit and not inp.mowing_requested) \
        else limits.max_speed
    blade = inp.mowing_requested
    block_forward = False

    def raise_to(new: Level, why: str) -> None:
        nonlocal level
        level = max(level, new)
        reasons.append(why)

    # SF1 mirror: hardware e-stop chain open
    if inp.estop_hw:
        raise_to(Level.ESTOP, 'hardware e-stop')

    # Data freshness (a silent sensor must never look like "no danger")
    for name in inp.required:
        seen = inp.last_seen.get(name)
        if seen is None or inp.now - seen > limits.stale_timeout:
            raise_to(Level.STOP, f'stale {name}')

    # SF3 tilt protection
    roll_deg = abs(math.degrees(inp.roll))
    pitch_deg = abs(math.degrees(inp.pitch))
    if roll_deg > limits.max_roll_deg or pitch_deg > limits.max_pitch_deg:
        raise_to(Level.STOP, f'tilt roll={roll_deg:.0f} pitch={pitch_deg:.0f}')
    elif roll_deg > limits.warn_roll_deg or pitch_deg > limits.warn_pitch_deg:
        raise_to(Level.SLOW, 'steep slope')
        speed = min(speed, limits.slow_speed)

    # SF5 person / animal protection zones
    if inp.nearest_person < limits.person_stop_m:
        raise_to(Level.STOP, f'person at {inp.nearest_person:.1f} m')
    elif inp.nearest_person < limits.person_blade_off_m:
        raise_to(Level.SLOW, f'person at {inp.nearest_person:.1f} m')
        blade = False
        speed = min(speed, limits.slow_speed)
    elif inp.nearest_person < limits.person_slow_m:
        raise_to(Level.SLOW, f'person at {inp.nearest_person:.1f} m')
        speed = min(speed, limits.slow_speed)

    # SF7 localisation integrity
    if inp.pos_std > limits.max_pos_std_driving:
        raise_to(Level.STOP, f'localization std {inp.pos_std:.2f} m')
    else:
        if inp.mowing_requested and inp.pos_std > limits.max_pos_std_mowing:
            raise_to(Level.PAUSE, f'no RTK fix for mowing (std {inp.pos_std:.2f} m)')
            blade = False
        if inp.pos_std > limits.max_pos_std_slow:
            raise_to(Level.SLOW, f'degraded localization {inp.pos_std:.2f} m')
            speed = min(speed, limits.slow_speed)

    # SF6 geofence
    if fence is not None:
        if inp.x is None or inp.y is None:
            raise_to(Level.STOP, 'no position for geofence')
        else:
            clearance = fence.clearance((inp.x, inp.y))
            if clearance < 0.0:
                raise_to(Level.STOP, f'outside geofence by {-clearance:.2f} m')
            else:
                look = max(speed, limits.slow_speed) * limits.geofence_lookahead
                ahead = (inp.x + look * math.cos(inp.yaw), inp.y + look * math.sin(inp.yaw))
                if fence.clearance(ahead) < limits.geofence_margin:
                    block_forward = True
                    raise_to(Level.SLOW, 'approaching geofence')
                    speed = min(speed, limits.slow_speed)

    if inp.paused_by_operator:
        raise_to(Level.PAUSE, 'operator pause')

    if level >= Level.PAUSE:
        speed = 0.0
        blade = False

    return Decision(level=level, reasons=reasons, max_speed=speed,
                    max_yaw_rate=0.0 if level >= Level.PAUSE else limits.max_yaw_rate,
                    blade_allowed=blade, block_forward=block_forward)


def apply_to_twist(v: float, w: float, d: Decision) -> tuple[float, float]:
    """Clamp a commanded (v, w) according to the decision.

    Curvature is preserved when the linear speed is scaled, so the vehicle stays on its
    path while slowing down.
    """
    if d.level >= Level.PAUSE:
        return 0.0, 0.0
    if d.block_forward and v > 0.0:
        v = 0.0  # rotating in place / reversing away from the fence is still allowed
    if abs(v) > d.max_speed > 0.0:
        scale = d.max_speed / abs(v)
        v *= scale
        w *= scale
    elif d.max_speed == 0.0:
        v = 0.0
    w = max(-d.max_yaw_rate, min(d.max_yaw_rate, w))
    return v, w
