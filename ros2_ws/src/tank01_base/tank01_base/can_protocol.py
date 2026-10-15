"""CAN protocol between the Jetson and the Teensy base controller.

Must stay in sync with firmware/base_controller/include/protocol.h and
docs/A_can_protokoll.md.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

# --- Jetson -> Teensy -------------------------------------------------------
CMD_DRIVE = 0x100
CMD_BLADE = 0x101
HEARTBEAT = 0x102
TIME_SYNC = 0x103
CMD_FAULT_RESET = 0x104

# --- Teensy -> Jetson -------------------------------------------------------
STATUS = 0x200
ENCODERS = 0x201
MOTOR_LEFT = 0x202
MOTOR_RIGHT = 0x203
MOTOR_BLADE = 0x204
BLADE_STATUS = 0x205
ULTRASONIC = 0x206
BMS = 0x207

FORMATS = {
    CMD_DRIVE: '<hhBBH',
    CMD_BLADE: '<HHB3x',
    HEARTBEAT: '<IB3x',
    TIME_SYNC: '<Q',
    CMD_FAULT_RESET: '<B7x',
    STATUS: '<BBBBHBb',
    ENCODERS: '<ii',
    MOTOR_LEFT: '<hhhh',
    MOTOR_RIGHT: '<hhhh',
    MOTOR_BLADE: '<hhhh',
    BLADE_STATUS: '<HHB3x',
    ULTRASONIC: '<6B2x',
    BMS: '<HhBbH',
}

FAULT_RESET_MAGIC = 0xA5


class DriveMode(IntEnum):
    STANDBY = 0
    MANUAL = 1
    AUTO = 2


class BaseState(IntEnum):
    BOOT = 0
    STANDBY = 1
    ACTIVE = 2
    RAMP_DOWN = 3
    FAULT = 4
    ESTOP = 5


class SafetyFlag(IntEnum):
    CHAIN_OPEN = 1 << 0
    TILT_SWITCH = 1 << 1
    MOWER_LIFTED = 1 << 2
    WATCHDOG = 1 << 3
    VESC_TIMEOUT = 1 << 4
    UNDERVOLTAGE = 1 << 5
    OVERTEMP = 1 << 6


class DriveFlag(IntEnum):
    RELEASE_BRAKE = 1 << 0
    BEACON = 1 << 1
    HORN = 1 << 2


def _clamp_int(value: float, lo: int, hi: int) -> int:
    return int(max(lo, min(hi, round(value))))


def encode_drive(left_m_s: float, right_m_s: float, mode: DriveMode,
                 flags: int, seq: int) -> bytes:
    return struct.pack(FORMATS[CMD_DRIVE],
                       _clamp_int(left_m_s * 1000.0, -32768, 32767),
                       _clamp_int(right_m_s * 1000.0, -32768, 32767),
                       int(mode), flags & 0xFF, seq & 0xFFFF)


def encode_blade(rpm: float, height_m: float, enable: bool) -> bytes:
    return struct.pack(FORMATS[CMD_BLADE],
                       _clamp_int(rpm, 0, 65535),
                       _clamp_int(height_m * 1000.0, 0, 65535),
                       1 if enable else 0)


def encode_heartbeat(counter: int, safety_state: int) -> bytes:
    return struct.pack(FORMATS[HEARTBEAT], counter & 0xFFFFFFFF, safety_state & 0xFF)


def encode_time_sync(unix_us: int) -> bytes:
    return struct.pack(FORMATS[TIME_SYNC], unix_us & 0xFFFFFFFFFFFFFFFF)


def encode_fault_reset() -> bytes:
    return struct.pack(FORMATS[CMD_FAULT_RESET], FAULT_RESET_MAGIC)


@dataclass
class Status:
    state: BaseState
    safety_flags: int
    vesc_ok_mask: int
    fault_code: int
    battery_v: float
    soc: int
    temp_c: int

    @property
    def estop(self) -> bool:
        return bool(self.safety_flags & SafetyFlag.CHAIN_OPEN) or self.state == BaseState.ESTOP


@dataclass
class MotorStatus:
    rpm: int
    current_a: float
    temp_c: float
    duty: float


@dataclass
class BmsStatus:
    voltage_v: float
    current_a: float
    soc: int
    temp_c: int
    cell_min_v: float


def decode(can_id: int, data: bytes):
    """Decode a Teensy->Jetson frame. Returns a dataclass/tuple or None if unknown."""
    fmt = FORMATS.get(can_id)
    if fmt is None or can_id < STATUS:
        return None
    values = struct.unpack(fmt, bytes(data[:struct.calcsize(fmt)]))
    if can_id == STATUS:
        st, flags, vesc, fault, bat_cv, soc, temp = values
        return Status(BaseState(st), flags, vesc, fault, bat_cv / 100.0, soc, temp)
    if can_id == ENCODERS:
        return values  # (left_ticks, right_ticks)
    if can_id in (MOTOR_LEFT, MOTOR_RIGHT, MOTOR_BLADE):
        rpm, cur, temp, duty = values
        return MotorStatus(rpm, cur / 10.0, temp / 10.0, duty / 1000.0)
    if can_id == BLADE_STATUS:
        rpm, height_mm, flags = values
        return {'rpm': rpm, 'height_m': height_mm / 1000.0, 'running': bool(flags & 1),
                'brake': bool(flags & 2), 'lifted': bool(flags & 4)}
    if can_id == ULTRASONIC:
        return [None if v in (0, 255) else v * 0.02 for v in values]
    if can_id == BMS:
        v_cv, cur, soc, temp, cell_mv = values
        return BmsStatus(v_cv / 100.0, cur / 10.0, soc, temp, cell_mv / 1000.0)
    return values
