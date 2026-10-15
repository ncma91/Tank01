import re
import struct
from pathlib import Path

import pytest

from tank01_base import can_protocol as p

FIRMWARE_HEADER = (Path(__file__).resolve().parents[4] / 'firmware' / 'base_controller'
                   / 'include' / 'protocol.h')


def test_all_frames_fit_in_8_bytes():
    for can_id, fmt in p.FORMATS.items():
        assert struct.calcsize(fmt) == 8, hex(can_id)


def test_drive_encoding_and_clamping():
    data = p.encode_drive(1.234, -0.5, p.DriveMode.AUTO, 0x3, 70000)
    left, right, mode, flags, seq = struct.unpack(p.FORMATS[p.CMD_DRIVE], data)
    assert (left, right, mode, flags) == (1234, -500, 2, 3)
    assert seq == 70000 & 0xFFFF
    left, *_ = struct.unpack(p.FORMATS[p.CMD_DRIVE], p.encode_drive(100.0, 0, 0, 0, 0))
    assert left == 32767


def test_blade_encoding():
    rpm, height, flags = struct.unpack(p.FORMATS[p.CMD_BLADE], p.encode_blade(2400, 0.07, True))
    assert (rpm, height, flags) == (2400, 70, 1)


def test_decode_status():
    raw = struct.pack(p.FORMATS[p.STATUS], 5, p.SafetyFlag.CHAIN_OPEN, 0b111, 0, 5230, 77, -3)
    st = p.decode(p.STATUS, raw)
    assert st.state == p.BaseState.ESTOP
    assert st.estop
    assert st.battery_v == pytest.approx(52.3)
    assert st.temp_c == -3


def test_decode_motor_and_bms():
    m = p.decode(p.MOTOR_LEFT, struct.pack('<hhhh', 150, 455, 412, 300))
    assert m.current_a == pytest.approx(45.5)
    assert m.duty == pytest.approx(0.3)
    b = p.decode(p.BMS, struct.pack(p.FORMATS[p.BMS], 5200, -120, 64, 21, 3250))
    assert b.current_a == pytest.approx(-12.0)
    assert b.cell_min_v == pytest.approx(3.25)


def test_decode_ultrasonic_invalid_values():
    d = p.decode(p.ULTRASONIC, struct.pack(p.FORMATS[p.ULTRASONIC], 0, 50, 255, 1, 2, 3))
    assert d[0] is None and d[2] is None
    assert d[1] == pytest.approx(1.0)


def test_commands_are_not_decoded():
    assert p.decode(p.CMD_DRIVE, bytes(8)) is None


@pytest.mark.skipif(not FIRMWARE_HEADER.exists(), reason='firmware header not found')
def test_ids_match_firmware_header():
    text = FIRMWARE_HEADER.read_text()
    found = dict(re.findall(r'constexpr\s+uint32_t\s+(\w+)\s*=\s*(0x[0-9A-Fa-f]+)', text))
    for name in ('CMD_DRIVE', 'CMD_BLADE', 'HEARTBEAT', 'TIME_SYNC', 'CMD_FAULT_RESET',
                 'STATUS', 'ENCODERS', 'MOTOR_LEFT', 'MOTOR_RIGHT', 'MOTOR_BLADE',
                 'BLADE_STATUS', 'ULTRASONIC', 'BMS'):
        assert int(found[f'ID_{name}'], 16) == getattr(p, name), name
