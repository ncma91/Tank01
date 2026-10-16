"""Parsers for the Unicore UM982 output used by Tank01 (pure Python, no ROS).

* ``$xxGGA``        position, fix quality, satellites            (NMEA 0183)
* ``$xxGST``        position error statistics (1-sigma in metres)  (NMEA 0183)
* ``#UNIHEADINGA``  dual-antenna heading, Unicore ASCII log (NovAtel-style layout)

Configure the receiver once, e.g. with these commands (saved with ``SAVECONFIG``):
    GPGGA 0.1        GPGST 0.1        UNIHEADINGA 0.1
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# GGA fix quality → (name, 1-sigma horizontal std [m]) used when no GST is available
FIX_QUALITY = {
    0: ('invalid', math.inf),
    1: ('single', 2.5),
    2: ('dgnss', 0.8),
    4: ('rtk_fixed', 0.02),
    5: ('rtk_float', 0.3),
    6: ('dead_reckoning', 5.0),
}


def nmea_checksum_ok(sentence: str) -> bool:
    sentence = sentence.strip()
    if not sentence.startswith('$') or '*' not in sentence:
        return False
    body, _, cs = sentence[1:].partition('*')
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(cs[:2], 16)
    except ValueError:
        return False


def _crc32_table() -> list[int]:
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0xEDB88320 if c & 1 else c >> 1
        table.append(c)
    return table


_TABLE = _crc32_table()


def unicore_crc32(data: str) -> int:
    """CRC-32 as used by NovAtel/Unicore ASCII logs (no init/final xor)."""
    crc = 0
    for b in data.encode('ascii'):
        crc = _TABLE[(crc ^ b) & 0xFF] ^ (crc >> 8)
    return crc


def unicore_crc_ok(sentence: str) -> bool:
    sentence = sentence.strip()
    if not sentence.startswith('#') or '*' not in sentence:
        return False
    body, _, cs = sentence[1:].rpartition('*')
    try:
        return unicore_crc32(body) == int(cs[:8], 16)
    except ValueError:
        return False


def _latlon(value: str, hemi: str, deg_digits: int) -> float:
    deg = float(value[:deg_digits])
    minutes = float(value[deg_digits:])
    out = deg + minutes / 60.0
    return -out if hemi in ('S', 'W') else out


@dataclass
class Gga:
    lat: float
    lon: float
    alt: float           # ellipsoidal height = MSL altitude + geoid separation
    quality: int
    satellites: int
    hdop: float
    age: float | None

    @property
    def quality_name(self) -> str:
        return FIX_QUALITY.get(self.quality, ('unknown', math.inf))[0]

    @property
    def nominal_std(self) -> float:
        return FIX_QUALITY.get(self.quality, ('unknown', math.inf))[1]


@dataclass
class Gst:
    lat_std: float
    lon_std: float
    alt_std: float


@dataclass
class Heading:
    solution_ok: bool
    pos_type: str        # e.g. NARROW_INT (RTK fixed baseline), NARROW_FLOAT, NONE
    baseline_m: float
    heading_deg: float   # clockwise from true north, ANT1 → ANT2
    pitch_deg: float
    heading_std_deg: float
    pitch_std_deg: float
    satellites: int      # satellites used in the heading solution

    @property
    def fixed(self) -> bool:
        return self.solution_ok and self.pos_type in ('NARROW_INT', 'L1_INT', 'WIDE_INT')


def parse_gga(sentence: str) -> Gga | None:
    f = sentence.strip().split('*')[0].split(',')
    if len(f) < 15 or not f[0].endswith('GGA') or not f[2] or not f[4]:
        return None
    msl = float(f[9]) if f[9] else 0.0
    sep = float(f[11]) if f[11] else 0.0
    return Gga(lat=_latlon(f[2], f[3], 2), lon=_latlon(f[4], f[5], 3), alt=msl + sep,
               quality=int(f[6] or 0), satellites=int(f[7] or 0),
               hdop=float(f[8]) if f[8] else math.nan,
               age=float(f[13]) if f[13] else None)


def parse_gst(sentence: str) -> Gst | None:
    f = sentence.strip().split('*')[0].split(',')
    if len(f) < 9 or not f[0].endswith('GST') or not f[6] or not f[7]:
        return None
    return Gst(lat_std=float(f[6]), lon_std=float(f[7]),
               alt_std=float(f[8]) if f[8] else math.nan)


def parse_uniheading(sentence: str) -> Heading | None:
    s = sentence.strip()
    if not s.startswith('#UNIHEADINGA') or ';' not in s:
        return None
    f = s.split(';', 1)[1].split('*')[0].split(',')
    if len(f) < 11:
        return None
    return Heading(solution_ok=f[0] == 'SOL_COMPUTED', pos_type=f[1],
                   baseline_m=float(f[2]), heading_deg=float(f[3]), pitch_deg=float(f[4]),
                   heading_std_deg=float(f[6]), pitch_std_deg=float(f[7]),
                   satellites=int(f[10]) if f[10] else 0)   # SVs used in solution


def heading_to_enu_yaw(heading_deg: float, mount_offset_deg: float = 0.0) -> float:
    """Convert a compass heading (clockwise from north) to an ENU yaw of the vehicle.

    ``mount_offset_deg`` is the direction of the ANT1→ANT2 baseline relative to the
    vehicle x-axis, measured counter-clockwise (0 = ANT2 straight ahead of ANT1,
    90 = ANT2 to the left of ANT1).
    """
    yaw = math.radians(90.0 - heading_deg - mount_offset_deg)
    return math.atan2(math.sin(yaw), math.cos(yaw))
