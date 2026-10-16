import math

import pytest

from tank01_gnss import um982_protocol as p


def nmea(body: str) -> str:
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f'${body}*{cs:02X}'


def unicore(body: str) -> str:
    return f'#{body}*{p.unicore_crc32(body):08x}'


GGA = nmea('GNGGA,101530.00,4715.60600,N,01123.52600,E,4,32,0.6,598.123,M,47.000,M,1.0,0000')
GST = nmea('GNGST,101530.00,0.9,0.012,0.009,45.0,0.010,0.014,0.025')
HEADING = unicore('UNIHEADINGA,97,GPS,FINE,2190,365174000,0,0,18,12;'
                  'SOL_COMPUTED,NARROW_INT,0.6123,123.4567,-0.5000,0.0000,0.1500,0.3000,'
                  '"999",34,30,30,0,3,00,1,51')


def test_nmea_checksum():
    assert p.nmea_checksum_ok(GGA)
    assert not p.nmea_checksum_ok(GGA[:-2] + '00')
    assert not p.nmea_checksum_ok('GNGGA,no-dollar')


def test_parse_gga():
    g = p.parse_gga(GGA)
    assert g.lat == pytest.approx(47.0 + 15.606 / 60.0)
    assert g.lon == pytest.approx(11.0 + 23.526 / 60.0)
    assert g.alt == pytest.approx(645.123)
    assert g.quality_name == 'rtk_fixed'
    assert g.satellites == 32
    assert g.age == pytest.approx(1.0)
    assert g.nominal_std == pytest.approx(0.02)


def test_parse_gga_southern_western_and_empty():
    g = p.parse_gga(nmea('GPGGA,000000.00,3330.000,S,07030.000,W,1,8,1.0,500,M,0,M,,'))
    assert g.lat == pytest.approx(-33.5) and g.lon == pytest.approx(-70.5)
    assert g.age is None
    assert p.parse_gga(nmea('GPGGA,000000.00,,,,,0,0,,,M,,M,,')) is None


def test_parse_gst():
    s = p.parse_gst(GST)
    assert (s.lat_std, s.lon_std, s.alt_std) == pytest.approx((0.010, 0.014, 0.025))


def test_unicore_crc_roundtrip():
    assert p.unicore_crc_ok(HEADING)
    assert not p.unicore_crc_ok(HEADING.replace('123.4567', '123.4568'))


def test_parse_uniheading():
    h = p.parse_uniheading(HEADING)
    assert h.fixed
    assert h.baseline_m == pytest.approx(0.6123)
    assert h.heading_deg == pytest.approx(123.4567)
    assert h.heading_std_deg == pytest.approx(0.15)
    assert h.satellites == 30
    float_h = p.parse_uniheading(HEADING.replace('NARROW_INT', 'NARROW_FLOAT'))
    assert not float_h.fixed


@pytest.mark.parametrize('heading,offset,yaw_deg', [
    (0.0, 0.0, 90.0),      # pointing north → ENU yaw 90°
    (90.0, 0.0, 0.0),      # east
    (180.0, 0.0, -90.0),   # south
    (270.0, 0.0, 180.0),   # west
    (0.0, 90.0, 0.0),      # baseline points left; vehicle faces east
])
def test_heading_to_enu_yaw(heading, offset, yaw_deg):
    yaw = p.heading_to_enu_yaw(heading, offset)
    assert math.cos(yaw) == pytest.approx(math.cos(math.radians(yaw_deg)), abs=1e-9)
    assert math.sin(yaw) == pytest.approx(math.sin(math.radians(yaw_deg)), abs=1e-9)
