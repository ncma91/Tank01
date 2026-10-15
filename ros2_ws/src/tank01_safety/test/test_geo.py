import math

import pytest

from tank01_safety.geo import Datum, datum_from_geojson, ecef_to_geodetic, geodetic_to_ecef


def test_ecef_roundtrip():
    lat, lon, alt = ecef_to_geodetic(*geodetic_to_ecef(47.26, 11.39, 600.0))
    assert lat == pytest.approx(47.26, abs=1e-9)
    assert lon == pytest.approx(11.39, abs=1e-9)
    assert alt == pytest.approx(600.0, abs=1e-3)


def test_enu_of_datum_is_zero():
    d = Datum(47.26, 11.39, 600.0)
    assert d.to_enu(47.26, 11.39, 600.0) == pytest.approx((0, 0, 0), abs=1e-6)


def test_enu_north_and_east_distances():
    d = Datum(47.26, 11.39, 0.0)
    # 1/1000 deg latitude ≈ 111.2 m north at this latitude
    e, n, _ = d.to_enu(47.261, 11.39, 0.0)
    assert e == pytest.approx(0.0, abs=1e-3)
    assert n == pytest.approx(111.2, abs=0.2)
    # 1/1000 deg longitude ≈ 111.32 * cos(lat) m east
    e, n, _ = d.to_enu(47.26, 11.391, 0.0)
    assert e == pytest.approx(111.32 * math.cos(math.radians(47.26)), abs=0.2)


def test_enu_roundtrip_far_away():
    d = Datum(47.26, 11.39, 600.0)
    lat, lon, alt = d.to_geodetic(*d.to_enu(47.29, 11.45, 650.0))
    assert (lat, lon, alt) == pytest.approx((47.29, 11.45, 650.0), abs=1e-7)


def test_datum_from_geojson_falls_back_to_first_coordinate():
    data = {'type': 'FeatureCollection', 'features': [
        {'type': 'Feature', 'properties': {},
         'geometry': {'type': 'Polygon', 'coordinates': [[[11.0, 47.0], [11.1, 47.0],
                                                          [11.1, 47.1], [11.0, 47.0]]]}}]}
    assert datum_from_geojson(data) == Datum(47.0, 11.0)
