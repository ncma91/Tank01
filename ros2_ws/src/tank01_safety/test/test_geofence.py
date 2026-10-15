import pytest

from tank01_safety.geofence import (Geofence, distance_to_boundary, point_in_polygon,
                                    signed_distance)

SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
L_SHAPE = [(0, 0), (10, 0), (10, 4), (4, 4), (4, 10), (0, 10)]


def test_point_in_polygon():
    assert point_in_polygon((5, 5), SQUARE)
    assert not point_in_polygon((11, 5), SQUARE)
    assert point_in_polygon((2, 8), L_SHAPE)
    assert not point_in_polygon((8, 8), L_SHAPE)


def test_distances():
    assert distance_to_boundary((5, 5), SQUARE) == pytest.approx(5.0)
    assert signed_distance((1, 5), SQUARE) == pytest.approx(1.0)
    assert signed_distance((-2, 5), SQUARE) == pytest.approx(-2.0)


def test_keepout_reduces_clearance():
    fence = Geofence(SQUARE, keepouts=[[(4, 4), (6, 4), (6, 6), (4, 6)]])
    assert fence.clearance((5, 5)) < 0          # inside keep-out
    assert fence.clearance((5, 2.5)) == pytest.approx(1.5)
    assert fence.is_allowed((1, 1), margin=0.5)
    assert not fence.is_allowed((0.2, 5), margin=0.5)


def test_rejects_degenerate_polygon():
    with pytest.raises(ValueError):
        Geofence([(0, 0), (1, 1)])
