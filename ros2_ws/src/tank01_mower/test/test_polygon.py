import pytest

from tank01_mower.polygon import (circle, ensure_ccw, longest_edge_angle, offset,
                                  scanline_intervals, signed_area)

SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]


def test_area_and_orientation():
    assert signed_area(SQUARE) == pytest.approx(100.0)
    assert signed_area(SQUARE[::-1]) == pytest.approx(-100.0)
    assert signed_area(ensure_ccw(SQUARE[::-1])) > 0


def test_inset_square():
    inner = offset(SQUARE, 1.0)
    assert signed_area(inner) == pytest.approx(64.0)
    assert min(p[0] for p in inner) == pytest.approx(1.0)


def test_outset_square():
    assert signed_area(offset(SQUARE, -1.0)) == pytest.approx(144.0)


def test_inset_collapses():
    assert offset(SQUARE, 6.0) == []


def test_scanline_with_hole():
    hole = [(4.0, 4.0), (6.0, 4.0), (6.0, 6.0), (4.0, 6.0)]
    assert scanline_intervals(5.0, [SQUARE, hole]) == [(0.0, 4.0), (6.0, 10.0)]
    assert scanline_intervals(2.0, [SQUARE, hole]) == [(0.0, 10.0)]
    assert scanline_intervals(12.0, [SQUARE]) == []


def test_longest_edge():
    rect = [(0, 0), (20, 0), (20, 5), (0, 5)]
    assert longest_edge_angle(rect) == pytest.approx(0.0)


def test_circle_area():
    assert signed_area(circle((0, 0), 1.0, 64)) == pytest.approx(3.14, abs=0.01)
