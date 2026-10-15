import pytest

from tank01_mower.routing import RouteGraph, path_length, resample


def test_shortest_path_prefers_shortcut():
    long_way = [(0, 0), (0, 100), (100, 100)]
    shortcut = [(0, 0), (60, 50), (100, 100)]
    g = RouteGraph([long_way, shortcut])
    path = g.shortest_path((0, 0), (100, 100))
    assert (60.0, 50.0) in path
    assert path_length(path) < 200


def test_snapping_joins_lines():
    g = RouteGraph([[(0, 0), (50, 0)], [(50.5, 0.5), (50, 80)]], snap=2.0)
    path = g.shortest_path((0, 0), (50, 80))
    assert path[-1] == (50, 80)


def test_unreachable_goal():
    g = RouteGraph([[(0, 0), (10, 0)], [(100, 0), (110, 0)]])
    with pytest.raises(ValueError):
        g.shortest_path((0, 0), (110, 0))


def test_resample_spacing():
    pts = resample([(0, 0), (10, 0), (10, 10)], 3.0)
    assert pts[0] == (0, 0) and pts[-1] == (10, 10)
    for a, b in zip(pts[:-2], pts[1:-1]):
        assert abs(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5) <= 3.0 + 1e-9
    assert len(pts) == 8
