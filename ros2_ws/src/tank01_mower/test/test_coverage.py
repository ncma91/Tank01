import math
from pathlib import Path

import pytest

from tank01_mower.coverage import CoverageParams, main, plan_coverage, to_svg
from tank01_mower.field import Field, load_field
from tank01_safety.geo import Datum

EXAMPLE = (Path(__file__).resolve().parents[2] / 'tank01_bringup' / 'missions'
           / 'beispiel_wiese.geojson')


def rect_field(w=40.0, h=20.0, keepouts=()):
    return Field('rect', Datum(47.0, 11.0), [(0, 0), (w, 0), (w, h), (0, h)],
                 keepouts=list(keepouts))


def coverage_ratio(fld, plan, params, step=0.25):
    """Fraction of grid points inside the field within cut_width/2 of the path."""
    poses = plan.all_poses()
    segs = [(poses[i], poses[i + 1]) for i in range(len(poses) - 1)
            if math.dist(poses[i][:2], poses[i + 1][:2]) < params.waypoint_spacing * 1.01]
    xs = [p[0] for p in fld.boundary]
    ys = [p[1] for p in fld.boundary]
    hit = total = 0
    y = min(ys) + step / 2
    while y < max(ys):
        x = min(xs) + step / 2
        while x < max(xs):
            total += 1
            for a, b in segs:
                dx, dy = b[0] - a[0], b[1] - a[1]
                t = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / (dx * dx + dy * dy)))
                if math.hypot(x - a[0] - t * dx, y - a[1] - t * dy) <= params.cut_width / 2:
                    hit += 1
                    break
            x += step
        y += step
    return hit / total


def test_rectangle_swaths_alternate():
    p = CoverageParams(cut_width=1.0, overlap=0.0, headland_passes=1)
    plan = plan_coverage(rect_field(), p, start=(0, 0))
    swaths = [s for s in plan.segments if s.kind == 'swath']
    # Inner area 38 x 18 m, swaths along the long edge → 18 swaths
    assert len(swaths) == 18
    yaws = [round(math.cos(s.poses[0][2])) for s in swaths]
    assert all(a == -b for a, b in zip(yaws, yaws[1:]))
    assert plan.segments[-1].kind == 'headland'


def test_rectangle_full_coverage():
    p = CoverageParams(cut_width=1.0, overlap=0.1, headland_passes=2, waypoint_spacing=1.0)
    fld = rect_field(20.0, 12.0)
    plan = plan_coverage(fld, p)
    assert coverage_ratio(fld, plan, p) > 0.99


def test_keepout_is_avoided():
    tree = [(18, 8), (22, 8), (22, 12), (18, 12)]
    fld = rect_field(keepouts=[(tree, 1.0)])
    p = CoverageParams(cut_width=1.0, headland_passes=1, waypoint_spacing=0.5)
    plan = plan_coverage(fld, p)
    for s in plan.segments:
        for x, y, _ in s.poses:
            assert not (17.0 < x < 23.0 and 7.0 < y < 13.0), (x, y)


def test_given_angle_is_used():
    p = CoverageParams(swath_angle_deg=90.0, headland_passes=0)
    plan = plan_coverage(rect_field(), p)
    assert plan.swath_angle == pytest.approx(math.pi / 2)
    s = plan.segments[0]
    assert abs(math.sin(s.poses[0][2])) == pytest.approx(1.0)


def test_stats_are_plausible():
    p = CoverageParams(cut_width=1.0, overlap=0.0, headland_passes=0)
    plan = plan_coverage(rect_field(40, 20), p)
    assert plan.stats['covered_area_m2'] == pytest.approx(800, rel=0.05)


@pytest.mark.skipif(not EXAMPLE.exists(), reason='example mission missing')
def test_example_field(tmp_path, capsys):
    fld = load_field(EXAMPLE)
    assert fld.name == 'Wiese Nord'
    assert len(fld.keepouts) == 3
    assert fld.depot is not None and fld.routes
    plan = plan_coverage(fld, CoverageParams())
    assert plan.stats['swaths'] > 50
    svg = to_svg(fld, plan)
    assert svg.startswith('<svg')
    main([str(EXAMPLE), '--svg', str(tmp_path / 'plan.svg')])
    assert 'swaths' in capsys.readouterr().out
    assert (tmp_path / 'plan.svg').exists()
