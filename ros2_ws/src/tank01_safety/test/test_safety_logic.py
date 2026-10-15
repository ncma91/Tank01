import math

import pytest

from tank01_safety.geofence import Geofence
from tank01_safety.safety_logic import Inputs, Level, Limits, apply_to_twist, evaluate

NOW = 100.0
FENCE = Geofence([(-50, -50), (50, -50), (50, 50), (-50, 50)])


def healthy(**kw):
    base = dict(now=NOW, last_seen={k: NOW for k in ('imu', 'odom', 'base', 'localization')},
                x=0.0, y=0.0, yaw=0.0, pos_std=0.02)
    base.update(kw)
    return Inputs(**base)


def test_all_ok():
    d = evaluate(healthy(), Limits(), FENCE)
    assert d.level == Level.OK
    assert d.max_speed == Limits().max_speed


def test_hardware_estop_dominates():
    d = evaluate(healthy(estop_hw=True, mowing_requested=True), Limits(), FENCE)
    assert d.level == Level.ESTOP
    assert not d.blade_allowed
    assert apply_to_twist(1.0, 0.5, d) == (0.0, 0.0)


def test_stale_sensor_stops():
    inp = healthy()
    inp.last_seen['imu'] = NOW - 2.0
    d = evaluate(inp, Limits(), FENCE)
    assert d.level == Level.STOP
    assert any('imu' in r for r in d.reasons)


def test_missing_sensor_stops():
    inp = healthy()
    del inp.last_seen['localization']
    assert evaluate(inp, Limits(), FENCE).level == Level.STOP


def test_tilt_limits():
    assert evaluate(healthy(roll=math.radians(30)), Limits(), FENCE).level == Level.STOP
    d = evaluate(healthy(pitch=math.radians(24)), Limits(), FENCE)
    assert d.level == Level.SLOW
    assert d.max_speed == Limits().slow_speed


@pytest.mark.parametrize('dist,level,blade', [
    (math.inf, Level.OK, True),
    (8.0, Level.SLOW, True),
    (4.0, Level.SLOW, False),
    (2.0, Level.STOP, False),
])
def test_person_zones(dist, level, blade):
    d = evaluate(healthy(nearest_person=dist, mowing_requested=True), Limits(), FENCE)
    assert d.level == level
    assert d.blade_allowed == blade


def test_mowing_requires_rtk():
    d = evaluate(healthy(pos_std=0.3, mowing_requested=True), Limits(), FENCE)
    assert d.level == Level.PAUSE
    assert not d.blade_allowed
    # Transit with the same accuracy is fine
    assert evaluate(healthy(pos_std=0.3), Limits(), FENCE).level == Level.OK


def test_bad_localization_stops():
    assert evaluate(healthy(pos_std=5.0), Limits(), FENCE).level == Level.STOP


def test_outside_geofence_stops():
    assert evaluate(healthy(x=60.0), Limits(), FENCE).level == Level.STOP


def test_approaching_geofence_blocks_forward_only():
    d = evaluate(healthy(x=49.0, yaw=0.0), Limits(), FENCE)
    assert d.block_forward
    assert apply_to_twist(1.0, 0.0, d) == (0.0, 0.0)
    v, w = apply_to_twist(-0.3, 0.2, d)
    assert v == pytest.approx(-0.3) and w == pytest.approx(0.2)
    # Facing inwards is fine
    assert not evaluate(healthy(x=49.0, yaw=math.pi), Limits(), FENCE).block_forward


def test_transit_allows_higher_speed():
    d = evaluate(healthy(transit=True), Limits(), FENCE)
    assert d.max_speed == Limits().max_speed_transit


def test_apply_to_twist_preserves_curvature():
    d = evaluate(healthy(nearest_person=8.0), Limits(), FENCE)
    v, w = apply_to_twist(1.0, 0.4, d)
    assert v == pytest.approx(0.5)
    assert w / v == pytest.approx(0.4)


def test_operator_pause():
    d = evaluate(healthy(paused_by_operator=True), Limits(), FENCE)
    assert d.level == Level.PAUSE
    assert apply_to_twist(0.5, 0.0, d) == (0.0, 0.0)
