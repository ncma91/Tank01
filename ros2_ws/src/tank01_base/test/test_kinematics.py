import math

import pytest

from tank01_base.kinematics import (Odometry, SlipEstimator, TrackParams, rate_limit,
                                    tick_delta, tracks_to_twist, twist_to_tracks)


@pytest.fixture
def params():
    return TrackParams(track_width=1.0, track_width_factor=1.5, sprocket_radius=0.1,
                       ticks_per_rev=1000, max_track_speed=2.0)


def test_straight(params):
    assert twist_to_tracks(1.0, 0.0, params) == (1.0, 1.0)


def test_turn_in_place_uses_effective_width(params):
    left, right = twist_to_tracks(0.0, 1.0, params)
    assert left == pytest.approx(-0.75)
    assert right == pytest.approx(0.75)


def test_roundtrip(params):
    left, right = twist_to_tracks(0.4, 0.3, params)
    v, w = tracks_to_twist(left, right, params)
    assert v == pytest.approx(0.4)
    assert w == pytest.approx(0.3)


def test_saturation_preserves_curvature(params):
    left, right = twist_to_tracks(2.0, 2.0, params)
    assert max(abs(left), abs(right)) == pytest.approx(2.0)
    v, w = tracks_to_twist(left, right, params)
    assert w / v == pytest.approx(1.0)


def test_rate_limit():
    assert rate_limit(1.0, 0.0, 1.0, 0.1) == pytest.approx(0.1)
    assert rate_limit(-1.0, 0.0, 1.0, 0.1) == pytest.approx(-0.1)
    assert rate_limit(0.05, 0.0, 1.0, 0.1) == pytest.approx(0.05)


def test_tick_delta_wraps():
    assert tick_delta(5, 2**32 - 5) == 10
    assert tick_delta(2**32 - 5, 5) == -10
    assert tick_delta(100, 40) == 60


def test_odometry_straight_line(params):
    odo = Odometry(params)
    odo.update(0, 0, 0.0)
    ticks = int(1.0 / params.metres_per_tick)
    odo.update(ticks, ticks, 1.0)
    assert odo.x == pytest.approx(1.0, abs=1e-3)
    assert odo.y == pytest.approx(0.0)
    assert odo.v == pytest.approx(1.0, abs=1e-3)


def test_odometry_full_rotation(params):
    odo = Odometry(params)
    odo.update(0, 0, 0.0)
    # Spin a quarter turn in 100 small steps.
    arc = (math.pi / 2) * params.effective_width / 2.0
    steps = 100
    for i in range(1, steps + 1):
        t = round(i * arc / steps / params.metres_per_tick)
        odo.update(-t, t, 0.01)
    assert odo.yaw == pytest.approx(math.pi / 2, abs=0.01)
    assert math.hypot(odo.x, odo.y) < 1e-6


def test_odometry_circle_returns_home(params):
    odo = Odometry(params)
    odo.update(0, 0, 0.0)
    # radius 2 m circle: v=1, w=0.5
    left, right = twist_to_tracks(1.0, 0.5, params)
    steps = 2000
    period = 2 * math.pi / 0.5
    dt = period / steps
    acc_l = acc_r = 0.0
    for _ in range(steps):
        acc_l += left * dt
        acc_r += right * dt
        odo.update(round(acc_l / params.metres_per_tick), round(acc_r / params.metres_per_tick), dt)
    assert math.hypot(odo.x, odo.y) < 0.05


def test_slip_estimator_converges():
    est = SlipEstimator(track_width=1.0, chi=1.0, gain=0.1)
    true_chi = 1.6
    left, right = -0.8, 0.8
    imu = (right - left) / (1.0 * true_chi)
    for _ in range(200):
        est.update(left, right, imu)
    assert est.chi == pytest.approx(true_chi, abs=1e-3)


def test_slip_estimator_ignores_small_rates():
    est = SlipEstimator(track_width=1.0, chi=1.3)
    assert est.update(0.5, 0.51, 0.01) == 1.3
