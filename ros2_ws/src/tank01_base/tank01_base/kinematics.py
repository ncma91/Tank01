"""Skid-steer (tracked) kinematics and odometry for Tank01.

Pure Python, no ROS dependency, so it can be unit tested anywhere.

Conventions (REP-103): x forward, y left, yaw counter-clockwise positive.
Track speeds are linear speeds of the track surface in m/s (+ = forward).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class TrackParams:
    track_width: float = 0.95          # geometric centre-to-centre distance [m]
    track_width_factor: float = 1.5    # chi: effective width = chi * track_width (slip)
    sprocket_radius: float = 0.105     # drive sprocket pitch radius [m]
    ticks_per_rev: int = 4096          # encoder ticks per sprocket revolution (1024 x4)
    max_track_speed: float = 2.2       # [m/s]
    max_accel: float = 1.0             # [m/s^2] per track

    @property
    def effective_width(self) -> float:
        return self.track_width * self.track_width_factor

    @property
    def metres_per_tick(self) -> float:
        return 2.0 * math.pi * self.sprocket_radius / self.ticks_per_rev


def twist_to_tracks(v: float, omega: float, p: TrackParams) -> tuple[float, float]:
    """Convert body twist (v [m/s], omega [rad/s]) to (left, right) track speeds.

    If one track would exceed max_track_speed, both are scaled down together so the
    commanded curvature is preserved (important for path tracking).
    """
    half = p.effective_width / 2.0
    left = v - omega * half
    right = v + omega * half
    peak = max(abs(left), abs(right))
    if peak > p.max_track_speed > 0.0:
        scale = p.max_track_speed / peak
        left *= scale
        right *= scale
    return left, right


def tracks_to_twist(left: float, right: float, p: TrackParams) -> tuple[float, float]:
    """Inverse of :func:`twist_to_tracks` (without saturation)."""
    v = 0.5 * (left + right)
    omega = (right - left) / p.effective_width
    return v, omega


def rate_limit(target: float, current: float, max_rate: float, dt: float) -> float:
    """Limit the change from current towards target to max_rate*dt."""
    if dt <= 0.0 or max_rate <= 0.0:
        return target
    step = max_rate * dt
    return current + max(-step, min(step, target - current))


def tick_delta(new: int, old: int, bits: int = 32) -> int:
    """Signed difference between two wrapping encoder counters."""
    span = 1 << bits
    d = (new - old) % span
    if d >= span // 2:
        d -= span
    return d


@dataclass
class Odometry:
    """Dead-reckoning pose integrated from track encoder ticks."""

    params: TrackParams
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    omega: float = 0.0
    _last_ticks: tuple[int, int] | None = field(default=None, repr=False)

    def reset(self) -> None:
        self.x = self.y = self.yaw = self.v = self.omega = 0.0
        self._last_ticks = None

    def update(self, left_ticks: int, right_ticks: int, dt: float) -> None:
        if self._last_ticks is None or dt <= 0.0:
            self._last_ticks = (left_ticks, right_ticks)
            return
        dl = tick_delta(left_ticks, self._last_ticks[0]) * self.params.metres_per_tick
        dr = tick_delta(right_ticks, self._last_ticks[1]) * self.params.metres_per_tick
        self._last_ticks = (left_ticks, right_ticks)

        ds = 0.5 * (dl + dr)
        dyaw = (dr - dl) / self.params.effective_width
        # Midpoint integration (exact for constant-curvature arcs to 2nd order).
        mid = self.yaw + 0.5 * dyaw
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.yaw = normalize_angle(self.yaw + dyaw)
        self.v = ds / dt
        self.omega = dyaw / dt


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class SlipEstimator:
    """Online estimate of the effective track width factor chi.

    Compares the yaw rate predicted by the *geometric* track width with the yaw rate
    measured by the IMU. Only updates while turning clearly, to avoid dividing by noise.
    """

    track_width: float
    chi: float = 1.5
    gain: float = 0.02
    min_yaw_rate: float = 0.15
    chi_min: float = 1.0
    chi_max: float = 2.5

    def update(self, left: float, right: float, imu_yaw_rate: float) -> float:
        if abs(imu_yaw_rate) < self.min_yaw_rate:
            return self.chi
        omega_geo = (right - left) / self.track_width
        if omega_geo * imu_yaw_rate <= 0.0:   # opposite sign → skidding or noise
            return self.chi
        measured = omega_geo / imu_yaw_rate
        self.chi += self.gain * (measured - self.chi)
        self.chi = max(self.chi_min, min(self.chi_max, self.chi))
        return self.chi
