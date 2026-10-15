"""Polygon geofence helpers (planar, local ENU metres). Pure Python."""

from __future__ import annotations

import math
from typing import Sequence

Point = tuple[float, float]


def point_in_polygon(pt: Point, poly: Sequence[Point]) -> bool:
    """Even-odd ray casting. Points exactly on the boundary may go either way."""
    x, y = pt
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y):
            x_cross = xi + (y - yi) * (xj - xi) / (yj - yi)
            if x < x_cross:
                inside = not inside
        j = i
    return inside


def distance_to_segment(pt: Point, a: Point, b: Point) -> float:
    px, py = pt
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    seg2 = dx * dx + dy * dy
    if seg2 == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def distance_to_boundary(pt: Point, poly: Sequence[Point]) -> float:
    n = len(poly)
    return min(distance_to_segment(pt, poly[i], poly[(i + 1) % n]) for i in range(n))


def signed_distance(pt: Point, poly: Sequence[Point]) -> float:
    """Positive inside the polygon, negative outside (metres)."""
    d = distance_to_boundary(pt, poly)
    return d if point_in_polygon(pt, poly) else -d


class Geofence:
    """Outer fence plus optional keep-out polygons."""

    def __init__(self, outer: Sequence[Point], keepouts: Sequence[Sequence[Point]] = ()):
        if len(outer) < 3:
            raise ValueError('geofence polygon needs at least 3 points')
        self.outer = list(outer)
        self.keepouts = [list(k) for k in keepouts if len(k) >= 3]

    def clearance(self, pt: Point) -> float:
        """Distance to the nearest forbidden area; negative if inside a forbidden area."""
        d = signed_distance(pt, self.outer)
        for k in self.keepouts:
            d = min(d, -signed_distance(pt, k))
        return d

    def is_allowed(self, pt: Point, margin: float = 0.0) -> bool:
        return self.clearance(pt) >= margin
