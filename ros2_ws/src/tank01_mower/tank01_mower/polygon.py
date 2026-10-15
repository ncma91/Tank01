"""Small planar polygon toolbox (local metres). Pure Python, no shapely needed."""

from __future__ import annotations

import math
from typing import Sequence

Point = tuple[float, float]
Ring = list[Point]


def signed_area(ring: Sequence[Point]) -> float:
    """Positive for counter-clockwise rings."""
    a = 0.0
    n = len(ring)
    for i in range(n):
        x1, y1 = ring[i]
        x2, y2 = ring[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return 0.5 * a


def ensure_ccw(ring: Sequence[Point]) -> Ring:
    ring = list(ring)
    return ring if signed_area(ring) >= 0.0 else ring[::-1]


def rotate(pts: Sequence[Point], angle: float) -> Ring:
    c, s = math.cos(angle), math.sin(angle)
    return [(c * x - s * y, s * x + c * y) for x, y in pts]


def _line_intersection(p1: Point, d1: Point, p2: Point, d2: Point) -> Point | None:
    cross = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(cross) < 1e-12:
        return None
    t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / cross
    return p1[0] + t * d1[0], p1[1] + t * d1[1]


def remove_collinear(ring: Sequence[Point], eps: float = 1e-9) -> Ring:
    out: Ring = []
    n = len(ring)
    for i in range(n):
        a, b, c = ring[i - 1], ring[i], ring[(i + 1) % n]
        if math.dist(a, b) < 1e-9:
            continue
        cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
        if abs(cross) > eps:
            out.append(b)
    return out


def offset(ring: Sequence[Point], distance: float, miter_limit: float = 4.0) -> Ring:
    """Offset a simple polygon; distance > 0 shrinks (inset), < 0 grows.

    Miter offset of every edge followed by intersection of neighbouring edges. Exact
    for convex polygons and fine for mildly concave field boundaries. Returns [] if the
    polygon collapses. For very complex boundaries use Fields2Cover / shapely instead.
    """
    ring = remove_collinear(ensure_ccw(ring))
    n = len(ring)
    if n < 3:
        return []
    lines = []
    for i in range(n):
        a, b = ring[i], ring[(i + 1) % n]
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        nx, ny = -dy / length, dx / length          # left normal = inward for CCW
        lines.append(((a[0] + nx * distance, a[1] + ny * distance), (dx, dy)))
    out: Ring = []
    for i in range(n):
        p_prev, d_prev = lines[i - 1]
        p_cur, d_cur = lines[i]
        pt = _line_intersection(p_prev, d_prev, p_cur, d_cur)
        if pt is None:
            pt = p_cur
        # Clamp extreme miters at very sharp corners.
        corner = ring[i]
        if math.dist(pt, corner) > abs(distance) * miter_limit:
            k = abs(distance) * miter_limit / math.dist(pt, corner)
            pt = (corner[0] + (pt[0] - corner[0]) * k, corner[1] + (pt[1] - corner[1]) * k)
        out.append(pt)
    area_in, area_out = signed_area(ring), signed_area(out)
    if area_out <= 0.0 or (distance > 0 and area_out >= area_in):
        return []
    if distance > 0 and _edges_flipped(ring, out):
        return []
    return out


def _edges_flipped(original: Ring, shifted: Ring) -> bool:
    n = len(original)
    for i in range(n):
        a, b = original[i], original[(i + 1) % n]
        c, d = shifted[i], shifted[(i + 1) % n]
        if (b[0] - a[0]) * (d[0] - c[0]) + (b[1] - a[1]) * (d[1] - c[1]) < 0:
            return True
    return False


def circle(center: Point, radius: float, segments: int = 12) -> Ring:
    cx, cy = center
    return [(cx + radius * math.cos(2 * math.pi * i / segments),
             cy + radius * math.sin(2 * math.pi * i / segments)) for i in range(segments)]


def scanline_intervals(y: float, rings: Sequence[Sequence[Point]]) -> list[tuple[float, float]]:
    """Intervals of the horizontal line at height y that lie inside the region.

    The region is defined by the even-odd rule over all rings (outer boundary + holes).
    """
    xs: list[float] = []
    for ring in rings:
        n = len(ring)
        for i in range(n):
            (x1, y1), (x2, y2) = ring[i], ring[(i + 1) % n]
            if (y1 > y) != (y2 > y):
                xs.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
    xs.sort()
    return [(xs[i], xs[i + 1]) for i in range(0, len(xs) - 1, 2)]


def longest_edge_angle(ring: Sequence[Point]) -> float:
    best, angle = -1.0, 0.0
    n = len(ring)
    for i in range(n):
        (x1, y1), (x2, y2) = ring[i], ring[(i + 1) % n]
        length = math.hypot(x2 - x1, y2 - y1)
        if length > best:
            best, angle = length, math.atan2(y2 - y1, x2 - x1)
    return angle


def polyline_length(pts: Sequence[Point]) -> float:
    return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
