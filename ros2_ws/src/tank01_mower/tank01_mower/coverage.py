"""Coverage (mowing) path planner: headland passes + boustrophedon swaths.

See docs/08_maehplanung_missionen.md. Pure Python; run as a module for an offline
preview:  python3 -m tank01_mower.coverage feld.geojson [--svg out.svg]
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

from tank01_mower.field import Field, load_field
from tank01_mower.polygon import (Point, Ring, longest_edge_angle, offset, polyline_length,
                                  rotate, scanline_intervals)

Pose = tuple[float, float, float]   # x, y, yaw


@dataclass
class CoverageParams:
    cut_width: float = 1.0
    overlap: float = 0.10
    headland_passes: int = 2
    swath_angle_deg: float | None = None      # None = along the longest boundary edge
    min_swath_length: float = 2.0
    waypoint_spacing: float = 2.0
    keepout_clearance: float = 0.3            # extra margin around keep-outs [m]
    mow_speed: float = 1.0                    # m/s, for time estimate
    transit_speed: float = 1.5

    @property
    def effective_width(self) -> float:
        return self.cut_width * (1.0 - self.overlap)


@dataclass
class Segment:
    kind: str                 # 'swath' | 'headland'
    poses: list[Pose]
    blade: bool = True

    @property
    def length(self) -> float:
        return polyline_length([(p[0], p[1]) for p in self.poses])


@dataclass
class CoveragePlan:
    segments: list[Segment]
    swath_angle: float
    inner_area: Ring
    stats: dict[str, float] = field(default_factory=dict)

    def all_poses(self) -> list[Pose]:
        return [p for s in self.segments for p in s.poses]


def densify(a: Point, b: Point, spacing: float) -> list[Pose]:
    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    n = max(1, math.ceil(math.dist(a, b) / spacing))
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n, yaw)
            for i in range(n + 1)]


def _ring_path(ring: Ring, start_near: Point | None, spacing: float) -> list[Pose]:
    """Closed loop along a ring (CCW), starting at the vertex closest to start_near."""
    k = 0
    if start_near is not None:
        k = min(range(len(ring)), key=lambda i: math.dist(ring[i], start_near))
    ordered = ring[k:] + ring[:k] + [ring[k]]
    poses: list[Pose] = []
    for a, b in zip(ordered, ordered[1:]):
        seg = densify(a, b, spacing)
        poses.extend(seg if not poses else seg[1:])
    return poses


def _obstacles(fld: Field, p: CoverageParams) -> list[Ring]:
    obs = []
    for ring, buf in fld.keepouts:
        grown = offset(ring, -(buf + p.cut_width / 2.0 + p.keepout_clearance))
        obs.append(grown or ring)
    for hole in fld.holes:
        grown = offset(hole, -(p.cut_width / 2.0 + p.keepout_clearance))
        obs.append(grown or hole)
    return obs


def plan_coverage(fld: Field, p: CoverageParams, start: Point | None = None) -> CoveragePlan:
    w = p.effective_width
    if w <= 0.0:
        raise ValueError('effective cut width must be positive')
    start = start or fld.entry or fld.boundary[0]

    # 1. Headland rings (driven last, they clean up the turning marks of the swaths).
    rings = []
    for i in range(p.headland_passes):
        r = offset(fld.boundary, (i + 0.5) * w)
        if not r:
            break
        rings.append(r)
    inner = offset(fld.boundary, p.headland_passes * w) if p.headland_passes else \
        list(fld.boundary)
    if not inner:
        inner = rings[-1] if rings else list(fld.boundary)

    # 2. Swath direction
    if p.swath_angle_deg is None:
        angle = longest_edge_angle(inner)
    else:
        angle = math.radians(p.swath_angle_deg)

    # 3. Rotate everything so that swaths are horizontal, intersect scan lines.
    obstacles = _obstacles(fld, p)
    rot_rings = [rotate(inner, -angle)] + [rotate(o, -angle) for o in obstacles]
    ys = [pt[1] for pt in rot_rings[0]]
    y_min, y_max = min(ys), max(ys)
    swaths_rot: list[list[tuple[Point, Point]]] = []
    # Centre the swath pattern so the leftover strip is split evenly at both sides.
    count = max(1, math.ceil((y_max - y_min) / w - 1e-9))
    first = y_min + ((y_max - y_min) - (count - 1) * w) / 2.0
    for k in range(count):
        y = first + k * w
        row = [((x0, y), (x1, y)) for x0, x1 in scanline_intervals(y, rot_rings)
               if x1 - x0 >= p.min_swath_length]
        if row:
            swaths_rot.append(row)

    # 4. Order: boustrophedon with greedy nearest-end selection (handles splits by holes).
    remaining = [(rot_a, rot_b) for row in swaths_rot for rot_a, rot_b in row]
    remaining = [tuple(rotate([a, b], angle)) for a, b in remaining]
    segments: list[Segment] = []
    pos = start
    while remaining:
        best_i, best_rev, best_d = 0, False, math.inf
        for i, (a, b) in enumerate(remaining):
            da, db = math.dist(pos, a), math.dist(pos, b)
            if da < best_d:
                best_i, best_rev, best_d = i, False, da
            if db < best_d:
                best_i, best_rev, best_d = i, True, db
        a, b = remaining.pop(best_i)
        if best_rev:
            a, b = b, a
        segments.append(Segment('swath', densify(a, b, p.waypoint_spacing)))
        pos = b

    # 5. Headland passes from the innermost outwards, ending close to the boundary/exit.
    for ring in reversed(rings):
        poses = _ring_path(ring, pos, p.waypoint_spacing)
        segments.append(Segment('headland', poses))
        pos = poses[-1][:2]

    # 6. Statistics
    mow_len = sum(s.length for s in segments)
    jumps = sum(math.dist(segments[i].poses[-1][:2], segments[i + 1].poses[0][:2])
                for i in range(len(segments) - 1))
    plan = CoveragePlan(segments, angle, inner)
    plan.stats = {
        'swaths': float(sum(1 for s in segments if s.kind == 'swath')),
        'headland_passes': float(len(rings)),
        'mow_length_m': mow_len,
        'transition_length_m': jumps,
        'covered_area_m2': mow_len * w,
        'estimated_time_min': (mow_len / p.mow_speed + jumps / p.transit_speed) / 60.0 * 1.15,
    }
    return plan


def to_svg(fld: Field, plan: CoveragePlan, scale: float = 4.0) -> str:
    pts = fld.boundary + [q[:2] for q in plan.all_poses()]
    xs, ys = [q[0] for q in pts], [q[1] for q in pts]
    x0, y0 = min(xs) - 5, min(ys) - 5
    width, height = (max(xs) - x0 + 5) * scale, (max(ys) - y0 + 5) * scale

    def tr(q):
        return f'{(q[0] - x0) * scale:.1f},{height - (q[1] - y0) * scale:.1f}'

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}">',
           '<rect width="100%" height="100%" fill="#f4f7f0"/>',
           f'<polygon points="{" ".join(tr(q) for q in fld.boundary)}" '
           'fill="#cfe8b5" stroke="#3a6b1f" stroke-width="2"/>']
    for ring, _ in fld.keepouts:
        out.append(f'<polygon points="{" ".join(tr(q) for q in ring)}" fill="#b55" />')
    colors = {'swath': '#1f4e9b', 'headland': '#d9822b'}
    prev = None
    for s in plan.segments:
        if prev is not None:
            out.append(f'<polyline points="{tr(prev)} {tr(s.poses[0])}" stroke="#999" '
                       'stroke-dasharray="4 3" fill="none"/>')
        out.append(f'<polyline points="{" ".join(tr(q) for q in s.poses)}" '
                   f'stroke="{colors[s.kind]}" stroke-width="1.5" fill="none"/>')
        prev = s.poses[-1]
    out.append('</svg>')
    return '\n'.join(out)


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description='Offline coverage plan preview for Tank01')
    ap.add_argument('geojson')
    ap.add_argument('--field', default=None, help='name of the field feature')
    ap.add_argument('--width', type=float, default=1.0)
    ap.add_argument('--overlap', type=float, default=0.10)
    ap.add_argument('--headland', type=int, default=2)
    ap.add_argument('--angle', type=float, default=None, help='swath angle in degrees (ENU)')
    ap.add_argument('--svg', default=None, help='write an SVG preview to this file')
    args = ap.parse_args(argv)

    fld = load_field(args.geojson, name=args.field)
    params = CoverageParams(cut_width=args.width, overlap=args.overlap,
                            headland_passes=args.headland, swath_angle_deg=args.angle)
    plan = plan_coverage(fld, params)
    print(f'Feld: {fld.name}  (Datum {fld.datum.lat:.6f}, {fld.datum.lon:.6f})')
    print(f'Streifenrichtung: {math.degrees(plan.swath_angle):.1f}° (ENU, 0° = Ost)')
    for k, v in plan.stats.items():
        print(f'  {k:22s} {v:10.1f}')
    if args.svg:
        with open(args.svg, 'w', encoding='utf-8') as fh:
            fh.write(to_svg(fld, plan))
        print(f'SVG geschrieben: {args.svg}')


if __name__ == '__main__':
    main()
