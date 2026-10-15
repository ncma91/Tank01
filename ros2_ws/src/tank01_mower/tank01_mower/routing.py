"""Route graph for km-scale transit between depot and fields.

Routes are GeoJSON LineStrings (already converted to ENU). Line endpoints and points
where lines come within ``snap`` metres of each other become graph nodes; the shortest
path is found with Dijkstra and returned as a densified waypoint list.
"""

from __future__ import annotations

import heapq
import math
from typing import Sequence

from tank01_mower.polygon import Point


class RouteGraph:

    def __init__(self, lines: Sequence[Sequence[Point]], snap: float = 2.0):
        self.snap = snap
        self.nodes: list[Point] = []
        self.edges: dict[int, list[tuple[int, float]]] = {}
        for line in lines:
            prev = None
            for pt in line:
                idx = self._node(pt)
                if prev is not None and prev != idx:
                    d = math.dist(self.nodes[prev], self.nodes[idx])
                    self.edges.setdefault(prev, []).append((idx, d))
                    self.edges.setdefault(idx, []).append((prev, d))
                prev = idx

    def _node(self, pt: Point) -> int:
        for i, n in enumerate(self.nodes):
            if math.dist(n, pt) <= self.snap:
                return i
        self.nodes.append((float(pt[0]), float(pt[1])))
        return len(self.nodes) - 1

    def nearest_node(self, pt: Point) -> int:
        if not self.nodes:
            raise ValueError('route graph is empty')
        return min(range(len(self.nodes)), key=lambda i: math.dist(self.nodes[i], pt))

    def shortest_path(self, start: Point, goal: Point) -> list[Point]:
        """Waypoints from start to goal via the graph (start/goal included)."""
        s, g = self.nearest_node(start), self.nearest_node(goal)
        dist = {s: 0.0}
        prev: dict[int, int] = {}
        heap = [(0.0, s)]
        while heap:
            d, u = heapq.heappop(heap)
            if u == g:
                break
            if d > dist.get(u, math.inf):
                continue
            for v, w in self.edges.get(u, []):
                nd = d + w
                if nd < dist.get(v, math.inf):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(heap, (nd, v))
        if g not in dist:
            raise ValueError('goal not reachable on route graph')
        chain = [g]
        while chain[-1] != s:
            chain.append(prev[chain[-1]])
        pts = [self.nodes[i] for i in reversed(chain)]
        return [tuple(start)] + pts + [tuple(goal)]


def resample(path: Sequence[Point], spacing: float) -> list[Point]:
    """Resample a polyline to roughly equidistant points (keeps first and last)."""
    if len(path) < 2:
        return list(path)
    out = [tuple(path[0])]
    carry = 0.0
    for a, b in zip(path, path[1:]):
        seg = math.dist(a, b)
        t = spacing - carry
        while t < seg:
            out.append((a[0] + (b[0] - a[0]) * t / seg, a[1] + (b[1] - a[1]) * t / seg))
            t += spacing
        carry = seg - (t - spacing)
    if math.dist(out[-1], path[-1]) > 1e-6:
        out.append(tuple(path[-1]))
    return out


def path_length(path: Sequence[Point]) -> float:
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))
