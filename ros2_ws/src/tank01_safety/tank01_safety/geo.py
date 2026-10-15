"""WGS84 <-> local ENU conversion and GeoJSON helpers. Pure Python.

Uses the exact geodetic -> ECEF -> ENU transformation, so it stays accurate over
several kilometres around the datum (unlike an equirectangular approximation).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

_A = 6378137.0                    # WGS84 semi-major axis
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    s = math.sin(lat)
    n = _A / math.sqrt(1.0 - _E2 * s * s)
    x = (n + alt) * math.cos(lat) * math.cos(lon)
    y = (n + alt) * math.cos(lat) * math.sin(lon)
    z = (n * (1.0 - _E2) + alt) * s
    return x, y, z


def ecef_to_geodetic(x: float, y: float, z: float) -> tuple[float, float, float]:
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - _E2))
    alt = 0.0
    for _ in range(6):
        s = math.sin(lat)
        n = _A / math.sqrt(1.0 - _E2 * s * s)
        alt = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1.0 - _E2 * n / (n + alt)))
    return math.degrees(lat), math.degrees(lon), alt


@dataclass(frozen=True)
class Datum:
    lat: float
    lon: float
    alt: float = 0.0

    def to_enu(self, lat: float, lon: float,
               alt: float | None = None) -> tuple[float, float, float]:
        alt = self.alt if alt is None else alt
        x0, y0, z0 = geodetic_to_ecef(self.lat, self.lon, self.alt)
        x, y, z = geodetic_to_ecef(lat, lon, alt)
        dx, dy, dz = x - x0, y - y0, z - z0
        la, lo = math.radians(self.lat), math.radians(self.lon)
        sl, cl, so, co = math.sin(la), math.cos(la), math.sin(lo), math.cos(lo)
        e = -so * dx + co * dy
        n = -sl * co * dx - sl * so * dy + cl * dz
        u = cl * co * dx + cl * so * dy + sl * dz
        return e, n, u

    def to_geodetic(self, e: float, n: float, u: float = 0.0) -> tuple[float, float, float]:
        x0, y0, z0 = geodetic_to_ecef(self.lat, self.lon, self.alt)
        la, lo = math.radians(self.lat), math.radians(self.lon)
        sl, cl, so, co = math.sin(la), math.cos(la), math.sin(lo), math.cos(lo)
        dx = -so * e - sl * co * n + cl * co * u
        dy = co * e - sl * so * n + cl * so * u
        dz = cl * n + sl * u
        return ecef_to_geodetic(x0 + dx, y0 + dy, z0 + dz)

    def ring_to_enu(self, ring: list[list[float]]) -> list[tuple[float, float]]:
        """GeoJSON ring ([lon, lat(, alt)] ...) -> [(e, n)], closing point removed."""
        pts = [self.to_enu(c[1], c[0])[:2] for c in ring]
        if len(pts) > 1 and math.dist(pts[0], pts[-1]) < 1e-6:
            pts = pts[:-1]
        return pts


def load_geojson(path: str | Path) -> dict[str, Any]:
    with open(path, encoding='utf-8') as fh:
        data = json.load(fh)
    if data.get('type') != 'FeatureCollection':
        raise ValueError(f'{path}: expected a GeoJSON FeatureCollection')
    return data


def datum_from_geojson(data: dict[str, Any]) -> Datum:
    """Use properties.datum if present, otherwise the first coordinate found."""
    d = data.get('properties', {}).get('datum')
    if d:
        return Datum(float(d[0]), float(d[1]), float(d[2]) if len(d) > 2 else 0.0)
    for feat in data.get('features', []):
        coords = feat['geometry']['coordinates']
        while isinstance(coords[0], list):
            coords = coords[0]
        return Datum(coords[1], coords[0])
    raise ValueError('GeoJSON has no coordinates')


def features_by_role(data: dict[str, Any], role: str) -> list[dict[str, Any]]:
    return [f for f in data.get('features', [])
            if f.get('properties', {}).get('role') == role]
