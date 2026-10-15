"""Field model loaded from GeoJSON and converted to the local ENU (map) frame."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from tank01_safety.geo import Datum, datum_from_geojson, features_by_role, load_geojson

from tank01_mower.polygon import Point, Ring, circle, ensure_ccw


@dataclass
class Field:
    name: str
    datum: Datum
    boundary: Ring
    holes: list[Ring] = field(default_factory=list)          # field-internal holes
    keepouts: list[tuple[Ring, float]] = field(default_factory=list)  # (polygon, buffer)
    entry: Point | None = None
    routes: list[list[Point]] = field(default_factory=list)
    depot: Point | None = None


def load_field(path: str | Path, datum: Datum | None = None, name: str | None = None) -> Field:
    """Load the first ``field`` feature (or the one called *name*) of a GeoJSON file."""
    data = load_geojson(path)
    datum = datum or datum_from_geojson(data)
    fields = features_by_role(data, 'field')
    if name is not None:
        fields = [f for f in fields if f['properties'].get('name') == name]
    if not fields:
        raise ValueError(f'{path}: no field feature' + (f' named {name!r}' if name else ''))
    feat = fields[0]
    if feat['geometry']['type'] != 'Polygon':
        raise ValueError('field geometry must be a Polygon')
    rings = feat['geometry']['coordinates']
    fld = Field(
        name=feat['properties'].get('name', Path(path).stem),
        datum=datum,
        boundary=ensure_ccw(datum.ring_to_enu(rings[0])),
        holes=[ensure_ccw(datum.ring_to_enu(r)) for r in rings[1:]],
    )
    for k in features_by_role(data, 'keepout'):
        buf = float(k['properties'].get('buffer_m', 0.5))
        geom = k['geometry']
        if geom['type'] == 'Point':
            lon, lat = geom['coordinates'][:2]
            e, n, _ = datum.to_enu(lat, lon)
            fld.keepouts.append((circle((e, n), max(buf, 0.3)), 0.0))
        elif geom['type'] == 'Polygon':
            fld.keepouts.append((ensure_ccw(datum.ring_to_enu(geom['coordinates'][0])), buf))
    for e in features_by_role(data, 'entry'):
        lon, lat = e['geometry']['coordinates'][:2]
        fld.entry = datum.to_enu(lat, lon)[:2]
    for d in features_by_role(data, 'depot'):
        lon, lat = d['geometry']['coordinates'][:2]
        fld.depot = datum.to_enu(lat, lon)[:2]
    for r in features_by_role(data, 'route'):
        if r['geometry']['type'] == 'LineString':
            fld.routes.append([datum.to_enu(c[1], c[0])[:2] for c in r['geometry']['coordinates']])
    return fld
