"""Read/write the YAML schema consumed by hybrid_smooth_path_follower.

Schema (from ``visual_debug_node._load_waypoints``):

    coordinate_mode: latlon
    waypoint_order: lat_lon   # or lon_lat
    waypoints:
      - [lat, lon]            # row ordering follows waypoint_order
      - ...

The loader also accepts dict rows ``{lat, lon}`` (or ``{latitude, longitude}``),
which ignore ``waypoint_order``. We write list rows because the existing
production routes use them and they're slightly more compact.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import yaml


_VALID_ORDERS = ('lat_lon', 'lon_lat')


@dataclass
class WaypointFile:
    """In-memory representation of a hybrid_smooth_path_follower waypoint YAML."""
    waypoints_latlon: List[Tuple[float, float]]  # always stored (lat, lon) internally
    waypoint_order: str = 'lat_lon'              # how to write rows on save
    coordinate_mode: str = 'latlon'              # consumer accepts xy too, but we author latlon


def _load_yaml_dict(path: str) -> dict:
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(
            f'{path}: top level must be a mapping with coordinate_mode/waypoint_order/waypoints.'
        )
    return data


def load_waypoints(path: str) -> WaypointFile:
    """Read a YAML written for hybrid_smooth_path_follower.

    Accepts both list rows (respecting ``waypoint_order``) and dict rows
    (which carry their own keys). Always returns waypoints as (lat, lon).
    """
    expanded = os.path.expanduser(path)
    if not os.path.exists(expanded):
        raise FileNotFoundError(f'Waypoint file not found: {expanded}')

    data = _load_yaml_dict(expanded)
    mode = str(data.get('coordinate_mode', 'latlon')).lower().strip()
    if mode != 'latlon':
        raise ValueError(
            f'{expanded}: coordinate_mode={mode!r} is not supported by this tool '
            '(only "latlon" is authored here).'
        )
    order = str(
        data.get('waypoint_order', data.get('gps_waypoint_order', 'lat_lon'))
    ).lower().strip()
    if order not in _VALID_ORDERS:
        raise ValueError(
            f'{expanded}: waypoint_order={order!r} must be one of {_VALID_ORDERS}.'
        )

    rows = data.get('waypoints', [])
    if not isinstance(rows, list):
        raise ValueError(f'{expanded}: "waypoints" must be a list.')

    pts: List[Tuple[float, float]] = []
    for idx, row in enumerate(rows):
        try:
            pts.append(_parse_row(row, order))
        except Exception as exc:  # noqa: BLE001 - one bad row shouldn't kill the load
            raise ValueError(f'{expanded}: row {idx} is malformed: {row} ({exc})') from exc

    return WaypointFile(waypoints_latlon=pts, waypoint_order=order, coordinate_mode='latlon')


def _parse_row(row, order: str) -> Tuple[float, float]:
    if isinstance(row, dict):
        lat = row.get('lat', row.get('latitude'))
        lon = row.get('lon', row.get('lng', row.get('longitude')))
        if lat is None or lon is None:
            raise ValueError('dict row needs lat/latitude and lon/lng/longitude')
        return float(lat), float(lon)

    if not (isinstance(row, (list, tuple)) and len(row) >= 2):
        raise ValueError('list row must have at least two numbers')
    a, b = float(row[0]), float(row[1])
    if order == 'lon_lat':
        return b, a   # second is lat
    return a, b       # lat_lon


def save_waypoints(
    waypoints_latlon: Sequence[Tuple[float, float]],
    output_path: str,
    waypoint_order: str = 'lat_lon',
    coord_decimals: int = 8,
    header_comment: str | None = None,
) -> str:
    """Write a YAML the hybrid_smooth_path_follower will load directly.

    Returns the absolute path that was written. Existing files are
    overwritten (the editor manages its own backups if it wants any).
    """
    if waypoint_order not in _VALID_ORDERS:
        raise ValueError(f'waypoint_order must be one of {_VALID_ORDERS}, got {waypoint_order!r}')
    if not waypoints_latlon:
        raise ValueError('Refusing to write an empty waypoint file.')

    expanded = os.path.abspath(os.path.expanduser(output_path))
    parent = os.path.dirname(expanded)
    if parent:
        os.makedirs(parent, exist_ok=True)

    fmt = f'{{:.{int(coord_decimals)}f}}'
    lines: List[str] = []
    if header_comment:
        for ln in header_comment.splitlines():
            lines.append(f'# {ln}' if ln else '#')
    lines.append('coordinate_mode: latlon')
    lines.append(f'waypoint_order: {waypoint_order}')
    lines.append('waypoints:')
    for lat, lon in waypoints_latlon:
        if waypoint_order == 'lon_lat':
            a, b = fmt.format(lon), fmt.format(lat)
        else:
            a, b = fmt.format(lat), fmt.format(lon)
        lines.append(f'  - [{a}, {b}]')
    lines.append('')

    with open(expanded, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))
    return expanded


__all__ = ['WaypointFile', 'load_waypoints', 'save_waypoints']
