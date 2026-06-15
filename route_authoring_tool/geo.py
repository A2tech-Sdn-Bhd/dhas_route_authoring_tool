"""Geo helpers: lat/lon <-> local meters (equirectangular projection).

The editor draws and measures distances in a flat local-meters frame so all
the projection math (click-to-segment, snap tolerance, RDP epsilon) speaks
the same units. We use an equirectangular projection around an anchor lat,
which is accurate to ~0.1% at < 5 km scales — well below RTK fixes' own
noise for a parking yard.

Anchor convention: the first lat/lon in the trail. ``lat0`` is fixed for the
whole session so frame coordinates are comparable across edits.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple


EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True)
class LocalFrame:
    """Equirectangular projection around a fixed (lat0, lon0) anchor."""
    lat0_deg: float
    lon0_deg: float

    @property
    def _cos_lat0(self) -> float:
        return math.cos(math.radians(self.lat0_deg))

    def to_xy(self, lat_deg: float, lon_deg: float) -> Tuple[float, float]:
        x = math.radians(lon_deg - self.lon0_deg) * EARTH_RADIUS_M * self._cos_lat0
        y = math.radians(lat_deg - self.lat0_deg) * EARTH_RADIUS_M
        return x, y

    def to_latlon(self, x_m: float, y_m: float) -> Tuple[float, float]:
        lat = self.lat0_deg + math.degrees(y_m / EARTH_RADIUS_M)
        lon = self.lon0_deg + math.degrees(x_m / (EARTH_RADIUS_M * self._cos_lat0))
        return lat, lon

    def batch_to_xy(self, latlons: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
        return [self.to_xy(lat, lon) for lat, lon in latlons]

    def batch_to_latlon(self, xys: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
        return [self.to_latlon(x, y) for x, y in xys]


def frame_from_latlons(latlons: Iterable[Tuple[float, float]]) -> LocalFrame:
    """Anchor the frame on the FIRST sample in the iterable.

    First-sample anchoring (rather than centroid) keeps the frame stable when
    the user adds points off the original trail — the centroid would shift
    every edit and break absolute position comparisons.
    """
    it = iter(latlons)
    try:
        lat, lon = next(it)
    except StopIteration as exc:
        raise ValueError('Cannot anchor a local frame on an empty trail.') from exc
    return LocalFrame(lat0_deg=float(lat), lon0_deg=float(lon))


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance between two lat/lon points, in meters."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def polyline_length_m_latlon(latlons: Sequence[Tuple[float, float]]) -> float:
    if len(latlons) < 2:
        return 0.0
    total = 0.0
    for (a_lat, a_lon), (b_lat, b_lon) in zip(latlons, latlons[1:]):
        total += haversine_m(a_lat, a_lon, b_lat, b_lon)
    return total


def polyline_length_m_xy(xys: Sequence[Tuple[float, float]]) -> float:
    if len(xys) < 2:
        return 0.0
    total = 0.0
    for (x1, y1), (x2, y2) in zip(xys, xys[1:]):
        total += math.hypot(x2 - x1, y2 - y1)
    return total


__all__ = [
    'EARTH_RADIUS_M',
    'LocalFrame',
    'frame_from_latlons',
    'haversine_m',
    'polyline_length_m_latlon',
    'polyline_length_m_xy',
]
