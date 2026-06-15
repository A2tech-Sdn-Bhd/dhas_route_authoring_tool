"""Click-to-polyline projection for the editor.

When the user clicks anywhere near the path, we don't insert at the raw
click — we project the click perpendicularly onto the nearest segment of
the current waypoint polyline and insert at that foot of perpendicular.
The math mirrors ``_project_to_polyline`` in ``goal_directed_heading.py``
so editor behaviour matches what the path follower itself does at runtime.

All coordinates here are in the local-meters frame from ``geo.LocalFrame``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class ProjectionHit:
    """Result of projecting a query point onto a polyline."""
    segment_index: int          # index i means the segment between waypoint i and i+1
    t: float                    # fractional position along the segment, 0..1
    x: float                    # projected x in local meters
    y: float                    # projected y in local meters
    distance_m: float           # perpendicular distance from query to (x, y)
    segment_length_m: float     # |waypoint[i+1] - waypoint[i]|


def project_point_to_segment(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> Tuple[float, float, float, float]:
    """Project (px, py) onto segment a->b.

    Returns (foot_x, foot_y, t, dist). ``t`` is clamped to [0, 1] so the foot
    never escapes the segment endpoints.
    """
    dx = bx - ax
    dy = by - ay
    seg_sq = dx * dx + dy * dy
    if seg_sq <= 1e-12:
        return ax, ay, 0.0, math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / seg_sq
    t_clamped = max(0.0, min(1.0, t))
    fx = ax + t_clamped * dx
    fy = ay + t_clamped * dy
    return fx, fy, t_clamped, math.hypot(px - fx, py - fy)


def project_to_polyline(
    px: float,
    py: float,
    polyline_xy: Sequence[Tuple[float, float]],
) -> Optional[ProjectionHit]:
    """Find the closest segment of ``polyline_xy`` to (px, py).

    Returns ``None`` if the polyline has fewer than two vertices. Otherwise
    returns the best :class:`ProjectionHit`. The caller decides whether to
    accept the hit based on its ``distance_m`` and a snap tolerance.
    """
    if len(polyline_xy) < 2:
        return None

    best: Optional[ProjectionHit] = None
    for i in range(len(polyline_xy) - 1):
        ax, ay = polyline_xy[i]
        bx, by = polyline_xy[i + 1]
        fx, fy, t, dist = project_point_to_segment(px, py, ax, ay, bx, by)
        if best is None or dist < best.distance_m:
            seg_len = math.hypot(bx - ax, by - ay)
            best = ProjectionHit(
                segment_index=i,
                t=t,
                x=fx,
                y=fy,
                distance_m=dist,
                segment_length_m=seg_len,
            )
    return best


def nearest_waypoint(
    px: float,
    py: float,
    polyline_xy: Sequence[Tuple[float, float]],
) -> Tuple[int, float]:
    """Return (index, distance_m) of the closest waypoint to (px, py).

    Returns (-1, +inf) on empty input.
    """
    best_idx = -1
    best_dist = float('inf')
    for idx, (x, y) in enumerate(polyline_xy):
        d = math.hypot(px - x, py - y)
        if d < best_dist:
            best_dist = d
            best_idx = idx
    return best_idx, best_dist


def insert_point_at_hit(
    polyline_xy: List[Tuple[float, float]],
    hit: ProjectionHit,
) -> List[Tuple[float, float]]:
    """Return a NEW polyline with the projected point inserted at the right slot.

    The projected point goes between ``hit.segment_index`` and
    ``hit.segment_index + 1`` (i.e. it splits the segment in two).
    """
    new_pt = (hit.x, hit.y)
    insertion = hit.segment_index + 1
    return polyline_xy[:insertion] + [new_pt] + polyline_xy[insertion:]


__all__ = [
    'ProjectionHit',
    'project_point_to_segment',
    'project_to_polyline',
    'nearest_waypoint',
    'insert_point_at_hit',
]
