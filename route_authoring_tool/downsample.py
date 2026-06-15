"""Downsample a dense GPS trail to a sparse waypoint polyline.

Two-stage pipeline, operating in the local-meters frame:

1. **Ramer-Douglas-Peucker** with an epsilon in METERS. Keeps corners,
   drops straights, gives non-uniform spacing that matches the route's
   own curvature.

2. **Max-segment-length pass**. After RDP, any segment longer than
   ``max_segment_m`` is split with evenly-spaced intermediates. This
   guarantees the consumer's lookahead window (~2.5 m spacing assumption
   in ``hybrid_smooth_path_follower``) always has enough samples.

Both stages preserve the FIRST and LAST samples of the input. The output
is a list of ``(lat, lon)`` pairs ready to write to the consumer YAML.
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

from .geo import LocalFrame, frame_from_latlons


def rdp_xy(
    points: Sequence[Tuple[float, float]],
    epsilon_m: float,
) -> List[Tuple[float, float]]:
    """Ramer-Douglas-Peucker in a flat XY frame.

    Iterative implementation (a stack, not recursion) so 10k-point trails
    don't blow Python's recursion limit.
    """
    n = len(points)
    if n < 3 or epsilon_m <= 0.0:
        return list(points)

    keep = [False] * n
    keep[0] = True
    keep[-1] = True

    stack: List[Tuple[int, int]] = [(0, n - 1)]
    while stack:
        i0, i1 = stack.pop()
        if i1 <= i0 + 1:
            continue
        ax, ay = points[i0]
        bx, by = points[i1]
        max_dist = -1.0
        max_idx = -1
        for k in range(i0 + 1, i1):
            px, py = points[k]
            d = _perp_distance(px, py, ax, ay, bx, by)
            if d > max_dist:
                max_dist = d
                max_idx = k
        if max_dist > epsilon_m and max_idx != -1:
            keep[max_idx] = True
            stack.append((i0, max_idx))
            stack.append((max_idx, i1))

    return [p for p, k in zip(points, keep) if k]


def _perp_distance(
    px: float, py: float,
    ax: float, ay: float,
    bx: float, by: float,
) -> float:
    """Perpendicular distance from (px,py) to the infinite line through a,b.

    Falls back to the point-to-point distance when a == b so we never divide
    by zero on duplicate samples.
    """
    dx = bx - ax
    dy = by - ay
    seg_sq = dx * dx + dy * dy
    if seg_sq <= 1e-12:
        return math.hypot(px - ax, py - ay)
    # |(b - a) x (p - a)| / |b - a|
    cross = dx * (py - ay) - dy * (px - ax)
    return abs(cross) / math.sqrt(seg_sq)


def enforce_max_segment_xy(
    points: Sequence[Tuple[float, float]],
    max_segment_m: float,
) -> List[Tuple[float, float]]:
    """Split segments longer than ``max_segment_m`` with evenly-spaced points.

    The number of inserts per segment is ``ceil(seg_len / max) - 1`` so the
    largest resulting sub-segment is just at or below the cap.
    """
    if len(points) < 2 or max_segment_m <= 0.0:
        return list(points)

    out: List[Tuple[float, float]] = [points[0]]
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        seg_len = math.hypot(bx - ax, by - ay)
        if seg_len > max_segment_m + 1e-9:
            n_inserts = max(1, int(math.ceil(seg_len / max_segment_m)) - 1)
            for k in range(1, n_inserts + 1):
                t = k / (n_inserts + 1)
                out.append((ax + t * (bx - ax), ay + t * (by - ay)))
        out.append((bx, by))
    return out


def downsample_trail_latlon(
    latlons: Sequence[Tuple[float, float]],
    rdp_epsilon_m: float,
    max_segment_m: float,
    trim_head: int = 0,
    trim_tail: int = 0,
    frame: LocalFrame | None = None,
) -> Tuple[List[Tuple[float, float]], LocalFrame]:
    """End-to-end downsampler.

    Returns ``(waypoints_latlon, frame)`` so the caller can reuse the same
    anchor when projecting clicks in the editor. If ``frame`` is supplied,
    it's reused; otherwise a fresh frame is anchored on the first sample.
    """
    if not latlons:
        raise ValueError('Cannot downsample an empty trail.')

    trimmed = list(latlons)
    if trim_head:
        trimmed = trimmed[trim_head:]
    if trim_tail:
        trimmed = trimmed[:-trim_tail] if trim_tail < len(trimmed) else []
    if not trimmed:
        raise ValueError('Trim removed all samples; reduce trim_head/trim_tail.')

    fr = frame if frame is not None else frame_from_latlons(trimmed)
    xys = fr.batch_to_xy(trimmed)

    simplified = rdp_xy(xys, rdp_epsilon_m)
    spaced = enforce_max_segment_xy(simplified, max_segment_m)
    out_latlon = fr.batch_to_latlon(spaced)
    return out_latlon, fr


__all__ = [
    'rdp_xy',
    'enforce_max_segment_xy',
    'downsample_trail_latlon',
]
