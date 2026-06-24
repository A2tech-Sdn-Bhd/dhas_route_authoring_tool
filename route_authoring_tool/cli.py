"""Command-line entry points.

Two tools share one config file:

* ``bag_to_route`` — read a bag, downsample, write a waypoints YAML. No GUI.
* ``route_editor`` — open the matplotlib editor on an existing YAML, or
  extract from a bag first and then open. Save back to disk on demand.

Both honour the same ``--config`` file. Common per-run overrides
(``--bag``, ``--input``, ``--output``) are also accepted on the command
line and win over the config file's values.
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional, Tuple

from .bag_reader import read_bag
from .config_loader import RouteAuthoringConfig, default_config_path, load_config
from .downsample import downsample_trail_latlon
from .geo import LocalFrame, frame_from_latlons
from .waypoint_io import load_waypoints, save_waypoints


# ---------------------------------------------------------------- shared helpers
def _build_common_parser(description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument(
        '--config', '-c',
        help='Path to route_authoring.yaml. Defaults to the installed share copy.',
    )
    parser.add_argument('--bag', help='Override io.bag_path from the config.')
    parser.add_argument('--input', help='Override io.input_waypoints from the config.')
    parser.add_argument('--output', '-o', help='Override io.output_path from the config.')
    return parser


def _resolve_config(args: argparse.Namespace) -> RouteAuthoringConfig:
    path = args.config or (default_config_path() if os.path.exists(default_config_path()) else None)
    cfg = load_config(path)
    if args.bag:
        cfg.io.bag_path = args.bag
    if args.input:
        cfg.io.input_waypoints = args.input
    if args.output:
        cfg.io.output_path = args.output
    return cfg


def _default_output_for_bag(bag_path: str) -> str:
    parent = os.path.dirname(os.path.abspath(bag_path.rstrip('/')))
    name = os.path.basename(os.path.abspath(bag_path.rstrip('/'))) or 'route_from_bag'
    return os.path.join(parent, f'{name}_waypoints.yaml')


def _default_output_for_yaml(input_path: str) -> str:
    parent = os.path.dirname(os.path.abspath(input_path))
    stem, ext = os.path.splitext(os.path.basename(input_path))
    return os.path.join(parent, f'{stem}_edited{ext or ".yaml"}')


def _extract_from_bag(
    cfg: RouteAuthoringConfig,
) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]], LocalFrame]:
    """Read the bag, downsample, return (waypoints, raw_trail, frame)."""
    trail = read_bag(
        cfg.io.bag_path,
        rtk_fix_topic=cfg.bag.rtk_fix_topic,
        rtk_heading_topic=cfg.bag.rtk_heading_topic,
        nav_sat_status_min=cfg.bag.nav_sat_status_min,
        pre_decimate_every_n=cfg.bag.pre_decimate_every_n,
    )
    if not trail:
        raise RuntimeError(
            f'No usable fixes found on {cfg.bag.rtk_fix_topic} in {cfg.io.bag_path}. '
            f'Counts seen: {trail.counts}.'
        )
    raw_latlons = trail.latlons
    waypoints_latlon, frame = downsample_trail_latlon(
        raw_latlons,
        rdp_epsilon_m=cfg.downsample.rdp_epsilon_m,
        max_segment_m=cfg.downsample.max_segment_m,
        trim_head=cfg.downsample.trim_head_samples,
        trim_tail=cfg.downsample.trim_tail_samples,
    )
    return waypoints_latlon, raw_latlons, frame


# ---------------------------------------------------------------- bag_to_route
def bag_to_route_main(argv: Optional[List[str]] = None) -> int:
    parser = _build_common_parser(
        'Extract a waypoint YAML from a rosbag2 directory (no GUI).'
    )
    args = parser.parse_args(argv)
    try:
        cfg = _resolve_config(args)
    except Exception as exc:  # noqa: BLE001 - top-level CLI surface
        print(f'[bag_to_route] config error: {exc}', file=sys.stderr)
        return 2

    if not cfg.io.bag_path:
        print('[bag_to_route] no bag path set. Pass --bag or set io.bag_path in the config.',
              file=sys.stderr)
        return 2

    output_path = cfg.io.output_path or _default_output_for_bag(cfg.io.bag_path)

    try:
        waypoints_latlon, raw_latlons, _frame = _extract_from_bag(cfg)
    except Exception as exc:  # noqa: BLE001
        print(f'[bag_to_route] extraction failed: {exc}', file=sys.stderr)
        return 1

    saved = save_waypoints(
        waypoints_latlon,
        output_path,
        waypoint_order=cfg.output.waypoint_order,
        coord_decimals=cfg.output.coord_decimals,
        nominal_speed_mps=cfg.estimation.nominal_speed_mps,
        header_comment=(
            f'Extracted by route_authoring_tool from {cfg.io.bag_path}\n'
            f'raw fix samples kept: {len(raw_latlons)}\n'
            f'RDP eps: {cfg.downsample.rdp_epsilon_m} m   '
            f'max segment: {cfg.downsample.max_segment_m} m'
        ),
    )
    print(
        f'[bag_to_route] {len(raw_latlons)} raw fixes -> {len(waypoints_latlon)} waypoints   '
        f'wrote {saved}'
    )
    return 0


# ---------------------------------------------------------------- route_editor
def route_editor_main(argv: Optional[List[str]] = None) -> int:
    parser = _build_common_parser(
        'Open the interactive route editor. Source can be a bag or an existing YAML.'
    )
    args = parser.parse_args(argv)
    try:
        cfg = _resolve_config(args)
    except Exception as exc:  # noqa: BLE001
        print(f'[route_editor] config error: {exc}', file=sys.stderr)
        return 2

    raw_latlons: List[Tuple[float, float]] = []
    waypoints_latlon: List[Tuple[float, float]] = []
    frame: Optional[LocalFrame] = None
    reload_path: Optional[str] = None
    output_path: str = cfg.io.output_path

    if cfg.io.input_waypoints:
        wf = load_waypoints(cfg.io.input_waypoints)
        waypoints_latlon = wf.waypoints_latlon
        frame = frame_from_latlons(waypoints_latlon)
        reload_path = cfg.io.input_waypoints
        if not output_path:
            output_path = cfg.io.input_waypoints
    elif cfg.io.bag_path:
        try:
            waypoints_latlon, raw_latlons, frame = _extract_from_bag(cfg)
        except Exception as exc:  # noqa: BLE001
            print(f'[route_editor] extraction failed: {exc}', file=sys.stderr)
            return 1
        if not output_path:
            output_path = _default_output_for_bag(cfg.io.bag_path)
    else:
        print(
            '[route_editor] no input. Set io.bag_path or io.input_waypoints in the config, '
            'or pass --bag / --input.',
            file=sys.stderr,
        )
        return 2

    if not waypoints_latlon or frame is None:
        print('[route_editor] no waypoints to edit.', file=sys.stderr)
        return 1

    # Import here so matplotlib is only loaded for the interactive path.
    from .editor import RouteEditor

    editor = RouteEditor(
        waypoints_latlon=waypoints_latlon,
        frame=frame,
        output_path=output_path,
        editor_cfg=cfg.editor,
        output_cfg=cfg.output,
        nominal_speed_mps=cfg.estimation.nominal_speed_mps,
        raw_trail_latlon=raw_latlons if raw_latlons else None,
        input_path_for_reload=reload_path,
    )
    editor.run()
    return 0


__all__ = ['bag_to_route_main', 'route_editor_main']
