"""Command-line entry points.

Two tools share one config file:

* ``bag_to_route`` — read a bag, downsample, write a waypoints YAML. No GUI.
* ``route_editor`` — open the matplotlib editor on one or more existing
  YAMLs, or extract from a bag first and then open. Save back to disk
  on demand.

Both honour the same ``--config`` file. Common per-run overrides
(``--bag``, ``--input``, ``--output``) are also accepted on the command
line and win over the config file's values. ``--input`` accepts multiple
paths so the editor can hold more than one route at a time.
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
    parser.add_argument(
        '--input', nargs='+',
        help=(
            'One or more existing waypoint YAML paths. Overrides io.input_waypoints. '
            'Pass multiple to load several routes into the editor at once.'
        ),
    )
    parser.add_argument(
        '--output', '-o', nargs='+',
        help=(
            'Output YAML path(s). With a single input you may pass one path; with '
            'multiple inputs pass N paths positionally paired with the inputs, or '
            'omit to default each to <input_stem>_edited.yaml.'
        ),
    )
    return parser


def _resolve_config(args: argparse.Namespace) -> RouteAuthoringConfig:
    path = args.config or (default_config_path() if os.path.exists(default_config_path()) else None)
    cfg = load_config(path)
    if args.bag:
        cfg.io.bag_path = args.bag
    # input/output overrides are list-shaped now — handled in the route_editor
    # entry point so the config dataclass stays single-string.
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


def _resolve_input_paths(args: argparse.Namespace, cfg: RouteAuthoringConfig) -> List[str]:
    """CLI multi-input wins over the single-string config field."""
    if args.input:
        return [os.path.expanduser(p) for p in args.input]
    if cfg.io.input_waypoints:
        return [os.path.expanduser(cfg.io.input_waypoints)]
    return []


def _resolve_output_paths(
    args: argparse.Namespace,
    cfg: RouteAuthoringConfig,
    input_paths: List[str],
) -> List[str]:
    """Pair output paths to inputs. Defaults to <stem>_edited.yaml per input."""
    n = len(input_paths)
    if args.output:
        outs = [os.path.expanduser(p) for p in args.output]
        if len(outs) == 1 and n > 1:
            raise ValueError(
                f'--output got 1 path but {n} inputs were supplied; pass {n} outputs '
                f'positionally, or omit --output to use <input>_edited.yaml defaults.'
            )
        if len(outs) not in (0, n):
            raise ValueError(
                f'--output count ({len(outs)}) must match --input count ({n}).'
            )
        return outs
    # No CLI --output. Single-input case can fall back to cfg.io.output_path.
    if n == 1 and cfg.io.output_path:
        return [os.path.expanduser(cfg.io.output_path)]
    # Multi-input case (or no config output_path): default per-input.
    return [_default_output_for_yaml(p) for p in input_paths]


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

    # bag_to_route is intentionally single-output. If the user passed multiple
    # --output paths it's almost certainly a mistake — fail loudly.
    if args.output and len(args.output) > 1:
        print(
            '[bag_to_route] only one --output is meaningful for a single bag extract; '
            f'got {len(args.output)}.',
            file=sys.stderr,
        )
        return 2

    cli_output = args.output[0] if args.output else None
    output_path = cli_output or cfg.io.output_path or _default_output_for_bag(cfg.io.bag_path)

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
        'Open the interactive route editor. Source can be a bag or one or more YAMLs.'
    )
    args = parser.parse_args(argv)
    try:
        cfg = _resolve_config(args)
    except Exception as exc:  # noqa: BLE001
        print(f'[route_editor] config error: {exc}', file=sys.stderr)
        return 2

    # Import LayerSpec lazily so the bag-only path doesn't pay for matplotlib.
    from .editor import LayerSpec

    layer_specs: List[LayerSpec] = []
    frame: Optional[LocalFrame] = None

    input_paths = _resolve_input_paths(args, cfg)

    if input_paths:
        try:
            output_paths = _resolve_output_paths(args, cfg, input_paths)
        except ValueError as exc:
            print(f'[route_editor] {exc}', file=sys.stderr)
            return 2

        loaded_routes: List[List[Tuple[float, float]]] = []
        for path in input_paths:
            if not os.path.exists(path):
                print(f'[route_editor] input not found: {path}', file=sys.stderr)
                return 2
            wf = load_waypoints(path)
            if not wf.waypoints_latlon:
                print(f'[route_editor] {path}: file has no waypoints.', file=sys.stderr)
                return 1
            loaded_routes.append(wf.waypoints_latlon)

        # Shared frame anchored on the first route's first waypoint so every
        # route projects into the same x/y plane.
        frame = frame_from_latlons(loaded_routes[0])
        for i, (in_path, out_path, waypoints) in enumerate(
            zip(input_paths, output_paths, loaded_routes)
        ):
            layer_specs.append(LayerSpec(
                waypoints_latlon=waypoints,
                output_path=out_path,
                reload_path=in_path,
                raw_trail_latlon=None,
                label=os.path.basename(in_path),
            ))

    elif cfg.io.bag_path:
        try:
            waypoints_latlon, raw_latlons, frame = _extract_from_bag(cfg)
        except Exception as exc:  # noqa: BLE001
            print(f'[route_editor] extraction failed: {exc}', file=sys.stderr)
            return 1

        # Bag extraction is single-route. Honour --output[0] if given, else
        # config, else default-from-bag.
        if args.output and len(args.output) > 1:
            print(
                '[route_editor] --output takes a single path when --bag is the '
                f'source; got {len(args.output)}.',
                file=sys.stderr,
            )
            return 2
        cli_output = args.output[0] if args.output else None
        out_path = (
            os.path.expanduser(cli_output) if cli_output
            else (os.path.expanduser(cfg.io.output_path) if cfg.io.output_path
                  else _default_output_for_bag(cfg.io.bag_path))
        )
        layer_specs.append(LayerSpec(
            waypoints_latlon=waypoints_latlon,
            output_path=out_path,
            reload_path=None,
            raw_trail_latlon=raw_latlons if raw_latlons else None,
            label=os.path.basename(cfg.io.bag_path.rstrip('/')) or 'from_bag',
        ))

    else:
        print(
            '[route_editor] no input. Set io.bag_path or io.input_waypoints in the config, '
            'or pass --bag / --input.',
            file=sys.stderr,
        )
        return 2

    if not layer_specs or frame is None:
        print('[route_editor] no waypoints to edit.', file=sys.stderr)
        return 1

    # Import here so matplotlib is only loaded for the interactive path.
    from .editor import RouteEditor

    editor = RouteEditor(
        layers=layer_specs,
        frame=frame,
        editor_cfg=cfg.editor,
        output_cfg=cfg.output,
        nominal_speed_mps=cfg.estimation.nominal_speed_mps,
    )
    editor.run()
    return 0


__all__ = ['bag_to_route_main', 'route_editor_main']
