"""Config loader.

Loads ``config/route_authoring.yaml`` (or a user-supplied path), deep-merges it
on top of the built-in defaults, and exposes a single ``RouteAuthoringConfig``
dataclass. Keeping defaults inline means the package still runs if the
installed share-dir copy is missing, and there is exactly one place to look
up the meaning of every knob.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import yaml


DEFAULTS: Dict[str, Any] = {
    'io': {
        'bag_path': '',
        'input_waypoints': '',
        'output_path': '',
    },
    'bag': {
        'rtk_fix_topic': '/rtk/fix',
        'rtk_heading_topic': '/rtk_heading/float',
        'imu_topic': '/imu/data',
        'odom_topic': '/Odometry',
        'nav_sat_status_min': 0,
        'pre_decimate_every_n': 1,
    },
    'downsample': {
        'rdp_epsilon_m': 0.4,
        'max_segment_m': 5.0,
        'trim_head_samples': 0,
        'trim_tail_samples': 0,
    },
    'output': {
        'waypoint_order': 'lat_lon',
        'coord_decimals': 8,
    },
    'estimation': {
        # Nominal forward speed used to estimate route drive-time at save
        # time. Written into the YAML as ``estimated_duration_min`` for the
        # UI/mission_server to consume. To override per route, edit the
        # ``estimated_duration_min`` field in the saved YAML directly —
        # ``route_authoring_tool`` only writes the auto-computed value.
        'nominal_speed_mps': 0.5,
    },
    'editor': {
        'figure_size_inches': [11.0, 9.0],
        'background_color': '#ffffff',
        'show_raw_trail': True,
        'raw_trail_alpha': 0.35,
        'raw_trail_color': '#888888',
        'waypoint_color': '#cc2222',
        'waypoint_marker_size': 9,
        'path_line_color': '#225a99',
        'path_line_width': 1.8,
        'selected_color': '#ffaa00',
        'selected_marker_size': 14,
        'waypoint_pick_radius_m': 0.6,
        'snap_tolerance_m': 1.5,
        'undo_depth': 50,
        'autosave_on_quit': False,
        'status_refresh_hz': 20.0,
    },
}


@dataclass
class IOConfig:
    bag_path: str
    input_waypoints: str
    output_path: str


@dataclass
class BagConfig:
    rtk_fix_topic: str
    rtk_heading_topic: str
    imu_topic: str
    odom_topic: str
    nav_sat_status_min: int
    pre_decimate_every_n: int


@dataclass
class DownsampleConfig:
    rdp_epsilon_m: float
    max_segment_m: float
    trim_head_samples: int
    trim_tail_samples: int


@dataclass
class OutputConfig:
    waypoint_order: str
    coord_decimals: int


@dataclass
class EstimationConfig:
    nominal_speed_mps: float


@dataclass
class EditorConfig:
    figure_size_inches: Tuple[float, float]
    background_color: str
    show_raw_trail: bool
    raw_trail_alpha: float
    raw_trail_color: str
    waypoint_color: str
    waypoint_marker_size: int
    path_line_color: str
    path_line_width: float
    selected_color: str
    selected_marker_size: int
    waypoint_pick_radius_m: float
    snap_tolerance_m: float
    undo_depth: int
    autosave_on_quit: bool
    status_refresh_hz: float


@dataclass
class RouteAuthoringConfig:
    io: IOConfig
    bag: BagConfig
    downsample: DownsampleConfig
    output: OutputConfig
    estimation: EstimationConfig
    editor: EditorConfig
    source_path: Optional[str] = None
    raw: Dict[str, Any] = field(default_factory=dict)


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    out = dict(base)
    for key, value in override.items():
        if key in out and isinstance(out[key], dict) and isinstance(value, dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def load_config(path: Optional[str] = None) -> RouteAuthoringConfig:
    """Load a config from ``path`` if given, else just return defaults.

    Missing keys in the user file fall back to the built-in defaults. Unknown
    keys are kept in ``RouteAuthoringConfig.raw`` so future fields don't break
    older configs.
    """
    merged: Dict[str, Any] = {k: dict(v) for k, v in DEFAULTS.items()}
    source_path: Optional[str] = None

    if path:
        expanded = os.path.expanduser(path)
        if not os.path.exists(expanded):
            raise FileNotFoundError(f'Config file not found: {expanded}')
        with open(expanded, 'r', encoding='utf-8') as f:
            user = yaml.safe_load(f) or {}
        if not isinstance(user, dict):
            raise ValueError(f'Config root must be a mapping, got {type(user).__name__}')
        merged = _deep_merge(merged, user)
        source_path = expanded

    return RouteAuthoringConfig(
        io=IOConfig(**merged['io']),
        bag=BagConfig(**merged['bag']),
        downsample=DownsampleConfig(**merged['downsample']),
        output=OutputConfig(**merged['output']),
        estimation=EstimationConfig(**merged['estimation']),
        editor=_editor_from_dict(merged['editor']),
        source_path=source_path,
        raw=merged,
    )


def _editor_from_dict(d: Dict[str, Any]) -> EditorConfig:
    fig = d['figure_size_inches']
    if not (isinstance(fig, (list, tuple)) and len(fig) == 2):
        raise ValueError('editor.figure_size_inches must be a 2-element list')
    return EditorConfig(
        figure_size_inches=(float(fig[0]), float(fig[1])),
        background_color=str(d['background_color']),
        show_raw_trail=bool(d['show_raw_trail']),
        raw_trail_alpha=float(d['raw_trail_alpha']),
        raw_trail_color=str(d['raw_trail_color']),
        waypoint_color=str(d['waypoint_color']),
        waypoint_marker_size=int(d['waypoint_marker_size']),
        path_line_color=str(d['path_line_color']),
        path_line_width=float(d['path_line_width']),
        selected_color=str(d['selected_color']),
        selected_marker_size=int(d['selected_marker_size']),
        waypoint_pick_radius_m=float(d['waypoint_pick_radius_m']),
        snap_tolerance_m=float(d['snap_tolerance_m']),
        undo_depth=int(d['undo_depth']),
        autosave_on_quit=bool(d['autosave_on_quit']),
        status_refresh_hz=float(d['status_refresh_hz']),
    )


def default_config_path() -> str:
    """Return the path to the installed default config, if discoverable.

    Looks first in the ament share dir (production), falls back to the source
    tree relative to this file (development from a colcon overlay).
    """
    try:
        from ament_index_python.packages import get_package_share_directory  # type: ignore
        share = get_package_share_directory('route_authoring_tool')
        candidate = os.path.join(share, 'config', 'route_authoring.yaml')
        if os.path.exists(candidate):
            return candidate
    except Exception:  # noqa: BLE001 - ament not on path, fall through
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, '..', 'config', 'route_authoring.yaml'))


__all__ = [
    'RouteAuthoringConfig',
    'IOConfig',
    'BagConfig',
    'DownsampleConfig',
    'OutputConfig',
    'EstimationConfig',
    'EditorConfig',
    'DEFAULTS',
    'load_config',
    'default_config_path',
]
