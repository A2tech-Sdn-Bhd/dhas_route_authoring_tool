"""Interactive matplotlib editor for one or more waypoint polylines.

Coordinate convention: everything in here is in local meters (``LocalFrame``),
so distances and snap tolerances behave intuitively. The frame is shared
across every loaded route so they live in the same x/y plane. On save,
points are converted back to lat/lon via the same frame so the output
stays in WGS84.

When more than one route is loaded, ONE is "active" at a time. The
active route is drawn at full opacity and accepts mouse edits; the
others are drawn dimmed and are read-only until you cycle to them.

Controls:

    Left click on path  -> insert waypoint at projected foot (active route)
    Left click + drag   -> move the picked waypoint (active route)
    Right click on pt   -> delete that waypoint (active route)
    t / T (shift+t)     -> cycle the active route (forward / backward)
    s                   -> save the active route to its output path
    S (shift+s)         -> save all dirty routes
    u                   -> undo (active route, one step)
    r                   -> reload active route from disk (discards unsaved edits)
    h                   -> toggle raw trail visibility
    q                   -> quit (saves any dirty routes before closing)
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.lines import Line2D

from .config_loader import EditorConfig, OutputConfig
from .geo import LocalFrame, polyline_length_m_xy
from .projection import nearest_waypoint, project_to_polyline, insert_point_at_hit
from .waypoint_io import save_waypoints


# Stable per-route palette. Active route uses its assigned color at full
# alpha; inactive routes use the same color dimmed, so the visual identity
# of a route doesn't change when you cycle the active one.
_ROUTE_COLOR_PALETTE: Tuple[str, ...] = (
    '#225a99',  # blue   (matches editor.path_line_color default)
    '#cc6611',  # orange
    '#2ca02c',  # green
    '#d62728',  # red
    '#9467bd',  # purple
    '#8c564b',  # brown
)

_INACTIVE_ALPHA = 0.30
_INACTIVE_MARKER_SCALE = 0.65


@dataclass
class LayerSpec:
    """One route to load into the editor.

    ``waypoints_latlon`` is what the editor will project into the shared
    local frame and let the user edit. ``output_path`` is where 's' saves
    this route. ``reload_path`` is what 'r' reloads from when active
    (typically equal to the input file).
    """
    waypoints_latlon: Sequence[Tuple[float, float]]
    output_path: str
    reload_path: Optional[str] = None
    raw_trail_latlon: Optional[Sequence[Tuple[float, float]]] = None
    label: str = ''


@dataclass
class _Layer:
    """Internal per-route mutable state plus its matplotlib artists."""
    spec: LayerSpec
    color: str
    waypoints_xy: List[Tuple[float, float]]
    raw_trail_xy: List[Tuple[float, float]] = field(default_factory=list)
    undo_stack: List[List[Tuple[float, float]]] = field(default_factory=list)
    selected_idx: Optional[int] = None
    dirty: bool = False
    # Artists — bound in RouteEditor._build_artists.
    raw_line: Optional[Line2D] = None
    path_line: Optional[Line2D] = None
    waypoint_scatter: Optional[Line2D] = None


class RouteEditor:
    """Self-contained matplotlib editor. Call ``run()`` to block on the window."""

    def __init__(
        self,
        *,
        layers: Sequence[LayerSpec],
        frame: LocalFrame,
        editor_cfg: EditorConfig,
        output_cfg: OutputConfig,
        nominal_speed_mps: float = 0.5,
    ) -> None:
        if not layers:
            raise ValueError('RouteEditor needs at least one layer to start.')
        for i, spec in enumerate(layers):
            if not spec.waypoints_latlon:
                raise ValueError(f'Layer {i} has no waypoints.')

        self._frame = frame
        self._editor = editor_cfg
        self._output = output_cfg
        self._nominal_speed_mps = float(nominal_speed_mps)

        self._layers: List[_Layer] = []
        for i, spec in enumerate(layers):
            wp_xy = list(frame.batch_to_xy(spec.waypoints_latlon))
            raw_xy = (
                list(frame.batch_to_xy(spec.raw_trail_latlon))
                if spec.raw_trail_latlon else []
            )
            color = _ROUTE_COLOR_PALETTE[i % len(_ROUTE_COLOR_PALETTE)]
            self._layers.append(_Layer(
                spec=spec,
                color=color,
                waypoints_xy=wp_xy,
                raw_trail_xy=raw_xy,
            ))

        self._active_idx: int = 0
        self._dragging: bool = False

        self._fig: Figure = plt.figure(figsize=editor_cfg.figure_size_inches)
        self._fig.patch.set_facecolor(editor_cfg.background_color)
        self._ax: Axes = self._fig.add_subplot(111)
        self._ax.set_facecolor(editor_cfg.background_color)
        self._ax.set_aspect('equal', adjustable='datalim')
        self._ax.grid(True, linestyle=':', linewidth=0.5, color='#cccccc')
        self._ax.set_xlabel('East (m, relative to anchor)')
        self._ax.set_ylabel('North (m, relative to anchor)')

        self._build_artists()

        # Single shared "selected waypoint" highlight + click snap marker —
        # both belong to the active route at any moment.
        self._selected_scatter, = self._ax.plot(
            [], [],
            linestyle='',
            marker='o',
            markersize=editor_cfg.selected_marker_size,
            markerfacecolor=editor_cfg.selected_color,
            markeredgecolor='black',
            markeredgewidth=1.0,
            zorder=10,
            label='selected',
        )
        self._snap_marker, = self._ax.plot(
            [], [],
            linestyle='',
            marker='x',
            markersize=8,
            markeredgewidth=1.5,
            color=editor_cfg.selected_color,
            alpha=0.0,
            zorder=11,
        )

        self._fig.canvas.mpl_connect('button_press_event', self._on_button_press)
        self._fig.canvas.mpl_connect('button_release_event', self._on_button_release)
        self._fig.canvas.mpl_connect('motion_notify_event', self._on_motion)
        self._fig.canvas.mpl_connect('key_press_event', self._on_key_press)
        self._fig.canvas.mpl_connect('close_event', self._on_close)

        self._ax.legend(loc='upper right', fontsize=8, framealpha=0.85)
        self._draw_help_footer()
        self._apply_layer_styles()
        self._redraw(initial=True)

    # ------------------------------------------------------------------ run loop
    def run(self) -> None:
        plt.show()

    # ------------------------------------------------------------------ artists
    def _build_artists(self) -> None:
        """Create the per-layer line/scatter artists once at startup."""
        for i, layer in enumerate(self._layers):
            display_name = self._display_name(layer)
            layer.raw_line, = self._ax.plot(
                [], [],
                linestyle='-',
                linewidth=1.0,
                color=self._editor.raw_trail_color,
                alpha=self._editor.raw_trail_alpha if self._editor.show_raw_trail else 0.0,
                zorder=1,
                label=f'raw trail [{i + 1}] {display_name}' if layer.raw_trail_xy else None,
            )
            layer.path_line, = self._ax.plot(
                [], [],
                linestyle='-',
                linewidth=self._editor.path_line_width,
                color=layer.color,
                zorder=2,
                label=f'[{i + 1}] {display_name}',
            )
            layer.waypoint_scatter, = self._ax.plot(
                [], [],
                linestyle='',
                marker='o',
                markersize=self._editor.waypoint_marker_size,
                markerfacecolor=layer.color,
                markeredgecolor='black',
                markeredgewidth=0.6,
                zorder=3,
            )

    def _display_name(self, layer: _Layer) -> str:
        if layer.spec.label:
            return layer.spec.label
        if layer.spec.output_path:
            return os.path.basename(layer.spec.output_path)
        if layer.spec.reload_path:
            return os.path.basename(layer.spec.reload_path)
        return '(unnamed)'

    def _apply_layer_styles(self) -> None:
        """Set alpha / marker-size / zorder per layer based on which is active."""
        active_marker = self._editor.waypoint_marker_size
        inactive_marker = max(2.0, active_marker * _INACTIVE_MARKER_SCALE)
        active_width = self._editor.path_line_width
        inactive_width = max(0.6, active_width * 0.7)
        for i, layer in enumerate(self._layers):
            is_active = (i == self._active_idx)
            alpha = 1.0 if is_active else _INACTIVE_ALPHA
            layer.path_line.set_alpha(alpha)
            layer.path_line.set_linewidth(active_width if is_active else inactive_width)
            layer.path_line.set_zorder(4 if is_active else 2)
            layer.waypoint_scatter.set_alpha(alpha)
            layer.waypoint_scatter.set_markersize(active_marker if is_active else inactive_marker)
            layer.waypoint_scatter.set_zorder(5 if is_active else 3)

    # ------------------------------------------------------------------ helpers
    def _draw_help_footer(self) -> None:
        multi_hint = '   t: switch route' if len(self._layers) > 1 else ''
        text = (
            'left-click path: insert (snapped)   |   left-drag pt: move   |   '
            'right-click pt: delete   |   s: save active   S: save all   '
            'u: undo   r: reload   h: toggle raw   q: quit' + multi_hint
        )
        self._fig.text(
            0.5, 0.015, text,
            ha='center', va='bottom', fontsize=8, color='#444444',
        )

    def _active(self) -> _Layer:
        return self._layers[self._active_idx]

    def _push_undo(self, layer: _Layer) -> None:
        layer.undo_stack.append(list(layer.waypoints_xy))
        if len(layer.undo_stack) > self._editor.undo_depth:
            layer.undo_stack.pop(0)

    def _undo_active(self) -> None:
        layer = self._active()
        if not layer.undo_stack:
            return
        layer.waypoints_xy = layer.undo_stack.pop()
        layer.selected_idx = None
        layer.dirty = True
        self._redraw()

    def _redraw(self, initial: bool = False) -> None:
        for layer in self._layers:
            if layer.raw_trail_xy:
                xs, ys = zip(*layer.raw_trail_xy)
                layer.raw_line.set_data(xs, ys)
            else:
                layer.raw_line.set_data([], [])

            if layer.waypoints_xy:
                xs, ys = zip(*layer.waypoints_xy)
                layer.path_line.set_data(xs, ys)
                layer.waypoint_scatter.set_data(xs, ys)
            else:
                layer.path_line.set_data([], [])
                layer.waypoint_scatter.set_data([], [])

        active = self._active()
        if (active.selected_idx is not None
                and 0 <= active.selected_idx < len(active.waypoints_xy)):
            sx, sy = active.waypoints_xy[active.selected_idx]
            self._selected_scatter.set_data([sx], [sy])
        else:
            self._selected_scatter.set_data([], [])

        self._update_title()

        if initial:
            self._ax.relim()
            self._ax.autoscale_view()
            self._fig.tight_layout(rect=(0, 0.04, 1, 1))
        self._fig.canvas.draw_idle()

    def _update_title(self) -> None:
        active = self._active()
        n = len(active.waypoints_xy)
        length_m = polyline_length_m_xy(active.waypoints_xy)
        speed = max(1e-3, self._nominal_speed_mps)
        duration_min = (length_m / speed) / 60.0
        dirty_mark = ' *' if active.dirty else ''
        out_name = (
            os.path.basename(active.spec.output_path)
            if active.spec.output_path else '(no output set)'
        )
        position = (
            f'[{self._active_idx + 1}/{len(self._layers)}] '
            if len(self._layers) > 1 else ''
        )
        any_other_dirty = any(
            i != self._active_idx and lyr.dirty
            for i, lyr in enumerate(self._layers)
        )
        global_dirty = '  +others dirty' if any_other_dirty else ''
        self._ax.set_title(
            f'{position}{out_name}{dirty_mark}   |   {n} waypoints   |   '
            f'length {length_m:.1f} m   |   '
            f'~{duration_min:.2f} min @ {speed:.2f} m/s{global_dirty}',
            fontsize=11,
        )

    # ------------------------------------------------------------------ event handlers
    def _on_button_press(self, event) -> None:
        if event.inaxes is not self._ax or event.xdata is None or event.ydata is None:
            return
        # Skip clicks during pan/zoom (toolbar grabs them).
        toolbar = self._fig.canvas.toolbar
        if toolbar is not None and getattr(toolbar, 'mode', '') != '':
            return

        layer = self._active()
        px, py = float(event.xdata), float(event.ydata)
        nearest_idx, nearest_dist = nearest_waypoint(px, py, layer.waypoints_xy)

        if event.button == 3:  # right click -> delete a waypoint
            if (nearest_idx >= 0
                    and nearest_dist <= self._editor.waypoint_pick_radius_m
                    and len(layer.waypoints_xy) > 2):
                self._push_undo(layer)
                del layer.waypoints_xy[nearest_idx]
                layer.selected_idx = None
                layer.dirty = True
                self._redraw()
            return

        if event.button != 1:
            return

        # Left click on a waypoint -> select / start drag.
        if nearest_idx >= 0 and nearest_dist <= self._editor.waypoint_pick_radius_m:
            layer.selected_idx = nearest_idx
            self._dragging = True
            self._push_undo(layer)  # snapshot now so a single drag is one undo step
            self._redraw()
            return

        # Otherwise: try to project onto the active route's path.
        hit = project_to_polyline(px, py, layer.waypoints_xy)
        if hit is None:
            return
        if hit.distance_m <= self._editor.snap_tolerance_m:
            self._push_undo(layer)
            layer.waypoints_xy = insert_point_at_hit(layer.waypoints_xy, hit)
            layer.selected_idx = hit.segment_index + 1
            layer.dirty = True
            self._redraw()

    def _on_button_release(self, event) -> None:
        if not self._dragging:
            return
        self._dragging = False
        # The drag may have updated the point's xy already in _on_motion.
        # Nothing else to commit here — the undo snapshot was pushed at press.

    def _on_motion(self, event) -> None:
        if event.inaxes is not self._ax or event.xdata is None or event.ydata is None:
            return
        layer = self._active()
        px, py = float(event.xdata), float(event.ydata)

        if self._dragging and layer.selected_idx is not None:
            layer.waypoints_xy[layer.selected_idx] = (px, py)
            layer.dirty = True
            self._redraw()
            return

        # Hover: light snap marker if cursor is near the active path.
        hit = project_to_polyline(px, py, layer.waypoints_xy)
        if hit and hit.distance_m <= self._editor.snap_tolerance_m:
            self._snap_marker.set_data([hit.x], [hit.y])
            self._snap_marker.set_alpha(0.8)
        else:
            self._snap_marker.set_alpha(0.0)
        self._fig.canvas.draw_idle()

    def _on_key_press(self, event) -> None:
        key = event.key or ''
        lower = key.lower()
        # Shift+S -> save all dirty. Backends vary on how this arrives:
        # Qt/Tk/GTK deliver 'S'; some report 'shift+s' verbatim.
        if key == 'S' or lower == 'shift+s':
            self._save_all_dirty()
            return
        if lower == 's':
            self._save_active()
        elif lower == 'u':
            self._undo_active()
        elif lower == 'r':
            self._reload_active_from_disk()
        elif lower == 'h':
            self._toggle_raw_trail()
        elif lower == 'q':
            plt.close(self._fig)
        elif lower == 't':
            # 't' cycles forward; Shift+T cycles backward.
            step = -1 if (key == 'T' or lower == 'shift+t') else +1
            self._cycle_active(step)
        elif lower in ('delete', 'backspace'):
            layer = self._active()
            if layer.selected_idx is not None and len(layer.waypoints_xy) > 2:
                self._push_undo(layer)
                del layer.waypoints_xy[layer.selected_idx]
                layer.selected_idx = None
                layer.dirty = True
                self._redraw()

    def _on_close(self, _event) -> None:
        # Always save dirty routes on close. The original autosave_on_quit
        # config knob silently dropped edits, which led to "I pressed q and
        # nothing happened" — losing work is the worst possible failure mode
        # for an editor. Respect the knob only as a hint that we should
        # avoid touching disk when nothing is dirty (cheap default).
        dirty = [lyr for lyr in self._layers if lyr.dirty]
        if not dirty:
            print('[route_editor] closed; no unsaved edits.')
            return
        saved_paths = []
        for layer in dirty:
            if not layer.spec.output_path:
                print(
                    f'[route_editor] WARNING: {self._display_name(layer)} had unsaved '
                    'edits but no output path was set; changes were dropped.'
                )
                continue
            self._save_layer(layer)
            saved_paths.append(layer.spec.output_path)
        if saved_paths:
            print(
                f'[route_editor] closed; saved {len(saved_paths)} dirty '
                f'route(s) on quit.'
            )

    # ------------------------------------------------------------------ active-route control
    def _cycle_active(self, step: int) -> None:
        if len(self._layers) <= 1:
            return
        n = len(self._layers)
        self._active_idx = (self._active_idx + step) % n
        # Cancel any in-progress drag — it belonged to the previous active route.
        self._dragging = False
        self._snap_marker.set_alpha(0.0)
        self._apply_layer_styles()
        self._redraw()
        layer = self._active()
        print(
            f'[route_editor] active route: [{self._active_idx + 1}/{n}] '
            f'{self._display_name(layer)}'
        )

    # ------------------------------------------------------------------ I/O
    def _save_active(self) -> None:
        self._save_layer(self._active())

    def _save_all_dirty(self) -> None:
        saved = 0
        for layer in self._layers:
            if layer.dirty:
                self._save_layer(layer)
                saved += 1
        if saved == 0:
            print('[route_editor] save-all: nothing dirty.')

    def _save_layer(self, layer: _Layer) -> None:
        if not layer.spec.output_path:
            print(
                f'[route_editor] no output path configured for '
                f'{self._display_name(layer)}; save skipped.'
            )
            return
        latlons = self._frame.batch_to_latlon(layer.waypoints_xy)
        path = save_waypoints(
            latlons,
            layer.spec.output_path,
            waypoint_order=self._output.waypoint_order,
            coord_decimals=self._output.coord_decimals,
            nominal_speed_mps=self._nominal_speed_mps,
            header_comment='Generated by route_authoring_tool. Re-open to edit.',
        )
        layer.dirty = False
        self._update_title()
        self._fig.canvas.draw_idle()
        print(f'[route_editor] saved {len(latlons)} waypoints to {path}')

    def _reload_active_from_disk(self) -> None:
        layer = self._active()
        if not layer.spec.reload_path or not os.path.exists(layer.spec.reload_path):
            print('[route_editor] reload skipped: no input path on disk.')
            return
        from .waypoint_io import load_waypoints
        wf = load_waypoints(layer.spec.reload_path)
        layer.waypoints_xy = list(self._frame.batch_to_xy(wf.waypoints_latlon))
        layer.undo_stack.clear()
        layer.selected_idx = None
        layer.dirty = False
        self._redraw()
        print(
            f'[route_editor] reloaded {len(wf.waypoints_latlon)} waypoints '
            f'from {layer.spec.reload_path}'
        )

    def _toggle_raw_trail(self) -> None:
        # Toggle visibility of every layer's raw trail in lockstep.
        any_visible = any(
            (layer.raw_line.get_alpha() or 0.0) > 0.0
            for layer in self._layers
            if layer.raw_trail_xy
        )
        new_alpha = 0.0 if any_visible else self._editor.raw_trail_alpha
        for layer in self._layers:
            if layer.raw_trail_xy:
                layer.raw_line.set_alpha(new_alpha)
        self._fig.canvas.draw_idle()


__all__ = ['RouteEditor', 'LayerSpec']
