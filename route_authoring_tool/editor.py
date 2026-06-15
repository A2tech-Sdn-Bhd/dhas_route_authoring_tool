"""Interactive matplotlib editor for the waypoint polyline.

Coordinate convention: everything in here is in local meters (``LocalFrame``),
so distances and snap tolerances behave intuitively. On save, points are
converted back to lat/lon via the same frame so the output stays in WGS84.

Controls:

    Left click on path  -> insert waypoint at projected foot
    Left click + drag   -> move the picked waypoint
    Right click on pt   -> delete that waypoint
    s                   -> save to output path
    u                   -> undo (one step)
    r                   -> reload from disk (discards unsaved edits)
    h                   -> toggle raw trail visibility
    q                   -> quit (auto-saves if autosave_on_quit is true)
"""

from __future__ import annotations

import os
from typing import List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure

from .config_loader import EditorConfig, OutputConfig
from .geo import LocalFrame, polyline_length_m_xy
from .projection import nearest_waypoint, project_to_polyline, insert_point_at_hit
from .waypoint_io import save_waypoints


class RouteEditor:
    """Self-contained matplotlib editor. Call ``run()`` to block on the window."""

    def __init__(
        self,
        *,
        waypoints_latlon: Sequence[Tuple[float, float]],
        frame: LocalFrame,
        output_path: str,
        editor_cfg: EditorConfig,
        output_cfg: OutputConfig,
        raw_trail_latlon: Optional[Sequence[Tuple[float, float]]] = None,
        input_path_for_reload: Optional[str] = None,
    ) -> None:
        if not waypoints_latlon:
            raise ValueError('RouteEditor needs at least one waypoint to start.')
        self._frame = frame
        self._output_path = output_path
        self._editor = editor_cfg
        self._output = output_cfg
        self._reload_path = input_path_for_reload

        self._waypoints_xy: List[Tuple[float, float]] = list(frame.batch_to_xy(waypoints_latlon))
        self._raw_trail_xy: List[Tuple[float, float]] = (
            list(frame.batch_to_xy(raw_trail_latlon)) if raw_trail_latlon else []
        )
        self._undo_stack: List[List[Tuple[float, float]]] = []
        self._selected_idx: Optional[int] = None
        self._dragging: bool = False
        self._dirty: bool = False

        self._fig: Figure = plt.figure(figsize=editor_cfg.figure_size_inches)
        self._fig.patch.set_facecolor(editor_cfg.background_color)
        self._ax: Axes = self._fig.add_subplot(111)
        self._ax.set_facecolor(editor_cfg.background_color)
        self._ax.set_aspect('equal', adjustable='datalim')
        self._ax.grid(True, linestyle=':', linewidth=0.5, color='#cccccc')
        self._ax.set_xlabel('East (m, relative to anchor)')
        self._ax.set_ylabel('North (m, relative to anchor)')

        self._raw_line, = self._ax.plot(
            [], [],
            linestyle='-',
            linewidth=1.0,
            color=editor_cfg.raw_trail_color,
            alpha=editor_cfg.raw_trail_alpha if editor_cfg.show_raw_trail else 0.0,
            zorder=1,
            label='raw GPS trail',
        )
        self._path_line, = self._ax.plot(
            [], [],
            linestyle='-',
            linewidth=editor_cfg.path_line_width,
            color=editor_cfg.path_line_color,
            zorder=2,
            label='waypoint polyline',
        )
        self._waypoint_scatter, = self._ax.plot(
            [], [],
            linestyle='',
            marker='o',
            markersize=editor_cfg.waypoint_marker_size,
            markerfacecolor=editor_cfg.waypoint_color,
            markeredgecolor='black',
            markeredgewidth=0.6,
            zorder=3,
            label='waypoints',
        )
        self._selected_scatter, = self._ax.plot(
            [], [],
            linestyle='',
            marker='o',
            markersize=editor_cfg.selected_marker_size,
            markerfacecolor=editor_cfg.selected_color,
            markeredgecolor='black',
            markeredgewidth=1.0,
            zorder=4,
            label='selected',
        )
        # Faint marker showing where the next click would project to.
        self._snap_marker, = self._ax.plot(
            [], [],
            linestyle='',
            marker='x',
            markersize=8,
            markeredgewidth=1.5,
            color=editor_cfg.selected_color,
            alpha=0.0,
            zorder=5,
        )

        self._fig.canvas.mpl_connect('button_press_event', self._on_button_press)
        self._fig.canvas.mpl_connect('button_release_event', self._on_button_release)
        self._fig.canvas.mpl_connect('motion_notify_event', self._on_motion)
        self._fig.canvas.mpl_connect('key_press_event', self._on_key_press)
        self._fig.canvas.mpl_connect('close_event', self._on_close)

        self._ax.legend(loc='upper right', fontsize=8, framealpha=0.85)
        self._draw_help_footer()
        self._redraw(initial=True)

    # ------------------------------------------------------------------ run loop
    def run(self) -> None:
        plt.show()

    # ------------------------------------------------------------------ helpers
    def _draw_help_footer(self) -> None:
        text = (
            'left-click path: insert (snapped)   |   left-drag pt: move   |   '
            'right-click pt: delete   |   s: save   u: undo   r: reload   '
            'h: toggle raw   q: quit'
        )
        self._fig.text(
            0.5, 0.015, text,
            ha='center', va='bottom', fontsize=8, color='#444444',
        )

    def _push_undo(self) -> None:
        self._undo_stack.append(list(self._waypoints_xy))
        if len(self._undo_stack) > self._editor.undo_depth:
            self._undo_stack.pop(0)

    def _undo(self) -> None:
        if not self._undo_stack:
            return
        self._waypoints_xy = self._undo_stack.pop()
        self._selected_idx = None
        self._dirty = True
        self._redraw()

    def _redraw(self, initial: bool = False) -> None:
        if self._raw_trail_xy:
            xs, ys = zip(*self._raw_trail_xy)
            self._raw_line.set_data(xs, ys)
        else:
            self._raw_line.set_data([], [])

        if self._waypoints_xy:
            xs, ys = zip(*self._waypoints_xy)
            self._path_line.set_data(xs, ys)
            self._waypoint_scatter.set_data(xs, ys)
        else:
            self._path_line.set_data([], [])
            self._waypoint_scatter.set_data([], [])

        if self._selected_idx is not None and 0 <= self._selected_idx < len(self._waypoints_xy):
            sx, sy = self._waypoints_xy[self._selected_idx]
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
        n = len(self._waypoints_xy)
        length_m = polyline_length_m_xy(self._waypoints_xy)
        dirty_mark = ' *' if self._dirty else ''
        out_name = os.path.basename(self._output_path) if self._output_path else '(no output set)'
        self._ax.set_title(
            f'{out_name}{dirty_mark}   |   {n} waypoints   |   total length {length_m:.1f} m',
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

        px, py = float(event.xdata), float(event.ydata)
        nearest_idx, nearest_dist = nearest_waypoint(px, py, self._waypoints_xy)

        if event.button == 3:  # right click -> delete a waypoint
            if (nearest_idx >= 0
                    and nearest_dist <= self._editor.waypoint_pick_radius_m
                    and len(self._waypoints_xy) > 2):
                self._push_undo()
                del self._waypoints_xy[nearest_idx]
                self._selected_idx = None
                self._dirty = True
                self._redraw()
            return

        if event.button != 1:
            return

        # Left click on a waypoint -> select / start drag.
        if nearest_idx >= 0 and nearest_dist <= self._editor.waypoint_pick_radius_m:
            self._selected_idx = nearest_idx
            self._dragging = True
            self._push_undo()  # snapshot now so a single drag is one undo step
            self._redraw()
            return

        # Otherwise: try to project onto the path.
        hit = project_to_polyline(px, py, self._waypoints_xy)
        if hit is None:
            return
        if hit.distance_m <= self._editor.snap_tolerance_m:
            self._push_undo()
            self._waypoints_xy = insert_point_at_hit(self._waypoints_xy, hit)
            self._selected_idx = hit.segment_index + 1
            self._dirty = True
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
        px, py = float(event.xdata), float(event.ydata)

        if self._dragging and self._selected_idx is not None:
            self._waypoints_xy[self._selected_idx] = (px, py)
            self._dirty = True
            self._redraw()
            return

        # Hover: light snap marker if cursor is near the path.
        hit = project_to_polyline(px, py, self._waypoints_xy)
        if hit and hit.distance_m <= self._editor.snap_tolerance_m:
            self._snap_marker.set_data([hit.x], [hit.y])
            self._snap_marker.set_alpha(0.8)
        else:
            self._snap_marker.set_alpha(0.0)
        self._fig.canvas.draw_idle()

    def _on_key_press(self, event) -> None:
        key = (event.key or '').lower()
        if key == 's':
            self._save()
        elif key == 'u':
            self._undo()
        elif key == 'r':
            self._reload_from_disk()
        elif key == 'h':
            self._toggle_raw_trail()
        elif key == 'q':
            plt.close(self._fig)
        elif key in ('delete', 'backspace') and self._selected_idx is not None:
            if len(self._waypoints_xy) > 2:
                self._push_undo()
                del self._waypoints_xy[self._selected_idx]
                self._selected_idx = None
                self._dirty = True
                self._redraw()

    def _on_close(self, _event) -> None:
        if self._dirty and self._editor.autosave_on_quit:
            self._save()

    # ------------------------------------------------------------------ I/O
    def _save(self) -> None:
        if not self._output_path:
            print('[route_editor] no output path configured; press cancelled.')
            return
        latlons = self._frame.batch_to_latlon(self._waypoints_xy)
        path = save_waypoints(
            latlons,
            self._output_path,
            waypoint_order=self._output.waypoint_order,
            coord_decimals=self._output.coord_decimals,
            header_comment='Generated by route_authoring_tool. Re-open to edit.',
        )
        self._dirty = False
        self._update_title()
        self._fig.canvas.draw_idle()
        print(f'[route_editor] saved {len(latlons)} waypoints to {path}')

    def _reload_from_disk(self) -> None:
        if not self._reload_path or not os.path.exists(self._reload_path):
            print('[route_editor] reload skipped: no input path on disk.')
            return
        from .waypoint_io import load_waypoints
        wf = load_waypoints(self._reload_path)
        self._waypoints_xy = list(self._frame.batch_to_xy(wf.waypoints_latlon))
        self._undo_stack.clear()
        self._selected_idx = None
        self._dirty = False
        self._redraw()
        print(f'[route_editor] reloaded {len(wf.waypoints_latlon)} waypoints from {self._reload_path}')

    def _toggle_raw_trail(self) -> None:
        if not self._raw_trail_xy:
            return
        current = self._raw_line.get_alpha() or 0.0
        new_alpha = 0.0 if current > 0.0 else self._editor.raw_trail_alpha
        self._raw_line.set_alpha(new_alpha)
        self._fig.canvas.draw_idle()


__all__ = ['RouteEditor']
