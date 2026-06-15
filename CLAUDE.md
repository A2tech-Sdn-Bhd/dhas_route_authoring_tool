# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this package is

An **offline authoring tool** that turns a teleop-recorded ROS 2 bag into a waypoint YAML consumable by the sibling package `hybrid_smooth_path_follower`. Two entry points:

- `bag_to_route` — bag in, YAML out, no GUI.
- `route_editor` — matplotlib interactive editor that lets the user add, drag, and delete waypoints, with click-to-path projection.

It is **not** a runtime ROS 2 node: no `rclpy` node is started. It still ships as an `ament_python` package so it inherits `rosbag2_py` and the message types it needs to deserialize bag records.

## Build, run, lint

```bash
# From the workspace root.
cd ~/dhas_dev_ws
colcon build --packages-select route_authoring_tool
source install/setup.bash

# Bag -> YAML (no GUI). Override io.* in the config or on the command line.
ros2 run route_authoring_tool bag_to_route --bag /path/to/bag --output route.yaml

# Interactive editor on a fresh bag extract:
ros2 run route_authoring_tool route_editor --bag /path/to/bag --output route.yaml

# Interactive editor on an existing waypoints YAML:
ros2 run route_authoring_tool route_editor --input route.yaml

# Convenience launch (config:= override only).
ros2 launch route_authoring_tool route_editor.launch.py config:=/path/to/route_authoring.yaml

# Linters declared in package.xml.
ament_flake8 route_authoring_tool
ament_pep257 route_authoring_tool
ament_copyright route_authoring_tool
```

All tunables live in `config/route_authoring.yaml`. The loader deep-merges the user file on top of `config_loader.DEFAULTS`, so missing keys are fine — the defaults block in `config_loader.py` is the authoritative reference for every knob.

## Architecture — big picture

Everything works in one of two coordinate systems:

- **lat/lon (WGS84)** — what the bag carries, what the consumer YAML stores.
- **local meters via `geo.LocalFrame`** — equirectangular projection anchored on the FIRST sample of the trail. Anchored on first-sample (not centroid) so the frame stays stable across edits.

The pipeline:

1. `bag_reader.read_bag` opens the rosbag2 directory with `SequentialReader`, filters `NavSatFix` by `status.status >= nav_sat_status_min`, decimates by `pre_decimate_every_n`, returns a `BagTrail` of `(timestamp_ns, lat, lon)` tuples. Heading samples are kept for visualization only and never reach the YAML.
2. `downsample.downsample_trail_latlon` projects the trail to the local frame, runs **RDP with epsilon in METERS** (iterative, stack-based — no recursion limit on 10k-point trails), then a **max-segment-length pass** that splits any leftover segment longer than `max_segment_m` with evenly-spaced intermediates. Both stages preserve the first and last samples.
3. `editor.RouteEditor` opens a matplotlib window with the raw trail, the simplified polyline, and the waypoint markers. State is held in the **local-meters frame** so distances and snap tolerances behave intuitively.
4. `waypoint_io.save_waypoints` writes the YAML in the exact schema `visual_debug_node._load_waypoints` accepts.

### Click projection (the core editor behaviour)

When the user left-clicks anywhere near the path, the editor does **not** insert at the cursor. It runs `projection.project_to_polyline` (same math as `_project_to_polyline` in `goal_directed_heading.py` — for each segment, foot of perpendicular, clamp `t` to `[0,1]`, pick the nearest). If the foot is within `editor.snap_tolerance_m` of the click, the new waypoint is inserted at the projected point between `segment_index` and `segment_index + 1`.

`editor.waypoint_pick_radius_m` is checked first: a click inside that radius of an existing waypoint is treated as "pick that waypoint" (select / start drag / right-click delete) rather than insert.

### Output YAML schema (must match the consumer)

The consumer accepts both `[a, b]` list rows (respecting top-level `waypoint_order`) and `{lat, lon}` dict rows (which ignore order). We write list rows because the existing production routes use them. See `hybrid_smooth_path_follower/visual_debug_node.py:_load_waypoints` for the loader and `hybrid_smooth_path_follower/waypoints/example_*.yaml` for reference files.

```yaml
coordinate_mode: latlon
waypoint_order: lat_lon   # or lon_lat
waypoints:
  - [lat, lon]
```

## Important constraints

- **Output must round-trip through `visual_debug_node._load_waypoints` without parameter overrides.** Top-level `coordinate_mode` and `waypoint_order` in the file override the consumer's params at load time — wrong values silently change the runtime mode. The writer must keep emitting both keys.
- **No `rclpy.init`** in this package. `rclpy.serialization.deserialize_message` and `rosidl_runtime_py.utilities.get_message` work without it; spinning a node would be wrong for an offline tool.
- **`LocalFrame` anchor is the first sample of the source trail (or the first sample of the input YAML when re-editing).** Do not switch to centroid-anchoring without updating every distance threshold — RDP epsilon, snap tolerance, pick radius, max-segment cap are all expressed in meters of THIS frame.
- **The matplotlib editor blocks the main thread.** It is called via `plt.show()`; do not try to interleave it with other rclpy spin loops.
- **All tunables live in `config/route_authoring.yaml`.** When adding a knob: extend `DEFAULTS` in `config_loader.py`, add the dataclass field, and document the new key in the YAML (the YAML doubles as user-facing documentation).
- **Bag format defaults to sqlite3.** `bag_reader._infer_storage_id` peeks at the directory for `.db3` / `.mcap` extensions and falls through to sqlite3. If the user records in mcap, no config change is needed.

## Module layout

| File | Role |
|---|---|
| `cli.py` | Entry points `bag_to_route_main`, `route_editor_main`. Arg parsing, config + override resolution. |
| `config_loader.py` | `DEFAULTS` block (authoritative), `load_config`, dataclasses for each section. |
| `bag_reader.py` | `rosbag2_py.SequentialReader` -> `BagTrail`. Status filtering + pre-decimation. |
| `downsample.py` | `rdp_xy` (iterative), `enforce_max_segment_xy`, `downsample_trail_latlon` glue. |
| `geo.py` | `LocalFrame` (equirectangular), haversine, polyline length helpers. |
| `projection.py` | `project_to_polyline`, `nearest_waypoint`, `insert_point_at_hit`. Pure math. |
| `waypoint_io.py` | Read/write the consumer YAML schema (list rows, decimal formatting). |
| `editor.py` | `RouteEditor` (matplotlib). Event handlers, undo stack, redraw. Imports matplotlib at module load. |

## Testing this package

`test/` is empty. There is no pytest suite. Smoke-test changes by:

1. AST-parse every module: `for f in route_authoring_tool/*.py; do python3 -c "import ast; ast.parse(open('$f').read())"; done`
2. Import the non-ROS modules in a plain Python REPL (geo, projection, downsample, waypoint_io, config_loader).
3. Round-trip a synthetic lat/lon trail through `downsample_trail_latlon` -> `save_waypoints` -> `load_waypoints` and assert the count is preserved.
4. For matplotlib changes, run `MPLBACKEND=Agg python3 -c "from route_authoring_tool.editor import RouteEditor"` to confirm the module still imports under a headless backend.
