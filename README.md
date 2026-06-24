# route_authoring_tool

Offline ROS 2 helper to turn a teleop-recorded bag into a waypoint YAML
consumable by [`hybrid_smooth_path_follower`](../hybrid_smooth_path_follower).

You drive the robot along the route you want it to learn while recording
`/rtk/fix` (and a couple of optional topics). This tool then:

1. Reads the bag, filters RTK fixes by status, and extracts the lat/lon trail.
2. Downsamples the dense trail into a sparse waypoint polyline:
   **Ramer–Douglas–Peucker** (epsilon in meters, keeps corners) followed by a
   **max-segment split pass** (guarantees no segment is longer than your cap).
3. Opens an interactive matplotlib editor so you can add, drag, and delete
   waypoints. Left-clicks are projected perpendicularly onto the path — you
   don’t need pixel-perfect aim.
4. Writes a YAML the path follower loads directly.

It is **not** a runtime ROS 2 node. There is no `rclpy.init`, no `/cmd_vel`,
no subscribers. It is a CLI plus a GUI window.

---

## Install

The package lives in this workspace’s `src/`. Standard colcon build:

```bash
cd ~/dhas_dev_ws
colcon build --packages-select route_authoring_tool
source install/setup.bash
```

Dependencies (`package.xml`): `rclpy` (only for `serialization.deserialize_message`),
`rosbag2_py`, `rosidl_runtime_py`, `sensor_msgs`, `std_msgs`, `nav_msgs`,
`python3-yaml`, `python3-numpy`, `python3-matplotlib`.

---

## Record a route

Drive the robot manually along the route. Record at least `/rtk/fix`. The
other topics are optional (kept for richer visualization in the editor, but
never written to the output YAML):

```bash
ros2 bag record \
  /rtk/fix \
  /rtk_heading/float \
  /imu/data \
  /Odometry \
  -o my_route_bag
```

Default storage is `sqlite3`. The tool also detects `mcap` automatically.

---

## Use it

There are two entry points, both run via `ros2 run`:

### One-shot: bag → YAML (no GUI)

```bash
ros2 run route_authoring_tool bag_to_route \
  --bag ./my_route_bag \
  --output ./my_route.yaml
```

Prints a one-liner with raw → downsampled counts and the output path.

### Interactive: open the editor

Starting from a bag (extract + open):

```bash
ros2 run route_authoring_tool route_editor \
  --bag ./my_route_bag \
  --output ./my_route.yaml
```

Re-editing an existing YAML:

```bash
ros2 run route_authoring_tool route_editor \
  --input ./my_route.yaml
```

If `--output` is omitted, the editor saves back over `--input` (or next to
the bag, with a sensible default name).

### Via a launch file

```bash
ros2 launch route_authoring_tool route_editor.launch.py \
  config:=/path/to/route_authoring.yaml
```

The launch wrapper only exists for the `config:=` override. For per-run
`--bag` / `--input` / `--output`, call the entry points directly.

---

## Editor controls

| Action | Behaviour |
|---|---|
| **left-click** anywhere near the path | inserts a new waypoint at the **projected** foot of perpendicular (snapped to the path, not at the cursor) |
| **left-click on a waypoint** + drag | moves the waypoint |
| **right-click on a waypoint** | deletes the waypoint |
| **`s`** | save to the output path |
| **`u`** | undo (one step) |
| **`r`** | reload from disk (discards unsaved edits) |
| **`h`** | toggle the raw GPS trail visibility |
| **`q`** | quit (auto-saves if `editor.autosave_on_quit: true`) |
| **`Delete` / `Backspace`** | delete the currently selected waypoint |

A faint `×` follows the cursor whenever it is within `snap_tolerance_m` of
the path, previewing exactly where a click would insert.

Two thresholds control click behaviour, both in **meters of the local frame**:

- `editor.waypoint_pick_radius_m` — a click inside this radius of an existing
  waypoint is treated as a pick (select / drag / delete), not an insert.
- `editor.snap_tolerance_m` — outside the pick radius, the click is projected
  onto the nearest segment; if the perpendicular distance is within this
  tolerance, a new waypoint is inserted at the projected point. Farther
  clicks are ignored.

The title bar shows the file name, waypoint count, total path length, and a
`*` dirty marker when unsaved edits exist.

---

## Configuration

All tunables live in [`config/route_authoring.yaml`](config/route_authoring.yaml).
Either edit it in place after build, or copy it next to your bag and pass
`--config path/to/your_copy.yaml`. Missing keys fall back to the built-in
defaults in `config_loader.DEFAULTS`.

The headline knobs you will actually touch:

| Key | Meaning |
|---|---|
| `io.bag_path`, `io.input_waypoints`, `io.output_path` | inputs/outputs (CLI flags override these) |
| `bag.rtk_fix_topic` (+ heading/imu/odom) | topic names if your robot publishes under different ones |
| `bag.nav_sat_status_min` | `0` keeps any fix, `2` keeps only RTK-grade |
| `downsample.rdp_epsilon_m` | smaller = keeps more vertices, larger = drops more straights |
| `downsample.max_segment_m` | cap on segment length after RDP (~5 m fits the consumer’s lookahead) |
| `output.waypoint_order` | `lat_lon` or `lon_lat` row ordering in the YAML |
| `editor.snap_tolerance_m`, `editor.waypoint_pick_radius_m` | the two thresholds above |
| `editor.show_raw_trail`, `editor.background_color`, `editor.*_color` | visuals |

---

## Output format

The YAML matches the schema accepted by `hybrid_smooth_path_follower`’s
`route_follower_node._load_waypoints`:

```yaml
coordinate_mode: latlon
waypoint_order: lat_lon
estimated_duration_min: 19.02      # auto-computed at save; hand-edit to override
waypoints:
  - [2.90279575, 101.28969915]
  - [2.90281099, 101.28969915]
  ...
```

`estimated_duration_min` is computed as
`polyline_length / estimation.nominal_speed_mps / 60` and surfaced via
`mission_server`'s `RouteInfo` for operator UIs. The follower itself
ignores the field; legacy routes without it still load (the consumer
recomputes from its own configured speed).

The file’s top-level `coordinate_mode` and `waypoint_order` keys
**override** the consumer’s parameter values at load time, so you can
drop this file into any existing `route_follower_params.yaml` setup
without changing the consumer’s `waypoint_coordinate_mode` /
`gps_waypoint_order` params.

Two ways to point the follower at your route:

```yaml
# A. Standalone route_follower: edit route_follower_params.yaml
waypoints_file: "/abs/path/to/my_route.yaml"
```

```bash
# B. Via mission_server (recommended for real missions): drop the file
#    into mission_server/config/routes/ and run a mission by name.
ros2 action send_goal /mission/run \
  mission_server_interfaces/action/RunMission \
  "{route_name: 'my_route'}" --feedback
```

---

## How the math works (one paragraph each)

**Coordinate frame.** Everything inside the tool (RDP epsilon, max-segment
cap, snap tolerance, pick radius, drag math) operates in **local meters**
via an equirectangular projection anchored on the first sample of the
trail. The anchor is fixed for the session, so spatial reasoning is stable
across edits. At parking-lot scale (< 1 km) the projection error is below
0.1 % — well under RTK noise.

**RDP simplification.** Iterative Ramer–Douglas–Peucker (no recursion limit
on long drives). Walks the trail with a stack; for each sub-range, finds the
point with the largest perpendicular distance to the chord between
endpoints; keeps it if that distance exceeds `rdp_epsilon_m`; recurses on
the two halves. Always preserves the first and last samples.

**Max-segment pass.** After RDP, any segment longer than `max_segment_m`
is split with evenly-spaced intermediates so the consumer’s lookahead
window (which expects ~2.5 m spacing) always has enough samples in
straight stretches.

**Click projection.** For each segment of the current polyline, computes
the foot of perpendicular from the click, clamps to the segment endpoints,
and picks the segment with the smallest perpendicular distance. Mirrors
`_project_to_polyline` in `goal_directed_heading.py` so the editor’s
geometric model matches what the path follower itself does at runtime.

---

## Troubleshooting

**“No usable fixes found on /rtk/fix in …”**
The bag opens but the fix topic has no messages above
`bag.nav_sat_status_min`. Either the topic name doesn’t match
(`ros2 bag info <bag>` to check), or all fixes were `NavSatStatus.STATUS_NO_FIX`
(−1). Lower `nav_sat_status_min` to `-1` to keep everything, or re-record
with antenna sky-view.

**Editor opens but the path looks tiny / huge.**
The plot is in local meters anchored on the first sample. Pan/zoom with
the matplotlib toolbar. The axes are equal-aspect so distances on screen
match meters on the ground.

**“Topic '/rtk/fix' not found in bag.”**
Your topic is named something else. Either pass `--config` with the
topics block updated, or temporarily edit `config/route_authoring.yaml`
under `bag.rtk_fix_topic`.

**Saved YAML doesn’t load in the path follower.**
Check that `route_follower_params.yaml` points at the absolute path, and
that the file you’re writing has `coordinate_mode: latlon` and a
`waypoints:` list — those three pieces are what the consumer keys on.

**Matplotlib backend errors on headless machines.**
The editor needs an interactive backend (it calls `plt.show()`). If you
are on a remote box, X11-forward or use the non-interactive
`bag_to_route` entry point instead.

---

## Where the code lives

```
route_authoring_tool/
├── package.xml                       ament_python manifest
├── setup.py / setup.cfg              entry points
├── config/route_authoring.yaml       every tunable
├── launch/route_editor.launch.py     convenience wrapper
└── route_authoring_tool/
    ├── cli.py                        bag_to_route_main, route_editor_main
    ├── config_loader.py              DEFAULTS + dataclasses
    ├── geo.py                        LocalFrame (lat/lon <-> meters)
    ├── projection.py                 click-to-polyline math
    ├── bag_reader.py                 rosbag2_py.SequentialReader -> BagTrail
    ├── downsample.py                 RDP + max-segment split
    ├── waypoint_io.py                read/write the consumer YAML schema
    └── editor.py                     matplotlib interactive editor
```

For implementation-level guidance (when changing the code), see
[`CLAUDE.md`](CLAUDE.md).

---

## License

Apache-2.0.
