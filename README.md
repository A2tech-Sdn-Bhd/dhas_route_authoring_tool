# route_authoring_tool

## Table of Contents

- [Overview](#overview)
- [Installation & Build](#installation--build)
- [Nodes / Executables](#nodes--executables)
- [Launch Files](#launch-files)
- [Interfaces](#interfaces)
- [Configuration](#configuration)
- [Testing](#testing)
- [Known Issues](#known-issues)
- [Glossary](#glossary)

## Overview

`route_authoring_tool` is an offline ROS 2 utility that extracts a GPS trail from a teleoperation rosbag, simplifies it into a waypoint route, optionally edits one or more routes in a Matplotlib GUI, and writes latitude/longitude YAML for the workspace's `hybrid_smooth_path_follower`. It uses ROS libraries only to read and deserialize bag records: neither executable creates a ROS node or joins the live ROS graph. The processing pipeline is rosbag2 reading -> `NavSatFix` filtering and optional decimation -> a first-sample-anchored local metric frame -> iterative Ramer-Douglas-Peucker (RDP) simplification -> maximum-segment splitting -> YAML output (`route_authoring_tool/bag_reader.py:49`, `route_authoring_tool/downsample.py:110`, `route_authoring_tool/waypoint_io.py:134`).

| Property | Value |
|---|---|
| Package name/version | `route_authoring_tool` / `0.1.0` |
| Build type | `ament_python` (`package.xml:30`) |
| Build definition | `setup.py`; there is no `CMakeLists.txt` |
| ROS distro | ROS 2 Humble is assumed because every bundled bag database records `ros_distro=humble`. The manifest does not constrain a distro, so broader compatibility is **[UNCLEAR — verify]**. |
| Installed executables | `bag_to_route`, `route_editor` (`setup.py:25`) |
| Installed data | Ament resource-index marker, `package.xml`, `config/*.yaml`, and `launch/*.py` (`setup.py:10`) |
| Not installed | `resource/routes/`, `bags/`, `README.md`, and `CLAUDE.md` |
| License | Apache-2.0 |

### Dependencies

All manifest dependencies are external to this workspace; no declared dependency has a matching package manifest under this workspace's `src/` tree. `<exec_depend>` means runtime-only in this package; there are no declared `<depend>`, `<build_depend>`, or `<buildtool_depend>` entries.

| Name | Declared type | Internal? | Purpose |
|---|---|---:|---|
| `rclpy` | exec | No | `deserialize_message`; no node or executor is created |
| `rosbag2_py` | exec | No | `SequentialReader`, storage options/filter, SQLite3 or MCAP bag access |
| `rosidl_runtime_py` | exec | No | Resolves recorded ROS interface types by name |
| `sensor_msgs` | exec | No | Expected `sensor_msgs/msg/NavSatFix`; configured `sensor_msgs/msg/Imu` bags are present but IMU data is not read |
| `std_msgs` | exec | No | Expected optional `std_msgs/msg/Float32` heading samples |
| `nav_msgs` | exec | No | Config/bags name `nav_msgs/msg/Odometry`, but the implementation does not read it |
| `python3-yaml` | exec | No | Configuration and waypoint YAML parsing |
| `python3-numpy` | exec | No | Declared but not imported anywhere in this package |
| `python3-matplotlib` | exec | No | Interactive route editor |
| `ament_copyright` | test | No | Declared linter dependency; no registered linter test exists |
| `ament_flake8` | test | No | Declared linter dependency; no registered linter test exists |
| `ament_pep257` | test | No | Declared linter dependency; no registered linter test exists |

`setuptools` is also listed in `setup.py:17` as a Python install requirement. Imports of `ament_index_python` and `launch` are not represented in `package.xml`; see [Known Issues](#known-issues).

## Installation & Build

From the ROS 2 workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/navigation/dhas_route_authoring_tool --ignore-src -r -y
colcon build --packages-select route_authoring_tool
source install/setup.bash
```

No package-specific `colcon` flags are required by the repository. `setup.cfg` places both console scripts in `lib/route_authoring_tool`, which is the location expected by ROS 2 tooling.

There are no custom build targets, compiled libraries, C/C++ executables, generated interfaces, or `rosidl` generation steps. The build installs:

```text
share/ament_index/resource_index/packages/route_authoring_tool
share/route_authoring_tool/package.xml
share/route_authoring_tool/config/route_authoring.yaml
share/route_authoring_tool/launch/route_editor.launch.py
lib/route_authoring_tool/bag_to_route
lib/route_authoring_tool/route_editor
```

Runtime requirements not fully captured for `rosdep` are:

- `ament_index_python`, imported when locating the installed default config (`route_authoring_tool/config_loader.py:213`), and the ROS 2 `launch` Python package used by the launch file (`launch/route_editor.launch.py:12`). Neither is declared.
- An interactive Matplotlib backend and graphical display for `route_editor`; use `bag_to_route` on a headless system.
- The rosbag2 storage plugin matching the input. SQLite3 is the default. MCAP files are detected by extension, but a corresponding MCAP storage plugin must be installed (`route_authoring_tool/bag_reader.py:70`). The exact OS package name is distro-dependent **[UNCLEAR — verify]**.

Typical invocations are:

```bash
# Bag to YAML, without a GUI
ros2 run route_authoring_tool bag_to_route \
  --bag /path/to/bag_directory \
  --output /path/to/route.yaml

# Edit a newly extracted bag route
ros2 run route_authoring_tool route_editor \
  --bag /path/to/bag_directory \
  --output /path/to/route.yaml

# Edit one or several existing routes
ros2 run route_authoring_tool route_editor \
  --input route_a.yaml route_b.yaml \
  --output route_a_edited.yaml route_b_edited.yaml
```

## Nodes / Executables

This package contains **zero ROS nodes**. Consequently, the internal graph contains no ROS topic edges and no Mermaid node/topic diagram is applicable. Both executables are synchronous Python console programs; names below are executable names, not ROS node names.

### `bag_to_route`

| Property | Details |
|---|---|
| Entry point | `route_authoring_tool.cli:bag_to_route_main` (`setup.py:26`) |
| Principal source | `route_authoring_tool/cli.py:141`; bag access in `route_authoring_tool/bag_reader.py` |
| Node name / namespace | None; it does not call `rclpy.init()` or construct a node |
| Publishers / subscribers | None. Recorded topics are read directly from a rosbag2 `SequentialReader`, not subscribed to live. |
| Services / actions | None |
| ROS parameters | None; configuration comes from CLI arguments and YAML |
| Threading / executor | Single synchronous process; no ROS executor or callback groups |
| Lifecycle | Not a lifecycle node |
| Composable | No |

#### CLI arguments

| Argument | Default | Behavior |
|---|---|---|
| `--config`, `-c` | Installed `config/route_authoring.yaml` when discoverable; otherwise built-in defaults | Deep-merges the file over `config_loader.DEFAULTS` |
| `--bag` | `io.bag_path` | Required effective input; must be the directory containing `metadata.yaml` |
| `--input` | Unused | Accepted by the shared parser but has no effect in `bag_to_route` (`route_authoring_tool/cli.py:31`, `route_authoring_tool/cli.py:141`) |
| `--output`, `-o` | `io.output_path`, else `<bag-parent>/<bag-name>_waypoints.yaml` | Exactly zero or one output is accepted |

`bag_to_route` filters fixes whose `msg.status.status` is below `nav_sat_status_min`, keeps every Nth accepted fix, trims configured head/tail samples, simplifies and spaces the route, computes estimated duration, and overwrites the output YAML (`route_authoring_tool/cli.py:78`, `route_authoring_tool/cli.py:176`). Exit code `0` means success, `1` extraction failure, and `2` configuration/usage failure.

### `route_editor`

| Property | Details |
|---|---|
| Entry point | `route_authoring_tool.cli:route_editor_main` (`setup.py:27`) |
| Principal source | `route_authoring_tool/cli.py:197`, GUI in `route_authoring_tool/editor.py:92` |
| Node name / namespace | None; it does not call `rclpy.init()` or construct a node |
| Publishers / subscribers | None. Bag mode reads recorded data directly. |
| Services / actions | None |
| ROS parameters | None; configuration comes from CLI arguments and YAML |
| Threading / executor | Matplotlib event loop blocks the main thread via `plt.show()`; no ROS executor or callback groups (`route_authoring_tool/editor.py:179`) |
| Lifecycle | Not a lifecycle node |
| Composable | No |

#### CLI arguments and source selection

| Argument | Default | Behavior |
|---|---|---|
| `--config`, `-c` | Installed config, then built-in defaults | Same deep-merge behavior as `bag_to_route` |
| `--bag` | `io.bag_path` | Used only when no effective `--input`/`io.input_waypoints` exists |
| `--input` | `io.input_waypoints` | One or more waypoint YAMLs; CLI input takes precedence over bag input |
| `--output`, `-o` | For one input: `io.output_path` or `<stem>_edited.yaml`; for multiple: one paired path per input or per-input defaults; for bag: config path or `<bag-name>_waypoints.yaml` | Multiple explicit outputs must match the number of inputs |

All loaded routes share a local frame anchored on the first waypoint of the first route (`route_authoring_tool/cli.py:234`). Only one route is active/editable at a time; inactive routes are dimmed. Controls implemented in `route_authoring_tool/editor.py:330` are:

| Input | Effect |
|---|---|
| Left-click within pick radius | Select and begin dragging the nearest waypoint |
| Left-click near path | Project to the nearest segment and insert if within snap tolerance |
| Right-click / `Delete` / `Backspace` | Delete a selected/nearby waypoint, but never reduce the route below two points |
| `t` / `Shift+T` | Cycle active route forward/backward |
| `s` / `Shift+S` | Save active route / all dirty routes |
| `u` | Undo one active-route edit, bounded by `undo_depth` |
| `r` | Reload the active route when it has an input path |
| `h` | Toggle all available raw trails |
| `q` or window close | Close the editor; current code saves every dirty route with an output path |

## Launch Files

### `launch/route_editor.launch.py`

This convenience wrapper starts the offline `route_editor` process; it does not launch a ROS node (`launch/route_editor.launch.py:17`).

| Launch argument | Default | Description / use |
|---|---|---|
| `config` | Empty string | Path to `route_authoring.yaml`; an empty value causes the CLI to discover the installed default config |

It unconditionally executes `route_editor --config <config>` with output forwarded to the screen. It has no conditions, namespaces, remappings, event handlers, or additional processes. The selected YAML is loaded by the CLI, not by ROS parameter machinery. Bag/input/output overrides are not launch arguments; place them in the selected config or invoke `ros2 run` directly.

```bash
ros2 launch route_authoring_tool route_editor.launch.py \
  config:=/absolute/path/to/route_authoring.yaml
```

In a clean Humble install this launch file fails to resolve the bare `route_editor` command because the console script is installed under `lib/route_authoring_tool`, not on `PATH`; see [Known Issues](#known-issues).

## Interfaces

This package defines no `.msg`, `.srv`, or `.action` files and performs no interface generation. It consumes these external message interfaces from recorded bags:

| Interface | Defining package | Use |
|---|---|---|
| `sensor_msgs/msg/NavSatFix` | `sensor_msgs` | Required configured fix topic. Reads `status.status`, `latitude`, and `longitude` (`route_authoring_tool/bag_reader.py:143`). |
| `std_msgs/msg/Float32` | `std_msgs` | Expected optional heading topic. Reads `data` into `BagTrail.headings`, but no later code consumes those samples (`route_authoring_tool/bag_reader.py:151`). |
| `sensor_msgs/msg/Imu` | `sensor_msgs` | Named in config and present in bundled bags, but never requested or deserialized by `read_bag`. |
| `nav_msgs/msg/Odometry` | `nav_msgs` | Named in config and present in bundled bags, but never requested or deserialized by `read_bag`. |

The reader resolves the actual recorded type string dynamically with `rosidl_runtime_py.utilities.get_message`. Nevertheless, fix and heading handling assumes the fields above, so configuring an incompatible type will fail at runtime. No QoS is created by this package because bag reading is offline; offered QoS stored in bag metadata is historical recording metadata, not an active publisher/subscriber policy.

The generated waypoint file is a non-ROS YAML interface consumed by `hybrid_smooth_path_follower`:

```yaml
coordinate_mode: latlon
waypoint_order: lat_lon       # or lon_lat
estimated_duration_min: 18.72
waypoints:
  - [2.90287156, 101.28983872]
```

List rows follow `waypoint_order`. The loader also accepts mappings with `lat`/`latitude` and `lon`/`lng`/`longitude`; the writer always emits list rows and refuses an empty route (`route_authoring_tool/waypoint_io.py:52`, `route_authoring_tool/waypoint_io.py:154`). `estimated_duration_min` is route length divided by `estimation.nominal_speed_mps` and 60, rounded to two decimals on write.

## Configuration

### `config/route_authoring.yaml`

Both executables load this installed file by default and deep-merge it over identical built-in defaults in `route_authoring_tool/config_loader.py:19`. A user-supplied `--config` replaces the selected file, while missing keys retain built-in values. These are application settings, not ROS parameters; they have no ROS parameter type, descriptor, read-only flag, or dynamic parameter callback. Values are read once at startup.

| Key | Type | Default | Consumer and behavior |
|---|---|---:|---|
| `io.bag_path` | string | `""` | Bag directory; overridden by `--bag` |
| `io.input_waypoints` | string | `""` | Single existing route; overridden by multi-value `--input` |
| `io.output_path` | string | `""` | Output path fallback |
| `bag.rtk_fix_topic` | string | `/rtk/fix` | Required recorded fix topic passed to `read_bag` |
| `bag.rtk_heading_topic` | string | `/rtk_heading/float` | Optional recorded heading topic passed to `read_bag`; samples are collected but otherwise unused |
| `bag.imu_topic` | string | `/imu/data` | **Unused by implementation** |
| `bag.odom_topic` | string | `/Odometry` | **Unused by implementation** |
| `bag.nav_sat_status_min` | integer | `0` | Drops fixes with lower `status.status` |
| `bag.pre_decimate_every_n` | integer >= 1 | `1` | Keeps every Nth fix after status filtering |
| `downsample.rdp_epsilon_m` | float | `0.4` | RDP perpendicular-error tolerance in local metres |
| `downsample.max_segment_m` | float | `5.0` | Splits simplified segments above this length; non-positive disables splitting |
| `downsample.trim_head_samples` | integer | `0` | Removes this many pre-decimated leading fixes |
| `downsample.trim_tail_samples` | integer | `0` | Removes this many pre-decimated trailing fixes |
| `output.waypoint_order` | string | `lat_lon` | Output list ordering; valid values are `lat_lon`, `lon_lat` |
| `output.coord_decimals` | integer | `8` | Decimal places written for coordinates |
| `estimation.nominal_speed_mps` | float | `0.5` | Duration estimate speed; internally clamped to at least `0.001` m/s |
| `editor.figure_size_inches` | two-float list | `[11.0, 9.0]` | Matplotlib figure size |
| `editor.background_color` | color string | `#ffffff` | Figure and axes background |
| `editor.show_raw_trail` | boolean | `true` | Initial raw-trail visibility |
| `editor.raw_trail_alpha` | float | `0.35` | Raw-trail opacity and toggle-on opacity |
| `editor.raw_trail_color` | color string | `#888888` | Raw-trail color |
| `editor.waypoint_color` | color string | `#cc2222` | **Unused**; waypoint markers use the hard-coded per-route palette |
| `editor.waypoint_marker_size` | integer | `9` | Active marker size; inactive markers use 65% |
| `editor.path_line_color` | color string | `#225a99` | **Unused as a setting**; the first hard-coded palette entry happens to match it |
| `editor.path_line_width` | float | `1.8` | Active path width; inactive routes use 70% with a 0.6 minimum |
| `editor.selected_color` | color string | `#ffaa00` | Selection and snap-preview color |
| `editor.selected_marker_size` | integer | `14` | Selected marker size |
| `editor.waypoint_pick_radius_m` | float | `0.6` | Maximum local-metre distance for selecting a waypoint |
| `editor.snap_tolerance_m` | float | `1.5` | Maximum click-to-polyline distance for insertion/preview |
| `editor.undo_depth` | integer | `50` | Per-route undo history limit |
| `editor.autosave_on_quit` | boolean | `false` | **Ignored by close behavior**; dirty routes are always saved when possible (`route_authoring_tool/editor.py:435`) |
| `editor.status_refresh_hz` | float | `20.0` | **Unused**; motion/redraw events are not throttled |

Internally, latitude/longitude is projected into a WGS84-based equirectangular local frame anchored on the first source point. All editor and downsampling distances are measured in that frame (`route_authoring_tool/geo.py:23`).

### Repository-only route and bag assets

These assets are not included by `setup.py` and therefore do not appear in an installed package share directory.

| Asset | Contents |
|---|---|
| `resource/routes/area_d.yaml` | 127 `lat_lon` waypoints, estimated 18.72 min, explicitly closed |
| `resource/routes/area_d_e.yaml` | 196 waypoints, estimated 28.15 min |
| `resource/routes/area_e.yaml` | 250 waypoints, estimated 35.74 min, explicitly closed |
| `resource/routes/route_1.yaml` | 132 waypoints, estimated 18.72 min |
| `resource/routes/route_2.yaml` | 243 waypoints, estimated 35.53 min |
| `resource/routes/route_3.yaml` | 132 waypoints, estimated 18.73 min |
| `resource/routes/original/route_1.yaml` | 131-waypoint earlier route, estimated 18.66 min |
| `resource/routes/original/route_2.yaml` | 244-waypoint earlier route, estimated 35.58 min |
| `resource/routes/Figure_1.png` | 1100 x 900 screenshot of a single-route editor view |
| `bags/route_area_3/` | Humble SQLite3 bag: 339,150 messages over about 788.76 s |
| `bags/route_area_d/` | Humble SQLite3 bag: 347,517 messages over about 808.33 s |
| `bags/route_area_e/` | Humble SQLite3 bag: 454,841 messages over about 1057.78 s |

Each bundled bag records `/rtk/fix` (`sensor_msgs/msg/NavSatFix`), `/rtk_heading/float` (`std_msgs/msg/Float32`), `/imu/data` (`sensor_msgs/msg/Imu`), and `/Odometry` (`nav_msgs/msg/Odometry`) using CDR serialization. The bag directories are ignored by `.gitignore:31`, so their availability in other clones is **[UNCLEAR — verify]**.

## Testing

There is no `test/` directory and no unit, integration, launch, or registered lint test in this package. `setup.py:23` names `pytest` in the legacy `tests_require` field, and `package.xml` declares three ament linter dependencies, but no test modules or `setup.py` test hooks use them.

Run the package's registered test set (currently expected to contain no tests) with:

```bash
colcon test --packages-select route_authoring_tool
colcon test-result --verbose
```

Manual linter invocations, if installed, are:

```bash
ament_flake8 route_authoring_tool
ament_pep257 route_authoring_tool
ament_copyright route_authoring_tool
```

Audit-time verification on ROS 2 Humble (2026-08-06) successfully built the package in an isolated `/tmp` build/install tree, compiled every Python module, and opened `--help` through `ros2 run` for both executables. An isolated `colcon test` reported `0 tests, 0 errors, 0 failures, 0 skipped`. The manual linters are currently failing: `ament_flake8` reports 21 findings, `ament_pep257` reports 30 findings, and `ament_copyright` reports 10 missing notices. These tools are not registered with `colcon test`, so that command alone does not expose the failures.

Notable gaps include bag-reader behavior, storage-plugin detection, fix-status filtering, CLI precedence/error codes, RDP and maximum-segment math, coordinate round trips, waypoint schema validation, duration calculation, multi-route output pairing, GUI event behavior, save/reload/undo, and launch execution. The bundled bags and route files are examples, not automated fixtures.

## Known Issues

- No `TODO`, `FIXME`, `HACK`, or `XXX` comments were found in package source, launch, config, setup, or resource text files.
- `bag.imu_topic` and `bag.odom_topic` are parsed but never passed into `read_bag`; IMU and odometry records are not read. Heading records are read into `BagTrail.headings` but are never visualized or otherwise consumed, despite source/config commentary describing richer visualization (`route_authoring_tool/config_loader.py:25`, `route_authoring_tool/bag_reader.py:85`, `route_authoring_tool/cli.py:82`).
- `editor.waypoint_color`, `editor.path_line_color`, and `editor.status_refresh_hz` are parsed but unused. Route colors come from `_ROUTE_COLOR_PALETTE`, and redraws are not rate-limited (`route_authoring_tool/editor.py:47`, `route_authoring_tool/editor.py:122`, `route_authoring_tool/editor.py:383`).
- `editor.autosave_on_quit` is a dead setting: `RouteEditor._on_close` always saves dirty layers that have output paths, regardless of the configured value (`route_authoring_tool/editor.py:435`). This contradicts `config/route_authoring.yaml:108`.
- The shared parser exposes `--input` for `bag_to_route`, but that executable ignores it (`route_authoring_tool/cli.py:31`, `route_authoring_tool/cli.py:141`). This is a dead CLI path.
- `ament_index_python` and `launch` are runtime imports but are missing from `package.xml` (`route_authoring_tool/config_loader.py:213`, `launch/route_editor.launch.py:12`). The manifest also omits the conventional `ament_python` build-tool dependency. Build/runtime success may currently rely on the surrounding ROS installation.
- `python3-numpy` is declared but unused. `nav_msgs` is not used by the current reader, and `sensor_msgs/msg/Imu` is only represented in config/bag metadata.
- The launch file is broken in a clean Humble install: `ExecuteProcess(cmd=['route_editor', ...])` searches `PATH` and raises `FileNotFoundError`, while the script is installed under `lib/route_authoring_tool`. It needs package-aware executable lookup or an installed-path substitution (`launch/route_editor.launch.py:25`).
- Loading an existing route preserves only its coordinate list. On save, the editor uses the global output config and recomputes duration; the input file's `waypoint_order` and manual `estimated_duration_min` are not preserved (`route_authoring_tool/cli.py:223`, `route_authoring_tool/editor.py:491`).
- No active ROS QoS profiles exist, so there is no in-package publisher/subscriber QoS to compare. The numeric profiles in bundled bag metadata describe the original recording endpoints only; consumer compatibility cannot be assessed from this package.
- `hybrid_smooth_path_follower` is external to this package. Its current YAML loader, lookahead assumptions, mission-server propagation, and compatibility with generated files were not traced here; those cross-package claims remain **[UNCLEAR — verify]**.
- MCAP is inferred solely from file extension and requires an available rosbag2 MCAP storage plugin. Bags with another storage backend fall back to `sqlite3` and may fail to open (`route_authoring_tool/bag_reader.py:70`).
- Generated CPython 3.10 bytecode exists under `route_authoring_tool/__pycache__/`. It is ignored and not installed; source files, not bytecode, are authoritative.
- Repository route examples, the screenshot, and bundled bags are not installed by `setup.py`. The bags are also gitignored, so they may be local-only data.
- Automated coverage is absent; all behavioral paths listed in [Testing](#testing) remain unverified by repository tests.
- The declared manual lint tools do not currently pass: Flake8 reports import ordering, docstring spacing, one line-length violation, and unused imports; PEP 257 reports docstring-style violations; every checked Python/launch file lacks the copyright notice expected by `ament_copyright`.

## Glossary

| Term | Meaning in this package |
|---|---|
| Authoring | Offline creation or editing of a route; not live navigation |
| CDR | Common Data Representation, the serialization format stored in the bundled ROS 2 bags |
| Fix | One GNSS latitude/longitude sample from `NavSatFix` that passes the configured status threshold |
| GBAS / SBAS | Ground-/Satellite-Based Augmentation System status values represented by `NavSatStatus`; the filter compares their numeric status to `nav_sat_status_min` |
| GNSS / GPS | Satellite positioning; this package uses GPS colloquially while consuming ROS `NavSatFix` |
| Local frame | Equirectangular East/North metric coordinates anchored on the first route sample |
| MCAP | Optional rosbag2 storage format detected from `.mcap` files |
| RDP | Ramer-Douglas-Peucker polyline simplification; `rdp_epsilon_m` controls retained deviation |
| RTK | Real-Time Kinematic GNSS, used here for the fix and heading topic naming |
| Raw trail | Filtered and pre-decimated bag fixes before RDP/segment processing |
| Route layer | One editable waypoint polyline in the multi-route GUI |
| Snap | Projection of a nearby click onto the closest point of the active route polyline |
| Waypoint order | Whether each output list row stores latitude then longitude (`lat_lon`) or the reverse (`lon_lat`) |
| WGS84 | Latitude/longitude coordinate convention used by input fixes and output route YAML |
