"""Read a rosbag2 directory and pull out the GPS trail (and optional aux data).

Uses ``rosbag2_py.SequentialReader`` so this works without spinning up a node.
The user records a teleop drive of the desired route; this module extracts:

* the lat/lon timeline from ``sensor_msgs/NavSatFix`` (the only data actually
  saved to the waypoint YAML),
* optional RTK compass heading samples (``std_msgs/Float32``) and IMU/odom
  for richer visualization — none of these reach the saved YAML.

The reader filters ``NavSatFix`` by ``status.status >= nav_sat_status_min``
so dropped fixes (-1) and low-quality SBAS samples can be excluded for RTK
routes. Empty trails surface as a clear error to the caller.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from rclpy.serialization import deserialize_message  # type: ignore
from rosidl_runtime_py.utilities import get_message  # type: ignore

import rosbag2_py  # type: ignore


@dataclass
class BagTrail:
    """Everything we pulled out of the bag."""
    # (timestamp_ns, lat_deg, lon_deg). Filtered + decimated.
    fixes: List[Tuple[int, float, float]] = field(default_factory=list)
    # (timestamp_ns, heading_compass_deg) — optional.
    headings: List[Tuple[int, float]] = field(default_factory=list)
    # Topic -> count of messages observed in the bag (after status filtering
    # but before pre_decimate). Useful for "did the bag even have /rtk/fix?".
    counts: Dict[str, int] = field(default_factory=dict)
    # Storage id reported by rosbag2 ("sqlite3", "mcap", ...).
    storage_id: str = ''

    @property
    def latlons(self) -> List[Tuple[float, float]]:
        return [(lat, lon) for _, lat, lon in self.fixes]

    def __bool__(self) -> bool:
        return bool(self.fixes)


def _open_reader(bag_dir: str) -> Tuple[rosbag2_py.SequentialReader, str]:
    if not os.path.isdir(bag_dir):
        raise FileNotFoundError(f'Bag directory does not exist: {bag_dir}')
    metadata_path = os.path.join(bag_dir, 'metadata.yaml')
    if not os.path.exists(metadata_path):
        raise FileNotFoundError(
            f'No metadata.yaml in {bag_dir} — pass the FOLDER that contains '
            'metadata.yaml + *.db3, not the .db3 file itself.'
        )

    storage_id = _infer_storage_id(bag_dir)
    storage = rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id)
    converter = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)
    return reader, storage_id


def _infer_storage_id(bag_dir: str) -> str:
    """Best-effort: prefer sqlite3, fall back to mcap by inspecting files."""
    for name in os.listdir(bag_dir):
        lower = name.lower()
        if lower.endswith('.db3'):
            return 'sqlite3'
        if lower.endswith('.mcap'):
            return 'mcap'
    return 'sqlite3'  # rosbag2 default


def _topic_type_map(reader: rosbag2_py.SequentialReader) -> Dict[str, str]:
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def read_bag(
    bag_dir: str,
    rtk_fix_topic: str,
    rtk_heading_topic: str = '',
    nav_sat_status_min: int = 0,
    pre_decimate_every_n: int = 1,
) -> BagTrail:
    """Read ``bag_dir`` and return the extracted trail.

    Only ``rtk_fix_topic`` is mandatory. ``rtk_heading_topic`` can be empty
    or absent from the bag; its samples are nice-to-have for visualization
    but never written to the output YAML.

    ``pre_decimate_every_n`` keeps every Nth filtered fix sample (1 = keep
    all). Applied AFTER the status filter so the kept stride is uniform.
    """
    if pre_decimate_every_n < 1:
        raise ValueError('pre_decimate_every_n must be >= 1')

    reader, storage_id = _open_reader(bag_dir)
    topic_types = _topic_type_map(reader)

    wanted = {rtk_fix_topic}
    if rtk_heading_topic:
        wanted.add(rtk_heading_topic)

    filter_obj = rosbag2_py.StorageFilter(topics=list(wanted))
    try:
        reader.set_filter(filter_obj)
    except Exception:  # noqa: BLE001 - some builds lack set_filter; fall back to per-msg branch.
        pass

    if rtk_fix_topic not in topic_types:
        available = ', '.join(sorted(topic_types.keys()))
        raise ValueError(
            f'Topic {rtk_fix_topic!r} not found in bag. Available topics: {available}'
        )

    msg_cache: Dict[str, type] = {}

    def get_type(type_name: str) -> type:
        if type_name not in msg_cache:
            msg_cache[type_name] = get_message(type_name)
        return msg_cache[type_name]

    trail = BagTrail(storage_id=storage_id)
    fix_kept_idx = 0

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic not in wanted:
            continue

        type_name = topic_types.get(topic)
        if not type_name:
            continue
        msg = deserialize_message(raw, get_type(type_name))

        if topic == rtk_fix_topic:
            status = int(getattr(getattr(msg, 'status', None), 'status', 0))
            if status < nav_sat_status_min:
                continue
            trail.counts[topic] = trail.counts.get(topic, 0) + 1
            if (fix_kept_idx % pre_decimate_every_n) == 0:
                trail.fixes.append((int(t_ns), float(msg.latitude), float(msg.longitude)))
            fix_kept_idx += 1
        elif rtk_heading_topic and topic == rtk_heading_topic:
            trail.counts[topic] = trail.counts.get(topic, 0) + 1
            trail.headings.append((int(t_ns), float(msg.data)))

    return trail


__all__ = ['BagTrail', 'read_bag']
