"""Read a rosbag2 recording into the same streams the live recorder produces."""

from __future__ import annotations

import numpy as np

from trajectory_eval.frames import geodetic_to_enu, gps_fields_to_lla, ned_frd_to_enu_flu, quat_xyzw_to_wxyz
from trajectory_eval.metrics import PoseSample
from trajectory_eval.record import _resolve_px4_topic


def read_bag(args) -> dict[str, list[PoseSample]]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=args.storage),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    type_map = {name: [type_name] for name, type_name in topic_types.items()}
    gt_topic = args.gt_topic if args.gt_topic in topic_types else None
    rtab_topic = args.rtabmap_topic if args.rtabmap_topic in topic_types else None
    ekf_topic = _resolve_px4_topic(type_map, args.ekf_topic, "vehicle_odometry")
    gps_topic = _resolve_px4_topic(type_map, args.gps_topic, "vehicle_gps_position")
    wanted = {topic for topic in (gt_topic, rtab_topic, ekf_topic, gps_topic) if topic}
    classes = {name: get_message(topic_types[name]) for name in wanted}

    ground_truth: list[PoseSample] = []
    rtabmap: list[PoseSample] = []
    ekf: list[PoseSample] = []
    gps_raw: list[tuple[float, float, float, float]] = []

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic not in classes:
            continue
        msg = deserialize_message(data, classes[topic])
        stamp = stamp_ns * 1e-9
        if topic == gt_topic or topic == rtab_topic:
            sample = _odom(msg, stamp)
            if topic == gt_topic:
                ground_truth.append(sample)
            else:
                rtabmap.append(sample)
        elif topic == ekf_topic:
            position_enu, quat_enu = ned_frd_to_enu_flu(
                np.array(list(msg.position), dtype=float),
                np.array(list(msg.q), dtype=float),
            )
            ekf.append(PoseSample(stamp, position_enu, quat_enu))
        elif topic == gps_topic:
            lla = gps_fields_to_lla(msg)
            if lla is not None:
                gps_raw.append((stamp, *lla))

    gps: list[PoseSample] = []
    if gps_raw:
        _, lat0, lon0, alt0 = gps_raw[0]
        for stamp, lat, lon, alt in gps_raw:
            gps.append(PoseSample(
                stamp,
                geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0),
                np.array([1.0, 0.0, 0.0, 0.0]),
            ))
    return {
        "ground_truth": ground_truth,
        "gps": gps,
        "rtabmap": rtabmap,
        "ekf2": ekf,
    }


def _odom(msg, stamp: float) -> PoseSample:
    position = msg.pose.pose.position
    quat = msg.pose.pose.orientation
    return PoseSample(
        stamp,
        np.array([position.x, position.y, position.z]),
        quat_xyzw_to_wxyz(np.array([quat.x, quat.y, quat.z, quat.w])),
    )
