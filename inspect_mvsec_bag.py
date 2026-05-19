from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores
from typing import Any


def main():
    db_bag_path: Path = Path("/home/ckelton/repos/misc/VisualInertialOdometry/mvsec_test_ros2/mvsec_test_ros2.db3")
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    msgs_by_type: dict[str, list[Any]] = {}
    with Reader(str(db_bag_path)) as reader:
        for connection, timestamp, rawdata in reader.messages():
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            msgs_by_type.setdefault(str(msg.__class__.__name__), []).append(msg)

    print(f"Message Types: '{list(msgs_by_type.keys())}'")
    for key, vals in msgs_by_type.items():
        print(f"\t'{key}' count: '{len(vals)}'")


if __name__ == '__main__':
    main()
