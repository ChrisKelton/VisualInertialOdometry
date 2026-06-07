"""
Publish MVSEC HDF5 data as ROS2 topics in real time.

Topics published:
  /visensor/imu              sensor_msgs/msg/Imu          200 Hz
  /visensor/left/image_raw   sensor_msgs/msg/Image         20 Hz  (752x480, mono8)
  /ground_truth/pose         geometry_msgs/msg/PoseStamped ~17 Hz
  /ground_truth/path         nav_msgs/msg/Path             ~17 Hz (accumulated)

Usage:
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  conda run -n ncsu-masters python3 mvsec_hdf5_publisher.py [--speed 1.0]

Then in a separate terminal launch the VIO system:
  ros2 launch launch/mvsec_test.launch.py

Note: Ground truth poses are in the davis/left camera frame. The VIO output is in
the IMU/world frame defined by the prior in mvsec_test.launch.py. To compare them
you must align the two trajectories (e.g. using ATE alignment).
"""

import argparse
import time
import sys
import numpy as np
import h5py
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from builtin_interfaces.msg import Time as RosTime
import matplotlib.pyplot as plt
from pathlib import Path as PPath

DATA_PATH = '/home/ckelton/data/MVSEC/outdoor_day/outdoor_day1_data.hdf5'
GT_PATH   = '/home/ckelton/data/MVSEC/outdoor_day/outdoor_day1_gt.hdf5'


def to_ros_time(ts_sec: float) -> RosTime:
    t = RosTime()
    t.sec = int(ts_sec)
    t.nanosec = int((ts_sec - int(ts_sec)) * 1e9)
    return t


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(R).as_quat()
    # """Rotation matrix → quaternion [x, y, z, w] via Shepperd's method."""
    # trace = R[0, 0] + R[1, 1] + R[2, 2]
    # if trace > 0:
    #     s = 0.5 / np.sqrt(trace + 1.0)
    #     w = 0.25 / s
    #     x = (R[2, 1] - R[1, 2]) * s
    #     y = (R[0, 2] - R[2, 0]) * s
    #     z = (R[1, 0] - R[0, 1]) * s
    # elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
    #     s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
    #     w = (R[2, 1] - R[1, 2]) / s
    #     x = 0.25 * s
    #     y = (R[0, 1] + R[1, 0]) / s
    #     z = (R[0, 2] + R[2, 0]) / s
    # elif R[1, 1] > R[2, 2]:
    #     s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
    #     w = (R[0, 2] - R[2, 0]) / s
    #     x = (R[0, 1] + R[1, 0]) / s
    #     y = 0.25 * s
    #     z = (R[1, 2] + R[2, 1]) / s
    # else:
    #     s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
    #     w = (R[1, 0] - R[0, 1]) / s
    #     x = (R[0, 2] + R[2, 0]) / s
    #     y = (R[1, 2] + R[2, 1]) / s
    #     z = 0.25 * s
    # return np.array([x, y, z, w])


def plot_imu_data(imu_data: np.ndarray, imu_ts: np.ndarray, output_dir: PPath):
    output_dir.mkdir(exist_ok=True, parents=True)
    df = pd.DataFrame(data=np.column_stack([imu_ts, imu_data]),
                      columns=["ts", "Accel X", "Accel Y", "Accel Z", "Gyro X", "Gyro Y", "Gyro Z"],
                      index=np.arange(0, len(imu_ts)))
    ts = df["ts"] - df["ts"][0]
    colors = ['r', 'g', 'b']
    ylabel_units = ["[m/s^2]", "[rad/s]"]
    output_names = ["acceleration.png", "gyrometer.png"]
    for col_idx, column_name_group in enumerate(zip(*[iter(df.columns[1:])]*3)):
        fig, axs = plt.subplots(nrows=3, ncols=1, figsize=(8, 10), sharex=True)
        units = ylabel_units[col_idx]
        min_ = 1e12
        max_ = 0.0
        stds_ = []
        for column_name in column_name_group:
            min_ = min(np.min(df[column_name]), min_)
            max_ = max(np.max(df[column_name]), max_)
            stds_.append(np.std(df[column_name]))
        for idx, column_name in enumerate(column_name_group):
            axs[idx].plot(ts, df[column_name], colors[idx])
            axs[idx].set_title(column_name)
            axs[idx].set_ylabel(f"{column_name} {units}")
            axs[idx].set_ylim(min_ - np.mean(stds_), max_ + np.mean(stds_))
        axs[-1].set_xlabel("Time (s)")
        plt.tight_layout()
        fig.savefig(str(output_dir / output_names[col_idx]))
        plt.close()

class MvsecPublisher(Node):
    def __init__(self, data_path: str, gt_path: str, speed: float = 1.0):
        super().__init__('mvsec_publisher')
        self.speed = speed

        self.pub_imu   = self.create_publisher(Imu,           '/visensor/imu',              2000)
        self.pub_img   = self.create_publisher(Image,          '/visensor/left/image_raw',   100)
        self.pub_pose  = self.create_publisher(PoseStamped,    '/ground_truth/pose',         100)
        self.pub_path  = self.create_publisher(Path,           '/ground_truth/path',         10)

        self._load(data_path, gt_path)
        self._build_timeline()

        self.gt_path_msg = Path()
        self.gt_path_msg.header.frame_id = 'world'

        self.idx = 0
        self.start_wall: float | None = None
        self.start_data: float | None = None

        # 1 ms tick — tight enough to keep up with 200 Hz IMU
        self.timer = self.create_timer(0.001, self._tick)

        # Progress report every 5 s
        self.progress_timer = self.create_timer(5.0, self._report_progress)

        self.get_logger().info(
            f'Ready: {len(self.imu_ts)} IMU | {len(self.img_ts)} images | '
            f'{len(self.gt_ts)} GT poses | speed={self.speed}x'
        )

    # ------------------------------------------------------------------
    def _load(self, data_path: str, gt_path: str) -> None:
        self.get_logger().info('Loading data HDF5...')
        t0 = time.time()
        with h5py.File(data_path, 'r') as f:
            total = 3
            self.imu_data = f['visensor/imu'][:]
            self.get_logger().info(f'  [1/{total}] imu loaded  ({time.time()-t0:.1f}s)')
            self.imu_ts   = f['visensor/imu_ts'][:]
            self.get_logger().info(f'  [2/{total}] imu_ts loaded  ({time.time()-t0:.1f}s)')
            self.img_data = f['visensor/left/image_raw'][:]
            self.img_ts   = f['visensor/left/image_raw_ts'][:]
            self.get_logger().info(f'  [3/{total}] images loaded  ({time.time()-t0:.1f}s)')

        self.get_logger().info('Loading ground truth HDF5...')
        t0 = time.time()
        with h5py.File(gt_path, 'r') as f:
            total = 2
            self.gt_pose = f['davis/left/pose'][:]
            self.get_logger().info(f'  [1/{total}] pose loaded  ({time.time()-t0:.1f}s)')
            self.gt_ts   = f['davis/left/pose_ts'][:]
            self.get_logger().info(f'  [2/{total}] pose_ts loaded  ({time.time()-t0:.1f}s)')

    def _build_timeline(self) -> None:
        events = (
            [(ts, 'imu', i) for i, ts in enumerate(self.imu_ts)] +
            [(ts, 'img', i) for i, ts in enumerate(self.img_ts)] +
            [(ts, 'gt',  i) for i, ts in enumerate(self.gt_ts)]
        )
        events.sort(key=lambda e: e[0])
        self.timeline = events
        self.get_logger().info(f'Timeline built: {len(self.timeline)} events')

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        if self.idx >= len(self.timeline):
            self.get_logger().info('Finished — all events published.')
            self.timer.cancel()
            self.progress_timer.cancel()
            return

        now = time.time()
        if self.start_wall is None:
            self.start_wall = now
            self.start_data = self.timeline[0][0]

        # Drain all events that should have fired by now
        while self.idx < len(self.timeline):
            ts, kind, i = self.timeline[self.idx]
            data_elapsed = (ts - self.start_data) / self.speed
            wall_elapsed = now - self.start_wall
            if data_elapsed > wall_elapsed:
                break

            stamp = to_ros_time(ts)
            if kind == 'imu':
                self._pub_imu(i, stamp)
                # pass
            elif kind == 'img':
                self._pub_img(i, stamp)
                # pass
            else:
                self._pub_gt(i, stamp)

            self.idx += 1

    def _report_progress(self) -> None:
        if self.idx == 0 or self.start_data is None:
            return
        pct = 100.0 * self.idx / len(self.timeline)
        ts_now = self.timeline[min(self.idx, len(self.timeline) - 1)][0]
        data_elapsed = ts_now - self.start_data
        wall_elapsed = time.time() - self.start_wall
        total_data = self.timeline[-1][0] - self.start_data
        remaining_data = total_data - data_elapsed
        eta = remaining_data / self.speed - wall_elapsed + (time.time() - self.start_wall)
        self.get_logger().info(
            f'Progress: {pct:.1f}%  ({self.idx}/{len(self.timeline)} events)  '
            f'ETA: {max(0.0, remaining_data / self.speed):.0f}s'
        )

    # ------------------------------------------------------------------
    def _pub_imu(self, idx: int, stamp: RosTime) -> None:
        ax, ay, az, gx, gy, gz = self.imu_data[idx]
        msg = Imu()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'imu'
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        # No orientation in this dataset — signal with -1 in first covariance element
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0
        self.pub_imu.publish(msg)

    def _pub_img(self, idx: int, stamp: RosTime) -> None:
        img = np.flipud(self.img_data[idx])   # (480, 752) uint8; HDF5 rows are bottom-up
        msg = Image()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'camera'
        msg.height   = img.shape[0]
        msg.width    = img.shape[1]
        msg.encoding = 'mono8'
        msg.step     = img.shape[1]
        msg.data     = img.tobytes()
        self.pub_img.publish(msg)

    def _pub_gt(self, idx: int, stamp: RosTime) -> None:
        T = self.gt_pose[idx]      # (4, 4) SE3
        t = T[:3, 3]
        q = rot_to_quat(T[:3, :3])

        pose_msg = PoseStamped()
        pose_msg.header.stamp    = stamp
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x    = t[0]
        pose_msg.pose.position.y    = t[1]
        pose_msg.pose.position.z    = t[2]
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        self.pub_pose.publish(pose_msg)

        self.gt_path_msg.header.stamp = stamp
        self.gt_path_msg.poses.append(pose_msg)
        self.pub_path.publish(self.gt_path_msg)


# ----------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(description='Publish MVSEC HDF5 data as ROS2 topics')
    parser.add_argument('--data', default=DATA_PATH,  help='Path to outdoor_day1_data.hdf5')
    parser.add_argument('--gt',   default=GT_PATH,    help='Path to outdoor_day1_gt.hdf5')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Playback speed multiplier (e.g. 0.5 = half speed)')
    args = parser.parse_args()

    rclpy.init()
    node = MvsecPublisher(args.data, args.gt, args.speed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
