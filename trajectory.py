from pathlib import Path
import pandas as pd
import h5py
import numpy as np
from scipy.spatial.transform import Rotation as R

def main():
    data_dir: Path = Path("/home/ckelton/vio/output/trajectories")
    trajectory_files = sorted(data_dir.glob("*.txt"))

    gt_path = Path("/home/ckelton/data/MVSEC/outdoor_day/outdoor_day1_gt.hdf5")

    # df = pd.DataFrame(
    #     columns=[
    #         "timestamp",
    #         "px", "py", "pz",
    #         "qw", "qx", "qy", "qz",
    #         "vx", "vy", "vz",
    #         "bax", "bay", "baz", "bgx", "bgy", "bgz"
    #     ]
    # )
    # for trajectory_file in trajectory_files:
    #     df_ = pd.read_csv(str(trajectory_file), sep=" ")
    #     df.loc[len(df)] = df_.iloc[0]

    with h5py.File(str(gt_path), 'r') as f:
        gt_pose = f['davis/left/pose'][:]
        gt_ts = f['davis/left/pose_ts'][:]

    for file_idx, trajectory_file in enumerate(trajectory_files[-1:]):
        print(f"File '{trajectory_file}'")
        df = pd.read_csv(str(trajectory_file), sep=" ")
        poses: list[np.ndarray] = []
        # (3, 3)
        #   (0, :) = velocities : [vx, vy, vz]
        #   (1, :) = accel bias : [bax, bay, baz]
        #   (2, :) = gyro bias  : [bgx, bgy, bgz]
        imu_vbs: list[np.ndarray] = []
        for idx in range(len(df)):
            series = df.iloc[idx]

            quat_arr = [series["qx"], series["qy"], series["qz"], series["qw"]]
            rotation = R.from_quat(quat_arr)
            rot_mat = rotation.as_matrix()
            translation = np.asarray([series["px"], series["py"], series["pz"]])
            pose = np.eye(4)
            pose[:3, :] = np.column_stack([rot_mat, translation])
            poses.append(pose)

            imu_vbs.append(np.asarray(series["vx":"bgz"]).reshape(3, 3))

        if len(poses) > 1:
            imu_vbs = np.stack(imu_vbs)
            imu_vbs_stats: dict[str, dict[str, np.ndarray]] = {}
            for idx, name in enumerate(["velocity", "accel_bias", "gyro_bias"]):
                median_ = np.median(imu_vbs[:, idx, :], axis=0)
                mean_ = np.mean(imu_vbs[:, idx, :], axis=0)
                max_ = np.max(imu_vbs[:, idx, :], axis=0)
                min_ = np.min(imu_vbs[:, idx, :], axis=0)
                abs_max_ = np.max(np.abs(imu_vbs[:, idx, :]), axis=0)
                abs_min_ = np.min(np.abs(imu_vbs[:, idx, :]), axis=0)
                imu_vbs_stats[name] = {
                    "median": median_,
                    "mean": mean_,
                    "max": max_,
                    "min": min_,
                    "abs_max": abs_max_,
                    "abs_min": abs_min_,
                }
                print(f"\t{name}:")
                for stat_key, vals in imu_vbs_stats[name].items():
                    print(f"\t\t{stat_key}: {vals}")
            a = 0

if __name__ == '__main__':
    main()
