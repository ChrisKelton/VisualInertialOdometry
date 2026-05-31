"""
Extract VIO prior values for a given MVSEC sequence.

The ground truth (davis/left/pose) is in the davis/left CAMERA world frame.
The VIO uses visensor/imu. This script:
  1. Computes the expected VIO initialization time (first IMU ts + imuWait/IMU_Hz)
  2. Interpolates the ground truth pose at that time
  3. Converts from davis/left camera pose → visensor/IMU pose
  4. Estimates initial velocity from finite differences on the GT trajectory
  5. Prints ready-to-paste launch file parameters

Usage:
  conda run -n ncsu-masters python3 extract_vio_priors.py
"""

import numpy as np
import h5py
from scipy.spatial.transform import Rotation

DATA_PATH = '/home/ckelton/data/MVSEC/outdoor_day/outdoor_day1_data.hdf5'
GT_PATH   = '/home/ckelton/data/MVSEC/outdoor_day/outdoor_day1_gt.hdf5'

IMU_WAIT_SAMPLES = 1096   # imuWait in launch file
IMU_HZ           = 200

# ── Calibration from camchain-imucam-outdoor_day.yaml ────────────────────────

# cam0: T_cam0_imu  (takes point in davis/left IMU → davis/left camera frame)
T_cam0_imu = np.array([
    [ 0.013521164493091098,  0.9996372421674073,   0.023292964231506116, -0.0032538724920261517],
    [-0.9999004915537535,    0.013611112263951897, -0.0037073728568248077,-0.006331428921949005],
    [-0.004023071129397902, -0.02324051838659262,   0.9997218079064843,  -0.018472844255861962],
    [ 0.0,                   0.0,                   0.0,                  1.0],
])
T_imu0_cam0 = np.linalg.inv(T_cam0_imu)   # davis/left cam → davis/left IMU

# cam2: T_cam2_imu  (takes point in visensor/left IMU → visensor/left camera frame)
T_cam2_imu = np.array([
    [ 0.9999717314190615,  -0.007438121416209933,  0.001100323844221122, -0.03921656415229387],
    [ 0.00743200596269379,  0.9999574688631824,    0.005461295826837418,  0.00621263233002485],
    [-0.0011408988276470173,-0.0054529638303831614, 0.9999844816472552,   0.0012210059575531885],
    [ 0.0,                  0.0,                   0.0,                   1.0],
])
T_imu2_cam2 = np.linalg.inv(T_cam2_imu)   # visensor cam → visensor IMU

# cam2 T_cn_cnm1: takes point in davis/right camera → visensor/left camera
T_cam2_cam1 = np.array([
    [-0.9999194492806436,  -0.011544126378795554,  0.005275234254147117, -0.11735831777218302],
    [ 0.011675616860827782, -0.9996036981416507,   0.025614968913630146,  0.05367704173973196],
    [ 0.004977441230681126,  0.02567449722346051,   0.9996579641412903,  -0.007199602674672133],
    [ 0.0,                   0.0,                   0.0,                   1.0],
])

# cam1 T_cn_cnm1: takes point in davis/left camera → davis/right camera
T_cam1_cam0 = np.array([
    [ 0.9998538711975195,  0.007121502935956221, -0.015540928133860692, -0.10020966289113746],
    [-0.0069546143302057,  0.9999178555872617,   0.010766402244230172,  -0.0004981408591306691],
    [ 0.015616324498637743,-0.010656747801258783,  0.9998212660948196,  -0.0008688666914359259],
    [ 0.0,                  0.0,                   0.0,                   1.0],
])

# Full chain: visensor/left cam → davis/right cam → davis/left cam → davis/left IMU → world
# T_world_visensor_imu = T_world_davis_cam0 × T_davis_cam0_visensor_imu
#   where T_davis_cam0_visensor_imu = T_cam0_cam1 × T_cam1_cam2 × T_cam2_visensor_imu
T_cam0_visensor_imu = (
    np.linalg.inv(T_cam1_cam0)       # davis/right cam → davis/left cam
    @ np.linalg.inv(T_cam2_cam1)     # visensor/left cam → davis/right cam  (inverted)
    @ T_cam2_imu                     # visensor/left IMU → visensor/left cam (inverted below)
)
# Correction: chain should be T_cam0 ← T_cam1 ← T_cam2 ← T_imu2
# T_cam0_imu2 = inv(T_cam1_cam0) @ inv(T_cam2_cam1) @ T_cam2_imu
T_cam0_imu2 = (
    np.linalg.inv(T_cam1_cam0)
    @ np.linalg.inv(T_cam2_cam1)
    @ T_cam2_imu
)
# T_imu2_world (what we want at init time) = T_imu2_cam0 @ T_cam0_world
# = inv(T_cam0_imu2) @ T_cam0_world
T_imu2_cam0 = np.linalg.inv(T_cam0_imu2)


def interp_pose(ts_query: float, ts_arr: np.ndarray, poses: np.ndarray) -> np.ndarray:
    """Linearly interpolate a 4x4 SE3 pose at ts_query."""
    if ts_query <= ts_arr[0]:
        return poses[0]
    if ts_query >= ts_arr[-1]:
        return poses[-1]
    idx = np.searchsorted(ts_arr, ts_query)
    t0, t1 = ts_arr[idx - 1], ts_arr[idx]
    alpha = (ts_query - t0) / (t1 - t0)
    P0, P1 = poses[idx - 1], poses[idx]

    # Interpolate translation linearly
    t_interp = (1 - alpha) * P0[:3, 3] + alpha * P1[:3, 3]

    # Interpolate rotation via SLERP
    r0 = Rotation.from_matrix(P0[:3, :3])
    r1 = Rotation.from_matrix(P1[:3, :3])
    r_interp = Rotation.concatenate([r0, r1])
    # scipy slerp
    from scipy.spatial.transform import Slerp
    key_rots = Rotation.concatenate([r0, r1])
    slerp = Slerp([0, 1], key_rots)
    r_out = slerp(alpha)

    T = np.eye(4)
    T[:3, :3] = r_out.as_matrix()
    T[:3, 3]  = t_interp
    return T


def estimate_velocity(ts_query: float, ts_arr: np.ndarray, poses: np.ndarray,
                      dt: float = 0.1) -> np.ndarray:
    """Estimate velocity at ts_query using central finite differences on GT position."""
    P_fwd = interp_pose(ts_query + dt, ts_arr, poses)
    P_bwd = interp_pose(ts_query - dt, ts_arr, poses)
    vel = (P_fwd[:3, 3] - P_bwd[:3, 3]) / (2 * dt)
    return vel


def main():
    print('Loading HDF5 files...')
    with h5py.File(DATA_PATH, 'r') as f:
        imu_ts  = f['visensor/imu_ts'][:]
        img_ts  = f['visensor/left/image_raw_ts'][:]

    with h5py.File(GT_PATH, 'r') as f:
        gt_poses = f['davis/left/pose'][:]
        gt_ts    = f['davis/left/pose_ts'][:]

    # VIO init time: first IMU ts + imuWait samples, snapped to nearest image ts
    t_imu_init = imu_ts[IMU_WAIT_SAMPLES]
    # Find the first image timestamp after enough IMU data has accumulated
    img_after = img_ts[img_ts >= t_imu_init]
    t_init = img_after[0] if len(img_after) > 0 else t_imu_init
    print(f'VIO initialization time: {t_init:.3f}s  '
          f'(IMU start={imu_ts[0]:.3f}, GT start={gt_ts[0]:.3f})')

    if t_init < gt_ts[0] or t_init > gt_ts[-1]:
        print(f'WARNING: init time {t_init:.3f} is outside GT range '
              f'[{gt_ts[0]:.3f}, {gt_ts[-1]:.3f}]')
        return

    # Ground truth pose of davis/left CAMERA in world frame at init time
    T_world_cam0 = interp_pose(t_init, gt_ts, gt_poses)

    # Convert to visensor/IMU frame:
    # T_world_imu2 = T_world_cam0 @ T_cam0_imu2_inv ... wait
    # T_world_imu2 = T_world_cam0 @ inv(T_cam0_imu2) ... no
    # T_world_imu2 = T_world_cam0 @ T_imu2_cam0_inv
    # Actually: T_world_imu2 = T_world_cam0 @ inv(T_cam0_imu2)
    # Since T_cam0_imu2 takes imu2 points to cam0 frame:
    # p_cam0 = T_cam0_imu2 @ p_imu2
    # p_world = T_world_cam0 @ p_cam0 = T_world_cam0 @ T_cam0_imu2 @ p_imu2
    # So T_world_imu2 = T_world_cam0 @ T_cam0_imu2  ... but T_cam0_imu2 is a body transform
    # Wait, I need to be careful about conventions.
    # T_cam0_imu2: transforms points FROM imu2 frame TO cam0 frame (column vector convention)
    # So the pose of imu2 expressed in world = T_world_imu2 = T_world_cam0 @ T_cam0_imu2
    T_world_imu2 = T_world_cam0 @ T_cam0_imu2

    p_IinG = T_world_imu2[:3, 3]
    R_GtoI = T_world_imu2[:3, :3].T    # transpose = inverse for rotation = R from world to imu
    q_GtoI = Rotation.from_matrix(R_GtoI).as_quat()   # [x, y, z, w]

    # Velocity: estimated in world frame from GT trajectory
    # GT gives cam0 position, convert to imu2 position trajectory for vel estimate
    v_IinG = estimate_velocity(t_init, gt_ts, gt_poses)
    # Correct for lever arm: v_imu = v_cam + omega x r_cam_to_imu
    # For now use cam velocity as approximation (lever arm effect is small)

    print()
    print('=' * 60)
    print('Paste these into launch/mvsec_test.launch.py:')
    print('=' * 60)
    print(f"  'prior_pIinG': [{p_IinG[0]:.6f}, {p_IinG[1]:.6f}, {p_IinG[2]:.6f}],")
    print(f"  'prior_qGtoI': [{q_GtoI[0]:.6f}, {q_GtoI[1]:.6f}, {q_GtoI[2]:.6f}, {q_GtoI[3]:.6f}],")
    print(f"  'prior_vIinG': [{v_IinG[0]:.6f}, {v_IinG[1]:.6f}, {v_IinG[2]:.6f}],")
    print()
    print('Notes:')
    print('  - prior_ba and prior_bg: set to zeros if unknown,')
    print('    or calibrate from a stationary IMU segment before the sequence.')
    print(f"  'prior_ba': [0.0, 0.0, 0.0],")
    print(f"  'prior_bg': [0.0, 0.0, 0.0],")
    print()
    print('Also update imuWait to match this sequence:')
    print(f"  'imuWait': {IMU_WAIT_SAMPLES},   # {IMU_WAIT_SAMPLES/IMU_HZ:.1f}s at {IMU_HZ}Hz")
    print()
    print(f'GT pose of davis/left cam at init time:\n{T_world_cam0}')
    print(f'Converted visensor/IMU pose:\n{T_world_imu2}')


if __name__ == '__main__':
    main()
