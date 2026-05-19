"""
Generate a synthetic ROS2 bag that mimics the MVSEC VI-Sensor format used by mvsec_test.launch.py.

Topics produced:
  /visensor/imu              sensor_msgs/msg/Imu     200 Hz
  /visensor/left/image_raw   sensor_msgs/msg/Image   25  Hz  (752x480, mono8)

Motion model: slow forward arc (so the feature tracker has something to track and
              the VIO system can initialize and estimate a trajectory).

Run:
  conda run -n ncsu-masters python generate_mvsec_test_bag.py
  ros2 bag play mvsec_test_ros2/
"""

import os
import math
import numpy as np
import cv2

from rosbags.rosbag2 import Writer
from rosbags.typesys import get_typestore, Stores

# ── parameters ──────────────────────────────────────────────────────────────
DURATION_S   = 60          # seconds of data
IMU_HZ       = 200
CAM_HZ       = 25
IMG_W, IMG_H = 752, 480

# Camera intrinsics (from mvsec_vins_config.yaml)
FX, FY = 465.66479689, 465.53104947
CX, CY = 373.17652925, 232.29333312

# Gravity
G = np.array([0.0, 0.0, 9.81007])

# Motion: gentle forward arc
#   x(t) = R * sin(omega*t)     (lateral)
#   y(t) = -R * (1-cos(omega*t)) (forward)
#   z(t) = 0
SPEED   = 2.0      # m/s
RADIUS  = 50.0     # m  (large arc → nearly straight)
OMEGA   = SPEED / RADIUS   # rad/s

# IMU noise (from config)
ACC_NOISE  = 0.08   # m/s² / sqrt(Hz)
GYR_NOISE  = 0.004  # rad/s / sqrt(Hz)

# Output bag path
BAG_PATH = os.path.join(os.path.dirname(__file__), 'mvsec_test_ros2')

# ── world points for the synthetic scene ────────────────────────────────────
rng = np.random.default_rng(42)
# 200 random 3-D points spread across a volume ahead of the camera.
# Camera starts at (0, 0, 1.5) looking in the +Y direction, so points at
# y=5..50, x=-15..15, z=0..3 will fill the field of view at various depths.
WORLD_PTS = rng.uniform(
    low=[-15,  5, 0.0],
    high=[ 15, 50, 3.0],
    size=(200, 3)
).astype(np.float64)

# ── helpers ─────────────────────────────────────────────────────────────────
def pose_at(t):
    """Return (position_world, R_camera_from_world) for the camera at time t.

    The camera drives along a gentle arc in the XY plane at height 1.5 m,
    with its optical axis pointing forward (in the direction of motion) and
    image-Y pointing down (world -Z direction).

    Camera frame convention (OpenCV):
        +X = right,  +Y = image-down,  +Z = forward (into scene)
    """
    theta = OMEGA * t
    # Camera position in world
    pos = np.array([RADIUS * math.sin(theta),
                    -RADIUS * (1 - math.cos(theta)),
                    1.5], dtype=np.float64)

    # Camera axes expressed in world coordinates
    fwd   = np.array([ math.sin(theta),  math.cos(theta), 0.0])  # +Z cam
    right = np.array([ math.cos(theta), -math.sin(theta), 0.0])  # +X cam
    down  = np.array([0.0,  0.0, -1.0])                          # +Y cam (gravity)

    # R_CW: rows are camera axes in world → transforms world→camera
    R_CW = np.array([right, down, fwd], dtype=np.float64)
    return pos, R_CW

def project_points(pos, R_CW):
    """Project WORLD_PTS into the image given world pose."""
    # Transform world points to camera frame: p_cam = R_CW @ (p_world - pos)
    rel = WORLD_PTS - pos                     # (N, 3) in world coords
    pts_cam = rel @ R_CW.T                   # (N, 3) in camera coords
    # Only keep points in front of the camera
    mask = pts_cam[:, 2] > 0.5
    pts_cam = pts_cam[mask]
    if len(pts_cam) == 0:
        return []
    u = (FX * pts_cam[:, 0] / pts_cam[:, 2] + CX).astype(int)
    v = (FY * pts_cam[:, 1] / pts_cam[:, 2] + CY).astype(int)
    in_img = (u >= 0) & (u < IMG_W) & (v >= 0) & (v < IMG_H)
    return list(zip(u[in_img], v[in_img]))

def make_image(t):
    """Render a 752×480 grayscale image with projected features and noise texture.

    goodFeaturesToTrack needs strong corners/edges.  We overlay:
      - Random Gaussian noise base  (lots of detectable texture)
      - Bright squares at projected 3-D point locations  (trackable landmarks)
    """
    # Noise base gives corner-like structure everywhere
    noise = rng.integers(30, 80, size=(IMG_H, IMG_W), dtype=np.uint8)
    img = noise.copy()

    pos, R = pose_at(t)
    for (u, v) in project_points(pos, R):
        # Small bright rectangles (sharp corners → detected by Shi-Tomasi)
        cv2.rectangle(img, (u - 4, v - 4), (u + 4, v + 4), 230, -1)
        cv2.rectangle(img, (u - 4, v - 4), (u + 4, v + 4), 50,  1)  # dark border

    return img

def imu_at(t, dt):
    """Return (angular_velocity, linear_acceleration) in camera/body frame at time t."""
    # Yaw in world = rotation around world +Z.
    # In camera frame, +Z world = -Y_cam (since down = -Z_world mapped to +Y_cam).
    # So OMEGA around world +Z  →  -OMEGA around camera +Y  →  gyro = (0, -OMEGA, 0).
    gyro_true = np.array([0.0, -OMEGA, 0.0])

    # Centripetal acceleration in world frame (toward arc centre) + gravity
    theta = OMEGA * t
    centripetal_dir = np.array([-math.cos(theta), math.sin(theta), 0.0])
    acc_world = centripetal_dir * (SPEED**2 / RADIUS) - G   # sensor reads -g at rest

    _, R_CW = pose_at(t)
    acc_body = R_CW @ acc_world

    # Add noise (scaled by sqrt(Hz))
    acc_body  += rng.standard_normal(3) * ACC_NOISE  * math.sqrt(IMU_HZ)
    gyro_true += rng.standard_normal(3) * GYR_NOISE  * math.sqrt(IMU_HZ)
    return gyro_true, acc_body

# ── build the bag ────────────────────────────────────────────────────────────
def make_time(t_sec):
    sec  = int(t_sec)
    nsec = int((t_sec - sec) * 1e9)
    return sec, nsec

def main():
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    Image   = typestore.types['sensor_msgs/msg/Image']
    Imu     = typestore.types['sensor_msgs/msg/Imu']
    Header  = typestore.types['std_msgs/msg/Header']
    Time    = typestore.types['builtin_interfaces/msg/Time']
    Vector3 = typestore.types['geometry_msgs/msg/Vector3']
    Quaternion = typestore.types['geometry_msgs/msg/Quaternion']

    img_topic = '/visensor/left/image_raw'
    imu_topic = '/visensor/imu'

    # Remove old bag if it exists
    import shutil
    if os.path.exists(BAG_PATH):
        shutil.rmtree(BAG_PATH)

    n_imu = int(DURATION_S * IMU_HZ)
    n_cam = int(DURATION_S * CAM_HZ)
    total = n_imu + n_cam
    print(f"Generating {DURATION_S}s of data: {n_imu} IMU msgs + {n_cam} camera frames")

    with Writer(BAG_PATH, version=9) as writer:
        img_conn = writer.add_connection(
            img_topic,
            Image.__msgtype__,
            typestore=typestore,
        )
        imu_conn = writer.add_connection(
            imu_topic,
            Imu.__msgtype__,
            typestore=typestore,
        )

        count = 0
        for i in range(n_imu):
            t = i / IMU_HZ
            sec, nsec = make_time(t)
            timestamp_ns = int(t * 1e9)

            gyro, acc = imu_at(t, 1.0 / IMU_HZ)

            msg = Imu(
                header=Header(
                    stamp=Time(sec=sec, nanosec=nsec),
                    frame_id='imu',
                ),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                orientation_covariance=np.array([-1.0] + [0.0]*8, dtype=np.float64),
                angular_velocity=Vector3(x=gyro[0], y=gyro[1], z=gyro[2]),
                angular_velocity_covariance=np.eye(3, dtype=np.float64).flatten() * (GYR_NOISE**2),
                linear_acceleration=Vector3(x=acc[0], y=acc[1], z=acc[2]),
                linear_acceleration_covariance=np.eye(3, dtype=np.float64).flatten() * (ACC_NOISE**2),
            )
            writer.write(imu_conn, timestamp_ns, typestore.serialize_cdr(msg, Imu.__msgtype__))

            count += 1
            if count % 2000 == 0:
                print(f"  {count}/{total} messages...", end='\r')

        for j in range(n_cam):
            t = j / CAM_HZ
            sec, nsec = make_time(t)
            timestamp_ns = int(t * 1e9)

            # TODO: Replace this with actual imagery
            frame = make_image(t)
            msg = Image(
                header=Header(
                    stamp=Time(sec=sec, nanosec=nsec),
                    frame_id='camera',
                ),
                height=IMG_H,
                width=IMG_W,
                encoding='mono8',
                is_bigendian=0,
                step=IMG_W,
                data=frame.flatten(),
            )
            writer.write(img_conn, timestamp_ns, typestore.serialize_cdr(msg, Image.__msgtype__))

            count += 1
            if count % 500 == 0:
                print(f"  {count}/{total} messages...", end='\r')

    # rosbags 0.11 writes offered_qos_profiles as a YAML sequence ([])
    # but rosbag2/yaml-cpp expects it as a serialized YAML string.
    # Also custom_data: null → custom_data: {} for the same reason.
    meta = os.path.join(BAG_PATH, 'metadata.yaml')
    with open(meta) as f:
        text = f.read()
    qos_str = (
        '"- history: 3\\n  depth: 0\\n  reliability: 1\\n  durability: 2\\n'
        '  deadline:\\n    sec: 2147483647\\n    nsec: 4294967295\\n'
        '  lifespan:\\n    sec: 2147483647\\n    nsec: 4294967295\\n'
        '  liveliness: 1\\n  liveliness_lease_duration:\\n    sec: 2147483647\\n'
        '    nsec: 4294967295\\n  avoid_ros_namespace_conventions: false"'
    )
    text = text.replace('offered_qos_profiles: []', f'offered_qos_profiles: {qos_str}')
    text = text.replace('custom_data: null', 'custom_data: {}')
    with open(meta, 'w') as f:
        f.write(text)

    print(f"\nDone. Bag written to: {BAG_PATH}")
    print(f"Play with:  ros2 bag play {BAG_PATH}")

if __name__ == '__main__':
    main()
