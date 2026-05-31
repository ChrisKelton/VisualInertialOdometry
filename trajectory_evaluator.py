"""
Align VIO estimated trajectory to ground truth and compute ATE.

Subscribes to:
  /vio/path_imu        nav_msgs/msg/Path  — VIO estimated trajectory
  /ground_truth/path   nav_msgs/msg/Path  — MVSEC ground truth

Publishes:
  /evaluation/aligned_path   nav_msgs/msg/Path  — VIO path after SE3 alignment
  /evaluation/error_path     nav_msgs/msg/Path  — per-pose error as z-displacement

Prints ATE (RMSE) to the terminal each time both paths are updated.

Usage:
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  conda run -n ncsu-masters python3 trajectory_evaluator.py

Theory — Umeyama alignment:
  Given N corresponding position pairs (p_i estimated, q_i ground truth),
  find R, t, s minimising:
      sum_i || q_i - s*R*p_i - t ||^2
  Solution (Umeyama 1991):
    1. Centre both point sets (subtract mean)
    2. SVD of cross-covariance matrix H = P_c^T Q_c
    3. R = V diag(1,...,1,det(VU^T)) U^T
    4. s = trace(diag(1,...,det) S) / var(P_c)
    5. t = q_mean - s * R * p_mean
  ATE = sqrt( mean( || q_i - (s*R*p_i + t) ||^2 ) )
"""

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def umeyama(P: np.ndarray, Q: np.ndarray, allow_scale: bool = True):
    """
    Align P to Q using the Umeyama method.

    Args:
        P: (N, 3) estimated positions
        Q: (N, 3) ground truth positions
        allow_scale: if True compute optimal scale (Sim3), else fix s=1 (SE3)

    Returns:
        R (3,3), t (3,), s (float), P_aligned (N,3)
    """
    assert P.shape == Q.shape and P.ndim == 2 and P.shape[1] == 3

    p_mean = P.mean(axis=0)
    q_mean = Q.mean(axis=0)
    P_c = P - p_mean
    Q_c = Q - q_mean

    var_p = np.mean(np.sum(P_c ** 2, axis=1))
    H = P_c.T @ Q_c / len(P)

    U, S_diag, Vt = np.linalg.svd(H)
    V = Vt.T

    # Correct reflection
    d = np.ones(3)
    if np.linalg.det(V @ U.T) < 0:
        d[2] = -1

    R = V @ np.diag(d) @ U.T
    s = np.dot(S_diag, d) / var_p if allow_scale else 1.0
    t = q_mean - s * R @ p_mean

    P_aligned = (s * (R @ P_c.T).T) + q_mean
    return R, t, s, P_aligned


def interpolate_path(ref_ts: np.ndarray, path_ts: np.ndarray,
                     path_pos: np.ndarray) -> np.ndarray:
    """
    Linearly interpolate path_pos at ref_ts timestamps.
    Only returns values within the path_ts time range.

    Returns:
        valid_ref_ts (M,), interp_pos (M, 3)
    """
    valid = (ref_ts >= path_ts[0]) & (ref_ts <= path_ts[-1])
    ref_ts = ref_ts[valid]
    interp = np.zeros((len(ref_ts), 3))
    for dim in range(3):
        interp[:, dim] = np.interp(ref_ts, path_ts, path_pos[:, dim])
    return ref_ts, interp, valid


def path_to_arrays(path_msg: Path):
    """Extract timestamps and positions from a Path message."""
    ts  = np.array([p.header.stamp.sec + p.header.stamp.nanosec * 1e-9
                    for p in path_msg.poses])
    pos = np.array([[p.pose.position.x,
                     p.pose.position.y,
                     p.pose.position.z]
                    for p in path_msg.poses])
    return ts, pos


class TrajectoryEvaluator(Node):
    def __init__(self):
        super().__init__('trajectory_evaluator')

        self.vio_path = None
        self.gt_path  = None

        self.sub_vio = self.create_subscription(
            Path, '/vio/path_imu',      self._cb_vio, 10)
        self.sub_gt  = self.create_subscription(
            Path, '/ground_truth/path', self._cb_gt,  10)

        self.pub_aligned = self.create_publisher(Path, '/evaluation/aligned_path', 10)
        self.pub_error   = self.create_publisher(Path, '/evaluation/error_path',   10)

        # Evaluate at most every 2 s to avoid overwhelming the system
        self.eval_timer = self.create_timer(2.0, self._evaluate)

        self.get_logger().info(
            'Waiting for /vio/path_imu and /ground_truth/path ...'
        )

    def _cb_vio(self, msg: Path) -> None:
        self.vio_path = msg

    def _cb_gt(self, msg: Path) -> None:
        self.gt_path = msg

    def _evaluate(self) -> None:
        if self.vio_path is None or self.gt_path is None:
            return
        if len(self.vio_path.poses) < 4 or len(self.gt_path.poses) < 4:
            return

        vio_ts,  vio_pos  = path_to_arrays(self.vio_path)
        gt_ts,   gt_pos   = path_to_arrays(self.gt_path)

        # Interpolate ground truth at VIO timestamps
        valid_ts, gt_interp, valid_mask = interpolate_path(vio_ts, gt_ts, gt_pos)
        vio_pos_valid = vio_pos[valid_mask]

        if len(valid_ts) < 4:
            self.get_logger().warn(
                'Insufficient overlapping timestamps for alignment — '
                'check that timestamps are synchronised.'
            )
            return

        # Umeyama SE3 alignment (scale fixed at 1 — VIO is metric)
        R, t, s, vio_aligned = umeyama(vio_pos_valid, gt_interp, allow_scale=False)

        # ATE (RMSE of position error after alignment)
        errors = np.linalg.norm(vio_aligned - gt_interp, axis=1)
        ate    = np.sqrt(np.mean(errors ** 2))
        ate_max = errors.max()
        ate_min = errors.min()

        self.get_logger().info(
            f'ATE  RMSE={ate:.3f}m  max={ate_max:.3f}m  min={ate_min:.3f}m  '
            f'N={len(errors)} poses  scale={s:.4f}'
        )

        # Publish aligned path for RViz overlay
        aligned_msg = Path()
        aligned_msg.header.frame_id = 'world'
        aligned_msg.header.stamp = self.get_clock().now().to_msg()
        for i, pos in enumerate(vio_aligned):
            ps = PoseStamped()
            ps.header.frame_id = 'world'
            ps.header.stamp = self.vio_path.poses[np.where(valid_mask)[0][i]].header.stamp
            ps.pose.position.x = pos[0]
            ps.pose.position.y = pos[1]
            ps.pose.position.z = pos[2]
            ps.pose.orientation.w = 1.0
            aligned_msg.poses.append(ps)
        self.pub_aligned.publish(aligned_msg)

        # Publish error path — encode per-pose error as z-displacement from ground truth
        # so it renders as a height-coded curve in RViz
        error_msg = Path()
        error_msg.header.frame_id = 'world'
        error_msg.header.stamp = aligned_msg.header.stamp
        for i, (pos, err) in enumerate(zip(gt_interp, errors)):
            ps = PoseStamped()
            ps.header.frame_id = 'world'
            ps.header.stamp = aligned_msg.poses[i].header.stamp
            ps.pose.position.x = pos[0]
            ps.pose.position.y = pos[1]
            ps.pose.position.z = pos[2] + err   # lift by error magnitude
            ps.pose.orientation.w = 1.0
            error_msg.poses.append(ps)
        self.pub_error.publish(error_msg)


def main() -> None:
    rclpy.init()
    node = TrajectoryEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
