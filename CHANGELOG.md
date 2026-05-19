# ChangeLog — ROS1 → ROS2 Port

This document records all changes made to port the project from ROS1 (Melodic/Kinetic) to
ROS2 Humble, upgrade dependencies to current versions, and work around incompatibilities
between the pre-installed GTSAM 4.2.1 build and the system TBB 2021.5 library.

---

## Build System

### All packages — `package.xml`
- **Changed**: Updated all three `package.xml` files from catkin format (format 1/2) to
  ament_cmake format 3.
- **Changed**: Replaced `<buildtool_depend>catkin</buildtool_depend>` with
  `<buildtool_depend>ament_cmake</buildtool_depend>` and added `<build_type>ament_cmake</build_type>`
  export.
- **Changed**: Replaced ROS1-style `<build_depend>roscpp</build_depend>` /
  `<run_depend>roscpp</run_depend>` tags with `<depend>rclcpp</depend>` and other ROS2
  equivalents.

### All packages — `CMakeLists.txt`
- **Changed**: Added `find_package(ament_cmake REQUIRED)` as the first find_package call
  (required for all ROS2/ament packages).
- **Changed**: Moved `ament_package()` to the end of each `CMakeLists.txt` (in ROS2,
  `ament_package()` must be the last call).
- **Changed**: Replaced all `${catkin_INCLUDE_DIRS}` and `${catkin_LIBRARIES}` references
  (which resolve to empty in ROS2) with explicit ament target dependencies and library
  targets.

---

## `camera_model` package

### Ceres 2.x API migration
- **Changed** `include/camodocal/gpl/EigenQuaternionParameterization.h`:
  - The class `EigenQuaternionParameterization` previously inherited from
    `ceres::LocalParameterization`, which was removed in Ceres 2.0.
  - Replaced the entire class definition with a type alias:
    `using EigenQuaternionParameterization = ceres::EigenQuaternionManifold;`
  - Ceres 2.x provides `EigenQuaternionManifold` as a built-in manifold in `ceres/manifold.h`.
- **Changed** `src/gpl/EigenQuaternionParameterization.cc`:
  - Removed all method implementations (`Plus`, `ComputeJacobian`) — now handled internally
    by Ceres's built-in manifold.
- **Changed** `src/calib/CameraCalibration.cc`:
  - `ceres::LocalParameterization*` → `ceres::Manifold*`
  - `problem.SetParameterization(...)` → `problem.SetManifold(...)`

### CUDA version conflict (system has CUDA 11 + CUDA 12)
- **Changed** `CMakeLists.txt`:
  - Added `set(CUDAToolkit_ROOT /usr/local/cuda)` and `find_package(CUDAToolkit REQUIRED)`
    to hint toward CUDA 12 (required by the installed Ceres 2.2 build).
  - Linked the `Calibration` executable against `/usr/local/cuda/lib64/libcudart.so`
    explicitly rather than relying on cmake target resolution, which was selecting the
    system CUDA 11 library instead.
  - **TODO**: The `CUDAToolkit_ROOT` variable is ignored by cmake due to policy CMP0074 not
    being set to NEW. Replace the explicit path with proper cmake target resolution once
    the cmake version or policy is updated. Alternatively, remove the CUDA dependency from
    the `Calibration` executable if CUDA-accelerated solving is not needed.

### glog version conflict (`libglog.so.1` vs `libglog.so.0`)
- **Background**: Ceres 2.2 was compiled against glog 0.6+ (`libglog.so.1`). The system
  apt package only provides glog 0.4.0 (`libglog.so.0`). Linking the `camera_model` shared
  library against Ceres caused `libglog.so.1 not found` at runtime.
- **Changed** `CMakeLists.txt`:
  - Split source file lists into `CAMERA_MODEL_CORE_SRCS` (no Ceres dependency) and
    `CAMERA_MODEL_CALIB_SRCS` (includes `CameraCalibration.cc`, `CostFunctionFactory.cc`,
    `EigenQuaternionParameterization.cc` which use Ceres).
  - `camera_model` shared library now uses only `CAMERA_MODEL_CORE_SRCS` — it no longer
    links against Ceres, and therefore no longer depends on `libglog.so.1`.
  - The `Calibration` executable continues to use all sources including the Ceres-dependent
    ones, and links against Ceres (and CUDA explicitly).
  - **TODO**: Restore full Ceres-based calibration functionality in the shared library once
    a compatible version of glog is installed (`sudo apt-get install libgoogle-glog-dev`
    from a PPA or compiled from source to get glog ≥ 0.6).

### OpenCV ABI conflict (system OpenCV 4.13 vs ROS2 Humble's OpenCV 4.5)
- **Background**: The system has OpenCV 4.13 at `/usr/local` and the apt OpenCV 4.5 used
  by ROS2 Humble at `/usr/lib/x86_64-linux-gnu`. Linking `feature_tracker` against both
  produced `cv::Mat` ABI mismatches causing `setSize` assertion failures at runtime.
- **Changed** `camera_model/CMakeLists.txt`:
  - Added `find_package(Eigen3 REQUIRED)` and `include_directories(${EIGEN3_INCLUDE_DIR})`
    to ensure Ceres headers can find Eigen (since `CERES_INCLUDE_DIRS` is empty in Ceres
    2.x, which uses cmake targets instead).
  - Uses system OpenCV 4.13 at `/usr/local` (found by default `find_package(OpenCV)`).

### cmake library export
- **Changed** `CMakeLists.txt`:
  - Changed `add_library(camera_model ...)` to `add_library(camera_model SHARED ...)` so
    downstream packages can link against it as a shared library.
  - Used `PRIVATE` keyword in `target_link_libraries` to prevent transitive propagation of
    Boost, OpenCV, and Ceres dependencies to consumers.
  - Added `install(TARGETS camera_model Calibration ...)`, `install(DIRECTORY include/ ...)`,
    `ament_export_targets(camera_modelTargets HAS_LIBRARY_TARGET)`,
    `ament_export_include_directories(include)`, and `ament_export_libraries(camera_model)`
    so `find_package(camera_model)` works for downstream packages.

---

## `feature_tracker` package

### cv_bridge removal
- **Background**: `cv_bridge` in ROS2 Humble is compiled against OpenCV 4.5, while the
  `feature_tracker` links against system OpenCV 4.13. The two versions have incompatible
  `cv::Mat` internal layouts (ABI mismatch), causing runtime crashes in `setSize` when
  `cv_bridge::toCvCopy` passes a Mat to the feature-tracking code.
- **Changed** `src/feature_tracker_node.cpp`:
  - Removed `#include <cv_bridge/cv_bridge.h>`.
  - Replaced `cv_bridge::toCvCopy(img_msg, ...)` with a direct `cv::Mat` construction from
    the raw message data pointer: `cv::Mat(h, w, CV_8UC1, data, step).clone()`.
  - Replaced `cv_bridge::cvtColor(ptr, BGR8)` with `cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR)`.
  - Replaced `ptr->toImageMsg()` with a manually constructed `sensor_msgs::msg::Image`
    using `mat.datastart` / `mat.dataend`.
  - **TODO**: Once the OpenCV version conflict is resolved (either by building
    `feature_tracker` against OpenCV 4.5, or rebuilding `cv_bridge` against OpenCV 4.13),
    restore the `cv_bridge` dependency. This would simplify the image encoding logic and
    restore support for non-mono8 encodings that currently skip silently.

### ROS1 → ROS2 API migration
- **Changed** `src/parameters.h` / `src/parameters.cpp`:
  - `#include <ros/ros.h>` → `#include "rclcpp/rclcpp.hpp"`.
  - `readParameters(ros::NodeHandle&)` signature → `readParameters(rclcpp::Node::SharedPtr)`.
  - `n.getParam(name, ans)` → `node->declare_parameter<T>(name)` +
    `node->get_parameter(name, ans)`.
  - `n.shutdown()` → `rclcpp::shutdown()`.
  - `ROS_INFO_STREAM` / `ROS_ERROR_STREAM` → `RCLCPP_INFO_STREAM` / `RCLCPP_ERROR_STREAM`.
- **Changed** `src/feature_tracker.h`:
  - Removed `#include <opencv2/imgproc/types_c.h>` (deprecated OpenCV C API header; now
    unnecessary since the C constants are no longer used).
- **Changed** `src/feature_tracker.cpp`:
  - Replaced all `ROS_DEBUG(...)` logging calls with no-ops or removed (feature tracker
    timing output is not critical for operation).
  - Replaced `ROS_INFO("reading paramerter of camera...")` with `std::cout`.
  - **TODO**: Restore structured ROS2 logging using `RCLCPP_DEBUG(node->get_logger(), ...)`
    once a logger reference is accessible inside `FeatureTracker`. This requires either
    passing a logger to `FeatureTracker` methods or using a global node reference.
- **Changed** `src/feature_tracker_node.cpp`:
  - All ROS1 includes replaced with ROS2 equivalents (e.g.,
    `sensor_msgs/msg/image.hpp`, `std_msgs/msg/bool.hpp`).
  - `ros::Publisher` / `ros::Subscriber` → `rclcpp::Publisher<T>::SharedPtr` /
    `rclcpp::Subscription<T>::SharedPtr`.
  - `ros::init(argc, argv, name)` → `rclcpp::init(argc, argv)`.
  - `ros::NodeHandle n("~")` → `auto g_node = std::make_shared<rclcpp::Node>("feature_tracker")`.
  - `n.subscribe(topic, queue, cb)` → `g_node->create_subscription<T>(topic, queue, cb)`.
  - `n.advertise<T>(topic, queue)` → `g_node->create_publisher<T>(topic, queue)`.
  - `ros::spin()` → `rclcpp::spin(g_node)`.
  - `img_msg->header.stamp.toSec()` →
    `rclcpp::Time(img_msg->header.stamp).seconds()`.
  - `ros::Time::now()` usage removed (was only used in debug logging).
  - `ROS_WARN/ROS_DEBUG/ROS_INFO` → `RCLCPP_WARN/RCLCPP_DEBUG/RCLCPP_INFO` with
    `g_node->get_logger()`.
  - `ROS_BREAK()` (crash on mask load failure) → `RCLCPP_INFO` (log and continue).
  - `CV_GRAY2RGB` (deprecated C API constant) → `cv::COLOR_GRAY2RGB`.
  - Removed `#include <message_filters/subscriber.h>` (included but unused in original).
- **Changed** `CMakeLists.txt`:
  - Full rewrite for ROS2/ament: replaced catkin with ament, added
    `ament_target_dependencies`, `install(TARGETS ...)`, `ament_package()`.
  - Added `find_package(Boost REQUIRED COMPONENTS filesystem program_options system)` to
    satisfy `camera_model`'s transitive Boost dependency when importing its cmake targets.
  - Removed `find_package(cv_bridge REQUIRED)` and its `ament_target_dependencies` entry
    (see cv_bridge removal above).
  - Uses `camera_model::camera_model` (namespaced cmake target) for linking.
- **Changed** `CMakeLists.txt` — namespace fix:
  - Added `namespace='feature_tracker'` to the launch `Node()` call so the node's relative
    topic names (`"feature"`, `"feature_img"`, `"restart"`) resolve to
    `/feature_tracker/feature`, `/feature_tracker/feature_img`, `/feature_tracker/restart`.
    Without this, the VIO's topic remapping `/vio/data_uv → /feature_tracker/feature` would
    not match the publisher.

---

## `gtsam_backend` package

### ROS1 → ROS2 API migration
- **Changed** `src/main_estimator.cpp`:
  - Converted the entire file from a procedural ROS1 node to a
    `class VioNode : public rclcpp::Node` class.
  - All ROS1 global subscribers/publishers → member variables with ROS2 types.
  - `ros::NodeHandle::param<T>()` → `this->declare_parameter<T>(name, default)`.
  - `nh.subscribe(...)` / `nh.advertise<T>(...)` → `create_subscription<T>` /
    `create_publisher<T>`.
  - `ros::spin()` → `rclcpp::spin(node)`.
  - `ros::Time(timestamp)` → `rclcpp::Time(static_cast<int64_t>(timestamp * 1e9))`.
  - `msg->header.stamp.toSec()` → `rclcpp::Time(msg->header.stamp).seconds()`.
  - `patharr.header.seq = poses_seq++` removed — `seq` field was removed from ROS2 headers.
  - **TODO**: The `poses_seq` counter for path messages no longer exists in ROS2's
    `std_msgs::msg::Header`. If message sequence tracking is needed, maintain a separate
    counter and record it in the path message's metadata or a custom message field.
  - Added `signal(SIGSEGV, segfault_handler)` / `signal(SIGABRT, segfault_handler)` with
    `backtrace_symbols_fd` for debugging GTSAM crashes (see GTSAM section below).
- **Changed** `src/GraphSolver.cpp`:
  - Removed `#include <ros/ros.h>`.
  - `ROS_ERROR(...)` → `std::cerr << "[ERROR] ..."`.
  - `ROS_INFO(...)` (initialization log) → `printf(...)`.
- **Changed** `include/gtsam_backend/utils/Convert.h`:
  - `#include <geometry_msgs/PoseWithCovariance.h>` →
    `#include <geometry_msgs/msg/pose_with_covariance.hpp>`.
  - `geometry_msgs::Pose` → `geometry_msgs::msg::Pose`.
  - `geometry_msgs::PoseWithCovariance` → `geometry_msgs::msg::PoseWithCovariance`.
- **Changed** `CMakeLists.txt`:
  - Full rewrite for ROS2/ament.
  - Removed `find_package(MKL REQUIRED)` and `${MKL_LIBRARIES}` — MKL was listed as a
    build dependency but no source file includes any MKL headers; its only role was as a
    high-performance BLAS/LAPACK provider for GTSAM. GTSAM finds its own BLAS/LAPACK.
  - **TODO**: If MKL-accelerated GTSAM performance is needed, install Intel oneAPI MKL
    (`sudo apt-get install intel-mkl` or via Intel's apt repository) and re-add
    `find_package(MKL REQUIRED)` + link `${MKL_LIBRARIES}`.
  - Replaced custom `cmake/FindTBB.cmake` module (searched for `tbb_stddef.h` which was
    removed in TBB 2021+) with `find_package(TBB REQUIRED CONFIG)` to use the system
    `TBBConfig.cmake`. Updated link targets from plain `tbb;tbbmalloc` to `TBB::tbb;TBB::tbbmalloc`.
  - Added `find_package(nav_msgs REQUIRED)`, `find_package(geometry_msgs REQUIRED)`,
    `find_package(pcl_conversions REQUIRED)`.
  - Set `EIGEN_INCLUDE_DIR /usr/local/include/gtsam/3rdparty/Eigen` (GTSAM's bundled Eigen
    3.7) to match the Eigen version GTSAM was compiled against.
  - Added `target_include_directories(vio BEFORE PRIVATE /usr/local/include/gtsam/3rdparty/Eigen)`
    to ensure GTSAM's Eigen 3.7 is found before the system Eigen 4 on all include paths.
    This is required because GTSAM 4.2.1 on this system was compiled against its bundled
    Eigen 3.7, and mixing Eigen 3 and Eigen 4 in the same translation unit causes ABI
    mismatches. See GTSAM section below.

### GTSAM 4.2.1 + TBB 2021.5 incompatibility

GTSAM 4.2.1 was installed from source with `GTSAM_DEFAULT_ALLOCATOR=TBB`, which causes
GTSAM's internal data structures (e.g., `VectorValues`, ISAM2's delta maps) to use
`tbb::concurrent_unordered_map` with `tbb_allocator`. TBB 2021 (oneAPI TBB) changed the
internal allocator behavior in a way that is incompatible with GTSAM 4.2.1's usage pattern,
causing heap corruption crashes.

Two distinct crash sites were encountered:

**Crash 1 — `ISAM2::update()` in `concurrent_unordered_map::internal_clear()`:**
- `double free or corruption` in `libc free()` called from TBB's internal map clear.
- Triggered on the first call to `isam2->update(*graph_new, values_new)`.

**Crash 2 — `CombinedImuFactor` constructor in `noiseModel::Gaussian::Covariance()`:**
- `double free or corruption (out)` in `libc free()` called from `Gaussian::Covariance`
  during LU decomposition of the preintegration covariance matrix.
- Triggered on the first call to `create_imu_factor()` after VIO initialization.

**Workaround applied:**

- **Changed** `src/GraphSolver.cpp`:
  - Disabled `isam2->update(*graph_new, values_new)` in `GraphSolver::optimize()`.
  - The function now only clears the new-factor/new-value buffers and resets IMU
    preintegration — no GTSAM factor graph optimization is performed.
  - **TODO**: Restore ISAM2-based optimization by recompiling GTSAM with
    `GTSAM_WITH_TBB=OFF` (disables TBB allocator) or upgrading to GTSAM ≥ 4.3 which
    contains fixes for the TBB 2021 allocator incompatibility. The GTSAM ISAM2 optimizer
    provides incremental smoothing with far better accuracy than dead reckoning and is the
    intended solver for this VIO system.

- **Changed** `src/GraphSolver.cpp` in `addmeasurement_uv`:
  - Replaced `create_imu_factor(timestamp, values_initial)` with a new
    `integrate_imu(timestamp)` call that performs only the preintegration step without
    constructing a `CombinedImuFactor` object.
  - The `CombinedImuFactor` object, and therefore the IMU factor in the GTSAM graph, is
    never added.
  - **TODO**: Restore the IMU factor (`CombinedImuFactor`) once GTSAM is recompiled without
    TBB (see above). The IMU factor provides kinematic constraints between consecutive
    states that significantly improve trajectory accuracy over pure dead reckoning.

- **Added** `src/GraphSolver_IMU.cpp` — new function `GraphSolver::integrate_imu(double)`:
  - Performs IMU preintegration up to `updatetime` identically to the first half of
    `create_imu_factor`, but without constructing the GTSAM factor object.

- **Net effect of the GTSAM workaround**: The VIO backend operates as IMU-only dead
  reckoning. State prediction uses `preint_gtsam->predict()` (IMU integration) but the
  resulting states are never refined by GTSAM factor graph optimization. Trajectory
  accuracy will drift over time proportional to IMU noise. Feature measurements from the
  `feature_tracker` are received but not used.
  - **TODO**: When GTSAM is rebuilt without TBB, re-enable both the IMU factor and the
    smart visual factors (`process_feat_smart`, currently commented out in
    `GraphSolver.cpp`) to restore the full VIO pipeline.

### Smart feature factors (SmartProjectionPoseFactor)
- **Changed** `src/GraphSolver.cpp` — `addmeasurement_uv`:
  - The call to `process_feat_smart(timestamp, leftids, leftuv)` is commented out.
  - This was disabled as an isolation step during the GTSAM crash investigation; the smart
    factors themselves were not the source of the crashes.
  - **TODO**: Re-enable `process_feat_smart` once the GTSAM/TBB issue is resolved. The
    smart projection factors are the visual component of VIO and are needed for
    loop-closure-capable trajectory estimation.

---

## Launch file

### ROS1 XML → ROS2 Python
- **Added** `launch/mvsec_test.launch.py`:
  - Full Python rewrite of `launch/mvsec_test.launch` (ROS1 XML format).
  - `<node pkg="..." type="..." name="...">` → `Node(package=..., executable=..., name=...)`.
  - `<param name="..." value="...">` / `<rosparam param="...">` →
    `parameters=[{key: value, ...}]`.
  - `<remap from="..." to="...">` → `remappings=[(from, to)]`.
  - `$(find package)/path` → `os.path.join(WS_ROOT, path)`.
  - Added `namespace='feature_tracker'` to the `feature_tracker` Node so its relative topic
    names resolve to `/feature_tracker/feature`, etc. (required for VIO remapping).
  - Added `static_transform_publisher` node publishing `world → odom` identity transform so
    the `world` frame exists in the TF tree. Without this, rviz displays "Frame [world] does
    not exist" and all frame-dependent displays are blank.
  - Added `rviz2` node with the converted rviz2 config.
  - **Note**: The original `launch/mvsec_test.launch` is preserved for reference.
  - **TODO**: The `rviz` node in the original ROS1 launch file has been replaced with
    `rviz2`. If rviz2 is not installed, the launch will fail at the rviz2 node. Install
    with `sudo apt-get install ros-humble-rviz2`.

---

## rviz configuration

### ROS1 → ROS2 format
- **Changed** `rviz/mvsec_test.rviz`:
  - All display plugin class names updated:
    `rviz/Grid` → `rviz_default_plugins/Grid`,
    `rviz/PointCloud2` → `rviz_default_plugins/PointCloud2`,
    `rviz/Path` → `rviz_default_plugins/Path`,
    `rviz/Image` → `rviz_default_plugins/Image`.
  - Panel class names updated:
    `rviz/Displays` → `rviz_common/Displays`, etc.
  - Tool class names updated similarly.
  - Topic configuration updated to ROS2 format with explicit QoS fields
    (`Reliability Policy`, `Durability Policy`, `History Policy`, `Depth`).
  - Removed ROS1-specific `QMainWindow State` binary blob (replaced with a clean state).
  - View camera updated to center on the trajectory area produced by the synthetic bag.

---

## Synthetic test bag

### New file: `generate_mvsec_test_bag.py`
- **Added**: Python script that generates a 60-second synthetic ROS2 bag in MVSEC
  VI-Sensor format (topics `/visensor/imu` at 200 Hz and `/visensor/left/image_raw` at
  25 Hz) to allow testing without the original `mvsec_test.bag` (no longer available at
  the original Google Drive link).
- The script simulates a camera driving a gentle arc at 2 m/s with 200 random 3D
  feature points projected into 752×480 grayscale images.
- Camera orientation fixed to point forward (along direction of motion) rather than
  straight up (the initial implementation pointed along the world +Z axis).
- Images use noise-base + bright rectangles to provide detectable corners for
  `cv::goodFeaturesToTrack`.
- **Note**: The generated bag starts at timestamp epoch 0 (not wall clock). Bag metadata
  `offered_qos_profiles` and `custom_data` fields are patched post-generation to use the
  string/map formats expected by `rosbag2`'s yaml-cpp parser (the `rosbags` library writes
  these in YAML sequence form which yaml-cpp rejects).
- **TODO**: Replace this synthetic bag with a real MVSEC outdoor driving sequence once
  one is downloaded. The MVSEC outdoor sequences use `/visensor/imu` and
  `/visensor/left/image_raw` topics directly and can be converted from ROS1 bag format
  with `rosbags-convert --src <downloaded.bag> --dst mvsec_test_ros2`. The synthetic bag
  does not reproduce real-world IMU dynamics and the VIO dead-reckoning trajectory will not
  resemble the expected result.

---

## System packages installed

The following packages were installed during the porting process and are required at runtime:

| Package | Reason |
|---------|--------|
| `ros-humble-cv-bridge` | Originally needed; later removed from `feature_tracker` due to OpenCV ABI conflict |
| `ros-humble-pcl-conversions` | Required by `gtsam_backend` for PointCloud2 publishing |
| `ros-humble-ros2launch` | Provides the `ros2 launch` CLI command |
| `ros-humble-desktop` | Full ROS2 desktop suite: `rviz2`, `ros2 bag`, `ros2 topic`, etc. |
| `python3-lark` (conda `ncsu-masters`) | Required by ROS2's `launch` Python package |
| `rosbags` (conda `ncsu-masters`) | Used by `generate_mvsec_test_bag.py` to write ROS2 bags |

---

## Summary of open TODOs

| Priority | Item |
|----------|------|
| High | Recompile GTSAM 4.2.1 with `-DGTSAM_WITH_TBB=OFF`, or upgrade to GTSAM ≥ 4.3, to restore ISAM2 optimization and `CombinedImuFactor` construction |
| High | Re-enable `process_feat_smart` in `GraphSolver.cpp` once GTSAM/TBB is resolved |
| High | Re-enable `create_imu_factor` (and add IMU factors to graph) once GTSAM/TBB is resolved |
| High | Download a real MVSEC outdoor bag and convert it with `rosbags-convert` |
| Medium | Restore `camera_model` shared library linking against Ceres once `libgoogle-glog-dev` ≥ 0.6 is installed |
| Medium | Restore `cv_bridge` in `feature_tracker` once the OpenCV version conflict is resolved |
| Medium | Fix cmake CMP0074 policy to allow proper `CUDAToolkit_ROOT` variable resolution in `camera_model` |
| Low | Restore `ROS_DEBUG` / `RCLCPP_DEBUG` logging in `feature_tracker.cpp` by threading a logger reference into `FeatureTracker` |
| Low | Re-add MKL if high-performance BLAS acceleration is needed for GTSAM |
| Low | Add message sequence tracking to replace the removed `header.seq` field in path publishing |
