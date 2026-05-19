cmake_minimum_required(VERSION 3.10)
project(camera_model)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

set(CUDAToolkit_ROOT /usr/local/cuda CACHE PATH "CUDA Toolkit root")
find_package(CUDAToolkit REQUIRED)
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# Use system Eigen4 for camera_model (Ceres 2.x provides its own Eigen compatibility layer)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories("include")

# Core camera model sources that do NOT depend on Ceres (used by feature_tracker)
set(CAMERA_MODEL_CORE_SRCS
    src/chessboard/Chessboard.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc)

# Full sources including Ceres-dependent calibration code (only for Calibration executable)
set(CAMERA_MODEL_CALIB_SRCS
    ${CAMERA_MODEL_CORE_SRCS}
    src/calib/CameraCalibration.cc
    src/camera_models/CostFunctionFactory.cc)

# Shared library with only core sources (no Ceres → no glog dependency)
add_library(camera_model SHARED ${CAMERA_MODEL_CORE_SRCS})
target_link_libraries(camera_model PRIVATE ${OpenCV_LIBS} ${Boost_LIBRARIES})

# Calibration executable uses full source set including Ceres-dependent code
add_executable(Calibration
    src/intrinsic_calib.cc
    ${CAMERA_MODEL_CALIB_SRCS})
target_link_libraries(Calibration PRIVATE
    ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}
    /usr/local/cuda/lib64/libcudart.so)

install(TARGETS camera_model Calibration
    EXPORT camera_modelTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
    DESTINATION include)

ament_export_targets(camera_modelTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(camera_model)
ament_export_dependencies(rclcpp std_msgs)

ament_package()
