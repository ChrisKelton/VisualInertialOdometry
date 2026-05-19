cmake_minimum_required(VERSION 3.10)
project(gtsam_backend)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(TBB REQUIRED CONFIG)

# Use GTSAM's bundled Eigen 3 to match the version GTSAM was compiled with
set(EIGEN_INCLUDE_DIR /usr/local/include/gtsam/3rdparty/Eigen)

include_directories(
    include/gtsam_backend
    ${EIGEN_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
)

add_executable(vio
    src/main_estimator.cpp
    src/GraphSolver.cpp
    src/GraphSolver_IMU.cpp
    src/GraphSolver_FEAT.cpp
)

ament_target_dependencies(vio
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    pcl_conversions
)

# Must come BEFORE the system Eigen4 added by GTSAM's find_dependency(Eigen3)
target_include_directories(vio BEFORE PRIVATE
    /usr/local/include/gtsam/3rdparty/Eigen
)

target_link_libraries(vio
    TBB::tbb
    TBB::tbbmalloc
    gtsam
    gtsam_unstable
    ${PCL_LIBRARIES}
)

install(TARGETS vio
    DESTINATION lib/${PROJECT_NAME})

ament_package()
