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
find_package(Eigen3 REQUIRED NO_MODULE)
message(WARNING "Eigen version: ${Eigen3_VERSION}")
find_package(GTSAM REQUIRED)
message(WARNING "Eigen version: ${Eigen3_VERSION}")
#find_package(TBB REQUIRED CONFIG)

### Use GTSAM's bundled Eigen 3 to match the version GTSAM was compiled with
##set(EIGEN_INCLUDE_DIR /usr/local/include/gtsam/3rdparty/Eigen)
#find_package(Eigen3 CONFIG REQUIRED)
#set(EIGEN_INCLUDE_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)

include_directories(
    include/gtsam_backend
#    ${EIGEN_INCLUDE_DIR}
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
        Eigen3::Eigen
)

# Prevent weak TBB template symbols (e.g. concurrent_unordered_base::internal_clear)
# compiled into this executable from interposing on the same symbols in libgtsam.so.4.
# Without this, Linux's dynamic linker prefers the executable's weak definition, which
# can have different codegen than the libgtsam version and causes heap corruption when
# the two disagree on which allocator (scalable vs libc) owns a pointer.
target_compile_options(vio PRIVATE
    -fvisibility=hidden -fvisibility-inlines-hidden
#    # Match GTSAM's -march=native build so Eigen uses the same EIGEN_MAX_ALIGN_BYTES
#    # (32 bytes for AVX2 vs 16 bytes for SSE).  Without this, sizeof(Pose3) differs
#    # between gtsam_backend and libgtsam.so.4, causing GTSAM to read 16 bytes past
#    # the end of every Pose3 stored in gtsam::Values, corrupting the heap.
#    -march=native
)

install(TARGETS vio
    DESTINATION lib/${PROJECT_NAME})

ament_package()
