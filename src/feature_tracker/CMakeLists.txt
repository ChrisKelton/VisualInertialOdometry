cmake_minimum_required(VERSION 3.10)
project(feature_tracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(camera_model REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${camera_model_INCLUDE_DIRS}
)

add_executable(feature_tracker
    src/feature_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
)

ament_target_dependencies(feature_tracker
    rclcpp
    std_msgs
    sensor_msgs
)

target_link_libraries(feature_tracker
    ${OpenCV_LIBS}
    camera_model::camera_model
)

install(TARGETS feature_tracker
    DESTINATION lib/${PROJECT_NAME})

ament_package()
