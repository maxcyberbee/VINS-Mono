cmake_minimum_required(VERSION 3.5)
project(feature_tracker)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4.0.0 REQUIRED)
set(camera_model /home/ubuntu/dev_ws/src/VINS-Mono/camera_model)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

add_executable(feature_tracker src/feature_tracker_node.cpp)
target_link_libraries(feature_tracker camera_model)

ament_target_dependencies(feature_tracker rclcpp std_msgs sensor_msgs OpenCV cv_bridge)

install(TARGETS
feature_tracker
DESTINATION lib/${PROJECT_NAME})

ament_package()