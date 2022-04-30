cmake_minimum_required(VERSION 3.5)
project(pose_graph_raspberry)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -O3 -g )
endif()
#set(CMAKE_BUILD_TYPE "Release")

include_directories("include")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(pose_graph_raspberry
    src/pose_graph_raspberry_node.cpp
    )

target_link_libraries(pose_graph_raspberry  ${OpenCV_LIBS} ${Boost_LIBRARIES})
# message("catkin_lib  ${catkin_LIBRARIES}")

find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})
find_package(Boost REQUIRED )


ament_target_dependencies(pose_graph_raspberry rclcpp std_msgs nav_msgs cv_bridge sensor_msgs OpenCV)
ament_export_dependencies(
        rclcpp
)

install(TARGETS
        pose_graph_raspberry
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
        )

ament_package()