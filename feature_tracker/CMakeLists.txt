cmake_minimum_required(VERSION 3.5)
project(feature_tracker)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall )
endif()

include_directories("include")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(cv_bridge REQUIRED)  
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

find_package(camera_model REQUIRED)

#message(camera_model "test")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)



add_executable(feature_tracker src/feature_tracker_node.cpp src/feature_tracker.cpp src/parameters.cpp)
target_link_libraries(feature_tracker 
   camera_model::camera_model_lib
   ${Boost_LIBRARIES}
   )

find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})
find_package(Boost REQUIRED )

ament_target_dependencies(feature_tracker camera_model rclcpp std_msgs sensor_msgs OpenCV cv_bridge Boost  ${Boost_LIBRARIES})


install(TARGETS
  feature_tracker
  DESTINATION lib/${PROJECT_NAME})


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()

