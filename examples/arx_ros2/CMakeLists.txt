cmake_minimum_required(VERSION 3.8)
project(arx_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(arm_control REQUIRED)
find_package(xr_msgs REQUIRED)
# 如有自定义消息包，需find_package(pos_cmd_msg REQUIRED)等

include(../cmake/eigen_system.cmake)
include(../cmake/placo.cmake)
include(../cmake/nlohmann_json.cmake)
include(../cmake/httplib.cmake)

include_directories(
  include
  src
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(main_v1
  src/main_v1.cpp
  src/xr_to_arx.cpp
)
ament_target_dependencies(main_v1
  rclcpp
  std_msgs
  tf2
  arm_control
  xr_msgs
)
install(TARGETS main_v1 DESTINATION lib/${PROJECT_NAME})

add_executable(main_v2
  src/main_v2.cpp
  src/xr_to_arx.cpp
)
target_link_libraries(main_v2
  lib_placo
)
ament_target_dependencies(main_v2
  rclcpp
  std_msgs
  tf2
  Eigen3
  arm_control
  xr_msgs
)
install(TARGETS main_v2 DESTINATION lib/${PROJECT_NAME})

add_executable(urdf_viz 
  src/urdf_viz.cpp
  src/UrdfVizClient.cc
)
target_link_libraries(urdf_viz
  nlohmann_json::nlohmann_json
  httplib::httplib
)
ament_target_dependencies(urdf_viz
  rclcpp
  std_msgs
  arm_control
)
install(TARGETS urdf_viz
  DESTINATION lib/${PROJECT_NAME})

ament_package() 
