cmake_minimum_required(VERSION 3.8)
project(picoxr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

cmake_host_system_information(RESULT ISA_NAME QUERY OS_PLATFORM)
message(STATUS "OS_PLATFORM: ${ISA_NAME}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xr_msgs REQUIRED)

# nlohmann/json
include(FetchContent)
FetchContent_Declare(
  json
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(json)

add_executable(talker src/publisher.cpp)

ament_target_dependencies(talker
  rclcpp
  std_msgs
  xr_msgs
)

if(NOT UNIX)
  message(FATAL_ERROR "Only support UNIX.") 
endif()

set(ROBOT_SDK "/opt/apps/roboticsservice/SDK")
if(NOT EXISTS ${ROBOT_SDK})
  message(FATAL_ERROR "robot sdk not found. Please install robot sdk first.") 
endif()
set(PICORobotSDK_INCLUDE ${ROBOT_SDK}/include)
set(PICORobotSDK_LIB_PATH ${ROBOT_SDK}/x64)
set(PICORobotSDK_LIB_NAME PXREARobotSDK)

# set(ROBOT_SDK "/opt/apps/picobusinesssuite/SDK/clientso/64")
# if(NOT EXISTS ${ROBOT_SDK})
#   message(FATAL_ERROR "robot sdk not found. Please install robot sdk first.") 
# endif()
# set(PICORobotSDK_INCLUDE ${ROBOT_SDK})
# set(PICORobotSDK_LIB_PATH ${ROBOT_SDK})
# set(PICORobotSDK_LIB_NAME PXREARobotSDK)

message(STATUS "PICORobotSDK_INCLUDE: ${PICORobotSDK_INCLUDE}") 
message(STATUS "PICORobotSDK_LIB_PATH: ${PICORobotSDK_LIB_PATH}") 
message(STATUS "PICORobotSDK_LIB_NAME: ${PICORobotSDK_LIB_NAME}") 

target_include_directories(talker PUBLIC ${PICORobotSDK_INCLUDE})

target_link_directories(talker PUBLIC ${PICORobotSDK_LIB_PATH})

target_link_libraries(talker
  ${PICORobotSDK_LIB_NAME}
  nlohmann_json::nlohmann_json
  )

install(TARGETS talker DESTINATION lib/${PROJECT_NAME})
install(FILES ${PICORobotSDK_LIB_PATH}/lib${PICORobotSDK_LIB_NAME}.so
        DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
