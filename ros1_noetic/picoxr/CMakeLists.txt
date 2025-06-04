cmake_minimum_required(VERSION 3.0.2)
project(picoxr)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  xr_msgs
)

# nlohmann/json
include(FetchContent)
FetchContent_Declare(
  json
  URL file:///home/arx/Downloads/json.tar.xz
)
FetchContent_MakeAvailable(json)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs xr_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(ROBOT_SDK "/opt/apps/picobusinesssuite/SDK/clientso/64")
if(NOT EXISTS ${ROBOT_SDK})
  message(FATAL_ERROR "robot sdk not found. Please install robot sdk first.")
endif()
set(PICORobotSDK_INCLUDE ${ROBOT_SDK})
set(PICORobotSDK_LIB_PATH ${ROBOT_SDK})
set(PICORobotSDK_LIB_NAME PXREARobotSDK)

add_executable(talker src/publisher.cpp)
add_dependencies(talker ${catkin_EXPORTED_TARGETS})

target_include_directories(talker PUBLIC ${PICORobotSDK_INCLUDE})
target_link_directories(talker PUBLIC ${PICORobotSDK_LIB_PATH})
target_link_libraries(talker
  ${PICORobotSDK_LIB_NAME}
  nlohmann_json::nlohmann_json
  ${catkin_LIBRARIES}
)

install(TARGETS talker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES ${PICORobotSDK_LIB_PATH}/lib${PICORobotSDK_LIB_NAME}.so
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
