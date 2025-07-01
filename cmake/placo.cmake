# Install:
# >> cd $HOME/code/github
# >> git clone https://github.com/Rhoban/placo/ 
# >> cd placo; bash scripts/requirements.sh
# >> runcmake
# >> source ~/.bashrc
#
# Usage:
# >> include(cmake/placo.cmake)
# >> "Then you can add `lib_placo` as link library to your target."

set(PLACO_HOME $ENV{HOME}/code/github/placo)
set(OPENROBOT_HOME /opt/openrobots)
message(STATUS "PLACO_HOME: ${PLACO_HOME}")
message(STATUS "OPENROBOT_HOME: ${OPENROBOT_HOME}")

include_directories(${OPENROBOT_HOME}/include)
include_directories(${PLACO_HOME}/src)
add_library(lib_placo STATIC IMPORTED)

# Detect system and set library suffix
if(${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
    set(PLACO_LIB_SUFFIX ".dylib")
    # Set RPATH settings for macOS
    set(CMAKE_SKIP_RPATH FALSE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    set(CMAKE_INSTALL_RPATH "@executable_path")
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

    # Find Python
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

    # Add Python include directories
    include_directories(${Python3_INCLUDE_DIRS})

    # Add Python library to the imported target
    set_target_properties(lib_placo PROPERTIES
        IMPORTED_LOCATION ${PLACO_HOME}/build/host/${PLACO_LIB_NAME}
        INTERFACE_LINK_LIBRARIES ${Python3_LIBRARIES}
    )
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    set(PLACO_LIB_SUFFIX ".so")
else()
    message(FATAL_ERROR "Unsupported system: ${CMAKE_SYSTEM_NAME}")
endif()

set(PLACO_LIB_NAME "liblibplaco${PLACO_LIB_SUFFIX}")

set_target_properties(lib_placo PROPERTIES IMPORTED_LOCATION 
    ${PLACO_HOME}/build/host/${PLACO_LIB_NAME}
    )

file(COPY ${PLACO_HOME}/build/host/${PLACO_LIB_NAME} DESTINATION ${CMAKE_BINARY_DIR})