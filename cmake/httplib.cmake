#
# Add in your CMakeLists.txt
# >> include("cmake/httplib.cmake")
# >> target_link_libraries
#       ...
#       httplib::httplib
#    )

include(FetchContent)
FetchContent_Declare(
    cpp-httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG v0.15.3
)
FetchContent_MakeAvailable(cpp-httplib)
