#
# Add in your CMakeLists.txt
# >> include("cmake/nlohmann_json.cmake")
# >> target_link_libraries
#       ...
#       nlohmann_json::nlohmann_json
#    )
include(FetchContent)
FetchContent_Declare(
    json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    DOWNLOAD_EXTRACT_TIMESTAMP true
)
FetchContent_MakeAvailable(json)
