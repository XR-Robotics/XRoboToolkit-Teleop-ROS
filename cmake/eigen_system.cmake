# Usage:
# >> include(cmake/eigen_system.cmake)
# >> "Then you can add `Eigen3:Eigen` as link library to your target."

find_package(Eigen3 3 REQUIRED NO_MODULE)
# target_link_libraries(your_target Eigen3::Eigen)
