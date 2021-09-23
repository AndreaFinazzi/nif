cmake_minimum_required(VERSION 3.5)
project(bvs_localization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(raptor_dbw_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(novatel_gps_msgs REQUIRED)
find_package(bvs_utils REQUIRED)


find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIR}
)

# Nodes
add_executable(localization_node src/localization_node.cpp)
target_link_libraries(localization_node)
ament_target_dependencies(localization_node rclcpp tf2 std_msgs nav_msgs geometry_msgs novatel_gps_msgs tf2_msgs novatel_oem7_msgs bvs_utils)

add_executable(localization_executive
  src/localization_executive.cpp
  src/estimate/estimate_merger.cpp
  src/estimate/estimate_source.cpp
  src/sources/source_factory.cpp
  src/sources/novatel_oem7_inspva.cpp
  src/sources/novatel_oem7_bestpos.cpp
  src/sources/novatel_oem7_bestvel.cpp
  src/sources/rear_wheel_velocity.cpp
  src/sources/front_wheel_velocity.cpp
)
ament_target_dependencies(localization_executive rclcpp tf2 std_msgs nav_msgs geometry_msgs novatel_gps_msgs tf2_msgs novatel_oem7_msgs raptor_dbw_msgs bvs_utils)


# Install Executables
install(
  TARGETS localization_node localization_executive
  DESTINATION lib/${PROJECT_NAME}
)

# Install config / launch files
install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
