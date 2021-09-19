cmake_minimum_required(VERSION 3.5)
project(bvs_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(novatel_gps_msgs REQUIRED)
find_package(deep_orange_msgs REQUIRED)
find_package(raptor_dbw_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIR}
)

# Isolated logic that should reasonably not use ROS
add_library(lqr_control src/lqr/solver.cpp src/lqr/lateral_dynamics.cpp src/lqr/lateral_lqr.cpp)
target_link_libraries(lqr_control Eigen3::Eigen ${YAML_CPP_LIBRARIES})

# MISC Utils
add_library(bvs_control_utils src/utils/lateral_lqr_ros.cpp src/utils/pure_pursuit_tracker.cpp)
target_link_libraries(bvs_control_utils Eigen3::Eigen)
ament_target_dependencies(bvs_control_utils tf2 nav_msgs geometry_msgs)

# Nodes
add_executable(path_follower_node src/path_follower_node.cpp)
target_link_libraries(path_follower_node lqr_control bvs_control_utils Eigen3::Eigen)
ament_target_dependencies(path_follower_node tf2 rclcpp std_msgs nav_msgs geometry_msgs novatel_oem7_msgs deep_orange_msgs raptor_dbw_msgs)

install(
  TARGETS path_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

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
