cmake_minimum_required(VERSION 3.5)
project(bvs_safety)

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
find_package(std_srvs REQUIRED)
find_package(deep_orange_msgs REQUIRED)
find_package(raptor_dbw_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(bvs_msgs REQUIRED)

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

add_executable(safety_executive_node src/safety_executive_node.cpp)
ament_target_dependencies(safety_executive_node rclcpp std_msgs nav_msgs std_srvs deep_orange_msgs novatel_oem7_msgs bvs_msgs)

install(TARGETS
  safety_executive_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
