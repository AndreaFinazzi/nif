cmake_minimum_required(VERSION 3.5)
project(bvs_long_control)

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
find_package(raptor_dbw_msgs REQUIRED)
find_package(deep_orange_msgs REQUIRED)
find_package(bvs_msgs REQUIRED)

include_directories(include)

add_executable(long_control src/long_control_node.cpp src/PID.cpp)
ament_target_dependencies(long_control rclcpp std_msgs raptor_dbw_msgs deep_orange_msgs bvs_msgs)
install(
  TARGETS long_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
