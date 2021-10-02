cmake_minimum_required(VERSION 3.5)
project(nif_accel_control_nodes)

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
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Library
ament_auto_add_library(${PROJECT_NAME} SHARED
        src/accel_control_node.cpp
        src/control_model.cpp
        src/TractionABS.cpp)

ament_auto_add_executable (${PROJECT_NAME}_exe
        src/accel_control_node.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(raptor_dbw_msgs REQUIRED)
# find_package(deep_orange_msgs REQUIRED)
# find_package(bvs_msgs REQUIRED)
# find_package(vehicle_dynamics_manager REQUIRED)

# include_directories(include)

# add_executable(accel_controller src/accel_control_node.cpp src/control_model.cpp src/TractionABS.cpp)
# ament_target_dependencies(accel_controller rclcpp std_msgs raptor_dbw_msgs deep_orange_msgs bvs_msgs)
# install(
#   TARGETS accel_controller
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
