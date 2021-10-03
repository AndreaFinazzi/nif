cmake_minimum_required(VERSION 3.5)
project(nif_velocity_planning_node)

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
        src/nif_velocity_planning_node.cpp)

ament_auto_add_executable (${PROJECT_NAME}_exe
        src/nif_velocity_planning_main.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
        launch)

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(novatel_oem7_msgs REQUIRED)
# find_package(novatel_gps_msgs REQUIRED)
# find_package(deep_orange_msgs REQUIRED)
# find_package(raptor_dbw_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)

# find_package(nif_vehicle_dynamics_manager REQUIRED)

# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# find_package(yaml-cpp REQUIRED)

# include_directories(
#   include
#   ${YAML_CPP_INCLUDE_DIR}
# )

# # Nodes
# add_executable(nif_velocity_planning_node
#   src/nif_velocity_planning_node.cpp
# )
# # target_link_libraries(nif_velocity_planning_node lqr_control bvs_control_utils Eigen3::Eigen)
# ament_target_dependencies(nif_velocity_planning_node
#   tf2
#   rclcpp
#   std_msgs
#   nav_msgs
#   geometry_msgs
#   novatel_oem7_msgs
#   deep_orange_msgs
#   raptor_dbw_msgs
#   nif_vehicle_dynamics_manager
# )

# install(TARGETS
#   nif_velocity_planning_node
#   DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()

