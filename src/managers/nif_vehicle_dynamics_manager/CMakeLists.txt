cmake_minimum_required(VERSION 3.5)
project(nif_vehicle_dynamics_manager)

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
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(ament_index_cpp REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Include directories (header)
include_directories(
  include
)

ament_export_dependencies(ament_index_cpp)

# Library
ament_auto_add_library(${PROJECT_NAME} SHARED
        src/tire_manager.cpp
        src/kalman.cpp
        # src/pid_node.cpp
)

# # Add executable
# add_executable(tire_manager_node
#   src/tire_manager.cpp
# )

# ament_target_dependencies(tire_manager_node
#   rclcpp
#   std_msgs
#   sensor_msgs
#   nav_msgs
#   geometry_msgs
#   Eigen3
#   ament_index_cpp
# )

# # Add install
# install(
#   TARGETS tire_manager_node
#   DESTINATION lib/${PROJECT_NAME}
#   )
# Copy data directory in 'dev_ws/install/nif_vehicle_dynamics_manager'
install(
  DIRECTORY data
  DESTINATION .
)
# ament_package()

ament_auto_package()
