cmake_minimum_required(VERSION 3.5)
project(nif_mpcc_nodes)

#Default to C99
if(NOT CMAKE_C_STANDARD)
   set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options(-Wshadow -Werror)
endif()

set(CMAKE_CXX_FLAGS "-W -Wall -O3 -std=c++17")
# set(CMAKE_CXX_FLAGS "-W -Wall -O3 -std=c++17 -DNDEBUG")

# find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(ackermann_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(rviz_visual_tools REQUIRED)
# find_package(raptor_dbw_msgs REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_subdirectory(src/mpcc_controller)

# ament_auto_add_executable (${PROJECT_NAME}_exe
#         src/mpcc_controller/mpcc_controller_node.cpp)

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

ament_auto_package(INSTALL_TO_SHARE
  src/mpcc_controller/mpcc_controller/Params
  launch)
