cmake_minimum_required(VERSION 3.5)
project(bvs_utils)

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
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(rosbag2_cpp REQUIRED)
# find_package(rcpputils REQUIRED)
# find_package(rcutils REQUIRED)
# find_package(test_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
  include/
)

# Create Libraries
add_library(bvs_geodetic_conv src/bvs_utils/geodetic_conv.cpp)
target_link_libraries(bvs_geodetic_conv Eigen3::Eigen)

add_library(bvs_path_util src/bvs_utils/path_util.cpp)
target_link_libraries(bvs_path_util Eigen3::Eigen bvs_geodetic_conv)
ament_target_dependencies(bvs_path_util nav_msgs geometry_msgs)

# Make some executables!
add_executable(path_publisher_node src/path_publisher_node.cpp)
target_link_libraries(path_publisher_node bvs_path_util bvs_geodetic_conv)
ament_target_dependencies(path_publisher_node rclcpp std_msgs nav_msgs geometry_msgs)

add_executable(path_converter src/path_converter.cpp)
target_link_libraries(path_converter Eigen3::Eigen bvs_geodetic_conv)
ament_target_dependencies(path_converter rclcpp geometry_msgs nav_msgs)

# add_executable(rosbag_merger src/rosbag_merger.cpp)
# ament_target_dependencies(rosbag_merger rclcpp rosbag2_cpp rcpputils rcutils test_msgs)

# Install Exported Libraries
ament_export_targets(export_bvs_geodetic_conv HAS_LIBRARY_TARGET)
ament_export_targets(export_bvs_path_util HAS_LIBRARY_TARGET)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS bvs_geodetic_conv
  EXPORT export_bvs_geodetic_conv
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  TARGETS bvs_path_util
  EXPORT export_bvs_path_util
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_include_directories(include)
ament_export_libraries(bvs_geodetic_conv bvs_path_util)

# Install Executables
install(
  TARGETS path_publisher_node path_converter
  DESTINATION lib/${PROJECT_NAME}
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
