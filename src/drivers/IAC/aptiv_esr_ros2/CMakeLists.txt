cmake_minimum_required(VERSION 3.5)
project(aptiv_esr_ros2)

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
find_package(can_msgs REQUIRED)
find_package(delphi_esr_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)

# add executable for driver
add_executable(esr_driver src/aptiv_esr_main.cpp)
ament_target_dependencies(esr_driver rclcpp std_msgs can_msgs delphi_esr_msgs visualization_msgs)

# add executable for vehicle interface 
add_executable(esr_vehicle_interface src/aptiv_esr_vehicle_interface.cpp)
ament_target_dependencies(esr_vehicle_interface rclcpp std_msgs delphi_esr_msgs novatel_oem7_msgs)


install(TARGETS esr_driver esr_vehicle_interface DESTINATION lib/${PROJECT_NAME})

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
