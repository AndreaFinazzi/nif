cmake_minimum_required(VERSION 3.5)
project(userinput)

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


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(deep_orange_msgs REQUIRED)

add_executable(ui_velocity src/vel_input.cpp)
add_executable(ui_pt_gains src/pt_test_input.cpp)
add_executable(ui_raceflag src/race_flag_input.cpp)
add_executable(ui_publisher src/publisher.cpp)

ament_target_dependencies(ui_velocity rclcpp std_msgs)
ament_target_dependencies(ui_pt_gains rclcpp std_msgs)
ament_target_dependencies(ui_raceflag rclcpp std_msgs deep_orange_msgs)
ament_target_dependencies(ui_publisher rclcpp std_msgs deep_orange_msgs)

install(TARGETS
  ui_velocity
  ui_pt_gains
  ui_raceflag
  ui_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
