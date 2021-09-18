cmake_minimum_required(VERSION 3.5)
project(raceflag_input)

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
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(deep_orange_msgs REQUIRED)

add_executable(rcflaginput src/race_flag_input.cpp)
ament_target_dependencies(rcflaginput rclcpp std_msgs deep_orange_msgs)

install(TARGETS
  rcflaginput
  DESTINATION lib/${PROJECT_NAME})

ament_package()
