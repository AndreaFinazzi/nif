cmake_minimum_required(VERSION 3.5)
project(ros2_luminar)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/../LuminLib-all-2.16.0/include
  ${PROJECT_SOURCE_DIR}/../LuminLib-all-2.16.0/LumNet-includes
  ${PROJECT_SOURCE_DIR}/../LuminLib-all-2.16.0/Control_Client/include
  ${PROJECT_SOURCE_DIR}/../LuminLib-all-2.16.0/Control_Client/include/external
  ${PROJECT_SOURCE_DIR}/../LuminLib-all-2.16.0/Data_Client/include
  ${PROJECT_SOURCE_DIR}/../LuminLib-all-2.16.0/Data_Client/include/external
)

set(executable_name luminar_driver)
set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  builtin_interfaces
)

add_library(${library_name} SHARED
  src/driver_types.cpp
  src/luminar_driver.cpp
  src/lifecycle_interface.cpp
  src/H3/H3_sensor.cpp
  src/H3/ControlClientHelper.cpp
  src/H3/DataClientHelper.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

find_library(LUMINLIB_LIBRARY NAMES LuminLib HINTS ${PROJECT_SOURCE_DIR}/../../../build/LuminLib ${PROJECT_SOURCE_DIR}/../LuminLib-all-2.16.0/build )

target_link_libraries(${library_name}
  ${LUMINLIB_LIBRARY}
)

add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name})

rclcpp_components_register_nodes(${library_name} "${PROJECT_NAME}::H3Driver")
set(node_plugins "${node_plugins}${PROJECT_NAME}::H3Driver;$<TARGET_FILE:luminar_driver>\n")

install(TARGETS ${executable_name} ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
