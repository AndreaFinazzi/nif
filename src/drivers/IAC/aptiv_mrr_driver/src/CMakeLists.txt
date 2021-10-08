find_package(delphi_mrr_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(visualization_msgs REQUIRED)

# ===================== TK COMMON ===================================
add_library(tkCommon STATIC
  tkCommon/CanInterface.cpp
)
set_property(TARGET tkCommon PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(tkCommon
  PUBLIC
    ../include/
)

# ===================== mrr_pcan_core_lib ===================================
add_library(mrr_pcan_core_lib STATIC
  mrr_pcan_core_v07_02.c
)
set_property(TARGET mrr_pcan_core_lib PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(mrr_pcan_core_lib
  PUBLIC
    ../include/
)

# ===================== aptiv_mrr_driver_component ===================================
add_library(aptiv_mrr_driver_component SHARED
  driver_node.cpp
  can_serialization.cpp)
target_include_directories(aptiv_mrr_driver_component
  PUBLIC
    "../include"
)
ament_target_dependencies(aptiv_mrr_driver_component
  rclcpp
  delphi_mrr_msgs
  visualization_msgs
  rclcpp_components)
target_link_libraries(aptiv_mrr_driver_component
  tkCommon
  project_options
  project_warnings
  mrr_pcan_core_lib
)
rclcpp_components_register_nodes(aptiv_mrr_driver_component
  "aptiv_mrr_driver::DriverNode")

# ===================== add_executable(aptiv_mrr_driver_node ===================================
add_executable(aptiv_mrr_driver_node
  main.cpp
  driver_node.cpp
  can_serialization.cpp)
target_include_directories(aptiv_mrr_driver_node
  PUBLIC
    "../include"
)
ament_target_dependencies(aptiv_mrr_driver_node
  rclcpp
  delphi_mrr_msgs
  visualization_msgs
  rclcpp_components)
target_link_libraries(aptiv_mrr_driver_node
  tkCommon
  project_options
  project_warnings
  mrr_pcan_core_lib
)


install(TARGETS aptiv_mrr_driver_component aptiv_mrr_driver_node
  DESTINATION lib/${PROJECT_NAME})