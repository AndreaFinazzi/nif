cmake_minimum_required(VERSION 3.5)
project(nif_adaptive_cruise_control_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_index_cpp REQUIRED)
ament_auto_find_build_dependencies()

# Library
ament_auto_add_library(${PROJECT_NAME} SHARED
        include/idm_acc_lib/c++/IDM.cpp
        src/adaptive_cruise_node.cpp
        src/adaptive_cruise_node_main.cpp
        )

ament_auto_add_executable(${PROJECT_NAME}_exe
  src/adaptive_cruise_node_main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} yaml-cpp)
ament_target_dependencies(${PROJECT_NAME} yaml-cpp)

ament_export_dependencies(ament_index_cpp)


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   ament_add_gtest(${PROJECT_NAME}_test
#           test/gtest_main.cpp
#           )

#   target_include_directories(${PROJECT_NAME}_test PRIVATE "include")
#   target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
#   ament_target_dependencies(${PROJECT_NAME}_test
#           "rclcpp")
# endif()

ament_auto_package(INSTALL_TO_SHARE
        launch
        config
        )
