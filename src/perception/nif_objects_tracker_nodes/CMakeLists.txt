cmake_minimum_required(VERSION 3.5)
project(nif_objects_tracker_nodes)

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
        src/objects_tracker_node.cpp
        src/imm/ukf.cpp
        src/imm/imm_ukf_pda.cpp
        )

ament_auto_add_executable(${PROJECT_NAME}_exe
  src/objects_tracker_main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} yaml-cpp)
ament_target_dependencies(${PROJECT_NAME} yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test
          test/gtest_main.cpp
          )

  target_include_directories(${PROJECT_NAME}_test PRIVATE "include")
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  ament_target_dependencies(${PROJECT_NAME}_test
          "rclcpp")
endif()

ament_auto_package(INSTALL_TO_SHARE
        launch
        config
        )
