cmake_minimum_required(VERSION 3.5)
project(nif_opponent_prediction_nodes)

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
# ament_auto_add_library(${PROJECT_NAME} SHARED
        # src/opponent_prediction_main.cpp)

ament_auto_add_library(${PROJECT_NAME} SHARED
        src/cubic_spliner_2D.cc
        src/cubic_spliner.cc
        src/frenet_based_opponent_predictor.cc
        src/frenet_path_generator.cc
        src/frenet_path.cc
        src/quartic_polynomial.cc
        src/quaternion.cc
        src/quintic_polynomial.cc)

ament_auto_add_executable (${PROJECT_NAME}_exe
        src/opponent_prediction_main.cpp)

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