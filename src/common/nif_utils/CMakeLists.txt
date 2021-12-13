cmake_minimum_required(VERSION 3.5)
project(nif_utils)

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
        src/utils.cpp
        src/PID.cpp
        src/quartic_polynomial.cpp
        src/quintic_polynomial.cpp
        src/cubic_spliner.cpp
        src/cubic_spliner_2D.cpp
        src/frenet_path.cpp
        src/frenet_path_generator.cpp
        src/geodetic_converter.cpp
        lib/Amathutils.cpp
        lib/butterworth_filter.cpp
        lib/kalman_filter.cpp
        lib/time_delay_kalman_filter.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test
          test/gtest_main.cpp
          test/test_utils_geometry.cpp
          test/test_utils_numeric.cpp
          test/test_utils_time.cpp
          test/test_utils_polygon_helper.cpp
          )

  target_include_directories(${PROJECT_NAME}_test PRIVATE "include")
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
#  ament_target_dependencies(${PROJECT_NAME}_test Opencv)
endif()

ament_auto_package()
