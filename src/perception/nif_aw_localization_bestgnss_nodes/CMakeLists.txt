cmake_minimum_required(VERSION 3.5)
project(nif_aw_localization_bestgnss_nodes)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PCL REQUIRED)

find_path(
  GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

include_directories(
  include
  ${GeographicLib_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${GeographicLib_DEFINITIONS})

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
ament_auto_find_build_dependencies()

# Library
ament_auto_add_library(aw_geodetic_conv SHARED
        lib/geodetic_conv.cpp)

ament_auto_add_library(amathuutils_lib SHARED
        lib/Amathutils.cpp
        lib/butterworth_filter.cpp
        lib/kalman_filter.cpp
        lib/time_delay_kalman_filter.cpp)

ament_auto_add_library(nif_aw_localization_node SHARED
        src/nif_aw_localization_node.cpp)

target_link_libraries(nif_aw_localization_node
        amathuutils_lib
        aw_geodetic_conv
)

ament_auto_add_executable(${PROJECT_NAME}_exe
        src/nif_aw_localization_nodes_main.cpp)
target_link_libraries(${PROJECT_NAME}_exe
        nif_aw_localization_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

#   add_ros_test(
#           test/nif_aw_localization_nodes.launch.py
#           TIMEOUT "30"
#   )
  ament_add_gtest(${PROJECT_NAME}_test
          test/gtest_main.cpp
          )

  target_include_directories(${PROJECT_NAME}_test PRIVATE "include")
  target_link_libraries(${PROJECT_NAME}_test amathuutils_lib)
  ament_target_dependencies(${PROJECT_NAME}_test
          "rclcpp")
endif()

ament_auto_package(INSTALL_TO_SHARE
        launch
        config)
