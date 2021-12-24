cmake_minimum_required(VERSION 3.5)

project(nif_radar_clustering_nodes)
# set(CMAKE_CXX_FLAGS "-W -Wall -O3 -std=c++11 -DNDEBUG")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fPIC" )
set( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -fPIC" )

find_package(PCL REQUIRED QUIET)
find_package(CUDA REQUIRED)
find_package(yaml-cpp REQUIRED)
# find_package(nif_gpu_euclidean_clustering REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Library
ament_auto_add_library(${PROJECT_NAME} SHARED
  # src/radar_euclidean_cluster_node.cpp
  src/aptiv_interface_node.cpp
  )

ament_auto_add_executable(aptiv_esr_interface_exe
  src/aptiv_esr_interface_main.cpp
  )

# target_compile_definitions(${PROJECT_NAME} PRIVATE
#     GPU_CLUSTERING=1
# )
# ament_target_dependencies(${PROJECT_NAME}
#   nif_gpu_euclidean_clustering
# )
# target_link_libraries(${PROJECT_NAME}
#   nif_gpu_euclidean_clustering
# )

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
        config)