cmake_minimum_required(VERSION 3.5)
project(nif_points_preprocessor_nodes)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fPIC" )
set( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -fPIC" )

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
find_package(CUDA REQUIRED)
find_package(yaml-cpp REQUIRED)

ament_auto_find_build_dependencies()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
)

# Library
ament_auto_add_library(points_concat_filter SHARED
        src/points_concat_node.cpp)
target_link_libraries(points_concat_filter ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES})

ament_auto_add_library(points_concat_async_filter SHARED
        src/points_concat_async_node.cpp)
target_link_libraries(points_concat_async_filter ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES})


ament_auto_add_library(ego_shape_filter_node SHARED
        src/ego_shape_filter_node.cpp)
target_link_libraries(ego_shape_filter_node ${PCL_LIBRARIES})

ament_auto_add_library(wall_detection_node SHARED
        src/wall_detection_node.cpp)
target_link_libraries(wall_detection_node ${PCL_LIBRARIES})


message("-- USING ACCELERATED CLUSTERING --")
message("Version: " ${CUDA_VERSION})
message("Library: " ${CUDA_CUDA_LIBRARY})
message("Runtime: " ${CUDA_CUDART_LIBRARY})

target_compile_definitions(wall_detection_node PRIVATE
    GPU_CLUSTERING=1
)
cuda_add_library(gpu_euclidean_clustering
  src/gpu_euclidean_clustering.cu
)
target_link_libraries(wall_detection_node 
  gpu_euclidean_clustering
)

ament_auto_add_executable(${PROJECT_NAME}_exe
        src/points_preprocessor_main.cpp)

target_link_libraries(${PROJECT_NAME}_exe
        points_concat_filter
        points_concat_async_filter
        ego_shape_filter_node
        wall_detection_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test
          test/gtest_main.cpp
          )

  target_include_directories(${PROJECT_NAME}_test PRIVATE "include")
  target_link_libraries(${PROJECT_NAME}_test 
        points_concat_filter
        ego_shape_filter_node)
  ament_target_dependencies(${PROJECT_NAME}_test
          "rclcpp")
endif()

ament_auto_package(INSTALL_TO_SHARE
        config
        launch)