cmake_minimum_required(VERSION 3.5)

project(nif_points_clustering)
# set(CMAKE_CXX_FLAGS "-W -Wall -O3 -std=c++11 -DNDEBUG")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PCL REQUIRED QUIET)
find_package(CUDA REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


ament_auto_add_executable(${PROJECT_NAME}_exe
  src/lidar_euclidean_cluster_detect.cpp)


if(WITH_CUDA)
  message("-- USING ACCELERATED CLUSTERING --")
  message("Version: " ${CUDA_VERSION})
  message("Library: " ${CUDA_CUDA_LIBRARY})
  message("Runtime: " ${CUDA_CUDART_LIBRARY})

  target_compile_definitions(${PROJECT_NAME}_exe PRIVATE
      GPU_CLUSTERING=1
  )

  cuda_add_library(gpu_euclidean_clustering
    include/nif_points_clustering/gpu_euclidean_clustering.h
    src/gpu_euclidean_clustering.cu
  )

  target_link_libraries(${PROJECT_NAME}_exe
    gpu_euclidean_clustering
    ${PCL_LIBRARIES}
  )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    ament_add_gtest(${PROJECT_NAME}_test
            test/gtest_main.cpp
            )

    target_include_directories(${PROJECT_NAME}_test PRIVATE "include")
    target_link_libraries(${PROJECT_NAME}_test gpu_euclidean_clustering)
    ament_target_dependencies(${PROJECT_NAME}_test
            "rclcpp")
  endif()

endif()


ament_auto_package(INSTALL_TO_SHARE
        launch
        config)