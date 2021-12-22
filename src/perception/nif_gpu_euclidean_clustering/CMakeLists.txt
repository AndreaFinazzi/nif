cmake_minimum_required(VERSION 3.5)

project(nif_gpu_euclidean_clustering)
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

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

message("-- USING ACCELERATED CLUSTERING --")
message("Version: " ${CUDA_VERSION})
message("Library: " ${CUDA_CUDA_LIBRARY})
message("Runtime: " ${CUDA_CUDART_LIBRARY})

cuda_add_library(${PROJECT_NAME}
  src/gpu_euclidean_clustering.cu
)

ament_auto_package()