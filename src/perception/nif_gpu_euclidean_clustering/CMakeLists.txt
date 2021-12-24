cmake_minimum_required(VERSION 3.5)

project(nif_gpu_euclidean_clustering CUDA CXX)
# set(CMAKE_CXX_FLAGS "-W -Wall -O3 -std=c++11 -DNDEBUG")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# FLAGS for CUDA
set(CUDA_NVCC_FLAGS
  ${CUDA_NVCC_FLAGS};
  -O3 -lineinfo
  -gencode=arch=compute_30,code=sm_30
  -gencode=arch=compute_35,code=sm_35
  -gencode=arch=compute_50,code=sm_50
  -gencode=arch=compute_52,code=sm_52
  -gencode=arch=compute_61,code=sm_61
)


find_package(ament_cmake_auto REQUIRED)
find_package(CUDA REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

message("-- USING ACCELERATED CLUSTERING --")
message("Version: " ${CUDA_VERSION})
message("Library: " ${CUDA_CUDA_LIBRARY})
message("Runtime: " ${CUDA_CUDART_LIBRARY})

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/gpu_euclidean_clustering.cu
)

ament_auto_package()