cmake_minimum_required(VERSION 3.5)
project(nif_ghost_vehicle_spawner_nodes)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
        src/ghost_vehicle_spawner_node.cpp
)

ament_auto_add_executable(${PROJECT_NAME}_exe
        src/ghost_vehicle_spawner_main.cpp
)

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
    config
    launch
    )