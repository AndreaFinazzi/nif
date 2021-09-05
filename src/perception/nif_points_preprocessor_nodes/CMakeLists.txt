cmake_minimum_required(VERSION 3.5)
project(nif_points_preprocessor_nodes)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
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

ament_auto_add_library(ego_shape_filter_node SHARED
        src/ego_shape_filter_node.cpp)
target_link_libraries(ego_shape_filter_node ${PCL_LIBRARIES})

ament_auto_add_executable(${PROJECT_NAME}_exe
        src/points_preprocessor_main.cpp)

target_link_libraries(${PROJECT_NAME}_exe
        points_concat_filter
        ego_shape_filter_node
)


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   ament_add_gtest(${PROJECT_NAME}_test
#           test/gtest_main.cpp
#         )

#   target_include_directories(${PROJECT_NAME}_test PRIVATE "include")
#   target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
#   ament_target_dependencies(${PROJECT_NAME}_test
#           "rclcpp")
# endif()

ament_auto_package()
