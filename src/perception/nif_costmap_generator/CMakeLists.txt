cmake_minimum_required(VERSION 3.5)

project(nif_costmap_generator)
# set(CMAKE_CXX_FLAGS "-W -Wall -O3 -std=c++11 -DNDEBUG")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PCL REQUIRED QUIET)


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
# find dependencies
find_package(ament_cmake_auto REQUIRED)
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
ament_auto_find_build_dependencies()


# Library
ament_auto_add_library(nif_costmap_generator SHARED
    src/costmap_generator.cpp
    src/points_to_costmap.cpp
    # src/lane_to_costmap.cpp
    # src/laserscan_to_costmap.cpp
    # src/bboxes_to_costmap.cpp
    # src/visual_to_costmap.cpp 
)

target_link_libraries(nif_costmap_generator ${PCL_LIBRARIES})

add_executable(${PROJECT_NAME}_exe 
  src/costmap_generator_main.cpp
)
target_link_libraries(${PROJECT_NAME}_exe 
  nif_costmap_generator
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # add_ros_test(
  #         test/localization_test.launch.py
  #         TIMEOUT "30"
  # )
  ament_add_gtest(${PROJECT_NAME}_test
          test/gtest_main.cpp
          )

  target_include_directories(${PROJECT_NAME}_test PRIVATE "include")
  target_link_libraries(${PROJECT_NAME}_test nif_costmap_generator)
  ament_target_dependencies(${PROJECT_NAME}_test
          "rclcpp")
endif()

ament_auto_package(INSTALL_TO_SHARE
        launch)