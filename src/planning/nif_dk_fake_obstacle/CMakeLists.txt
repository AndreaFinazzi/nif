cmake_minimum_required(VERSION 3.5)
project(nif_dk_fake_obstacle)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_path (PUGIXML_INCLUDE_DIR NAMES pugixml.hpp PATHS ${PUGIXML_HOME}/include)
find_library (PUGIXML_LIBRARY NAMES pugixml PATHS ${PUGIXML_HOME}/lib)
include (FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS (PugiXML DEFAULT_MSG PUGIXML_LIBRARY
PUGIXML_INCLUDE_DIR)

if (PUGIXML_FOUND)
  set (PUGIXML_LIBRARIES ${PUGIXML_LIBRARY})
  message (STATUS "PugiXML include = ${PUGIXML_INCLUDE_DIR}")
  message (STATUS "PugiXML library = ${PUGIXML_LIBRARY}")
else (PUGIXML_FOUND)
  message (STATUS "No PugiXML found")
endif(PUGIXML_FOUND)

mark_as_advanced (PUGIXML_LIBRARY PUGIXML_INCLUDE_DIR)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
  ${PUGIXML_INCLUDE_DIR}
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(geodetic_util SHARED
        lib/geodetic_conv.cpp)

ament_auto_add_library(${PROJECT_NAME} SHARED
        src/nif_dk_fake_obstacle_node.cpp)

target_link_libraries(${PROJECT_NAME}
  geodetic_util
  ${PUGIXML_LIBRARIES}
  ${PCL_LIBRARIES}
)

ament_auto_add_executable(${PROJECT_NAME}_exe
    src/nif_dk_fake_obstacle_node_main.cpp)
target_link_libraries(${PROJECT_NAME}_exe
  ${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # add_ros_test(
  #         test/planner_test.launch.py
  #         TIMEOUT "30"
  # )

  ament_add_gtest(${PROJECT_NAME}_test
          test/gtest_main.cpp
          )

  target_include_directories(${PROJECT_NAME}_test PRIVATE "include")
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  ament_target_dependencies(${PROJECT_NAME}_test "rclcpp")
endif()

ament_auto_package(INSTALL_TO_SHARE
        launch
        map
)
