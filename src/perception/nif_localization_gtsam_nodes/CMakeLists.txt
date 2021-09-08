cmake_minimum_required(VERSION 3.5)
project(nif_localization_gtsam_nodes)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# add_subdirectory(src/StateEstimator)

###################
# State estimator #
###################
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

find_package (GeographicLib REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# set(GTSAM_DIR "/home/sw/external/gtsam/build/")
find_package(GTSAM)
if(GTSAM_FOUND)
        find_package(TBB)
        include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} "/usr/local/include" "/home/sw/external/gtsam/build/")
        include_directories(${GTSAM_INCLUDE_DIR})

        # ament_auto_add_executable(state_estimator_node
        #         src/StateEstimator/StateEstimator.cpp)
        ament_auto_add_library(state_estimator SHARED
                src/StateEstimator/StateEstimator.cpp)

        target_link_libraries(state_estimator ${catkin_LIBRARIES} ${ROS_LIBRARIES} gtsam ${GeographicLib_LIBRARIES} ${TBB_LIBRARIES}  ${Boost_LIBRARIES})
endif()

# Library
ament_auto_add_library(${PROJECT_NAME} SHARED
        src/localization_node.cpp)

ament_auto_add_executable(${PROJECT_NAME}_exe
        src/localization_main.cpp)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_ros_test(
          test/localization_test.launch.py
          TIMEOUT "30"
  )

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
