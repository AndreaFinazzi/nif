cmake_minimum_required(VERSION 3.5.0)
project(avt_vimba_camera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# [neil-rti]: set to directory of Vimba SDK 
# set(VIMBA_DIR Vimba_4_2)
set(VIMBA_DIR /opt/Vimba_4_2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_calibration_parsers REQUIRED)

###########
## Build ##
###########
include_directories(include ${VIMBA_DIR})

set(SOURCES_AVT_CPP
  ${VIMBA_DIR}/VimbaCPP/Source/AncillaryData.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/BaseFeature.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/BasicLockable.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/BoolFeature.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/Camera.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/Clock.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/CommandFeature.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/Condition.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/ConditionHelper.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/DefaultCameraFactory.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/EnumEntry.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/EnumFeature.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/FeatureContainer.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/Feature.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/FileLogger.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/FloatFeature.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/Frame.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/FrameHandler.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/Interface.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/IntFeature.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/Mutex.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/MutexGuard.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/RawFeature.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/Semaphore.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/StringFeature.cpp
  ${VIMBA_DIR}/VimbaCPP/Source/VimbaSystem.cpp
)

function(add_dependencies_and_linkings arg)
  ament_target_dependencies(${arg}
    "rclcpp"
    "rclcpp_components"
    "sensor_msgs"
    "std_msgs"
    "camera_info_manager"
    "image_transport"
    "camera_calibration_parsers")

  if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL i386)
    link_directories(${VIMBA_DIR}/VimbaCPP/DynamicLib/x86_32bit)
    target_link_libraries(${arg}
      VimbaC
      VimbaCPP
    )
  elseif("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL x86_64)
    link_directories(${VIMBA_DIR}/VimbaCPP/DynamicLib/x86_64bit)
    target_link_libraries(${arg}
      VimbaC
      VimbaCPP
    )
  else()
    message(FATAL_ERROR "[libvimba]: Architecture (${CMAKE_SYSTEM_PROCESSOR}) not suported. Exiting...")
  endif()
endfunction(add_dependencies_and_linkings)

link_directories(${VIMBA_DIR}/VimbaCPP/DynamicLib/x86_64bit)

add_executable(mono_camera_node
  src/nodes/mono_camera_node.cpp
  src/mono_camera.cpp
  src/avt_vimba_camera.cpp
  src/frame_observer.cpp
  ${SOURCES_AVT_CPP}
)
add_dependencies_and_linkings(mono_camera_node)

install(TARGETS mono_camera_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  set(ament_cmake_copyright_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#
#  find_package(ament_cmake_gtest REQUIRED)
#  add_subdirectory(test)
#endif()

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
