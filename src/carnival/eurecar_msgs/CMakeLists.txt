cmake_minimum_required(VERSION 3.5)
project(eurecar_msgs)

# find_package(catkin REQUIRED COMPONENTS
# ament_cmake REQUIRED
# genmsg
# std_msgs
# sensor_msgs
# geometry_msgs
# diagnostic_msgs
# vision_msgs
# nav_msgs
# )
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


# find_package(builtin_interfaces REQUIRED)
# find_package(rosidl_default_generators REQUIRED)


set(msg_files
  "msg/ControlCmd.msg"
  "msg/FrenetPath.msg"
  "msg/PathWTime.msg"
  "msg/PathWTimeArray.msg"
  "msg/PathWithCost.msg"
  "msg/PathWithCostArray.msg"
  "msg/CanInput.msg"
  "msg/VehicleSysControl.msg"
  "msg/NavigationEvaluation.msg"
)

# add_message_files(
#   DIRECTORY msg
#   FILES
#   trafficLightStatus.msg
#   px2ObjDetection.msg
#   px2LaneDetection.msg
#   px2SingleLaneDetection.msg
#   px2DetectionResult.msg
#   controlCMD.msg
#   frenetPath.msg
#   PathWTime.msg
#   PathWTimeArray.msg
#   PathWithCost.msg
#   PathWithCostArray.msg
#   CanInput.msg
#   VehicleSysControl.msg
#   NavigationEvaluation.msg
# )

# generate_messages(DEPENDENCIES
#   std_msgs
#   sensor_msgs
#   geometry_msgs
#   diagnostic_msgs
#   vision_msgs
#   nav_msgs
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs geometry_msgs diagnostic_msgs vision_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
