cmake_minimum_required(VERSION 3.10)
project(novatel_oem7_driver)


## Compile as C++14, supported on ROS2
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(pluginlib)
find_package(rclcpp REQUIRED) 
find_package(rclcpp_components REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(rosidl_default_generators  REQUIRED)
find_package(Boost  			REQUIRED)


# Make package available as a macro to C++
add_definitions("-D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\"")

###########
## Build ##
###########

include_directories(include
    "novatel_oem7/include"
    src
)

## All components are plugins
add_library(${PROJECT_NAME} SHARED
   src/oem7_receiver_net.cpp
   src/oem7_receiver_port.cpp
   src/oem7_receiver_file.cpp
   src/oem7_message_decoder.cpp
   src/oem7_message_util.cpp
   src/oem7_ros_messages.cpp
   src/oem7_debug_file.cpp
   src/bestpos_handler.cpp
   src/ins_handler.cpp
   src/align_handler.cpp
   src/time_handler.cpp
   src/receiverstatus_handler.cpp
   src/nmea_handler.cpp
)

## All components are plugins
add_executable(${PROJECT_NAME}_exe
   src/oem7_message_nodelet.cpp
   src/oem7_message_util.cpp
   src/oem7_ros_messages.cpp
   src/message_handler.cpp
) 


# Link with correct decoder library
message(STATUS "System Host Proc: '${CMAKE_HOST_SYSTEM_PROCESSOR}'")
message(STATUS "System Proc: '${CMAKE_SYSTEM_PROCESSOR}'")
message(STATUS "VOID_P: ${CMAKE_SIZEOF_VOID_P}")

if (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
        FIND_LIBRARY(OEM7_DECODER_LIB liboem7decoder.a "novatel_oem7/bin/arm64v8") 
    else ()
        FIND_LIBRARY(OEM7_DECODER_LIB liboem7decoder.a "novatel_oem7/bin/arm32v7")
    endif ()
else ()
    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
        FIND_LIBRARY(OEM7_DECODER_LIB liboem7decoder.a "novatel_oem7/bin/amd64") 
    else ()
        FIND_LIBRARY(OEM7_DECODER_LIB liboem7decoder.a "novatel_oem7/bin/i386")
    endif ()
endif ()

message(STATUS "Linking to Oem7 Decoder at: '${OEM7_DECODER_LIB}'")

target_link_libraries(${PROJECT_NAME}
   Boost::boost
   ${OEM7_DECODER_LIB}
)

ament_target_dependencies(${PROJECT_NAME} 
	pluginlib
	std_msgs 
	gps_msgs 
	sensor_msgs 
	nmea_msgs 	
	nav_msgs
	tf2_geometry_msgs 
	novatel_oem7_msgs 
)


ament_target_dependencies(${PROJECT_NAME}_exe 
    rclcpp
	pluginlib
	novatel_oem7_msgs 
)



#rclcpp_components_register_nodes(${PROJECT_NAME} "novatel_oem7_driver::Oem7MessageNodelet")
#rclcpp_components_register_nodes(${PROJECT_NAME} "novatel_oem7_driver::Oem7ConfigNodelet")
#rclcpp_components_register_nodes(${PROJECT_NAME} "novatel_oem7_driver::Oem7LogNodelet")


#add_executable(novatel_oem7_driver_exe src/novatel_oem7_driver.cpp)

#ament_target_dependencies(novatel_oem7_driver_exe
#  rclcpp
#  pluginlib
#)

install(TARGETS
  ${PROJECT_NAME}_exe
  DESTINATION lib/${PROJECT_NAME}
)


#############
## Install ##
#############

ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(${PROJECT_NAME} novatel_oem7_driver_plugins.xml)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} 
   ARCHIVE DESTINATION lib 
   LIBRARY DESTINATION lib 
   RUNTIME DESTINATION bin} 
)

install(FILES ${PROJECT_NAME}_plugins.xml DESTINATION share)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.yaml")
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.py")
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.launch")

#install(DIRECTORY include/${PROJECT_NAME}/
#	DESTINATION share/${PROJECT_NAME} 
#	FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
#)

ament_package()


#############
## Testing ##
#############

# Built-in Self Test (BIST)
#catkin_install_python(PROGRAMS test/oem7_bist.py test/oem7_message_test.py
# 	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# Instaleld into SHARED vs BINARY. BINARY does not allow BIST to be run using
# rostest novatel_oem7_driver your_launch_file.lauch" syntax.


#if (CATKIN_ENABLE_TESTING)
#	find_package(rostest REQUIRED)
#	add_rostest(test/align.test)
#	add_rostest(test/bestpos.test)
#	add_rostest(test/ins1.test)
#	add_rostest(test/ins2.test)
#	add_rostest(test/rxstatus.test)
#	add_rostest(test/time.test)
#endif()


#message("includes:")
#get_property(incdirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#foreach(dir ${incdirs})
#   message(STATUS "incdir= '${dir}'")
#endforeach()
