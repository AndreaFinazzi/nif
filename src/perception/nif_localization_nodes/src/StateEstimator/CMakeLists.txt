find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package (GeographicLib REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

find_package(GTSAM)
if(GTSAM_FOUND)
  find_package(TBB)
  include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} "/usr/local/include")
  include_directories(${GTSAM_INCLUDE_DIR})

  add_executable(StateEstimator StateEstimator.cpp)
  target_link_libraries(StateEstimator ${catkin_LIBRARIES} ${ROS_LIBRARIES} gtsam ${GeographicLib_LIBRARIES} ${TBB_LIBRARIES}  ${Boost_LIBRARIES})
  # add_dependencies(StateEstimator autorally_msgs_gencpp ${PROJECT_NAME}_gencfg)

  install(TARGETS StateEstimator
          ARCHIVE DESTINATION ${lib}
          LIBRARY DESTINATION ${lib}
          RUNTIME DESTINATION ${bin})
endif()
