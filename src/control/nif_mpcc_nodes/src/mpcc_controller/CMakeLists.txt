cmake_minimum_required(VERSION 3.7)

# find_package(vrxperience_msgs REQUIRED)
# find_package(vrxperience_custom_msg REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories("/usr/include/python3.6m/")
include_directories("/usr/include/python2.7/")
set(Python_DIR "/usr/include/python2.7/")
include_directories(.)
include_directories(
  mpcc_controller/
  External/blasfeo/lib/include
  External/hpipm/lib/include
  External/matplotlib
  External/Eigen
  External/Json/include
  Params
  Spline
  Interfaces
  Constraints
  Cost
  Model
  MPC
  ../../include
  ../../lib
  ../../lib/iac_aero_manager
)

add_library(c_terrain_manager ../../lib/iac_terrain_manager/c_terrain_manager.cpp ../../lib/iac_terrain_manager/c_terrain_manager.h mpcc_controller/types.h mpcc_controller/Spline/arc_length_spline.h)
target_link_libraries(c_terrain_manager ${catkin_LIBRARIES})
add_library(c_model_manager ../../lib/iac_model_manager/c_model_manager.cpp ../../lib/iac_model_manager/c_model_manager.h)
target_link_libraries(c_model_manager ${catkin_LIBRARIES})
file(GLOB_RECURSE cc_files "../../lib/iac_aero_manager/*.cpp")
# file(GLOB cc_files_models "../../lib/iac_aero_manager/**/*.cpp")

add_library(c_aero_manager ${cc_files})
target_link_libraries(c_aero_manager ${catkin_LIBRARIES})

add_executable(mpcc_controller_node
  mpcc_controller_node.cpp
  mpcc_controller/types.cpp
  mpcc_controller/types.h
  mpcc_controller/config.h
  mpcc_controller/Params/params.cpp
  mpcc_controller/Spline/cubic_spline.cpp
  mpcc_controller/Spline/arc_length_spline.cpp
  mpcc_controller/Interfaces/hpipm_interface.cpp
  mpcc_controller/Interfaces/solver_interface.cpp
  mpcc_controller/Constraints/constraints.cpp
  mpcc_controller/Constraints/bounds.cpp
  mpcc_controller/Cost/cost.cpp
  mpcc_controller/Model/model.cpp
  mpcc_controller/Model/integrator.cpp
  # mpcc_controller/Tests/spline_test.cpp
  # mpcc_controller/Tests/cost_test.cpp
  # mpcc_controller/Tests/model_integrator_test.cpp
  # mpcc_controller/Tests/constratins_test.cpp
  mpcc_controller/MPC/mpc.cpp
  mpcc_controller/Params/track.cpp
  mpcc_controller/Params/track.h
  # mpcc_controller/Plotting/plotting.cpp
  # mpcc_controller/Plotting/plotting.h
)

target_link_libraries(mpcc_controller_node
c_terrain_manager
c_model_manager
c_aero_manager
${CMAKE_CURRENT_SOURCE_DIR}/External/hpipm/lib/lib/libhpipm.a
${CMAKE_CURRENT_SOURCE_DIR}/External/blasfeo/lib/lib/libblasfeo.a m)

target_include_directories(mpcc_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
ament_target_dependencies(mpcc_controller_node
  rclcpp
  std_msgs
  "geometry_msgs"
  "nav_msgs"
  "ackermann_msgs"
  # vrxperience_msgs
  # vrxperience_custom_msg
  visualization_msgs
  raptor_dbw_msgs
  ament_index_cpp
  nif_common_nodes
)

install(TARGETS mpcc_controller_node
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)