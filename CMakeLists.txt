cmake_minimum_required(VERSION 2.8.3)
project(fw_ctrl)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fno-aggressive-loop-optimizations")

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs mavros message_generation grid_map_core grid_map_ros dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/fw_ctrl.cfg
  cfg/control.cfg
  cfg/manual_control.cfg
  cfg/occlusion_detection.cfg
  cfg/soft_constraints.cfg
  cfg/trajectory_generation.cfg
)

add_message_files(
  FILES
  NMPCAuxOut.msg
  NMPCControls.msg
  NMPCInfo.msg
  NMPCMeasurements.msg
  NMPCObjNRef.msg
  NMPCObjRef.msg
  NMPCOnlineData.msg
  NMPCStates.msg
  WindGroundTruth.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp std_msgs mavros message_runtime grid_map_core grid_map_ros
)

include_directories(
  include
  include/guidance
  include/common
  src
  src/acado_solver
  src/acado_solver/qpoases/INCLUDE
  src/acado_solver/qpoases/SRC
  ${catkin_INCLUDE_DIRS}
)

add_library(fw_nmpc_lib
  src/nonlinear_mpc.cpp
  src/nonlinear_mpc_objectives.cpp
  src/occlusion_detector.cpp
  src/trajectory_generator.cpp
  src/acado_solver/acado_auxiliary_functions.c
  src/acado_solver/acado_integrator.c
  src/acado_solver/acado_qpoases_interface.cpp
  src/acado_solver/acado_solver.c
  src/acado_solver/qpoases/SRC/Bounds.cpp
  src/acado_solver/qpoases/SRC/Constraints.cpp
  src/acado_solver/qpoases/SRC/CyclingManager.cpp
  src/acado_solver/qpoases/SRC/Indexlist.cpp
  src/acado_solver/qpoases/SRC/MessageHandling.cpp
  src/acado_solver/qpoases/SRC/QProblem.cpp
  src/acado_solver/qpoases/SRC/QProblemB.cpp
  src/acado_solver/qpoases/SRC/SubjectTo.cpp
  src/acado_solver/qpoases/SRC/Utils.cpp
  src/acado_solver/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  src/acado_solver/lsq_objective.c
  src/common/helpers.cpp
  src/common/huber_constraint.cpp
  src/guidance/npfg.cpp
  src/guidance/pwqg.cpp
)

add_executable(fw_nmpc
  src/nonlinear_mpc.cpp
)

add_dependencies(fw_nmpc ${PROJECT_NAME}_gencfg)

target_link_libraries(fw_nmpc
  fw_nmpc_lib
  ${catkin_LIBRARIES}
)
