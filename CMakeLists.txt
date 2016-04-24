cmake_minimum_required(VERSION 2.8.3)
project(fw_ctrl)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fno-aggressive-loop-optimizations")

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs mavros
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp std_msgs mavros
)

include_directories(
  include
  src
  src/acado_solver
  src/acado_solver/qpoases/INCLUDE
  src/acado_solver/qpoases/SRC
  ${catkin_INCLUDE_DIRS}
)

add_library(fw_nmpc_lib
  src/fw_nmpc.cpp
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
)

add_library(fw_sim_lib
  src/fw_sim.cpp
)

add_executable(fw_nmpc
  src/fw_nmpc.cpp
)

add_executable(fw_sim
  src/fw_sim.cpp
)

target_link_libraries(fw_nmpc
  fw_nmpc_lib
  ${catkin_LIBRARIES} 
)

target_link_libraries(fw_sim
  fw_sim_lib
  ${catkin_LIBRARIES} 
)

