cmake_minimum_required(VERSION 3.1.2)
project(thesis_trajectory_generator)

set(FILESNAME TrajectoryGenerator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -pedantic #[[-Werror -Werror=vla]])

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mrs_lib
  nav_msgs
  mrs_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp mrs_lib 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  pugixml/src
)

add_library(${FILESNAME} src/${FILESNAME}.cpp src/Graph.cpp src/MapPolygon.cpp src/utils.cpp src/algorithms.cpp pugixml/src/pugixml.cpp src/EnergyCalculator.cpp src/ShortestPathCalculator.cpp include/ShortestPathCalculator.hpp include/custom_types.hpp include/mstp_solver/Target.h src/mstp_solver/TargetSet.cpp include/mstp_solver/TargetSet.h include/mstp_solver/SolverConfig.h src/mstp_solver/MtspSolver.cpp include/mstp_solver/MtspSolver.h include/mstp_solver/Insertion.h)

add_dependencies(${FILESNAME} ${${FILESNAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${FILESNAME} ${catkin_LIBRARIES})
