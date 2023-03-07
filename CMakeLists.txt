cmake_minimum_required(VERSION 3.1.2)
project(thesis_path_generator)

set(FILESNAME PathGenerator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra) #-pedantic -Werror -Werror=vla)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  mrs_lib
  nav_msgs
  mrs_msgs
)

add_service_files (FILES GeneratePaths.srv CalculateEnergy.srv)

find_package(OpenCV REQUIRED)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        mrs_msgs
        nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp mrs_lib message_runtime std_msgs geometry_msgs
)

include_directories(include
  #      /home/mrs/workspace/devel/.private/thesis_path_generator/include   # Clion cannot find headers without this for some reason. TODO: remove this
  ${catkin_INCLUDE_DIRS}
)



add_library(${FILESNAME} src/${FILESNAME}.cpp src/MapPolygon.cpp include/MapPolygon.hpp src/utils.cpp src/algorithms.cpp src/EnergyCalculator.cpp src/ShortestPathCalculator.cpp include/ShortestPathCalculator.hpp include/custom_types.hpp include/mstsp_solver/Target.h src/mstsp_solver/TargetSet.cpp include/mstsp_solver/TargetSet.h include/mstsp_solver/SolverConfig.h src/mstsp_solver/MstspSolver.cpp include/mstsp_solver/MstspSolver.h include/mstsp_solver/Insertion.h include/SimpleLogger.h include/LoggerRos.h)

add_dependencies(${FILESNAME} ${${FILESNAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${FILESNAME} ${catkin_LIBRARIES})
