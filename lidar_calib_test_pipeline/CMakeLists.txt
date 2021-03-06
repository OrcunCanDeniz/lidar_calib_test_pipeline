cmake_minimum_required(VERSION 3.0.2)
project(lidar_calib_test_pipeline)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  lidar_calib_test_comms
)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 QUIET)
find_package(PCL 1.7 REQUIRED common io )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES data_handler_srv
 CATKIN_DEPENDS message_runtime  pcl_conversions pcl_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 11)

## build data handler 
add_library(extrinsic_lib src/extrinsics_parser.cpp)
add_library(data_handler_lib src/data_handler_server/data_handler.cpp)
add_executable(data_handling_server src/data_handler_server/data_handler_server_node.cpp)
target_link_libraries(data_handling_server
  ${catkin_LIBRARIES}
  data_handler_lib
  extrinsic_lib
  ${PCL_LIBRARIES}
  stdc++fs
  yaml-cpp
)
add_dependencies(data_handling_server ${catkin_EXPORTED_TARGETS})


## build evaluator
add_library(io_lib src/io_utils.cpp)
add_library(evaluator_lib src/evaluator/evaluator.cpp)
add_executable(evaluator src/evaluator/evaluator_node.cpp)
target_link_libraries(evaluator
  evaluator_lib
  io_lib
  ${catkin_LIBRARIES}
)
