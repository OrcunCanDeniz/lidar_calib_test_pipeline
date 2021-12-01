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
find_package(PCL 1.7 REQUIRED common io visualization )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES data_handler_srv
 CATKIN_DEPENDS message_runtime  pcl_conversions pcl_ros roscpp sensor_msgs std_msgs message_runtime
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

## build dummy calibrator
add_executable(calibrator src/dummy_calibrator.cpp)
target_link_libraries(calibrator
  ${catkin_LIBRARIES}
  )
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS data_handling_server
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_data_handler_srv.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
