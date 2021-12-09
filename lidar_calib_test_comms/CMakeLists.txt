cmake_minimum_required(VERSION 3.0.2)
project(lidar_calib_test_comms)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  test_pointcloud.msg
)

## Generate services in the 'srv' folder
add_service_files( 
  FILES
  calib_result.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_ext_test_msg
 CATKIN_DEPENDS message_generation message_runtime roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)