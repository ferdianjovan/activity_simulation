cmake_minimum_required(VERSION 2.8.3)
project(activity_simulation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  region_observation
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  ActivitiesMsg.msg
  ActivityMsg.msg
)

add_service_files(
  FILES
  ActivitiesSrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES activity_simulation
  CATKIN_DEPENDS rospy std_msgs message_runtime region_observation
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
