cmake_minimum_required(VERSION 2.8.3)
project(sensors)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  actionlib_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RawMeasurement.msg
  ProcessedMeasurement.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  RecordMeasurement.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  BigTempDiff.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
