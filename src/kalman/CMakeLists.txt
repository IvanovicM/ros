cmake_minimum_required(VERSION 2.8.3)
project(kalman)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  rospy
  std_msgs
)

catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)