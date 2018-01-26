cmake_minimum_required(VERSION 2.8.3)
project(robot_wrapper)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  blackbox
  cam_kinect
  std_msgs
  message_generation
)

catkin_package(
 CATKIN_DEPENDS rospy blackbox cam_kinect std_msgs message_runtime
)
