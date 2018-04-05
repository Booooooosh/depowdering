cmake_minimum_required(VERSION 2.8.3)
project(robot_interface)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  blackbox
  cam_kinect
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  geometric_shapes
  pluginlib
  tf
  tf_conversions
  std_msgs
  message_generation
)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

add_service_files(
  FILES
  IO_Control.srv
  FollowEdge.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
  std_msgs
  message_runtime
)

## build ##
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(robot_trajectory_interface src/robot_trajectory_interface.cpp)
add_dependencies(robot_trajectory_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_trajectory_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
