#!/usr/bin/python
# this script is temporally used as a interface b/w
# DENSO robot and ROS system. This will be deprecated
# once we move the system fully into ROS system

import rospy
from cam_kinect.srv import FetchOneFrame
from blackbox.msg import Trajectory

import logging
import listener
import transmitter

# ROS node
class RobotInterface():
  def __init__(self):
    # initialize this node
    rospy.loginfo('[robot_interface] initializing...')

    service_name = rospy.get_param('cam_service', '/kinect_feedback/next_frame')
    rospy.loginfo('[robot_interface] Waiting for service {0}.'.format(service_name))
    rospy.wait_for_service(service_name, 10)
    self.notify_cam = rospy.ServiceProxy(service_name, FetchOneFrame)
    rospy.loginfo('[robot_interface] Service[{0}] online.'.format(service_name))
    topic_name = rospy.get_param('blackbox_output', '/blackbox/robot_trajectory')
    self.trajectory_topic = rospy.Subscriber(topic_name, Trajectory, self.__trajectoryCb)

    # TCP connect with robot, DISABLE localhost
    self.receiver = listener.BlackboxListener(localhost = False)
    self.emitter = transmitter.BlackboxTransmitter(localhost = False)

    rospy.loginfo('[robot_interface] Done.')

  # what shall we do when we received trajectory from blackbox?
  def __trajectoryCb(self, trajectory):
    robot_dim = trajectory.robot_dim
    robot_trajectory = trajectory.trajectory
    if (len(robot_trajectory) % robot_dim != 0):
      rospy.logwarn('[robot_interface] Trajectory doesn not match robot dimension!')
      return
    else:
      # send the trajectory to the robot through TCP
      meta_data = dict()
      meta_data['size'] = len(robot_trajectory) / robot_dim
      meta_data['data'] = robot_trajectory
      self.emitter.send_data(meta_data)
      return

  # what shall we do if we received signal from the robot?
  def NotifyCamera(self, finished = True):
    # notify the camera to fetch the next frame
    response = self.notify_cam(finished)
    if response.success:
      rospy.logwarn('[robot_interface] Successfully notified the camera.')
    else:
      rospy.logwarn('[robot_interface] Failed ({0}).'.format(response.reason))
    return

  # start the TCP connection
  def StartCSModel(self):
    # self.receiver.start()
    self.emitter.start()

  # end the TCP connection
  def EndCSModel(self):
    # self.receiver.end()
    self.emitter.end()

if __name__ == '__main__':
  # initialize the node
  rospy.init_node('robot_wrapper')

  # bring up the node
  robot_interface = RobotInterface()
  robot_interface.StartCSModel()

  spinner = rospy.Rate(2)
  while not rospy.is_shutdown():
    # here we wants to notify the camera
    # if the robot finished its iteration
    # ret_val = robot_interface.receiver.retrieve_data(blocking = False)
    # if ret_val[0] == True:
    #   # robot finished interation!
    #   robot_interface.NotifyCamera()
    spinner.sleep()

  # exiting this node
  robot_interface.EndCSModel()
  rospy.loginfo('[robot_interface] Exiting robot interface...')
