#!/usr/bin/python
# this script simply subscribes to the trajectory topic
# and show the trajectory

import sys
import rospy
from blackbox.msg import Trajectory

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def show_trajectory(trajectory):
  robot_dim = trajectory.robot_dim
  data_output = list(trajectory.trajectory)

  data_for_plot = [data_output[item::robot_dim] for item in range(robot_dim)]
  fig = plt.figure()
  ax = fig.gca(projection='3d')
  ax.plot(data_for_plot[0], data_for_plot[1], data_for_plot[2])
  ax.legend()
  plt.show()

if __name__ == '__main__':
  rospy.init_node('show_trajectory')
  rospy.Subscriber("blackbox/robot_trajectory", Trajectory, show_trajectory)
  rospy.spin()
