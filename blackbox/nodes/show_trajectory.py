#! /usr/bin/python

# this script subscribe to the trajectory topic published
# by the blackbox and visualize the current generated
# trajectory. We do this in python just because matplotlib
# is easier to use in python

import sys
import numpy as np
import rospy

from blackbox.msg import *

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class TrajectoryReader():
  def __init__(self):
    # we only need one subscriber
    self.traj_reader = rospy.Subscriber("/blackbox/debug_traj", Trajectory, self.traj_visualizer)

    # and a place to store the trajectory
    self.traj = list()

    rospy.loginfo("[TrajectoryReader] Initialized.")

  def traj_visualizer(self, traj_msg):
    # visualize the trajectory within the callback function
    robot_dim = traj_msg.robot_dim
    self.traj = traj_msg.trajectory

    # check if the trajectory is valid
    if (len(self.traj) % robot_dim) != 0:
      rospy.logerr("[traj_visualizer] Trajectory doesn't match robot dimension!!")
      return

    # passed, sort the trajectory into three lists
    rospy.loginfo("[traj_visualizer] Showing trajectory.")
    Xs = self.traj[0::robot_dim]
    Ys = self.traj[1::robot_dim]
    Zs = self.traj[2::robot_dim]

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(Xs, Ys, Zs)
    ax.legend()
    plt.show()

    return

if __name__ == '__main__':
  # initialize this node
  rospy.init_node("show_trajectory")

  # spawn the class
  reader = TrajectoryReader()

  # wait for incoming request
  rospy.spin()

  sys.exit(0)