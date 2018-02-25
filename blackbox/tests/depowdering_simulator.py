#! /usr/bin/python

## this is a test script for bayes filter   ##
## pass this test before try your algorithm ##
## on the real point cloud                  ##

## Coordinate is defined as:
##  |TOP
##  |
##  |
##  |
##  |___________
## 0\  (FRONT)
##   \
##    \
##     \(LEFT)

import sys
import numpy as np

import rospy

from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as PointClouds
from sensor_msgs.msg import PointCloud2
from blackbox.srv import *

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class DepowderingSimulator():
  def __init__(self):
    ## Simulator Parameters ##
    rospy.loginfo("[DP_SIM] Initializing simulator parameters...")
    # define ROI
    self.pc_width = 100
    self.pc_height = 100
    self.pc_leftFront = (50.0, 50.0)
    self.pc_rightRear = (250.0, 250.0)

    # define initial height
    self.pc_initZ = 200.0

    # define noise
    self.pc_gaussian_mean = 0.0
    self.pc_gaussian_std = 0.8

    # define objects location
    self.part_length = 100.0
    self.part_width = 100.0
    self.part_height = 100.0
    self.part_topLeftFront = (100.0, 100.0, 170.0)
    self.part_attrs = None

    # define powder decrease after each iteration
    self.depowder_mean = -5.0
    self.depowder_std = 0.0001
    self.depowder_till_now = 0.0
    # define the duration take for each iteration
    self.layer_duration = 3.0

    ## ROS Parameters ##
    # bayes filter service
    rospy.loginfo("[DP_SIM] Looking for bayes filter service...")
    self.bayes_filter = rospy.ServiceProxy("/blackbox_auxillary/bayes_filter", BayesFilter)

    ## Visualization ##
    plt.ion()
    self.figure = plt.figure()
    self.chart = self.figure.add_subplot(111, projection = '3d')
    self.chart.set_xlabel('x')
    self.chart.set_ylabel('y')
    self.chart.set_zlabel('height')
    self.chart.set_title('simulated_depowdering')

    ## Simulator Local Variables ##
    # the point cloud
    # TODO (Check whether the object height is hegher than the initZ)
    self.finished = False
    self.frame = None
    self.step_sizeX = 0.0; self.step_sizeY = 0.0
    self.seq = 0
    self._sim_init_frame()

    rospy.loginfo("[DP_SIM] Everyone is happy. :)")

  def _world2grid(self, pos):
    grid_x = int((pos[0] - self.pc_leftFront[0]) / self.step_sizeX)
    grid_y = int((pos[1] - self.pc_leftFront[1]) / self.step_sizeY)

    # to prevent index error
    if grid_x >= self.pc_height:  grid_x = self.pc_height - 1
    if grid_y >= self.pc_width:   grid_y = self.pc_width - 1

    return (grid_x, grid_y)

  def _sim_init_frame(self):
    """
    initialize the inital frame using numpy
    should look like [[[x1, y1, z1], [x2, y2, z2]...], [[x38, y38, z38], ...], ...]
    """
    rospy.loginfo("[DP_SIM] Initializing simulated frame...")
    self.frame = np.zeros((self.pc_height, self.pc_width, 3), dtype = np.float32)

    self.step_sizeX = (self.pc_rightRear[0] - self.pc_leftFront[0]) * 1.0 / self.pc_height
    self.step_sizeY = (self.pc_rightRear[1] - self.pc_leftFront[1]) * 1.0 / self.pc_width
    for x_step in xrange(0, self.pc_height):
      for y_step in xrange(0, self.pc_width):
        self.frame[x_step][y_step][0], self.frame[x_step][y_step][1], self.frame[x_step][y_step][2] = \
            self.pc_leftFront[0] + x_step * self.step_sizeX, self.pc_leftFront[1] + y_step * self.step_sizeY, self.pc_initZ

    # add noise
    self.frame[:, :, 2] += np.random.normal(self.pc_gaussian_mean, self.pc_gaussian_std, (self.frame.shape[0], self.frame.shape[1]))

    rospy.loginfo("[DP_SIM] Analizing parts...")
    # analyzing where the part is within the grid
    partLeftFront = (self.part_topLeftFront[0], self.part_topLeftFront[1])
    partRightRear = (self.part_topLeftFront[0] + self.part_length, self.part_topLeftFront[1] + self.part_width)
    gridLeftFront = self._world2grid(partLeftFront)
    gridRightRear = self._world2grid(partRightRear)
    self.part_attrs = [[gridLeftFront[0], gridRightRear[0]], [gridLeftFront[1], gridRightRear[1]], self.part_topLeftFront[2]]
    rospy.loginfo("[DP_SIM] Row:[{0}:{1}], Col:[{2}:{3}], Height:{4}".format(self.part_attrs[0][0], self.part_attrs[0][1], 
                    self.part_attrs[1][0], self.part_attrs[1][1], self.part_attrs[2]))

    rospy.loginfo("[DP_SIM] Done. :)")

    return

  def _disturb_the_part(self, disturbance):
    """
    disturbe the position of the part to simulate
    the situation where the robot accidentally touch
    the part
    """
    if (self.pc_initZ - self.depowder_till_now) >= self.part_topLeftFront[2]:
      # part is still buried
      rospy.logwarn("[DP_SIM] Part is still buried, you can not move it. :(")
      return False

    # flatten where the part is to the height of the powder surface
    surface_bomber = np.full((self.part_attrs[0][1] - self.part_attrs[0][0], self.part_attrs[1][1] - self.part_attrs[1][0]), (self.pc_initZ - self.depowder_till_now))
    self.frame[self.part_attrs[0][0]:self.part_attrs[0][1], self.part_attrs[1][0]:self.part_attrs[1][1], 2] = surface_bomber

    # move the part
    if (self.part_attrs[0][0] + disturbance[0] < 0) or (self.part_attrs[0][1] + disturbance[0] >= self.frame.shape[0]):
      #invalid disturbance
      return False
    else:
      self.part_attrs[0][0] += disturbance[0]
      self.part_attrs[0][1] += disturbance[0]

    if (self.part_attrs[1][0] + disturbance[1] < 0) or (self.part_attrs[1][1] + disturbance[1] >= self.frame.shape[1]):
      return False
    else:
      self.part_attrs[1][0] += disturbance[1]
      self.part_attrs[1][1] += disturbance[1]

    return True

  def _random_part_movement(self, freq = 3):
    if (self.pc_initZ - self.depowder_till_now) >= self.part_topLeftFront[2]:
      # part is still buried
      return False
    else:
      if (self.seq % freq) == 0:
        disturbance = (np.random.randint(-10, 10), np.random.randint(-4, 4))
        rospy.logwarn("[DP_SIM] Disturb the part by [{0}, {1}] grids".format(disturbance[0], disturbance[1]))
        self._disturb_the_part(disturbance)

    return True

  def _next_layer(self):
    """
    here is where the simulation is done, we simply decrease
    the height of the point cloud by some fixed amount to
    simulate the effect of depowdering
    @Return: True if all layers are finished
    """
    assert self.frame != None and isinstance(self.frame, np.ndarray) and \
      self.frame.shape[0] == self.pc_height and self.frame.shape[1] == self.pc_width and self.frame.shape[2] == 3

    rospy.loginfo("[DP_SIM] Depowdering...")
    # simulated depowdering
    self.frame[:, :, 2] += np.random.normal(self.depowder_mean, self.depowder_std, (self.frame.shape[0], self.frame.shape[1]))
    self.depowder_till_now += (-self.depowder_mean)
    percentage = (self.depowder_till_now * 1.0 / (self.pc_initZ - (self.part_topLeftFront[2] - self.part_height)))

    # enforce the height of the object
    sub_matrix = self.frame[self.part_attrs[0][0]:self.part_attrs[0][1], self.part_attrs[1][0]:self.part_attrs[1][1], 2]
    sub_matrix = (sub_matrix - self.part_attrs[2]).clip(min = 0.0) + self.part_attrs[2]
    self.frame[self.part_attrs[0][0]:self.part_attrs[0][1], self.part_attrs[1][0]:self.part_attrs[1][1], 2] = sub_matrix

    # simulated capturing
    self.frame[:, :, 2] += np.random.normal(self.pc_gaussian_mean, self.pc_gaussian_std, (self.frame.shape[0], self.frame.shape[1]))

    # show status
    self.chart.clear()
    plot_x, plot_y, plot_z = self.frame[:, :, 0].tolist(), self.frame[:, :, 1].tolist(), self.frame[:, :, 2].tolist()
    self.chart.scatter(plot_x, plot_y, plot_z)
    self.figure.canvas.draw()

    rospy.loginfo("[DP_SIM] Layer finished ({0}/100 finished). :)".format(int(percentage * 100.0)))

    if percentage >= 0.9:
      return True
    else:
      return False

  def send_measurement(self):
    """
    pack the numpy array into PointCloud2 and
    send it to my bayes filter to test
    """
    assert self.frame != None and isinstance(self.frame, np.ndarray) and \
      self.frame.shape[0] == self.pc_height and self.frame.shape[1] == self.pc_width and self.frame.shape[2] == 3

    rospy.loginfo("[DP_SIM] Sending NEXT frame...")
    # reshape into N x 3
    pc = self.frame.reshape((self.pc_width * self.pc_height, 3))

    # transfer into point cloud and publish
    rospy.wait_for_service("/blackbox_auxillary/bayes_filter")

    try:
      request = BayesFilterRequest()
      request.header.seq = self.seq
      request.header.stamp = rospy.Time.now()
      request.header.frame_id = "sim_robot_work_frame"
      request.width = pc.shape[0]
      request.height = pc.shape[1]

      header = Header()
      header.seq = self.seq
      header.stamp = rospy.Time.now()
      header.frame_id = "sim_robot_work_frame"
      request.measurement = PointClouds.create_cloud_xyz32(header, pc)

      respond = self.bayes_filter(request)
    except rospy.ServiceException, err:
      rospy.logerr("[DP_SIM_STOPPED] Service call failed: {0}".format(err))
      return False
    else:
      self.seq += 1
      rospy.loginfo("[DP_SIM] Done. :)")
      return True

  def run(self):
    """
    run the simulator
    """
    waiter = rospy.Rate(1.0 / self.layer_duration)
    while not rospy.is_shutdown() and not self.finished:
      # depowder
      if self._next_layer() == True:
        self.finished = True
      # send data
      self.send_measurement()
      # randomly move the part
      self._random_part_movement()
      # wait
      waiter.sleep()

    if self.finished:
      rospy.loginfo("[DP_SIM_FINISHED] Finished. :))")
    else:
      rospy.loginfo("[DP_SIM_INTERRUPTED] Exit. :O ")


if __name__ == '__main__':
  rospy.init_node("depowdering_simulator")

  sim = DepowderingSimulator()
  
  sim.run()

  sys.exit(0)
