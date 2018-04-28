#!/usr/bin/python

##############################################################
# This implements a average/median filter to the 'alavr'
# output (AR marker pose readings)
##############################################################
# ASSUMPTION is that only one marker is used;
# another ASSUMPTION is that the marker's ideal pose is known
##############################################################

import sys
import math
import numpy as np
import threading

import rospy
import tf

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from ar_track_alvar_msgs.msg import *
from cam_kinect.srv import CamCalibrate
from cam_kinect.srv import CamCalibrateResponse

class ARMarkerFilter():

  def __init__(self):

    # filter type (median / average)
    self._type = "average"             # if averaging is desired, change to "average"
    self._windowSize = 501            # size of the sliding window
    self._windowConstructed = False
    self._seq = 0
    # results storage
    self._containerLock = threading.Lock()
    self._container = list()
    # pose error estimate
    self._calibrateRotOnly = False                  # if calibration of the full transformation is desired, set to False
    self._markerPosFromArm = [0.02323, 0.27645]     # if calibrateRotOnly is set to False, this must be specified to marker position in base_frame
    self._rootFrame = "/world"                      # static frame
    self._cameraLinkName = "/camera_link"           # frame to be calibrated
    self._root2cam = None
    self._root2camAng = None
    self._root2camStr = None
    self._tfListener = tf.TransformListener()
    rospy.loginfo("[alavr_filters] Looking for [{0}] frame...".format(self._cameraLinkName))
    try:
      self._tfListener.waitForTransform(self._rootFrame, self._cameraLinkName, rospy.Time(), rospy.Duration(5.0))
      self._root2cam = self._tfListener.lookupTransform(self._rootFrame, self._cameraLinkName, rospy.Time())
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      rospy.loginfo("[alavr_filters] Can't find transformation from [{0}] to [{1}].".foramt(self._root2cam, self._cameraLinkName))
      return None
    else:
      self._root2cam = self._tfListener.fromTranslationRotation(self._root2cam[0], self._root2cam[1])
      self._root2camAng = tf.transformations.quaternion_from_matrix(self._root2cam)
      self._root2camStr = np.array_str(self._root2cam)
    finally:
      rospy.loginfo("[alavr_filters] Done.")
    self._idealOrientation = [0.0, -1.0 * np.pi / 180.0, np.pi]    # set the orientation reference for the marker (angles in order XYZ)

    # subscribe from alavr
    self._markerSub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.markerCb)
    self._markerPub = rospy.Publisher("/ar_pose_filter/error", Pose, queue_size = 10)
    # publish the camera calibration service
    self._camCalibrate = rospy.Service("/ar_pose_filter/calculate", CamCalibrate, self.calculate)

    rospy.loginfo("[alavr_filters] Initialization done.")

  def _download(self, data):

    # save marker pose into the list
    # the caller should be responsible for the
    # constraint of the window size
    self._container.append(data.pose.pose)
    return

  def _popLeft(self):

    # pop the first element of the container
    self._container.pop(0)
    return

  def markerCb(self, marker_poses):

    if self._seq < self._windowSize:
      self._download(marker_poses.markers[0])
      self._windowConstructed = False
    else:
      with self._containerLock:
        self._popLeft()
        self._download(marker_poses.markers[0])
      if not self._windowConstructed:
        rospy.loginfo("[alavr_filters] Ready for calibration.")
      self._windowConstructed = True

    # increase the sequence
    self._seq += 1
    return

  def _roundTo2PiMask(self, data_mat):

    # this function will return a mask that will 
    # wrap all angle fields into 0 to 2PI
    angle_field_mask = np.atleast_2d(data_mat[5, :] < 0)
    return np.concatenate((np.zeros((5, np.size(data_mat, 1)), dtype = bool), angle_field_mask), axis = 0)

  def calculate(self, req):

    res = CamCalibrateResponse()
    # check whether we have enough
    # data to do the calibraiton
    if not self._windowConstructed:
      res.success = False
      res.reason = "[alavr_filters] Alavr Filter is not ready to calibrate yet."
      return res

    # calibrate
    # STEP 1: calculate the average marker_pose detected
    poses = None
    with self._containerLock:
      poses = list(self._container)
    poses_mat = np.zeros((6, len(poses)))
    for index in range(len(poses)):
      # transfer quaternions into euler angles (in XYZ order)
      position = poses[index].position
      quaternion = poses[index].orientation
      angles = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
      # add to the mat
      poses_mat[:, index] = [position.x, position.y, position.z, angles[0], angles[1], angles[2]]

    avg_pose = Twist()
    if self._type == "median":
      selected_index = int(math.floor(self._windowSize / 2.0))
      sorted_poses_mat = np.sort(poses_mat)
      selected_pose = sorted_poses_mat[:, selected_index]
      avg_pose.linear.x = selected_pose[0]
      avg_pose.linear.y = selected_pose[1]
      avg_pose.linear.z = selected_pose[2]
      avg_pose.angular.x = selected_pose[3]
      avg_pose.angular.y = selected_pose[4]
      avg_pose.angular.z = selected_pose[5]
    else:
      # average
      wrap_mask = self._roundTo2PiMask(poses_mat)
      poses_mat[wrap_mask] += 2 * np.pi
      avg_pose_vec = np.sum(poses_mat, axis = 1)
      avg_pose_vec = avg_pose_vec / self._windowSize
      avg_pose.linear.x = avg_pose_vec[0]
      avg_pose.linear.y = avg_pose_vec[1]
      avg_pose.linear.z = avg_pose_vec[2]
      avg_pose.angular.x = avg_pose_vec[3]
      avg_pose.angular.y = avg_pose_vec[4]
      avg_pose.angular.z = avg_pose_vec[5]

    cur_marker_trans = tf.transformations.euler_matrix(avg_pose.angular.x, avg_pose.angular.y, avg_pose.angular.z)
    cur_marker_trans[0 : 3, 3] = [avg_pose.linear.x, avg_pose.linear.y, avg_pose.linear.z]
    cur_marker_trans_str = np.array_str(cur_marker_trans)

    # STEP 2: construct a transformation matrix for ideal marker pose
    ideal_marker_trans = tf.transformations.euler_matrix(self._idealOrientation[0], self._idealOrientation[1], self._idealOrientation[2])
    if self._calibrateRotOnly:
      ideal_marker_trans[0 : 3, 3] = cur_marker_trans[0 : 3, 3]
    elif self._markerPosFromArm == None:
      rospy.logerr("[alavr_filters] Translation info not gathered from manipulator.")
      res.success = False
      res.reason = "[alavr_filters] Translation info not gathered from manipulator."
      return res
    else:
      ideal_marker_trans[0 : 3, 3] = cur_marker_trans[0 : 3, 3]
      ideal_marker_trans[0 : len(self._markerPosFromArm), 3] = self._markerPosFromArm

    # STEP 3: how much correction we need
    print ideal_marker_trans
    corr_trans = np.dot(ideal_marker_trans, np.linalg.inv(cur_marker_trans))
    corr_trans[3,:] = [0, 0, 0, 1]
    corr_angles = tf.transformations.euler_from_matrix(corr_trans)
    corr_trans_str = np.array_str(corr_trans)

    # STEP 4: apply the correction onto the camera link
    root2camCalibrated = np.dot(corr_trans, self._root2cam)
    root2camCalibratedAng = tf.transformations.quaternion_from_matrix(root2camCalibrated)
    root2camCalibratedStr = np.array_str(root2camCalibrated)

    # STEP 5: construct the response
    res.success = True
    res.reason = "[alavr_filters] =========== RESULTS ===========\n"
    res.reason += "Original [{0}] --> [{1}]: (x, y, z, w): ({2}, {3}, {4}, {5})\n".format(self._rootFrame, \
          self._cameraLinkName, self._root2camAng[0], self._root2camAng[1], self._root2camAng[2], self._root2camAng[3])
    res.reason += self._root2camStr + "\n"
    res.reason += "Correction transformation: (RX, RY, RZ): ({0}, {1}, {2})\n".format(corr_angles[0] * 180.0 / np.pi, corr_angles[1] * 180.0 / np.pi, corr_angles[2] * 180.0 / np.pi)
    res.reason += corr_trans_str + "\n"
    res.reason += "Corrected transformation: (X, Y, Z | x, y, z, w): ({0}, {1}, {2} | {3}, {4}, {5}, {6})\n".format(root2camCalibrated[0, 3], root2camCalibrated[1, 3], root2camCalibrated[2, 3], \
              root2camCalibratedAng[0], root2camCalibratedAng[1], root2camCalibratedAng[2], root2camCalibratedAng[3])
    res.reason += root2camCalibratedStr + "\n"
    res.reason += "[alavr_filters] =========== PASTE THIS PART ===========\n"
    if self._calibrateRotOnly:
      res.reason += "[{0} {1} {2} {3}]".format(root2camCalibratedAng[0], root2camCalibratedAng[1], root2camCalibratedAng[2], root2camCalibratedAng[3])
    else:
      res.reason += "[{0} {1} {2} {3} {4} {5} {6}]".format(root2camCalibrated[0, 3], root2camCalibrated[1, 3], root2camCalibrated[2, 3], \
          root2camCalibratedAng[0], root2camCalibratedAng[1], root2camCalibratedAng[2], root2camCalibratedAng[3])
    return res

# END of CLASS

if __name__ == '__main__':

  rospy.init_node('alavr_filters')

  marker_filter = ARMarkerFilter()

  rospy.spin()
