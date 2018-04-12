#! /usr/bin/python

# this script implements some experimental functionality other than
# those in the central_processor.cpp. after the functionalities tested, 
# these functionalities should be merged into the main cpp file OR
# by using nodelets

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
import sensor_msgs.point_cloud2 as PointClouds
from sensor_msgs.msg import PointCloud2
from blackbox.srv import *

import cv2

class Auxillary():
  def __init__(self):
    ## Bayes Filter ##
    ## Belif Groups ##
    self.bf_grid_width = 0
    self.bf_grid_height = 0
    self.belif_groups = None
    self.belif = None
    ## Measurement ##
    self.dynamic_initialized = False
    self.buildbox = None
    self.prev_frame = None
    self.prev_attr = None
    self.cur_frame = None
    self.cur_attr = None
    ## Decision Parameters ##
    self.bf_threshold = rospy.get_param("~decision/threshold", 0.1)
    self.bf_resolution = rospy.get_param("~discretization/resolution", 15.0)
    self.bf_parameters = dict()
    self.bf_parameters['free2free'] = rospy.get_param("~decision/free2free", 0.6)
    self.bf_parameters['occ2occ'] = rospy.get_param("~decision/occ2occ", 0.8)
    self.bf_parameters['obsGivenFree'] = dict()
    self.bf_parameters['obsGivenFree']['mean'] = rospy.get_param("~decision/observedGivenFree/mean", -5.0)
    self.bf_parameters['obsGivenFree']['std'] = rospy.get_param("~decision/observedGivenFree/std", 0.8)
    self.bf_parameters['obsGivenOcc'] = dict()
    self.bf_parameters['obsGivenOcc']['mean'] = rospy.get_param("~decision/observedGivenOcc/mean", 0.0)
    self.bf_parameters['obsGivenOcc']['std'] = rospy.get_param("~decision/observedGivenOcc/std", 0.8)

    ## Services ##
    self.bayes_filterHd = rospy.Service("/blackbox_auxillary/bayes_filter", BayesFilter, self.bayes_filter)

    ## Static Initialization ##
    self._bf_static_initialization()

    ## Visualization ##
    self._bf_visualize()

    rospy.loginfo("[BF_INIT] BayesFilter initialized.")

  def _bf_static_initialization(self):
    """
    here, we initialize the bayes filter
    """
    rospy.loginfo("[BF_INIT] BayesFilter static initializer...")
    # deterimine the grid size
    buildbox_dimension = rospy.get_param("~build_box/dimension", [300.0, 300.0])
    self.bf_grid_width = int(buildbox_dimension[1] * 1.0 / self.bf_resolution + 0.5)
    self.bf_grid_height = int(buildbox_dimension[0] * 1.0 / self.bf_resolution + 0.5)
    rospy.loginfo("[BF_INIT] Build box dimension: [{0}, {1}]. Approximated with grid size[{2}, {3}]".format(buildbox_dimension[0], 
        buildbox_dimension[1], self.bf_grid_height, self.bf_grid_width))
    # initiallize belif gird
    self.belif = np.full((self.bf_grid_height, self.bf_grid_width), fill_value = 0.5)
    #initialize buildbox point cloud grid
    self.buildbox = np.zeros((self.bf_grid_height, self.bf_grid_width))
    rospy.loginfo("[BF_INIT] BayesFilter static initialization finished.")

  def _bf_visualize(self):
    """
    visualize the belif distribution
    """
    resized = cv2.resize(self.belif, (600, 600))
    cv2.imshow('belif', resized)
    cv2.waitKey(2)

  def _world2grid(self, pos):
    """
    transfer world coordinates into grid coordinates
    """
    grid_x = int(pos[0] / self.bf_resolution)
    grid_y = int(pos[1] / self.bf_resolution)

    if grid_x >= self.bf_grid_height: grid_x = self.bf_grid_height - 1
    if grid_y >= self.bf_grid_width:  grid_y = self.bf_grid_width - 1

    return (grid_x, grid_y)

  def _pc_categorizer(self, pc):
    """
    categorize the point cloud into grids
    at the same time, return the location
    of this point cloud w.r.t the buildbox
    @Return: (category, attr)
    @Type:  category: sorted points within the bins
            attr:     relative position of this category w.r.t the global buildbox
    """
    rospy.loginfo("[BF_CATE] Categorizing...")

    # determine left-front and right-rear corner
    pc_array = np.array(list(pc))
    assert pc_array.shape[1] == 3
    rospy.loginfo("[BF_CATE] {0} points detected.".format(pc_array.shape[0]))
    pcLeftFront = (np.min(pc_array[:, 0]), np.min(pc_array[:, 1]))
    pcRightRear = (np.max(pc_array[:, 0]), np.max(pc_array[:, 1]))
    gridLeftFront = self._world2grid(pcLeftFront)
    gridRightRear = self._world2grid(pcRightRear)
    rospy.loginfo("[BF_CATE] LeftFrontPoint: [{0}, {1}] -> [{2}, {3}]; RightRearPoint: [{4}, {5}] -> [{6}, {7}]".format(\
        pcLeftFront[0], pcLeftFront[1], gridLeftFront[0], gridLeftFront[1], pcRightRear[0], pcRightRear[1], gridRightRear[0], gridRightRear[1]))

    # define point cloud location
    attr = ((gridLeftFront[0], gridRightRear[0]), (gridLeftFront[1], gridRightRear[1]))

    # categorize
    category = np.zeros((attr[0][1] - attr[0][0] + 1, attr[1][1] - attr[1][0] + 1))
    counter = np.zeros((attr[0][1] - attr[0][0] + 1, attr[1][1] - attr[1][0] + 1))
    for point in pc_array:
      # calculate global grid index
      gb_grid_pos = self._world2grid((point[0], point[1]))
      lc_grid_pos = (gb_grid_pos[0] - gridLeftFront[0], gb_grid_pos[1] - gridLeftFront[1])
      category[lc_grid_pos] += point[2]
      counter[lc_grid_pos] += 1

    if np.count_nonzero(counter) != counter.shape[0] * counter.shape[1]:
      rospy.logwarn("[BF_CATE] Some of the bins has zeros points.")
      ## TODO (Bosch): Linear intepolation of nearby bins
      counter.clip(min = 1)
    
    category = category / counter

    rospy.loginfo("[BF_CATE] Done. :)")

    return (category, attr)

  def _bf_update_bins(self, new_bins, cutter):
    """
    update the global point cloud map using the
    most recent input point cloud
    """
    self.buildbox[cutter[0][0] : (cutter[0][1] + 1), cutter[1][0] : (cutter[1][1] + 1)] = new_bins

    return

  def _bf_frame_mismatch_resolver(self, prev_frame, cur_frame, prev_attr, cur_attr):
    """
    resolve the conflict when the current frame
    and previous frame have different roi
    @Return: resolved_cutter
    """
    rospy.logwarn("[BF_RESOLVER] Cutter before resolution: [{0}:{1}, {2}:{3}].".format(prev_attr[0][0], prev_attr[0][1], prev_attr[1][0], prev_attr[1][1]))
    # always preserve the cur_frame since this is our observations
    resolved_cutter = cur_attr
    # so we need to tweak the previous frame in order to compare the same region
    self.prev_frame = self._bf_clip_matrix(self.buildbox, resolved_cutter)
    self.prev_attr = resolved_cutter
    rospy.logwarn("[BF_RESOLVER] Cutter after resolution: [{0}:{1}, {2}:{3}].".format(resolved_cutter[0][0], resolved_cutter[0][1], resolved_cutter[1][0], resolved_cutter[1][1]))

    return resolved_cutter

  def _bf_get_sub_matrix(self, mat, leftFront, rightRear):
    """
    here, we fetch a ROI within the buildbox for us to update
    any update in the submatrix will be reflected in the original
    matrix
    """
    try:
      assert rightRear[0] > leftFront[0] > 0 and rightRear[1] > leftFront[1] > 0
      assert (rightRear[0] + 1) < mat.shape[0] and (rightRear[1] + 1) < mat.shape[1]
    except AssertionError:
      rospy.logwarn("[BF_SUB_MTX] Sub area exceeds the original matrix.")
      return None
    else:
      return mat[leftFront[0] : (rightRear[0] + 1), leftFront[1] : (rightRear[1] + 1)]

  def _bf_clip_matrix(self, mat, cutter):
    """
    same as _bf_get_sub_matrix
    @Type: cutter:  ((leftFront[0], rightRear[0]), (leftFront[1], rightRear[1]))
    """
    try:
      assert cutter[0][1] >= cutter[0][0] >= 0 and cutter[1][1] >= cutter[1][0] >= 0
      assert (cutter[0][1] + 1) <= mat.shape[0] and (cutter[1][1] + 1) <= mat.shape[1]
    except AssertionError:
      rospy.logwarn("[BF_SUB_MTX] Sub area exceeds the original matrix.")
      return None
    else:
      return mat[cutter[0][0] : (cutter[0][1] + 1), cutter[1][0] : (cutter[1][1] + 1)]

  def _bf_patch_matrix(self, patch, mat, cutter):
    """
    this will modify the sub-matrix whose area
    designated by cutter
    @Type: cutter:  ((leftFront[0], rightRear[0]), (leftFront[1], rightRear[1]))
    """
    try:
      assert (cutter[0][1] - cutter[0][0] + 1) == patch.shape[0] and (cutter[1][1] - cutter[1][0] + 1) == patch.shape[1]
    except AssertionError:
      rospy.logerr("[BF_PACH_MTX] Patch dimension is not the same as specified by cutter.")
      return False
    else:
      mat[cutter[0][0] : (cutter[0][1] + 1), cutter[1][0] : (cutter[1][1] + 1)] = patch
      return True

  def _gaussian(self, x, mean, std):
    return ((1.0 / (std * np.sqrt(2 * np.pi))) * np.exp(-0.5 * np.square((x - mean) / std)))

  def _bf_obsGivenOCC(self, frame_diff):
    return self._gaussian(frame_diff, self.bf_parameters['obsGivenOcc']['mean'], self.bf_parameters['obsGivenOcc']['std'])

  def _bf_obsGivenFREE(self, frame_diff):
    return self._gaussian(frame_diff, self.bf_parameters['obsGivenFree']['mean'], self.bf_parameters['obsGivenFree']['std'])

  def _bf_estimate(self, roi_cutter):
    """
    bayes filter estimation step
    """
    roi = self._bf_clip_matrix(self.belif, roi_cutter)

    Pfree2free = np.full(roi.shape, fill_value = self.bf_parameters['free2free'])
    Pocc2free = 1 - np.full(roi.shape, fill_value = self.bf_parameters['occ2occ'])
    roi_estimate = roi * Pfree2free + (1.0 - roi) * Pocc2free

    self._bf_patch_matrix(roi_estimate, self.belif, roi_cutter)

    return

  def _bf_update(self, roi_cutter):
    """
    bayes filter observation update step
    """
    roi = self._bf_clip_matrix(self.belif, roi_cutter)

    # calculate frame diff
    frame_diff = self.cur_frame - self.prev_frame
    roi_free_updated = self._bf_obsGivenFREE(frame_diff) * roi
    roi_occ_updated = self._bf_obsGivenOCC(frame_diff) * (1.0 - roi)
    normalizer = 1.0 / (roi_free_updated + roi_occ_updated)
    roi_updated = roi_free_updated * normalizer

    self._bf_patch_matrix(roi_updated, self.belif, roi_cutter)

    return

  def _bf_belif_scrutinizer(self):
    """
    the scrutinizer will check the validness of the belif(zero probability should not happen...)
    and analyze where our part is according to the belif
    """
    # check validness
    if np.count_nonzero(self.belif) != self.belif.shape[0] * self.belif.shape[1]:
      rospy.logwarn("[BF_SCRUT] Bins with zero-belif detected!")

    # analyze this frame of belif
    rospy.loginfo("[BF_SCRUT] Analyze the belif map with threshold: {0}...".format(self.bf_threshold))
    result = np.nonzero(self.belif < self.bf_threshold)
    num_of_bins_as_parts = result[0].shape[0]
    rospy.logwarn("[BF_SCRUT] {0} bins detected as part.".format(num_of_bins_as_parts))

    return

  def bayes_filter(self, req):
    """
    this is where we implement the bayes recursive update
    """
    respond = BayesFilterResponse()
    # preprocess points if necessary (scale matric, offset, etc...)
    input_cloud_tmp = list(PointClouds.read_points(req.measurement, field_names = ('x', 'y', 'z'), skip_nans = False))
    input_cloud = list()
    if req.scale != 1.0:
      for point in input_cloud_tmp:
        # rescale each point
        input_cloud.append((point[0] * req.scale, point[1] * req.scale, point[2] * req.scale))
    else:
      input_cloud = input_cloud_tmp

    # initialize if encountered the first frame
    if not self.dynamic_initialized:
      rospy.loginfo("[BF] Dynamic initiallization...")
      self.prev_frame, self.prev_attr = self._pc_categorizer(input_cloud)
      self._bf_update_bins(self.prev_frame, self.prev_attr)
      rospy.loginfo("[BF] First frame located @ buildbox[{0}:{1}, {2}:{3}].".format(self.prev_attr[0][0], self.prev_attr[0][1], self.prev_attr[1][0], self.prev_attr[1][1]))
      rospy.loginfo("[BF] Done.")
      
      self.dynamic_initialized = True
      
      respond.success = True
      respond.reason = "The first frame is used for dynamic initiallization."
      return respond

    # receive point cloud
    self.cur_frame, self.cur_attr = self._pc_categorizer(input_cloud)
    rospy.loginfo("[BF] Current frame located @ buildbox[{0}:{1}, {2}:{3}].".format(self.cur_attr[0][0], self.cur_attr[0][1], self.cur_attr[1][0], self.cur_attr[1][1]))
    resolved_attr = None
    if self.cur_attr != self.prev_attr:
      rospy.logwarn("[BF] Location of current frame mismatch the previous frame. Resolving...")
      ## TODO (Bosch): frame mismatch resolve
      resolved_attr = self._bf_frame_mismatch_resolver(self.prev_frame, self.cur_frame, self.prev_attr, self.cur_attr)
    else:
      resolved_attr = self.cur_attr

    # update the global point cloud map
    rospy.loginfo("[BF] Merge point cloud into local point cloud map...")
    self._bf_update_bins(self.cur_frame, resolved_attr)

    # update (prediction)
    rospy.loginfo("[BF] Predicting...")
    self._bf_estimate(resolved_attr)

    # update (sensor reading update)
    rospy.loginfo("[BF] Updating...")
    self._bf_update(resolved_attr)

    # scrutinize the belif matrix (see if we have zero belifs)
    self._bf_belif_scrutinizer()

    # TODO (Bosch): plot
    self._bf_visualize()

    # set previous frame to be the current frame
    self.prev_frame = self.cur_frame
    self.prev_attr = self.cur_attr

    # finished
    rospy.loginfo("[BF] Done.")

    # respond
    respond.success = True
    respond.reason = "test success!"
    return respond

if __name__ == '__main__':
  # initialize this node
  rospy.init_node("blackbox_auxillary")

  # spawn the class
  auxillary = Auxillary()

  # wait for incoming request
  rospy.spin()

  sys.exit(0)
