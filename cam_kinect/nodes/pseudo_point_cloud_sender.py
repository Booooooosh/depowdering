#!/usr/bin/python
# Pseudo point cloud sender to test the path generation function
# WARNING: this node can not be launched together with the cam_kinect node
#          since they have the same namespace and topic name

import sys
import rospy
import rospkg
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from cam_kinect.srv import SendPseudoCloud

import file_parser

class PseudoCloudPublisher():
  def __init__(self):
    self.cloud_pub = rospy.Publisher('/kinect_feedback/point_cloud', PointCloud2, queue_size=1)
    self.cloud_generator = rospy.Service('/kinect_feedback/generate', SendPseudoCloud, self.generate_point_cloud)
  
  def generate_point_cloud(self, req):
    rospack = rospkg.RosPack()
    path = rospack.get_path("cam_kinect") + "/data/" + req.name
    rospy.loginfo("[generate_point_cloud] Get point cloud form {0}.".format(path))
    frame = file_parser.file_parser(path)
    if not frame:
      return None
    else:
      data = frame['data']
      data_input = []
      for each_row in data:
        data_input += each_row
      data_input = zip(*[data_input[i::3] for i in range(3)])
      rospy.loginfo("[generate_point_cloud] Point cloud size: {0}.".format(len(data_input)))
      # create point cloud
      header = Header()
      header.frame_id = "world"
      header.stamp = rospy.Time.now()
      pcloud = pc2.create_cloud_xyz32(header, data_input)
      self.cloud_pub.publish(pcloud)
      return (True, "Success.")

if __name__ == '__main__':
  rospy.init_node('kinect_feedback')

  pseudo_publisher = PseudoCloudPublisher()
  rospy.spin()
