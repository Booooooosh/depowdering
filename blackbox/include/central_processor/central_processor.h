#ifndef _CENTRAL_PROCESSOR_H
#define _CENTRAL_PROCESSOR_H

/*
 * blackbox processing module node
 */
#include <ros/ros.h>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread/mutex.hpp>

#include <vector>
#include <algorithm>

#include <sensor_msgs/PointCloud2.h>
#include <blackbox/Trajectory.h>

#include <blackbox/BlackboxConfig.h>

class CentralProcessor {
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef PointCloud::Ptr PointCloudPtr;
  typedef PointCloud::ConstPtr PointCloudConstPtr;

 private:
  ros::NodeHandle nh, pnh;
  ros::Subscriber cam_feedback;
  ros::Publisher to_robot, _debug_window;

  tf::TransformListener tf_listener;
  std::string target_frame;

  dynamic_reconfigure::Server<blackbox::BlackboxConfig> reconfig_server;

  // ROI region
  float lowerX, upperX, lowerY, upperY, lowerZ, upperZ;
  // grid ALG resolution
  float resX, resY, EEFOffset;

  PointCloudPtr last_frame;
  unsigned int seq;
 
 public:
  CentralProcessor() : nh("blackbox"), pnh("~"), seq(0) {
    // node initialization //
    ROS_INFO("[blackbox] Initializing...");
    // ROI //
    std::vector<double> roi;
    if (!this->pnh.getParam("roi/x", roi)) {
      ROS_WARN("[blackbox] ROI(X-axis) not specified, using default value!");
      this->lowerX = -10.0; this->upperX = 10.0;
    } else {
      this->lowerX = roi[0]; this->upperX = roi[1];
    }
    roi.clear();
    if (!this->pnh.getParam("roi/y", roi)) {
      ROS_WARN("[blackbox] ROI(Y-axis) not specified, using default value!");
      this->lowerY = -10.0; this->upperY = 10.0;
    } else {
      this->lowerY = roi[0]; this->upperY = roi[1];
    }
    roi.clear();
    if (!this->pnh.getParam("roi/z", roi)) {
      ROS_WARN("[blackbox] ROI(Z-axis) not specified, using default value!");
      this->lowerZ = -10.0; this->upperZ = 10.0;
    } else {
      this->lowerZ = roi[0]; this->upperZ = roi[1];
    }
    roi.clear();

    // transformation //
    this->target_frame = pnh.param<std::string>("target_frame", "robot_work_frame");

    // grid resolution //
    this->resX = this->pnh.param<double>("resolution/x", 15.0);
    this->resY = this->pnh.param<double>("resolution/y", 15.0);
    this->EEFOffset = this->pnh.param<double>("end_effector/offset", 5.0);

    // topics //
    std::string sub_topic_name = this->pnh.param<std::string>("ros_node/sub_topic_name", "/kinect_feedback/point_cloud");
    this->cam_feedback = this->nh.subscribe(sub_topic_name, 1, &CentralProcessor::processCb, this);
    std::string pub_topic_name = this->pnh.param<std::string>("ros_node/pub_topic_name", "robot_trajectory");
    this->to_robot = this->nh.advertise<blackbox::Trajectory>(pub_topic_name, 1);
    this->_debug_window = this->nh.advertise<sensor_msgs::PointCloud2>("debug", 1);

    // reconfiguration //
    dynamic_reconfigure::Server<blackbox::BlackboxConfig>::CallbackType funcHd;
    funcHd = boost::bind(&CentralProcessor::dynamic_reconfigurationCb, this, _1, _2);
    this->reconfig_server.setCallback(funcHd);

    // PCL //
    this->last_frame = PointCloudPtr(new PointCloud);
    this->seq = 0;

    ROS_INFO("[blackbox] Done.");
  }

 private:
  /*
   * Function @ processCb
   * callback function for processing point data
   */
  void processCb(const sensor_msgs::PointCloud2ConstPtr &frame);
  /*
   * Function @ pcl_passthrough
   * pass through filter used for setting ROI
   */
  PointCloudPtr pcl_passthrough(PointCloudPtr cloud);
  /*
   * Function @ path_from_grid
   * generate path using simple grid method
   * Assumpotion: Units are in(mm)
   *              Input point cloud is preprocessed and within ROI
   */
  bool path_from_grid(const PointCloudPtr &cloud, std::vector<float> &trajectory);
  /*
   * Function @ dynamic_reconfigurationCb
   * callback function for dynamic reconfiguration
   */
  void dynamic_reconfigurationCb(blackbox::BlackboxConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure request: ROI_X:[%f, %f], ROI_Y:[%f, %f], ROI_Z:[%f, %f].", 
              config.roi_lower_x, config.roi_upper_x,
              config.roi_lower_y, config.roi_upper_y,
              config.roi_lower_z, config.roi_upper_z);
    this->lowerX = (float)config.roi_lower_x; this->upperX = (float)config.roi_upper_x;
    this->lowerY = (float)config.roi_lower_y; this->upperY = (float)config.roi_upper_y;
    this->lowerZ = (float)config.roi_lower_z; this->upperZ = (float)config.roi_upper_z;

    ROS_INFO("Reconfigure request: EEFOffset: %f.", config.EEFOffset);
    this->EEFOffset = (float)config.EEFOffset;
  }
};

#endif
