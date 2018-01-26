/*
 * Copyright @ CERLAB
 * this node fetch a single frame form the Kinect XBOX 360
 * when the service is being called
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <sensor_msgs/PointCloud2.h>
#include <cam_kinect/FetchOneFrame.h>
#include <cam_kinect/SaveCloud.h>

#include <boost/thread/mutex.hpp>

class CamKinect {
 private:
  ros::NodeHandle nh, pnh;
  ros::Subscriber kinect_frame_cont;
  ros::Publisher kinect_frame_discrete;
  ros::ServiceServer next_frame, save_cloud;

  std::string frame_id;
  unsigned int seq;

  sensor_msgs::PointCloud2Ptr previous_frame;
  boost::shared_mutex update_frame_lock;

 public:
  CamKinect() : nh("kinect_feedback"), pnh("~"), frame_id("cam"), seq(0) {
    // initialize the node functionality here
    ROS_INFO("[cam_kinect] Initializing...");
    std::string sub_topic_name = this->pnh.param<std::string>("kinect_frame_topic", "/camera/depth/points");
    this->kinect_frame_cont = this->nh.subscribe(sub_topic_name, 20, &CamKinect::kinect_framesCb, this);

    std::string pub_topic_name = this->pnh.param<std::string>("publish_frame_topic", "point_cloud");
    this->kinect_frame_discrete = this->nh.advertise<sensor_msgs::PointCloud2>(pub_topic_name, 20);

    std::string srv_name = this->pnh.param<std::string>("ack_service_name", "next_frame");
    this->next_frame = this->nh.advertiseService(srv_name, &CamKinect::acknowledgeCb, this);

    this->save_cloud = this->nh.advertiseService("save_cloud", &CamKinect::save_cloudCb, this);
    
    this->previous_frame = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2());
    ROS_INFO("[cam_kinect] Done.");
  }

 private:
  void kinect_framesCb(const sensor_msgs::PointCloud2ConstPtr &frame) {
    // download to local point cloud
    boost::upgrade_lock<boost::shared_mutex> _lock(this->update_frame_lock);
    boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(_lock);
    this->previous_frame = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2(*frame));

    return;
  }

  bool acknowledgeCb(cam_kinect::FetchOneFrame::Request &req,
                      cam_kinect::FetchOneFrame::Response &res) {
    if (req.cycle_done) {
      ROS_INFO("[cam_kinect] Cycle done. Sending next frame...");
      {
        boost::shared_lock<boost::shared_mutex> read_lock(this->update_frame_lock);
        this->kinect_frame_discrete.publish(*this->previous_frame);
      }
      res.success = true;
      res.reason = "Success!";
      ROS_INFO("[cam_kinect] Done.");
    } else {
      ROS_WARN("[cam_kinect] Cycle is not done, yet the robot send frame request!");
      res.success = false;
      res.reason = "Previous iteration is still under processing...";
    }
    return true;
  }

  bool save_cloudCb(cam_kinect::SaveCloud::Request &req,
                      cam_kinect::SaveCloud::Response &res) {
    ROS_INFO("[cam_kinect] Saving current frame to the disk...");
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_frame(new pcl::PointCloud<pcl::PointXYZ>);
    {
      boost::shared_lock<boost::shared_mutex> read_lock(this->update_frame_lock);
      pcl::fromROSMsg(*(this->previous_frame), *current_frame);
    }
    std::string save_path = ros::package::getPath("cam_kinect") + "/data/" + req.file_name + ".pcd";
    pcl::io::savePCDFileASCII(save_path, *current_frame);

    res.success = true;
    res.reason = "Success.";
    ROS_INFO("[cam_kinect] Done.");
    return true;
  }
};

/************************************ MAIN ************************************/
int main(int argc, char **argv) {
  // initialize ROS node
  ros::init(argc, argv, "kinect_grabber");
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // bring up the node
  CamKinect kinect;

  // do nothing here
  ros::waitForShutdown();
  spinner.stop();

  return 0;
}
