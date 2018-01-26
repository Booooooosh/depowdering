/*
 * Copyright @ CERLAB
 * this is a MoveIt interface for robot trajectory
 * execution
 */

#include <vector>

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/Mesh.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>

#include <cam_kinect/FetchOneFrame.h>
#include <blackbox/Trajectory.h>

class RobotTrajectoryInterface {
 private:
  // frame transformer
  std::string target_frame;
  tf::TransformListener tf_listener;
  // start/end approaching point for this application
  std::vector<double> start_approach;
  std::vector<std::vector<double> > end_approach;
  // current pose for the end effector
  geometry_msgs::Pose current_pose;
  boost::shared_mutex pose_update_lock;

  // ros publisher and subscriber
  ros::NodeHandle nh, pnh;
  ros::Subscriber robot_trajectory;
  ros::Publisher  eef_pose;

  // moveit interface
  moveit::planning_interface::MoveGroup manipulator;
  moveit::planning_interface::PlanningSceneInterface planning_scene;

 public:
  RobotTrajectoryInterface() : manipulator("manipulator"), nh("robot_interface"), pnh("~") {
    ROS_INFO("[rbt_trj] Initializing robot_trajectory node...");
    // subscribe to trajectory topic
    std::string sub_topic_name = this->pnh.param<std::string>("general/trajectory_topic_name", "/blackbox/robot_trajectory");
    this->robot_trajectory = this->nh.subscribe(sub_topic_name, 1, &RobotTrajectoryInterface::trajectoryCb, this);
    // publish end effector pose
    this->eef_pose = this->nh.advertise<geometry_msgs::PoseStamped>("eef_pose", 10);

    // load start/end pose
    this->target_frame = this->pnh.param<std::string>("general/robot_frame_name", "world");
    if (!this->pnh.getParam("approaching_points/start", this->start_approach)) {
      ROS_WARN("[rbt_trj] Start approaching point (Joint Space) not specified!");
      ROS_WARN("[rbt_trj] Assume [1.725956, 0.201411, 1.93173, 0.0706858, 0.9936159, 1.345474]...");
      this->start_approach.push_back(1.725956);
      this->start_approach.push_back(0.201411);
      this->start_approach.push_back(1.93173);
      this->start_approach.push_back(0.0706858);
      this->start_approach.push_back(0.9936159);
      this->start_approach.push_back(1.345474);
    }
    std::vector<double> end_poseN;
    if (!this->pnh.getParam("approaching_points/end", end_poseN)) {
      ROS_WARN("[rbt_trj] End approaching point (Joint Space) not specified!");
      ROS_WARN("[rbt_trj] Assume [1.957212, -0.1488766, 2.290745, 0.065275, 0.971625, 1.578650]...");
      end_poseN.push_back(1.957212);
      end_poseN.push_back(-0.1488766);
      end_poseN.push_back(2.290745);
      end_poseN.push_back(0.065275);
      end_poseN.push_back(0.971625);
      end_poseN.push_back(1.578650);
    }
    this->end_approach.push_back(end_poseN);

    // load buildbox into planning scene
    std::map<std::string, std::string> models;
    std::string model_ref_frame = this->pnh.param<std::string>("workspace/ref_frame", "robot_work_frame");
    if (this->pnh.getParam("workspace/meshes", models) == false) {
      ROS_WARN("[rbt_trj] Workcell environment not loaded, yaml file not found.");
    } else {
      ROS_INFO("[rbt_trj] Loading workspace environment...");
      for (std::map<std::string, std::string>::iterator it = models.begin(); it != models.end(); ++ it) {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = model_ref_frame;
        collision_object.id = it->first;

        shapes::Mesh *cell = shapes::createMeshFromResource(it->second);
        if (!cell) {
          ROS_WARN("[rbt_trj] Work cell environment not loaded. File not found.");
          continue;
        }
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(cell, mesh_msg);
        shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
        collision_object.meshes.resize(1);
        collision_object.mesh_poses.resize(1);

        collision_object.meshes[0] = mesh;
        collision_object.mesh_poses[0].position.x = 0.f;
        collision_object.mesh_poses[0].position.y = 0.f;
        collision_object.mesh_poses[0].position.z = 0.f;
        collision_object.mesh_poses[0].orientation.w = 1.0;
        collision_object.mesh_poses[0].orientation.x = 0.f;
        collision_object.mesh_poses[0].orientation.y = 0.f;
        collision_object.mesh_poses[0].orientation.z = 0.f;

        collision_object.operation = collision_object.ADD;

        this->planning_scene.applyCollisionObject(collision_object);
      }
    }

    // start state monitor
    this->manipulator.startStateMonitor();
    ROS_INFO("[rbt_trj] Done.");

    ROS_INFO("[rbt_trj] Here is some info w.r.t move_group:");
    // joint names
    ROS_INFO("\t[Joint Names]:");
    std::vector<std::string> joint_names = this->manipulator.getJointNames();
    for (int jnt_index = 0; jnt_index < joint_names.size(); jnt_index ++) {
      ROS_INFO("\t\t[%d]: %s", jnt_index + 1, joint_names[jnt_index].c_str());
    }
    // planning frame
    ROS_INFO("\t[Planning Frame]:");
    ROS_INFO("\t\t%s", this->manipulator.getPlanningFrame().c_str());
    // default planner id
    this->manipulator.setPlannerId("RRTConnectkConfigDefault");
    ROS_INFO("\t[Default PlannerID]:");
    ROS_INFO("\t\t%s", this->manipulator.getDefaultPlannerId("manipulator").c_str());
    // end effector name
    ROS_INFO("\t[End Effector]:");
    ROS_INFO("\t\t%s", this->manipulator.getEndEffector().c_str());

    // finished
    ROS_INFO("[rbt_trj] Done.");
  }

  void trajectoryCb(const blackbox::TrajectoryConstPtr &trajectory) {
    ROS_INFO("[rbt_trj] Trajectory received, forwarding to MoveIt...");

    // check trajectory validness
    ROS_INFO("[rbt_trj] Checking validness of the trajectory...");
    std::string trj_ref_frame = trajectory->header.frame_id;
    int robot_dim = trajectory->robot_dim;
    if ((trajectory->trajectory.size() % robot_dim) != 0) {
      ROS_ERROR("[rbt_trj] Trajectory size doesn't match trajectory dimension!");
      return;
    } else {
      ROS_INFO("[rbt_trj] Trajectory valid.");
    }

    // compose transformation matrix
    ROS_INFO("[rbt_trj] Looking for transformation matrix from %s to %s.", trj_ref_frame.c_str(), this->target_frame.c_str());
    tf::StampedTransform work2rob;
    Eigen::Affine3d transformer;
    try {
      this->tf_listener.waitForTransform(this->target_frame, trj_ref_frame, ros::Time::now(), ros::Duration(4.f));
      this->tf_listener.lookupTransform(this->target_frame, trj_ref_frame, ros::Time::now(), work2rob);
    } catch (tf::LookupException &err) {
      ROS_ERROR("[rbt_trj] Lookup [Work Frame --> Robot Frame] falied.");
      return;
    } catch (tf::ExtrapolationException &err) {
      ROS_ERROR("[rbt_trj] Lookup [Work Frame --> Robot Frame] falied.");
      return;
    } catch (tf::TransformException &err) {
      ROS_ERROR("[rbt_trj] Lookup [Work Frame --> Robot Frame] falied.");
      return;
    }
    tf::transformTFToEigen(work2rob, transformer);
    ROS_INFO("[rbt_trj] Done.");

    // move to the start-approaching point
    ROS_INFO("[rbt_trj] Approaching start point...");
    this->manipulator.setJointValueTarget(this->start_approach);

    moveit::planning_interface::MoveGroup::Plan exe_plan;
    if (this->manipulator.plan(exe_plan)) {
      // got a plan, now execute
      bool success = this->manipulator.execute(exe_plan);
      if (success)  {ROS_INFO("[rbt_trj] Done.");}
      else          {ROS_ERROR("[rbt_trj] Failed to approach start point!"); return;}
    } else {
      ROS_ERROR("[rbt_trj] No plan generated for approaching start point!");
      return;
    }

    // cartesian plan
    geometry_msgs::Pose waypoint;
    {
      boost::shared_lock<boost::shared_mutex> read_lock(this->pose_update_lock);
      waypoint = this->current_pose;
    }
    ROS_INFO("[rbt_trj] Currently @ P(%lf, %lf, %lf).", waypoint.position.x, waypoint.position.y, waypoint.position.z);

    Eigen::Affine3d scaler = Eigen::Affine3d::Identity();
    scaler.scale(0.001);

    int trj_length = trajectory->trajectory.size() / robot_dim;
    std::vector<float> trj = trajectory->trajectory;

    std::vector<geometry_msgs::Pose> waypoints;
    for (int trj_index = 0; trj_index < trj_length; trj_index ++) {
      // move to each waypoint
      Eigen::Vector3d point(trj[trj_index * robot_dim], trj[trj_index * robot_dim + 1], trj[trj_index * robot_dim + 2]);
      point = transformer * scaler * point;
      waypoint.position.x = point(0);
      waypoint.position.y = point(1);
      waypoint.position.z = point(2);
      waypoints.push_back(waypoint);
    }

    ROS_INFO("[rbt_trj] Computing waypoints for the rest of the trajectory...");
    moveit_msgs::RobotTrajectory actions;
    double fraction = this->manipulator.computeCartesianPath(waypoints, 0.015, 0.0, actions);
    ROS_INFO("[rbt_trj] Done. (%.2f%% acheived)", fraction * 100.0);

    ROS_INFO("[rbt_trj] Executing the trajectory...");
    moveit::planning_interface::MoveGroup::Plan exe_plan_2;
    exe_plan_2.trajectory_ = actions;
    bool success = this->manipulator.execute(exe_plan_2);
    if (success) {ROS_INFO("[rbt_trj] Done.");}
    else {ROS_ERROR("[rbt_trj] Failed to move according to the trajectory..."); return;}

    // move to the ending pose
    {
      boost::shared_lock<boost::shared_mutex> read_lock(this->pose_update_lock);
      waypoint = this->current_pose;
    }
    ROS_INFO("[rbt_trj] Currently @ P(%lf, %lf, %lf).", waypoint.position.x, waypoint.position.y, waypoint.position.z);
    ROS_INFO("[rbt_trj] Returning to initial position...");
    waypoint.position.z = 0.20740845427;
    this->manipulator.setPoseTarget(waypoint);
    if (this->manipulator.plan(exe_plan)) {
      // got a plan, now execute
      bool success = this->manipulator.execute(exe_plan);
      if (!success) {ROS_ERROR("[rbt_trj] Failed to return to initial point!"); return;}
    } else {
      ROS_ERROR("[rbt_trj] No plan generated for returning to initial point!");
      return;
    }
    for (int end_approach_index = 0; end_approach_index < this->end_approach.size(); end_approach_index ++) {
      this->manipulator.setJointValueTarget(this->end_approach[end_approach_index]);

      if (this->manipulator.plan(exe_plan)) {
        bool success = this->manipulator.execute(exe_plan);
        if (!success) {ROS_ERROR("[rbt_trj] Failed to return to initial point!"); return;}
      } else {
        ROS_ERROR("[rbt_trj] No plan generated for end pose!");
        return;
      }
    }
    ROS_INFO("[rbt_trj] Done.");
    ROS_INFO("[rbt_trj] Layer Done. Waiting for the next frame...");

    return;
  }

  void publish_eef_pose(void) {
    geometry_msgs::PoseStamped cur_eef_pose = this->manipulator.getCurrentPose();
    {
      boost::upgrade_lock<boost::shared_mutex> lock(this->pose_update_lock);
      boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(lock);
      this->current_pose = cur_eef_pose.pose;
    }
    this->eef_pose.publish(cur_eef_pose);
    return;
  }
};

int main(int argc, char **argv) {
  // initialize the node
  ros::init(argc, argv, "robot_interface");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // bring up the node
  RobotTrajectoryInterface robot_interface;

  // do the publishing here
  ros::Rate rate(100);
  while (ros::ok()) {
    robot_interface.publish_eef_pose();
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
