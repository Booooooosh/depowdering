/*
 * Copyright @ CERLAB
 * this is a MoveIt interface for robot trajectory
 * execution
 */

#include <vector>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

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
#include <std_msgs/Int32.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>

#include <cam_kinect/FetchOneFrame.h>
#include <blackbox/Trajectory.h>
#include <robot_interface/IO_Control.h>
#include <robot_interface/FollowEdge.h>

#include <dynamic_reconfigure/server.h>
#include <robot_interface/RobotInterfaceConfig.h>

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
  // waypoints interval
  double cartesian_interval;
  // cartesian path velocity factor
  double cartesian_VFactor;
  // cartesian path acceleration factor
  double cartesian_AccFactor;

  // cartesian path computation threading
  int numof_threads;

  // ros publisher and subscriber
  ros::NodeHandle nh, pnh;
  ros::Subscriber robot_trajectory;
  ros::ServiceServer io_controller;
  ros::ServiceServer edge_follower;
  ros::Publisher  eef_pose;
  ros::Publisher  io_writter;
  ros::Subscriber io_reader;

  // moveit interface
  moveit::planning_interface::MoveGroup manipulator;
  moveit::planning_interface::PlanningSceneInterface planning_scene;

  // IO port and EEF
  std::string robot_name;
  int io_port;

  // edge follower
  double bb_edge_len;

  // dynamic reconfiguration
  dynamic_reconfigure::Server<robot_interface::RobotInterfaceConfig> reconfig_server;

 public:
  RobotTrajectoryInterface() : manipulator("arm"), nh("robot_interface"), pnh("~") {
    ROS_INFO("[rbt_trj] Initializing robot_trajectory node...");
    // subscribe to trajectory topic
    std::string sub_topic_name = this->pnh.param<std::string>("general/trajectory_topic_name", "/blackbox/robot_trajectory");
    this->robot_trajectory = this->nh.subscribe(sub_topic_name, 1, &RobotTrajectoryInterface::trajectoryCb, this);
    // publish end effector pose
    this->eef_pose = this->nh.advertise<geometry_msgs::PoseStamped>("eef_pose", 10);
    // publish io relative services
    this->robot_name = this->pnh.param<std::string>("general/robot_name", "vs6577");
    this->io_port = this->pnh.param<int>("general/io_port", 30);
    this->io_writter = this->nh.advertise<std_msgs::Int32>(this->robot_name + "/Write_MiniIO", 5);
    this->io_controller = this->nh.advertiseService("io_controller", &RobotTrajectoryInterface::io_controllerCb, this);
    this->bb_edge_len = this->pnh.param<double>("general/buildbox_edge_len", 0.3);
    this->edge_follower = this->nh.advertiseService("edge_follower", &RobotTrajectoryInterface::edge_followerCb, this);
    // cartesian path relative
    this->cartesian_interval = 0.001;   // set from dynamic reconfiguration
    this->cartesian_VFactor = 0.2;      // set from dynamic reconfiguration
    this->cartesian_AccFactor = 0.2;    // set from dynamic reconfiguration
    this->numof_threads = this->pnh.param<int>("motion_planning/numof_threads", 2);

    // dynamic reconfiguration server setup
    dynamic_reconfigure::Server<robot_interface::RobotInterfaceConfig>::CallbackType funcHd;
    funcHd = boost::bind(&RobotTrajectoryInterface::dynamic_reconfigurationCb, this, _1, _2);
    this->reconfig_server.setCallback(funcHd);

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

    ROS_INFO("[rbt_trj] ===================================");
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
    ROS_INFO("\t\t%s", this->manipulator.getDefaultPlannerId("arm").c_str());
    // end effector name
    ROS_INFO("\t[End Effector]:");
    ROS_INFO("\t\t%s", this->manipulator.getEndEffector().c_str());
    // cartesian path computation
    ROS_INFO("[rbt_trj] Here is some info w.r.t motion planner:");
    ROS_INFO("\t[# of computation therads]:");
    ROS_INFO("\t\t%d", this->numof_threads);
    ROS_INFO("\t[Distance b/w two cartesian waypoints]");
    ROS_INFO("\t\t%lfm", this->cartesian_interval);
    ROS_INFO("[rbt_trj] ===================================");

    // finished
    ROS_INFO("[rbt_trj] Done.");
  }

  void dynamic_reconfigurationCb(robot_interface::RobotInterfaceConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure request: Waypoint interval: [%lf]", config.WaypointInterval);
    this->cartesian_interval = config.WaypointInterval;
    ROS_INFO("Reconfigure request: Velocity / Acceleration factor: [%lf / %lf]", config.VelocityFactor, config.AccFactor);
    this->cartesian_VFactor = config.VelocityFactor;
    this->cartesian_AccFactor = config.AccFactor;
  }

  bool io_controllerCb(robot_interface::IO_Control::Request &req, 
                        robot_interface::IO_Control::Response &res) {
    std_msgs::Int32 cmd;
    if (req.request == "on") {
      // turn ON the IO${this->io_port}
      cmd.data = (0x1 << this->io_port);
    } else if (req.request == "off") {
      // turn OFF
      cmd.data = 0x0;
    }

    // send the command
    ROS_INFO("[rbt_trj] Command to be sent to IO port is [%x].", cmd.data);
    this->io_writter.publish(cmd);
    ROS_INFO("[rbt_trj] Sent.");

    res.success = true;
    res.reason = "Done.";
    return true;
  }

  bool edge_followerCb(robot_interface::FollowEdge::Request &req, 
                        robot_interface::FollowEdge::Response &res) {
    // check request validness
    // offset (we impose a minimum 10mm offset) //
    double offset = 0.01;
    if (req.offset < 0.5 && req.offset > offset) {
      offset = req.offset;
    } else if (req.offset >= 0.5) {
      ROS_WARN("[bb_frame_test] Offsets should be in specified meters.");
      res.success = false;
      res.reason = "Check the offet metric.";
      return true;
    }
    // delay (2 seconds by default) //
    int delay = 2;
    if (req.delay > 0) {
      delay = req.delay;
    }

    // lookup transformation b/w work frame and robot frame
    // by default, the frame will be "robot_work_frame"
    std::string ref_frame = "robot_work_frame";
    if (req.ref_frame.length() > 0) {
      ROS_WARN("[bb_frame_test] Changing reference frame to %s.", req.ref_frame.c_str());
    } else {
      ROS_INFO("[bb_frame_test] Using default reference frame: %s.", req.ref_frame.c_str());
    }

    ROS_INFO("[bb_frame_test] Looking for transformation matrix from %s to %s.", ref_frame.c_str(), this->target_frame.c_str());
    tf::StampedTransform work2rob;
    Eigen::Affine3d transformer;
    try {
      this->tf_listener.waitForTransform(this->target_frame, ref_frame, ros::Time::now(), ros::Duration(4.f));
      this->tf_listener.lookupTransform(this->target_frame, ref_frame, ros::Time::now(), work2rob);
    } catch (tf::LookupException &err) {
      ROS_ERROR("[bb_frame_test] Lookup [Work Frame --> Robot Frame] falied.");
      res.success = false;
      res.reason = "Frame transformation not found.";
      return true;
    } catch (tf::ExtrapolationException &err) {
      ROS_ERROR("[bb_frame_test] Lookup [Work Frame --> Robot Frame] falied.");
      res.success = false;
      res.reason = "Frame transformation not found.";
      return true;
    } catch (tf::TransformException &err) {
      ROS_ERROR("[bb_frame_test] Lookup [Work Frame --> Robot Frame] falied.");
      res.success = false;
      res.reason = "Frame transformation not found.";
      return true;
    }
    tf::transformTFToEigen(work2rob, transformer);
    ROS_INFO("[bb_frame_test] Done.");

    // set test trajectory
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose waypoint;
    {
      boost::shared_lock<boost::shared_mutex> read_lock(this->pose_update_lock);
      waypoint = this->current_pose;
    }
    // waypoint 1 (0.0, 0.0, offset) //
    waypoint.position.x = 0.0; waypoint.position.y = 0.0; waypoint.position.z = offset;
    waypoints.push_back(waypoint);
    // waypoint 2 (300.0, 0.0, offset) //
    waypoint.position.x = 0.0; waypoint.position.y = this->bb_edge_len; waypoint.position.z = offset;
    waypoints.push_back(waypoint);
    // waypoint 3 (300.0, 300.0, offset) //
    waypoint.position.x = this->bb_edge_len; waypoint.position.y = this->bb_edge_len; waypoint.position.z = offset;
    waypoints.push_back(waypoint);
    // waypoint 4 (0.0, 300.0, offset) //
    waypoint.position.x = this->bb_edge_len; waypoint.position.y = 0.0; waypoint.position.z = offset;
    waypoints.push_back(waypoint);
    // waypoint 5 (0.0, 0.0, offset) //
    waypoint.position.x = 0.0; waypoint.position.y = 0.0; waypoint.position.z = offset;
    waypoints.push_back(waypoint);
    // waypoint 6 (0.0, -100.0, offset) //
    waypoint.position.x = 0.0; waypoint.position.y = -0.1; waypoint.position.z = offset;
    waypoints.push_back(waypoint);

    // transfer to robot frame and execute
    for (std::vector<geometry_msgs::Pose>::iterator it = waypoints.begin(); it != waypoints.end(); ++ it) {
      Eigen::Vector3d point(it->position.x, it->position.y, it->position.z);
      point = transformer * point;
      it->position.x = point(0);
      it->position.y = point(1);
      it->position.z = point(2);

      ROS_INFO("[bb_frame_test] Tracing edge to next point (%lf, %lf, %lf)...", it->position.x, it->position.y, it->position.z);

      // try execution
      this->manipulator.setPoseTarget(*it);

      moveit::planning_interface::MoveGroup::Plan exe_plan;
      if (this->manipulator.plan(exe_plan)) {
        // got a plan, now execute
        bool success = this->manipulator.execute(exe_plan);
        if (success)  {ROS_INFO("[bb_frame_test] Done.");}
        else          {ROS_ERROR("[bb_frame_test] Failed to approach start point!"); res.success = false; res.reason = "Execution failed :("; return true;}
      } else {
        ROS_ERROR("[bb_frame_test] No plan generated for approaching start point!");
        res.success = false;
        res.reason = "No plan found by MoveIt :(";
        return true;
      }

      // wait for a while...
      ros::Duration(delay).sleep();
    }

    // done
    ROS_INFO("[bb_frame_test] Done.");
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
    this->manipulator.setMaxVelocityScalingFactor(1.0);
    this->manipulator.setMaxAccelerationScalingFactor(1.0);

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
    ros::Time computation_begin = ros::Time::now();
    moveit_msgs::RobotTrajectory actions;
    double fraction = this->manipulator.computeCartesianPath(waypoints, this->cartesian_interval, 0.0, actions);
    ros::Duration time_elapsed = ros::Time::now() - computation_begin;
    ROS_INFO("[rbt_trj] Done. (%.2f%% acheived) [time elapsed: %lf secs]", fraction * 100.0, time_elapsed.toSec());

    ROS_INFO("[rbt_trj] Executing the trajectory...");
    moveit::planning_interface::MoveGroup::Plan exe_plan_2;
    exe_plan_2.trajectory_ = actions;
    bool success = this->manipulator.execute(exe_plan_2);
    if (success) {
      ROS_INFO("[rbt_trj] Done.");
    }
    else {
      ROS_ERROR("[rbt_trj] Failed to move according to the trajectory...");
      return;
    }

    // move to the ending pose
    {
      boost::shared_lock<boost::shared_mutex> read_lock(this->pose_update_lock);
      waypoint = this->current_pose;
    }
    ROS_INFO("[rbt_trj] Currently @ P(%lf, %lf, %lf).", waypoint.position.x, waypoint.position.y, waypoint.position.z);
    ROS_INFO("[rbt_trj] Returning to initial position...");
    waypoint.position.z = 0.20740845427;  // TODO (get rid of this megic number)
    this->manipulator.setPoseTarget(waypoint);
    this->manipulator.setMaxVelocityScalingFactor(1.0);
    this->manipulator.setMaxAccelerationScalingFactor(1.0);
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
