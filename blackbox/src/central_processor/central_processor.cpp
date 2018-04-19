/*
 * Copyright @ CERLAB
 * This file is the implementation of blackbox
 * node.
 */

#include <central_processor.h>

void CentralProcessor::processCb(const sensor_msgs::PointCloud2ConstPtr &frame) {
  // point cloud transformation from Camera Frame --> Robot/Work Frame
  tf::StampedTransform cam2rob;
  std::string fixed_frame = frame->header.frame_id;
  try {
    this->tf_listener.waitForTransform(this->target_frame, fixed_frame, ros::Time::now(), ros::Duration(4.f));
    this->tf_listener.lookupTransform(this->target_frame, fixed_frame, ros::Time::now(), cam2rob);
  } catch (tf::LookupException &err) {
    ROS_ERROR("[blackbox] Lookup [Camera Frame --> Robot/Work Frame] falied.");
    return;
  } catch (tf::ExtrapolationException &err) {
    ROS_ERROR("[blackbox] Lookup [Camera Frame --> Robot/Work Frame] falied.");
    return;
  } catch (tf::TransformException &err) {
    ROS_ERROR("[blackbox] Lookup [Camera Frame --> Robot/Work Frame] falied.");
    return;
  }

  // transfer from PointCloud2(ROS) --> PointCloud(PCL)
  PointCloudPtr input_cloud(new PointCloud);
  pcl::fromROSMsg(*frame, *input_cloud);
  ROS_INFO("[processing] Received point cloud [%d x %d].", input_cloud->width, input_cloud->height);

  // transform point cloud into work frame
  PointCloudPtr processed(new PointCloud);
  Eigen::Affine3d transformer;
  tf::transformTFToEigen(cam2rob, transformer);
  pcl::transformPointCloud(*input_cloud, *processed, transformer);

  // point cloud static outlier remover
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(processed);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*processed);

  // ROI
  processed = this->pcl_passthrough(processed);
  ROS_INFO("[processing] ROI passed: point cloud reduced to [%d x %d].", processed->width, processed->height);

  // Show intermediate result
  sensor_msgs::PointCloud2 debug_cloud;
  pcl::toROSMsg(*processed, debug_cloud);
  debug_cloud.header.frame_id = this->target_frame;
  this->_debug_window.publish(debug_cloud);

  // normal estimator (if enabled)
  // normal estimator should ONLY be enabled when calibrating the camera pose
  if (this->normal_estimator) {
    ROS_INFO("[normal_estimator] Estimating surface normal...");
    this->pcl_normal_estimation(processed);

    // since we are in calibration mode, no following processing is needed
    ROS_WARN("[normal_estimator] In normal estimation mode, processing ABORTED.");
    return;
  }

  // save processed point cloud (if enabled)
  // this is especially for debugging the bayes filter
  if (this->save_debug_pc) {
    // construct the save path
    std::string save_path = ros::package::getPath("blackbox");
    save_path += "/data/processed_pc/frame_" + boost::lexical_cast<std::string>(this->seq) + ".pcd";

    // save this frame
    pcl::io::savePCDFileASCII(save_path, *processed);
    ROS_INFO("[processing] processed point cloud saved to %s.", save_path.c_str());
  }

  // pass through bayes_filter
  if (!this->bayes_filter.exists()) {
    ROS_WARN("[processing] Bayes filter is offline. Skip bayes update.");
  } else {
    blackbox::BayesFilter filter;
    filter.request.header.stamp = ros::Time::now();
    filter.request.header.frame_id = this->target_frame;
    filter.request.header.seq = this->seq;
    filter.request.width = processed->width;
    filter.request.height = processed->height;
    filter.request.measurement = debug_cloud;
    // bayes filter uses (mm) while we are operating in (m)
    // it is our responsibility to tell bayes filter that
    filter.request.scale = 1000;

    if (!this->bayes_filter.call(filter)) {
      ROS_ERROR("[processing] Failed to call BayesFilter. :(");
    } else {
      ROS_INFO("[processing] Current frame passed to BayesFilter.");
      // TODO (Bosch): fetch the result from bayes filter
    }
  }

  // generating trajectory
  std::vector<float> trajectory;
  // transfer the cloud from (m) to (mm)
  Eigen::Affine3d scaler = Eigen::Affine3d::Identity();
  scaler.scale(1000);
  pcl::transformPointCloud(*processed, *processed, scaler);
  this->path_from_grid(processed, trajectory);
  ROS_INFO("[processing] Path generated.");

  // send trajectory to manipulator and debug node (if any exists)
  blackbox::Trajectory trj_to_robot;
  trj_to_robot.header.seq = this->seq;
  trj_to_robot.header.frame_id = this->target_frame;
  trj_to_robot.header.stamp = ros::Time::now();

  trj_to_robot.relative = true;
  trj_to_robot.robot_dim = 6;
  trj_to_robot.trajectory = trajectory;

  this->_debug_traj.publish(trj_to_robot);
  this->to_robot.publish(trj_to_robot);
  ROS_INFO("[processing] Trajectory sent.");

  // update sequence counter
  seq += 1;

  return;
}

CentralProcessor::PointCloudPtr CentralProcessor::pcl_passthrough(PointCloudPtr cloud) {
  if (cloud->size() <= 0) {
    ROS_ERROR("[pcl_passthrough] Empty point cloud received.");
    return PointCloudPtr();
  }

  PointCloudPtr filteredZ(new PointCloud), filteredZY(new PointCloud);
  PointCloudPtr processed(new PointCloud);
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  // filter Z-axis (only if we are NOT in calibration mode)
  if (!this->normal_estimator) {
    passthrough.setInputCloud(cloud);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(this->lowerZ, this->upperZ);
    passthrough.filter(*filteredZ);
  } else {
    ROS_WARN("[pcl_passthrough] In calibration mode, skip filtering Z value.");
  }
  // filter Y-axis
  if (!this->normal_estimator) {
    passthrough.setInputCloud(filteredZ);
  } else {
    passthrough.setInputCloud(cloud);
  }
  passthrough.setFilterFieldName("y");
  passthrough.setFilterLimits(this->lowerY, this->upperY);
  passthrough.filter(*filteredZY);
  // filter X-axis
  passthrough.setInputCloud(filteredZY);
  passthrough.setFilterFieldName("x");
  passthrough.setFilterLimits(this->lowerX, this->upperX);
  passthrough.filter(*processed);

  return processed;
}

bool CentralProcessor::pcl_normal_estimation(PointCloudPtr cloud) {
  if (cloud->size() <= 0) {
    ROS_ERROR("[pcl_normal_estimation] Empty point cloud received.");
    return false;
  }

  // normal estimation here
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> estimator;
  estimator.setInputCloud(cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
  estimator.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  estimator.setRadiusSearch(0.03);
  estimator.compute(*cloud_normals);

  ROS_INFO("[pcl_normal_estimation] Retrived %ld normals from the point cloud.", cloud_normals->points.size());
  for (pcl::PointCloud<pcl::Normal>::const_iterator it = cloud_normals->begin(); it != cloud_normals->end(); ++ it) {
    printf("[%lf, %lf, %lf]\n", it->normal_x, it->normal_y, it->normal_z);
  }

  return true;
}

bool CentralProcessor::path_from_grid(const PointCloudPtr &cloud, std::vector<float> &trajectory) {
  // check the input cloud //
  if (!cloud || cloud->empty()) {
    ROS_ERROR("[pcl_passthrough] Input cloud invalid!");
    return false;
  }

  // construct the grid //
  std::vector<std::vector<float> > grid;
  // bounding box
  float XRange[2] = {cloud->points[0].x, cloud->points[0].x};
  float YRange[2] = {cloud->points[0].y, cloud->points[0].y};
  float LayerBottom = cloud->points[0].z;
  for (PointCloud::const_iterator it = cloud->begin() + 1; it != cloud->end(); ++ it) {
    if (it->x > XRange[1]) {XRange[1] = it->x;}
    else if (it->x < XRange[0]) {XRange[0] = it->x;}

    if (it->y > YRange[1]) {YRange[1] = it->y;}
    else if (it->y < YRange[0]) {YRange[0] = it->y;}

    if (it->z < LayerBottom) {LayerBottom = it->z;}
  }
  ROS_INFO("[path_from_grid] BoundingBox: X->[%lf, %lf]\tY->[%lf, %lf]\tZMin = %lf", 
              XRange[0], XRange[1], YRange[0], YRange[1], LayerBottom);
  // recalculate step size
  int grid_width = (int)((YRange[1] - YRange[0]) / this->resY + 0.5);
  int grid_length = (int)((XRange[1] - XRange[0]) / this->resX + 0.5);
  this->resY = (YRange[1] - YRange[0]) / grid_width;
  this->resX = (XRange[1] - XRange[0]) / grid_length;
  // initialize the grid
  for (int step_x = 0; step_x < grid_length; step_x ++) {
    std::vector<float> row; row.clear();
    for (int step_y = 0; step_y < grid_width; step_y ++) {
      float center_x = XRange[0] + step_x * this->resX + 0.5 * this->resX;
      float center_y = YRange[0] + step_y * this->resY + 0.5 * this->resY;
      row.push_back(center_x);
      row.push_back(center_y);
      row.push_back(LayerBottom);
      row.push_back(0.f);
      row.push_back(0.f);
      row.push_back(0.f);
    }
    grid.push_back(row);
  }
  ROS_INFO("[path_from_grid] Grid [%d x %d] initialized", grid_length, grid_width);

  // update the grid //
  for (PointCloud::const_iterator it = cloud->begin(); it != cloud->end(); ++ it) {
    // calculate the cooresponding index in the grid
    int index_x = (int)((it->x - XRange[0]) / this->resX);
    int index_y = (int)((it->y - YRange[0]) / this->resY);
    if (index_x == grid_length) {index_x -= 1;}
    if (index_y == grid_width) {index_y -= 1;}

    // update the grid
    if (it->z >= grid[index_x][6 * index_y + 2]) {
      grid[index_x][6 * index_y + 2] = (it->z + this->EEFOffset + this->kinectOffsetZ);
    }
  }

  // path smoothing and offsetting //
  for (int index_row = 0; index_row < grid.size(); index_row ++) {
    int num_of_points = grid[index_row].size() / 6;
    for (int index_col = 0; index_col < num_of_points; index_col ++) {
      // check whether this area is not updated
      if (grid[index_row][index_col * 6 + 2] == LayerBottom) {
        // OK, am I at the start of the line?
        if (index_col == 0) {
          grid[index_row][index_col * 6 + 2] = grid[index_row][(index_col + 1) * 6 + 2];
        } else if (index_col == (num_of_points - 1)) {
          grid[index_row][index_col * 6 + 2] = grid[index_row][(index_col - 1) * 6 + 2];
        } else {
          double front_point = grid[index_row][(index_col + 1) * 6 + 2];
          double rear_point = grid[index_row][(index_col - 1) * 6 + 2];
          grid[index_row][index_col * 6 + 2] = (front_point + rear_point) / 2.0;
        }
      }
    }
  }

  // formulate the path
  bool need_reverse = false;
  trajectory.clear();
  for (int index_row = 0; index_row < grid.size(); index_row ++) {
    if (need_reverse) {
      // we need to reverse in the space of points
      for (std::vector<float>::const_iterator inserter = grid[index_row].end(); inserter != grid[index_row].begin(); inserter -= 6) {
        trajectory.insert(trajectory.end(), (inserter - 6), inserter);
      }
    } else {
      trajectory.insert(trajectory.end(), grid[index_row].begin(), grid[index_row].end());
    }
    need_reverse = !need_reverse;
  }
  ROS_INFO("[path_from_grid] Trajectory [%ld x 6] generated.", (trajectory.size() / 6));

  // Done //
  ROS_INFO("[path_from_grid] Done.");

  return true;
}
