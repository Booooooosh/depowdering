<!-- README for Automatic Depowdering System -->
<!-- Author:      Bosch Tang -->
<!-- Email:       booooooshat1994@gmail.com -->

Automatic Depowdering System
---

# Description 
This repository is the software that controls the 3D automatic depowdering system. The entire system is built upon the Robotic operating System (ROS). The current system is able to complete one or multiple layers of depowdering, without any human interferece. However, the system can still be improved with regards to path planning, vision pipeline and execution speed. Details of the system can be found within _FinalReport.pdf_.

# Run the system
## Before bringing up the system
+ Make sure that you are using a Linux System (Ubuntu 14.04) with ROS Indigo
+ Connect the Kinect Sensor, make sure that the sensor is connected by **lsusb** command
+ Connect the manipulator using Ethernet cable, you can ping the manipulator by **ping** command. Make sure that DENSO VS6577 manipulator is used since the default model loaded by _MoveIt_ corresponds to VS6577.

## Bringup the system
There are several _bash_ scripts within the **startup** folder, simply executing:
```
./startup/start.bash
```
you will be able to bringup the entire system (vision as well as the manipulator).

OR, you can execute:
```
./startup/start.bash true
```
to bringup the system with the _Bayes Filter_. As mentioned within the report, the bayes filter might need some improvements after the end effector is changed and the depowdering process becomes more robust.

There is also another script called _sim.bash_, where you can, similarly, bringup the entire system. Howeverm this time, it's only simulation with Gazebo environment instead of connecting to the real robot. _**Warning: Gazebo might take up lots of computing resources, so make sure that you have enough RAMs and a decent GPU to do this. Also, try to upgrade your Gazebo to version 7 for a better simulation, which does not come with ROS Indigo**_.
```
./startup/sim.bash
```
You can always take a look into the bash script if you are not sure what to do.

Well, in case you only have the **src** folder and **catkin_make**d the project, you might need to bringup the system using **roslaunch**:
```
roslaunch depowdering_bringup depowdering.launch sim:=false robot_ip:=192.168.1.2 send_format:=288 recv_format:=290
```
_robot_ip_ might be different and keep the _send_format_ and _recv_format_ the same.
Similarly, to bringup the system with _Bayes Filter_:
```
roslaunch depowdering_bringup depowdering.launch sim:=false robot_ip:=192.168.1.2 send_format:=288 recv_format:=290 use_bayes_filter:=true
```

## Other modes of the system
Apart from bringing up the entire system, you might only want to calibrate the system OR simply test the vision pipeline. In these cases, you only need to bringup relative part of the system:
+ **Camera Extrinsic Matrix(Pose) Calibration**
```
roslaunch depowdering_bringup perception_test.launch camera_calibration:=true
```
+ **Test the vision system**
```
roslaunch depowdering_bringup perception_test.launch use_bayes_filter:=true
```
You can choose not to test the _Bayes Filter_ by simply turning _use_bayes_filter_ to **false**.
+ **Use previous point cloud data instead**
Sometimes you might want to use previous stored point cloud data instead of grabbing the frame from the stream provided by the camera. This might be when you collect the data beforehand and want to use them to test the _Bayes Filter_. Simply turning on the _use_sim_camera_ when launching the _perception_test.launch_ and putting all previous data into **blackbox/data/point_cloud_display/**.

## Dynamic Reconfiguration
For simpler debugging, some of the parameters can be tuned on fly. Those parameters are integrated through _Dynamic Reconfiguration_ features provided by ROS. Most of them have detailed explaination when you open the GUI of the _Dynamic Reconfiguration_ by (**Note that you have to launch the system before doing this**):
```
rosrun rqt_reconfigure rqt_reconfigure
```
Some of the key parameters will be explained here:
+ **robot_interface::Iter**: This controls how many iterations will the manipulator perform before it stops. Since the system is still being developped, the default value is 1, meaning that the system will execute one iteration at one time. You can set this value to 0 to set the system into infinite loop mode (**This is not recommendded since we don't have a method to detect when to stop yet**), or else you can set it to, say 5, so that the system will depowder 5 layers before it stops.
+ **robot_interface::VelocityFactor / AccFactor**: Currently, these two are not used. These two parameters are intended to change the execution speed on the fly. However, _MoveIt_ does not support this feature when _Cartesian Path Planning_ is involved.
+ **robot_interface::WaypointInterval**: This is one of the parameters that get passed into the _Cartesian Path Planning_ function. This parameter controls the maximum jump between two consecutive waypoints. As shown in _Table 1_ in _FinalReport.pdf_, this value largely affects the time required to plan the cartesian path.

# System Services
## Trigger one iteration
The system is still under development so by default, we still need the user to call a specific service in order to trigger the next iteration:
```
rosservice call /kinect_feedback/next_frame "cycle_done: true"
```
However, you can always set **robot_interface::Iter** to somewhere above 1 in order for the system to execute multiple iterations automatically.

## Calibrate camera (when in Camera Extrinsic Matrix(Pose) Calibration mode)
To get the calibrated transformation between _camera_link_ frame and the _world_ frame:
```
rosservice call /ar_pose_filter/calculate "press_enter: {}"
```
As this will return the rotation matrix OR transformation matrix depending on what type of calibration you required from the _ar_marker_filter.py_ which located in **cam_kinect/nodes/ar_marker_filter.py**. There are other paramteres you can play around with in _ar_marker_filter.py_, for details please look at the comments within _ar_marker_filter.py_.

# A outline of what each package does
Here is a description of what each package does within this repository:
1. **blackbox**: Point cloud processing unit, process the incoming point cloud and generate trajectory for the manipulator.
2. **cam_kinect**: A wrapper for **freenect_stack**
3. **denso_robot_ros**: ROS driver for connecting to different types of industrial manipulators provided by DENSO.
4. **depowdering_bringup**: Includes some top-level roslaunch files that launches the entire system.
5. **robot_interface**: An interface to _MoveIt_, do some post-processing of the trajectory and forward it to _MoveIt_ for execution.
6. **robot_wrapper**: Outdated. Originally this is the python interface to connect DNESO manipulators, replaced by _robot_interface_ and _MoveIt_.
