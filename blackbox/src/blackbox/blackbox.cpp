/*
 * Copyright @ CERLAB
 * this node receives a single frame from the camera
 * and generate trajectory for the robot
 * CURRENT ALG: LawnMow
 */

#include <central_processor.h>

int main(int argc, char **argv) {
  // initialize the node
  ros::init(argc, argv, "blackbox");
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // bring up the node
  CentralProcessor blackbox;

  // do nothing here
  ros::waitForShutdown();
  spinner.stop();

  return 0;
}
