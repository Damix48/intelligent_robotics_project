#include <ros/ros.h>

#include "tiago_iaslab_simulation/arm_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "arm_node");

  auto nh_ptr = std::make_shared<ros::NodeHandle>();

  ArmServer armServer(nh_ptr);

  ros::spin();

  return 0;
}
