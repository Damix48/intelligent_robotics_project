#include <ros/ros.h>

#include "tiago_iaslab_simulation/head_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "head_node");

  auto nh_ptr = std::make_shared<ros::NodeHandle>();

  HeadServer headServer(nh_ptr);

  ros::spin();

  return 0;
}
