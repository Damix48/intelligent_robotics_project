#include <ros/ros.h>

#include "tiago_iaslab_simulation/scanner_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "scanner_node");

  auto nh_ptr = std::make_shared<ros::NodeHandle>();

  ScannerServer scannerServer(nh_ptr);

  ros::spin();

  return 0;
}
