#include <ros/ros.h>

#include "tiago_iaslab_simulation/move_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_node");

  auto nh_ptr = std::make_shared<ros::NodeHandle>();

  MoveServer moveServer(nh_ptr, "move_server");

  ros::spin();

  return 0;
}
