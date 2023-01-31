#include <ros/ros.h>

#include "tiago_iaslab_simulation/corridor_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "corridor_node");

  auto nh_ptr = std::make_shared<ros::NodeHandle>();

  CorridorServer corridorServer(nh_ptr, 1.5);

  ros::spin();

  return 0;
}
