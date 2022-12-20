#include <ros/ros.h>

#include "tiago_iaslab_simulation/corridor_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "corridor_node");

  CorridorServer corridorServer(1.1);

  ros::spin();

  return 0;
}
