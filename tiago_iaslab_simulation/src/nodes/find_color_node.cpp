#include <ros/ros.h>

#include "tiago_iaslab_simulation/Colors.h"
#include "tiago_iaslab_simulation/find_color_server.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "find_color_node");
  auto nh_ptr = std::make_shared<ros::NodeHandle>();

  if (argc != 3) {
    ROS_ERROR("Usage: executable, int tolerance, int crop_size");
    return 1;
  }

  int tol = atol(argv[1]);
  int crop = atol(argv[2]);

  FindColorServer findColorServer(nh_ptr, tol, crop);
  ros::spin();

  return 0;
}