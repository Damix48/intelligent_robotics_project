#include <ros/ros.h>

#include "tiago_iaslab_simulation/move_client.h"
#include "tiago_iaslab_simulation/map2d.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "client_node");

  Map2D map("map");

  float x_;
  float y_;
  float yaw_;
  std::string frameId_;

  if (argc == 4) {
    x_ = atof(argv[1]);
    y_ = atof(argv[2]);
    yaw_ = atof(argv[3]);
  } else {
    do {
      printf("Enter your position goal x: ");
      scanf("%f", &x_);
      printf("Enter your position goal y: ");
      scanf("%f", &y_);
      printf("Enter your orientation goal yaw: ");
      scanf("%f", &yaw_);

      if (!map.isValidPoint(x_, y_)) {
        printf("Invalid point");
      }
    } while (!map.isValidPoint(x_, y_));
  }

  if (!map.isValidPoint(x_, y_)) {
    return EXIT_FAILURE;
  }

  auto nh_ptr = std::make_shared<ros::NodeHandle>();

  MoveClient client(nh_ptr);
  client.moveTo(x_, y_, yaw_);

  ros::spin();

  return EXIT_SUCCESS;
}
