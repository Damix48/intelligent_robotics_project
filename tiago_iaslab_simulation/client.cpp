#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

#include "tiago_iaslab_simulation/map2d.h"
#include "tiago_iaslab_simulation/moveScanAction.h"
#include "tiago_iaslab_simulation/utils.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "client");

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

  actionlib::SimpleActionClient<tiago_iaslab_simulation::moveScanAction> actionClient("move_server");

  actionClient.waitForServer();

  tiago_iaslab_simulation::moveScanGoal goal;

  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = ros::Time::now();
  goal.pose.pose = iaslab::createPose(x_, y_, yaw_);

  actionClient.sendGoal(goal);

  actionClient.waitForResult();

  ros::spin();

  return 0;
}
