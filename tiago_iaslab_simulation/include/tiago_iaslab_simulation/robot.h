#ifndef TIAGO_IASLAB_SIMULATION_ROBOT_H
#define TIAGO_IASLAB_SIMULATION_ROBOT_H

#include <geometry_msgs/PoseStamped.h>
#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>

#include "tiago_iaslab_simulation/client.h"
#include "tiago_iaslab_simulation/object.h"

class Robot {
 private:
  std::shared_ptr<ros::NodeHandle> nodeHandle;
  ros::ServiceClient apriltagDetectionsService;
  Client moveScanClient;

  ros::Publisher visualizer;

  void pickRoutine();
  void placeRoutine();

 public:
  Robot(std::shared_ptr<ros::NodeHandle> nodeHandle_, std::string apriltagServiceTopic = "apriltag_poses_srv");
  void start(std::map<int, Object> objects, std::vector<int> sequence);
  void removeObject(int id);
};

#endif  // TIAGO_IASLAB_SIMULATION_ROBOT_H