#ifndef TIAGO_IASLAB_SIMULATION_HEAD_CLIENT_H
#define TIAGO_IASLAB_SIMULATION_HEAD_CLIENT_H

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include "tiago_iaslab_simulation/headAction.h"

class HeadClient {
 private:
  std::map<int, geometry_msgs::Pose> tags;

  std::shared_ptr<ros::NodeHandle> nodeHandle;

  actionlib::SimpleActionClient<tiago_iaslab_simulation::headAction> actionClient;
  actionlib::SimpleActionClient<control_msgs::PointHeadAction> pointHeadClient;

  bool print;

 public:
  HeadClient(std::shared_ptr<ros::NodeHandle> nodeHandle_,
             bool print_ = true,
             std::string headServerTopic = "head_server",
             std::string pointHeadTopic = "head_controller/point_head_action");

  bool move(float pitch, float yaw, bool getObject = true);
  bool move(geometry_msgs::PointStamped target);
  std::map<int, geometry_msgs::Pose> getTags();
};

#endif  // TIAGO_IASLAB_SIMULATION_HEAD_CLIENT_H