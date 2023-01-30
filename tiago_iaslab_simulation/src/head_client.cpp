#include "tiago_iaslab_simulation/head_client.h"

HeadClient::HeadClient(std::shared_ptr<ros::NodeHandle> nodeHandle_,
                       bool print_,
                       std::string headServerTopic) : nodeHandle(nodeHandle_),
                                                      actionClient(headServerTopic),
                                                      print(print_) {}

bool HeadClient::move(float pitch, float yaw, bool getObject) {
  actionClient.waitForServer();

  tiago_iaslab_simulation::headGoal goal;
  goal.pitch = pitch;
  goal.yaw = yaw;
  goal.return_object = getObject;

  actionClient.sendGoal(goal);

  actionClient.waitForResult();

  for (const auto& tag : actionClient.getResult()->tags) {
    tags[tag.id[0]] = tag.pose.pose.pose;
  }

  return actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

std::map<int, geometry_msgs::Pose> HeadClient::getTags() {
  return tags;
}
