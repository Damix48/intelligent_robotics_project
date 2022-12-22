#ifndef CLIENT_H
#define CLIENT_H

#include <actionlib/client/simple_action_client.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include "tiago_iaslab_simulation/moveScanAction.h"

class Client {
 private:
  std::shared_ptr<ros::NodeHandle> nodeHandle;

  actionlib::SimpleActionClient<tiago_iaslab_simulation::moveScanAction> actionClient;

  ros::Publisher visualizer;

  void doneCallback(const actionlib::SimpleClientGoalState& state,
                    const tiago_iaslab_simulation::moveScanResultConstPtr& result);
  void feedbackCallback(const tiago_iaslab_simulation::moveScanFeedbackConstPtr& feedback);

 public:
  Client(std::shared_ptr<ros::NodeHandle> nodeHandle_,
         std::string moveServerTopic = "move_server",
         std::string visualizerTopic = "visualization_marker");

  void sendPose(float x, float y, float yaw);
};

#endif  // CLIENT_H