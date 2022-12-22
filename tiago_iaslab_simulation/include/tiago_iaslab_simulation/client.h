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

  /// @brief Callback to process the result from the #MoveServer action.
  /// @param state
  /// @param result
  void doneCallback(const actionlib::SimpleClientGoalState& state,
                    const tiago_iaslab_simulation::moveScanResultConstPtr& result);
  /// @brief Callback for disaply the feedback from the #MoveServer action.
  /// @param feedback
  void feedbackCallback(const tiago_iaslab_simulation::moveScanFeedbackConstPtr& feedback);

 public:
  /// @brief
  /// @param nodeHandle_
  /// @param moveServerTopic topic for send the goal to #MoveServer action
  /// @param visualizerTopic topic for publish the marker of centers
  Client(std::shared_ptr<ros::NodeHandle> nodeHandle_,
         std::string moveServerTopic = "move_server",
         std::string visualizerTopic = "visualization_marker");

  void sendPose(float x, float y, float yaw);
};

#endif  // CLIENT_H