#ifndef TIAGO_IASLAB_SIMULATION_MOVE_SERVER_H
#define TIAGO_IASLAB_SIMULATION_MOVE_SERVER_H

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <std_msgs/UInt8.h>

#include "tiago_iaslab_simulation/moveScanAction.h"

class MoveServer {
 private:
  std::shared_ptr<ros::NodeHandle> nodeHandle;

  actionlib::SimpleActionServer<tiago_iaslab_simulation::moveScanAction> actionServer;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveActionClient;

  ros::ServiceClient scannerClient;

  ros::Subscriber feedbackRelay;

  /// @brief Callback for the action server execution.
  /// @param goal
  void move(const tiago_iaslab_simulation::moveScanGoalConstPtr& goal);
  /// @brief Scan for obstacles.
  void scan();

  /// @brief Publish the feedback, the status of the task.
  /// @param status
  void publishFeedback(const uint status);
  /// @brief Relay a feedback to the client action subscriber.
  /// @param feedback
  void feedbackRelayCallback(const std_msgs::UInt8ConstPtr& feedback);

 public:
  /// @brief Server that move the robot and controll the scanner.
  /// @param nodeHandle_
  /// @param moveServerTopic topic for the action server
  /// @param moveBaseTopic topic for send the goal to \c move_base
  /// @param scannerTopic topic for the service #ScannerServer
  /// @param feedbackTopic topic for getting feedback from other node
  MoveServer(std::shared_ptr<ros::NodeHandle> nodeHandle_,
             std::string moveServerTopic = "move_server",
             std::string moveBaseTopic = "move_base",
             std::string scannerTopic = "scan_obstacles",
             std::string feedbackTopic = "move_server_feedback");
};

#endif  // TIAGO_IASLAB_SIMULATION_MOVE_SERVER_H