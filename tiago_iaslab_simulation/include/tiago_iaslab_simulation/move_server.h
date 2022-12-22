#ifndef TIAGO_IASLAB_SIMULATION_MOVE_SERVER_H
#define TIAGO_IASLAB_SIMULATION_MOVE_SERVER_H

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/node_handle.h>
#include <ros/service_client.h>

#include "tiago_iaslab_simulation/moveScanAction.h"

class MoveServer {
 private:
  std::shared_ptr<ros::NodeHandle> nodeHandle;

  actionlib::SimpleActionServer<tiago_iaslab_simulation::moveScanAction> actionServer;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveActionClient;

  ros::ServiceClient scannerClient;

  void move(const tiago_iaslab_simulation::moveScanGoalConstPtr& goal);

  void doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& moveBaseResult);
  void publishFeedback(const uint status);

 public:
  MoveServer(std::shared_ptr<ros::NodeHandle> nodeHandle_,
             std::string moveServerTopic = "move_server",
             std::string moveBaseTopic = "move_base",
             std::string scannerTopic = "scan_obstacles");
};

#endif  // TIAGO_IASLAB_SIMULATION_MOVE_SERVER_H