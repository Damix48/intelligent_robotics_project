#include "tiago_iaslab_simulation/move_server.h"

#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_ros/transform_listener.h>

#include "tiago_iaslab_simulation/scanObstacles.h"
#include "tiago_iaslab_simulation/status_constant.h"

MoveServer::MoveServer(std::shared_ptr<ros::NodeHandle> nodeHandle_,
                       std::string moveServerTopic,
                       std::string moveBaseTopic,
                       std::string scannerTopic) : nodeHandle(nodeHandle_),
                                                   actionServer(*nodeHandle, moveServerTopic, boost::bind(&MoveServer::move, this, _1), false),
                                                   moveActionClient(moveBaseTopic) {
  scannerClient = nodeHandle->serviceClient<tiago_iaslab_simulation::scanObstacles>(scannerTopic);

  actionServer.start();

  publishFeedback(status::READY);
}

void MoveServer::move(const tiago_iaslab_simulation::moveScanGoalConstPtr& goal) {
  moveActionClient.waitForServer();

  move_base_msgs::MoveBaseGoal moveGoal;
  moveGoal.target_pose = goal->pose;

  moveActionClient.sendGoal(moveGoal, boost::bind(&MoveServer::doneCallback, this, _1, _2));
  publishFeedback(status::MOVING);

  bool timeout = moveActionClient.waitForResult(ros::Duration(300));

  if (!timeout) {
    publishFeedback(status::FAILED);
    actionServer.setAborted();
  }
}

void MoveServer::doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& moveBaseResult) {
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    publishFeedback(status::ARRIVED);

    if (scannerClient.exists()) {
      tiago_iaslab_simulation::scanObstacles service;

      publishFeedback(status::SCANNING);

      if (scannerClient.call(service)) {
        publishFeedback(status::DONE);

        tiago_iaslab_simulation::moveScanResult result;
        result.obstacles = service.response.obstacles;

        actionServer.setSucceeded(result);
      }
    }
  } else {
    publishFeedback(status::NOT_ARRIVED);
  }

  publishFeedback(status::FAILED);
  actionServer.setAborted();
}

void MoveServer::publishFeedback(const uint status) {
  tiago_iaslab_simulation::moveScanFeedback feedback;
  feedback.current_status = status;

  actionServer.publishFeedback(feedback);
}
