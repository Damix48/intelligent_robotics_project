#include "tiago_iaslab_simulation/client.h"

#include "tiago_iaslab_simulation/moveScanAction.h"
#include "tiago_iaslab_simulation/status_constant.h"
#include "tiago_iaslab_simulation/utils.h"

Client::Client(std::shared_ptr<ros::NodeHandle> nodeHandle_, std::string moveServerTopic) : nodeHandle(nodeHandle_),
                                                                                            actionClient(moveServerTopic) {}

void Client::doneCallback(const actionlib::SimpleClientGoalState& state,
                          const tiago_iaslab_simulation::moveScanResultConstPtr& result) {
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("There are %i obstacles.", result->obstacles.size());

    for (size_t i = 0; i < result->obstacles.size(); i++) {
      ROS_INFO("Obstacle found in (x, y) = (%f, %f).", result->obstacles[i].point.x, result->obstacles[i].point.y);
    }

  } else {
    ROS_ERROR("The action is aborted.");
  }
}

void Client::feedbackCallback(const tiago_iaslab_simulation::moveScanFeedbackConstPtr& feedback) {
  switch (feedback->current_status) {
    case status::READY:
      ROS_INFO("The move server is ready.");
      break;

    case status::MOVING:
      ROS_INFO("The robot is moving using move_base.");
      break;

    case status::MOVING_CORRIDOR:
      ROS_INFO("The robot is moving using the corridor motion law.");
      break;

    case status::ARRIVED:
      ROS_INFO("The robot is arrived in the target location.");
      break;

    case status::NOT_ARRIVED:
      ROS_WARN("The robot is NOT arrived in the target location.");
      break;

    case status::SCANNING:
      ROS_INFO("The robot is scanning the obstacles.");
      break;

    case status::DONE:
      ROS_INFO("The robot scanned the enviroment and complete its task.");
      break;

    case status::FAILED:
      ROS_WARN("Something failed.");
      break;

    default:
      ROS_ERROR("Something very strange happened.");
      break;
  }
}

void Client::sendPose(float x, float y, float yaw) {
  actionClient.waitForServer();

  tiago_iaslab_simulation::moveScanGoal goal;

  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = ros::Time::now();
  goal.pose.pose = iaslab::createPose(x, y, yaw);

  actionClient.sendGoal(goal,
                        boost::bind(&Client::doneCallback, this, _1, _2),
                        actionlib::SimpleActionClient<tiago_iaslab_simulation::moveScanAction>::SimpleActiveCallback(),
                        boost::bind(&Client::feedbackCallback, this, _1));

  actionClient.waitForResult();
}