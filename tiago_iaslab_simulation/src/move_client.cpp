#include "tiago_iaslab_simulation/move_client.h"

#include <visualization_msgs/Marker.h>

#include "tiago_iaslab_simulation/circle.h"
#include "tiago_iaslab_simulation/constant.h"
#include "tiago_iaslab_simulation/moveScanAction.h"
#include "tiago_iaslab_simulation/utils.h"

MoveClient::MoveClient(std::shared_ptr<ros::NodeHandle> nodeHandle_,
               bool print_,
               std::string moveServerTopic,
               std::string visualizerTopic) : nodeHandle(nodeHandle_),
                                              actionClient(moveServerTopic),
                                              print(print_) {
  visualizer = nodeHandle->advertise<visualization_msgs::Marker>(visualizerTopic, 1);
}

void MoveClient::doneCallback(const actionlib::SimpleClientGoalState& state,
                          const tiago_iaslab_simulation::moveScanResultConstPtr& result) {
  obstacles = result->obstacles;

  if (!print) {
    return;
  }

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("There are %i obstacles.", result->obstacles.size());

    for (size_t i = 0; i < obstacles.size(); i++) {
      tiago_iaslab_simulation::circle obstacle = obstacles[i];

      ROS_INFO("Obstacle found in (x, y) = (%f, %f).", obstacle.center.point.x, obstacle.center.point.y);

      visualization_msgs::Marker center;

      center.action = center.ADD;
      center.header = obstacle.center.header;
      center.ns = "centers";
      center.id = i;
      center.type = center.SPHERE;

      center.pose.position = obstacle.center.point;

      center.scale.x = 0.1;
      center.scale.y = 0.1;
      center.scale.z = 0.1;

      center.color.r = 0.0f;
      center.color.g = 1.0f;
      center.color.b = 0.0f;
      center.color.a = 1.0;

      center.lifetime = ros::Duration(0);

      visualizer.publish(center);
    }

  } else {
    ROS_ERROR("The action is aborted.");
  }
}

void MoveClient::feedbackCallback(const tiago_iaslab_simulation::moveScanFeedbackConstPtr& feedback) {
  if (!print) {
    return;
  }

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

bool MoveClient::moveTo(float x, float y, float yaw, bool scan) {
  geometry_msgs::PoseStamped pose;

  pose.header.frame_id = "map";
  pose.header.stamp = ros::Time::now();
  pose.pose = iaslab::createPose(x, y, yaw);

  return moveTo(pose, scan);
}

bool MoveClient::moveTo(geometry_msgs::PoseStamped pose, bool scan) {
  actionClient.waitForServer();

  tiago_iaslab_simulation::moveScanGoal goal;

  goal.pose = pose;
  goal.scan = scan;

  actionClient.sendGoal(goal,
                        boost::bind(&MoveClient::doneCallback, this, _1, _2),
                        actionlib::SimpleActionClient<tiago_iaslab_simulation::moveScanAction>::SimpleActiveCallback(),
                        boost::bind(&MoveClient::feedbackCallback, this, _1));

  actionClient.waitForResult();

  return actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

std::vector<tiago_iaslab_simulation::circle> MoveClient::getObstacles() const {
  return obstacles;
}