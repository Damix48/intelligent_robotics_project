#ifndef TIAGO_IASLAB_SIMULATION_HEAD_SERVER_H
#define TIAGO_IASLAB_SIMULATION_HEAD_SERVER_H

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "tiago_iaslab_simulation/headAction.h"

class HeadServer {
 private:
  std::shared_ptr<ros::NodeHandle> nodeHandle;

  actionlib::SimpleActionServer<tiago_iaslab_simulation::headAction> actionServer;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> headController;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torsoController;

  std::vector<std::string> headJointNames;
  std::vector<std::string> torsoJointNames;
  std::string apriltagTopic;
  std::string baseFrame;

  void move(const tiago_iaslab_simulation::headGoalConstPtr& goal);
  void getObjects();
  control_msgs::FollowJointTrajectoryGoal generateHeadTrajectoryGoal(float pitch, float yaw);
  control_msgs::FollowJointTrajectoryGoal generateTorsoTrajectoryGoal(float v);

  void publishFeedback(const uint status);

 public:
  HeadServer(std::shared_ptr<ros::NodeHandle> nodeHandle_,
             std::string headServerTopic = "head_server",
             std::string headControllerTopic = "head_controller/follow_joint_trajectory",
             std::vector<std::string> headJointNames_ = {"head_1_joint", "head_2_joint"},
             std::string torsoControllerTopic = "torso_controller/follow_joint_trajectory",
             std::vector<std::string> torsoJointNames_ = {"torso_lift_joint"},
             std::string apriltagTopic_ = "tag_detections",
             std::string baseFrame_ = "base_footprint");
};

#endif  // TIAGO_IASLAB_SIMULATION_HEAD_SERVER_H