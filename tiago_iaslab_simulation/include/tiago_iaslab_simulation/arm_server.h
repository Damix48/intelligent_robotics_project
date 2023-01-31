#ifndef TIAGO_IASLAB_SIMULATION_ARM_SERVER_H
#define TIAGO_IASLAB_SIMULATION_ARM_SERVER_H

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/Bool.h>

#include "tiago_iaslab_simulation/object.h"
#include "tiago_iaslab_simulation/pickAction.h"
#include "tiago_iaslab_simulation/placeAction.h"

class ArmServer {
 private:
  std::shared_ptr<ros::NodeHandle> nodeHandle;

  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

  actionlib::SimpleActionServer<tiago_iaslab_simulation::pickAction> pickServer;
  actionlib::SimpleActionServer<tiago_iaslab_simulation::placeAction> placeServer;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripperController;

  moveit::planning_interface::MoveGroupInterface moveGroup;
  moveit::planning_interface::MoveGroupInterface moveArmOnlyGroup;

  geometry_msgs::Pose armDefaultPose;

  std::vector<std::string> gripperJointNames;
  std::string apriltagTopic;
  std::string baseFrame;
  std::string linkAttacherTopic;
  std::string robotModelName;
  std::string robotLinkName;

  void pick(const tiago_iaslab_simulation::pickGoalConstPtr& goal);
  void place(const tiago_iaslab_simulation::placeGoalConstPtr& goal);

  void move(geometry_msgs::Pose pose);
  void toggleGripper(bool close, std::string name = "");

  void moveSafestPose();

  geometry_msgs::Pose generateSafePose(const geometry_msgs::Pose& pose_, float height, ObjectConstant::PICK_MODE pickMode_) const;
  geometry_msgs::Pose generatePickPose(const geometry_msgs::Pose& pose_, float height, ObjectConstant::PICK_MODE pickMode_) const;
  control_msgs::FollowJointTrajectoryGoal generateGripperTrajectoryGoal(float distance) const;

 public:
  ArmServer(std::shared_ptr<ros::NodeHandle> nodeHandle_,
            std::string armServerTopic = "arm_server",
            std::string moveGroupName = "arm_torso",
            std::string moveArmOnlyGroupName = "arm",
            std::string gripperControllerTopic = "gripper_controller/follow_joint_trajectory",
            std::vector<std::string> gripperJointNames_ = {"gripper_left_finger_joint", "gripper_right_finger_joint"},
            std::string linkAttacherTopic_ = "link_attacher_node",
            std::string robotModelName_ = "tiago",
            std::string robotLinkName_ = "arm_7_link");
};

#endif  // TIAGO_IASLAB_SIMULATION_ARM_SERVER_H