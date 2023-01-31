#include "tiago_iaslab_simulation/arm_server.h"

#include <gazebo_ros_link_attacher/Attach.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "tiago_iaslab_simulation/object.h"
#include "tiago_iaslab_simulation/planning_helper.h"
#include "tiago_iaslab_simulation/utils.h"

ArmServer::ArmServer(std::shared_ptr<ros::NodeHandle> nodeHandle_,
                     std::string armServerTopic,
                     std::string moveGroupName,
                     std::string moveArmOnlyGroupName,
                     std::string gripperControllerTopic,
                     std::vector<std::string> gripperJointNames_,
                     std::string linkAttacherTopic_,
                     std::string robotModelName_,
                     std::string robotLinkName_) : nodeHandle(nodeHandle_),
                                                   pickServer(*nodeHandle, armServerTopic + "/pick", boost::bind(&ArmServer::pick, this, _1), false),
                                                   placeServer(*nodeHandle, armServerTopic + "/place", boost::bind(&ArmServer::place, this, _1), false),
                                                   moveGroup(moveGroupName),
                                                   moveArmOnlyGroup(moveArmOnlyGroupName),
                                                   gripperController(gripperControllerTopic),
                                                   gripperJointNames(gripperJointNames_),
                                                   linkAttacherTopic(linkAttacherTopic_),
                                                   robotModelName(robotModelName_),
                                                   robotLinkName(robotLinkName_) {
  gripperController.waitForServer();

  pickServer.start();
  placeServer.start();
}

void ArmServer::pick(const tiago_iaslab_simulation::pickGoalConstPtr& goal) {
  armDefaultPose = moveGroup.getCurrentPose().pose;

  moveSafestPose();

  geometry_msgs::Pose safePickPose = generateSafePose(goal->pose, goal->height, static_cast<ObjectConstant::PICK_MODE>(goal->pick_mode));
  move(safePickPose);
  ros::Duration(1).sleep();

  PlanningHelper planningHelper;
  planningHelper.removeObject(goal->id);

  geometry_msgs::Pose pickPose = generatePickPose(safePickPose, goal->height, static_cast<ObjectConstant::PICK_MODE>(goal->pick_mode));
  move(pickPose);
  ros::Duration(1).sleep();

  toggleGripper(true, goal->name);
  ros::Duration(1).sleep();

  move(safePickPose);
  ros::Duration(1).sleep();

  moveSafestPose();

  move(armDefaultPose);
  ros::Duration(1).sleep();

  pickServer.setSucceeded();
}

void ArmServer::place(const tiago_iaslab_simulation::placeGoalConstPtr& goal) {
  armDefaultPose = moveGroup.getCurrentPose().pose;

  moveSafestPose();

  geometry_msgs::Pose safePlacePose = generateSafePose(goal->pose, goal->height, static_cast<ObjectConstant::PICK_MODE>(goal->pick_mode));
  move(safePlacePose);
  ros::Duration(1).sleep();

  geometry_msgs::Pose placePose = generatePickPose(safePlacePose, goal->height, static_cast<ObjectConstant::PICK_MODE>(goal->pick_mode));
  move(placePose);
  ros::Duration(1).sleep();

  toggleGripper(false, goal->name);
  ros::Duration(1).sleep();

  move(safePlacePose);
  ros::Duration(1).sleep();

  moveSafestPose();

  move(armDefaultPose);
  ros::Duration(1).sleep();

  placeServer.setSucceeded();
}

void ArmServer::move(geometry_msgs::Pose pose) {
  moveGroup.setPoseTarget(pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success = (moveGroup.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  moveGroup.move();
}

void ArmServer::moveSafestPose() {
  // moveit::planning_interface::MoveGroupInterface move_group("arm");

  // // choose your preferred planner
  // move_group.setPlannerId("SBLkConfigDefault");

  moveit::core::RobotStatePtr current_state = moveArmOnlyGroup.getCurrentState();

  std::vector<double> jointGroupPositions;
  current_state->copyJointGroupPositions(moveArmOnlyGroup.getName(), jointGroupPositions);

  jointGroupPositions[0] = 10 * (M_PI / 180);  // in radians
  jointGroupPositions[1] = -1 * (M_PI / 180);
  jointGroupPositions[2] = -190 * (M_PI / 180);
  jointGroupPositions[3] = 90 * (M_PI / 180);
  jointGroupPositions[4] = 0 * (M_PI / 180);
  jointGroupPositions[5] = 0 * (M_PI / 180);
  jointGroupPositions[6] = -0 * (M_PI / 180);

  moveArmOnlyGroup.setJointValueTarget(jointGroupPositions);

  moveArmOnlyGroup.setMaxVelocityScalingFactor(0.5);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success = (moveArmOnlyGroup.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  moveArmOnlyGroup.move();
}

void ArmServer::toggleGripper(bool close, std::string name) {
  control_msgs::FollowJointTrajectoryGoal trajectoryGoal;

  if (close) {
    trajectoryGoal = generateGripperTrajectoryGoal(0.035);
  } else {
    trajectoryGoal = generateGripperTrajectoryGoal(0.08);
  }

  gripperController.sendGoal(trajectoryGoal);
  bool timeout = gripperController.waitForResult(ros::Duration(10));

  ros::ServiceClient serviceClient;
  if (close) {
    serviceClient = nodeHandle->serviceClient<gazebo_ros_link_attacher::Attach>(linkAttacherTopic + "/attach");
  } else {
    serviceClient = nodeHandle->serviceClient<gazebo_ros_link_attacher::Attach>(linkAttacherTopic + "/detach");
  }

  serviceClient.waitForExistence(ros::Duration(2));

  gazebo_ros_link_attacher::Attach service;
  service.request.model_name_1 = robotModelName;
  service.request.link_name_1 = robotLinkName;
  service.request.model_name_2 = name;
  service.request.link_name_2 = name + "_link";

  serviceClient.call(service);
}

geometry_msgs::Pose ArmServer::generateSafePose(const geometry_msgs::Pose& pose_, float height, ObjectConstant::PICK_MODE pickMode_) const {
  geometry_msgs::Pose temp = pose_;

  tf2::Quaternion orientation;

  switch (pickMode_) {
    case ObjectConstant::PICK_MODE::SIDE:
      // TODO
      break;

    default:
      temp.position.z += height / 2 + 0.25;
      iaslab::EulerAngles angles = iaslab::convertToEulerAngles(temp.orientation);
      angles.yaw -= M_PI / 2;
      orientation.setRPY(-M_PI / 2, M_PI / 2, angles.yaw);
      break;
  }

  temp.orientation = tf2::toMsg(orientation);

  return temp;
}

geometry_msgs::Pose ArmServer::generatePickPose(const geometry_msgs::Pose& pose_, float height, ObjectConstant::PICK_MODE pickMode_) const {
  geometry_msgs::Pose temp = pose_;

  switch (pickMode_) {
    case ObjectConstant::PICK_MODE::SIDE:
      // TODO
      break;

    default:
      temp.position.z -= height / 2 + 0.065;
      break;
  }

  return temp;
}

control_msgs::FollowJointTrajectoryGoal ArmServer::generateGripperTrajectoryGoal(float distance) const {
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = gripperJointNames;

  goal.trajectory.points.resize(1);

  goal.trajectory.points[0].positions.resize(2);

  goal.trajectory.points[0].positions[0] = distance / 2;
  goal.trajectory.points[0].positions[1] = distance / 2;

  // goal.trajectory.points[0].velocities.resize(2);

  // goal.trajectory.points[0].velocities[0] = 0;
  // goal.trajectory.points[0].velocities[1] = 0.3;

  goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

  return goal;
}
