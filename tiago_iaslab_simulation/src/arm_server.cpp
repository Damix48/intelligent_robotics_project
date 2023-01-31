#include "tiago_iaslab_simulation/arm_server.h"

#include <gazebo_ros_link_attacher/Attach.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "tiago_iaslab_simulation/object.h"
#include "tiago_iaslab_simulation/utils.h"

ArmServer::ArmServer(std::shared_ptr<ros::NodeHandle> nodeHandle_,
                     std::string armServerTopic,
                     std::string moveGroupName,
                     std::string gripperControllerTopic,
                     std::vector<std::string> gripperJointNames_,
                     std::string linkAttacherTopic_,
                     std::string robotModelName_,
                     std::string robotLinkName_) : nodeHandle(nodeHandle_),
                                                   pickServer(*nodeHandle, armServerTopic + "/pick", boost::bind(&ArmServer::pick, this, _1), false),
                                                   placeServer(*nodeHandle, armServerTopic + "/place", boost::bind(&ArmServer::place, this, _1), false),
                                                   moveGroup(moveGroupName),
                                                   gripperController(gripperControllerTopic),
                                                   gripperJointNames(gripperJointNames_),
                                                   linkAttacherTopic(linkAttacherTopic_),
                                                   robotModelName(robotModelName_),
                                                   robotLinkName(robotLinkName_) {}

void ArmServer::setArmDefaultPose(geometry_msgs::Pose pose) {
  armDefaultPose = pose;
}

void ArmServer::pick(const tiago_iaslab_simulation::pickGoalConstPtr& goal) {
  geometry_msgs::Pose safePickPose = generateSafePose(goal->pose, static_cast<ObjectConstant::PICK_MODE>(goal->pick_mode));

  move(safePickPose);
  ros::Duration(1).sleep();

  move(goal->pose);
  ros::Duration(1).sleep();

  toggleGripper(true, goal->name);
  ros::Duration(1).sleep();

  move(safePickPose);
  ros::Duration(1).sleep();

  move(armDefaultPose);
  ros::Duration(1).sleep();

  pickServer.setSucceeded();
}

void ArmServer::place(const tiago_iaslab_simulation::placeGoalConstPtr& goal) {}

void ArmServer::move(geometry_msgs::Pose pose) {
  moveGroup.setPoseTarget(pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success = (moveGroup.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  moveGroup.move();
}

void ArmServer::toggleGripper(bool close, std::string name) {
  control_msgs::FollowJointTrajectoryGoal trajectoryGoal;

  if (close) {
    trajectoryGoal = generateGripperTrajectoryGoal(0.35);
  } else {
    trajectoryGoal = generateGripperTrajectoryGoal(0.8);
  }

  gripperController.sendGoal(trajectoryGoal);
  bool timeout = gripperController.waitForResult(ros::Duration(10));

  ros::ServiceClient serviceClient = nodeHandle->serviceClient<gazebo_ros_link_attacher::Attach>(linkAttacherTopic + (close ? "/attach" : "/detach"));
  serviceClient.waitForExistence(ros::Duration(2));

  gazebo_ros_link_attacher::Attach service;
  service.request.model_name_1 = robotModelName;
  service.request.link_name_1 = robotLinkName;
  service.request.model_name_2 = name;
  service.request.link_name_2 = name + "_link";

  serviceClient.call(service);
}

geometry_msgs::Pose ArmServer::generateSafePose(const geometry_msgs::Pose& pose_, ObjectConstant::PICK_MODE pickMode_) const {
  geometry_msgs::Pose temp = pose_;

  tf2::Quaternion orientation;

  switch (pickMode_) {
    case ObjectConstant::PICK_MODE::SIDE:
      /* code */
      break;

    default:
      temp.position.z += 0.4;
      iaslab::EulerAngles angles = iaslab::convertToEulerAngles(temp.orientation);
      // angles.yaw -= M_PI / 4;
      orientation.setRPY(-M_PI / 2, M_PI / 2, angles.yaw);
      break;
  }

  temp.orientation = tf2::toMsg(orientation);

  return temp;
}

control_msgs::FollowJointTrajectoryGoal ArmServer::generateGripperTrajectoryGoal(float distance) const {
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = gripperJointNames;

  goal.trajectory.points.resize(1);

  goal.trajectory.points[0].positions.resize(2);

  goal.trajectory.points[0].positions[0] = distance / 2;
  goal.trajectory.points[0].positions[1] = distance / 2;

  goal.trajectory.points[0].velocities.resize(2);

  goal.trajectory.points[0].velocities[0] = 0.3;
  goal.trajectory.points[0].velocities[1] = 0.3;

  goal.trajectory.points[0].time_from_start = ros::Duration(2.0);
}
