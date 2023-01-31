#include "tiago_iaslab_simulation/robot.h"

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "tiago_iaslab_simulation/apriltagDetections.h"
#include "tiago_iaslab_simulation/object.h"
#include "tiago_iaslab_simulation/planning_helper.h"
#include "tiago_iaslab_simulation/utils.h"

Robot::Robot(std::shared_ptr<ros::NodeHandle> nodeHandle_) : nodeHandle(nodeHandle_),
                                                             moveScanClient(nodeHandle_, true),
                                                             headClient(nodeHandle_, true),
                                                             armClient(nodeHandle_, true),
                                                             findColorClient(nodeHandle_, &headClient) {}

void Robot::setPickWaypoint(geometry_msgs::PoseStamped pickWaypoint_) {
  pickWaypoint = pickWaypoint_;
}

void Robot::setPlaceWaypoint(geometry_msgs::PoseStamped placeWaypoint_) {
  placeWaypoint = placeWaypoint_;
}

void Robot::start(std::map<int, Object> objects, std::vector<int> sequence) {
  armClient.setDefaultArmPose(true);

  PlanningHelper planningHelper;

  moveScanClient.moveTo(pickWaypoint, false);  // Move to waypoint, stay still if no waypoint in the configuration

  for (int id : sequence) {
    planningHelper.clear();

    Object objectToPick;

    try {
      objectToPick = objects.at(id);
    } catch (const std::out_of_range& e) {
      ROS_WARN("Object with id '%i' not found. Check your configuration", id);
      break;
    }

    // Move to pick pose
    ROS_INFO_STREAM("Picking object with id=" << id);
    moveScanClient.moveTo(objectToPick.getRobotPickPose(), false);  // Move to object to pick

    ros::Duration(1).sleep();

    headClient.move(-0.45, 0);
    std::map<int, geometry_msgs::Pose> poses = headClient.getTags();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    for (const auto& pair : poses) {
      if (pair.first == id) {
        planningHelper.addObject(objectToPick, pair.second);
      } else {
        planningHelper.addObstacle(pair.second);
      }
    }

    planningHelper.addPickTable();

    armClient.pick(objectToPick, poses[id]);

    planningHelper.clear();

    moveScanClient.moveTo(placeWaypoint, true);

    geometry_msgs::PoseStamped placeObjectPose;

    if (!objectToPick.getRobotPlaceAuto()) {
      placeObjectPose = objectToPick.getPlacePosition();
    } else {
      auto tables = moveScanClient.getObstacles();
      placeObjectPose = findColorClient.getPose(objectToPick.getColor(), tables);
    }

    moveScanClient.moveTo(generatePlacePose(placeObjectPose), false);

    planningHelper.addPlaceTable(placeObjectPose);

    armClient.place(objectToPick, placeObjectPose);

    planningHelper.clear();

    moveScanClient.moveTo(placeWaypoint, false);

    spinner.stop();
  }
}

void Robot::moveTo(geometry_msgs::PoseStamped pose_, bool scan) {
  moveScanClient.moveTo(pose_, scan);
}

geometry_msgs::PoseStamped Robot::generatePlacePose(geometry_msgs::PoseStamped pose) {
  geometry_msgs::PoseStamped temp = pose;
  pose.pose.position.y -= 0.6;
  return pose;
}
