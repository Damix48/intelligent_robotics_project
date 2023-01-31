#ifndef TIAGO_IASLAB_SIMULATION_ROBOT_H
#define TIAGO_IASLAB_SIMULATION_ROBOT_H

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>

#include "tiago_iaslab_simulation/arm_client.h"
#include "tiago_iaslab_simulation/find_color_client.h"
#include "tiago_iaslab_simulation/head_client.h"
#include "tiago_iaslab_simulation/move_client.h"
#include "tiago_iaslab_simulation/object.h"

class Robot {
 private:
  std::shared_ptr<ros::NodeHandle> nodeHandle;

  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

  MoveClient moveScanClient;
  HeadClient headClient;
  ArmClient armClient;
  FindColorClient findColorClient;

  std::vector<moveit_msgs::CollisionObject> collisionObjects;

  geometry_msgs::PoseStamped pickWaypoint;
  geometry_msgs::PoseStamped placeWaypoint;

  geometry_msgs::PoseStamped generatePlacePose(geometry_msgs::PoseStamped pose);

  void addPlaceTable(geometry_msgs::PoseStamped pose_);

 public:
  Robot(std::shared_ptr<ros::NodeHandle> nodeHandle_);

  void setPickWaypoint(geometry_msgs::PoseStamped pickWaypoint_);
  void setPlaceWaypoint(geometry_msgs::PoseStamped placeWaypoint_);

  void start(std::map<int, Object> objects, std::vector<int> sequence);

  void moveTo(geometry_msgs::PoseStamped pose, bool scan = false);
};

#endif  // TIAGO_IASLAB_SIMULATION_ROBOT_H