#include "tiago_iaslab_simulation/arm_client.h"

#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "tiago_iaslab_simulation/pickAction.h"
#include "tiago_iaslab_simulation/placeAction.h"
#include "tiago_iaslab_simulation/planning_helper.h"

ArmClient::ArmClient(std::shared_ptr<ros::NodeHandle> nodeHandle_,
                     bool print_,
                     std::string armServerTopic) : nodeHandle(nodeHandle_),
                                                   pickClient(armServerTopic + "/pick"),
                                                   placeClient(armServerTopic + "/place"),
                                                   print(print_) {
  defaultArmPosePubblisher = nodeHandle->advertise<std_msgs::Bool>(armServerTopic + "/set_default_pose", 1);
}

void ArmClient::setDefaultArmPose(bool shutdown) {
  std_msgs::Bool temp;
  temp.data = shutdown;

  defaultArmPosePubblisher.publish(temp);
}

bool ArmClient::pick(Object obj, geometry_msgs::Pose pose) {
  tiago_iaslab_simulation::pickGoal goal;

  goal.id = obj.getId();
  goal.name = obj.getName();
  goal.pick_mode = obj.getPickMode();
  goal.height = obj.getDimensions()[0];
  goal.pose = pose;
  goal.pose.position.z += obj.getOffset();

  pickClient.sendGoal(goal);

  pickClient.waitForResult();

  return pickClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

bool ArmClient::place(Object obj, geometry_msgs::PoseStamped pose_) {
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  geometry_msgs::Pose pose = buffer.transform(pose_, "base_footprint", ros::Time(0), pose_.header.frame_id, ros::Duration(5)).pose;
  pose.position.z = 0.75 + obj.getDimensions()[0];

  tiago_iaslab_simulation::placeGoal goal;

  goal.id = obj.getId();
  goal.name = obj.getName();
  goal.pick_mode = obj.getPickMode();
  goal.height = obj.getDimensions()[0];
  goal.pose = pose;

  // PlanningHelper planningHelper;
  // planningHelper.addObject(obj, pose);

  placeClient.sendGoal(goal);

  placeClient.waitForResult();

  return placeClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}
