#include "tiago_iaslab_simulation/head_client.h"

#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

HeadClient::HeadClient(std::shared_ptr<ros::NodeHandle> nodeHandle_,
                       bool print_,
                       std::string headServerTopic,
                       std::string pointHeadTopic) : nodeHandle(nodeHandle_),
                                                     actionClient(headServerTopic),
                                                     pointHeadClient(pointHeadTopic),
                                                     print(print_) {}

bool HeadClient::move(float pitch, float yaw, bool getObject) {
  tags.clear();

  actionClient.waitForServer();

  tiago_iaslab_simulation::headGoal goal;
  goal.pitch = pitch;
  goal.yaw = yaw;
  goal.return_object = getObject;

  actionClient.sendGoal(goal);

  actionClient.waitForResult();

  for (const auto& tag : actionClient.getResult()->tags) {
    tags[tag.id[0]] = tag.pose.pose.pose;
  }
  return actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

bool HeadClient::move(geometry_msgs::PointStamped target) {
  pointHeadClient.waitForServer();
  ROS_INFO("Connection with server for head motion established");

  std::string frame = "xtion_rgb_optical_frame";
  target.point.z = 0.4;

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  geometry_msgs::PointStamped out = buffer.transform(target, frame, ros::Time(0), target.header.frame_id, ros::Duration(2));

  geometry_msgs::PointStamped target_transformed;
  target_transformed.header.frame_id = frame;
  target_transformed.point.x = out.point.x;
  target_transformed.point.y = out.point.y;
  target_transformed.point.z = out.point.z;

  control_msgs::PointHeadGoal goalHead;
  goalHead.target = target;
  goalHead.pointing_axis.x = 0;
  goalHead.pointing_axis.y = 0;
  goalHead.pointing_axis.z = 1;
  goalHead.pointing_frame = frame;
  goalHead.min_duration.sec = 1;
  goalHead.min_duration.nsec = 0;
  goalHead.max_velocity = 0.25;

  pointHeadClient.sendGoal(goalHead);
  pointHeadClient.waitForResult();

  return pointHeadClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

std::map<int, geometry_msgs::Pose> HeadClient::getTags() {
  return tags;
}
