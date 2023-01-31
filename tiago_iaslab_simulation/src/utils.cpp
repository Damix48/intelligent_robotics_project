#include "tiago_iaslab_simulation/utils.h"

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

geometry_msgs::Pose iaslab::createPose(geometry_msgs::Point point_, float yaw_) {
  tf2::Quaternion orientation_;
  orientation_.setRPY(0, 0, yaw_);
  orientation_.normalize();

  geometry_msgs::Quaternion orientation;
  orientation.x = orientation_.getX();
  orientation.y = orientation_.getY();
  orientation.z = orientation_.getZ();
  orientation.w = orientation_.getW();

  geometry_msgs::Pose temp;
  temp.position = point_;
  temp.orientation = orientation;

  return temp;
}

geometry_msgs::Pose iaslab::createPose(float x_, float y_, float yaw_) {
  geometry_msgs::Point point;

  point.x = x_;
  point.y = y_;
  point.z = 0;

  return iaslab::createPose(point, yaw_);
}

iaslab::EulerAngles iaslab::convertToEulerAngles(geometry_msgs::Quaternion quaternion_) {
  iaslab::EulerAngles orientation;

  // roll (x-axis rotation)
  float sinr_cosp = 2 * (quaternion_.w * quaternion_.x + quaternion_.y * quaternion_.z);
  float cosr_cosp = 1 - 2 * (quaternion_.x * quaternion_.x + quaternion_.y * quaternion_.y);
  orientation.roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  float sinp = std::sqrt(1 + 2 * (quaternion_.w * quaternion_.y - quaternion_.x * quaternion_.z));
  float cosp = std::sqrt(1 - 2 * (quaternion_.w * quaternion_.y - quaternion_.x * quaternion_.z));
  orientation.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

  // yaw (z-axis rotation)
  float siny_cosp = 2 * (quaternion_.w * quaternion_.z + quaternion_.x * quaternion_.y);
  float cosy_cosp = 1 - 2 * (quaternion_.y * quaternion_.y + quaternion_.z * quaternion_.z);
  orientation.yaw = std::atan2(siny_cosp, cosy_cosp);

  return orientation;
}
