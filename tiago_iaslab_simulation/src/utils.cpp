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
