#ifndef TIAGO_IASLAB_SIMULATION_UTILS_H
#define TIAGO_IASLAB_SIMULATION_UTILS_H

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>

namespace iaslab {

struct EulerAngles {
  float roll, pitch, yaw;
};

geometry_msgs::Pose createPose(geometry_msgs::Point point_, float yaw_);
geometry_msgs::Pose createPose(float x_, float y_, float yaw_);

EulerAngles convertToEulerAngles(geometry_msgs::Quaternion quaternion_);

}  // namespace iaslab

#endif  // TIAGO_IASLAB_SIMULATION_UTILS_H