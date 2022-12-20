#ifndef TIAGO_IASLAB_SIMULATION_MAP2D_H
#define TIAGO_IASLAB_SIMULATION_MAP2D_H

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/node_handle.h>
#include <string.h>

class Map2D {
 private:
  ros::NodeHandle nodeHandle;
  std::string topic;
  costmap_2d::Costmap2D map;

  void generateMap(nav_msgs::OccupancyGrid grid_);

 public:
  Map2D(std::string topic_);
  void update();

  bool isValidPoint(float x_, float y_) const;
  bool isValidPoint(geometry_msgs::Point point_) const;
};

#endif  // TIAGO_IASLAB_SIMULATION_MAP2D_H