#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/node_handle.h>
#include <ros/topic.h>
#include <string.h>

#include "tiago_iaslab_simulation/map2d.h"

Map2D::Map2D(std::string topic_) : topic(topic_) {
  update();
}

void Map2D::generateMap(nav_msgs::OccupancyGrid grid_) {
  map = costmap_2d::Costmap2D(grid_.info.width,
                              grid_.info.height,
                              grid_.info.resolution,
                              grid_.info.origin.position.x,
                              grid_.info.origin.position.y);

  for (size_t i = 0; i < grid_.data.size(); i++) {
    u_int mx;
    u_int my;

    map.indexToCells(i, mx, my);

    map.setCost(mx, my, grid_.data[i]);
  }
}

void Map2D::update() {
  boost::shared_ptr<const nav_msgs::OccupancyGrid> map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(topic, nodeHandle);

  generateMap(*map_msg);
}

bool Map2D::isValidPoint(float x_, float y_) const {
  u_int mapX;
  u_int mapY;

  if (map.worldToMap(x_, y_, mapX, mapY)) {
    unsigned char cost = map.getCost(mapX, mapY);

    if (cost == costmap_2d::FREE_SPACE) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool Map2D::isValidPoint(geometry_msgs::Point point_) const {
  return isValidPoint(point_.x, point_.y);
}
