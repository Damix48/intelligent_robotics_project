#ifndef TIAGO_IASLAB_SIMULATION_POINT_H
#define TIAGO_IASLAB_SIMULATION_POINT_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <cmath>

class Point {
 private:
 public:
  float x;
  float y;

  Point();
  Point(float x_, float y_);
  Point(geometry_msgs::Point point);
  Point(geometry_msgs::PointStamped point);
  Point(geometry_msgs::Pose pose);
  Point(geometry_msgs::PoseStamped pose);
  Point(geometry_msgs::PoseWithCovariance pose);
  Point(geometry_msgs::PoseWithCovarianceStamped pose);

  static Point fromPolar(float radius, float angle);

  float distance(const Point& other) const;

  Point operator+(const Point& other) const;
  Point operator-(const Point& other) const;
  Point operator*(const float& factor) const;
  Point operator/(const float& factor) const;
};

std::ostream& operator<<(std::ostream& os, const Point& point) {
  os << "(" << point.x << ", " << point.y << ")";
}

#endif  // TIAGO_IASLAB_SIMULATION_POINT_H