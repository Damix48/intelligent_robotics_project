#include "tiago_iaslab_simulation/point.h"

Point::Point() : x(0), y(0) {}

Point::Point(float x_, float y_) : x(x_), y(y_) {}

Point::Point(geometry_msgs::Point point) : x(point.x), y(point.y) {}

Point::Point(geometry_msgs::PointStamped point) : Point(point.point) {}

Point::Point(geometry_msgs::Pose pose) : Point(pose.position) {}

Point::Point(geometry_msgs::PoseStamped pose) : Point(pose.pose) {}

Point::Point(geometry_msgs::PoseWithCovariance pose) : Point(pose.pose) {}

Point::Point(geometry_msgs::PoseWithCovarianceStamped pose) : Point(pose.pose.pose) {}

Point Point::fromPolar(float radius, float angle) {
  float x_ = radius * std::cos(angle);
  float y_ = radius * std::sin(angle);

  return Point(x_, y_);
}

float Point::distance(const Point& other) const {
  float a = other.x - x;
  float b = other.y - y;

  return std::sqrt(a * a + b * b);
}

Point Point::operator+(const Point& other) const {
  float x_ = x + other.x;
  float y_ = y + other.y;

  return Point(x_, y_);
}

Point Point::operator-(const Point& other) const {
  float x_ = x - other.x;
  float y_ = y - other.y;

  return Point(x_, y_);
}

Point Point::operator*(const float& factor) const {
  float x_ = x * factor;
  float y_ = y * factor;

  return Point(x_, y_);
}

Point Point::operator/(const float& factor) const {
  float x_ = x / factor;
  float y_ = y / factor;

  return Point(x_, y_);
}
