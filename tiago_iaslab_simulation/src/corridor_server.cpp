#include "tiago_iaslab_simulation/corridor_server.h"

#include <geometry_msgs/Twist.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

#include "tiago_iaslab_simulation/point.h"

CorridorServer::CorridorServer(float maxCorridorWidth_,
                               std::string goalTopic_,
                               std::string robotPoseTopic_,
                               std::string scanTopic_,
                               std::string cmdVelTopic_,
                               std::string pauseNavigationTopic_) : maxCorridorWidth(maxCorridorWidth_),
                                                                    scanTopic(scanTopic_) {
  getGoal = nodeHandle.subscribe(goalTopic_, 1000, &CorridorServer::getGoalCallback, this);
  getRobotPose = nodeHandle.subscribe(robotPoseTopic_, 1000, &CorridorServer::getRobotPoseCallback, this);
  cmdVel = nodeHandle.advertise<geometry_msgs::Twist>(cmdVelTopic_, 1);
  navigation = nodeHandle.advertise<std_msgs::Bool>(pauseNavigationTopic_, 1);
}

void CorridorServer::getGoalCallback(const move_base_msgs::MoveBaseActionGoalConstPtr& goal) {
  target = goal->goal.target_pose;

  start();
}

void CorridorServer::getRobotPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {
  Point robotPosition(*pose);

  if (isNearTarget(robotPosition)) {
    stop();
    resumeNavigation();
  }
}

void CorridorServer::start() {
  scanner = nodeHandle.subscribe(scanTopic, 1000, &CorridorServer::scannerCallback, this);
}

void CorridorServer::stop() {
  scanner.shutdown();
}

void CorridorServer::scannerCallback(const sensor_msgs::LaserScanConstPtr& msg) {
  float distanceRight = msg->ranges[66];
  float distanceLeft = msg->ranges[msg->ranges.size() - 66];

  if (distanceRight + distanceLeft < maxCorridorWidth) {
    stopNavigation();
    move(distanceRight, distanceLeft);
  } else {
    resumeNavigation();
  }
}

void CorridorServer::stopNavigation() {
  std_msgs::Bool msg;
  msg.data = true;
  navigation.publish(msg);
  ROS_INFO("Navigation stopped.");
}

void CorridorServer::resumeNavigation() {
  std_msgs::Bool msg;
  msg.data = false;
  navigation.publish(msg);
  ROS_INFO("Navigation resumed.");
}

void CorridorServer::move(float distanceRight_, float distanceLeft_) {
  geometry_msgs::Twist vel;
  vel.linear.x = 0.5;
  vel.angular.z = 0.5 * -(distanceRight_ - distanceLeft_);

  cmdVel.publish(vel);
}

bool CorridorServer::isNearTarget(Point point) {
  Point target_(target);

  return point.distance(target_) < 0.5;
}
