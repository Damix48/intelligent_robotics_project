#include "tiago_iaslab_simulation/corridor_server.h"

#include <geometry_msgs/Twist.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

#include "tiago_iaslab_simulation/constant.h"
#include "tiago_iaslab_simulation/point.h"

CorridorServer::CorridorServer(std::shared_ptr<ros::NodeHandle> nodeHandle_,
                               float maxCorridorWidth_,
                               std::string goalTopic_,
                               std::string robotPoseTopic_,
                               std::string scanTopic_,
                               std::string cmdVelTopic,
                               std::string pauseNavigationTopic,
                               std::string feedbackTopic) : nodeHandle(nodeHandle_),
                                                            maxCorridorWidth(maxCorridorWidth_),
                                                            scanTopic(scanTopic_),
                                                            pauseNavigation(false) {
  getGoal = nodeHandle->subscribe(goalTopic_, 1000, &CorridorServer::getGoalCallback, this);
  getRobotPose = nodeHandle->subscribe(robotPoseTopic_, 1000, &CorridorServer::getRobotPoseCallback, this);
  cmdVel = nodeHandle->advertise<geometry_msgs::Twist>(cmdVelTopic, 1);
  navigation = nodeHandle->advertise<std_msgs::Bool>(pauseNavigationTopic, 1);
  feedback = nodeHandle->advertise<std_msgs::UInt8>(feedbackTopic, 1);
}

void CorridorServer::getGoalCallback(const move_base_msgs::MoveBaseActionGoalConstPtr& goal) {
  target = goal->goal.target_pose;

  start();
}

void CorridorServer::getRobotPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {
  Point robotPosition(*pose);

  if (isNearTarget(robotPosition)) {
    stop();
    togglePauseNavigation(false);
  }
}

void CorridorServer::start() {
  scanner = nodeHandle->subscribe(scanTopic, 1000, &CorridorServer::scannerCallback, this);
}

void CorridorServer::stop() {
  scanner.shutdown();
}

void CorridorServer::scannerCallback(const sensor_msgs::LaserScanConstPtr& msg) {
  float distanceRight = msg->ranges[66];
  float distanceLeft = msg->ranges[msg->ranges.size() - 66];
  float distanceFrontalRight = msg->ranges[191];
  float distanceFrontalLeft = msg->ranges[msg->ranges.size() - 191];

  if (distanceRight + distanceLeft < maxCorridorWidth) {
    togglePauseNavigation(true);
    move(distanceRight, distanceLeft);
  } else if (distanceFrontalRight + distanceFrontalLeft < maxCorridorWidth && distanceFrontalRight > 0.2 * maxCorridorWidth && distanceFrontalLeft > 0.2 * maxCorridorWidth) {
    togglePauseNavigation(true);
    move(distanceFrontalRight, distanceFrontalLeft);
  } else {
    togglePauseNavigation(false);
  }
}

void CorridorServer::togglePauseNavigation(bool pauseNavigation_) {
  if (pauseNavigation != pauseNavigation_) {
    pauseNavigation = pauseNavigation_;

    std_msgs::Bool msg;
    msg.data = pauseNavigation;
    navigation.publish(msg);

    std_msgs::UInt8 feedback_;
    if (pauseNavigation) {
      feedback_.data = status::MOVING_CORRIDOR;
    } else {
      feedback_.data = status::MOVING;
    }
    feedback.publish(feedback_);
  }
}

void CorridorServer::move(float distanceRight_, float distanceLeft_) {
  geometry_msgs::Twist vel;
  vel.linear.x = 0.2;
  vel.angular.z = 0.7 * -(distanceRight_ - distanceLeft_);

  cmdVel.publish(vel);
}

bool CorridorServer::isNearTarget(Point point) {
  Point target_(target);

  return point.distance(target_) < maxCorridorWidth / 2;
}
