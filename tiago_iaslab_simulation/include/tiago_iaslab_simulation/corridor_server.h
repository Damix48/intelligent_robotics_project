#ifndef TIAGO_IASLAB_SIMULATION_CORRIDOR_SERVER_H
#define TIAGO_IASLAB_SIMULATION_CORRIDOR_SERVER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <string.h>

#include "tiago_iaslab_simulation/point.h"

class CorridorServer {
 private:
  ros::NodeHandle nodeHandle;

  std::string scanTopic;

  ros::Subscriber getGoal;
  ros::Subscriber getRobotPose;
  ros::Subscriber scanner;
  ros::Publisher cmdVel;
  ros::Publisher navigation;

  float maxCorridorWidth;
  geometry_msgs::PoseStamped target;

  void start();
  void stop();

  void getGoalCallback(const move_base_msgs::MoveBaseActionGoalConstPtr& goal);
  void getRobotPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);
  void scannerCallback(const sensor_msgs::LaserScanConstPtr& msg);

  void stopNavigation();
  void resumeNavigation();

  void move(float distanceRight_, float distanceLeft_);

  bool isNearTarget(Point point);

 public:
  CorridorServer(float maxCorridorWidth_,
                 std::string goalTopic_ = "move_base/goal",
                 std::string robotPoseTopic_ = "robot_pose",
                 std::string scanTopic_ = "scan",
                 std::string cmdVelTopic_ = "mobile_base_controller/cmd_vel",
                 std::string pauseNavigationTopic_ = "pause_navigation");
};

#endif  // TIAGO_IASLAB_SIMULATION_CORRIDOR_SERVER_H