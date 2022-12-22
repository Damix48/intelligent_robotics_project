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
  std::shared_ptr<ros::NodeHandle> nodeHandle;

  std::string scanTopic;

  ros::Subscriber getGoal;
  ros::Subscriber getRobotPose;
  ros::Subscriber scanner;
  ros::Publisher cmdVel;
  ros::Publisher navigation;
  ros::Publisher feedback;

  float maxCorridorWidth;
  geometry_msgs::PoseStamped target;

  bool pauseNavigation;

  /// @brief Function to start the detection of a corridor.
  void start();
  /// @brief Function to stop the detection of a corridor.
  void stop();

  /// @brief Callback to get the target goal.
  /// @param goal
  void getGoalCallback(const move_base_msgs::MoveBaseActionGoalConstPtr& goal);
  /// @brief Callback to process the robot pose.
  /// @param pose
  void getRobotPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);
  /// @brief Callback to process the laser scan data.
  /// @param msg
  void scannerCallback(const sensor_msgs::LaserScanConstPtr& msg);

  /// @brief Publish \c pauseNavigation_ to pause or resume the \c move_base navigation.
  /// @param pauseNavigation_
  void togglePauseNavigation(bool pauseNavigation_);

  /// @brief Publish the velocities to keep the robot in the middle.
  /// @param distanceRight_
  /// @param distanceLeft_
  void move(float distanceRight_, float distanceLeft_);

  /// @brief Calculate if the robot is near the target.
  /// @param point
  /// @return if \c point is near the target
  bool isNearTarget(Point point);

 public:
  /// @brief It's a service that detect circular obstacles usign the laser range sensor.
  /// @param nodeHandle_
  /// @param maxCorridorWidth_ maximum width to consider the robot in a corridor
  /// @param goalTopic_ topic to get the goal from \c move_base
  /// @param robotPoseTopic_ topic to get the robot pose
  /// @param scanTopic_ topic to get the laser scan data
  /// @param cmdVelTopic topic to control the robot velocities
  /// @param pauseNavigationTopic topic to pause the \c move_base navigation
  /// @param feedbackTopic topic to publish the feedback
  CorridorServer(std::shared_ptr<ros::NodeHandle> nodeHandle_,
                 float maxCorridorWidth_,
                 std::string goalTopic_ = "move_base/goal",
                 std::string robotPoseTopic_ = "robot_pose",
                 std::string scanTopic_ = "scan",
                 std::string cmdVelTopic = "mobile_base_controller/cmd_vel",
                 std::string pauseNavigationTopic = "pause_navigation",
                 std::string feedbackTopic = "move_server_feedback");
};

#endif  // TIAGO_IASLAB_SIMULATION_CORRIDOR_SERVER_H