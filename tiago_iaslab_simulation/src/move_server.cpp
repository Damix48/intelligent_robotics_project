#include "tiago_iaslab_simulation/move_server.h"

#include <costmap_2d/costmap_2d_ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_ros/transform_listener.h>

#include "tiago_iaslab_simulation/scanObstacles.h"

MoveServer::MoveServer(std::string topic) : actionServer(nodeHandle, topic, boost::bind(&MoveServer::move, this, _1), false),
                                            moveActionClient("move_base") {
  scannerClient = nodeHandle.serviceClient<tiago_iaslab_simulation::scanObstacles>("scan_obstacles");

  actionServer.start();
}

void MoveServer::move(const tiago_iaslab_simulation::moveScanGoalConstPtr& goal) {
  ROS_INFO("Waiting for action server to start.");
  moveActionClient.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  move_base_msgs::MoveBaseGoal moveGoal;
  moveGoal.target_pose.header.frame_id = "map";
  moveGoal.target_pose.header.stamp = ros::Time::now();
  moveGoal.target_pose.pose = goal->pose;

  moveActionClient.sendGoal(moveGoal);

  moveActionClient.waitForResult();

  tiago_iaslab_simulation::scanObstacles service;

  if (scannerClient.call(service)) {
    ROS_INFO("numero di ostacoli: %i", service.response.obstacles.size());
  }

  actionServer.setSucceeded();
}
