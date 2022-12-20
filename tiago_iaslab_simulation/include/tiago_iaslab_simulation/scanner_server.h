#ifndef TIAGO_IASLAB_SIMULATION_SCANNER_SERVER_H
#define TIAGO_IASLAB_SIMULATION_SCANNER_SERVER_H

#include <geometry_msgs/PointStamped.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>
#include <sensor_msgs/LaserScan.h>

#include "tiago_iaslab_simulation/scanObstacles.h"

class ScannerServer {
 private:
  std::shared_ptr<ros::NodeHandle> nodeHandle;

  ros::ServiceServer service;
  std::string scanTopic;

  bool sendObstacles(tiago_iaslab_simulation::scanObstaclesRequest& request, tiago_iaslab_simulation::scanObstaclesResponse& response);

  std::vector<geometry_msgs::PointStamped> getObstaclesPosition(const sensor_msgs::LaserScan laserScan);

 public:
  ScannerServer(std::shared_ptr<ros::NodeHandle> nodeHandle_, std::string scanTopic_ = "scan");
};

#endif  // TIAGO_IASLAB_SIMULATION_SCANNER_SERVER_H