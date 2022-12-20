#include "tiago_iaslab_simulation/scanner_server.h"

#include <ros/topic.h>
#include <sensor_msgs/LaserScan.h>

ScannerServer::ScannerServer(std::shared_ptr<ros::NodeHandle> nodeHandle_,
                             std::string scanTopic_) : nodeHandle(nodeHandle_),
                                                       scanTopic(scanTopic_) {
  service = nodeHandle->advertiseService("scan_obstacles", &ScannerServer::sendObstacles, this);
}

bool ScannerServer::sendObstacles(tiago_iaslab_simulation::scanObstaclesRequest& request, tiago_iaslab_simulation::scanObstaclesResponse& response) {
  boost::shared_ptr<const sensor_msgs::LaserScan> laser_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(scanTopic, *nodeHandle);

  response.obstacles = getObstaclesPosition(*laser_msg);

  return true;
}

std::vector<geometry_msgs::PointStamped> ScannerServer::getObstaclesPosition(const sensor_msgs::LaserScan laserScan) {
  return std::vector<geometry_msgs::PointStamped>();
}
