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

std::vector<geometry_msgs::PointStamped> ScannerServer::getObstaclesPosition(const sensor_msgs::LaserScanConstPrt laserScan) {
  std::vector<std::vector<geometry_msgs::PointStamped>> clusters;
  std::vector<float> ranges = laserScan->ranges;
  float range_min = laserScan->range_min;
  float range_max = laserScan->range_max;
  float angle_min = laserScan->angle_min;
  float angle_increment = laserScan->angle_increment;
  float current_angle = angle_min;
  
  // store first point in first cluster
  geometry_msgs::PointStamped p;
  std::vector<geometry_msgs::PointStamped> first_cluster;
  p.point.x = ranges[0] * cos(current_angle);
  p.point.y = ranges[0] * sin(current_angle);
  first_cluster.push_back(p);
  clusters.push_back(first_cluster);
  int j = 0; //index to keep track of # of cluster in which add points
  
  // threshold in radial distance for clustering 
  float t = 0.5; 

  for (int i = 1; i < ranges.size(); i++)
  {
    if (ranges[i] >= range_min && ranges[i] <= range_max)
    {
      // from polar to cartesian coordinates
      geometry_msgs::PointStamped pt;
      pt.point.x = ranges[i] * cos(current_angle);
      pt.point.y = ranges[i] * sin(current_angle);

      // check if a new cluster must be created
      if (std::abs(ranges[i] - ranges[i - 1]) >= t)
      {
        // new cluster
        std::vector<geometry_msgs::PointStamped> new_cluster;
        new_cluster.push_back(pt);
        clusters.push_back(new_cluster);
        j++;
      }
      else
      {
        clusters[j].push_back(pt);
      }
    }
    current_angle += angle_increment;
  }

  return std::vector<geometry_msgs::PointStamped>();
}

 std::vector<std::vector<geometry_msgs::PointStamped>> ScannerServer::removeSmallClusters(std::vector<std::vector<geometry_msgs::PointStamped>> clusters, int thresh){
  std::vector<std::vector<geometry_msgs::PointStamped>> out;
  for (int i = 0; i < clusters.size(); i++)
  {
    if (clusters[i].size() > thresh)
    {
      out.push_back(clusters[i]);
    }
  }
  return out;
}