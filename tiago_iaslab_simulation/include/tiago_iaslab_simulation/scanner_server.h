#ifndef TIAGO_IASLAB_SIMULATION_SCANNER_SERVER_H
#define TIAGO_IASLAB_SIMULATION_SCANNER_SERVER_H

#include <geometry_msgs/PointStamped.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>
#include <sensor_msgs/LaserScan.h>

#include "tiago_iaslab_simulation/circle.h"
#include "tiago_iaslab_simulation/scanObstacles.h"

class ScannerServer {
 private:
  std::shared_ptr<ros::NodeHandle> nodeHandle;

  ros::ServiceServer service;
  std::string scanTopic;

  float radialDistanceThreshold;
  int clusterMinSize;
  float minRadius;
  float maxRadius;

  /// @brief Send the position of circular obstacle detected by the laser.
  /// @param request
  /// @param response
  /// @return always true
  bool sendObstacles(tiago_iaslab_simulation::scanObstaclesRequest& request, tiago_iaslab_simulation::scanObstaclesResponse& response);

  /// @brief Returns a vector with the estimated coordinates of the center of the cylindrical obstacles starting from laser scans.
  /// @param laserScan
  /// @return \c std::vector of \c tiago_iaslab_simulation::circle with the position and radius of the obstacles
  std::vector<tiago_iaslab_simulation::circle> getObstaclesPosition(const sensor_msgs::LaserScan laserScan);

  /// @brief Check if a set of points can represent a circle basing on a threshold on the value of radius. It compute radius and center of the circle with the equation of a circle through 3 points.
  /// @param points
  /// @param radius
  /// @param center
  /// @return true if the estimated radius of the circle is between two thresholds, false otherwise
  bool isCircle(std::vector<geometry_msgs::PointStamped> points, float& radius, geometry_msgs::PointStamped& center);

  /// @brief Return a vector which contains only the cluster of the input vector which have size greater or equal than a threshold.
  /// @param clusters
  /// @return vector of cluster of points with size greater than threshold
  std::vector<std::vector<geometry_msgs::PointStamped>> removeSmallClusters(std::vector<std::vector<geometry_msgs::PointStamped>> clusters);

 public:
  /// @brief It's a service that detect circular obstacles usign the laser range sensor.
  /// @param nodeHandle_
  /// @param scanTopic_ topic for subscribing to the robot laser range sensor
  /// @param radialDistanceThreshold_ threshold in meters radial distance for clustering
  /// @param clusterMinSize_ minumum number of points to create a cluster
  /// @param minRadius_ minimum radius to consider a circle an obstacle
  /// @param maxRadius_ maximum radius to consider a circle an obstacle
  ScannerServer(std::shared_ptr<ros::NodeHandle> nodeHandle_,
                std::string scannerServerTopic = "scan_obstacles",
                std::string scanTopic_ = "scan",
                float radialDistanceThreshold_ = 0.5,
                int clusterMinSize_ = 10,
                float minRadius_ = 0.15,
                float maxRadius_ = 0.23);
};

#endif  // TIAGO_IASLAB_SIMULATION_SCANNER_SERVER_H