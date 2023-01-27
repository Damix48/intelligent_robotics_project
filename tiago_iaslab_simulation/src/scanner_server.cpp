#include "tiago_iaslab_simulation/scanner_server.h"

#include <ros/topic.h>
#include <sensor_msgs/LaserScan.h>

#include "tiago_iaslab_simulation/circle.h"
#include "tiago_iaslab_simulation/point.h"

ScannerServer::ScannerServer(std::shared_ptr<ros::NodeHandle> nodeHandle_,
                             std::string scannerServerTopic,
                             std::string scanTopic_,
                             float radialDistanceThreshold_,
                             int clusterMinSize_,
                             float minRadius_,
                             float maxRadius_) : nodeHandle(nodeHandle_),
                                                 scanTopic(scanTopic_),
                                                 radialDistanceThreshold(radialDistanceThreshold_),
                                                 clusterMinSize(clusterMinSize_),
                                                 minRadius(minRadius_),
                                                 maxRadius(maxRadius_) {
  service = nodeHandle->advertiseService(scannerServerTopic, &ScannerServer::sendObstacles, this);
}

bool ScannerServer::sendObstacles(tiago_iaslab_simulation::scanObstaclesRequest &request, tiago_iaslab_simulation::scanObstaclesResponse &response) {
  boost::shared_ptr<const sensor_msgs::LaserScan> laser_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(scanTopic, *nodeHandle);

  response.obstacles = getObstaclesPosition(*laser_msg);

  return true;
}

std::vector<tiago_iaslab_simulation::circle> ScannerServer::getObstaclesPosition(const sensor_msgs::LaserScan laserScan) {
  std::vector<std::vector<geometry_msgs::PointStamped>> clusters;
  std::vector<float> ranges = laserScan.ranges;
  float range_min = laserScan.range_min;
  float range_max = laserScan.range_max;
  float angle_min = laserScan.angle_min;
  float angle_increment = laserScan.angle_increment;
  float current_angle = angle_min;

  // store first point in first cluster
  geometry_msgs::PointStamped p;
  std::vector<geometry_msgs::PointStamped> first_cluster;
  p.point.x = ranges[0] * cos(current_angle);
  p.point.y = ranges[0] * sin(current_angle);
  first_cluster.push_back(p);
  clusters.push_back(first_cluster);
  int j = 0;  // index to keep track of # of cluster in which add points

  for (int i = 1; i < ranges.size(); i++) {
    if (ranges[i] >= range_min && ranges[i] <= range_max) {
      // from polar to cartesian coordinates
      geometry_msgs::PointStamped pt;
      pt.point.x = ranges[i] * cos(current_angle);
      pt.point.y = ranges[i] * sin(current_angle);

      // check if a new cluster must be created
      if (std::abs(ranges[i] - ranges[i - 1]) >= radialDistanceThreshold) {
        // new cluster
        std::vector<geometry_msgs::PointStamped> new_cluster;
        new_cluster.push_back(pt);
        clusters.push_back(new_cluster);
        j++;
      } else {
        clusters[j].push_back(pt);
      }
    }
    current_angle += angle_increment;
  }

  // remove small clusters
  std::vector<std::vector<geometry_msgs::PointStamped>> clusters2 = removeSmallClusters(clusters);

  // ROS_INFO_STREAM(clusters2.size())

  // for each cluster find if a circle fits
  std::vector<tiago_iaslab_simulation::circle> obstacles;
  for (int i = 0; i < clusters2.size(); i++) {
    float r;
    geometry_msgs::PointStamped c;
    tiago_iaslab_simulation::circle circle;

    if (isCircle(clusters2[i], r, c)) {
      c.header.frame_id = laserScan.header.frame_id;
      c.header.stamp = laserScan.header.stamp;
      c.header.seq = i;

      circle.center = c;
      circle.radius = r;

      obstacles.push_back(circle);
    }
  }

  return obstacles;
}

bool ScannerServer::isCircle(std::vector<geometry_msgs::PointStamped> points, float &radius, geometry_msgs::PointStamped &center) {
  // take first, middle and last point and find the circle radius and center
  Point pt1 = points[0];
  Point pt2 = points[std::round((points.size() - 1) / 2)];
  Point pt3 = points[points.size() - 1];

  float x12 = (pt1 - pt2).x;
  float x13 = (pt1 - pt3).x;
  float y12 = (pt1 - pt2).y;
  float y13 = (pt1 - pt3).y;

  float x21 = (pt2 - pt1).x;
  float x31 = (pt3 - pt1).x;
  float y21 = (pt2 - pt1).y;
  float y31 = (pt3 - pt1).y;

  float sx13 = pow(pt1.x, 2) - pow(pt3.x, 2);
  float sy13 = pow(pt1.y, 2) - pow(pt3.y, 2);
  float sx21 = pow(pt2.x, 2) - pow(pt1.x, 2);
  float sy21 = pow(pt2.y, 2) - pow(pt1.y, 2);

  float f = ((sx13) * (x12) + (sy13) * (x12) + (sx21) * (x13) + (sy21) * (x13)) / (2 * ((y31) * (x12) - (y21) * (x13)));
  float g = ((sx13) * (y12) + (sy13) * (y12) + (sx21) * (y13) + (sy21) * (y13)) / (2 * ((x31) * (y12) - (x21) * (y13)));

  float c = -pow(pt1.x, 2) - pow(pt1.y, 2) - 2 * g * pt1.x - 2 * f * pt1.y;

  // center coordinates
  float h = -g;
  float k = -f;

  // radius
  float sqr_of_r = h * h + k * k - c;
  float r = sqrt(sqr_of_r);

  // set radius and center
  radius = r;
  center.header.frame_id = points[0].header.frame_id;
  center.point.x = h;
  center.point.y = k;

  if (r >= minRadius && r <= maxRadius) {
    return true;
  }

  return false;
}

std::vector<std::vector<geometry_msgs::PointStamped>> ScannerServer::removeSmallClusters(std::vector<std::vector<geometry_msgs::PointStamped>> clusters) {
  std::vector<std::vector<geometry_msgs::PointStamped>> out;
  for (int i = 0; i < clusters.size(); i++) {
    if (clusters[i].size() > clusterMinSize) {
      out.push_back(clusters[i]);
    }
  }
  return out;
}
