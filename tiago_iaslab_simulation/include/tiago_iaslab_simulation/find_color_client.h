#ifndef TIAGO_IASLAB_SIMULATION_FIND_COLOR_CLIENT_H
#define TIAGO_IASLAB_SIMULATION_FIND_COLOR_CLIENT_H

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Vector3.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "tiago_iaslab_simulation/Colors.h"
#include "tiago_iaslab_simulation/circle.h"
#include "tiago_iaslab_simulation/head_client.h"

class FindColorClient {
 private:
  std::shared_ptr<ros::NodeHandle> nodeHandle;
  HeadClient* headClient;

  ros::ServiceClient colorClient;

  geometry_msgs::PoseStamped getPlacePose(tiago_iaslab_simulation::circle target, float distance);

 public:
  FindColorClient(std::shared_ptr<ros::NodeHandle> nodeHandle_,
                  HeadClient* headClient_,
                  std::string colorServerTopic = "find_color_srv");

  geometry_msgs::PoseStamped getPose(geometry_msgs::Vector3 targetColor,
                                 std::vector<tiago_iaslab_simulation::circle> tables);
};

#endif  // TIAGO_IASLAB_SIMULATION_FIND_COLOR_CLIENT_H