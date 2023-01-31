#ifndef TIAGO_IASLAB_SIMULATION_FIND_COLOR_SERVER_H
#define TIAGO_IASLAB_SIMULATION_FIND_COLOR_SERVER_H

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

class FindColorServer {
 private:
  std::shared_ptr<ros::NodeHandle> nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::ServiceServer colors_server_;

  cv::Vec3b target_color_;
  tiago_iaslab_simulation::circle scan_;
  bool done_ = false;
  bool found_ = false;
  int tol_;
  int dim_crop_;

 public:
  FindColorServer(std::shared_ptr<ros::NodeHandle> nh, int tol, int dim_crop);
  bool FindColorsrv(tiago_iaslab_simulation::Colors::Request &req, tiago_iaslab_simulation::Colors::Response &resp);
  void imageCb(const sensor_msgs::ImageConstPtr &msg);
};

#endif  // TIAGO_IASLAB_SIMULATION_FIND_COLOR_SERVER_H