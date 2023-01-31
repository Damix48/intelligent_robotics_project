#include <ros/callback_queue.h>
#include <tiago_iaslab_simulation/find_color_server.h>

FindColorServer::FindColorServer(std::shared_ptr<ros::NodeHandle> nh, int tol, int dim_crop) : nh_(nh), tol_(tol), dim_crop_(dim_crop), it_(*nh) {
  colors_server_ = nh_->advertiseService("/find_color_srv", &FindColorServer::FindColorsrv, this);
  ROS_INFO("Service done!");
}

bool FindColorServer::FindColorsrv(tiago_iaslab_simulation::Colors::Request &req, tiago_iaslab_simulation::Colors::Response &resp) {
  target_color_[0] = req.target_color.x;
  target_color_[1] = req.target_color.y;
  target_color_[2] = req.target_color.z;

  scan_ = req.scan;
  found_ = false;
  done_ = false;

  // Subscribe to the topic of the camera, check if what you are seen is the target color
  image_sub_ = it_.subscribe("xtion/rgb/image_raw", 1, &FindColorServer::imageCb, this);
  while (done_ == false) {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }
  resp.found = found_;
}

void FindColorServer::imageCb(const sensor_msgs::ImageConstPtr &msg) {
  // convert image
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat img = cv_ptr->image;

  // Take the central part of the image
  int middle_row = std::floor(img.rows / 2);
  int middle_col = std::floor(img.cols / 2);
  int filter_size_2 = std::floor(dim_crop_ / 2);

  cv::Range rows(middle_row - filter_size_2, middle_row + filter_size_2);
  cv::Range cols(middle_col - filter_size_2, middle_col + filter_size_2);
  cv::Mat cropped = cv_ptr->image(rows, cols);

  // look if there are pixels with colos similar to target_color_: similar defined by the parametere tol_
  cv::Vec3b lb;
  lb[0] = std::max(0, target_color_[0] - tol_);
  lb[1] = std::max(0, target_color_[1] - tol_);
  lb[2] = std::max(0, target_color_[2] - tol_);
  cv::Vec3b ub;
  ub[0] = std::min(255, target_color_[0] + tol_);
  ub[1] = std::min(255, target_color_[1] + tol_);
  ub[2] = std::min(255, target_color_[2] + tol_);

  cv::Mat img2;
  cv::inRange(cropped, lb, ub, img2);

  int nonZero = cv::countNonZero(img2);
  if (nonZero != 0) {
    found_ = true;
  }
  done_ = true;
  image_sub_.shutdown();
}