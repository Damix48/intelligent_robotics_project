#include <tiago_iaslab_simulation/ApriltagPoses.h>

ApriltagPoses::ApriltagPoses(std::shared_ptr<ros::NodeHandle> nh_ptr, std::string target_frame) : listener_(buffer_) {
  nh_ptr_ = nh_ptr;
  target_frame_ = target_frame;

  start();
}

void ApriltagPoses::start() {
  apriltagPoses_server_ = nh_ptr_->advertiseService("/apriltag_poses_srv", &ApriltagPoses::apriltagPosesService, this);
  ROS_INFO_STREAM("Service done!");
}

bool ApriltagPoses::apriltagPosesService(tiago_iaslab_simulation::apriltagDetections::Request &req, tiago_iaslab_simulation::apriltagDetections::Response &res) {
  ids_.clear();
  transformed_poses_.clear();
  ROS_INFO("Sono qui ciao");
  // sub_ = nh_ptr_->subscribe("tag_detections", 1000, &ApriltagPoses::tag_callback, this);

  boost::shared_ptr<const apriltag_ros::AprilTagDetectionArray>
      msg = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("tag_detections", ros::Duration(1));
  std::string source_frame = msg->detections[0].pose.header.frame_id;  //"xtion_rgb_optical_frame";

  geometry_msgs::TransformStamped transformStamped = buffer_.lookupTransform(target_frame_, source_frame, ros::Time(0));
  for (int i = 0; i < msg->detections.size(); i++) {
    geometry_msgs::PoseWithCovarianceStamped outPose;
    tf2::doTransform(msg->detections[i].pose, outPose, transformStamped);

    // outPose = buffer_.transform(msg->detections[i].pose, target_frame_, ros::Time(0), msg->detections[i].pose.header.frame_id);
    int id = msg->detections[i].id[0];
    ids_.push_back(id);
    transformed_poses_.push_back(outPose);
  }

  res.poses = transformed_poses_;
  res.ids = ids_;
  return true;
}

void ApriltagPoses::tag_callback(const apriltag_ros::AprilTagDetectionArrayConstPtr &msg) {
  std::string source_frame = msg->detections[0].pose.header.frame_id;
  geometry_msgs::TransformStamped transformStamped = buffer_.lookupTransform(target_frame_, source_frame, ros::Time(0));
  for (int i = 0; i < msg->detections.size(); i++) {
    geometry_msgs::PoseWithCovarianceStamped outPose;
    tf2::doTransform(msg->detections[i].pose, outPose, transformStamped);

    // outPose = buffer_.transform(msg->detections[i].pose, target_frame_, ros::Time(0), msg->detections[i].pose.header.frame_id);
    int id = msg->detections[i].id[0];
    ids_.push_back(id);
    transformed_poses_.push_back(outPose);
  }
}
