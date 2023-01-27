#ifndef TIAGO_IASLAB_SIMULATION_APRILTAGPOSES_H
#define TIAGO_IASLAB_SIMULATION_APRILTAGPOSES_H

#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tiago_iaslab_simulation/apriltagDetections.h>

class ApriltagPoses
{
public:
    ApriltagPoses(std::shared_ptr<ros::NodeHandle> nh_ptr, std::string target_frame);

private:
    std::shared_ptr<ros::NodeHandle> nh_ptr_;
    ros::Subscriber sub_;
    tf2_ros::Buffer buffer_;
    std::string target_frame_;
    ros::ServiceServer apriltagPoses_server_;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> transformed_poses_;
    std::vector<int> ids_;
    tf2_ros::TransformListener listener_;

    void start();
    void tag_callback(const apriltag_ros::AprilTagDetectionArrayConstPtr &msg);

    bool apriltagPosesService(tiago_iaslab_simulation::apriltagDetections::Request &req, tiago_iaslab_simulation::apriltagDetections::Response &res);
};
#endif