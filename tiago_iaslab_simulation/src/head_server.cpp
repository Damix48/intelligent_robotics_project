#include "tiago_iaslab_simulation/head_server.h"

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "tiago_iaslab_simulation/apriltagDetections.h"

HeadServer::HeadServer(std::shared_ptr<ros::NodeHandle> nodeHandle_,
                       std::string headServerTopic,
                       std::string headControllerTopic,
                       std::vector<std::string> headJointNames_,
                       std::string torsoControllerTopic,
                       std::vector<std::string> torsoJointNames_,
                       std::string apriltagTopic_,
                       std::string baseFrame_) : nodeHandle(nodeHandle_),
                                                 actionServer(*nodeHandle, headServerTopic, boost::bind(&HeadServer::move, this, _1), false),
                                                 headController(headControllerTopic),
                                                 headJointNames(headJointNames_),
                                                 torsoController(torsoControllerTopic),
                                                 torsoJointNames(torsoJointNames_),
                                                 apriltagTopic(apriltagTopic_),
                                                 baseFrame(baseFrame_) {
  actionServer.start();
}

void HeadServer::move(const tiago_iaslab_simulation::headGoalConstPtr& goal) {
  headController.waitForServer();
  control_msgs::FollowJointTrajectoryGoal trajectoryGoal = generateHeadTrajectoryGoal(goal->pitch, goal->yaw);

  headController.sendGoal(trajectoryGoal);

  bool timeout = headController.waitForResult(ros::Duration(10));

  ros::Duration(1).sleep();

  if (!timeout) {
    actionServer.setAborted();
  } else {
    if (headController.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      if (goal->return_object) {
        torsoController.waitForServer();
        control_msgs::FollowJointTrajectoryGoal trajectoryTorsoGoal = generateTorsoTrajectoryGoal(0.15);
        torsoController.sendGoal(trajectoryTorsoGoal);
        torsoController.waitForResult(ros::Duration(10));

        getObjects();
      } else {
        actionServer.setSucceeded();
      }
    }
  }
}

void HeadServer::getObjects() {
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);

  boost::shared_ptr<const apriltag_ros::AprilTagDetectionArray> msg = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>(apriltagTopic, ros::Duration(1));

  std::vector<apriltag_ros::AprilTagDetection> tags;

  for (const auto& detection : msg->detections) {
    geometry_msgs::TransformStamped transformStamped = buffer.lookupTransform(baseFrame, detection.pose.header.frame_id, ros::Time(0), ros::Duration(5));
    geometry_msgs::PoseWithCovarianceStamped outPose;
    tf2::doTransform(detection.pose, outPose, transformStamped);

    apriltag_ros::AprilTagDetection tag;
    tag.id = detection.id;
    tag.size = detection.size;
    tag.pose = outPose;

    tags.push_back(tag);
  }

  tiago_iaslab_simulation::headResult result;
  result.tags = tags;

  actionServer.setSucceeded(result);
}

control_msgs::FollowJointTrajectoryGoal HeadServer::generateHeadTrajectoryGoal(float pitch, float yaw) {
  control_msgs::FollowJointTrajectoryGoal headGoal;

  headGoal.trajectory.joint_names = headJointNames;

  headGoal.trajectory.points.resize(1);

  headGoal.trajectory.points[0].positions.resize(2);
  headGoal.trajectory.points[0].positions[0] = yaw;
  headGoal.trajectory.points[0].positions[1] = pitch;

  headGoal.trajectory.points[0].velocities.resize(2);
  headGoal.trajectory.points[0].velocities[0] = yaw;
  headGoal.trajectory.points[0].velocities[1] = pitch;

  headGoal.trajectory.points[0].time_from_start = ros::Duration(2.0);

  return headGoal;
}

control_msgs::FollowJointTrajectoryGoal HeadServer::generateTorsoTrajectoryGoal(float v) {
  control_msgs::FollowJointTrajectoryGoal torsoGoal;

  torsoGoal.trajectory.joint_names = torsoJointNames;

  torsoGoal.trajectory.points.resize(1);

  torsoGoal.trajectory.points[0].positions.resize(1);
  torsoGoal.trajectory.points[0].positions[0] = v;

  torsoGoal.trajectory.points[0].time_from_start = ros::Duration(2.0);

  return torsoGoal;
}

void HeadServer::publishFeedback(const uint status) {
  tiago_iaslab_simulation::headFeedback feedback;
  feedback.current_status = status;

  actionServer.publishFeedback(feedback);
}
