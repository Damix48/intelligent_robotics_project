#ifndef TIAGO_IASLAB_SIMULATION_ARM_CLIENT_H
#define TIAGO_IASLAB_SIMULATION_ARM_CLIENT_H

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include "tiago_iaslab_simulation/object.h"
#include "tiago_iaslab_simulation/pickAction.h"
#include "tiago_iaslab_simulation/placeAction.h"

class ArmClient {
 private:
  std::shared_ptr<ros::NodeHandle> nodeHandle;

  ros::Publisher defaultArmPosePubblisher;
  actionlib::SimpleActionClient<tiago_iaslab_simulation::pickAction> pickClient;
  actionlib::SimpleActionClient<tiago_iaslab_simulation::placeAction> placeClient;

  bool print;

 public:
  ArmClient(std::shared_ptr<ros::NodeHandle> nodeHandle_,
            bool print_ = true,
            std::string armServerTopic = "arm_server");
  void setDefaultArmPose(bool shutdown);
  bool pick(Object obj, geometry_msgs::Pose pose);
  bool place(Object obj, geometry_msgs::PoseStamped pose);
};

#endif  // TIAGO_IASLAB_SIMULATION_ARM_CLIENT_H