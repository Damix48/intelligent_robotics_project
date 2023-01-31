#ifndef TIAGO_IASLAB_SIMULATION_ARM_CLIENT_H
#define TIAGO_IASLAB_SIMULATION_ARM_CLIENT_H

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include "tiago_iaslab_simulation/object.h"
#include "tiago_iaslab_simulation/pickAction.h"
#include "tiago_iaslab_simulation/placeAction.h"

class ArmClient {
 private:
  std::shared_ptr<ros::NodeHandle> nodeHandle;

  actionlib::SimpleActionClient<tiago_iaslab_simulation::pickAction> pickClient;
  actionlib::SimpleActionClient<tiago_iaslab_simulation::placeAction> placeClient;

  bool print;

 public:
  ArmClient(std::shared_ptr<ros::NodeHandle> nodeHandle_,
            bool print_ = true,
            std::string armServerTopic = "arm_server");
  bool pick(Object obj);
  bool place(Object obj, geometry_msgs::Pose pose = geometry_msgs::Pose());
};

#endif  // TIAGO_IASLAB_SIMULATION_ARM_CLIENT_H