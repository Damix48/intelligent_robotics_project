#include <ros/node_handle.h>
#include <ros/ros.h>

#include "tiago_iaslab_simulation/Objs.h"
#include "tiago_iaslab_simulation/object.h"
#include "tiago_iaslab_simulation/robot.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_node");

  auto nh_ptr = std::make_shared<ros::NodeHandle>();

  // GETTING ALL OBJECTS
  XmlRpc::XmlRpcValue objects_;
  nh_ptr->getParam("/objects_to_pick", objects_);

  std::map<int, Object> objects;

  for (size_t i = 0; i < objects_.size(); i++) {
    ROS_INFO_STREAM(objects_[i]);
    Object temp(objects_[i]);
    objects[temp.getId()] = temp;
  }

  // GETTING SEQUENCE
  ros::ServiceClient sequenceClient = nh_ptr->serviceClient<tiago_iaslab_simulation::Objs>("human_objects_srv", true);
  if (!sequenceClient.waitForExistence(ros::Duration(5))) {
    ROS_ERROR("Human object service not available");
    return EXIT_FAILURE;
  }

  tiago_iaslab_simulation::Objs service;
  service.request.ready = true;
  service.request.all_objs = true;

  sequenceClient.call(service);

  std::vector<int> sequence = service.response.ids;
  std::vector<int> sequence_test = {2};

  // STARTING ROBOT
  Robot robot(nh_ptr);
  robot.start(objects, sequence_test);
  // robot.start(objects, sequence);
  // robot.removeObject(1);

  ros::spinOnce();

  return EXIT_SUCCESS;
}
