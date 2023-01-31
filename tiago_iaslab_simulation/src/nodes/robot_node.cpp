#include <ros/node_handle.h>
#include <ros/ros.h>

#include "tiago_iaslab_simulation/Objs.h"
#include "tiago_iaslab_simulation/object.h"
#include "tiago_iaslab_simulation/param_utils.h"
#include "tiago_iaslab_simulation/robot.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_node");

  auto nh_ptr = std::make_shared<ros::NodeHandle>();

  // GETTING PARAMS
  XmlRpc::XmlRpcValue param;

  geometry_msgs::PoseStamped pickWaypoint;
  if (nh_ptr->getParam("/pick_waypoint", param)) {
    pickWaypoint = iaslab::PropertyParser::parse<geometry_msgs::PoseStamped>(param);
  }

  geometry_msgs::PoseStamped placeWaypoint;
  if (nh_ptr->getParam("/place_waypoint", param)) {
    placeWaypoint = iaslab::PropertyParser::parse<geometry_msgs::PoseStamped>(param);
  }

  // GETTING ALL OBJECTS
  XmlRpc::XmlRpcValue objects_;
  nh_ptr->getParam("/objects_to_pick", objects_);

  std::map<int, Object> objects;

  for (size_t i = 0; i < objects_.size(); i++) {
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

  // STARTING ROBOT
  Robot robot(nh_ptr);
  robot.setPickWaypoint(pickWaypoint);
  robot.setPlaceWaypoint(placeWaypoint);
  robot.start(objects, sequence);

  ros::spinOnce();

  return EXIT_SUCCESS;
}
