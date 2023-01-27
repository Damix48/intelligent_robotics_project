#include "tiago_iaslab_simulation/object.h"

#include "tiago_iaslab_simulation/param_utils.h"

Object::Object(const XmlRpc::XmlRpcValue& object_) {
  parseObject(object_);
}

void Object::parseObject(const XmlRpc::XmlRpcValue& object_) {
  id = iaslab::PropertyParser::parse<int>(object_, "id");

  name = iaslab::PropertyParser::parse<std::string>(object_, "name", "");

  std::map<std::string, PICK_MODE> mapper = {{"top", PICK_MODE::TOP}, {"side", PICK_MODE::SIDE}};
  pickMode = iaslab::PropertyParser::parseEnum<PICK_MODE>(object_, "pick_mode", mapper);

  robotPickPose = iaslab::PropertyParser::parse<geometry_msgs::PoseStamped>(object_, "robot_pick_pose");

  robotPlaceAuto = iaslab::PropertyParser::parse<bool>(object_, "auto_place", false);

  if (!robotPlaceAuto) {
    robotPickPose = iaslab::PropertyParser::parse<geometry_msgs::PoseStamped>(object_, "place_pose");
  }
}

const int Object::getId() const {
  return id;
}

const geometry_msgs::PoseStamped Object::getRobotPickPose() const {
  return robotPickPose;
}