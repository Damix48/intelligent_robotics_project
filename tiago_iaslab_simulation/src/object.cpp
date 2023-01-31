#include "tiago_iaslab_simulation/object.h"

#include "tiago_iaslab_simulation/param_utils.h"

Object::Object(const XmlRpc::XmlRpcValue& object_) {
  parseObject(object_);
}

void Object::parseObject(const XmlRpc::XmlRpcValue& object_) {
  id = iaslab::PropertyParser::parse<int>(object_, "id");

  name = iaslab::PropertyParser::parse<std::string>(object_, "name", "");

  std::map<std::string, ObjectConstant::PICK_MODE> mapper_mode = {{"top", ObjectConstant::PICK_MODE::TOP}, {"side", ObjectConstant::PICK_MODE::SIDE}};
  pickMode = iaslab::PropertyParser::parseEnum<ObjectConstant::PICK_MODE>(object_, "pick_mode", mapper_mode);

  std::map<std::string, ObjectConstant::SHAPE> mapper_shape = {{"cube", ObjectConstant::SHAPE::CUBE}, {"cylinder", ObjectConstant::SHAPE::CYLINDER}};
  shape = iaslab::PropertyParser::parseEnum<ObjectConstant::SHAPE>(object_, "shape", mapper_shape, ObjectConstant::SHAPE::CUBE);

  std::vector<float> color_ = iaslab::PropertyParser::parseVector<float>(object_, "color");
  color.x = color_[0];
  color.y = color_[1];
  color.z = color_[2];

  dimensions = iaslab::PropertyParser::parseVector<float>(object_, "dimensions");

  offset = iaslab::PropertyParser::parse<float>(object_, "offset", 0);

  robotPickPose = iaslab::PropertyParser::parse<geometry_msgs::PoseStamped>(object_, "robot_pick_pose");

  robotPlaceAuto = iaslab::PropertyParser::parse<bool>(object_, "auto_place", false);

  if (!robotPlaceAuto) {
    placePosition = iaslab::PropertyParser::parse<geometry_msgs::PoseStamped>(object_, "place_pose");
  }
}

const int Object::getId() const {
  return id;
}

const std::string Object::getName() const {
  return name;
}

const ObjectConstant::PICK_MODE Object::getPickMode() const {
  return pickMode;
}

const ObjectConstant::SHAPE Object::getShape() const {
  return shape;
}

const geometry_msgs::Vector3 Object::getColor() const {
  return color;
}

std::vector<float> Object::getDimensions() const {
  return dimensions;
}

float Object::getOffset() const {
  return offset;
}

const geometry_msgs::PoseStamped Object::getRobotPickPose() const {
  return robotPickPose;
}

const bool Object::getRobotPlaceAuto() const {
  return robotPlaceAuto;
}

const geometry_msgs::PoseStamped Object::getPlacePosition() const {
  return placePosition;
}