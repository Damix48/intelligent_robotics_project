#ifndef PARAM_UTILS_H
#define PARAM_UTILS_H

#include <XmlRpcValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include "tiago_iaslab_simulation/utils.h"

namespace iaslab {

class XmlRpcValue : public XmlRpc::XmlRpcValue {
 public:
  XmlRpcValue(XmlRpc::XmlRpcValue const& copy) : XmlRpc::XmlRpcValue(copy) {}

  operator float() {
    try {
      assertTypeOrInvalid(TypeDouble);
      return _value.asDouble;
    } catch (...) {
    }

    assertTypeOrInvalid(TypeInt);
    return _value.asInt;
  }

  operator geometry_msgs::PoseStamped() {
    XmlRpcValue x = (*_value.asStruct)["x"];
    XmlRpcValue y = (*_value.asStruct)["y"];
    XmlRpcValue yaw = (*_value.asStruct)["yaw"];

    geometry_msgs::PoseStamped pose_;
    pose_.header.frame_id = "map";
    pose_.header.stamp = ros::Time::now();
    pose_.pose = iaslab::createPose(x, y, yaw);

    return pose_;
  }
};

class PropertyParser {
 private:
  template <typename T>
  static T parse_(const iaslab::XmlRpcValue& object, std::string name) {
    iaslab::XmlRpcValue temp = object[name];

    return static_cast<T>(temp);
  }

  template <typename T>
  static T parseEnum_(const iaslab::XmlRpcValue& object, std::string name, std::map<std::string, T>& mapper) {
    ROS_ASSERT(std::is_enum<T>::value);

    std::string key = parse_<std::string>(object, name);

    return mapper[key];
  }

 public:
  template <typename T>
  static T parse(const iaslab::XmlRpcValue& object) {
    iaslab::XmlRpcValue temp = object;

    return static_cast<T>(temp);
  }

  template <typename T>
  static T parse(const iaslab::XmlRpcValue& object, std::string name) {
    if (!object.hasMember(name)) {
      throw ros::InvalidParameterException(name + " not found");
    }

    return parse_<T>(object, name);
  }

  template <typename T>
  static T parse(const iaslab::XmlRpcValue& object, std::string name, T default_) {
    if (!object.hasMember(name)) {
      return default_;
    }

    return parse_<T>(object, name);
  }

  template <typename T>
  static T parseEnum(const iaslab::XmlRpcValue& object, std::string name, std::map<std::string, T>& mapper) {
    if (!object.hasMember(name)) {
      throw ros::InvalidParameterException(name + " not found");
    }

    return parseEnum_<T>(object, name, mapper);
  }

  template <typename T>
  static T parseEnum(const iaslab::XmlRpcValue& object, std::string name, std::map<std::string, T>& mapper, T default_) {
    if (!object.hasMember(name)) {
      return default_;
    }

    return parseEnum_<T>(object, name, mapper);
  }

  template <typename T>
  static std::vector<T> parseVector(const iaslab::XmlRpcValue& object, std::string name) {
    std::vector<T> temp;

    const iaslab::XmlRpcValue vec = parse_<iaslab::XmlRpcValue>(object, name);

    for (size_t i = 0; i < vec.size(); i++) {
      iaslab::XmlRpcValue t = vec[i];

      temp.push_back(static_cast<T>(t));
    }

    return temp;
  }

  template <typename T>
  static std::vector<T> parseVector(const iaslab::XmlRpcValue& object) {
    std::vector<T> temp;

    const iaslab::XmlRpcValue vec = object;

    for (size_t i = 0; i < vec.size(); i++) {
      iaslab::XmlRpcValue t = vec[i];

      temp.push_back(static_cast<T>(t));
    }

    return temp;
  }
};

}  // namespace iaslab

#endif  // PARAM_UTILS_H