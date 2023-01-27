#ifndef OBJECT_H
#define OBJECT_H

#include <XmlRpcValue.h>
#include <geometry_msgs/PoseStamped.h>

class Object {
 private:
  enum PICK_MODE { TOP = 0,
                   SIDE = 1 };
  int id;
  std::string name;

  PICK_MODE pickMode;

  geometry_msgs::PoseStamped robotPickPose;

  bool robotPlaceAuto;
  geometry_msgs::PoseStamped placePosition;

 public:
  Object() = default;
  Object(const XmlRpc::XmlRpcValue& object_);
  void parseObject(const XmlRpc::XmlRpcValue& object_);

  const int getId() const;
  const std::string getName() const;
  const PICK_MODE getPickMode() const;
  const geometry_msgs::PoseStamped getRobotPickPose() const;
};

#endif  // OBJECT_H