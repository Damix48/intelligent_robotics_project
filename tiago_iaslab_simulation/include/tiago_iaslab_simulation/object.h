#ifndef TIAGO_IASLAB_SIMULATION_OBJECT_H
#define TIAGO_IASLAB_SIMULATION_OBJECT_H

#include <XmlRpcValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

namespace ObjectConstant {

enum PICK_MODE {
  TOP = 0,
  SIDE = 1
};

enum SHAPE {
  CUBE = 0,
  CYLINDER = 1
};

}  // namespace ObjectConstant

class Object {
 private:
  int id;
  std::string name;

  ObjectConstant::PICK_MODE pickMode;
  ObjectConstant::SHAPE shape;
  geometry_msgs::Vector3 color;
  std::vector<float> dimensions;
  float offset;

  geometry_msgs::PoseStamped robotPickPose;

  bool robotPlaceAuto;
  geometry_msgs::PoseStamped placePosition;

 public:
  Object() = default;
  Object(const XmlRpc::XmlRpcValue& object_);
  void parseObject(const XmlRpc::XmlRpcValue& object_);

  const int getId() const;
  const std::string getName() const;
  const ObjectConstant::PICK_MODE getPickMode() const;
  const ObjectConstant::SHAPE getShape() const;
  const geometry_msgs::Vector3 getColor() const;
  std::vector<float> getDimensions() const;
  float getOffset() const;
  const geometry_msgs::PoseStamped getRobotPickPose() const;
  const bool getRobotPlaceAuto() const;
  const geometry_msgs::PoseStamped getPlacePosition() const;
};

#endif  // TIAGO_IASLAB_SIMULATION_OBJECT_H