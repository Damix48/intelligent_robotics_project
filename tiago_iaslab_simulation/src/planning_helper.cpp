#include "tiago_iaslab_simulation/planning_helper.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "tiago_iaslab_simulation/param_utils.h"

PlanningHelper::PlanningHelper() {
  XmlRpc::XmlRpcValue pickTableParam;
  ros::param::get("pick_table_dimensions", pickTableParam);
  pickTableDimensions = iaslab::PropertyParser::parseVector<float>(pickTableParam);

  XmlRpc::XmlRpcValue obstacleParam;
  ros::param::get("obstacle_dimensions", obstacleParam);
  obstacleDimensions = iaslab::PropertyParser::parseVector<float>(obstacleParam);

  XmlRpc::XmlRpcValue placeTableParam;
  ros::param::get("place_table_dimensions", placeTableParam);
  placeTableDimensions = iaslab::PropertyParser::parseVector<float>(placeTableParam);
}

void PlanningHelper::addObject(Object object, geometry_msgs::Pose pose) {
  moveit_msgs::CollisionObject obj;
  obj.header.frame_id = "base_footprint";
  obj.id = std::to_string(object.getId());

  obj.primitives.resize(1);

  switch (object.getShape()) {
    case ObjectConstant::CYLINDER:
      obj.primitives[0].type = obj.primitives[0].CYLINDER;
      break;

    default:
      obj.primitives[0].type = obj.primitives[0].BOX;
      break;
  }

  std::vector<float> dimensions_ = object.getDimensions();
  std::vector<double> dimensions(dimensions_.begin(), dimensions_.end());
  obj.primitives[0].dimensions = dimensions;

  obj.primitive_poses.resize(1);
  obj.primitive_poses[0].position = pose.position;
  obj.primitive_poses[0].position.z -= (obj.primitives[0].dimensions[0] - 0.02) / 2;

  obj.operation = obj.ADD;

  auto t_ = planningSceneInterface.getObjects();
  std::vector<moveit_msgs::CollisionObject> t;
  for (auto p : t_) {
    t.push_back(p.second);
  }
  t.push_back(obj);

  planningSceneInterface.addCollisionObjects(t);
}

void PlanningHelper::addObstacle(geometry_msgs::Pose pose) {
  moveit_msgs::CollisionObject obj;

  obj.header.frame_id = "base_footprint";
  obj.id = "obstacle_" + std::to_string(std::rand());

  obj.primitives.resize(1);

  obj.primitives[0].type = obj.primitives[0].CYLINDER;

  std::vector<double> dimensions(obstacleDimensions.begin(), obstacleDimensions.end());
  obj.primitives[0].dimensions = dimensions;

  obj.primitive_poses.resize(1);
  obj.primitive_poses[0].position = pose.position;
  obj.primitive_poses[0].position.z -= (dimensions[0] - 0.02) / 2;

  obj.operation = obj.ADD;

  auto t_ = planningSceneInterface.getObjects();
  std::vector<moveit_msgs::CollisionObject> t;
  for (auto p : t_) {
    t.push_back(p.second);
  }
  t.push_back(obj);

  planningSceneInterface.addCollisionObjects(t);
}

void PlanningHelper::addPickTable() {
  float length = pickTableDimensions[0],
        width = pickTableDimensions[1],
        height = pickTableDimensions[2],
        z = pickTableDimensions[3],
        distance = pickTableDimensions[4];

  moveit_msgs::CollisionObject obj;

  obj.header.frame_id = "base_footprint";
  obj.id = "table";

  obj.primitives.resize(1);
  obj.primitives[0].type = obj.primitives[0].BOX;
  obj.primitives[0].dimensions.resize(3);
  obj.primitives[0].dimensions[0] = width;
  obj.primitives[0].dimensions[1] = length;
  obj.primitives[0].dimensions[2] = height;

  obj.primitive_poses.resize(1);
  obj.primitive_poses[0].position.x = distance;
  obj.primitive_poses[0].position.y = 0;
  obj.primitive_poses[0].position.z = z - 0.05;

  obj.operation = obj.ADD;

  auto t_ = planningSceneInterface.getObjects();
  std::vector<moveit_msgs::CollisionObject> t;
  for (auto p : t_) {
    t.push_back(p.second);
  }
  t.push_back(obj);

  planningSceneInterface.addCollisionObjects(t);
}

void PlanningHelper::addPlaceTable(geometry_msgs::PoseStamped pose_) {
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  geometry_msgs::Pose pose = buffer.transform(pose_, "base_footprint", ros::Time(0), pose_.header.frame_id, ros::Duration(5)).pose;

  moveit_msgs::CollisionObject obj;

  obj.header.frame_id = "base_footprint";
  obj.id = "place_table";

  obj.primitives.resize(1);

  obj.primitives[0].type = obj.primitives[0].CYLINDER;

  std::vector<double> dimensions(placeTableDimensions.begin(), placeTableDimensions.end());
  obj.primitives[0].dimensions = dimensions;

  obj.primitive_poses.resize(1);
  obj.primitive_poses[0].position = pose.position;
  obj.primitive_poses[0].position.z = placeTableDimensions[0] / 2;

  obj.operation = obj.ADD;

  auto t_ = planningSceneInterface.getObjects();
  std::vector<moveit_msgs::CollisionObject> t;
  for (auto p : t_) {
    t.push_back(p.second);
  }
  t.push_back(obj);

  planningSceneInterface.addCollisionObjects(t);
}

void PlanningHelper::removeObject(std::string id) {
  planningSceneInterface.removeCollisionObjects({id});
}

void PlanningHelper::removeObject(int id) {
  removeObject(std::to_string(id));
}

void PlanningHelper::clear() {
  planningSceneInterface.removeCollisionObjects(planningSceneInterface.getKnownObjectNames());
}