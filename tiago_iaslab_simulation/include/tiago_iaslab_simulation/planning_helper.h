#ifndef TIAGO_IASLAB_SIMULATION_PLANNING_HELPER_H
#define TIAGO_IASLAB_SIMULATION_PLANNING_HELPER_H

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include "tiago_iaslab_simulation/object.h"

class PlanningHelper {
 private:
  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

  std::vector<float> pickTableDimensions;
  std::vector<float> obstacleDimensions;
  std::vector<float> placeTableDimensions;

 public:
  PlanningHelper();

  void addObject(Object object, geometry_msgs::Pose pose);
  void addObstacle(geometry_msgs::Pose pose);
  void addPickTable();
  void addPlaceTable(geometry_msgs::PoseStamped pose_);
  void removeObject(std::string id);
  void removeObject(int id);
  void clear();
};

#endif  // TIAGO_IASLAB_SIMULATION_PLANNING_HELPER_H