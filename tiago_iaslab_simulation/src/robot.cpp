#include "tiago_iaslab_simulation/robot.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "tiago_iaslab_simulation/apriltagDetections.h"
#include "tiago_iaslab_simulation/object.h"

Robot::Robot(std::shared_ptr<ros::NodeHandle> nodeHandle_,
             std::string apriltagServiceTopic) : nodeHandle(nodeHandle_),
                                                 moveScanClient(nodeHandle_, true) {
  apriltagDetectionsService = nodeHandle->serviceClient<tiago_iaslab_simulation::apriltagDetections>(apriltagServiceTopic, true);
  visualizer = nodeHandle->advertise<visualization_msgs::Marker>("visualization_marker", 1);
}

void Robot::start(std::map<int, Object> objects, std::vector<int> sequence) {
  for (int id : sequence) {
    // Move to pick pose without scanning
    ROS_INFO_STREAM(id);
    moveScanClient.moveTo(8.55, -1.5, 0, false);

    ros::Duration(2).sleep();
    moveScanClient.moveTo(objects[id].getRobotPickPose(), false);
    ros::Duration(2).sleep();

    // AprilTag stuff
    if (!apriltagDetectionsService.waitForExistence(ros::Duration(5))) {
      throw ros::Exception("Apriltag detections service not exist.");
    }

    tiago_iaslab_simulation::apriltagDetections apriltagDetectionsMessage;
    apriltagDetectionsService.call(apriltagDetectionsMessage);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // moveit::planning_interface::MoveGroupInterface move_group("arm_torso");
    // moveit::planning_interface::MoveGroupInterface gripper_group("gripper");

    // for (auto in : move_group.getNamedTargets()) {
    //   ROS_INFO_STREAM(in);
    // }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group("arm_torso");

    std::vector<moveit_msgs::CollisionObject> collision_objects;

    int q = 0;

    collision_objects.resize(apriltagDetectionsMessage.response.ids.size() + 1);
    for (size_t i = 0; i < apriltagDetectionsMessage.response.ids.size(); i++) {
      if (apriltagDetectionsMessage.response.ids[i] == id) {
        q = i;
        collision_objects[i].header.frame_id = "base_footprint";
        collision_objects[i].id = "object";

        /* Define the primitive and its dimensions. */
        collision_objects[i].primitives.resize(1);
        collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].BOX;
        collision_objects[i].primitives[0].dimensions.resize(3);
        collision_objects[i].primitives[0].dimensions[0] = 0.05;
        collision_objects[i].primitives[0].dimensions[1] = 0.05;
        collision_objects[i].primitives[0].dimensions[2] = 0.05;

        collision_objects[i].primitive_poses.resize(1);
        collision_objects[i].primitive_poses[0].position = apriltagDetectionsMessage.response.poses[i].pose.pose.position;
        collision_objects[i].primitive_poses[0].position.z -= 0.05 / 2;

        collision_objects[i].operation = collision_objects[i].ADD;

        // moveit::planning_interface::MoveGroupInterface move_group("arm_torso");
        // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // // move_group.setPlannerId("SBLkConfigDefault");
        // // move_group.setPoseReferenceFrame("base_footprint");

        // move_group.setStartStateToCurrentState();
        // // move_group.setMaxVelocityScalingFactor(1.0);

        // geometry_msgs::Pose target_pose1;
        // target_pose1 = apriltagDetectionsMessage.response.poses[i].pose.pose;

        // // ROS_INFO_STREAM(target_pose1);

        // target_pose1.position.z += 0.3;
        // tf2::Quaternion orientation;
        // orientation.setRPY(-M_PI / 2, M_PI / 2, M_PI / 2);
        // target_pose1.orientation.x = orientation.x();
        // target_pose1.orientation.y = orientation.y();
        // target_pose1.orientation.z = orientation.z();
        // target_pose1.orientation.w = orientation.w();

        // target_pose1.orientation = apriltagDetectionsMessage.response.poses[i].pose.pose.orientation;

        // // target_pose1.orientation.x -= M_PI / 2;

        // tf2::Quaternion boh;
        // boh.setX(target_pose1.orientation.x);
        // boh.setY(target_pose1.orientation.y);
        // boh.setZ(target_pose1.orientation.z);
        // boh.setW(target_pose1.orientation.w);

        // visualization_msgs::Marker center;
        // center.pose = target_pose1;

        // center.action = center.ADD;
        // center.header = apriltagDetectionsMessage.response.poses[i].header;
        // center.ns = "centers";
        // center.id = i;
        // center.type = center.SPHERE;

        // center.scale.x = 0.1;
        // center.scale.y = 0.1;
        // center.scale.z = 0.1;

        // center.color.r = 0.0f;
        // center.color.g = 1.0f;
        // center.color.b = 0.0f;
        // center.color.a = 1.0;

        // center.lifetime = ros::Duration(0);

        // visualizer.publish(center);

        // // ROS_INFO_STREAM(target_pose1);

        // move_group.setPoseTarget(target_pose1);

        // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

        // move_group.move();
      } else {
        collision_objects[i].header.frame_id = "base_footprint";
        collision_objects[i].id = "obstacle_" + i;

        /* Define the primitive and its dimensions. */
        collision_objects[i].primitives.resize(1);
        collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].CYLINDER;
        collision_objects[i].primitives[0].dimensions.resize(3);
        collision_objects[i].primitives[0].dimensions[0] = 0.25;
        collision_objects[i].primitives[0].dimensions[1] = 0.07;

        collision_objects[i].primitive_poses.resize(1);
        collision_objects[i].primitive_poses[0].position = apriltagDetectionsMessage.response.poses[i].pose.pose.position;
        collision_objects[i].primitive_poses[0].position.z -= 0.1;

        collision_objects[i].operation = collision_objects[i].ADD;
      }
    }
    auto boh = move_group.getCurrentPose();
    ROS_INFO_STREAM(boh);

    collision_objects.back().header.frame_id = "base_footprint";
    collision_objects.back().id = "table";

    /* Define the primitive and its dimensions. */
    collision_objects.back().primitives.resize(1);
    collision_objects.back().primitives[0].type = collision_objects.back().primitives[0].BOX;
    collision_objects.back().primitives[0].dimensions.resize(3);
    collision_objects.back().primitives[0].dimensions[0] = 1;
    collision_objects.back().primitives[0].dimensions[1] = 3;
    collision_objects.back().primitives[0].dimensions[2] = 0.75;

    collision_objects.back().primitive_poses.resize(1);
    collision_objects.back().primitive_poses[0].position.x = 0.85;
    collision_objects.back().primitive_poses[0].position.y = 0;
    collision_objects.back().primitive_poses[0].position.z = 0.75 / 2;

    collision_objects.back().operation = collision_objects.back().ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);

    // move_group.setPlannerId("SBLkConfigDefault");
    // move_group.setPoseReferenceFrame("base_footprint");

    move_group.setStartStateToCurrentState();
    // move_group.setMaxVelocityScalingFactor(1.0);

    geometry_msgs::Pose target_pose1;
    target_pose1 = apriltagDetectionsMessage.response.poses[q].pose.pose;

    // ROS_INFO_STREAM(target_pose1);

    target_pose1.position.z += 0.3;
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, M_PI / 2, M_PI / 2);
    target_pose1.orientation.x = orientation.x();
    target_pose1.orientation.y = orientation.y();
    target_pose1.orientation.z = orientation.z();
    target_pose1.orientation.w = orientation.w();

    // target_pose1.orientation = apriltagDetectionsMessage.response.poses[q].pose.pose.orientation;

    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group.move();

    ros::Duration(2).sleep();

    collision_objects[q].operation = collision_objects[q].REMOVE;
    planning_scene_interface.applyCollisionObjects(collision_objects);

    ros::Duration(2).sleep();

    // target_pose1.position.y -= 0.2;
    target_pose1.position.z -= 0.1;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan3;

    move_group.setGoalPositionTolerance(0.01);
    move_group.setPoseTarget(target_pose1);

    bool success3 = (move_group.plan(my_plan3) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success3 ? "" : "FAILED");

    move_group.move();

    ros::Duration(10).sleep();

    move_group.setPoseTarget(boh);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan2;

    bool success2 = (move_group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success2 ? "" : "FAILED");

    move_group.move();

    // // GRASPS
    // std::vector<moveit_msgs::Grasp> grasps;
    // grasps.resize(1);
    // grasps[0].allowed_touch_objects.push_back("arm_tool_link");
    // grasps[0].allowed_touch_objects.push_back("arm_7_link");
    // grasps[0].allowed_touch_objects.push_back("gripper_left_finger_link");
    // grasps[0].allowed_touch_objects.push_back("gripper_left_finger_link");

    // grasps[0].grasp_pose.header.frame_id = "base_footprint";
    // tf2::Quaternion orientation;
    // orientation.setRPY(-M_PI / 2, M_PI / 2, M_PI / 2);
    // grasps[0].grasp_pose.pose.orientation.x = orientation.x();
    // grasps[0].grasp_pose.pose.orientation.y = orientation.y();
    // grasps[0].grasp_pose.pose.orientation.z = orientation.z();
    // grasps[0].grasp_pose.pose.orientation.w = orientation.w();

    // grasps[0].grasp_pose.pose = collision_objects[q].primitive_poses[0];
    // grasps[0].grasp_pose.pose.position.z += 0.025;
    // // // grasps[0].grasp_pose.pose.position.y

    // grasps[0].pre_grasp_approach.direction.header.frame_id = "base_footprint";
    // /* Direction is set as positive x axis */
    // grasps[0].pre_grasp_approach.direction.vector.z = 0.4;
    // grasps[0].pre_grasp_approach.min_distance = 0.195;
    // grasps[0].pre_grasp_approach.desired_distance = 0.255;

    // grasps[0].post_grasp_retreat.direction.header.frame_id = "base_footprint";
    // /* Direction is set as positive z axis */
    // grasps[0].post_grasp_retreat.direction.vector.z = 0.4;
    // grasps[0].post_grasp_retreat.min_distance = 0.1;
    // grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // auto &posture = grasps[0].pre_grasp_posture;

    // posture.joint_names.resize(2);
    // posture.joint_names[0] = "gripper_left_finger_joint";
    // posture.joint_names[1] = "gripper_right_finger_joint";

    // /* Set them as closed. */
    // posture.points.resize(1);
    // posture.points[0].positions.resize(2);
    // posture.points[0].positions[0] = 0.044;
    // posture.points[0].positions[1] = 0.044;
    // posture.points[0].time_from_start = ros::Duration(0.5);

    // auto &posture2 = grasps[0].grasp_posture;

    // posture2.joint_names.resize(2);
    // posture2.joint_names[0] = "gripper_left_finger_joint";
    // posture2.joint_names[1] = "gripper_right_finger_joint";

    // /* Set them as closed. */
    // posture2.points.resize(1);
    // posture2.points[0].positions.resize(2);
    // posture2.points[0].positions[0] = 0.00;
    // posture2.points[0].positions[1] = 0.00;
    // posture2.points[0].time_from_start = ros::Duration(0.5);

    // ROS_INFO("ciao");

    // // move_group.setSupportSurfaceName("table");

    // move_group.pick("object", grasps);

    spinner.stop();
  }
}

void Robot::removeObject(int id) {
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("arm_torso");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  collision_object.id = "box1";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = -0.2;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // ROS_INFO("boh-1");

  // move_group.setPlannerId("SBLkConfigDefault");
  // move_group.setPoseReferenceFrame("base_footprint");
  // // move_group.setPoseTarget(goal_pose);

  // ROS_INFO_STREAM("Planning to move " << move_group.getEndEffectorLink() << " to a target pose expressed in " << move_group.getPlanningFrame());

  // ROS_INFO("boh");

  // // const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("arm_torso");

  // geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.w = 1;
  // target_pose1.position.x = 0.1;
  // target_pose1.position.y = 0.1;
  // target_pose1.position.z = 0.1;
  // move_group.setPoseTarget(target_pose1);

  // move_group.setStartStateToCurrentState();
  // move_group.setMaxVelocityScalingFactor(1.0);
  // ROS_INFO("boh2");

  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // ROS_INFO("boh3");

  // ROS_INFO_STREAM(move_group.getPlanningTime());
  // // move_group.setPlanningTime(1);
  // // ROS_INFO_STREAM(move_group.getPlanningTime());

  // // ros::Duration(5).sleep();

  // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // ROS_INFO("boh4");

  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // move_group.move();

  spinner.stop();

  // ROS_INFO("boh5");
}