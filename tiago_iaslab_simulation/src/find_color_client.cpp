#include "tiago_iaslab_simulation/find_color_client.h"

#include "tiago_iaslab_simulation/Colors.h"

FindColorClient::FindColorClient(std::shared_ptr<ros::NodeHandle> nodeHandle_,
                                 HeadClient* headClient_,
                                 std::string colorServerTopic) : nodeHandle(nodeHandle_),
                                                                 headClient(headClient_) {
  colorClient = nodeHandle->serviceClient<tiago_iaslab_simulation::Colors>(colorServerTopic);
}

geometry_msgs::PoseStamped FindColorClient::getPose(geometry_msgs::Vector3 targetColor,
                                                std::vector<tiago_iaslab_simulation::circle> tables) {
  tiago_iaslab_simulation::Colors service;

  service.request.target_color = targetColor;

  int idx = -1;
  for (int i = 0; i < tables.size(); i++) {
    headClient->move(tables[i].center);

    service.request.scan = tables[i];
    if (colorClient.call(service)) {
      if (service.response.found) {
        ROS_INFO("Target color found");
        idx = i;
        break;
      }
      ROS_INFO("Ok");
    } else {
      ROS_WARN("Can not establish connection with ColorOrder service");
    }
  }

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);

  geometry_msgs::PoseStamped center;
  center.header.frame_id = "map";
  center.pose.position = buffer.transform(tables[idx].center, center.header.frame_id, ros::Time(0), tables[idx].center.header.frame_id, ros::Duration(2)).point;

  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI / 2);
  q.normalize();

  geometry_msgs::Quaternion orientation;
  orientation.x = q.getX();
  orientation.y = q.getY();
  orientation.z = q.getZ();
  orientation.w = q.getW();
  center.pose.orientation = orientation;

  return center;
}
