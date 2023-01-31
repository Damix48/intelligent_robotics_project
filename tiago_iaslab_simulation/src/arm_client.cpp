#include "tiago_iaslab_simulation/arm_client.h"

ArmClient::ArmClient(std::shared_ptr<ros::NodeHandle> nodeHandle_,
                     bool print_,
                     std::string armServerTopic) : nodeHandle(nodeHandle_),
                                                   pickClient(armServerTopic + "/pick"),
                                                   placeClient(armServerTopic + "/place"),
                                                   print(print_) {}