#include <ros/ros.h>
#include <tiago_iaslab_simulation/ApriltagPoses.h>

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        ROS_INFO("Usage: file_name target_frame");
        return 1;
    }
    ros::init(argc, argv, "nodeB");
    auto nh_ptr = std::make_shared<ros::NodeHandle>();

    std::string target_frame = argv[1];
    ApriltagPoses apriltagPoses(nh_ptr, target_frame);

    ros::spin();

    return 0;
}