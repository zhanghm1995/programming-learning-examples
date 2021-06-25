#include <ros/ros.h>
#include <std_srvs/Trigger.h>

bool Generate(std_srvs::TriggerRequest& req,
              std_srvs::TriggerResponse& res)
{
    ROS_INFO("Has a request...");
    res.success = true;
    res.message = "zhanghaiming";
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_ros_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("generate_random_center", Generate);
    ros::spin();

    return 0;
}