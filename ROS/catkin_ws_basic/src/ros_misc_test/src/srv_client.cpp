#include <ros/ros.h>
#include <std_srvs/Trigger.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_ros_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("generate_random_center");

    std_srvs::Trigger data;
    if (client.call(data)) {
        ROS_INFO("Request the data from server: %d", data.response.success);
    } else {
        ROS_INFO("Not get response");
    }

    return 0;
}