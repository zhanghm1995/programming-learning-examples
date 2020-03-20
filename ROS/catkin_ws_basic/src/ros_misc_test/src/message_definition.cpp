/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2020年03月20日
* Copyright    :
* Descriptoin  : Learn how to define our own ROS messages and how to use them in
                               our own project
* References   :
======================================================================*/

#include <ros/ros.h>
#include <ros_misc_test/TestMsg.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_message_tutorial");
    ros::NodeHandle nh;

    ros_misc_test::TestMsg test_msg;
    test_msg.name = "zhang";

    ros::spin();
    return 0;
}