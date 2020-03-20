/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2020年03月20日
* Copyright    :
* Descriptoin  :
* References   :
======================================================================*/

// C++
#include <iostream>
// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>

/**
 * @brief Test the ros::Time usage
 */ 
void TestRosTime() {
    ros::Time header1 = ros::Time::now();

    double seconds1 = header1.toSec();
    uint64_t nanoseconds1 = header1.toNSec(); // Note: must using uint64_t in case the precision loss
    printf("%.9f %ld\n", seconds1, nanoseconds1);

    std::cout<<"====================="<<std::endl;
    ros::Time header2;
    // header2.fromSec(seconds1);
    header2.fromNSec(nanoseconds1);
    double seconds2 = header1.toSec();
    uint64_t nanoseconds2 = header1.toNSec();
    printf("%.9f %ld\n", seconds2, nanoseconds2);

    std::cout<<"====================="<<std::endl;
    ros::Time header3 = header1;
    double seconds3 = header1.toSec();
    uint64_t nanoseconds3 = header1.toNSec();
    printf("%.9f %ld\n", seconds3, nanoseconds3);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_basic_tutorial");
    ros::NodeHandle nh;

    TestRosTime();
    ros::spin();
    return 0;
}