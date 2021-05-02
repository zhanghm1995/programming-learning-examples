/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2020年03月20日
* Copyright    :
* Descriptoin  : Collection of basic ROS operation
* References   :
======================================================================*/

// C++
#include <iostream>
// ROS
#include <ros/ros.h>
#include <ros/time.h>

/**
 * @brief Test the ros::Time usage
 * @ref http://wiki.ros.org/roscpp/Overview/Time
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

/**
 * @brief ros::Time的成员变量其实是两个uint32_t的整数sec和nsec
 * 凡是转换为秒,都会变为double浮点类型,存在精度损失问题,想要保证输入输出
 * 的时间戳保证完全一致,最好使用纳秒或者秒和纳秒单独赋值
 * 
 * 注意纳秒是uint64_t类型,单独的sec和nsec都是uint32_t类型,完整的秒是double类型
 */ 
void TestRosTime2() {
    std::cout<<"==========Begin TestRosTime2=================="<<std::endl;
    ros::Time time = ros::Time(1578365260, 24367000);
    double timestamp = time.toSec();
    ROS_INFO("Oringin seconds is %.9f  %.20f", timestamp, timestamp);
    
    /// 直接使用double类型的秒赋值,会和输入精确值有小许区别
    ros::Time header = ros::Time(timestamp);
    // ros::Time header = time; // using the orinal timestamp directly
    
    ROS_INFO("Input seconds is %u, nanoseconds is %u", header.sec, header.nsec);
    ROS_INFO("Input seconds is %.9f, nanoseconds is %u", header.toSec(), header.nsec);

    ros::Time header2;
    uint64_t nanosconds1 = time.toNSec();
    header2.fromNSec(nanosconds1);
    double seconds2 = header2.toSec();
    uint64_t nanosconds2 = header2.toNSec();
    ROS_INFO("Output seconds is %u, nanoseconds is %u",header2.sec, header2.nsec);
    std::cout<<"==========End TestRosTime2=================="<<std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_basic_tutorial");
    ros::NodeHandle nh;

    TestRosTime();
    TestRosTime2();
    
    ros::spin();
    return 0;
}