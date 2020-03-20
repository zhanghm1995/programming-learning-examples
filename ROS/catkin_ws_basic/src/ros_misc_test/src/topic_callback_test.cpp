/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年12月24日
* Copyright    :
* Descriptoin  : Learn different callback functions definition in ROS
* References   : https://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
======================================================================*/

#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Int32.h>

class Singleton {
public:
  static Singleton* Instance() {
    static Singleton instance;
    return &instance;
  }

  void MsgCallback(const std_msgs::Int32::ConstPtr msg) {
    std::cout<<msg->data<<std::endl;
  }
  
private:
  Singleton() = default;
  Singleton(const Singleton&) = delete;
};

class Example {
public:
  void MsgCallback(const std_msgs::Int32::ConstPtr msg) {
    std::cout<<msg->data<<std::endl;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "topic_callback_test");
  ros::NodeHandle nh;
  
  Singleton* instance = Singleton::Instance();

  Example ex;

  ros::Subscriber sub = nh.subscribe("/Int32_msg", 2, &Example::MsgCallback, &ex);

  ros::Subscriber sub2 = nh.subscribe("/Int32_msg2", 2, &Singleton::MsgCallback, instance);
  ros::spin();

  return 1;
}