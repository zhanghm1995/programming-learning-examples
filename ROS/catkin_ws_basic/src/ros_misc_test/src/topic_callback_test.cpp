/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年12月24日
* Copyright    :
* Descriptoin  :
* References   :
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

  ros::Subscriber sub = nh.subscribe("/Int32_msg", 2, ex.MsgCallback, &ex);

  ros::spin();

  return 1;
}