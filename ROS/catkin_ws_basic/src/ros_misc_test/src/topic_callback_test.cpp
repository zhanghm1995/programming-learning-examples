/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年12月24日
* Copyright    :
* Descriptoin  : Learn different callback functions definition in ROS
* References   : https://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
======================================================================*/

#include <iostream>
#include <thread>

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

/**
 * @brief 如何使用在类中开启新的线程处理任务
 * 
 */
class ObjectProcess {
public:
  void Process()
  {
    process_thread_ =   std::thread(&ObjectProcess::ProcessThread, this);
  }

  static void ProcessThread(void *const args)
  {
    auto * const _impl = reinterpret_cast<ObjectProcess *>(args);
    while (_impl->workable) {
      // Do something

    }
  }

  ~ObjectProcess()
  {
    workable = false;
    process_thread_.join();
  }

private:
  bool workable = true;
  std::thread process_thread_;
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