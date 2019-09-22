/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年5月29日
* Copyright    :
* Descriptoin  :Learn how to use ROS parameters server
* References   :
======================================================================*/
#include <string>
#include <map>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
using namespace std;

#define AINFO(args, ...) ROS_INFO("Hello " args, ##__VA_ARGS__)
#define AWARN(...) ROS_WARN(__VA_ARGS__)

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_param_test");
  ros::NodeHandle nh;

  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
     ros::console::notifyLoggerLevelsChanged();
  }


  AINFO("Set this one %d", 26);
  AINFO("Set this one");

  AWARN("Set this one %d", 26);
  AWARN("Set this one");

  ROS_DEBUG("HELLO %s", "World");
  ROS_INFO_NAMED("test_only", "Hello World");
  ROS_FATAL("HELLO WORLD");
  ROS_INFO_NAMED("test_only", "Hello Set this one");
  ros::spin();
}
