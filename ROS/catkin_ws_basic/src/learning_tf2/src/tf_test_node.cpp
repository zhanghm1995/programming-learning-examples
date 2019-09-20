/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2018年12月31日
* Copyright    :
* Descriptoin  :
* References   :
======================================================================*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  geometry_msgs::PointStamped base_pos, world_pos;
  base_pos.header.frame_id = "base_link";
  base_pos.point.x = 10.0;
  base_pos.point.y = 16.0;
  base_pos.point.z = 0.1;

  while (node.ok()){
    try{
      //得到turtle1到turtle2的变换,即相当于知道turtle2要去的目标位置,从而引导turtle2跟随turtle1
      listener.transformPoint("world", base_pos, world_pos);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    printf("----x:%f-----y:%f----\n\n",world_pos.point.x,world_pos.point.y);
    rate.sleep();
  }
  return 0;
};

