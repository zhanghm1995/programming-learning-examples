/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :
* Copyright    :
* Descriptoin  : 订阅坐标系tf消息,使第二个turtle能够跟随第一个运动
* References   :
======================================================================*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <iostream>
int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle =
    node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);

  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      //得到turtle1到turtle2的变换,即相当于知道turtle2要去的目标位置,从而引导turtle2跟随turtle1
      listener.lookupTransform("/turtle2", "/turtle1",ros::Time(0), transform);

      //another method
//      ros::Time now = ros::Time::now();
//      listener.waitForTransform("turtle2","turtle1",now,ros::Duration(3.0));
//      listener.lookupTransform("/turtle2", "/turtle1",
//                                     now, transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
//    std::cout<<"==================="<<std::endl;
    printf("----x:%f-----y:%f----\n\n",transform.getOrigin().x(),transform.getOrigin().y());
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};

