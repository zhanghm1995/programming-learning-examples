/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年4月11日
* Copyright    :
* Descriptoin  : Learning how to show path or trajectory in Rviz
* References   :
======================================================================*/

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
int main( int argc, char** argv )
{
  ros::init(argc, argv, "path_plot_node");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("trajectory", 1, true);

  // TF listener
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "odom";

  double x = 0.0, y = 0.0, th = 0.0, vx = 0.1, vy = -0.1, vth = 0.1;
  ros::Time current_time(ros::Time::now()), last_time(ros::Time::now());
  while (ros::ok())
  {
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = x;
    this_pose_stamped.pose.position.y = y;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "odom";
    path.poses.push_back(this_pose_stamped);

    path_pub.publish(path);
    last_time = current_time;
    r.sleep();
  }
}
