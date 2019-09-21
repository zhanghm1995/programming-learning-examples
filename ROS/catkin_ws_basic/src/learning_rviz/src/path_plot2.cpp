/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年4月11日
* Copyright    :
* Descriptoin  : Learning how to show path or trajectory in Rviz
*                by subscribe the tf topic
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
  ros::Rate r(100);
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("trajectory", 1);

  // TF listener
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  nav_msgs::Path path;
  while (ros::ok())
  {
    geometry_msgs::TransformStamped transformStamped;
    try{
      if (!tf_buffer.canTransform("world", "base_link",
          ros::Time(0), ros::Duration(1.0))) {
        ROS_WARN("Can't transform");
        continue;
      }
      else {
        ROS_WARN("Doing transform");
      }

      transformStamped = tf_buffer.lookupTransform("world", "base_link",
          ros::Time(0), ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = transformStamped.transform.translation.x;
    this_pose_stamped.pose.position.y = transformStamped.transform.translation.y;
    this_pose_stamped.pose.position.z = transformStamped.transform.translation.z;

//    geometry_msgs::Quaternion goal_quat = transformStamped.transform.rotation;
    this_pose_stamped.pose.orientation.x = 0;
    this_pose_stamped.pose.orientation.y = 0;
    this_pose_stamped.pose.orientation.z = 0;
    this_pose_stamped.pose.orientation.w = 1;

    this_pose_stamped.header.stamp = transformStamped.header.stamp;
    this_pose_stamped.header.frame_id = "base_link";

    // Publish nav_msgs
    path.header.stamp = transformStamped.header.stamp;
    path.header.frame_id = "world";
    path.poses.push_back(this_pose_stamped);
//    if (path.poses.size() >= 100) {
////      ROS_WARN("FFFFF");
//      path.poses.erase(path.poses.begin());
//    }

    path_pub.publish(path);
    r.sleep();
  }
}
