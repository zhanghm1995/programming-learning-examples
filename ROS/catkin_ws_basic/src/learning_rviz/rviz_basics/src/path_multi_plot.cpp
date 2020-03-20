/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年4月11日
* Copyright    :
* Descriptoin  : Learning how to show multiple paths or trajectory in Rviz
*                by subscribe the tf topic
* References   :
======================================================================*/

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


void publishMultiPath(ros::Publisher& path_pub, const std::vector<visualization_msgs::Marker> marker_vec)
{
  for (size_t i = 0; i < marker_vec.size(); ++i) {
    path_pub.publish(marker_vec[i]);
  }
}
int main( int argc, char** argv )
{
  ros::init(argc, argv, "path_plot_node");
  ros::NodeHandle n;
  ros::Rate r(10);
  ros::Publisher path_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_msgs/MarkerArray", 1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_msgs/Marker", 1);

  // TF listener
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  visualization_msgs::Marker path1, path2;
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
    geometry_msgs::Point this_pose_stamped;
    this_pose_stamped.x = transformStamped.transform.translation.x;
    this_pose_stamped.y = transformStamped.transform.translation.y;
    this_pose_stamped.z = transformStamped.transform.translation.z;

    // Publish


    path1.id = 1;
    path2.id = 2;
    path1.ns =  path2.ns = "path1";
//    path2.ns = "path2";
    path1.scale.x = path2.scale.x = 0.5;
    path1.color.g = path2.color.g = 1.0f;
    path1.color.a = path2.color.a = 1.0;
    path1.points.push_back(this_pose_stamped);

    this_pose_stamped.x += 10.0;
    path2.points.push_back(this_pose_stamped);

    path1.header.stamp = path2.header.stamp = transformStamped.header.stamp;
    path1.header.frame_id =  path2.header.frame_id = "world";
//    path1.pose.orientation.w = path2.pose.orientation.w = 1.0;
    path1.action =  path2.action = visualization_msgs::Marker::ADD;
    path1.type = path2.type = visualization_msgs::Marker::LINE_STRIP;
    path1.lifetime = path2.lifetime = ros::Duration();


    visualization_msgs::MarkerArray path_array;
    path_array.markers.push_back(path1);
    path_array.markers.push_back(path2);

    path_pub.publish(path_array);
//    publishMultiPath(marker_pub, path_array.markers);
    r.sleep();
  }
}
