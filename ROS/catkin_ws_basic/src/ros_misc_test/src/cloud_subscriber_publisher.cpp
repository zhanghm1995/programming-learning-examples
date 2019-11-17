/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年11月16日
* Copyright    :
* Descriptoin  : Learn how to subsribe point cloud and publish it
* References   :
======================================================================*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl_conversions/pcl_conversions.h> //  for function pcl::fromROSMsg
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h> // for publish cloud directly

ros::Publisher g_pub_cloud;

void PublishCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *temp_cloud);

  temp_cloud->header.stamp = pcl_conversions::toPCL(msg->header.stamp);
  g_pub_cloud.publish(temp_cloud);
}

void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *temp_cloud);
    ROS_INFO("CloudCallback size is %d", temp_cloud->size());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_subsriber_publisher_node");
  ros::NodeHandle nh;
  ros::Subscriber sub_pointcloud = nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointcloud", 10, CloudCallback);
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/velody_cloud", 2);

  ros::spin();

  return 0;
}