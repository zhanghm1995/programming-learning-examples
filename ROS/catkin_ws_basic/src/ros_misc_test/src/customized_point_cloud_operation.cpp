// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
// PCL
#include <pcl/point_cloud.h>
// Projects
#include "point_type_define.h"

ros::Publisher g_pub_cloud;

using namespace perception;
using std::cout;
using std::endl;

template <typename PointType>
void PublishCloud(const typename pcl::PointCloud<PointType>::Ptr msg)
{
//    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::fromROSMsg(*msg, *temp_cloud);
//
//    temp_cloud->header.stamp = pcl_conversions::toPCL(msg->header.stamp);
    g_pub_cloud.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_publisher_node");
    ros::NodeHandle nh;
    g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/velody_cloud", 2);

    pcl::PointCloud<PointXYZIR>::Ptr cloud(new pcl::PointCloud<PointXYZIR>);
    cloud->header.frame_id = "base_link";
    cloud->width = 100;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size (); ++i) {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].intensity = i;
        cloud->points[i].ring = 100 - i;
    }

    // pcl::PointCloud<PointXYZR>::Ptr cloud_xyz(new pcl::PointCloud<PointXYZR>);
    // pcl::copyPointCloud(*cloud, *cloud_xyz);

    ros::Rate rate(10);
    while (ros::ok()) {
        ROS_INFO("--------");
        ros::spinOnce();
        PublishCloud<PointXYZIR>(cloud);
        rate.sleep();
    }

    return 0;
}

#ifdef false
// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
// PCL
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h> //  for function pcl::fromROSMsg
// Projects
#include "point_type_define.h"

using namespace noah::perception;
using std::cout;
using std::endl;

void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    ROS_WARN("Cloud call back");
    pcl::PointCloud<PointXYZR16u> cloud;
    pcl::fromROSMsg(*msg, cloud);
//    cout<<cloud.points[0].ring<<" "<<cloud.points[0].intensity<<endl;
    cout<<cloud.points[0].ring<<endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_subsriber_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_pointcloud = nh.subscribe<sensor_msgs::PointCloud2>("/velody_cloud", 10, CloudCallback);

    ros::spin();

    return 0;
}
#endif