// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
// PCL
#include <pcl/point_cloud.h>
// Projects
#include "point_type_define.h"

ros::Publisher g_pub_cloud;

using namespace noah::perception;
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

struct EIGEN_ALIGN16 PointDD {
    union  { \
    float data[4]; \
    struct { \
      float x; \
      float y; \
      float z; \
    }; \
    };

    uint32_t ring;
};

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
        cloud->points[i].intensity = 50;
        cloud->points[i].ring = 100;
    }

    pcl::PointCloud<PointXYZR>::Ptr cloud_xyz(new pcl::PointCloud<PointXYZR>);
    pcl::copyPointCloud(*cloud, *cloud_xyz);


    PointDD pp;
    pp.x = 10;
    pp.y = 20;
    cout<<pp.data[0]<<endl;
    cout<<pp.x<<endl;

    PointXYZR my_point;
//    pcl::PointXYZHSV my_point;
    my_point.x = my_point.y = my_point.z = 10;
    cout<<"my_point "<<my_point<<endl;

    ros::Rate rate(10);
    while (ros::ok()) {
        ROS_INFO("--------");
        cout<<"PointDD "<<sizeof(PointDD)<<endl;
        cout<<sizeof(float)<<" "<<sizeof(int)<<endl;
        cout<<sizeof(PointXYZIR)<<endl;
        cout<<sizeof(PointXYZR)<<endl;
        cout<<"PointXYZ "<< sizeof(pcl::PointXYZ)<<endl;
        cout<<"PointXYZI "<< sizeof(pcl::PointXYZI)<<endl;
        ros::spinOnce();
        PublishCloud<PointXYZR>(cloud_xyz);
        rate.sleep();
    }

    return 0;
}