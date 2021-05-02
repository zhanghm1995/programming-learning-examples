/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年11月16日
* Copyright    :
* Descriptoin  : Learn how to subsribe point cloud and publish it
* References   :
======================================================================*/
#include <sstream>
#include <algorithm>
#include <numeric>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl_conversions/pcl_conversions.h> //  for function pcl::fromROSMsg
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h> // for publish cloud directly

#include "point_type_define.h"

ros::Publisher g_pub_cloud;
uint32_t g_frame_cnt = 0;

void PublishCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *temp_cloud);

  temp_cloud->header.stamp = pcl_conversions::toPCL(msg->header.stamp);
  g_pub_cloud.publish(temp_cloud);
}

/**
 * @brief Get average of the container elements
 * @tparam Container Can be std::vector, std::deque
 * @param c
 * @return
 */
template<typename Container>
double Mean(const Container& c)
{
    return (std::accumulate(c.begin(), c.end(), 0.0) / c.size());
}

void ComputeFOV(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  std::vector<double> angle_vec;
  angle_vec.reserve(cloud->size());

  for (int i = 0; i < cloud->size(); ++i) {
    double pitch_angle = std::atan2(cloud->points[i].z, cloud->points[i].x);
    angle_vec.emplace_back(pitch_angle*180.0/ M_PI);
  }

  // Get limits values
  auto it_min_max = std::minmax_element(angle_vec.begin(), angle_vec.end(), [](const double obj1, double obj2) {
    return obj1 < obj2;
  });

  double mean_value = Mean(angle_vec);

  std::cout<<"min_element is "<<*it_min_max.first
           <<" max_element is "<<*it_min_max.second
           <<" mean value is "<<mean_value<<std::endl;
}

void SavePointCloud(const std::string& file_name, const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  // Convert to pcl cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_in(new pcl::PointCloud<pcl::PointXYZI>);
  sensor_msgs::PointCloud2Ptr cloudMsg(new sensor_msgs::PointCloud2(*cloud));
  cloudMsg->fields[3].name = "intensity";
  pcl::fromROSMsg(*cloudMsg, *pcl_in);
  
  /// Compute the Vertical FOV
  if (g_frame_cnt == 1) {
    ComputeFOV(pcl_in);
  }

  size_t num = pcl_in->size();

  // Begin save data
  FILE* stream = fopen(file_name.c_str(), "wb");
  if(stream == NULL) {
    std::cout<<"error open "<<file_name<<std::endl;
    return ;
  }
  float* data = (float*)malloc(4*num*sizeof(float));
  float* px = data + 0;
  float* py = data + 1;
  float* pz = data + 2;
  float* pI = data + 3;

  for(int i = 0; i < num; ++i) {
    *px = (float)pcl_in->points[i].x;
    *py = (float)pcl_in->points[i].y;
    *pz = (float)pcl_in->points[i].z;
    *pI = (float)pcl_in->points[i].intensity;
    px += 4;
    py += 4;
    pz += 4;
    pI += 4;
  }
  fwrite(data, sizeof(float), 4*num, stream);
  fclose(stream);
}

//! Save point cloud with ring field
void SavePointCloud2(const std::string& file_name, const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  // Convert to pcl cloud
  pcl::PointCloud<perception::PointXYZIR>::Ptr pcl_in(new pcl::PointCloud<perception::PointXYZIR>);
  pcl::fromROSMsg(*cloud, *pcl_in);
  pcl_in->header.frame_id = "/base_link";
  g_pub_cloud.publish(pcl_in);

  size_t num = pcl_in->size();

  // Begin save data
  FILE* stream = fopen(file_name.c_str(), "wb");
  if(stream == NULL) {
    std::cout<<"error open "<<file_name<<std::endl;
    return ;
  }
  float* data = (float*)malloc(5*num*sizeof(float));
  float* px = data + 0;
  float* py = data + 1;
  float* pz = data + 2;
  float* pI = data + 3;
  float* pring = data + 4;

  for(int i = 0; i < num; ++i) {
    *px = (float)pcl_in->points[i].x;
    *py = (float)pcl_in->points[i].y;
    *pz = (float)pcl_in->points[i].z;
    *pI = (float)pcl_in->points[i].intensity;
    *pring = (float)pcl_in->points[i].ring;
    px += 5;
    py += 5;
    pz += 5;
    pI += 5;
    pring += 5;
  }
  fwrite(data, sizeof(float), 5*num, stream);
  fclose(stream);
}

std::string GetSavePath(const uint32_t frame_cnt)
{
  // Set your saving path
  std::string folder_path = "/mnt/study/Datasets/kitti/";
  std::ostringstream ss;
  ss << std::setw(6) << std::setfill('0') << frame_cnt;

  const std::string file_name = folder_path + "/" + ss.str() + ".bin";
  return file_name;
}

void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ROS_INFO("Point number: %d", msg->height * msg->width);

  /// Get file name
  std::string file_name = GetSavePath(g_frame_cnt);
  
  ROS_INFO("Start saving %s", file_name.c_str());

  /// Saving
  SavePointCloud2(file_name, msg);

  ++g_frame_cnt;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_subsriber_publisher_node");
  ros::NodeHandle nh;

  ros::Subscriber sub_pointcloud = nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/point_cloud", 10, CloudCallback);
  
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/velody_cloud", 2);

  ros::spin();

  return 0;
}