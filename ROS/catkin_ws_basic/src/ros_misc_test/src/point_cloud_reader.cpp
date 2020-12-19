#include <sstream>
#include <algorithm>
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

inline bool ReadVeloPoints(const std::string& velo_bin_path,
                           pcl::PointCloud<pcl::PointXYZI>& point_cloud) {
    std::fstream input(velo_bin_path.c_str(), std::ios::in | std::ios::binary);
    if (!input.good()) {
        std::cout<<"[ReadVeloPoints] Could not read file: "<<velo_bin_path<<std::endl;
        return false;
    } else {
        input.seekg(0, std::ios::beg);
        int i;
        for (i = 0; input.good() && !input.eof(); i++) {
            pcl::PointXYZI point;
            input.read((char*)&point.x, 3 * sizeof(float));
            input.read((char*)&point.intensity, sizeof(float));
            point_cloud.push_back(point);
        }
        input.close();
        return true;
    }
}

//! Read *.bin format point cloud with ring field
inline bool ReadVeloPoints2(const std::string& velo_bin_path,
                                                             pcl::PointCloud<perception::PointXYZIR>& point_cloud) {
    std::fstream input(velo_bin_path.c_str(), std::ios::in | std::ios::binary);
    if (!input.good()) {
        std::cout<<"[ReadVeloPoints] Could not read file: "<<velo_bin_path<<std::endl;
        return false;
    } else {
        input.seekg(0, std::ios::beg);
        int i;
        for (i = 0; input.good() && !input.eof(); i++) {
            perception::PointXYZIR point;
            input.read((char*)&point.x, 3 * sizeof(float));
            input.read((char*)&point.intensity, sizeof(float));
            float ring_temp;
            input.read((char*)&ring_temp, sizeof(float));
            // printf("%.3f\n", ring_temp);
            point.ring = static_cast<uint16_t>(ring_temp);
            point_cloud.push_back(point);
        }
        input.close();
        return true;
    }
}

void PublishCloud(const std::string& velo_dir_path, const int frame_cnt)
{
  std::ostringstream ss;
  ss << std::setw(6) << std::setfill('0') << frame_cnt;
  std::string velodyne_full_path = velo_dir_path + "/" + ss.str() + ".bin";
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  ReadVeloPoints(velodyne_full_path, *temp_cloud);
  temp_cloud->header.frame_id = "/base_link";
  g_pub_cloud.publish(temp_cloud);
}

//! With ring field
bool PublishCloud2(const std::string& velo_dir_path, const int frame_cnt)
{
  std::ostringstream ss;
  ss << std::setw(6) << std::setfill('0') << frame_cnt;
  std::string velodyne_full_path = velo_dir_path + "/" + ss.str() + ".bin";
  pcl::PointCloud<perception::PointXYZIR>::Ptr temp_cloud(new pcl::PointCloud<perception::PointXYZIR>);

  if (ReadVeloPoints2(velodyne_full_path, *temp_cloud)) {
    temp_cloud->header.frame_id = "/base_link";
    g_pub_cloud.publish(temp_cloud);
  } else {
      return false;
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_reader_node");
  ros::NodeHandle nh;
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/velody_cloud", 2);

  int frame_cnt = 0;
  // std::string velo_dir_path("/mnt/study/Datasets/point_cloud");
  std::string velo_dir_path("/mnt/study/Datasets/point_cloud_with_ring");

  ros::Rate rate(10);

  while (frame_cnt < 200 && ros::ok()) {
      printf("Begin publishing %d\n", frame_cnt);
    if (!PublishCloud2(velo_dir_path, frame_cnt)) {
        printf("Couldn't read file successfully, exiting...\n");
        ros::shutdown();
        break;
    }
    ++frame_cnt;
    rate.sleep();
  }

  ros::spin();

  return 0;
}