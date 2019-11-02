/*
 * registration.cpp
 *
 *  Created on: 2018年5月21日
 *      Author: zhanghm
 */
#include <iostream>                 //标准输入输出头文件
#include <pcl/io/pcd_io.h>         //I/O操作头文件
#include <pcl/point_types.h>        //点类型定义头文件
#include <pcl/registration/icp.h>   //ICP配准类相关头文件

int
 main (int argc, char** argv)
{
  //创建两个pcl::PointCloud<pcl::PointXYZ>共享指针，并初始化它们
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  // 随机填充点云
  cloud_in->width    = 5;               //设置点云宽度
  cloud_in->height   = 1;               //设置点云为无序点
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  std::cout << "Saved " << cloud_in->points.size () << " data points to input:"//打印处点云总数
      << std::endl;

  for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<    //打印处实际坐标
      cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
      cloud_in->points[i].z << std::endl;
  *cloud_out = *cloud_in;
  std::cout << "size:" << cloud_out->points.size() << std::endl;

   //实现一个简单的点云刚体变换，以构造目标点云
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
    cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
  std::cout << "Transformed " << cloud_in->points.size () << " data points:"
      << std::endl;
  for (size_t i = 0; i < cloud_out->points.size (); ++i)     //打印构造出来的目标点云
    std::cout << "    " << cloud_out->points[i].x << " " <<
      cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;   //创建IterativeClosestPoint的对象
  icp.setInputCloud(cloud_in);                 //cloud_in设置为点云的源点
  icp.setInputTarget(cloud_out);               //cloud_out设置为与cloud_in对应的匹配目标
  pcl::PointCloud<pcl::PointXYZ> Final;         //存储经过配准变换点云后的点云
  icp.align(Final);                             //打印配准相关输入信息
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

 return (0);
}



