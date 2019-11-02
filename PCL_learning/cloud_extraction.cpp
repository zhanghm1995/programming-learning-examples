/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2018年10月29日
* Copyright    :
* Descriptoin  :
* References   :
======================================================================*/

#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
using std::cout; using std::endl;

void test()
{
  cout<<"rand number "<<rand()/ (RAND_MAX + 1.0f)<<endl;
}
int main()
{
  test();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  std::cerr << "Cloud before extract: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
    << cloud->points[i].y << " "
    << cloud->points[i].z << std::endl;

  std::vector<int> index = {1,3,4};
  boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(index);
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // Extract the inliers
  extract.setInputCloud (cloud);
  extract.setIndices (index_ptr);
  extract.setNegative (false);
  extract.filter (*cloud_p);

  cout<<"----------------"<<endl;
  for (size_t i = 0; i < cloud_p->points.size (); ++i)
      std::cerr << "    " << cloud_p->points[i].x << " "
      << cloud_p->points[i].y << " "
      << cloud_p->points[i].z << std::endl;
}

