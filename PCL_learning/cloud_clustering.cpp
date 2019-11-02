#include <algorithm>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

//Input: 点云
//Output: 选出最可靠聚类,输出该聚类离原点平均距离
float CalculateAverageDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  float totalDistance = 0.0;
  for(int i = 0; i < cloud->size(); ++i) {
    float x = cloud->points[i].x;
    float y = cloud->points[i].y;
    float z = cloud->points[i].z;
    totalDistance += sqrt(x*x + y*y + z*z);
  }
  return totalDistance/cloud->size();
}

float ExtractOptCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_in); //创建点云索引向量，用于存储实际的点云信息

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); //设置近邻搜索的搜索半径为2cm
  ec.setMinClusterSize (10);//设置一个聚类需要的最少点数目为100
  ec.setMaxClusterSize (200);//设置一个聚类需要的最大点数目为25000
  ec.setSearchMethod (tree);//设置点云的搜索机制
  ec.setInputCloud (cloud_in);
  ec.extract (cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中

  /*为了从点云索引向量中分割出每个聚类，必须迭代访问点云索引，每次创建一个新的点云数据集，并且将所有当前聚类的点写入到点云数据集中。*/
  //迭代访问点云索引cluster_indices，直到分割出所有聚类
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters_vec;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    //创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_in->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    cloud_clusters_vec.push_back(cloud_cluster);
  }

  //选出离原点最近的聚类
  auto it_min_dis = std::min_element(cloud_clusters_vec.begin(), cloud_clusters_vec.end(), [](const pcl::PointCloud<pcl::PointXYZ>::Ptr lhs, const pcl::PointCloud<pcl::PointXYZ>::Ptr rhs){
    return CalculateAverageDistance(lhs) < CalculateAverageDistance(rhs);
  });
  if(it_min_dis != cloud_clusters_vec.end()) {
    return CalculateAverageDistance(*it_min_dis);
  }
  else
    return -1;
}

int
main (int argc, char** argv)
{
  //读入点云数据table_scene_lms400.pcd
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read ("table_scene_lms400.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*
  /*从输入的.PCD文件载入数据后，我们创建了一个VoxelGrid滤波器对数据进行下采样，我们在这里进行下采样的原因是来加速处理过程，越少的点意味着分割循环中处理起来越快。*/
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg; //体素栅格下采样对象
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f); //设置采样的体素大小
  vg.filter (*cloud_filtered);  //执行采样保存数据
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;//创建分割对象
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);  //设置对估计的模型参数进行优化处理
  seg.setModelType (pcl::SACMODEL_PLANE);//设置分割模型类别
  seg.setMethodType (pcl::SAC_RANSAC);//设置用哪个随机参数估计方法
  seg.setMaxIterations (100);  //设置最大迭代次数
  seg.setDistanceThreshold (0.02);    //设置判断是否为模型内点的距离阈值

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    /*为了处理点云中包含多个模型，我们在一个循环中执行该过程，并在每次模型被提取后，我们保存剩余的点，进行迭代。模型内点通过分割过程获取，如下*/
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    //移去平面局内点，提取剩余点云
    pcl::ExtractIndices<pcl::PointXYZ> extract;   //创建点云提取对象
    extract.setInputCloud (cloud_filtered);    //设置输入点云
    extract.setIndices (inliers);   //设置分割后的内点为需要提取的点集
    extract.setNegative (false); //设置提取内点而非外点
    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);   //提取输出存储到cloud_plane
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered); //创建点云索引向量，用于存储实际的点云信息

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); //设置近邻搜索的搜索半径为2cm
  ec.setMinClusterSize (100);//设置一个聚类需要的最少点数目为100
  ec.setMaxClusterSize (25000);//设置一个聚类需要的最大点数目为25000
  ec.setSearchMethod (tree);//设置点云的搜索机制
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中

  /*为了从点云索引向量中分割出每个聚类，必须迭代访问点云索引，每次创建一个新的点云数据集，并且将所有当前聚类的点写入到点云数据集中。*/
  //迭代访问点云索引cluster_indices，直到分割出所有聚类
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    //创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

  return (0);
}
