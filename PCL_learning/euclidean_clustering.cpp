/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年1月13日
* Copyright    :
* Descriptoin  :
* References   :
======================================================================*/

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>


static int64_t gtm() {
    struct timeval tm;
    gettimeofday(&tm, 0);
    // return ms
    int64_t re = (((int64_t)tm.tv_sec) * 1000 * 1000 + tm.tv_usec);
    return re;
}

using namespace std;

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//初始化点云

    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud);//加载pcd点云并放入cloud中

    cout<<"点云数量："<<cloud->points.size () <<endl;
    int64_t tm0 = gtm();

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;   //欧式聚类对象
    ec.setClusterTolerance (0.2);                     // 设置近邻搜索的搜索半径为65cm
    ec.setMinClusterSize (30);                 //设置一个聚类需要的最少的点数目为8
    ec.setMaxClusterSize (25000);               //设置一个聚类需要的最大点数目为25000
    ec.setSearchMethod (tree);                    //设置点云的搜索机制
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中
    //迭代访问点云索引cluster_indices,直到分割处所有聚类
    int64_t tm1 = gtm();
    printf("[INFO]update cast time:%ld us\n",  tm1-tm0);

    int j = 0;
    int num=0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Colorcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //迭代容器中的点云的索引，并且分开保存索引的点云
    for (auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
        pcl::PointXYZRGB p;
        p.x = cloud->points[*pit].x;
        p.y = cloud->points[*pit].y;
        p.z = cloud->points[*pit].z;
        srand(j+1);
        int c=rand()%80+160;
        int k=rand()%96+160;
        int g=rand()%256;

        p.r=c+j*10;
        p.g=k-j*40;
        p.b=g;
        Colorcloud->points.push_back(p);
        num++;
      }
      j++;
    }
    cout<<"簇的数量"<<j<<endl;
    Colorcloud->height = 1;
    Colorcloud->width = num;
    cout<<"Finalpointcloud data has "<<num<<" points"<<endl;
    Colorcloud->is_dense = false;//最终优化结果
    pcl::io::savePCDFileASCII<pcl::PointXYZRGB> ("test_simple.pcd", *Colorcloud);
   return 0;
}




