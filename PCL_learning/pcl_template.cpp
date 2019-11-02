/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年1月13日
* Copyright    :
* Descriptoin  :
* References   :
======================================================================*/

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/bilateral.h>
#include <boost/preprocessor/cat.hpp>
using namespace std;
template<typename PointT>
void projectCloud2Image(typename pcl::PointCloud<PointT>::Ptr cloud_p)
{
  cout<<"cloud size is "<<cloud_p->size()<<endl;
}

template <typename T>
class Motion
{
public:
  Motion();
};


template <typename T>
Motion<T>::Motion()
{

}
template class  Motion<int>;

int main()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  projectCloud2Image<pcl::PointXYZ>(cloud);
  Motion<string> ff;

  pcl::BilateralFilter<pcl::PointXYZI> fff;

  return 0;
}
