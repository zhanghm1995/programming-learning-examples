/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年1月12日
* Copyright    :
* Descriptoin  :
* References   :
======================================================================*/

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace cv;
using namespace std;
class OGMProperty
{
public:
  OGMProperty();
  OGMProperty(float ogm_width, float ogm_height, float ogm_y_offset, float resolution);

  cv::Size mapSize() const;
  cv::Point mapAnchor() const;
  int mapOffset() const;//from bottom to up
  int mapCellNum() const;
  float ogmYForwardLim() const {return ogm_height_ - ogm_y_offset_; }
  float ogmYBackwardLim() const {return -ogm_y_offset_; }
  //米制单位
  float ogm_width_;
  float ogm_height_;
  float resolution_;//unit pixel equal to how many(resolution_) meter long
  float ogm_y_offset_;
};

//////////////////////////////// OGMProperty ////////////////////////////////
OGMProperty::OGMProperty():
    ogm_width_(0), ogm_height_(0), ogm_y_offset_(0), resolution_(0.0)
{

}
OGMProperty::OGMProperty(float ogm_width, float ogm_height, float ogm_y_offset,float resolution):
    ogm_width_(ogm_width),
    ogm_height_(ogm_height),
    ogm_y_offset_(ogm_y_offset),
    resolution_(resolution)
{

}

int OGMProperty::mapOffset() const
{
  return std::round(ogm_y_offset_ / resolution_);
}

cv::Size OGMProperty::mapSize() const
{
  return cv::Size(std::round(ogm_width_ / resolution_) + 1,
                  std::round(ogm_height_ / resolution_) + 1);
}

int OGMProperty::mapCellNum() const
{
  return mapSize().width * mapSize().height;
}

cv::Point OGMProperty::mapAnchor() const
{
  cv::Point res;
  res.x = mapSize().width/2;
  res.y = mapSize().height - 1 - mapOffset();
  return res;
}


inline bool fromVeloCoordsToMapCell(const OGMProperty& ogm_property, const float x, const float y,
          int & grid_x, int & grid_y)
{
  float xC = x + ogm_property.ogm_y_offset_;
  float yC = y + ogm_property.ogm_width_/2;
  // exclude outside roi points
  if(xC < 0 || xC >= ogm_property.ogm_height_ || yC < 0 || yC >=ogm_property.ogm_width_)
    return false;

  int map_width = ogm_property.mapSize().width;
  int map_height = ogm_property.mapSize().height;
  grid_x  = map_width - 1 - floor(yC/ogm_property.resolution_);
  grid_y  = map_height - 1 - floor(xC/ogm_property.resolution_);
  return true;
}

inline void fromMapCellToVeloCoords(const OGMProperty& ogm_property, const int grid_x,const int grid_y,
         float & x, float & y)
{
  const int& map_width = ogm_property.mapSize().width;
  const int& map_height = ogm_property.mapSize().height;
  x = (map_height - 1 - grid_y)*ogm_property.resolution_ - ogm_property.ogm_y_offset_;
  y = (map_width - 1 - grid_x)*ogm_property.resolution_ - ogm_property.ogm_width_/2;
}

int main()
{
  // 1) Read pcd file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//初始化点云

  pcl::io::loadPCDFile<pcl::PointXYZ>("0_0.pcd", *cloud);//加载pcd点云并放入cloud中

  // Define ogm
  OGMProperty ogm_property(80,100,50,0.2);
  Mat image(ogm_property.mapSize().height, ogm_property.mapSize().width, CV_8UC3, Scalar(0));

  // 2) Put lidar points to a vector
  vector<cv::Point> pointVec;
  for(size_t i = 0; i < cloud->size(); ++i){
    pcl::PointXYZ& point = cloud->points[i];
    int x,y;
    if(!fromVeloCoordsToMapCell(ogm_property,point.x, point.y, x, y))
      continue;
    pointVec.push_back(cv::Point(x, y));
    image.at<cv::Vec3b>(y, x) = cv::Vec3b(255,255,255);//BGR color
  }

  // 3) minAreaRect
  RotatedRect rectInfo = minAreaRect(pointVec);

  // 4) Print info
  printf("rect info width: %.3f, height: %.3f, center: (%.3f, %.3f)\n", rectInfo.size.width,
      rectInfo.size.height, rectInfo.center.x, rectInfo.center.y);

  // 5) Draw rect
  Point2f vertices[4];
  rectInfo.points(vertices);//获取矩形的四个点
  for (int i = 0; i < 4; i++)
    line(image, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));

  namedWindow("rectangles", CV_WINDOW_NORMAL);
  imshow("rectangles", image);
  waitKey(0);
}

