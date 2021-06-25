/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年6月18日
* Copyright    :
* Descriptoin  :
* References   :
======================================================================*/

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  cv::Mat img_src(501, 401, CV_8UC1, cv::Scalar(0));
  cv::rectangle(img_src, cv::Point(100,100), cv::Point(300,400), cv::Scalar(255), -1);

  cv::Mat mask1(501, 401, CV_8UC1, cv::Scalar(255));
  cv::Point vertexPoints[4];
  vertexPoints[0] = Point(200,180);
  vertexPoints[1] = Point(250,190);
  vertexPoints[2] = Point(280,250);
  vertexPoints[3] = Point(190,260);
  const cv::Point* ppt = vertexPoints;
  int point_count = 4;
  cv::fillPoly(mask1,&ppt,&point_count,1,cv::Scalar(0));//目标矩形框
  Mat res1;
  bitwise_and(img_src,img_src,res1,mask1);
  imshow("img_src", img_src);
  imshow("img_mask1", mask1);
  imshow("img_res1", res1);
  waitKey(0);
}

