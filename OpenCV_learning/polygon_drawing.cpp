/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年6月19日
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
  vertexPoints[1] = Point(250,180);
  vertexPoints[2] = Point(250,250);
  vertexPoints[3] = Point(200,250);

  double delta_x = 0;
  double delta_y = 100;

  cv::Point vertexPoints2[4];
  vertexPoints2[0] = Point(vertexPoints[0].x + delta_x, vertexPoints[0].y + delta_y);
  vertexPoints2[1] = Point(vertexPoints[1].x + delta_x, vertexPoints[1].y + delta_y);
  vertexPoints2[2] = Point(vertexPoints[2].x + delta_x, vertexPoints[2].y + delta_y);
  vertexPoints2[3] = Point(vertexPoints[3].x + delta_x, vertexPoints[3].y + delta_y);
  const cv::Point* ppt = vertexPoints;
  int point_count = 4;
  cv::fillPoly(mask1,&ppt,&point_count,1,cv::Scalar(0));//目标矩形框

  const cv::Point* ppt2 = vertexPoints2;
  cv::fillPoly(mask1,&ppt2,&point_count,1,cv::Scalar(0));//目标矩形框

  cv::Point vertexPoints3[8];
  for (int i = 0; i < 4; ++i) {
    vertexPoints3[i] = vertexPoints[i];
  }
  for (int i = 0; i < 4; ++i) {
    vertexPoints3[i + 4] = vertexPoints2[i];
  }
  cv::Mat mask2(501, 401, CV_8UC1, cv::Scalar(0));
  const cv::Point* ppt3 = vertexPoints3;
  int point_count3 = 8;
  cv::fillConvexPoly(mask2, ppt3,point_count3,cv::Scalar(255));//目标矩形框

  cv::circle(mask2, Point(100,100),50, Scalar(255), -1);
//  Mat res1;
//  bitwise_and(img_src,img_src,res1,mask1);
  imshow("img_src", img_src);
  imshow("img_mask1", mask1);
  imshow("img_mask2", mask2);
//  imshow("img_res1", res1);
  waitKey(0);
}

