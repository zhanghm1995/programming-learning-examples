// Copyright (C) 2018  Haiming Zhang<zhanghm_1995@qq.com>,
// Intelligent Vehicle Research Center, Beijing Institute of Technology

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.


// C++
#include <iostream>
#include <string>
#include <vector>
#include <cstdio>
#include <sstream>
#include <iomanip>
// ROS

#include <opencv2/opencv.hpp>

#include "ImageRectify.h"

using namespace std;
using namespace cv;


int main(int argc, char **argv){
  ImageRectify image_left_rectify("/home/zhanghm/Datasets/SAIC/calibration/offline_0.5.yaml", "LEFT");
  ImageRectify image_right_rectify("/home/zhanghm/Datasets/SAIC/calibration/offline_0.5.yaml", "RIGHT");

  string image_left_path = "/home/zhanghm/Datasets/SAIC/1212_1300/image_02/data/";
  string image_right_path = "/home/zhanghm/Datasets/SAIC/1212_1300/image_03/data/";
  string image_left_save_path = "/home/zhanghm/1212_1320/image_02/";
  string image_right_save_path = "/home/zhanghm/1212_1320/image_03/";

  for(int i = 0; i < 1346; ++i){
    stringstream image_left_ss, image_right_ss,
                 image_left_save_ss, image_right_save_ss;

    // Read left image
    image_left_ss<<image_left_path<<std::setfill('0')<<std::setw(10)<<i<<".png";
    cv::Mat img_left_src = cv::imread(image_left_ss.str());

    // Read right image
    image_right_ss<<image_right_path<<std::setfill('0')<<std::setw(10)<<i<<".png";
    cv::Mat img_right_src = cv::imread(image_right_ss.str());

    // Rectify
    cv::Mat img_dst;
    image_left_rectify.doRectify(img_left_src, img_dst);
    cv::namedWindow("img_left_dst", CV_WINDOW_NORMAL);
    cv::imshow("img_left_dst", img_dst);
    cv::waitKey(5);

    image_left_save_ss<<image_left_save_path<<std::setfill('0')<<std::setw(10)<<i<<".png";
    cv::imwrite(image_left_save_ss.str(), img_dst);

    image_right_rectify.doRectify(img_right_src, img_dst);
    image_right_save_ss<<image_right_save_path<<std::setfill('0')<<std::setw(10)<<i<<".png";
    cv::imwrite(image_right_save_ss.str(), img_dst);
  }
  return 0;
}

