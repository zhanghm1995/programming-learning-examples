/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-08-03 18:18:52
 * @References: 
 * @Description:  学习OpenCV中如何对图像进行算术计算,叠加处理等图像处理
 */

#include <iostream>

#include <opencv2/opencv.hpp>

using std::cout;
using std::endl;

/**
 * @brief Learn use CV_Assert and learn how to convert the data type in cv::Mat
 */ 
void ProcessMat(const cv::Mat& coefs) {
    CV_Assert(coefs.channels() == 1);

    cv::Mat coefs_temp = coefs;
    coefs.convertTo(coefs_temp, CV_64FC1);
    
    for (int i = 0; i < coefs.rows; ++i) {
        cout<<coefs_temp.at<double>(i, 0)<<endl;
    }
}

// TODO: how to get the cv::Mat data type and access it by choosing correct method

int main() {
    cv::Mat src = cv::Mat::zeros(3, 1, CV_8UC1);
    ProcessMat(src);

    return 0;
}