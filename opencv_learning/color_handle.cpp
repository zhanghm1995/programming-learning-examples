/**
 * Learn how to use color related handling function in OpenCV
 * 
 * References: https://docs.opencv.org/3.1.0/df/d9d/tutorial_py_colorspaces.html
 *  
 */

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;

/**
 * @brief Convert HSV color value to BGR value
 * H-Hue, S-Saturation, V-Value. And the Hue in OpenCV is 0~180
 * Using HSV can generate specific color you want
 * @param unique_id[in] Hue value
 */
cv::Scalar HSV2BGR(int unique_id)
{
    int hue_value = unique_id  % 180;
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue_value, 255, 255));
    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, CV_HSV2BGR);

    // Return BGR color value
    cv::Scalar color((int)bgr.at<cv::Vec3b>(0,0)[0],
                    (int)bgr.at<cv::Vec3b>(0,0)[1],
                    (int)bgr.at<cv::Vec3b>(0,0)[2]);
    return color;
}

int main()
{
    //////////////HSV2BGR//////////////
    cv::Scalar BGR = HSV2BGR(120); // 0, 60, 120 Red, Green, Blue
    cv::Mat image(500, 500, CV_8UC3, BGR);
    cv::imshow("HSV2BGR Image", image);
    cv::waitKey(0);
    
    return 0;
}