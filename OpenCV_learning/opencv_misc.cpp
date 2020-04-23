/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019-10-25
* Copyright    :
* Descriptoin  :This cpp is used to test miscellaneous OpenCV usages
* References   :
======================================================================*/
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
    /// Print the OpenCV library compiling information
    std::cout << cv::getBuildInformation() << std::endl;
    return 0;
}