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


void CreateMat()
{
    /// Create a single-channel gray image
    cv::Mat img1(600, 400, CV_8UC1, cv::Scalar(0));
    auto img1_mask = img1(cv::Rect(100, 100, 100, 100));
    img1_mask.setTo(125);

    uchar* data = img1.ptr<uchar>(100);
    uchar value = data[99];
    cout<<int(value)<<endl;

    uchar value1 = 255;
    uchar ret = value & value1; // and operation

    cv::imshow("img1", img1);
    cv::waitKey(0);
}


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

/**
 * @brief Learn how to use rowRange and colRange function to extract the mask of Mat
 * @ref https://blog.csdn.net/jpc20144055069/article/details/102800181
 */ 
void UseRowColRangeMask() {
    cv::Mat Test = (cv::Mat_<double>(3, 3) << 0, 1, 2, 3, 4, 5, 6, 7, 8);
	cout << "Total matrix:" << endl;
	cout << Test << endl << endl;

	cv::Mat testrow = Test.rowRange(0, 2).clone();   // 包括左边界，但不包括右边界
	cout << "Row range:" << endl;
	cout << testrow << endl;
	cout << "Test 1 row:" << endl;
	cout << Test.row(0) << endl << endl;

	cv::Mat testcol = Test.colRange(0, 2).clone();   // 包括左边界，但不包括右边界
	cout << "Col range:" << endl;
	cout << testcol << endl;
}

// TODO: how to get the cv::Mat data type and access it by choosing correct method

int main() {
    cv::Mat src = cv::Mat::zeros(3, 1, CV_8UC1);
    ProcessMat(src);

    return 0;
}