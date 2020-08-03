/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-19 18:18:52
 * @LastEditTime: 2020-04-19 21:17:14
 * @References: 
 * @Description:  学习OpenCV中如何对图像进行算术计算,叠加处理等图像处理
 */

#include <opencv2/opencv.hpp>

/**
 * @brief 对图像进行逐位操作,图像可以是单通道或多通道,多通道图像每一个通道独立进行位操作
 * @ref https://docs.opencv.org/2.4/modules/core/doc/operations_on_arrays.html#bitwise-and
 */ 
void BitwiseOperation(const cv::Mat& img1, const cv::Mat& img2) {
    cv::Mat bitwise_result;
    
    cv::bitwise_and(img1, img2, bitwise_result);
    cv::imshow("bitwise_and", bitwise_result);
 
    cv::bitwise_or(img1, img2, bitwise_result);
    cv::imshow("bitwise_or", bitwise_result);

    cv::bitwise_xor(img1, img2, bitwise_result);
    cv::imshow("bitwise_xor", bitwise_result);
}

void TransparancyOperation() {
    cv::Mat bottom_img(800, 600, CV_8UC3, cv::Scalar(50, 255, 255));
    cv::Mat top_img(800, 600, CV_8UC3, cv::Scalar(125, 0, 0));

    cv::imshow("bottom_img", bottom_img);
    cv::imshow("top_image", top_img);
    
    cv::Mat background_img = cv::imread("../data/background_image.jpeg");
    cv::imshow("background_image", background_img);

    cv::Mat mask(background_img.rows, background_img.cols, CV_8UC1, cv::Scalar(0));
    cv::rectangle(mask, cv::Rect(100, 100, 150, 200), cv::Scalar(255), -1);

    cv::Mat temp(background_img.rows, background_img.cols, CV_8UC3, cv::Scalar(0, 0, 255));
    background_img.copyTo(temp, mask);
    cv::imshow("temp", temp);

    double alpha = 0.5;
    cv::Mat result_img;
    cv::addWeighted(temp,  alpha, background_img, 1 - alpha, 0, result_img);
    cv::imshow("result_img", result_img);
    cv::waitKey(0);
}

void MergeTwoImages() {
    cv::Mat img1(800, 600, CV_8UC3, cv::Scalar(128, 255, 0));
    cv::Mat img2(800, 600, CV_8UC3, cv::Scalar(128, 128, 128));

    cv::Mat merged_img(img1.rows, img1.cols + img2.cols + 1,
                                                 img1.type(), cv::Scalar(0));

    img1.copyTo(merged_img.colRange(0, img1.cols));
    img2.copyTo(merged_img.colRange(img2.cols + 1, merged_img.cols));

    // Draw the boundary line
    cv::line(merged_img, cv::Point(img1.cols, 0), cv::Point(img1.cols, merged_img.rows),
                    cv::Scalar(255, 255, 255));

    cv::imshow("merged_img", merged_img);
    cv::waitKey(0);
}

void MergeTwoImages2() {
    cv::Mat img1(800, 600, CV_8UC3, cv::Scalar(128, 255, 0));
    cv::Mat img2(700, 500, CV_8UC3, cv::Scalar(128, 128, 128));

    
    int merged_rows = std::max(img1.rows, img2.rows);
    int merged_cols = img1.cols + img2.cols;

    cv::Mat merged_img(merged_rows, merged_cols + 1, img1.type(), cv::Scalar(0));

    img1.copyTo(merged_img(cv::Rect(0, 0, img1.cols, img1.rows)));
    img2.copyTo(merged_img(cv::Rect(img1.cols + 1, 0, img2.cols, img2.rows)));

    // Draw the boundary line
    cv::line(merged_img, cv::Point(img1.cols, 0), cv::Point(img1.cols, merged_img.rows),
                    cv::Scalar(255, 255, 255));

    cv::imshow("merged_img2", merged_img);
    cv::waitKey(0);
}

int main() {
    // TransparancyOperation();

    MergeTwoImages();

    MergeTwoImages2();
    return 0;
}