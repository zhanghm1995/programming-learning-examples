/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019-10-25
* Copyright    :
* Descriptoin  :Learn how to draw basic geometry by using OpenCV
* References   :
======================================================================*/

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

void DrawRotatedRect(cv::Mat &img)
{
    cv::RotatedRect rotated_rect(cv::Point2f(400, 400), cv::Size2f(50, 100), 45);
    Point2f vertices[4];
    rotated_rect.points(vertices);
    for (int i = 0; i < 4; i++)
        cv::line(img, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0));
}

void DrawPolygon(cv::Mat& img)
{
    std::vector<cv::Point> vertexes = {cv::Point(205, 145), cv::Point(260, 150), cv::Point(245, 205), cv::Point(236, 150)};
    cv::fillConvexPoly(img, vertexes, cv::Scalar::all(255));
}

int main(int argc, char **argv)
{
    cv::Mat img_src(800, 800, CV_8UC3, cv::Scalar(0));

    DrawRotatedRect(img_src);

    DrawPolygon(img_src);

    cv::namedWindow("img_src", CV_WINDOW_NORMAL);
    imshow("img_src", img_src);
    waitKey(0);
}
