/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019-10-25
* Copyright    :
* Descriptoin  :Learn how to show text in image by using OpenCV
* References   : https://docs.opencv.org/3.4.4/d6/d6e/group__imgproc__draw.html#ga5126f47f883d730f633d74f07456c576
======================================================================*/

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

/**
 * @brief Understand the getTextSize usage
 */ 
void DrawTextBox()
{
    String text = "Funny text inside the box";
    int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 2;
    int thickness = 3;

    Mat img(600, 800, CV_8UC3, Scalar::all(0));

    int baseline=0;
    Size textSize = getTextSize(text, fontFace, fontScale, thickness, &baseline);
    cout<<textSize<<endl;
    baseline += thickness;

    // center the text
    Point textOrg((img.cols - textSize.width)/2,
                  (img.rows + textSize.height)/2);

    // draw the box
    rectangle(img, textOrg + Point(0, baseline),
              textOrg + Point(textSize.width, -textSize.height),
              Scalar(0,0,255));
    // ... and the baseline first
    line(img, textOrg + Point(0, thickness),
         textOrg + Point(textSize.width, thickness),
         Scalar(0, 0, 255));

    // then put the text itself
    putText(img, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
    cv::namedWindow("img_src", CV_WINDOW_NORMAL);
    imshow("img_src", img);
    waitKey(0);
}

std::vector<std::string> WrapText(const std::string& str_input, const int length)
{
    std::vector<std::string> ret;
    if (str_input.size() <= length) {
        ret.push_back(str_input);
        return ret;
    }

    std::string str;
    int pos = 0;
    const int temp_length = static_cast<int>(str_input.size()) - length;
    while (pos < temp_length) {
        ret.push_back(str_input.substr(pos, length));
        pos += length;
    }

    ret.push_back(str_input.substr(pos));

    return ret;
}

void ShowMultiLineText()
{
    std::stringstream sstr;
    sstr<<"This is a long long str, so we want to split it in multiple lines. This is a long long str,  so we want to split it in multiple lines.";
    int idx = -1;
    auto text_str = WrapText(sstr.str(), 20);
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(sstr.str(), FONT_HERSHEY_COMPLEX, 0.7, 1, &baseline);
    cout<<text_size<<endl;

    cv::Mat img(800, 800, CV_8UC3, cv::Scalar(0));

    for (size_t line = 0; line < text_str.size(); ++line) {
        int y = text_size.height + line * (text_size.height + 5);
        cv::putText(img, text_str[line], cv::Point(5, y), FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(255,0,0));
    }

    cv::namedWindow("ShowMultiLineText", CV_WINDOW_NORMAL);
    imshow("ShowMultiLineText", img);
    waitKey(0);
}

int main()
{
    DrawTextBox();

    ShowMultiLineText();
    return 0;
}