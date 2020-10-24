/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2020年10月23日
* Copyright    :
* Descriptoin  : Define a powerful color manager  class for widely usages in visualization needs
* References   :
======================================================================*/

#include <iostream>

#include <opencv2/opencv.hpp>

using std::cout;
using std::endl;

class ColorManager {
public:
    ColorManager()
    {
        color_container_ = std::vector<cv::Scalar>({cv::viz::Color::red(),
                                                                                                   cv::viz::Color::green(),
                                                                                                   cv::viz::Color::blue()});
    }

    std::vector<cv::Scalar> GetColorVec(const int length)
    {
        if (length <= color_container_.size()) {
            return std::vector<cv::Scalar>(color_container_.begin(), color_container_.begin() + length);
        }

        int increase = ((length % color_container_.size()) == 0) ? 0 : 1;
        int num = std::floor(length / color_container_.size()) + increase;

        auto ret = color_container_;

        ret.reserve(color_container_.size() * num);
        for (int i = 0; i < num - 1; ++i) {
            ret.insert(ret.end(), color_container_.begin(), color_container_.end());
        }

        return std::vector<cv::Scalar>(ret.begin(), ret.begin() + length);
    }

private:
    std::vector<cv::Scalar> color_container_;
};

int main()
{
    ColorManager color;
    auto ret = color.GetColorVec(10);

    for (const auto c : ret) {
        cout<<c<<endl;
    }
}