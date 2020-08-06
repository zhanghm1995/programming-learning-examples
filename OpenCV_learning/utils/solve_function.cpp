#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
//https://www.tutorialspoint.com/checking-if-a-double-or-float-is-nan-in-cplusplus
//http://www.cplusplus.com/reference/cmath/isnormal/
using std::cout;
using std::endl;

double Divide() {
    double a = 0.0;
    double b = 0.0;
    return a / b;
}

int main() {
    std::vector<double> coefs = {1, 0, 0};
    std::vector<double> coefs1 = {1, 5, 4};
    std::vector<double> coefs2 = {0, 0, 4};
    std::vector<double> coefs3 = {0, 0, 0};
    std::vector<double> roots;
    cv::Mat rots;
    auto ret = cv::solvePoly(coefs3, rots);
    cout<<rots.type()<<" "<<CV_64FC1<<endl;
    cout<<ret<<endl;

    cout<<rots<<endl;

    auto ret2 = Divide();
    auto ret3 = NAN;
    cout<<ret2<<endl;
    return 0;
}