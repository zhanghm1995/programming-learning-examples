/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-23 21:42:37
 * @References: 
 * @Description: Learn the numerical calculation related operations
 */

#include <cmath>
#include <iostream>
#include <limits>
#include <cfloat>

using std::cout;
using std::endl;

/**
 * @brief Test the nan in C++
 * @ref https://blog.csdn.net/wokaowokaowokao12345/article/details/72846436
 *           https://zh.cppreference.com/w/cpp/numeric/math/isnormal
 */ 
void TestNaN() {
    float a = NAN;
    cout<< a <<endl;
    if (std::isnormal(a)) {
        cout<<"is normal"<<endl;
    } else {
        cout<<"not normal"<<endl;
    }
}

void TestNumericalLimit() {
    float float_min = std::numeric_limits<float>::min();
    cout<<float_min<<endl;
     float float_max = std::numeric_limits<float>::max();
    cout<<float_max<<endl;

    double double_min = std::numeric_limits<double>::min();
    cout<<double_min<<endl;
    double double_max = std::numeric_limits<double>::max();
    cout<<double_max<<endl;

    double_min = DBL_MIN;
    cout<<double_min<<endl;
    double_max = DBL_MAX;
    cout<<double_max<<endl;
    
    uint16_t uint16_min = std::numeric_limits<uint16_t>::min();
    cout<<uint16_min<<endl;
    uint16_t uint16_max = std::numeric_limits<uint16_t>::max();
    cout<<uint16_max<<endl;
}

int main() {
    cout<<"=========TestNaN============"<<endl;
    TestNaN();
    cout<<"=========TestNumericalLimit============"<<endl;
    TestNumericalLimit();

    return 0;
}
