/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2020年8月8日
* Copyright    :
* Descriptoin  : Learning how to use template function or template class in C++
* References   :
======================================================================*/

#include <iostream>
#include <vector>

template <typename T>
struct Point {
    Point(T _x, T _y) : x(_x), y(_y) {
    }

    friend std::ostream& operator << (std::ostream& os, const Point& p)
    {
        os << "(" << p.x << "," << p.y << ")";
        return (os);
    }

    T x;
    T y;
};

template <typename PointType>
void ProcDiffType(const PointType& pt1, 
                                        const PointType& pt2)
{
    std::cout<<"ProcDiffType With same typename"<<std::endl;
    std::cout<<pt1<<std::endl;
    std::cout<<pt2<<std::endl;
}

template <typename PointType1, typename PointType2>
void ProcDiffType(const PointType1& pt1, 
                                        const PointType2& pt2)
{
    std::cout<<"ProcDiffType With two typename"<<std::endl;
    std::cout<<pt1<<std::endl;
    std::cout<<pt2<<std::endl;
}

template <typename PointType>
void ProcDiffVecType(const std::vector<PointType>& pt_vec1, 
                                               const std::vector<PointType>& pt_vec2)
{

}

int main()
{
    Point<float> pt_float(10.0, 13.0);
    Point<double> pt_double(14.0, 18.8);
    ProcDiffType(pt_float, pt_float);
    ProcDiffType(pt_float, pt_double);
    
    return 0;
}